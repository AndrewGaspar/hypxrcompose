#pragma once

// Video and audio I/O by talking to the `ffmpeg` and `ffprobe` binaries over
// pipes, rather than by linking libav*.
//
// Why subprocesses for v1 (this is a deliberate choice, not an omission):
//   - the libav* ABI moves, and a tool that must still open takes recorded a year
//     ago is better off depending on "an ffmpeg is installed" than on a specific
//     libavcodec soname;
//   - every command is printable, so `--dump-commands` turns any decode or encode
//     problem into a command line the user can run and inspect by hand;
//   - the codec menu is whatever the user's ffmpeg has, including hardware
//     encoders, with no build-time coupling;
//   - the cost is one memcpy per frame through a pipe, which at v1 resolutions is
//     far below the GPU and encoder time it sits next to (measured: see README).
// A v2 that wants zero-copy hardware frames should link libav*; the seam here is
// narrow enough that it is a contained change.

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace hxc {

    class CSubprocess;

    // How hard a probe works for its frame count.
    //
    // INDEX walks the container's packets. It decodes nothing, so it costs a read
    // of the file rather than a decode of it - seconds instead of minutes on a
    // multi-gigabyte lossless take. For every codec a `.hypxrtake` can hold
    // (ffv1, utvideo, huffyuv, png, rawvideo - all intra-only) one packet is
    // exactly one frame, so the count is not an estimate. It is also right for
    // the inter-coded camera sources, where a packet is still a frame; what B
    // frames change is the *order* of the timestamps, which is why ptsNs is
    // sorted rather than left in packet order.
    //
    // DEEP decodes every frame and counts what comes out. It is the arbiter when
    // a file is suspected of being truncated or of carrying packets that do not
    // decode, and it is what `validate --deep` runs.
    enum class eProbeDepth {
        INDEX,
        DEEP,
    };

    struct SVideoInfo {
        int                  width      = 0;
        int                  height     = 0;
        double               avgFps     = 0.0;
        std::string          codecName;
        std::string          pixelFormat;
        // Presentation timestamps in nanoseconds, ascending. Matroska quantizes
        // these to 1 ms (its timecode scale is fixed at 1e6 ns and ffmpeg exposes
        // no knob), so callers must treat them as approximate; see Bundle.cpp for
        // how overlay frames are matched.
        std::vector<int64_t> ptsNs;
        eProbeDepth          depth     = eProbeDepth::INDEX;
        // Whether one packet is guaranteed to be one independently decodable
        // frame, which is what makes both the INDEX count and segment seeking
        // exact rather than approximate.
        bool                 intraOnly = false;
    };

    bool probeVideo(const std::string& path, SVideoInfo& out, std::string& error, eProbeDepth depth = eProbeDepth::INDEX);

    // Probe results, written by one process and read by another.
    //
    // A segmented render is K processes opening the same take. Each of them
    // would otherwise demux every video in the bundle to count its frames -
    // 4.8 GB per worker on a two-eye take, for numbers the parent worked out
    // seconds earlier. The parent therefore saves what it learned and the
    // workers read it.
    //
    // An entry names the file's size and modification time and is ignored
    // unless both still match, so a cache cannot outlive the file it describes
    // or be pointed at the wrong one. A miss is not an error: it costs a probe.
    class CProbeCache {
      public:
        // A cache that knows nothing. Reading a file that is absent or malformed
        // gives the same thing, because a bad cache must degrade to no cache.
        static std::shared_ptr<CProbeCache> read(const std::string& path);

        bool                                lookup(const std::string& path, eProbeDepth depth, SVideoInfo& out) const;
        void                                record(const std::string& path, const SVideoInfo& info);
        bool                                write(const std::string& path, std::string& error) const;
        size_t                              size() const;

      private:
        struct SEntry {
            SVideoInfo info;
            int64_t    fileSize  = 0;
            int64_t    modifiedNs = 0;
        };
        std::map<std::string, SEntry> m_entries;
    };

    // probeVideo(), consulting `cache` first and recording into it after. A null
    // cache is exactly probeVideo().
    bool probeVideoCached(const std::string& path, SVideoInfo& out, std::string& error, eProbeDepth depth, const std::shared_ptr<CProbeCache>& cache);

    // Whether every frame of this codec stands alone. The list is closed on
    // purpose: a codec nobody named is assumed to be inter-coded, which costs a
    // deep probe and disables segment seeking rather than producing wrong frames.
    bool isIntraOnlyCodec(const std::string& codecName);

    // An md5 over every decoded frame's pixels, in order - `validate --deep
    // --checksum`'s answer to "did these bytes survive the disk". Decoding is what
    // it costs, which is why it is not on by default.
    bool checksumVideo(const std::string& path, int threads, std::string& digest, std::string& error);

    struct SAudioInfo {
        int         sampleRate = 0;
        int         channels   = 0;
        int64_t     durationNs = 0;
        std::string codecName;
    };

    bool probeAudio(const std::string& path, SAudioInfo& out, std::string& error);

    struct SReaderOptions {
        // 0 leaves ffmpeg's default, which is "as many threads as the machine
        // has". A segmented render sets this, because K workers each helping
        // themselves to the whole machine is how a parallel decode goes slower
        // than a serial one.
        int    threads     = 0;
        // Where in the stream to start. `startFrame` is the ordinal the first
        // decoded frame will be reported as, so an advanceTo() against indices
        // from the whole-file timeline keeps working; `seekSeconds` is the -ss
        // target that gets the decoder there. Use seekSecondsForFrame() to
        // compute the pair.
        size_t startFrame  = 0;
        double seekSeconds = -1.0; // < 0 means "no seek"
    };

    // The -ss target that makes frame `index` the first frame the decoder emits.
    //
    // ffmpeg's input seek lands on the last keyframe at or before the target and
    // then - accurate_seek being the default - drops decoded frames whose pts is
    // below it. So any instant strictly after frame index-1 and at or before
    // frame index selects exactly frame index, and the midpoint between the two
    // is the choice furthest from both rounding errors and from Matroska's 1 ms
    // timestamp quantization.
    //
    // Intra-only sources only: with inter-coded frames the seek lands on a
    // keyframe that may be far behind. nullopt means "cannot seek here exactly",
    // and the caller must decode from the beginning.
    std::optional<double> seekSecondsForFrame(const SVideoInfo& info, size_t index);

    // Sequential RGBA decoder. Access is forward-only by design: the composite
    // walks the output timeline in order, so a source frame is either the current
    // one or somewhere ahead of it, and holding one decoded frame is all the
    // buffering the pipeline needs. A segment worker starts the walk partway
    // along instead of at zero; see SReaderOptions.
    class CVideoReader {
      public:
        ~CVideoReader();
        CVideoReader(const CVideoReader&)            = delete;
        CVideoReader& operator=(const CVideoReader&) = delete;

        static std::unique_ptr<CVideoReader> open(const std::string& path, int width, int height, std::string& error, const SReaderOptions& options = {});

        // Decodes forward until frame `index` is current. A request for a frame
        // already behind the cursor keeps the current frame and counts a rewind,
        // which the render report surfaces rather than failing the run.
        bool                                 advanceTo(size_t index, std::string& error);

        const std::vector<uint8_t>&          rgba() const {
            return m_frame;
        }
        bool hasFrame() const {
            return m_hasFrame;
        }
        std::optional<size_t> currentIndex() const {
            return m_hasFrame ? std::optional<size_t>(m_current) : std::nullopt;
        }
        size_t rewindsAvoided() const {
            return m_rewinds;
        }
        size_t framesDecoded() const {
            return m_decoded;
        }

      private:
        CVideoReader() = default;

        std::unique_ptr<CSubprocess> m_process;
        std::vector<uint8_t>         m_frame;
        std::string                  m_path;
        int                          m_width    = 0;
        int                          m_height   = 0;
        size_t                       m_current  = 0;
        size_t                       m_decoded  = 0;
        size_t                       m_rewinds  = 0;
        bool                         m_hasFrame = false;
        bool                         m_eof      = false;
    };

    // One audio track to fold into the output. `startSample` is where the track's
    // first sample belongs on the output timeline, measured in output samples: a
    // positive value pads with silence, a negative one trims from the head. Both
    // are sample-exact (adelay's `S` suffix and atrim's `start_sample`).
    struct SAudioMixInput {
        std::string path;
        int64_t     startSample = 0;
        double      gain        = 1.0;
        std::string label;
    };

    struct SWriterSpec {
        std::string                 outPath;
        int                         width  = 0;
        int                         height = 0;
        double                      fps    = 60.0;
        std::string                 videoCodec  = "libx264";
        std::string                 pixelFormat = "yuv420p";
        int                         crf         = 18;
        std::string                 preset      = "medium";
        std::vector<SAudioMixInput> audio;
        int                         audioSampleRate = 48000;
        // As for SReaderOptions::threads: 0 leaves ffmpeg's default.
        int                         threads         = 0;
        // The output is a side-by-side stereo pair (left half = left eye), and
        // must say so in the file rather than in the filename. Without this a
        // player shows a stereo take as two squashed pictures and an XR
        // compositor cannot auto-tag the window. Mono output must leave it
        // false: a wrong stereo flag is worse than none.
        bool                        stereoSideBySide = false;
        // Linear ceiling for the summing limiter; <= 0 disables it. Only inserted
        // when two or more tracks are summed, since a single track cannot clip
        // against itself.
        double                      limiterCeiling  = 0.98;
    };

    class CVideoWriter {
      public:
        ~CVideoWriter();
        CVideoWriter(const CVideoWriter&)            = delete;
        CVideoWriter& operator=(const CVideoWriter&) = delete;

        static std::unique_ptr<CVideoWriter> open(const SWriterSpec& spec, std::string& error);

        bool                                 writeFrame(const uint8_t* rgba, size_t bytes);
        // Closes the pipe and waits for the muxer. Must be called explicitly; the
        // destructor only tears a failed run down.
        bool                                 finish(std::string& error);

        const std::vector<std::string>&      command() const {
            return m_command;
        }

      private:
        CVideoWriter() = default;

        std::unique_ptr<CSubprocess> m_process;
        std::vector<std::string>     m_command;
        size_t                       m_expectedBytes = 0;
        bool                         m_finished      = false;
    };

    // Joins already-encoded video segments into `spec.outPath` and folds
    // `spec.audio` in on the way past, in one ffmpeg invocation.
    //
    // The video is stream-copied: the segments were encoded once, by the workers
    // that composed them, and re-encoding here would both cost a second pass and
    // throw away the exactness the segmented render exists to preserve. The audio
    // is mixed here rather than in a worker because it is one track spanning the
    // whole take, not a per-segment thing - so it is placed against the finished
    // timeline exactly as the single-job writer places it.
    //
    // Every segment must carry the same codec, geometry and rate, which they do
    // by construction: they came from one SWriterSpec.
    bool concatSegments(const std::vector<std::string>& segmentPaths, const SWriterSpec& spec, std::string& error);

    // Encodes interleaved signed-16 PCM into a file (FLAC for `.flac`). Used by
    // the synthetic bundle generator.
    bool encodePcmS16(const std::string& outPath, const std::vector<int16_t>& interleaved, int sampleRate, int channels, std::string& error);

    // Decodes a whole audio file to interleaved signed-16 PCM at its native rate.
    // Used by the tests to measure where the synthetic clicks landed.
    bool decodePcmS16(const std::string& path, std::vector<int16_t>& interleaved, int& sampleRate, int& channels, std::string& error);

    // Writes a single RGBA image as a PNG. Used by --frames-dir and the tests.
    bool writePng(const std::string& outPath, const uint8_t* rgba, int width, int height, std::string& error);
    bool readPng(const std::string& path, std::vector<uint8_t>& rgba, int& width, int& height, std::string& error);

    // Encodes an RGBA frame sequence in one shot; the synth generator's video path.
    class CSimpleEncoder {
      public:
        ~CSimpleEncoder();
        static std::unique_ptr<CSimpleEncoder> open(const std::string& outPath, int width, int height, double fps, const std::vector<std::string>& codecArgs, double startTimeSeconds,
                                                    std::string& error);
        bool                                   writeFrame(const uint8_t* rgba, size_t bytes);
        bool                                   finish(std::string& error);

      private:
        CSimpleEncoder() = default;
        std::unique_ptr<CSubprocess> m_process;
        size_t                       m_expectedBytes = 0;
        bool                         m_finished      = false;
    };

}
