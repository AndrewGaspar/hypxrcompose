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
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace hxc {

    class CSubprocess;

    struct SVideoInfo {
        int                  width      = 0;
        int                  height     = 0;
        double               avgFps     = 0.0;
        std::string          codecName;
        std::string          pixelFormat;
        // Presentation timestamps in nanoseconds, in decode order, as the container
        // stores them. Matroska quantizes these to 1 ms (its timecode scale is fixed
        // at 1e6 ns and ffmpeg exposes no knob), so callers must treat them as
        // approximate; see Bundle.cpp for how overlay frames are matched.
        std::vector<int64_t> ptsNs;
    };

    bool probeVideo(const std::string& path, SVideoInfo& out, std::string& error);

    struct SAudioInfo {
        int         sampleRate = 0;
        int         channels   = 0;
        int64_t     durationNs = 0;
        std::string codecName;
    };

    bool probeAudio(const std::string& path, SAudioInfo& out, std::string& error);

    // Sequential RGBA decoder. Access is forward-only by design: the composite
    // walks the output timeline in order, so a source frame is either the current
    // one or somewhere ahead of it, and holding one decoded frame is all the
    // buffering the pipeline needs.
    class CVideoReader {
      public:
        ~CVideoReader();
        CVideoReader(const CVideoReader&)            = delete;
        CVideoReader& operator=(const CVideoReader&) = delete;

        static std::unique_ptr<CVideoReader> open(const std::string& path, int width, int height, std::string& error);

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
