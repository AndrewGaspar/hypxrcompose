#include "Ffmpeg.hpp"
#include "Log.hpp"
#include "Process.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <format>
#include <set>
#include <sstream>

namespace hxc {

    namespace {

        std::vector<std::string> splitLines(const std::string& text) {
            std::vector<std::string> lines;
            std::string              current;
            for (char c : text) {
                if (c == '\n') {
                    lines.push_back(current);
                    current.clear();
                } else if (c != '\r')
                    current += c;
            }
            if (!current.empty())
                lines.push_back(current);
            return lines;
        }

        std::optional<double> parseDouble(const std::string& text) {
            try {
                size_t     consumed = 0;
                const auto VALUE    = std::stod(text, &consumed);
                if (consumed == 0)
                    return std::nullopt;
                return VALUE;
            } catch (...) { return std::nullopt; }
        }

        // "30000/1001" and "30" both appear in ffprobe output.
        double parseRational(const std::string& text) {
            const size_t SLASH = text.find('/');
            if (SLASH == std::string::npos)
                return parseDouble(text).value_or(0.0);
            const auto NUM = parseDouble(text.substr(0, SLASH));
            const auto DEN = parseDouble(text.substr(SLASH + 1));
            if (!NUM || !DEN || *DEN == 0.0)
                return 0.0;
            return *NUM / *DEN;
        }

        int64_t secondsToNs(double seconds) {
            return static_cast<int64_t>(std::llround(seconds * 1e9));
        }

    }

    bool isIntraOnlyCodec(const std::string& codecName) {
        // Everything a `.hypxrtake` writes for its overlay, plus the still-image
        // codecs the tests round-trip through. Deliberately not "anything without
        // a known GOP": an unrecognized codec falls through to the safe answer.
        static const std::set<std::string> INTRA_ONLY = {
            "ffv1", "utvideo", "huffyuv", "ffvhuff", "rawvideo", "png", "apng", "mjpeg", "qtrle", "v210", "r210", "dpx", "tiff", "bmp", "magicyuv", "prores", "dnxhd", "cfhd", "jpeg2000", "libopenjpeg",
        };
        return INTRA_ONLY.count(codecName) > 0;
    }

    std::optional<double> seekSecondsForFrame(const SVideoInfo& info, size_t index) {
        if (!info.intraOnly || index == 0 || index >= info.ptsNs.size())
            return std::nullopt;
        const int64_t PREVIOUS = info.ptsNs[index - 1];
        const int64_t TARGET   = info.ptsNs[index];
        if (TARGET <= PREVIOUS)
            return std::nullopt; // duplicate or non-monotonic stamps: refuse rather than guess
        return static_cast<double>(PREVIOUS + (TARGET - PREVIOUS) / 2) * 1e-9;
    }

    bool checksumVideo(const std::string& path, int threads, std::string& digest, std::string& error) {
        digest.clear();

        std::vector<std::string> argv{"ffmpeg", "-hide_banner", "-v", "error"};
        if (threads > 0) {
            argv.push_back("-threads");
            argv.push_back(std::to_string(threads));
        }
        argv.insert(argv.end(), {"-i", path, "-map", "0:v:0", "-f", "hash", "-hash", "md5", "-"});

        std::string text;
        if (!runCapture(argv, text, error))
            return false;
        for (const auto& LINE : splitLines(text)) {
            const size_t EQUALS = LINE.find('=');
            if (EQUALS != std::string::npos)
                digest = LINE.substr(EQUALS + 1);
        }
        if (digest.empty()) {
            error = std::format("{}: the frame hash came back empty", path);
            return false;
        }
        return true;
    }

    bool probeVideo(const std::string& path, SVideoInfo& out, std::string& error, eProbeDepth depth) {
        out       = {};
        out.depth = depth;

        std::string text;
        if (!runCapture({"ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=width,height,avg_frame_rate,codec_name,pix_fmt", "-of", "default=noprint_wrappers=1", path},
                        text, error))
            return false;

        for (const auto& LINE : splitLines(text)) {
            const size_t EQUALS = LINE.find('=');
            if (EQUALS == std::string::npos)
                continue;
            const std::string KEY   = LINE.substr(0, EQUALS);
            const std::string VALUE = LINE.substr(EQUALS + 1);
            if (KEY == "width")
                out.width = static_cast<int>(parseDouble(VALUE).value_or(0));
            else if (KEY == "height")
                out.height = static_cast<int>(parseDouble(VALUE).value_or(0));
            else if (KEY == "avg_frame_rate")
                out.avgFps = parseRational(VALUE);
            else if (KEY == "codec_name")
                out.codecName = VALUE;
            else if (KEY == "pix_fmt")
                out.pixelFormat = VALUE;
        }

        if (out.width <= 0 || out.height <= 0) {
            error = std::format("{}: ffprobe reported no video stream dimensions", path);
            return false;
        }
        out.intraOnly = isIntraOnlyCodec(out.codecName);

        // `frame=` makes ffprobe decode; `packet=` makes it demux. On a 2.3 GB
        // ffv1 take that is fourteen minutes against a fraction of a second, and
        // the two produce the same list.
        const char* ENTRIES = depth == eProbeDepth::DEEP ? "frame=pts_time" : "packet=pts_time";
        if (!runCapture({"ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", ENTRIES, "-of", "csv=p=0", path}, text, error))
            return false;

        for (const auto& LINE : splitLines(text)) {
            if (LINE.empty() || LINE == "N/A")
                continue;
            const auto VALUE = parseDouble(LINE);
            if (VALUE)
                out.ptsNs.push_back(secondsToNs(*VALUE));
        }
        // Packets arrive in decode order, which for an inter-coded stream with B
        // frames is not presentation order. Nothing downstream wants decode order
        // - the uses are "how many" and "how long" - so sort and be done.
        std::sort(out.ptsNs.begin(), out.ptsNs.end());
        return true;
    }

    bool probeAudio(const std::string& path, SAudioInfo& out, std::string& error) {
        out = {};

        std::string text;
        if (!runCapture({"ffprobe", "-v", "error", "-select_streams", "a:0", "-show_entries", "stream=sample_rate,channels,duration,codec_name", "-of", "default=noprint_wrappers=1", path},
                        text, error))
            return false;

        for (const auto& LINE : splitLines(text)) {
            const size_t EQUALS = LINE.find('=');
            if (EQUALS == std::string::npos)
                continue;
            const std::string KEY   = LINE.substr(0, EQUALS);
            const std::string VALUE = LINE.substr(EQUALS + 1);
            if (KEY == "sample_rate")
                out.sampleRate = static_cast<int>(parseDouble(VALUE).value_or(0));
            else if (KEY == "channels")
                out.channels = static_cast<int>(parseDouble(VALUE).value_or(0));
            else if (KEY == "duration")
                out.durationNs = secondsToNs(parseDouble(VALUE).value_or(0.0));
            else if (KEY == "codec_name")
                out.codecName = VALUE;
        }

        if (out.sampleRate <= 0 || out.channels <= 0) {
            error = std::format("{}: ffprobe reported no audio stream", path);
            return false;
        }
        return true;
    }

    CVideoReader::~CVideoReader() = default;

    std::unique_ptr<CVideoReader> CVideoReader::open(const std::string& path, int width, int height, std::string& error, const SReaderOptions& options) {
        if (width <= 0 || height <= 0) {
            error = std::format("{}: refusing to decode at {}x{}", path, width, height);
            return nullptr;
        }

        std::vector<std::string> argv{"ffmpeg", "-hide_banner", "-v", "error"};
        if (options.threads > 0) {
            argv.push_back("-threads");
            argv.push_back(std::to_string(options.threads));
        }
        // Before -i, so this is an input seek: ffmpeg jumps in the container
        // rather than decoding and discarding from the start.
        if (options.seekSeconds >= 0.0) {
            argv.push_back("-ss");
            argv.push_back(std::format("{:.6f}", options.seekSeconds));
        }
        argv.insert(argv.end(), {"-i", path, "-f", "rawvideo", "-pix_fmt", "rgba", "-"});

        CSubprocess::SOptions spawnOptions;
        spawnOptions.pipeStdout = true;

        auto process = CSubprocess::spawn(argv, spawnOptions, error);
        if (!process)
            return nullptr;

        auto reader        = std::unique_ptr<CVideoReader>(new CVideoReader());
        reader->m_process  = std::move(process);
        reader->m_path     = path;
        reader->m_width    = width;
        reader->m_height   = height;
        // The first frame out of a seeked decoder is `startFrame`, not zero.
        // advanceTo()'s first read leaves m_current alone, so seeding it here is
        // all the whole-file numbering costs.
        reader->m_current  = options.startFrame;
        reader->m_frame.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 4);
        return reader;
    }

    bool CVideoReader::advanceTo(size_t index, std::string& error) {
        if (m_hasFrame && index <= m_current) {
            if (index < m_current)
                ++m_rewinds;
            return true;
        }

        while (!m_hasFrame || m_current < index) {
            bool eof = false;
            if (!m_process->readExact(m_frame.data(), m_frame.size(), eof)) {
                m_eof = true;
                if (m_hasFrame) {
                    // Running past the end is normal when the source stops before
                    // the composite does; the last frame is held.
                    HXC_DEBUG("{}: decode ended at frame {} while seeking {}", m_path, m_current, index);
                    return true;
                }
                error = std::format("{}: decoded no frames{}", m_path, eof ? "" : " (short read)");
                return false;
            }
            ++m_decoded;
            if (m_hasFrame)
                ++m_current;
            m_hasFrame = true;
        }
        return true;
    }

    CVideoWriter::~CVideoWriter() {
        if (m_process && !m_finished) {
            m_process->closeStdin();
            m_process->wait();
        }
    }

    namespace {

        // ffmpeg's `-c:a` for a container, chosen from the output extension so the
        // caller never has to know the pairing rules.
        std::string audioCodecFor(const std::string& outPath) {
            if (outPath.size() >= 4 && outPath.compare(outPath.size() - 4, 4, ".mkv") == 0)
                return "flac";
            if (outPath.size() >= 5 && outPath.compare(outPath.size() - 5, 5, ".webm") == 0)
                return "libopus";
            return "aac";
        }

        std::string audioFilterChain(const SWriterSpec& spec, size_t& mixedCount) {
            std::string chain;
            mixedCount = 0;

            for (size_t i = 0; i < spec.audio.size(); ++i) {
                const auto& TRACK = spec.audio[i];
                // ffmpeg input index: 0 is the rawvideo pipe, so audio starts at 1.
                std::string step  = std::format("[{}:a]aresample={}", i + 1, spec.audioSampleRate);
                if (TRACK.startSample < 0)
                    step += std::format(",atrim=start_sample={},asetpts=PTS-STARTPTS", -TRACK.startSample);
                else if (TRACK.startSample > 0)
                    step += std::format(",adelay=delays={}S:all=1", TRACK.startSample);
                if (TRACK.gain != 1.0)
                    step += std::format(",volume={:.6f}", TRACK.gain);
                step += std::format("[a{}];", i);
                chain += step;
                ++mixedCount;
            }

            if (mixedCount == 0)
                return {};

            if (mixedCount == 1)
                chain += "[a0]anull[aout]";
            else {
                for (size_t i = 0; i < mixedCount; ++i)
                    chain += std::format("[a{}]", i);
                // normalize=0 keeps a straight sum; the limiter below is what stops
                // the sum from clipping, rather than pre-attenuating every track.
                chain += std::format("amix=inputs={}:duration=longest:normalize=0", mixedCount);
                if (spec.limiterCeiling > 0.0)
                    chain += std::format(",alimiter=limit={:.4f}:level=false", spec.limiterCeiling);
                chain += "[aout]";
            }
            return chain;
        }

    }

    std::unique_ptr<CVideoWriter> CVideoWriter::open(const SWriterSpec& spec, std::string& error) {
        if (spec.width <= 0 || spec.height <= 0 || !(spec.fps > 0.0)) {
            error = std::format("invalid writer geometry {}x{} @ {} fps", spec.width, spec.height, spec.fps);
            return nullptr;
        }

        std::vector<std::string> argv{
            "ffmpeg", "-hide_banner", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgba", "-s", std::format("{}x{}", spec.width, spec.height), "-r", std::format("{:.9g}", spec.fps),
            "-i",     "pipe:0",
        };

        for (const auto& TRACK : spec.audio) {
            argv.push_back("-i");
            argv.push_back(TRACK.path);
        }

        size_t            mixed = 0;
        const std::string CHAIN = audioFilterChain(spec, mixed);
        if (!CHAIN.empty()) {
            argv.push_back("-filter_complex");
            argv.push_back(CHAIN);
            argv.push_back("-map");
            argv.push_back("0:v");
            argv.push_back("-map");
            argv.push_back("[aout]");
            argv.push_back("-c:a");
            argv.push_back(audioCodecFor(spec.outPath));
            argv.push_back("-shortest");
        } else {
            argv.push_back("-map");
            argv.push_back("0:v");
        }

        if (spec.threads > 0) {
            argv.push_back("-threads");
            argv.push_back(std::to_string(spec.threads));
        }
        argv.push_back("-c:v");
        argv.push_back(spec.videoCodec);
        if (spec.videoCodec == "libx264" || spec.videoCodec == "libx265") {
            argv.push_back("-crf");
            argv.push_back(std::to_string(spec.crf));
            argv.push_back("-preset");
            argv.push_back(spec.preset);
        }
        argv.push_back("-pix_fmt");
        argv.push_back(spec.pixelFormat);
        argv.push_back(spec.outPath);

        CSubprocess::SOptions options;
        options.pipeStdin = true;
        auto process      = CSubprocess::spawn(argv, options, error);
        if (!process)
            return nullptr;

        auto writer             = std::unique_ptr<CVideoWriter>(new CVideoWriter());
        writer->m_process       = std::move(process);
        writer->m_command       = argv;
        writer->m_expectedBytes = static_cast<size_t>(spec.width) * static_cast<size_t>(spec.height) * 4;
        return writer;
    }

    bool CVideoWriter::writeFrame(const uint8_t* rgba, size_t bytes) {
        if (bytes != m_expectedBytes)
            return false;
        return m_process->writeAll(rgba, bytes);
    }

    bool CVideoWriter::finish(std::string& error) {
        if (m_finished)
            return true;
        m_finished = true;
        m_process->closeStdin();
        const int STATUS = m_process->wait();
        if (STATUS != 0) {
            error = std::format("the output muxer exited with status {}; command was: {}", STATUS, describeArgv(m_command));
            return false;
        }
        return true;
    }

    CSimpleEncoder::~CSimpleEncoder() {
        if (m_process && !m_finished) {
            m_process->closeStdin();
            m_process->wait();
        }
    }

    std::unique_ptr<CSimpleEncoder> CSimpleEncoder::open(const std::string& outPath, int width, int height, double fps, const std::vector<std::string>& codecArgs, double startTimeSeconds,
                                                        std::string& error) {
        std::vector<std::string> argv{
            "ffmpeg", "-hide_banner", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgba", "-s", std::format("{}x{}", width, height), "-r", std::format("{:.9g}", fps), "-i", "pipe:0",
        };
        argv.insert(argv.end(), codecArgs.begin(), codecArgs.end());
        if (startTimeSeconds != 0.0) {
            argv.push_back("-output_ts_offset");
            argv.push_back(std::format("{:.9f}", startTimeSeconds));
        }
        argv.push_back(outPath);

        CSubprocess::SOptions options;
        options.pipeStdin = true;
        auto process      = CSubprocess::spawn(argv, options, error);
        if (!process)
            return nullptr;

        auto encoder             = std::unique_ptr<CSimpleEncoder>(new CSimpleEncoder());
        encoder->m_process       = std::move(process);
        encoder->m_expectedBytes = static_cast<size_t>(width) * static_cast<size_t>(height) * 4;
        return encoder;
    }

    bool CSimpleEncoder::writeFrame(const uint8_t* rgba, size_t bytes) {
        if (bytes != m_expectedBytes)
            return false;
        return m_process->writeAll(rgba, bytes);
    }

    bool CSimpleEncoder::finish(std::string& error) {
        if (m_finished)
            return true;
        m_finished = true;
        m_process->closeStdin();
        const int STATUS = m_process->wait();
        if (STATUS != 0) {
            error = std::format("encoder exited with status {}", STATUS);
            return false;
        }
        return true;
    }

    bool encodePcmS16(const std::string& outPath, const std::vector<int16_t>& interleaved, int sampleRate, int channels, std::string& error) {
        CSubprocess::SOptions options;
        options.pipeStdin = true;

        auto process = CSubprocess::spawn({"ffmpeg", "-hide_banner", "-v", "error", "-y", "-f", "s16le", "-ar", std::to_string(sampleRate), "-ac", std::to_string(channels), "-i", "pipe:0", outPath},
                                          options, error);
        if (!process)
            return false;

        if (!process->writeAll(interleaved.data(), interleaved.size() * sizeof(int16_t))) {
            error = std::format("{}: writing PCM to the encoder failed", outPath);
            process->wait();
            return false;
        }
        process->closeStdin();
        const int STATUS = process->wait();
        if (STATUS != 0) {
            error = std::format("{}: the audio encoder exited with status {}", outPath, STATUS);
            return false;
        }
        return true;
    }

    bool decodePcmS16(const std::string& path, std::vector<int16_t>& interleaved, int& sampleRate, int& channels, std::string& error) {
        SAudioInfo info;
        if (!probeAudio(path, info, error))
            return false;
        sampleRate = info.sampleRate;
        channels   = info.channels;

        CSubprocess::SOptions options;
        options.pipeStdout = true;
        auto process       = CSubprocess::spawn({"ffmpeg", "-hide_banner", "-v", "error", "-i", path, "-f", "s16le", "-acodec", "pcm_s16le", "-"}, options, error);
        if (!process)
            return false;

        interleaved.clear();
        std::vector<uint8_t> buffer(256 * 1024);
        for (;;) {
            const size_t READ = process->readSome(buffer.data(), buffer.size());
            if (READ == 0)
                break;
            const size_t BASE = interleaved.size();
            interleaved.resize(BASE + READ / sizeof(int16_t));
            std::memcpy(interleaved.data() + BASE, buffer.data(), (READ / sizeof(int16_t)) * sizeof(int16_t));
        }

        const int STATUS = process->wait();
        if (STATUS != 0) {
            error = std::format("{}: the audio decoder exited with status {}", path, STATUS);
            return false;
        }
        return true;
    }

    bool writePng(const std::string& outPath, const uint8_t* rgba, int width, int height, std::string& error) {
        CSubprocess::SOptions options;
        options.pipeStdin = true;

        auto process = CSubprocess::spawn({"ffmpeg", "-hide_banner", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgba", "-s", std::format("{}x{}", width, height), "-i", "pipe:0", "-frames:v",
                                           "1", "-c:v", "png", "-pix_fmt", "rgba", outPath},
                                          options, error);
        if (!process)
            return false;

        const bool WROTE = process->writeAll(rgba, static_cast<size_t>(width) * static_cast<size_t>(height) * 4);
        process->closeStdin();
        const int STATUS = process->wait();
        if (!WROTE || STATUS != 0) {
            error = std::format("{}: writing the PNG failed (status {})", outPath, STATUS);
            return false;
        }
        return true;
    }

    bool readPng(const std::string& path, std::vector<uint8_t>& rgba, int& width, int& height, std::string& error) {
        SVideoInfo info;
        if (!probeVideo(path, info, error))
            return false;
        width  = info.width;
        height = info.height;

        auto reader = CVideoReader::open(path, width, height, error);
        if (!reader || !reader->advanceTo(0, error))
            return false;
        rgba = reader->rgba();
        return true;
    }

}
