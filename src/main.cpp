// hypxrcompose - the offline compositor for `.hypxrtake` bundles.
//
// Three commands:
//   validate  structural and referential check of a bundle
//   synth     generate a synthetic bundle with known geometry
//   render    compose a bundle into a finished video

#include "Log.hpp"
#include "Process.hpp"
#include "Render.hpp"
#include "Synth.hpp"
#include "Validate.hpp"

#include <charconv>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace hxc;

namespace {

    void usage() {
        std::puts(R"(hypxrcompose - compose a .hypxrtake bundle into finished video

usage:
  hypxrcompose validate <take> [--strict] [--json] [--no-media]
  hypxrcompose synth <out-take> [options]
  hypxrcompose render <take> --out <file> [options]

common:
  -v, --verbose            debug logging
  -q, --quiet              errors only
      --dump-commands      print every ffmpeg/ffprobe command before running it

validate:
      --strict             treat warnings as failures
      --json               machine-readable report on stdout
      --no-media           structure only; skip ffprobe

synth:
      --frames N           telemetry records to generate (default 60)
      --hz N               session rate (default 60)
      --overlay-hz N       overlay capture rate (default: --hz)
      --overlay-size WxH   overlay resolution per eye (default 640x480)
      --cam-size WxH       passthrough camera resolution (default 640x480)
      --cam-hz N           camera rate (default 30)
      --no-cameras         host-only take
      --no-audio           no audio tracks
      --no-mic             app audio only
      --clock-offset-ms N  host->device offset at the take start (default 250)
      --clock-drift-ppm N  offset drift (default 20)
      --alpha premultiplied|straight   how the overlay stores colour (default premultiplied)
      --ipd N              interpupillary distance in metres (default 0.063)
      --cam-baseline N     camera separation in metres (default 0.084)

render:
      --out FILE           output video (required)
      --eye left|right|stereo-sbs      (default left)
      --framing asis|stabilized        (default asis)
      --size WxH           per-eye pane size; stereo SBS output is 2W x H
      --fps N              output rate (default: the take's target_hz)
      --background auto|camera|checker|solid
      --bg-depth M         assumed background distance in metres (default 2.0)
      --fg-depth M|inf     assumed overlay distance (default inf = rotation only)
      --stabilize-ms N     Gaussian sigma for --framing stabilized (default 200)
      --no-audio           video only
      --mic-gain G         linear gain applied to the mic track (default 1.0)
      --no-limiter         sum audio without the clip limiter
      --codec NAME         libx264 (default), libx265, ffv1, ...
      --crf N              quality for x264/x265 (default 18)
      --preset NAME        x264/x265 preset (default medium)
      --gpu SUBSTR         pin the EGL device by renderer/vendor/DRM node; `list` enumerates
      --frames-dir DIR     also write every output frame as a PNG
      --report FILE        write the per-frame render report as JSON
      --limit N            stop after N output frames)");
    }

    bool parseInt(const char* text, int64_t& out) {
        if (!text)
            return false;
        const size_t LENGTH = std::strlen(text);
        const auto   RESULT = std::from_chars(text, text + LENGTH, out);
        return RESULT.ec == std::errc{} && RESULT.ptr == text + LENGTH;
    }

    bool parseDouble(const char* text, double& out) {
        if (!text)
            return false;
        try {
            size_t consumed = 0;
            out             = std::stod(text, &consumed);
            return consumed == std::strlen(text);
        } catch (...) { return false; }
    }

    bool parseSize(const char* text, int& width, int& height) {
        if (!text)
            return false;
        const char* CROSS = std::strchr(text, 'x');
        if (!CROSS)
            return false;
        int64_t w = 0, h = 0;
        const std::string LEFT(text, CROSS);
        if (!parseInt(LEFT.c_str(), w) || !parseInt(CROSS + 1, h) || w <= 0 || h <= 0)
            return false;
        width  = static_cast<int>(w);
        height = static_cast<int>(h);
        return true;
    }

    // Returns nullptr and complains when the flag has no value after it.
    const char* value(int argc, char** argv, int& index) {
        if (index + 1 >= argc) {
            HXC_ERR("{} needs a value", argv[index]);
            return nullptr;
        }
        return argv[++index];
    }

}

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 2;
    }

    const std::string COMMAND = argv[1];
    if (COMMAND == "-h" || COMMAND == "--help" || COMMAND == "help") {
        usage();
        return 0;
    }

    // Global flags are accepted anywhere.
    std::vector<char*> args;
    for (int i = 2; i < argc; ++i) {
        const std::string ARG = argv[i];
        if (ARG == "-v" || ARG == "--verbose")
            setLogLevel(eLogLevel::DEBUG);
        else if (ARG == "-q" || ARG == "--quiet")
            setLogLevel(eLogLevel::ERR);
        else if (ARG == "--dump-commands")
            setCommandTracing(true);
        else
            args.push_back(argv[i]);
    }
    const int    COUNT = static_cast<int>(args.size());
    char** const REST  = args.data();

    if (COMMAND == "validate") {
        SValidateOptions options;
        for (int i = 0; i < COUNT; ++i) {
            const std::string ARG = REST[i];
            if (ARG == "--strict")
                options.strict = true;
            else if (ARG == "--json")
                options.jsonOutput = true;
            else if (ARG == "--no-media")
                options.skipMedia = true;
            else if (ARG.starts_with("-")) {
                HXC_ERR("unknown flag {}", ARG);
                return 2;
            } else if (options.root.empty())
                options.root = ARG;
            else {
                HXC_ERR("validate takes one take path");
                return 2;
            }
        }
        if (options.root.empty()) {
            HXC_ERR("validate needs a take path");
            return 2;
        }
        return runValidate(options);
    }

    if (COMMAND == "synth") {
        SSynthOptions options;
        for (int i = 0; i < COUNT; ++i) {
            const std::string ARG = REST[i];
            int64_t           integer = 0;
            double            number  = 0.0;
            if (ARG == "--frames" && parseInt(value(COUNT, REST, i), integer))
                options.frames = static_cast<int>(integer);
            else if (ARG == "--hz" && parseDouble(value(COUNT, REST, i), number))
                options.hz = number;
            else if (ARG == "--overlay-hz" && parseDouble(value(COUNT, REST, i), number))
                options.overlayHz = number;
            else if (ARG == "--overlay-size" && parseSize(value(COUNT, REST, i), options.overlayWidth, options.overlayHeight))
                ;
            else if (ARG == "--cam-size" && parseSize(value(COUNT, REST, i), options.cameraWidth, options.cameraHeight))
                ;
            else if (ARG == "--cam-hz" && parseDouble(value(COUNT, REST, i), number))
                options.cameraHz = number;
            else if (ARG == "--no-cameras")
                options.cameras = false;
            else if (ARG == "--no-audio")
                options.audio = false;
            else if (ARG == "--no-mic")
                options.mic = false;
            else if (ARG == "--alpha") {
                const char* CHOICE = value(COUNT, REST, i);
                if (!CHOICE)
                    return 2;
                options.alpha = CHOICE;
            } else if (ARG == "--clock-offset-ms" && parseDouble(value(COUNT, REST, i), number))
                options.clockOffsetMs = number;
            else if (ARG == "--clock-drift-ppm" && parseDouble(value(COUNT, REST, i), number))
                options.clockDriftPpm = number;
            else if (ARG == "--ipd" && parseDouble(value(COUNT, REST, i), number))
                options.ipd = number;
            else if (ARG == "--cam-baseline" && parseDouble(value(COUNT, REST, i), number))
                options.cameraBaseline = number;
            else if (ARG.starts_with("-")) {
                HXC_ERR("unknown or malformed flag {}", ARG);
                return 2;
            } else if (options.out.empty())
                options.out = ARG;
            else {
                HXC_ERR("synth takes one output path");
                return 2;
            }
        }
        if (options.out.empty()) {
            HXC_ERR("synth needs an output path");
            return 2;
        }
        return runSynth(options);
    }

    if (COMMAND == "render") {
        SRenderOptions options;
        for (int i = 0; i < COUNT; ++i) {
            const std::string ARG = REST[i];
            int64_t           integer = 0;
            double            number  = 0.0;
            if (ARG == "--out") {
                const char* PATH = value(COUNT, REST, i);
                if (!PATH)
                    return 2;
                options.outPath = PATH;
            } else if (ARG == "--eye") {
                const std::string CHOICE = value(COUNT, REST, i) ?: "";
                if (CHOICE == "left")
                    options.eye = eEyeSelection::LEFT;
                else if (CHOICE == "right")
                    options.eye = eEyeSelection::RIGHT;
                else if (CHOICE == "stereo-sbs")
                    options.eye = eEyeSelection::STEREO_SBS;
                else {
                    HXC_ERR("--eye takes left, right, or stereo-sbs");
                    return 2;
                }
            } else if (ARG == "--framing") {
                const std::string CHOICE = value(COUNT, REST, i) ?: "";
                if (CHOICE == "asis")
                    options.framing = eFraming::ASIS;
                else if (CHOICE == "stabilized")
                    options.framing = eFraming::STABILIZED;
                else {
                    HXC_ERR("--framing takes asis or stabilized");
                    return 2;
                }
            } else if (ARG == "--background") {
                const std::string CHOICE = value(COUNT, REST, i) ?: "";
                if (CHOICE == "auto")
                    options.background = eBackgroundChoice::AUTO;
                else if (CHOICE == "camera")
                    options.background = eBackgroundChoice::CAMERA;
                else if (CHOICE == "checker")
                    options.background = eBackgroundChoice::CHECKER;
                else if (CHOICE == "solid")
                    options.background = eBackgroundChoice::SOLID;
                else {
                    HXC_ERR("--background takes auto, camera, checker, or solid");
                    return 2;
                }
            } else if (ARG == "--size" && parseSize(value(COUNT, REST, i), options.width, options.height))
                ;
            else if (ARG == "--fps" && parseDouble(value(COUNT, REST, i), number))
                options.fps = number;
            else if (ARG == "--bg-depth" && parseDouble(value(COUNT, REST, i), number))
                options.backgroundDepth = number;
            else if (ARG == "--fg-depth") {
                const std::string CHOICE = value(COUNT, REST, i) ?: "";
                if (CHOICE == "inf" || CHOICE == "infinite")
                    options.overlayDepth = std::numeric_limits<double>::infinity();
                else if (!parseDouble(CHOICE.c_str(), options.overlayDepth)) {
                    HXC_ERR("--fg-depth takes a distance in metres or `inf`");
                    return 2;
                }
            } else if (ARG == "--stabilize-ms" && parseDouble(value(COUNT, REST, i), number))
                options.stabilizeSigmaNs = static_cast<int64_t>(number * 1e6);
            else if (ARG == "--no-audio")
                options.noAudio = true;
            else if (ARG == "--mic-gain" && parseDouble(value(COUNT, REST, i), number))
                options.micGain = number;
            else if (ARG == "--no-limiter")
                options.noLimiter = true;
            else if (ARG == "--codec") {
                const char* NAME = value(COUNT, REST, i);
                if (!NAME)
                    return 2;
                options.videoCodec = NAME;
            } else if (ARG == "--crf" && parseInt(value(COUNT, REST, i), integer))
                options.crf = static_cast<int>(integer);
            else if (ARG == "--preset") {
                const char* NAME = value(COUNT, REST, i);
                if (!NAME)
                    return 2;
                options.preset = NAME;
            } else if (ARG == "--gpu") {
                const char* HINT = value(COUNT, REST, i);
                if (!HINT)
                    return 2;
                options.gpuHint = HINT;
            } else if (ARG == "--frames-dir") {
                const char* PATH = value(COUNT, REST, i);
                if (!PATH)
                    return 2;
                options.framesDir = PATH;
            } else if (ARG == "--report") {
                const char* PATH = value(COUNT, REST, i);
                if (!PATH)
                    return 2;
                options.reportPath = PATH;
            } else if (ARG == "--limit" && parseInt(value(COUNT, REST, i), integer))
                options.limitFrames = static_cast<size_t>(integer);
            else if (ARG.starts_with("-")) {
                HXC_ERR("unknown or malformed flag {}", ARG);
                return 2;
            } else if (options.take.empty())
                options.take = ARG;
            else {
                HXC_ERR("render takes one take path");
                return 2;
            }
        }
        if (options.take.empty() || options.outPath.empty()) {
            HXC_ERR("render needs a take path and --out");
            return 2;
        }
        return runRender(options);
    }

    HXC_ERR("unknown command `{}`", COMMAND);
    usage();
    return 2;
}
