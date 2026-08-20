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
  hypxrcompose validate <take> [--strict] [--json] [--no-media] [--deep]
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
      --deep               count frames by decoding them, not from the container
                           index. Same answer, minutes instead of seconds; for
                           when a file is suspected of being truncated
      --checksum           md5 every decoded frame (implies --deep)

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
      --eye-fov l,r,u,d    per-eye frustum in radians, eye 0 (eye 1 mirrors it);
                           asymmetric by default, as a real headset's is
      --geometry-markers   four extra opaque markers on the overlay quad, at the
                           corners of a rectangle, for measuring output scale
      --camera-active-array-pad N
                           state camera intrinsics against a sensor active array
                           N pixels taller than the image top and bottom, with the
                           principal point in that array's coordinates (what
                           Android does)
      --head-speed N       multiply the head's motion rate (default 1.0)
      --legacy-mirrored-extrinsics
                           write `extrinsics_head_to_camera` with the cant
                           mirrored, as the buggy producer wrote it, while still
                           writing a correct `extrinsics_android_raw`
      --no-distortion      cameras publish no distortion coefficients, so the
                           sidecar carries `"distortion": null` (what the Meta
                           cameras do - they pre-undistort)

render:
      --out FILE           output video (required)
      --eye left|right|stereo-sbs      (default left)
      --framing asis|stabilized        (default asis)
      --frustum presentation|recorded  (default presentation)
                           presentation: one symmetric frustum shared by both
                             eyes, cropped to the pane. Frames line up, stereo
                             fuses, recorded periphery outside it is dropped
                           recorded: each eye keeps its own asymmetric frustum,
                             padded. Truer to what each eye saw, and right for
                             analysis or a headset-native player - but on a flat
                             viewer the two frames sit at different angles
      --size WxH           per-eye pane size; stereo SBS output is 2W x H
      --fps N              output rate (default: the take's target_hz)
      --background auto|camera|checker|solid
      --bg-depth M         assumed background distance in metres (default 1.0,
                           measured stillest on a seated desk take; raise it for
                           a take shot across a room)
      --bg-align recorded|auto         (default recorded)
                           recorded: the true geometry. Where the header carries
                             `extrinsics_android_raw` the extrinsic is recomputed
                             from it, repairing the mirrored cant that older
                             producers stored
                           auto: aim each camera's optical axis along the output's
                             forward, keeping the extrinsic's roll. A framing
                             choice that centres the passthrough and re-registers
                             it against the world - not a correction
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
      --limit N            stop after N output frames
      --jobs N             compose N chunks of the timeline concurrently, one
                           worker process each, joined by a stream copy.
                           Default 1: the serial pipeline already saturates a
                           24-thread machine, so on the measured take --jobs 4
                           came in slower than --jobs 1. Worth trying when the
                           encode is off the CPU (--codec hevc_nvenc)
      --decode-threads N   threads each worker's ffmpeg may use
                           (default: hardware threads / --jobs)
      --segment i/k        render only chunk i of k. What --jobs spawns; useful
                           by hand for resuming or distributing a long take
      --worker-binary PATH what to spawn for a worker (default: this binary))");
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
            else if (ARG == "--deep")
                options.deep = true;
            else if (ARG == "--checksum")
                options.checksum = true;
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
            else if (ARG == "--geometry-markers")
                options.geometryMarkers = true;
            else if (ARG == "--no-distortion")
                options.noDistortion = true;
            else if (ARG == "--head-speed" && parseDouble(value(COUNT, REST, i), number))
                options.headSpeed = number;
            else if (ARG == "--legacy-mirrored-extrinsics")
                options.legacyMirroredExtrinsics = true;
            else if (ARG == "--camera-active-array-pad" && parseInt(value(COUNT, REST, i), integer))
                options.cameraActiveArrayPad = static_cast<int>(integer);
            else if (ARG == "--eye-fov") {
                const std::string SPEC = value(COUNT, REST, i) ?: "";
                double            v[4] = {0, 0, 0, 0};
                size_t            at   = 0;
                int               n    = 0;
                for (; n < 4 && at <= SPEC.size(); ++n) {
                    const size_t COMMA = SPEC.find(',', at);
                    if (!parseDouble(SPEC.substr(at, COMMA - at).c_str(), v[n]))
                        break;
                    if (COMMA == std::string::npos) {
                        ++n;
                        break;
                    }
                    at = COMMA + 1;
                }
                if (n != 4) {
                    HXC_ERR("--eye-fov takes four radian angles, l,r,u,d (for example -0.9425,0.6981,0.7679,-0.9599)");
                    return 2;
                }
                options.eyeFov = {v[0], v[1], v[2], v[3]};
            }
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
            } else if (ARG == "--frustum") {
                const std::string CHOICE = value(COUNT, REST, i) ?: "";
                if (CHOICE == "presentation")
                    options.frustum = eFrustumMode::PRESENTATION;
                else if (CHOICE == "recorded")
                    options.frustum = eFrustumMode::RECORDED;
                else {
                    HXC_ERR("--frustum takes presentation or recorded");
                    return 2;
                }
            } else if (ARG == "--bg-align") {
                const std::string CHOICE = value(COUNT, REST, i) ?: "";
                if (CHOICE == "auto")
                    options.backgroundAlign = eBackgroundAlign::AUTO;
                else if (CHOICE == "recorded")
                    options.backgroundAlign = eBackgroundAlign::RECORDED;
                else {
                    HXC_ERR("--bg-align takes auto or recorded");
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
            else if (ARG == "--jobs" && parseInt(value(COUNT, REST, i), integer))
                options.jobs = static_cast<int>(integer);
            else if (ARG == "--decode-threads" && parseInt(value(COUNT, REST, i), integer))
                options.decodeThreads = static_cast<int>(integer);
            else if (ARG == "--segment") {
                const std::string SPEC  = value(COUNT, REST, i) ?: "";
                const size_t      SLASH = SPEC.find('/');
                int64_t           index = 0, count = 0;
                if (SLASH == std::string::npos || !parseInt(SPEC.substr(0, SLASH).c_str(), index) || !parseInt(SPEC.c_str() + SLASH + 1, count) || count <= 0 || index < 0 || index >= count) {
                    HXC_ERR("--segment takes i/k with 0 <= i < k");
                    return 2;
                }
                options.segmentIndex = static_cast<int>(index);
                options.segmentCount = static_cast<int>(count);
            } else if (ARG == "--worker-binary") {
                const char* PATH = value(COUNT, REST, i);
                if (!PATH)
                    return 2;
                options.workerBinary = PATH;
            } else if (ARG == "--probe-cache") {
                const char* PATH = value(COUNT, REST, i);
                if (!PATH)
                    return 2;
                options.probeCachePath = PATH;
            } else if (ARG.starts_with("-")) {
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
