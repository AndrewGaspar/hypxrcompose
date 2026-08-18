#include "Synth.hpp"
#include "Ffmpeg.hpp"
#include "Log.hpp"
#include "SynthScene.hpp"
#include "Timeline.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <set>
#include <thread>

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace hxc {

    namespace {

        struct SRgba {
            uint8_t r = 0, g = 0, b = 0, a = 255;
        };

        constexpr SRgba VOID_COLOR{12, 14, 18, 255};

        // Rows are independent, so the image loops split across cores. The camera
        // pass is the expensive one: a 640x480 frame is 300k rays and a take is
        // dozens of frames per camera.
        template <typename F>
        void parallelRows(int height, F&& body) {
            unsigned threads = std::thread::hardware_concurrency();
            if (threads == 0)
                threads = 1;
            threads = std::min<unsigned>(threads, static_cast<unsigned>(std::max(1, height)));
            if (threads == 1) {
                for (int y = 0; y < height; ++y)
                    body(y);
                return;
            }

            std::vector<std::thread> workers;
            workers.reserve(threads);
            for (unsigned t = 0; t < threads; ++t) {
                workers.emplace_back([&, t] {
                    for (int y = static_cast<int>(t); y < height; y += static_cast<int>(threads))
                        body(y);
                });
            }
            for (auto& WORKER : workers)
                WORKER.join();
        }

        bool insideSquare(double x, double y, double centreX, double centreY, double size) {
            return std::abs(x - centreX) <= size * 0.5 && std::abs(y - centreY) <= size * 0.5;
        }

        SRgba shadeWall(const SSynthScene& scene, double x, double y, int codeIndex) {
            if (std::abs(x) > scene.wallHalfX || std::abs(y) > scene.wallHalfY)
                return VOID_COLOR;

            for (const auto& MARKER : scene.wallMarkers) {
                if (insideSquare(x, y, MARKER.world.x, MARKER.world.y, MARKER.size))
                    return {static_cast<uint8_t>(MARKER.color[0]), static_cast<uint8_t>(MARKER.color[1]), static_cast<uint8_t>(MARKER.color[2]), 255};
            }

            if (insideSquare(x, y, scene.codeCentre.x, scene.codeCentre.y, scene.codeSize)) {
                const int RED = scene.codeBase + (codeIndex % scene.codeModulus) * scene.codeStep;
                return {static_cast<uint8_t>(std::clamp(RED, 0, 255)), static_cast<uint8_t>(scene.codeGreen), static_cast<uint8_t>(scene.codeBlue), 255};
            }

            const int  CELL_X = static_cast<int>(std::floor((x + 128.0) / scene.checkerCell));
            const int  CELL_Y = static_cast<int>(std::floor((y + 128.0) / scene.checkerCell));
            const bool DARK   = ((CELL_X + CELL_Y) & 1) == 0;
            return DARK ? SRgba{34, 42, 52, 255} : SRgba{158, 166, 176, 255};
        }

        // Ray/plane intersection against the wall, in world space.
        bool traceWall(const SSynthScene& scene, const SVec3& origin, const SVec3& direction, double& x, double& y) {
            if (std::abs(direction.z) < 1e-12)
                return false;
            const double TRAVEL = (scene.wallZ - origin.z) / direction.z;
            if (!(TRAVEL > 0.0))
                return false;
            x = origin.x + direction.x * TRAVEL;
            y = origin.y + direction.y * TRAVEL;
            return true;
        }

        // Premultiplied linear colour plus linear alpha - the space compositing is
        // defined in, and the space the producer's overlay tap works in before it
        // encodes.
        struct SLinearRgba {
            double r = 0.0, g = 0.0, b = 0.0, a = 0.0;
        };

        SLinearRgba fromSrgb(double red, double green, double blue, double alpha) {
            return {srgbToLinear(red / 255.0) * alpha, srgbToLinear(green / 255.0) * alpha, srgbToLinear(blue / 255.0) * alpha, alpha};
        }

        SLinearRgba fromSrgb(const std::array<int, 3>& color, double alpha) {
            return fromSrgb(color[0], color[1], color[2], alpha);
        }

        // The over operator on premultiplied linear values.
        SLinearRgba over(const SLinearRgba& front, const SLinearRgba& back) {
            const double KEEP = 1.0 - front.a;
            return {front.r + back.r * KEEP, front.g + back.g * KEEP, front.b + back.b * KEEP, front.a + back.a * KEEP};
        }

        // Encodes a premultiplied linear pixel the way the manifest says the file
        // stores it. "premultiplied" encodes the associated colour directly;
        // "straight" divides the association back out first, which is exactly the
        // step a compositor that mistook one for the other would skip.
        SRgba encodeOverlay(const SLinearRgba& pixel, bool premultiplied) {
            const auto CHANNEL = [&](double value) {
                const double LINEAR = premultiplied ? value : (pixel.a > 0.0 ? value / pixel.a : 0.0);
                return static_cast<uint8_t>(std::lround(std::clamp(linearToSrgb(LINEAR), 0.0, 1.0) * 255.0));
            };
            return {CHANNEL(pixel.r), CHANNEL(pixel.g), CHANNEL(pixel.b), static_cast<uint8_t>(std::lround(std::clamp(pixel.a, 0.0, 1.0) * 255.0))};
        }

        SLinearRgba shadeOverlayQuad(const SSynthScene& scene, double u, double v) {
            const double HALF_W = scene.quadWidth * 0.5;
            const double HALF_H = scene.quadHeight * 0.5;

            if (std::abs(u) <= HALF_W && std::abs(v) <= HALF_H) {
                for (const auto& MARKER : scene.overlayMarkers) {
                    if (insideSquare(u, v, MARKER.u, MARKER.v, MARKER.size))
                        return fromSrgb(MARKER.color, MARKER.alpha);
                }
                // A gradient so the panel is not a flat colour a resampling bug could
                // hide inside.
                const double NX = (u + HALF_W) / scene.quadWidth;
                const double NY = (v + HALF_H) / scene.quadHeight;
                return fromSrgb(40.0 + 120.0 * NX, 60.0 + 90.0 * NY, 150.0 - 60.0 * NX, 1.0);
            }

            // A half-transparent halo, so the composite exercises real alpha blending
            // rather than a binary matte.
            if (std::abs(u) <= HALF_W + scene.quadHalo && std::abs(v) <= HALF_H + scene.quadHalo)
                return fromSrgb(90, 70, 140, 0.5);

            return {};
        }

        std::string jsonDouble(double value) {
            return std::format("{:.17g}", value);
        }

        std::string poseJson(const SPose& pose) {
            return std::format(R"({{"pos":[{},{},{}],"quat":[{},{},{},{}]}})", jsonDouble(pose.pos.x), jsonDouble(pose.pos.y), jsonDouble(pose.pos.z), jsonDouble(pose.rot.x),
                               jsonDouble(pose.rot.y), jsonDouble(pose.rot.z), jsonDouble(pose.rot.w));
        }

        std::string fovJson(const SFov& fov) {
            return std::format(R"({{"l":{},"r":{},"u":{},"d":{}}})", jsonDouble(fov.l), jsonDouble(fov.r), jsonDouble(fov.u), jsonDouble(fov.d));
        }

    }

    int runSynth(const SSynthOptions& options) {
        if (options.out.empty()) {
            HXC_ERR("synth needs an output path");
            return 2;
        }
        if (options.frames <= 0 || !(options.hz > 0.0)) {
            HXC_ERR("synth needs a positive frame count and rate");
            return 2;
        }

        // ---- the scene ------------------------------------------------------------
        SSynthScene scene;
        scene.t0HostNs      = options.t0HostNs;
        scene.hz            = options.hz;
        scene.frames        = options.frames;
        scene.overlayHz     = options.overlayHz > 0.0 ? options.overlayHz : options.hz;
        scene.overlayWidth  = options.overlayWidth;
        scene.overlayHeight = options.overlayHeight;
        scene.wallZ         = options.wallZ;
        scene.wallHalfX     = 4.0;
        scene.wallHalfY     = 3.0;
        scene.checkerCell   = 0.25;

        scene.wallMarkers = {
            {"green", {0, 255, 0}, {-0.85, 0.30, options.wallZ}, 0.12},
            {"blue", {0, 0, 255}, {1.15, -0.75, options.wallZ}, 0.12},
            {"red", {255, 0, 0}, {-0.15, -0.75, options.wallZ}, 0.12},
        };
        scene.codeCentre = {0.0, 0.78, options.wallZ};

        scene.overlayAlpha    = options.alpha;
        scene.overlayQuad     = {{0.30, 0.02, -1.10}, SQuat::fromAxisAngle({0.0, 1.0, 0.0}, -0.14)};
        scene.overlayMarkers  = {
            {"centre", {255, 0, 255}, 0.0, 0.0, 0.06, 1.0},
            {"corner", {0, 255, 255}, 0.26, 0.14, 0.05, 1.0},
            // Half-alpha, and the reason the composite can be shown to blend in
            // linear light: the same blend done on encoded values lands tens of
            // levels away from this one.
            {"halfalpha", {240, 60, 30}, -0.24, -0.13, 0.10, 0.5},
        };
        if (options.geometryMarkers) {
            // Four large, well-separated, opaque markers at the corners of a
            // rectangle, *replacing* the default three. The defaults are nearly
            // collinear and only a few pixels across once an asymmetric frustum
            // has been padded out to a wide pane, and a centroid taken over six
            // pixels cannot resolve a one-percent scale error. Kept small in
            // *angle* on purpose - a big marker's filled centroid drifts off the
            // projection of its centre, because the mapping is linear in tangent
            // and a wide square does not stay a square - and resolved instead by
            // rendering the check at a large pane.
            scene.overlayMarkers = {
                {"geomTL", {0, 128, 255}, -0.30, 0.16, 0.03, 1.0},
                {"geomTR", {255, 128, 0}, 0.30, 0.16, 0.03, 1.0},
                {"geomBL", {120, 255, 120}, -0.30, -0.16, 0.03, 1.0},
                {"geomBR", {200, 0, 120}, 0.30, -0.16, 0.03, 1.0},
            };
        }

        // Head-locked, and far enough out of the way that nothing else in the frame
        // moves under it as the head sweeps.
        scene.hudQuad = {{-0.52, 0.26, -0.55}, SQuat::identity()};

        const bool PREMULTIPLIED = scene.overlayAlpha == "premultiplied";
        if (scene.overlayAlpha != "premultiplied" && scene.overlayAlpha != "straight") {
            HXC_ERR("--alpha takes premultiplied or straight, not `{}`", scene.overlayAlpha);
            return 2;
        }

        scene.ipd       = options.ipd;
        // Mirrored, the way a headset's two eyes are: the outward side of each
        // eye sees further, so the optical axis sits off-centre in opposite
        // directions. Keeping the mirror exact is what makes a stereo pair's two
        // panes agree about scale.
        scene.motionSpeed = options.headSpeed;
        scene.eyeFov[0] = options.eyeFov;
        scene.eyeFov[1] = {-options.eyeFov.r, -options.eyeFov.l, options.eyeFov.u, options.eyeFov.d};

        // Which telemetry records carry overlay pixels. Two causes, both expressed
        // the same way in the bundle - a `"dropped": true` on the record:
        //   - decimation, because the overlay is captured at target_hz while the
        //     session runs faster;
        //   - readback-queue losses, which are irregular by nature.
        // A bundle that never exercises either would let an ordinal-alignment bug
        // pass unnoticed, so the generator always produces some of both.
        size_t decimated = 0, readbackDropped = 0;
        {
            std::vector<int> captured;
            for (int k = 0; k < options.frames; ++k) {
                const int THIS_TICK = static_cast<int>(std::floor(static_cast<double>(k) * scene.overlayHz / scene.hz));
                const int LAST_TICK = k == 0 ? -1 : static_cast<int>(std::floor(static_cast<double>(k - 1) * scene.overlayHz / scene.hz));
                if (k == 0 || THIS_TICK > LAST_TICK)
                    captured.push_back(k);
                else
                    ++decimated;
            }

            // Two irregular losses at fixed fractions of the take, so the drop
            // pattern is not a clean stride and the tests can target a frame whose
            // nearest overlay record is not its own.
            std::vector<size_t> lose;
            if (captured.size() >= 12) {
                lose = {captured.size() / 3, (2 * captured.size()) / 3};
                for (auto it = lose.rbegin(); it != lose.rend(); ++it) {
                    captured.erase(captured.begin() + static_cast<long>(*it));
                    ++readbackDropped;
                }
            }
            scene.overlayFrames = std::move(captured);
        }
        const std::set<int> CAPTURED_RECORDS(scene.overlayFrames.begin(), scene.overlayFrames.end());

        // ---- the clock ------------------------------------------------------------
        const int64_t T0       = scene.t0HostNs;
        const int64_t DURATION = static_cast<int64_t>(std::llround(static_cast<double>(options.frames - 1) * 1e9 / options.hz));
        const int64_t T_END    = T0 + DURATION;

        std::vector<SClockSample> clockSamples;
        {
            const int64_t OFFSET0 = static_cast<int64_t>(std::llround(options.clockOffsetMs * 1e6));
            // A little deterministic jitter on each sample, because a real offset
            // estimator is noisy and the compositor must not depend on a clean line.
            uint64_t      rng     = 0x9E3779B97F4A7C15ULL;
            const auto    NEXT    = [&rng] {
                rng ^= rng << 13;
                rng ^= rng >> 7;
                rng ^= rng << 17;
                return rng;
            };
            for (int64_t t = T0 - options.clockIntervalNs; t <= T_END + 2 * options.clockIntervalNs; t += options.clockIntervalNs) {
                const double SECONDS = static_cast<double>(t - T0) * 1e-9;
                const int64_t DRIFT  = static_cast<int64_t>(std::llround(options.clockDriftPpm * SECONDS * 1000.0));
                const int64_t JITTER = static_cast<int64_t>(NEXT() % 40001) - 20000; // +/- 20 us
                clockSamples.push_back({t, OFFSET0 + DRIFT + JITTER, 1800.0 + static_cast<double>(NEXT() % 400)});
            }
        }
        const CClockMap CLOCK(clockSamples);

        // ---- camera cadence -------------------------------------------------------
        if (options.cameras) {
            const double  SPLAY      = options.cameraSplayDeg * M_PI / 180.0;
            const int64_t PHASE_NS   = 7000000; // deliberately not aligned with the telemetry cadence
            const int64_t FIRST      = CLOCK.deviceFromHost(T0 + PHASE_NS);
            const int     COUNT      = static_cast<int>(std::floor(static_cast<double>(DURATION) * 1e-9 * options.cameraHz)) + 1;

            for (int eye = 0; eye < 2; ++eye) {
                SSynthCamera camera;
                camera.key    = eye == 0 ? "L" : "R";
                camera.eye    = eye;
                camera.width  = options.cameraWidth;
                camera.height = options.cameraHeight;
                camera.hz     = options.cameraHz;
                camera.exposureNs = options.exposureNs;

                const double SIGN         = eye == 0 ? -1.0 : 1.0;
                camera.headToCamera.pos   = {SIGN * options.cameraBaseline * 0.5, -options.cameraDrop, -options.cameraForward};
                camera.headToCamera.rot   = SQuat::fromYawPitchRoll(-SIGN * SPLAY, -0.0175, 0.0);

                // Wide enough that the eye frustum sits inside the camera's, with a
                // little to spare for the splay and the head's translation - but only
                // a little, so the out-of-coverage fallback still gets exercised at
                // the extreme corners, exactly as a real passthrough camera does.
                camera.intrinsics.fx         = 190.0;
                camera.intrinsics.fy         = 190.0;
                camera.intrinsics.cx         = static_cast<double>(options.cameraWidth) * 0.5 + 3.5;
                camera.intrinsics.cy         = static_cast<double>(options.cameraHeight) * 0.5 - 2.5;
                camera.intrinsics.distortion = options.noDistortion ? std::vector<double>{} : std::vector<double>{-0.06, 0.008, 0.0009, -0.0006, 0.0};

                for (int j = 0; j < COUNT; ++j)
                    camera.deviceNs.push_back(FIRST + static_cast<int64_t>(std::llround(static_cast<double>(j) * 1e9 / options.cameraHz)));

                scene.cameras.push_back(std::move(camera));
            }
            scene.hasCameras = true;
        }

        // ---- audio stamps ---------------------------------------------------------
        if (options.audio) {
            scene.hasApp             = true;
            scene.app.sampleRate     = 48000;
            scene.app.startNs        = T0 + 120000000;
            scene.app.clickHostNs    = T0 + 300000000;
            scene.app.clickTrackNs   = scene.app.clickHostNs;
            scene.app.clickAmplitude = 0.90;

            if (options.mic) {
                scene.hasMic             = true;
                scene.mic.sampleRate     = 48000;
                scene.mic.startNs        = CLOCK.deviceFromHost(T0 + 60000000);
                scene.mic.clickHostNs    = T0 + 600000000;
                scene.mic.clickTrackNs   = CLOCK.deviceFromHost(scene.mic.clickHostNs);
                scene.mic.clickAmplitude = 0.55;
            }
        }

        // ---- directories ----------------------------------------------------------
        std::error_code ec;
        fs::remove_all(options.out, ec);
        for (const auto& SUBDIRECTORY : {fs::path{}, fs::path{"overlay"}, fs::path{"audio"}, fs::path{"synth"}, fs::path{"cameras"}}) {
            if (SUBDIRECTORY == "cameras" && !options.cameras)
                continue;
            fs::create_directories(options.out / SUBDIRECTORY, ec);
        }

        const std::string TAKE_ID = std::format("synth-{}", T0);

        // ---- manifest -------------------------------------------------------------
        {
            json manifest;
            manifest["take_id"] = TAKE_ID;
            manifest["host"]    = {
                {"tool", "hypxrcompose synth"},
                {"tool_version", "1"},
                {"synthetic", true},
                {"clock_offset_ms", options.clockOffsetMs},
                {"clock_drift_ppm", options.clockDriftPpm},
            };
            manifest["sources"] = {
                {"overlay", true},
                {"app_audio", scene.hasApp},
                {"cameras", scene.hasCameras},
                {"mic", scene.hasMic},
            };
            manifest["overlay"] = {
                {"width", scene.overlayWidth},   {"height", scene.overlayHeight}, {"format", "rgba"}, {"encoder", "ffv1"},
                {"target_hz", scene.overlayHz},  {"eye_count", 2},                {"alpha", scene.overlayAlpha},
            };
            manifest["notes"] = json::array({
                "synthetic bundle from `hypxrcompose synth`; the scene is described exactly in synth/ground-truth.json",
                std::format("host<->device offset starts at {:.3f} ms and drifts {:.1f} ppm", options.clockOffsetMs, options.clockDriftPpm),
                "camera extrinsics deliberately differ from the eye poses (wider baseline, forward offset, outward splay)",
                std::format("overlay colour is stored {}; the multiply happens in linear light and the sRGB encode after it", scene.overlayAlpha),
                "quads: index 0 is a room-anchored monitor, index 1 a head-locked HUD; both poses are recorded head-relative",
                std::format("overlay: {} of {} records carry pixels; dropped {} to decimation ({:.0f} Hz session -> {:.0f} Hz overlay) and {} to the readback queue",
                            scene.overlayFrames.size(), options.frames, decimated, scene.hz, scene.overlayHz, readbackDropped),
            });

            std::ofstream stream(options.out / "manifest.json");
            stream << manifest.dump(2) << "\n";
        }

        // ---- telemetry and clock --------------------------------------------------
        {
            std::ofstream telemetry(options.out / "telemetry.jsonl");
            // A constant session-lifetime stage correction, exactly the kind research
            // 27 footnote 2 says must be recorded. v1 records and reports it; it does
            // not re-derive geometry from it, because the stamped eye poses already
            // include it.
            const SPose STAGE_CORRECTION{{0.0, 0.0, 0.0}, SQuat::fromAxisAngle({0.0, 1.0, 0.0}, 0.004)};

            for (int k = 0; k < options.frames; ++k) {
                const int64_t T_HOST = scene.frameHostNs(k);
                const SPose   HEAD   = scene.headAt(T_HOST);

                std::string eyes;
                for (int eye = 0; eye < 2; ++eye) {
                    const SPose EYE_POSE = HEAD.compose({{(eye == 0 ? -0.5 : 0.5) * scene.ipd, 0.0, 0.0}, SQuat::identity()});
                    if (eye > 0)
                        eyes += ",";
                    eyes += std::format(R"({{"pose":{},"fov":{}}})", poseJson(EYE_POSE), fovJson(scene.eyeFov[static_cast<size_t>(eye)]));
                }

                // Composition layers, back to front, with their poses expressed
                // head-relative exactly as the producer records them. The monitor is
                // room-anchored, so its recorded pose is head(t) inverse composed
                // with its fixed STAGE pose and therefore changes every record; the
                // HUD is head-locked, so its recorded pose is constant.
                //
                // `visibility` is an XrEyeVisibility, and the primary case is the
                // per-eye pair a stereo-depth desktop submits: two quads sharing
                // one pose, one composed into each eye, taking opposite halves of
                // a side-by-side swapchain. The HUD is the other case, `both`.
                // The deprecated boolean/numeric spelling is exercised in the unit
                // tests instead, so that the bundle every end-to-end test shares
                // validates without a deprecation warning.
                const SPose MONITOR_RELATIVE = HEAD.inverse().compose(scene.overlayQuad);
                const auto  quadJson         = [&](int index, const SPose& pose, double width, double height, bool viewSpace, int swapchain, const char* visibility, int rectX) {
                    return std::format(R"({{"index":{},"name":null,"pose":{},"size":[{},{}],"visibility":"{}","view_space":{},"swapchain":{},"image":{},"array_layer":0,"rect":[{},0,{},{}]}})", index,
                                       poseJson(pose), jsonDouble(width), jsonDouble(height), visibility, viewSpace ? "true" : "false", swapchain, k % 3, rectX, scene.overlayWidth,
                                       scene.overlayHeight);
                };

                const bool CAPTURED = CAPTURED_RECORDS.count(k) > 0;
                telemetry << std::format(R"({{"t_host_ns":{},"frame":{},"eyes":[{}],"head":{},"quads":[{},{},{}],"stage_correction":{},"blend_mode":"alpha","dropped":{}}})", T_HOST, k, eyes,
                                         poseJson(HEAD), quadJson(0, MONITOR_RELATIVE, scene.quadWidth, scene.quadHeight, false, 7, "left", 0),
                                         quadJson(1, MONITOR_RELATIVE, scene.quadWidth, scene.quadHeight, false, 7, "right", scene.overlayWidth),
                                         quadJson(2, scene.hudQuad, scene.hudWidth, scene.hudHeight, true, 9, "both", 0), poseJson(STAGE_CORRECTION), CAPTURED ? "false" : "true")
                          << "\n";
            }

            std::ofstream clock(options.out / "clock.jsonl");
            for (const auto& SAMPLE : clockSamples)
                clock << std::format(R"({{"t_host_ns":{},"offset_ns":{},"rtt_us":{}}})", SAMPLE.tHostNs, SAMPLE.offsetNs, jsonDouble(SAMPLE.rttUs)) << "\n";
        }

        // ---- ground truth ---------------------------------------------------------
        {
            std::ofstream stream(options.out / SSynthScene::RELATIVE_PATH);
            stream << scene.toJson() << "\n";
        }

        std::string error;

        // ---- overlay videos -------------------------------------------------------
        {
            const int WIDTH  = scene.overlayWidth;
            const int HEIGHT = scene.overlayHeight;
            const int COUNT  = static_cast<int>(scene.overlayFrames.size());

            for (int eye = 0; eye < 2; ++eye) {
                // A uniform nominal timeline at target_hz starting at zero: the
                // container's pts say nothing about host time and nothing consumes
                // them for alignment. `-fps_mode passthrough` is what keeps ffmpeg
                // from duplicating or dropping a frame to hit that nominal rate,
                // which would break the ordinal correspondence the bundle promises.
                auto encoder = CSimpleEncoder::open((options.out / "overlay" / std::format("eye{}.mkv", eye)).string(), WIDTH, HEIGHT, scene.overlayHz,
                                                    {"-fps_mode", "passthrough", "-c:v", "ffv1", "-pix_fmt", "rgba"}, 0.0, error);
                if (!encoder) {
                    HXC_ERR("{}", error);
                    return 1;
                }

                std::vector<uint8_t> frame(static_cast<size_t>(WIDTH) * HEIGHT * 4);
                // One video frame per undropped telemetry record, in order, rendered
                // from *that record's* pose. This is the ordinal alignment rule the
                // compositor consumes, produced here rather than assumed there.
                for (int record : scene.overlayFrames) {
                    const int64_t T_HOST   = scene.frameHostNs(record);
                    const SPose   HEAD     = scene.headAt(T_HOST);
                    const SPose   EYE_POSE = HEAD.compose({{(eye == 0 ? -0.5 : 0.5) * scene.ipd, 0.0, 0.0}, SQuat::identity()});
                    const SFov&   FOV      = scene.eyeFov[static_cast<size_t>(eye)];
                    // Two layers, composited back to front exactly as the session
                    // would have: the room-anchored monitor at index 0 and the
                    // head-locked HUD at index 1. The HUD's pose is head-relative, so
                    // its world pose follows the head; the monitor's does not.
                    const SPose   HUD_WORLD    = HEAD.compose(scene.hudQuad);
                    const SVec3   LOCAL_ORIGIN = scene.overlayQuad.pointToLocal(EYE_POSE.pos);
                    const SVec3   HUD_ORIGIN   = HUD_WORLD.pointToLocal(EYE_POSE.pos);

                    parallelRows(HEIGHT, [&](int y) {
                        for (int x = 0; x < WIDTH; ++x) {
                            const SVec3 DIRECTION = EYE_POSE.dirToWorld(fovRay(FOV, x, y, WIDTH, HEIGHT));

                            SLinearRgba monitor;
                            const SVec3 LOCAL_DIR = scene.overlayQuad.dirToLocal(DIRECTION);
                            if (std::abs(LOCAL_DIR.z) > 1e-12) {
                                const double TRAVEL = -LOCAL_ORIGIN.z / LOCAL_DIR.z;
                                if (TRAVEL > 0.0)
                                    monitor = shadeOverlayQuad(scene, LOCAL_ORIGIN.x + LOCAL_DIR.x * TRAVEL, LOCAL_ORIGIN.y + LOCAL_DIR.y * TRAVEL);
                            }

                            SLinearRgba hud;
                            const SVec3 HUD_DIR = HUD_WORLD.dirToLocal(DIRECTION);
                            if (std::abs(HUD_DIR.z) > 1e-12) {
                                const double TRAVEL = -HUD_ORIGIN.z / HUD_DIR.z;
                                if (TRAVEL > 0.0) {
                                    const double U = HUD_ORIGIN.x + HUD_DIR.x * TRAVEL;
                                    const double V = HUD_ORIGIN.y + HUD_DIR.y * TRAVEL;
                                    if (std::abs(U) <= scene.hudWidth * 0.5 && std::abs(V) <= scene.hudHeight * 0.5)
                                        hud = fromSrgb(scene.hudColor, scene.hudAlpha);
                                }
                            }

                            const SRgba PIXEL = encodeOverlay(over(hud, monitor), PREMULTIPLIED);

                            uint8_t* out = frame.data() + (static_cast<size_t>(y) * WIDTH + static_cast<size_t>(x)) * 4;
                            out[0]       = PIXEL.r;
                            out[1]       = PIXEL.g;
                            out[2]       = PIXEL.b;
                            out[3]       = PIXEL.a;
                        }
                    });

                    if (!encoder->writeFrame(frame.data(), frame.size())) {
                        HXC_ERR("the overlay encoder for eye {} stopped accepting frames", eye);
                        return 1;
                    }
                }
                if (!encoder->finish(error)) {
                    HXC_ERR("{}", error);
                    return 1;
                }
            }
            if (!options.quiet)
                HXC_INFO("wrote {} overlay frames per eye at {}x{}", COUNT, WIDTH, HEIGHT);
        }

        // ---- camera videos --------------------------------------------------------
        if (scene.hasCameras) {
            // THE PRODUCER'S LAYOUT AND SHAPE, deliberately. The join drops the
            // sidecar at the take ROOT while the videos stay under cameras/, and
            // the header nests inversely: two parallel maps keyed by camera, with
            // the fields that are the same for every camera stated once. Emitting
            // what the device actually emits is the only way first contact with a
            // real take stops being a bug report - the previous shape here was a
            // guess, and four hand shims were needed to make the first joined
            // bundles load.
            std::ofstream sidecar(options.out / std::format("{}-cameras.jsonl", TAKE_ID));

            json header;
            header["record"]                    = "hypxrtake-cameras";
            header["record_version"]            = 1;
            header["take"]                      = TAKE_ID;
            header["intrinsics"]                = json::object();
            header["extrinsics_head_to_camera"] = json::object();
            for (const auto& CAMERA : scene.cameras) {
                json intrinsics;
                // The extra descriptive strings and the duplicate coefficient
                // array are the producer's, and a reader must step over them.
                intrinsics["camera_id"]     = CAMERA.key == "L" ? "50" : "51";
                intrinsics["camera_source"] = "0";
                intrinsics["position"]      = CAMERA.key == "L" ? "0" : "1";
                intrinsics["fx"]            = CAMERA.intrinsics.fx;
                intrinsics["fy"]            = CAMERA.intrinsics.fy;
                intrinsics["cx"]            = CAMERA.intrinsics.cx;
                intrinsics["cy"]            = CAMERA.intrinsics.cy;
                intrinsics["skew"]          = 0;
                intrinsics["intrinsics"]    = json::array({CAMERA.intrinsics.fx, CAMERA.intrinsics.fy, CAMERA.intrinsics.cx, CAMERA.intrinsics.cy, 0});
                // `null`, not `[]`, when there are none: the Meta cameras
                // pre-undistort and publish nothing, and the producer forwards
                // that faithfully.
                if (CAMERA.intrinsics.distortion.empty()) {
                    intrinsics["distortion"]       = nullptr;
                    intrinsics["distortion_model"] = nullptr;
                } else {
                    intrinsics["distortion"]       = CAMERA.intrinsics.distortion;
                    intrinsics["distortion_model"] = "opencv";
                }
                // Stated against the sensor's active array, not against the
                // image: cx/cy carry the crop offset, exactly as Android reports
                // them. `rebaseToImage` is what undoes it on the way in.
                const int PAD                         = options.cameraActiveArrayPad;
                intrinsics["cy"]                      = CAMERA.intrinsics.cy + PAD;
                intrinsics["intrinsics"]              = json::array({CAMERA.intrinsics.fx, CAMERA.intrinsics.fy, CAMERA.intrinsics.cx, CAMERA.intrinsics.cy + PAD, 0});
                intrinsics["active_array"]            = json::array({0, 0, CAMERA.width, CAMERA.height + 2 * PAD});
                intrinsics["pre_correction_active_array"] = json::array({0, 0, CAMERA.width, CAMERA.height + 2 * PAD});
                intrinsics["effective_size"]          = json::array({CAMERA.width, CAMERA.height});
                intrinsics["rolling_shutter_skew_ns"] = -1;
                header["intrinsics"][CAMERA.key]      = intrinsics;

                header["extrinsics_head_to_camera"][CAMERA.key] = {
                    {"pos", json::array({CAMERA.headToCamera.pos.x, CAMERA.headToCamera.pos.y, CAMERA.headToCamera.pos.z})},
                    {"quat", json::array({CAMERA.headToCamera.rot.x, CAMERA.headToCamera.rot.y, CAMERA.headToCamera.rot.z, CAMERA.headToCamera.rot.w})},
                    {"quat_order", "xyzw"},
                    {"axes", "openxr"},
                    {"reference", "GYROSCOPE"},
                };
            }
            // Shared once, not repeated per camera.
            header["axes"]               = "openxr";
            header["timestamp_source"]   = "SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME";
            header["t_device_ns_domain"] = "CLOCK_MONOTONIC";
            header["frame_alignment"]    = "ordinal";
            header["frame_rate"]         = 30;
            header["notes"]              = json::array();
            sidecar << header.dump() << "\n";

            for (const auto& CAMERA : scene.cameras) {
                const int WIDTH  = CAMERA.width;
                const int HEIGHT = CAMERA.height;

                // The per-pixel unprojection depends only on the intrinsics, so it is
                // computed once for the whole camera rather than per frame. This is
                // also the only place the *inverse* distortion model runs: the
                // compositor samples through the forward model, so a disagreement
                // between the two shows up as misalignment in the composite.
                std::vector<SVec3> rays(static_cast<size_t>(WIDTH) * HEIGHT);
                parallelRows(HEIGHT, [&](int y) {
                    for (int x = 0; x < WIDTH; ++x)
                        rays[static_cast<size_t>(y) * WIDTH + static_cast<size_t>(x)] = unprojectPinhole(CAMERA.intrinsics, static_cast<double>(x) + 0.5, static_cast<double>(y) + 0.5);
                });

                auto encoder = CSimpleEncoder::open((options.out / "cameras" / std::format("{}-cam{}.mp4", TAKE_ID, CAMERA.key)).string(), WIDTH, HEIGHT, CAMERA.hz,
                                                    {"-c:v", "libx264", "-qp", "0", "-pix_fmt", "yuv444p"}, static_cast<double>(CAMERA.deviceNs.front()) * 1e-9, error);
                if (!encoder) {
                    HXC_ERR("{}", error);
                    return 1;
                }

                std::vector<uint8_t> frame(static_cast<size_t>(WIDTH) * HEIGHT * 4);
                for (size_t j = 0; j < CAMERA.deviceNs.size(); ++j) {
                    // Mid-exposure, exactly as research 27 section 3 footnote 1
                    // requires and exactly as the compositor re-derives it.
                    const int64_t MID_EXPOSURE = CAMERA.deviceNs[j] + CAMERA.exposureNs / 2;
                    const int64_t HOST         = CLOCK.hostFromDevice(MID_EXPOSURE);
                    const SPose   CAMERA_POSE  = scene.headAt(HOST).compose(CAMERA.headToCamera);

                    parallelRows(HEIGHT, [&](int y) {
                        for (int x = 0; x < WIDTH; ++x) {
                            const size_t INDEX     = static_cast<size_t>(y) * WIDTH + static_cast<size_t>(x);
                            const SVec3  DIRECTION = CAMERA_POSE.dirToWorld(rays[INDEX]);

                            SRgba  pixel = VOID_COLOR;
                            double hitX = 0.0, hitY = 0.0;
                            if (traceWall(scene, CAMERA_POSE.pos, DIRECTION, hitX, hitY))
                                pixel = shadeWall(scene, hitX, hitY, static_cast<int>(j));

                            uint8_t* out = frame.data() + INDEX * 4;
                            out[0]       = pixel.r;
                            out[1]       = pixel.g;
                            out[2]       = pixel.b;
                            out[3]       = 255;
                        }
                    });

                    if (!encoder->writeFrame(frame.data(), frame.size())) {
                        HXC_ERR("the camera encoder for {} stopped accepting frames", CAMERA.key);
                        return 1;
                    }

                    // `capture`, `pts_us` and `t_xr_ns` are the producer's extra
                    // per-frame keys. Nothing here reads them; they are emitted so
                    // that "unknown keys are ignored" is exercised rather than
                    // merely asserted in prose.
                    sidecar << std::format(R"({{"cam":"{}","t_device_ns":{},"exposure_ns":{},"frame":{},"capture":{},"pts_us":{},"t_xr_ns":{}}})", CAMERA.key, CAMERA.deviceNs[j],
                                           CAMERA.exposureNs, j, j, CAMERA.deviceNs[j] / 1000, CAMERA.deviceNs[j])
                            << "\n";
                }

                if (!encoder->finish(error)) {
                    HXC_ERR("{}", error);
                    return 1;
                }
                if (!options.quiet)
                    HXC_INFO("wrote {} frames for camera {} at {}x{}", CAMERA.deviceNs.size(), CAMERA.key, WIDTH, HEIGHT);
            }
        }

        // ---- audio ----------------------------------------------------------------
        const auto writeTrack = [&](const SSynthAudio& track, double toneHz, double toneAmplitude, const fs::path& media, const fs::path& sidecar, const char* startKey) -> bool {
            const int64_t LENGTH_NS = DURATION + 400000000;
            const size_t  SAMPLES   = static_cast<size_t>(static_cast<double>(LENGTH_NS) * 1e-9 * track.sampleRate);

            std::vector<int16_t> pcm(SAMPLES, 0);
            for (size_t i = 0; i < SAMPLES; ++i) {
                const double T     = static_cast<double>(i) / track.sampleRate;
                const double VALUE = toneAmplitude * std::sin(2.0 * M_PI * toneHz * T);
                pcm[i]             = static_cast<int16_t>(std::lround(std::clamp(VALUE, -1.0, 1.0) * 32767.0));
            }

            // A four-sample impulse whose position is the assertion: it must land on
            // the output timeline at the host instant recorded in the ground truth,
            // whichever clock domain the track was stamped in.
            const int64_t OFFSET_NS   = track.clickTrackNs - track.startNs;
            const int64_t CLICK_INDEX = static_cast<int64_t>(std::llround(static_cast<double>(OFFSET_NS) * 1e-9 * track.sampleRate));
            for (int64_t i = 0; i < 4; ++i) {
                const int64_t INDEX = CLICK_INDEX + i;
                if (INDEX >= 0 && INDEX < static_cast<int64_t>(pcm.size()))
                    pcm[static_cast<size_t>(INDEX)] = static_cast<int16_t>(std::lround(track.clickAmplitude * 32767.0));
            }

            if (!encodePcmS16(media.string(), pcm, track.sampleRate, 1, error)) {
                HXC_ERR("{}", error);
                return false;
            }

            json meta;
            meta[startKey]         = track.startNs;
            meta["sample_rate_hz"] = track.sampleRate;
            meta["channels"]       = 1;
            if (media.extension() == ".wav") {
                // The device mic's sidecar, as the join writes it.
                meta["record"]             = "hypxrtake-mic";
                meta["record_version"]     = 1;
                meta["source"]             = "wivrn-mic-tee";
                meta["format"]             = "pcm_s16le";
                meta["container"]          = "wav";
                meta["t_device_ns_domain"] = "xr_time";
                meta["clock_anchor"]       = {{"xr_time_ns", track.startNs}, {"monotonic_ns", track.startNs}, {"boottime_ns", track.startNs}};
            } else
                meta["encoder"] = "flac";
            std::ofstream stream(sidecar);
            stream << meta.dump(2) << "\n";
            return true;
        };

        if (scene.hasApp && !writeTrack(scene.app, 440.0, 0.15, options.out / "audio" / "app.flac", options.out / "audio" / "app.json", "start_t_host_ns"))
            return 1;
        // At the take root, as a WAV: the producer's layout, not audio/*.flac.
        if (scene.hasMic && !writeTrack(scene.mic, 880.0, 0.10, options.out / std::format("{}-mic.wav", TAKE_ID), options.out / std::format("{}-mic.json", TAKE_ID), "start_t_device_ns"))
            return 1;

        if (!options.quiet) {
            HXC_INFO("synthesized {} ({} telemetry records over {:.3f} s)", options.out.string(), options.frames, static_cast<double>(DURATION) * 1e-9);
            HXC_INFO("clock offset starts at {:.3f} ms, drifting {:.1f} ppm; camera baseline {:.1f} mm against an IPD of {:.1f} mm", options.clockOffsetMs, options.clockDriftPpm,
                     options.cameraBaseline * 1000.0, options.ipd * 1000.0);
        }
        return 0;
    }

}
