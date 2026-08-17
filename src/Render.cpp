#include "Render.hpp"
#include "Bundle.hpp"
#include "ComposeGL.hpp"
#include "Ffmpeg.hpp"
#include "Log.hpp"
#include "Stabilize.hpp"
#include "Validate.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace hxc {

    namespace {

        using SClock = std::chrono::steady_clock;

        double secondsSince(const SClock::time_point& start) {
            return std::chrono::duration<double>(SClock::now() - start).count();
        }

        // Linear in position, slerp in rotation, between the telemetry records that
        // bracket `tHostNs`. Clamped at both ends.
        SPose interpolatePose(const std::vector<int64_t>& times, const std::vector<SPose>& poses, int64_t tHostNs) {
            if (poses.empty())
                return {};
            if (times.size() == 1 || tHostNs <= times.front())
                return poses.front();
            if (tHostNs >= times.back())
                return poses.back();

            const auto   IT   = std::upper_bound(times.begin(), times.end(), tHostNs);
            const size_t HIGH = static_cast<size_t>(IT - times.begin());
            const size_t LOW  = HIGH - 1;
            const int64_t SPAN = times[HIGH] - times[LOW];
            if (SPAN <= 0)
                return poses[LOW];

            const double T = static_cast<double>(tHostNs - times[LOW]) / static_cast<double>(SPAN);
            SPose        out;
            out.pos = poses[LOW].pos + (poses[HIGH].pos - poses[LOW].pos) * T;
            out.rot = slerp(poses[LOW].rot, poses[HIGH].rot, T);
            return out;
        }

        std::string framingName(eFraming framing) {
            return framing == eFraming::ASIS ? "asis" : "stabilized";
        }

    }

    std::string SRenderReport::toJson() const {
        json report;
        report["pane_width"]  = paneWidth;
        report["pane_height"] = paneHeight;
        report["pane_count"]  = paneCount;
        report["fps"]         = fps;
        report["first_t_host_ns"] = firstHostNs;
        report["gpu"]         = gpu;
        report["framing"]     = framing;
        report["throughput"]  = {
            {"wall_seconds", wallSeconds},        {"decode_seconds", decodeSeconds},
            {"gpu_seconds", gpuSeconds},          {"encode_seconds", encodeSeconds},
            {"frames_per_second", framesPerSecond}, {"megapixels_per_second", megapixelsPerSecond},
        };

        report["audio"] = json::array();
        for (const auto& TRACK : audio)
            report["audio"].push_back({{"role", TRACK.role}, {"start_sample", TRACK.startSample}, {"gain", TRACK.gain}});

        report["frames"] = json::array();
        for (const auto& FRAME : frames) {
            report["frames"].push_back({
                {"index", FRAME.index},
                {"t_host_ns", FRAME.tHostNs},
                {"telemetry_index", FRAME.telemetryIndex},
                {"overlay_frame", {FRAME.overlayFrame[0], FRAME.overlayFrame[1]}},
                {"overlay_telemetry_index", {FRAME.overlayTelemetryIndex[0], FRAME.overlayTelemetryIndex[1]}},
                {"camera_frame", {FRAME.cameraFrame[0], FRAME.cameraFrame[1]}},
                {"camera_t_host_ns", {FRAME.cameraHostNs[0], FRAME.cameraHostNs[1]}},
            });
        }
        return report.dump(2);
    }

    int runRender(const SRenderOptions& options, SRenderReport* reportOut) {
        SRenderReport report;
        report.framing = framingName(options.framing);

        CDiagnostics diags;
        const auto   LOADED = SBundle::load(options.take, diags, {});
        diags.print();
        if (!LOADED) {
            HXC_ERR("cannot open the bundle");
            return 2;
        }
        if (diags.hasErrors()) {
            HXC_ERR("the bundle has {} error(s); fix them or inspect with `hypxrcompose validate`", diags.errorCount());
            return 1;
        }
        const SBundle& BUNDLE = *LOADED;
        if (logEnabled(eLogLevel::INFO))
            std::fputs(describeBundle(BUNDLE).c_str(), stdout);

        if (BUNDLE.telemetry.empty()) {
            HXC_ERR("the take holds no telemetry; there is nothing to compose against");
            return 1;
        }

        // ---- panes ---------------------------------------------------------------
        std::vector<int> paneEyes;
        switch (options.eye) {
            case eEyeSelection::LEFT: paneEyes = {0}; break;
            case eEyeSelection::RIGHT: paneEyes = {1}; break;
            case eEyeSelection::STEREO_SBS: paneEyes = {0, 1}; break;
        }
        const int PANE_COUNT = static_cast<int>(paneEyes.size());
        for (int eye : paneEyes) {
            if (eye >= static_cast<int>(BUNDLE.telemetry.front().eyes.size())) {
                HXC_ERR("the take stamps {} eye(s); eye {} was requested", BUNDLE.telemetry.front().eyes.size(), eye);
                return 1;
            }
        }

        // ---- geometry ------------------------------------------------------------
        int paneWidth  = options.width;
        int paneHeight = options.height;
        if (paneWidth <= 0 || paneHeight <= 0) {
            if (BUNDLE.overlay.width > 0) {
                paneWidth  = BUNDLE.overlay.width;
                paneHeight = BUNDLE.overlay.height;
            } else if (!BUNDLE.cameras.empty() && BUNDLE.cameras.front().video.width > 0) {
                paneWidth  = BUNDLE.cameras.front().video.width;
                paneHeight = BUNDLE.cameras.front().video.height;
            } else {
                paneWidth  = 1280;
                paneHeight = 720;
            }
        }
        // H.264 and HEVC want even dimensions in both axes.
        paneWidth  &= ~1;
        paneHeight &= ~1;

        double fps = options.fps;
        if (!(fps > 0.0))
            fps = BUNDLE.overlay.targetHz;
        if (!(fps > 0.0)) {
            const int64_t INTERVAL = BUNDLE.medianTelemetryIntervalNs();
            fps                    = INTERVAL > 0 ? 1e9 / static_cast<double>(INTERVAL) : 60.0;
        }

        const int64_t T0   = BUNDLE.firstHostNs();
        const int64_t SPAN = BUNDLE.lastHostNs() - T0;
        // The epsilon is not cosmetic: at the common case of an output rate equal to
        // the capture rate the span is n-1 frames to within a few nanoseconds of
        // floating-point noise, and a bare truncation drops the last frame.
        size_t        frameCount = SPAN > 0 ? static_cast<size_t>(std::floor(static_cast<double>(SPAN) * 1e-9 * fps + 1e-6)) + 1 : 1;
        if (options.limitFrames > 0)
            frameCount = std::min(frameCount, options.limitFrames);

        // ---- pose tracks ---------------------------------------------------------
        std::vector<int64_t>            telemetryTimes = BUNDLE.telemetryHostNs;
        std::vector<SPose>              headPoses;
        std::vector<std::vector<SPose>> eyePoses(2);
        headPoses.reserve(BUNDLE.telemetry.size());
        for (const auto& RECORD : BUNDLE.telemetry) {
            headPoses.push_back(RECORD.headPose());
            for (int eye = 0; eye < 2; ++eye)
                eyePoses[static_cast<size_t>(eye)].push_back(eye < static_cast<int>(RECORD.eyes.size()) ? RECORD.eyes[static_cast<size_t>(eye)].pose : RECORD.headPose());
        }

        std::vector<SPose> smoothedHeads;
        if (options.framing == eFraming::STABILIZED) {
            std::vector<STimedPose> track;
            track.reserve(headPoses.size());
            for (size_t i = 0; i < headPoses.size(); ++i)
                track.push_back({telemetryTimes[i], headPoses[i]});
            smoothedHeads = gaussianSmoothPoses(track, options.stabilizeSigmaNs);
            HXC_INFO("stabilizing with a zero-phase Gaussian, sigma {:.0f} ms (-3 dB at {:.2f} Hz)", static_cast<double>(options.stabilizeSigmaNs) * 1e-6,
                     gaussianCutoffHz(options.stabilizeSigmaNs));
        }

        // ---- background choice ----------------------------------------------------
        eBackgroundChoice background = options.background;
        if (background == eBackgroundChoice::AUTO)
            background = BUNDLE.cameras.empty() ? eBackgroundChoice::CHECKER : eBackgroundChoice::CAMERA;
        if (background == eBackgroundChoice::CAMERA && BUNDLE.cameras.empty()) {
            HXC_WARN("--background camera was asked for but the take has no camera source; falling back to the checker");
            background = eBackgroundChoice::CHECKER;
        }

        // ---- GPU ------------------------------------------------------------------
        std::string error;
        auto        gl = CComposeGL::create(paneWidth, paneHeight, PANE_COUNT, options.gpuHint, error);
        if (!gl) {
            HXC_ERR("cannot bring up an offscreen GLES3 context: {}", error);
            return 1;
        }
        HXC_INFO("rendering {}x{} x{} pane(s) at {:.3f} fps on {}", paneWidth, paneHeight, PANE_COUNT, fps, gl->description());

        // ---- decoders -------------------------------------------------------------
        struct SPaneSources {
            int                           eye = 0;
            std::unique_ptr<CVideoReader> overlay;
            std::unique_ptr<CVideoReader> camera;
            const SCamera*                cameraMeta   = nullptr;
            int64_t                       overlayFrame = -1;
            int64_t                       cameraFrame  = -1;
        };

        std::vector<SPaneSources> panes(static_cast<size_t>(PANE_COUNT));
        for (int pane = 0; pane < PANE_COUNT; ++pane) {
            auto& SOURCES = panes[static_cast<size_t>(pane)];
            SOURCES.eye   = paneEyes[static_cast<size_t>(pane)];

            const size_t EYE = static_cast<size_t>(SOURCES.eye);
            if (BUNDLE.sources.overlay && EYE < BUNDLE.overlay.videoPaths.size() && !BUNDLE.overlay.videoPaths[EYE].empty()) {
                SOURCES.overlay = CVideoReader::open(BUNDLE.overlay.videoPaths[EYE], BUNDLE.overlay.width, BUNDLE.overlay.height, error);
                if (!SOURCES.overlay) {
                    HXC_ERR("{}", error);
                    return 1;
                }
            }

            if (background == eBackgroundChoice::CAMERA) {
                SOURCES.cameraMeta = BUNDLE.cameraForEye(SOURCES.eye);
                if (!SOURCES.cameraMeta) {
                    HXC_WARN("no camera for eye {}; that pane gets the checker background", SOURCES.eye);
                } else {
                    SOURCES.camera = CVideoReader::open(SOURCES.cameraMeta->videoPath, SOURCES.cameraMeta->video.width, SOURCES.cameraMeta->video.height, error);
                    if (!SOURCES.camera) {
                        HXC_ERR("{}", error);
                        return 1;
                    }
                }
            }
        }

        // ---- audio ----------------------------------------------------------------
        SWriterSpec spec;
        spec.outPath     = options.outPath;
        spec.width       = paneWidth * PANE_COUNT;
        spec.height      = paneHeight;
        spec.fps         = fps;
        spec.videoCodec  = options.videoCodec;
        spec.crf         = options.crf;
        spec.preset      = options.preset;
        spec.limiterCeiling = options.noLimiter ? 0.0 : 0.98;

        if (!options.noAudio) {
            const auto place = [&](const SAudioTrack& track, double gain) {
                SAudioMixInput input;
                input.path  = track.path;
                input.gain  = gain;
                input.label = track.role;
                // The track's first sample belongs at its start stamp on the host
                // timeline; the output's sample zero is at T0. Device-clocked tracks
                // have already been mapped through the clock series by the loader.
                input.startSample = static_cast<int64_t>(std::llround(static_cast<double>(track.startHostNs - T0) * 1e-9 * spec.audioSampleRate));
                HXC_INFO("audio {}: starts {:+.3f} ms into the take -> sample {:+}", track.role, static_cast<double>(track.startHostNs - T0) * 1e-6, input.startSample);
                spec.audio.push_back(input);
                report.audio.push_back({track.role, input.startSample, gain});
            };
            if (BUNDLE.appAudio)
                place(*BUNDLE.appAudio, 1.0);
            if (BUNDLE.mic)
                place(*BUNDLE.mic, options.micGain);
        }

        auto writer = CVideoWriter::open(spec, error);
        if (!writer) {
            HXC_ERR("{}", error);
            return 1;
        }

        if (!options.framesDir.empty()) {
            std::error_code ec;
            fs::create_directories(options.framesDir, ec);
        }

        // ---- the loop -------------------------------------------------------------
        report.paneWidth   = paneWidth;
        report.paneHeight  = paneHeight;
        report.paneCount   = PANE_COUNT;
        report.fps         = fps;
        report.firstHostNs = T0;
        report.gpu         = gl->description();
        report.frames.reserve(frameCount);

        std::vector<uint8_t> composed;
        double               sidecarSeconds = 0.0;
        const auto           STARTED        = SClock::now();

        for (size_t k = 0; k < frameCount; ++k) {
            const int64_t T_HOST = T0 + static_cast<int64_t>(std::llround(static_cast<double>(k) * 1e9 / fps));

            SRenderReport::SFrameRecord record;
            record.index   = k;
            record.tHostNs = T_HOST;

            const auto TELEMETRY_INDEX = nearestIndex(telemetryTimes, T_HOST);
            if (!TELEMETRY_INDEX)
                break;
            record.telemetryIndex = *TELEMETRY_INDEX;
            const auto& STAMPED   = BUNDLE.telemetry[*TELEMETRY_INDEX];

            for (int pane = 0; pane < PANE_COUNT; ++pane) {
                auto&        sources = panes[static_cast<size_t>(pane)];
                const size_t EYE     = static_cast<size_t>(sources.eye);

                SPaneDraw draw;
                draw.outputFov = STAMPED.eyes[std::min(EYE, STAMPED.eyes.size() - 1)].fov;

                if (options.framing == eFraming::STABILIZED && !smoothedHeads.empty()) {
                    // Re-apply the eye's own offset from the head so the smoothed
                    // camera keeps the recorded IPD and eye orientation.
                    const SPose& HEAD     = headPoses[*TELEMETRY_INDEX];
                    const SPose& EYE_POSE = eyePoses[EYE][*TELEMETRY_INDEX];
                    draw.outputCamera     = smoothedHeads[*TELEMETRY_INDEX].compose(HEAD.inverse().compose(EYE_POSE));
                } else
                    draw.outputCamera = eyePoses[EYE][*TELEMETRY_INDEX];

                // Overlay. The video's n-th frame is the n-th telemetry record
                // without `dropped`, so the frame is chosen among *those* records'
                // times, and the pose it is reprojected from is that record's - not
                // the output instant's. When frames were dropped the two differ, and
                // using the output instant's pose would warp the overlay from a
                // viewpoint it was never rendered at.
                const auto DECODE_START = SClock::now();
                if (sources.overlay && !BUNDLE.overlay.frameHostNs.empty()) {
                    const auto FRAME = nearestIndex(BUNDLE.overlay.frameHostNs, T_HOST);
                    if (FRAME) {
                        if (!sources.overlay->advanceTo(*FRAME, error)) {
                            HXC_ERR("{}", error);
                            return 1;
                        }
                        const auto CURRENT = sources.overlay->currentIndex();
                        if (CURRENT && static_cast<int64_t>(*CURRENT) != sources.overlayFrame) {
                            gl->uploadOverlay(pane, sources.overlay->rgba().data(), BUNDLE.overlay.width, BUNDLE.overlay.height);
                            sources.overlayFrame = static_cast<int64_t>(*CURRENT);
                        }

                        const size_t SOURCE_RECORD = BUNDLE.overlay.frameTelemetryIndex[*FRAME];
                        record.overlayFrame[EYE < 2 ? EYE : 0]          = sources.overlayFrame;
                        record.overlayTelemetryIndex[EYE < 2 ? EYE : 0] = static_cast<int64_t>(SOURCE_RECORD);
                        draw.hasOverlay                                 = true;
                        draw.overlayPose                                = eyePoses[EYE][SOURCE_RECORD];
                        draw.overlayFov                                 = BUNDLE.telemetry[SOURCE_RECORD].eyes[std::min(EYE, BUNDLE.telemetry[SOURCE_RECORD].eyes.size() - 1)].fov;
                        draw.overlayDepth                               = options.overlayDepth;
                        draw.premultipliedAlpha                         = BUNDLE.overlay.alpha == "premultiplied";
                    }
                }

                // Background.
                draw.backgroundMode = background == eBackgroundChoice::SOLID ? eBackgroundMode::SOLID : eBackgroundMode::CHECKER;
                if (sources.camera && sources.cameraMeta && !sources.cameraMeta->hostNs.empty()) {
                    const auto FRAME = nearestIndex(sources.cameraMeta->hostNs, T_HOST);
                    if (FRAME) {
                        if (!sources.camera->advanceTo(*FRAME, error)) {
                            HXC_ERR("{}", error);
                            return 1;
                        }
                        const auto CURRENT = sources.camera->currentIndex();
                        if (CURRENT && static_cast<int64_t>(*CURRENT) != sources.cameraFrame) {
                            gl->uploadBackground(pane, sources.camera->rgba().data(), sources.cameraMeta->video.width, sources.cameraMeta->video.height);
                            sources.cameraFrame = static_cast<int64_t>(*CURRENT);
                        }

                        const int64_t CAMERA_HOST = sources.cameraMeta->hostNs[*FRAME];
                        record.cameraFrame[EYE < 2 ? EYE : 0]  = sources.cameraFrame;
                        record.cameraHostNs[EYE < 2 ? EYE : 0] = CAMERA_HOST;

                        // The pose chain, in one line: the head where it was at the
                        // frame's mid-exposure instant, composed with the lens's
                        // factory extrinsic.
                        draw.cameraPose       = interpolatePose(telemetryTimes, headPoses, CAMERA_HOST).compose(sources.cameraMeta->headToCamera);
                        draw.intrinsics       = sources.cameraMeta->intrinsics;
                        draw.backgroundWidth  = sources.cameraMeta->video.width;
                        draw.backgroundHeight = sources.cameraMeta->video.height;
                        draw.backgroundDepth  = options.backgroundDepth;
                        draw.backgroundMode   = eBackgroundMode::CAMERA;
                    }
                }
                report.decodeSeconds += secondsSince(DECODE_START);

                const auto GPU_START = SClock::now();
                if (!gl->drawPane(pane, draw, error)) {
                    HXC_ERR("{}", error);
                    return 1;
                }
                report.gpuSeconds += secondsSince(GPU_START);
            }

            const auto READBACK_START = SClock::now();
            if (!gl->readback(composed)) {
                HXC_ERR("reading the composed frame back from the GPU failed");
                return 1;
            }
            report.gpuSeconds += secondsSince(READBACK_START);

            const auto ENCODE_START = SClock::now();
            if (!writer->writeFrame(composed.data(), composed.size())) {
                HXC_ERR("the encoder stopped accepting frames at output frame {}", k);
                return 1;
            }
            report.encodeSeconds += secondsSince(ENCODE_START);

            if (!options.framesDir.empty()) {
                // Debug output, and an expensive one: each PNG is its own ffmpeg. Its
                // cost is measured separately and taken back out of the throughput
                // figure, which is meant to describe the composite, not the dump.
                const auto        PNG_START = SClock::now();
                const std::string PNG       = (fs::path(options.framesDir) / std::format("frame_{:06}.png", k)).string();
                if (!writePng(PNG, composed.data(), paneWidth * PANE_COUNT, paneHeight, error))
                    HXC_WARN("{}", error);
                sidecarSeconds += secondsSince(PNG_START);
            }

            report.frames.push_back(record);

            if (k % 100 == 0 && k > 0)
                HXC_DEBUG("composed {} / {} frames", k, frameCount);
        }

        report.wallSeconds = secondsSince(STARTED) - sidecarSeconds;

        if (!writer->finish(error)) {
            HXC_ERR("{}", error);
            return 1;
        }

        const double MEGAPIXELS = static_cast<double>(report.frames.size()) * static_cast<double>(paneWidth * PANE_COUNT) * static_cast<double>(paneHeight) * 1e-6;
        report.framesPerSecond     = report.wallSeconds > 0.0 ? static_cast<double>(report.frames.size()) / report.wallSeconds : 0.0;
        report.megapixelsPerSecond = report.wallSeconds > 0.0 ? MEGAPIXELS / report.wallSeconds : 0.0;

        HXC_INFO("composed {} frames of {}x{} in {:.2f} s = {:.1f} fps, {:.1f} Mpix/s (decode {:.2f} s, gpu {:.2f} s, encode {:.2f} s)", report.frames.size(), paneWidth * PANE_COUNT, paneHeight,
                 report.wallSeconds, report.framesPerSecond, report.megapixelsPerSecond, report.decodeSeconds, report.gpuSeconds, report.encodeSeconds);
        HXC_INFO("wrote {}", options.outPath);

        if (!options.reportPath.empty()) {
            std::ofstream stream(options.reportPath);
            if (!stream)
                HXC_WARN("cannot write the render report to {}", options.reportPath);
            else
                stream << report.toJson() << "\n";
        }

        if (reportOut)
            *reportOut = std::move(report);
        return 0;
    }

}
