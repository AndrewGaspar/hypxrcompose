#include "Render.hpp"
#include "Bundle.hpp"
#include "ComposeGL.hpp"
#include "Ffmpeg.hpp"
#include "Log.hpp"
#include "Process.hpp"
#include "Stabilize.hpp"
#include "Validate.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <thread>
#include <unistd.h>

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

    std::pair<size_t, size_t> segmentRange(size_t frameCount, int index, int count) {
        if (count <= 1 || index < 0)
            return {0, frameCount};
        if (index >= count)
            return {frameCount, frameCount};

        const size_t BASE      = frameCount / static_cast<size_t>(count);
        const size_t REMAINDER = frameCount % static_cast<size_t>(count);
        const size_t I         = static_cast<size_t>(index);
        // The first REMAINDER segments carry one extra frame, so the chunk sizes
        // differ by at most one and the boundaries are a closed form rather than a
        // running sum - which is what lets a worker compute its own range without
        // being told the others'.
        const size_t BEGIN = I * BASE + std::min(I, REMAINDER);
        const size_t END   = BEGIN + BASE + (I < REMAINDER ? 1 : 0);
        return {BEGIN, END};
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
        report["jobs"]        = jobs;
        report["pane_fov"]    = json::array();
        for (const auto& FOV : paneFov)
            report["pane_fov"].push_back({{"l", FOV.l}, {"r", FOV.r}, {"u", FOV.u}, {"d", FOV.d}});
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

    bool SRenderReport::fromJson(const std::string& text, SRenderReport& out, std::string& error) {
        out = {};
        try {
            const json REPORT = json::parse(text);
            out.paneWidth     = REPORT.value("pane_width", 0);
            out.paneHeight    = REPORT.value("pane_height", 0);
            out.paneCount     = REPORT.value("pane_count", 1);
            out.fps           = REPORT.value("fps", 0.0);
            out.firstHostNs   = REPORT.value("first_t_host_ns", int64_t{0});
            out.gpu           = REPORT.value("gpu", std::string{});
            out.framing       = REPORT.value("framing", std::string{});
            out.jobs          = REPORT.value("jobs", 1);
            for (const auto& FOV : REPORT.value("pane_fov", json::array()))
                out.paneFov.push_back({FOV.value("l", 0.0), FOV.value("r", 0.0), FOV.value("u", 0.0), FOV.value("d", 0.0)});

            if (const auto IT = REPORT.find("throughput"); IT != REPORT.end()) {
                out.wallSeconds   = IT->value("wall_seconds", 0.0);
                out.decodeSeconds = IT->value("decode_seconds", 0.0);
                out.gpuSeconds    = IT->value("gpu_seconds", 0.0);
                out.encodeSeconds = IT->value("encode_seconds", 0.0);
            }

            for (const auto& FRAME : REPORT.value("frames", json::array())) {
                SFrameRecord record;
                record.index          = FRAME.value("index", size_t{0});
                record.tHostNs        = FRAME.value("t_host_ns", int64_t{0});
                record.telemetryIndex = FRAME.value("telemetry_index", size_t{0});
                const auto pair       = [&](const char* key, std::array<int64_t, 2>& into) {
                    const auto IT = FRAME.find(key);
                    if (IT == FRAME.end() || !IT->is_array() || IT->size() != 2)
                        return;
                    into[0] = (*IT)[0].get<int64_t>();
                    into[1] = (*IT)[1].get<int64_t>();
                };
                pair("overlay_frame", record.overlayFrame);
                pair("overlay_telemetry_index", record.overlayTelemetryIndex);
                pair("camera_frame", record.cameraFrame);
                pair("camera_t_host_ns", record.cameraHostNs);
                out.frames.push_back(record);
            }

            for (const auto& TRACK : REPORT.value("audio", json::array()))
                out.audio.push_back({TRACK.value("role", std::string{}), TRACK.value("start_sample", int64_t{0}), TRACK.value("gain", 1.0)});
        } catch (const std::exception& e) {
            error = std::format("cannot read a render report: {}", e.what());
            return false;
        }
        return true;
    }

    namespace {

        // Everything about the output that does not depend on *which* frames are
        // being composed. The parent of a segmented render and each of its workers
        // both derive this from the same bundle and the same options, so they agree
        // on the timeline by construction rather than by being told.
        struct SRenderPlan {
            std::vector<int>  paneEyes;
            int               paneWidth  = 0;
            int               paneHeight = 0;
            double            fps        = 0.0;
            int64_t           t0         = 0;
            size_t            frameCount = 0; // the whole timeline, before any segmenting
            eBackgroundChoice background = eBackgroundChoice::CHECKER;
            // The output camera's frustum, one per pane, derived once from the
            // recorded eye frusta and fixed for the whole render. See
            // deriveOutputFrusta.
            std::vector<SFov> paneFov;
        };

        // Builds each pane's output frustum from what the eyes actually recorded.
        //
        // Two things have to be reconciled and neither is negotiable. A recorded
        // eye frustum is asymmetric - on the reference take l=-0.507 r=0.698
        // u=0.727 d=-0.648, so the optical axis sits at 39.8% of the width, not at
        // half - and its angular aspect is whatever the runtime picked, 0.847 on
        // that take, against an eye buffer whose pixel aspect is 0.955. Handing
        // that fov straight to the shader maps the frustum linearly onto whatever
        // pane the user asked for, which stretches the picture by the ratio of the
        // two aspects: 12.8% at the take's own buffer size, and 110% at 1920x1080.
        // Circles came out as ellipses and straight lines survived only because
        // the stretch is uniform.
        //
        // So: keep the recorded frustum, pad it - never crop - until it fills the
        // pane at one tan-units-per-pixel scale, and keep that scale common to
        // both eyes of a stereo pair so the pair stays a pair. The result has
        // angularly square pixels (a world square renders square), keeps every
        // recorded pixel, and leaves the optical axis pointing exactly where the
        // eye was pointing.
        //
        // Derived once rather than per frame: the fov is a property of the
        // headset, and rebuilding it from each record's stamp would make the
        // framing breathe if a producer ever varied it.
        void deriveOutputFrusta(const SRenderOptions& options, const SBundle& BUNDLE, SRenderPlan& plan) {
            const size_t PANES = plan.paneEyes.size();
            plan.paneFov.assign(PANES, SFov{});

            if (options.frustum == eFrustumMode::PRESENTATION) {
                // ONE frustum, symmetric about forward, shared by every pane.
                //
                // A flat side-by-side viewer - or a person fusing two panes on a
                // desktop - needs both frames to subtend the same angles. Give
                // each eye its own asymmetric frustum and they do not: on the
                // reference take the optical axes sit at 62.1% and 37.9% of the
                // width, so a feature at infinity lands 242 px apart at 1440
                // wide. That is a constant disparity the wearer cannot fuse, and
                // the frame edges land at different visual angles, which reads as
                // the two images floating apart in opposite directions. Reported
                // from an in-headset viewing, which is the only place it shows.
                //
                // So: intersect what the eyes recorded, symmetrize about forward,
                // and crop to the pane. Every pane then shares one frustum and
                // one scale, and stereo parallax is carried entirely by the eye
                // positions - by the content - and never by frame placement.
                // Periphery outside the shared frustum is dropped, which is the
                // price of a fusable picture.
                //
                // Derived from every eye the take holds, not just the panes being
                // rendered, so a mono render frames identically to one pane of
                // the stereo pair.
                //
                // Per edge this takes the MEDIAN across records, not the
                // intersection. The reference take's stamped fov is not constant
                // - eye 0's `u` runs from 0.3964 to 0.7679 - and a strict
                // intersection would let the single narrowest frame of a
                // 92-second take decide the framing for all of it, cropping the
                // other 4600 records to a keyhole. The median lands on the value
                // the take actually holds nearly all the time; the few narrower
                // records simply run out of overlay near the edge, which the
                // sampler already handles by falling through to the background.
                const auto medianEdge = [&](auto pick) {
                    std::vector<double> values;
                    values.reserve(BUNDLE.telemetry.size());
                    for (const auto& RECORD : BUNDLE.telemetry) {
                        for (const auto& EYE : RECORD.eyes)
                            values.push_back(pick(EYE.fov));
                    }
                    if (values.empty())
                        return 0.0;
                    std::nth_element(values.begin(), values.begin() + static_cast<long>(values.size() / 2), values.end());
                    return values[values.size() / 2];
                };

                // Intersecting the two eyes is the same as taking the inner edge
                // on each side, which for a mirrored pair the median already is;
                // symmetrizeFov below makes that explicit and exact.
                std::optional<SFov> shared;
                if (!BUNDLE.telemetry.empty() && !BUNDLE.telemetry.front().eyes.empty()) {
                    shared = SFov{medianEdge([](const SFov& f) { return f.l; }), medianEdge([](const SFov& f) { return f.r; }), medianEdge([](const SFov& f) { return f.u; }),
                                  medianEdge([](const SFov& f) { return f.d; })};
                }
                if (!shared) {
                    plan.paneFov.assign(PANES, SFov{});
                    return;
                }

                const SFov SYMMETRIC = symmetrizeFov(*shared);
                const SFov FITTED    = cropFovToPane(SYMMETRIC, plan.paneWidth, plan.paneHeight);
                plan.paneFov.assign(PANES, FITTED);

                HXC_INFO("output frustum (presentation): both eyes share l={:.4f} r={:.4f} u={:.4f} d={:.4f} - symmetric about forward, aspect {:.4f} matching the {}x{} pane, optical axis at "
                         "the pane centre. Kept {:.0f}% of the horizontal field the eyes recorded and {:.0f}% of the vertical; parallax is carried by the content, not by frame placement",
                         FITTED.l, FITTED.r, FITTED.u, FITTED.d, FITTED.angularAspect(), plan.paneWidth, plan.paneHeight, 100.0 * FITTED.tanWidth() / shared->tanWidth(),
                         100.0 * FITTED.tanHeight() / shared->tanHeight());
                return;
            }

            // ---- eFrustumMode::RECORDED ------------------------------------------
            // The union of every frustum this eye recorded, so nothing any record
            // saw is cropped away.
            std::vector<SFov> recorded(PANES);
            std::vector<bool> seen(PANES, false);
            for (const auto& RECORD : BUNDLE.telemetry) {
                for (size_t pane = 0; pane < PANES; ++pane) {
                    const size_t EYE = static_cast<size_t>(plan.paneEyes[pane]);
                    if (EYE >= RECORD.eyes.size())
                        continue;
                    const SFov& FOV = RECORD.eyes[EYE].fov;
                    if (!seen[pane]) {
                        recorded[pane] = FOV;
                        seen[pane]     = true;
                        continue;
                    }
                    recorded[pane].l = std::min(recorded[pane].l, FOV.l);
                    recorded[pane].r = std::max(recorded[pane].r, FOV.r);
                    recorded[pane].u = std::max(recorded[pane].u, FOV.u);
                    recorded[pane].d = std::min(recorded[pane].d, FOV.d);
                }
            }

            // One scale for every pane: fitting two eyes independently would put
            // them at two different magnifications, which is not a stereo pair.
            double angularPixel = 0.0;
            for (size_t pane = 0; pane < PANES; ++pane) {
                if (seen[pane])
                    angularPixel = std::max(angularPixel, angularPixelForPane(recorded[pane], plan.paneWidth, plan.paneHeight));
            }

            for (size_t pane = 0; pane < PANES; ++pane) {
                plan.paneFov[pane] = seen[pane] ? fitFovToPane(recorded[pane], plan.paneWidth, plan.paneHeight, angularPixel) : SFov{};
                if (!seen[pane])
                    continue;
                HXC_INFO("pane {} (eye {}): recorded fov l={:.4f} r={:.4f} u={:.4f} d={:.4f} (angular aspect {:.4f}, optical centre {:.1f}% x {:.1f}%) -> output fov l={:.4f} r={:.4f} u={:.4f} "
                         "d={:.4f} (aspect {:.4f} matching the {}x{} pane, optical centre {:.1f}% x {:.1f}%)",
                         pane, plan.paneEyes[pane], recorded[pane].l, recorded[pane].r, recorded[pane].u, recorded[pane].d, recorded[pane].angularAspect(), recorded[pane].opticalCentreU() * 100.0,
                         recorded[pane].opticalCentreV() * 100.0, plan.paneFov[pane].l, plan.paneFov[pane].r, plan.paneFov[pane].u, plan.paneFov[pane].d, plan.paneFov[pane].angularAspect(),
                         plan.paneWidth, plan.paneHeight, plan.paneFov[pane].opticalCentreU() * 100.0, plan.paneFov[pane].opticalCentreV() * 100.0);
            }
        }

        bool makePlan(const SRenderOptions& options, const SBundle& BUNDLE, SRenderPlan& out) {
            // ---- panes -----------------------------------------------------------
            switch (options.eye) {
                case eEyeSelection::LEFT: out.paneEyes = {0}; break;
                case eEyeSelection::RIGHT: out.paneEyes = {1}; break;
                case eEyeSelection::STEREO_SBS: out.paneEyes = {0, 1}; break;
            }
            for (int eye : out.paneEyes) {
                if (eye >= static_cast<int>(BUNDLE.telemetry.front().eyes.size())) {
                    HXC_ERR("the take stamps {} eye(s); eye {} was requested", BUNDLE.telemetry.front().eyes.size(), eye);
                    return false;
                }
            }

            // ---- geometry --------------------------------------------------------
            out.paneWidth  = options.width;
            out.paneHeight = options.height;
            if (out.paneWidth <= 0 || out.paneHeight <= 0) {
                if (BUNDLE.overlay.width > 0) {
                    out.paneWidth  = BUNDLE.overlay.width;
                    out.paneHeight = BUNDLE.overlay.height;
                } else if (!BUNDLE.cameras.empty() && BUNDLE.cameras.front().video.width > 0) {
                    out.paneWidth  = BUNDLE.cameras.front().video.width;
                    out.paneHeight = BUNDLE.cameras.front().video.height;
                } else {
                    out.paneWidth  = 1280;
                    out.paneHeight = 720;
                }
            }
            // H.264 and HEVC want even dimensions in both axes.
            out.paneWidth  &= ~1;
            out.paneHeight &= ~1;

            out.fps = options.fps;
            if (!(out.fps > 0.0))
                out.fps = BUNDLE.overlay.targetHz;
            if (!(out.fps > 0.0)) {
                const int64_t INTERVAL = BUNDLE.medianTelemetryIntervalNs();
                out.fps                = INTERVAL > 0 ? 1e9 / static_cast<double>(INTERVAL) : 60.0;
            }

            out.t0             = BUNDLE.firstHostNs();
            const int64_t SPAN = BUNDLE.lastHostNs() - out.t0;
            // The epsilon is not cosmetic: at the common case of an output rate equal
            // to the capture rate the span is n-1 frames to within a few nanoseconds
            // of floating-point noise, and a bare truncation drops the last frame.
            out.frameCount = SPAN > 0 ? static_cast<size_t>(std::floor(static_cast<double>(SPAN) * 1e-9 * out.fps + 1e-6)) + 1 : 1;
            if (options.limitFrames > 0)
                out.frameCount = std::min(out.frameCount, options.limitFrames);

            // ---- background choice ------------------------------------------------
            out.background = options.background;
            if (out.background == eBackgroundChoice::AUTO)
                out.background = BUNDLE.cameras.empty() ? eBackgroundChoice::CHECKER : eBackgroundChoice::CAMERA;
            if (out.background == eBackgroundChoice::CAMERA && BUNDLE.cameras.empty()) {
                HXC_WARN("--background camera was asked for but the take has no camera source; falling back to the checker");
                out.background = eBackgroundChoice::CHECKER;
            }

            deriveOutputFrusta(options, BUNDLE, out);
            return true;
        }

        // The output instant of frame k. Depends on nothing but k, so a worker
        // starting at k = 1500 lands on exactly the instant the single-job run's
        // 1500th iteration did.
        int64_t outputHostNs(const SRenderPlan& PLAN, size_t k) {
            return PLAN.t0 + static_cast<int64_t>(std::llround(static_cast<double>(k) * 1e9 / PLAN.fps));
        }

        // Composes output frames [beginFrame, endFrame) into `outPath`.
        //
        // This is the whole compositor. A single-job render calls it once over the
        // whole timeline; a worker calls it once over its chunk. There is no second
        // code path, which is the entire argument for why the two produce the same
        // pixels.
        //
        // HOW EXACTLY THE SAME, measured rather than assumed.
        //
        // Which source frame every output frame draws on is identical whatever
        // --jobs was: that is arithmetic on the frame index, and the suite
        // asserts it frame by frame. Nearly every composed frame is then
        // byte-identical as well - but not quite all of them. A few come back
        // differing by one or two least-significant bits on a handful of pixels
        // out of ~170000, and only where the background is a photographic camera
        // texture; a flat synthetic overlay never shows it.
        //
        // That residue belongs to the driver rather than to this code. The same
        // binary composing the same range twice is bit-exact, so what varies is
        // how equal two *processes* can be made to be, and three attempts to
        // equalize the GL state - uploading through a single path, allocating
        // every texture up front, composing a throwaway frame first - each left
        // it precisely where it was. Worth keeping the scale in view: against
        // the ~10000 pixels at 161 LSB that a real off-by-one in the seek
        // produced while this was being debugged, the suite's tolerance of
        // <0.1% of pixels at <=4 LSB is three orders of magnitude tighter, so
        // nothing real can hide inside it.
        int composeRange(const SRenderOptions& options, const SBundle& BUNDLE, const SRenderPlan& PLAN, size_t beginFrame, size_t endFrame, const std::string& outPath, bool withAudio,
                         SRenderReport& report) {
            const int    PANE_COUNT = static_cast<int>(PLAN.paneEyes.size());
            const auto&  paneEyes   = PLAN.paneEyes;
            const int    paneWidth  = PLAN.paneWidth;
            const int    paneHeight = PLAN.paneHeight;
            const double fps        = PLAN.fps;
            const int64_t T0        = PLAN.t0;
            const eBackgroundChoice background = PLAN.background;
            const size_t frameCount = PLAN.frameCount;

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

        // A range that does not start at zero starts its decoders partway along
        // too. The frame to start at is not a guess: it is the same
        // nearestIndex() the loop below would have arrived at by walking there,
        // so the worker's first iteration sees exactly the source frame the
        // single-job run's iteration k = beginFrame saw. Where the source cannot
        // be seeked exactly (an inter-coded camera, say) the reader decodes from
        // the beginning and the answer is still right, only slower.
        const auto startFrameFor = [&](const std::vector<int64_t>& hostNs) -> size_t {
            if (beginFrame == 0 || hostNs.empty())
                return 0;
            const auto INDEX = nearestIndex(hostNs, outputHostNs(PLAN, beginFrame));
            return INDEX ? *INDEX : 0;
        };
        const auto readerOptionsFor = [&](const SVideoInfo& info, size_t startFrame, const std::string& label) {
            SReaderOptions reader;
            reader.threads = options.decodeThreads;
            if (startFrame == 0)
                return reader;
            const auto SEEK = seekSecondsForFrame(info, startFrame);
            if (!SEEK) {
                HXC_WARN("{}: cannot seek to frame {} exactly ({} is not intra-only, or its timestamps do not allow it); decoding from the start of the file instead", label, startFrame,
                         info.codecName);
                return reader;
            }
            reader.startFrame  = startFrame;
            reader.seekSeconds = *SEEK;
            return reader;
        };

        std::vector<SPaneSources> panes(static_cast<size_t>(PANE_COUNT));
        for (int pane = 0; pane < PANE_COUNT; ++pane) {
            auto& SOURCES = panes[static_cast<size_t>(pane)];
            SOURCES.eye   = paneEyes[static_cast<size_t>(pane)];

            const size_t EYE = static_cast<size_t>(SOURCES.eye);
            if (BUNDLE.sources.overlay && EYE < BUNDLE.overlay.videoPaths.size() && !BUNDLE.overlay.videoPaths[EYE].empty()) {
                const auto READER = readerOptionsFor(BUNDLE.overlay.videoInfo[EYE], startFrameFor(BUNDLE.overlay.frameHostNs), BUNDLE.overlay.videoPaths[EYE]);
                SOURCES.overlay   = CVideoReader::open(BUNDLE.overlay.videoPaths[EYE], BUNDLE.overlay.width, BUNDLE.overlay.height, error, READER);
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
                    const auto READER = readerOptionsFor(SOURCES.cameraMeta->video, startFrameFor(SOURCES.cameraMeta->hostNs), SOURCES.cameraMeta->videoPath);
                    SOURCES.camera    = CVideoReader::open(SOURCES.cameraMeta->videoPath, SOURCES.cameraMeta->video.width, SOURCES.cameraMeta->video.height, error, READER);
                    if (!SOURCES.camera) {
                        HXC_ERR("{}", error);
                        return 1;
                    }
                }
            }
        }

        // ---- audio ----------------------------------------------------------------
        SWriterSpec spec;
        spec.outPath     = outPath;
        spec.width       = paneWidth * PANE_COUNT;
        spec.height      = paneHeight;
        spec.fps         = fps;
        spec.videoCodec  = options.videoCodec;
        spec.crf         = options.crf;
        spec.preset      = options.preset;
        spec.threads     = options.decodeThreads;
        spec.limiterCeiling = options.noLimiter ? 0.0 : 0.98;
        // A worker writes a chunk of the same stereo pair, so its segment is
        // signalled too - the frame-packing SEI has to be in the bitstream
        // before the join stream-copies it.
        spec.stereoSideBySide = options.eye == eEyeSelection::STEREO_SBS;

        if (withAudio && !options.noAudio) {
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
        report.paneFov     = PLAN.paneFov;
        report.frames.reserve(endFrame - beginFrame);

        std::vector<uint8_t> composed;
        double               sidecarSeconds = 0.0;
        auto                 started        = SClock::now();

        for (size_t k = beginFrame; k < endFrame; ++k) {
            const int64_t T_HOST = outputHostNs(PLAN, k);

            SRenderReport::SFrameRecord record;
            record.index   = k;
            record.tHostNs = T_HOST;

            const auto TELEMETRY_INDEX = nearestIndex(telemetryTimes, T_HOST);
            if (!TELEMETRY_INDEX)
                break;
            record.telemetryIndex = *TELEMETRY_INDEX;

            for (int pane = 0; pane < PANE_COUNT; ++pane) {
                auto&        sources = panes[static_cast<size_t>(pane)];
                const size_t EYE     = static_cast<size_t>(sources.eye);

                SPaneDraw draw;
                // Fixed for the whole render and built from the recorded frusta;
                // see deriveOutputFrusta. Not the raw per-record stamp, which is
                // the *source's* frustum - that one is still used, below, to
                // sample the overlay.
                draw.outputFov = PLAN.paneFov[static_cast<size_t>(pane)];

                if (options.framing == eFraming::STABILIZED && !smoothedHeads.empty()) {
                    // Re-apply the eye's own offset from the head so the smoothed
                    // camera keeps the recorded IPD and eye orientation.
                    const SPose& HEAD     = headPoses[*TELEMETRY_INDEX];
                    const SPose& EYE_POSE = eyePoses[EYE][*TELEMETRY_INDEX];
                    draw.outputCamera     = smoothedHeads[*TELEMETRY_INDEX].compose(HEAD.inverse().compose(EYE_POSE));
                } else
                    draw.outputCamera = eyePoses[EYE][*TELEMETRY_INDEX];

                if (options.frustum == eFrustumMode::PRESENTATION) {
                    // A parallel rig: both panes look the same way, and the only
                    // thing that differs between them is where they look *from*.
                    // Keeping each eye's own orientation would toe the cameras in
                    // by whatever the runtime stamped and reintroduce a disparity
                    // at infinity - the very thing the shared frustum removes.
                    // The position is left alone, because that is what carries
                    // the parallax.
                    draw.outputCamera.rot = (options.framing == eFraming::STABILIZED && !smoothedHeads.empty()) ? smoothedHeads[*TELEMETRY_INDEX].rot : headPoses[*TELEMETRY_INDEX].rot;
                }

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

        report.wallSeconds = secondsSince(started) - sidecarSeconds;

        if (!writer->finish(error)) {
            HXC_ERR("{}", error);
            return 1;
        }

        const double MEGAPIXELS = static_cast<double>(report.frames.size()) * static_cast<double>(paneWidth * PANE_COUNT) * static_cast<double>(paneHeight) * 1e-6;
        report.framesPerSecond     = report.wallSeconds > 0.0 ? static_cast<double>(report.frames.size()) / report.wallSeconds : 0.0;
        report.megapixelsPerSecond = report.wallSeconds > 0.0 ? MEGAPIXELS / report.wallSeconds : 0.0;

        HXC_INFO("composed {} frames of {}x{} in {:.2f} s = {:.1f} fps, {:.1f} Mpix/s (decode {:.2f} s, gpu {:.2f} s, encode {:.2f} s)", report.frames.size(), paneWidth * PANE_COUNT, paneHeight,
                 report.wallSeconds, report.framesPerSecond, report.megapixelsPerSecond, report.decodeSeconds, report.gpuSeconds, report.encodeSeconds);
        HXC_INFO("wrote {}", outPath);
        return 0;
        }

        // How many workers, when the caller did not say. One - that is, none.
        //
        // This is a measured default, and it is not the one this code was written
        // expecting. Segmenting only pays when the serial pipeline leaves the
        // machine idle, and on the reference take it does not: a single-job render
        // already keeps 17 to 20 of 24 hardware threads busy, because ffmpeg's
        // ffv1 decode and x264's encode are each already multithreaded across the
        // whole machine. Running two whole renders side by side takes exactly
        // twice as long as running one (measured: 2.0x wall for 2x the work, 4.0x
        // for 4x) - so there is no idle capacity for a second worker to take, and
        // every worker added is pure overhead. Measured on the real bundle: --jobs
        // 4 came in 1.07x *slower* than --jobs 1, --jobs 8 1.10x slower.
        //
        // The machinery stays because the conclusion is about this workload on
        // this machine, not about the idea: --jobs pays wherever the per-frame
        // cost is not already spread across every core (a GPU-bound composite at
        // higher output resolutions, a hardware encoder that takes the encode off
        // the CPU entirely), and --segment i/k is how a long take gets split
        // across several machines or resumed after a failure. What it must not do
        // is cost a user time by default.
        //
        // See NEXT-STEPS "Throughput" for the numbers and for where the time
        // actually goes.
        int defaultJobs() {
            return 1;
        }

        // A worker's ffmpeg thread budget. The machine's hardware threads shared
        // out, floored at one: K workers each asking ffmpeg for "all of them" is
        // how a parallel decode ends up slower than a serial one.
        int defaultDecodeThreads(int jobs) {
            const unsigned THREADS = std::thread::hardware_concurrency();
            if (jobs <= 1 || THREADS == 0)
                return 0; // 0 = leave ffmpeg's own default, which is right when we are alone
            return std::max(1, static_cast<int>(THREADS) / jobs);
        }

        // Rebuilds this run's command line for a worker, differing only in which
        // chunk it renders and where it writes. Everything that could otherwise be
        // re-derived differently - the pane size, the output rate, the frame limit
        // - is passed explicitly, so a worker's plan cannot drift from the parent's
        // even if a default changes underneath it.
        std::vector<std::string> workerArgv(const std::string& binary, const SRenderOptions& options, const SRenderPlan& PLAN, int index, int count, const std::string& segmentPath,
                                            const std::string& reportPath) {
            const auto EYE_NAME = [&] {
                switch (options.eye) {
                    case eEyeSelection::LEFT: return "left";
                    case eEyeSelection::RIGHT: return "right";
                    default: return "stereo-sbs";
                }
            }();
            const auto BACKGROUND_NAME = [&] {
                switch (PLAN.background) {
                    case eBackgroundChoice::CAMERA: return "camera";
                    case eBackgroundChoice::SOLID: return "solid";
                    default: return "checker";
                }
            }();

            std::vector<std::string> argv{
                binary,
                "render",
                options.take.string(),
                "--out",
                segmentPath,
                "--segment",
                std::format("{}/{}", index, count),
                "--report",
                reportPath,
                "--eye",
                EYE_NAME,
                "--framing",
                options.framing == eFraming::STABILIZED ? "stabilized" : "asis",
                "--frustum",
                options.frustum == eFrustumMode::RECORDED ? "recorded" : "presentation",
                "--background",
                BACKGROUND_NAME,
                "--size",
                std::format("{}x{}", PLAN.paneWidth, PLAN.paneHeight),
                // {:.17g} round-trips a double exactly, so every worker's frame
                // count arithmetic is bit-for-bit the parent's.
                "--fps",
                std::format("{:.17g}", PLAN.fps),
                "--bg-depth",
                std::format("{:.17g}", options.backgroundDepth),
                "--fg-depth",
                std::isinf(options.overlayDepth) ? std::string("inf") : std::format("{:.17g}", options.overlayDepth),
                "--stabilize-ms",
                std::format("{:.17g}", static_cast<double>(options.stabilizeSigmaNs) * 1e-6),
                "--codec",
                options.videoCodec,
                "--crf",
                std::to_string(options.crf),
                "--preset",
                options.preset,
                "--decode-threads",
                std::to_string(options.decodeThreads),
                "--no-audio",
            };
            if (PLAN.frameCount > 0 && options.limitFrames > 0) {
                argv.push_back("--limit");
                argv.push_back(std::to_string(options.limitFrames));
            }
            if (!options.gpuHint.empty()) {
                argv.push_back("--gpu");
                argv.push_back(options.gpuHint);
            }
            if (!options.framesDir.empty()) {
                argv.push_back("--frames-dir");
                argv.push_back(options.framesDir);
            }
            if (!options.probeCachePath.empty()) {
                argv.push_back("--probe-cache");
                argv.push_back(options.probeCachePath);
            }
            argv.push_back(logEnabled(eLogLevel::DEBUG) ? "-v" : "-q");
            return argv;
        }

        int runSegmented(const SRenderOptions& options, const SBundle& BUNDLE, const SRenderPlan& PLAN, const std::shared_ptr<CProbeCache>& probeCache, SRenderReport& report) {
            const int JOBS = options.jobs;

            std::string binary = options.workerBinary.empty() ? executablePath() : options.workerBinary;
            if (binary.empty()) {
                HXC_ERR("cannot find this executable to spawn segment workers; pass --worker-binary or --jobs 1");
                return 1;
            }

            // Segments live beside the output, so the join is a rename-distance
            // stream copy rather than a copy across filesystems, and so a run that
            // dies leaves its debris somewhere the user will find it.
            const fs::path OUT       = fs::absolute(options.outPath);
            const fs::path EXTENSION = OUT.extension();
            const fs::path WORK      = OUT.parent_path() / std::format(".{}.segments.{}", OUT.filename().string(), ::getpid());
            std::error_code ec;
            fs::create_directories(WORK, ec);
            if (ec) {
                HXC_ERR("cannot create the segment directory {}: {}", WORK.string(), ec.message());
                return 1;
            }
            // Whatever happens below, the temporary files go away.
            struct SCleanup {
                fs::path path;
                ~SCleanup() {
                    std::error_code ec;
                    fs::remove_all(path, ec);
                }
            } cleanup{WORK};

            // What the parent already learned about every video in the take. The
            // workers read this instead of demuxing gigabytes to recount frames
            // their parent counted seconds ago - on a two-eye take that is 4.8 GB
            // of pointless reading per worker, and it lands all at once.
            SRenderOptions spawnOptions = options;
            if (probeCache && probeCache->size() > 0) {
                const std::string CACHE = (WORK / "probe.json").string();
                std::string       error;
                if (probeCache->write(CACHE, error))
                    spawnOptions.probeCachePath = CACHE;
                else
                    HXC_WARN("{}; the workers will re-probe the take themselves", error);
            }

            std::vector<std::string>                  segmentPaths, reportPaths;
            std::vector<std::unique_ptr<CSubprocess>> workers;
            std::vector<std::pair<size_t, size_t>>    ranges;

            HXC_INFO("segmenting {} output frames across {} worker(s), {} decode thread(s) each", PLAN.frameCount, JOBS, options.decodeThreads);

            for (int i = 0; i < JOBS; ++i) {
                const auto RANGE = segmentRange(PLAN.frameCount, i, JOBS);
                ranges.push_back(RANGE);
                if (RANGE.first >= RANGE.second) {
                    // More workers than frames. Nothing to do for this one.
                    segmentPaths.emplace_back();
                    reportPaths.emplace_back();
                    workers.emplace_back();
                    continue;
                }

                segmentPaths.push_back((WORK / std::format("seg{:03}{}", i, EXTENSION.string())).string());
                reportPaths.push_back((WORK / std::format("seg{:03}.json", i)).string());

                std::string error;
                auto        worker = CSubprocess::spawn(workerArgv(binary, spawnOptions, PLAN, i, JOBS, segmentPaths.back(), reportPaths.back()), {}, error);
                if (!worker) {
                    HXC_ERR("cannot start segment worker {}: {}", i, error);
                    for (auto& RUNNING : workers) {
                        if (RUNNING)
                            RUNNING->terminate();
                    }
                    return 1;
                }
                workers.push_back(std::move(worker));
            }

            bool   failed = false;
            size_t done   = 0;
            for (int i = 0; i < JOBS; ++i) {
                if (!workers[static_cast<size_t>(i)])
                    continue;
                const int STATUS = workers[static_cast<size_t>(i)]->wait();
                if (STATUS != 0) {
                    HXC_ERR("segment worker {} (frames {}..{}) exited with status {}", i, ranges[static_cast<size_t>(i)].first, ranges[static_cast<size_t>(i)].second, STATUS);
                    failed = true;
                    // The survivors are now composing frames nobody will read.
                    for (int j = i + 1; j < JOBS; ++j) {
                        if (workers[static_cast<size_t>(j)])
                            workers[static_cast<size_t>(j)]->terminate();
                    }
                    break;
                }
                done += ranges[static_cast<size_t>(i)].second - ranges[static_cast<size_t>(i)].first;
                HXC_INFO("segment {}/{} done ({} of {} frames composed)", i + 1, JOBS, done, PLAN.frameCount);
            }
            if (failed)
                return 1;

            // Fold the workers' reports into one describing the whole take.
            for (size_t i = 0; i < reportPaths.size(); ++i) {
                if (reportPaths[i].empty())
                    continue;
                std::ifstream     stream(reportPaths[i]);
                std::stringstream text;
                text << stream.rdbuf();
                SRenderReport segment;
                std::string   error;
                if (!SRenderReport::fromJson(text.str(), segment, error)) {
                    HXC_WARN("segment {}: {}", i, error);
                    continue;
                }
                if (report.gpu.empty())
                    report.gpu = segment.gpu;
                report.frames.insert(report.frames.end(), segment.frames.begin(), segment.frames.end());
                // Worker times are concurrent, so these sum to more than the wall
                // clock. That is what they are for: they say where the machine's
                // time went, not how long the user waited.
                report.decodeSeconds += segment.decodeSeconds;
                report.gpuSeconds    += segment.gpuSeconds;
                report.encodeSeconds += segment.encodeSeconds;
            }

            // ---- the join ---------------------------------------------------------
            SWriterSpec spec;
            spec.outPath        = options.outPath;
            spec.width          = PLAN.paneWidth * static_cast<int>(PLAN.paneEyes.size());
            spec.height         = PLAN.paneHeight;
            spec.fps            = PLAN.fps;
            spec.videoCodec     = options.videoCodec;
            spec.crf            = options.crf;
            spec.preset         = options.preset;
            spec.limiterCeiling = options.noLimiter ? 0.0 : 0.98;
            spec.stereoSideBySide = options.eye == eEyeSelection::STEREO_SBS;

            if (!options.noAudio) {
                const auto place = [&](const SAudioTrack& track, double gain) {
                    SAudioMixInput input;
                    input.path        = track.path;
                    input.gain        = gain;
                    input.label       = track.role;
                    input.startSample = static_cast<int64_t>(std::llround(static_cast<double>(track.startHostNs - PLAN.t0) * 1e-9 * spec.audioSampleRate));
                    HXC_INFO("audio {}: starts {:+.3f} ms into the take -> sample {:+}", track.role, static_cast<double>(track.startHostNs - PLAN.t0) * 1e-6, input.startSample);
                    spec.audio.push_back(input);
                    report.audio.push_back({track.role, input.startSample, gain});
                };
                if (BUNDLE.appAudio)
                    place(*BUNDLE.appAudio, 1.0);
                if (BUNDLE.mic)
                    place(*BUNDLE.mic, options.micGain);
            }

            std::vector<std::string> present;
            for (const auto& PATH : segmentPaths) {
                if (!PATH.empty())
                    present.push_back(PATH);
            }

            std::string error;
            if (!concatSegments(present, spec, error)) {
                HXC_ERR("{}", error);
                return 1;
            }
            return 0;
        }

    }

    int runRender(const SRenderOptions& options, SRenderReport* reportOut) {
        SRenderReport report;
        report.framing = framingName(options.framing);

        const auto STARTED = SClock::now();

        CDiagnostics diags;
        SLoadOptions load;
        // A worker reads what its parent already learned. A parent collects, so
        // it has something to hand over.
        load.probeCache     = options.probeCachePath.empty() ? std::make_shared<CProbeCache>() : CProbeCache::read(options.probeCachePath);
        const auto   LOADED = SBundle::load(options.take, diags, load);
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

        SRenderPlan plan;
        if (!makePlan(options, BUNDLE, plan))
            return 1;

        SRenderOptions effective = options;
        int            status    = 0;

        if (options.segmentCount > 0) {
            // A worker. One chunk, no audio, no further segmenting.
            const auto RANGE = segmentRange(plan.frameCount, options.segmentIndex, options.segmentCount);
            HXC_INFO("segment {}/{}: output frames [{}, {})", options.segmentIndex, options.segmentCount, RANGE.first, RANGE.second);
            report.jobs = 1;
            status      = composeRange(effective, BUNDLE, plan, RANGE.first, RANGE.second, options.outPath, false, report);
        } else {
            if (effective.jobs <= 0)
                effective.jobs = defaultJobs();
            // Nothing to gain from splitting fewer frames than workers, and a
            // segment shorter than a second or so is all fixed cost.
            effective.jobs = std::max(1, std::min<int>(effective.jobs, static_cast<int>(plan.frameCount)));
            if (effective.decodeThreads <= 0)
                effective.decodeThreads = defaultDecodeThreads(effective.jobs);
            report.jobs = effective.jobs;

            if (effective.jobs <= 1)
                status = composeRange(effective, BUNDLE, plan, 0, plan.frameCount, options.outPath, true, report);
            else
                status = runSegmented(effective, BUNDLE, plan, load.probeCache, report);
        }

        if (status != 0)
            return status;

        // A segmented run has no single loop to have timed itself, and its
        // workers' wall clocks overlap anyway, so the parent fills the geometry
        // in from the plan and reports the time the user actually waited through
        // - bundle load and join included.
        if (report.jobs > 1) {
            report.paneWidth   = plan.paneWidth;
            report.paneHeight  = plan.paneHeight;
            report.paneCount   = static_cast<int>(plan.paneEyes.size());
            report.fps         = plan.fps;
            report.firstHostNs = plan.t0;
            report.paneFov     = plan.paneFov;
            report.wallSeconds = secondsSince(STARTED);
        }

        if (report.wallSeconds > 0.0) {
            const double MEGAPIXELS    = static_cast<double>(report.frames.size()) * static_cast<double>(report.paneWidth * report.paneCount) * static_cast<double>(report.paneHeight) * 1e-6;
            report.framesPerSecond     = static_cast<double>(report.frames.size()) / report.wallSeconds;
            report.megapixelsPerSecond = MEGAPIXELS / report.wallSeconds;
        }

        if (report.jobs > 1) {
            HXC_INFO("composed {} frames of {}x{} in {:.2f} s across {} worker(s) = {:.1f} fps, {:.1f} Mpix/s (machine time: decode {:.2f} s, gpu {:.2f} s, encode {:.2f} s)", report.frames.size(),
                     report.paneWidth * report.paneCount, report.paneHeight, report.wallSeconds, report.jobs, report.framesPerSecond, report.megapixelsPerSecond, report.decodeSeconds,
                     report.gpuSeconds, report.encodeSeconds);
            HXC_INFO("wrote {}", options.outPath);
        }

        // A worker writes one too: it is how the parent learns what its segment
        // composed, and it is the same document, just narrower.
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
