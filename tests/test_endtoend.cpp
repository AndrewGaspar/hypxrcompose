// synth -> render -> verify.
//
// Every assertion here is quantitative, because the synthetic bundle's geometry
// is closed-form: the correct output pixel for a known feature is computable, so
// the test compares a measurement against a prediction rather than against a
// stored image. What each one proves is stated at the top of the test.

#include "ComposeGL.hpp"
#include "Ffmpeg.hpp"
#include "Harness.hpp"
#include "Log.hpp"
#include "Process.hpp"
#include "Render.hpp"
#include "Stabilize.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <format>
#include <limits>
#include <set>
#include <tuple>

using namespace hxc;
using namespace hxctest;
namespace fs = std::filesystem;

namespace {

    constexpr int PANE_WIDTH  = 480;
    constexpr int PANE_HEIGHT = 360;

    struct SRendered {
        SRenderReport report;
        fs::path      framesDir;
        fs::path      video;

        SImage        frame(size_t index) const {
            SImage      image;
            std::string error;
            if (!loadImage(framesDir / std::format("frame_{:06}.png", index), image, error))
                throw std::runtime_error(error);
            return image;
        }
    };

    // Renders the fixture with the given options and returns the report plus the
    // directory of exact (PNG) output frames. Codec loss is deliberately kept out
    // of the measurement path; a separate test covers the muxed video.
    SRendered renderCase(const std::string& name, const std::function<void(SRenderOptions&)>& configure, const SFixture& fix = fixture(), bool dumpFrames = true) {
        SRendered rendered;
        rendered.framesDir = scratchRoot() / ("render-" + name);
        rendered.video     = scratchRoot() / ("render-" + name + ".mp4");

        std::error_code ec;
        fs::remove_all(rendered.framesDir, ec);

        SRenderOptions options;
        options.take      = fix.take;
        options.outPath   = rendered.video.string();
        options.width     = PANE_WIDTH;
        options.height    = PANE_HEIGHT;
        options.noAudio   = true;
        options.framesDir = dumpFrames ? rendered.framesDir.string() : "";
        configure(options);

        setLogLevel(eLogLevel::WARN);
        const int STATUS = runRender(options, &rendered.report);
        setLogLevel(eLogLevel::INFO);
        if (STATUS != 0)
            throw std::runtime_error(std::format("render `{}` failed with status {}", name, STATUS));
        return rendered;
    }

    SVec3 overlayMarkerWorld(const SSynthScene& scene, const std::string& name) {
        for (const auto& MARKER : scene.overlayMarkers) {
            if (MARKER.name == name)
                return MARKER.world(scene.overlayQuad);
        }
        throw std::runtime_error("no such overlay marker: " + name);
    }

    // By value: the caller binds the result to a `const auto&`, and returning a
    // reference into the scene there is a dangling-reference warning waiting to
    // happen for no gain at this size.
    SSynthWallMarker wallMarker(const SSynthScene& scene, const std::string& name) {
        for (const auto& MARKER : scene.wallMarkers) {
            if (MARKER.name == name)
                return MARKER;
        }
        throw std::runtime_error("no such wall marker: " + name);
    }

    // Which camera frame should output frame `k` be made of, given the recorded
    // clock series? Computed here from the ground truth, independently of what the
    // compositor decided.
    size_t predictCameraFrame(const SFixture& fix, int eye, size_t k, double fps) {
        const int64_t        T0     = fix.bundle.firstHostNs();
        const int64_t        T_HOST = T0 + static_cast<int64_t>(std::llround(static_cast<double>(k) * 1e9 / fps));
        const auto&          CAMERA = fix.camera(eye);
        std::vector<int64_t> hosts;
        hosts.reserve(CAMERA.deviceNs.size());
        for (size_t j = 0; j < CAMERA.deviceNs.size(); ++j)
            hosts.push_back(fix.cameraHostNs(eye, j));
        return nearestIndex(hosts, T_HOST).value();
    }

    // What a compositor that read the device stamps as if they were host stamps
    // would have picked instead.
    size_t predictClockBlindCameraFrame(const SFixture& fix, int eye, size_t k, double fps) {
        const int64_t        T0     = fix.bundle.firstHostNs();
        const int64_t        T_HOST = T0 + static_cast<int64_t>(std::llround(static_cast<double>(k) * 1e9 / fps));
        const auto&          CAMERA = fix.camera(eye);
        std::vector<int64_t> stamps;
        for (int64_t device : CAMERA.deviceNs)
            stamps.push_back(device + CAMERA.exposureNs / 2);
        return nearestIndex(stamps, T_HOST).value();
    }

    // Mean colour of a small block, so codec noise on a flat patch averages out.
    std::array<double, 3> blockColor(const SImage& image, double centreX, double centreY, int radius) {
        std::array<double, 3> sum{0.0, 0.0, 0.0};
        int                   count = 0;
        for (int y = static_cast<int>(std::lround(centreY)) - radius; y <= static_cast<int>(std::lround(centreY)) + radius; ++y) {
            for (int x = static_cast<int>(std::lround(centreX)) - radius; x <= static_cast<int>(std::lround(centreX)) + radius; ++x) {
                if (x < 0 || y < 0 || x >= image.width || y >= image.height)
                    continue;
                const auto PIXEL = image.at(x, y);
                sum[0] += PIXEL[0];
                sum[1] += PIXEL[1];
                sum[2] += PIXEL[2];
                ++count;
            }
        }
        if (count == 0)
            return sum;
        for (auto& CHANNEL : sum)
            CHANNEL /= count;
        return sum;
    }

}

// ---------------------------------------------------------------------------
// 1. The overlay path: a stamped pose, a stamped frustum, and a resample.
//
// Proves: telemetry poses survive the write/parse round trip, the asymmetric
// frustum maths in the GLSL kernel agrees with the CPU model, the overlay frame
// selected for an output instant is the right one, and the resample to a pane
// size that is not the capture size lands the content in the right place.
// ---------------------------------------------------------------------------
TEST(EndToEnd, OverlayMarkersLandWherePredicted) {
    const auto& FIX      = fixture();
    const auto  RENDERED = renderCase("overlay-asis", [](SRenderOptions& o) { o.eye = eEyeSelection::LEFT; });
    const double FPS     = RENDERED.report.fps;

    double worst          = 0.0;
    size_t reprojections  = 0;
    // 15 and 30 are the output frames whose own telemetry record was lost to
    // the synthetic readback queue, so their overlay pixels must come from a
    // neighbouring record and be reprojected across the gap.
    for (size_t k : {size_t{5}, size_t{15}, size_t{30}, size_t{40}}) {
        const SImage IMAGE = RENDERED.frame(k);
        ASSERT_EQ(IMAGE.width, PANE_WIDTH);
        ASSERT_EQ(IMAGE.height, PANE_HEIGHT);

        // The output camera is the eye at the output instant's record; the overlay
        // pixels came from whichever undropped record the ordinal rule names, which
        // is not always the same one.
        const size_t OUTPUT_RECORD  = FIX.outputRecord(k, FPS);
        const size_t SOURCE_RECORD  = FIX.overlaySourceRecord(k, FPS);
        if (SOURCE_RECORD != OUTPUT_RECORD)
            ++reprojections;

        EXPECT_EQ(RENDERED.report.frames[k].overlayTelemetryIndex[0], static_cast<int64_t>(SOURCE_RECORD)) << "output frame " << k;

        // The camera the renderer used, and the frustum it published - not the
        // recorded eye pose and fov, which since the presentation framing landed
        // are the *source's* geometry rather than the output's.
        const SPose OUTPUT_EYE = FIX.outputCamera(static_cast<int>(OUTPUT_RECORD), 0);
        const SPose SOURCE_EYE = FIX.eyePose(static_cast<int>(SOURCE_RECORD), 0);
        const SFov& FOV        = RENDERED.report.paneFov.at(0);

        for (const auto& MARKER : FIX.scene.overlayMarkers) {
            // Partially transparent markers sit over the panel's own gradient, so
            // their composited colour is a blend and colour-matching cannot find
            // them; the linear-light test below measures those instead.
            if (MARKER.alpha < 1.0)
                continue;
            const SVec3 WORLD     = MARKER.world(FIX.scene.overlayQuad);
            const auto  PREDICTED = predictOverlayPixel(OUTPUT_EYE, FOV, PANE_WIDTH, PANE_HEIGHT, SOURCE_EYE, WORLD);
            ASSERT_TRUE(PREDICTED.has_value()) << MARKER.name;

            const auto MEASURED = findColor(IMAGE, MARKER.color, 30, 0, PANE_WIDTH);
            ASSERT_TRUE(MEASURED.has_value()) << "marker " << MARKER.name << " not found in frame " << k;
            EXPECT_GT(MEASURED->count, 20u) << MARKER.name;

            const double DX = MEASURED->x - (*PREDICTED)[0];
            const double DY = MEASURED->y - (*PREDICTED)[1];
            worst           = std::max(worst, std::hypot(DX, DY));
            EXPECT_NEAR(MEASURED->x, (*PREDICTED)[0], 1.5) << MARKER.name << " x, frame " << k;
            EXPECT_NEAR(MEASURED->y, (*PREDICTED)[1], 1.5) << MARKER.name << " y, frame " << k;
        }
    }
    std::cout << "[measured] worst overlay marker error: " << worst << " px (" << reprojections << " of the sampled frames reproject a dropped-neighbour overlay frame)\n";
    EXPECT_GT(reprojections, 0u) << "the fixture must exercise a frame whose overlay pixels come from a different record";
}

// ---------------------------------------------------------------------------
// 2. The background path: intrinsics, distortion, extrinsics, mid-exposure pose.
//
// Proves the whole camera chain closes. The prediction places the marker by
// intersecting the camera's ray to it with the assumed-depth sphere around the
// output camera - the v1 model exactly - and never touches the distortion model,
// so an inconsistency between the generator's inverse distortion and the shader's
// forward distortion shows up as a miss.
// ---------------------------------------------------------------------------
TEST(EndToEnd, BackgroundMarkersLandWherePredicted) {
    const auto& FIX      = fixture();
    const auto  RENDERED = renderCase("background-asis", [](SRenderOptions& o) {
        o.eye             = eEyeSelection::LEFT;
        o.background      = eBackgroundChoice::CAMERA;
        // The synth's extrinsic is ground truth, so `recorded` is the correct
        // model here; `auto` deliberately discards the swing and is measured
        // separately in BgAlignAutoTradesRegistrationForCoverage.
        o.backgroundAlign = eBackgroundAlign::RECORDED;
        o.backgroundDepth = 2.0;
    });

    const double FPS   = RENDERED.report.fps;
    double       worst = 0.0;

    for (size_t k : {size_t{10}, size_t{25}, size_t{40}}) {
        const SImage IMAGE = RENDERED.frame(k);
        const SPose  EYE   = FIX.outputCamera(static_cast<int>(FIX.outputRecord(k, FPS)), 0);
        const SFov&  FOV   = RENDERED.report.paneFov.at(0);

        const size_t CAMERA_FRAME = predictCameraFrame(FIX, 0, k, FPS);
        ASSERT_EQ(RENDERED.report.frames[k].cameraFrame[0], static_cast<int64_t>(CAMERA_FRAME)) << "output frame " << k;

        const SPose CAMERA_POSE = FIX.scene.headAt(FIX.cameraHostNs(0, CAMERA_FRAME)).compose(FIX.cameraExtrinsic(0, eBackgroundAlign::RECORDED));

        for (const char* NAME : {"green", "red", "blue"}) {
            const auto& MARKER    = wallMarker(FIX.scene, NAME);
            const auto  PREDICTED = predictBackgroundPixel(EYE, FOV, PANE_WIDTH, PANE_HEIGHT, CAMERA_POSE, MARKER.world, 2.0);
            ASSERT_TRUE(PREDICTED.has_value()) << NAME;
            if ((*PREDICTED)[0] < 20 || (*PREDICTED)[0] > PANE_WIDTH - 20)
                continue; // outside the pane at this head pose

            const auto MEASURED = findColor(IMAGE, MARKER.color, 60, 0, PANE_WIDTH);
            ASSERT_TRUE(MEASURED.has_value()) << "wall marker " << NAME << " not found in frame " << k;

            worst = std::max(worst, std::hypot(MEASURED->x - (*PREDICTED)[0], MEASURED->y - (*PREDICTED)[1]));
            EXPECT_NEAR(MEASURED->x, (*PREDICTED)[0], 2.0) << NAME << " x, frame " << k;
            EXPECT_NEAR(MEASURED->y, (*PREDICTED)[1], 2.0) << NAME << " y, frame " << k;
        }
    }
    std::cout << "[measured] worst background marker error: " << worst << " px\n";
}

// ---------------------------------------------------------------------------
// 3. The accepted v1 parallax error, measured rather than asserted away.
//
// research 27 section 5.1 says v1 accepts the error from using a camera that is
// not at the eye. This quantifies it for the synthetic rig: the gap between where
// the assumed-depth model puts a feature and where the feature really is.
// ---------------------------------------------------------------------------
TEST(EndToEnd, TheAcceptedParallaxErrorIsSmallAndMeasurable) {
    const auto&  FIX = fixture();
    const double FPS = FIX.scene.overlayHz;
    const SPose  EYE = FIX.eyePose(static_cast<int>(FIX.outputRecord(20, FPS)), 0);
    const SFov&  FOV = FIX.scene.eyeFov[0];

    const size_t CAMERA_FRAME = predictCameraFrame(FIX, 0, 20, FPS);
    const SPose  CAMERA_POSE  = FIX.scene.headAt(FIX.cameraHostNs(0, CAMERA_FRAME)).compose(FIX.cameraExtrinsic(0, eBackgroundAlign::RECORDED));

    double worst = 0.0;
    for (const auto& MARKER : FIX.scene.wallMarkers) {
        const auto MODELLED = predictBackgroundPixel(EYE, FOV, PANE_WIDTH, PANE_HEIGHT, CAMERA_POSE, MARKER.world, 2.0);
        // Where the marker truly is, from the output camera.
        double     idealX = 0.0, idealY = 0.0;
        ASSERT_TRUE(fovProject(FOV, EYE.dirToLocal(MARKER.world - EYE.pos), PANE_WIDTH, PANE_HEIGHT, idealX, idealY));
        ASSERT_TRUE(MODELLED.has_value());
        worst = std::max(worst, std::hypot((*MODELLED)[0] - idealX, (*MODELLED)[1] - idealY));
    }

    std::cout << "[measured] v1 assumed-depth parallax error on the wall: " << worst << " px of " << PANE_WIDTH << "\n";
    // It is real - the rig has a 48 mm eye-to-lens offset - but it is sub-pixel at
    // this pane size, which is what "v1 accepts it" is worth quantitatively.
    EXPECT_GT(worst, 0.0);
    EXPECT_LT(worst, 3.0);
}

// ---------------------------------------------------------------------------
// 4. The clock path.
//
// Proves the compositor maps device time into host time before choosing camera
// frames: it must pick the frame the clock series says, and that frame must be
// exactly `offset x camera rate` away from the one a clock-blind compositor would
// have taken. The frame-identity patch confirms it in the pixels, not just in the
// tool's own report.
// ---------------------------------------------------------------------------
TEST(EndToEnd, TheClockOffsetSelectsTheRightCameraFrame) {
    const auto& FIX      = fixture();
    const auto  RENDERED = renderCase("clock", [](SRenderOptions& o) {
        o.eye             = eEyeSelection::LEFT;
        o.background      = eBackgroundChoice::CAMERA;
        // The synth's extrinsic is ground truth, so `recorded` is the correct
        // model here; `auto` deliberately discards the swing and is measured
        // separately in BgAlignAutoTradesRegistrationForCoverage.
        o.backgroundAlign = eBackgroundAlign::RECORDED;
        o.backgroundDepth = 2.0;
    });

    const double FPS            = RENDERED.report.fps;
    const double CAMERA_HZ      = FIX.camera(0).hz;
    const int    EXPECTED_SHIFT = static_cast<int>(std::llround(FIX.options.clockOffsetMs * 1e-3 * CAMERA_HZ));
    ASSERT_EQ(EXPECTED_SHIFT, 6) << "the fixture is built so this is a whole number of camera frames";

    for (size_t k : {size_t{20}, size_t{30}, size_t{42}}) {
        const size_t CHOSEN = predictCameraFrame(FIX, 0, k, FPS);
        const size_t BLIND  = predictClockBlindCameraFrame(FIX, 0, k, FPS);

        EXPECT_EQ(RENDERED.report.frames[k].cameraFrame[0], static_cast<int64_t>(CHOSEN)) << "output frame " << k;
        EXPECT_EQ(static_cast<int>(CHOSEN) - static_cast<int>(BLIND), EXPECTED_SHIFT)
            << "a compositor that read device stamps as host stamps would have used camera frame " << BLIND << " instead of " << CHOSEN;

        // And the pixels agree: the frame-identity patch encodes which camera frame
        // was sampled, so this reads the answer out of the composite itself.
        const SImage IMAGE       = RENDERED.frame(k);
        const SPose  EYE         = FIX.outputCamera(static_cast<int>(FIX.outputRecord(k, FPS)), 0);
        const SPose  CAMERA_POSE = FIX.scene.headAt(FIX.cameraHostNs(0, CHOSEN)).compose(FIX.cameraExtrinsic(0, eBackgroundAlign::RECORDED));
        const auto   PATCH       = predictBackgroundPixel(EYE, RENDERED.report.paneFov.at(0), PANE_WIDTH, PANE_HEIGHT, CAMERA_POSE, FIX.scene.codeCentre, 2.0);
        ASSERT_TRUE(PATCH.has_value());

        const auto   COLOR   = blockColor(IMAGE, (*PATCH)[0], (*PATCH)[1], 3);
        const int    DECODED = static_cast<int>(std::lround((COLOR[0] - FIX.scene.codeBase) / FIX.scene.codeStep));
        EXPECT_NEAR(COLOR[1], FIX.scene.codeGreen, 6.0) << "the sampled block is not the code patch at output frame " << k;
        EXPECT_EQ(DECODED, static_cast<int>(CHOSEN % static_cast<size_t>(FIX.scene.codeModulus))) << "the composite shows camera frame " << DECODED << ", not " << CHOSEN;
    }
}

// ---------------------------------------------------------------------------
// 5. Stereo.
//
// Proves the two panes are genuinely two eyes: each lands where its own pose and
// its own (mirrored, asymmetric) frustum say, and the difference between them
// carries the synthetic IPD's parallax with the right sign.
// ---------------------------------------------------------------------------
// Under the default presentation frustum the two panes share one symmetric
// camera frustum and a common orientation, so ALL of the disparity is content
// parallax. That is the property a flat side-by-side viewer needs: frames that
// subtend the same angles, with the stereo carried by the picture inside them.
TEST(EndToEnd, StereoPanesCarryTheIpdParallaxAndNothingElse) {
    const auto& FIX      = fixture();
    const auto  RENDERED = renderCase("stereo", [](SRenderOptions& o) { o.eye = eEyeSelection::STEREO_SBS; });

    ASSERT_EQ(RENDERED.report.paneCount, 2);
    ASSERT_EQ(RENDERED.report.paneFov.size(), 2u);

    // One frustum, symmetric about forward, for both eyes.
    const SFov& L = RENDERED.report.paneFov[0];
    const SFov& R = RENDERED.report.paneFov[1];
    EXPECT_NEAR(L.l, R.l, 1e-12) << "the panes must share one frustum, or their frames sit at different visual angles";
    EXPECT_NEAR(L.r, R.r, 1e-12);
    EXPECT_NEAR(L.u, R.u, 1e-12);
    EXPECT_NEAR(L.d, R.d, 1e-12);
    EXPECT_NEAR(L.l, -L.r, 1e-12) << "and it must be symmetric, so the optical axis lands at the pane centre";
    EXPECT_NEAR(L.u, -L.d, 1e-12);
    EXPECT_NEAR(L.opticalCentreU(), 0.5, 1e-9);
    EXPECT_NEAR(L.opticalCentreV(), 0.5, 1e-9);

    const size_t K     = 25;
    const SImage IMAGE = RENDERED.frame(K);
    ASSERT_EQ(IMAGE.width, PANE_WIDTH * 2);

    const double FPS           = RENDERED.report.fps;
    const size_t OUTPUT_RECORD = FIX.outputRecord(K, FPS);
    const size_t SOURCE_RECORD = FIX.overlaySourceRecord(K, FPS);

    const SVec3 MARKER   = overlayMarkerWorld(FIX.scene, "centre");
    const SPose EYE_L    = FIX.outputCamera(static_cast<int>(OUTPUT_RECORD), 0);
    const SPose EYE_R    = FIX.outputCamera(static_cast<int>(OUTPUT_RECORD), 1);
    const SPose SOURCE_L = FIX.eyePose(static_cast<int>(SOURCE_RECORD), 0);
    const SPose SOURCE_R = FIX.eyePose(static_cast<int>(SOURCE_RECORD), 1);

    const auto PREDICTED_L = predictOverlayPixel(EYE_L, L, PANE_WIDTH, PANE_HEIGHT, SOURCE_L, MARKER);
    const auto PREDICTED_R = predictOverlayPixel(EYE_R, R, PANE_WIDTH, PANE_HEIGHT, SOURCE_R, MARKER);
    ASSERT_TRUE(PREDICTED_L.has_value());
    ASSERT_TRUE(PREDICTED_R.has_value());

    const auto MEASURED_L = findColor(IMAGE, {255, 0, 255}, 30, 0, PANE_WIDTH);
    const auto MEASURED_R = findColor(IMAGE, {255, 0, 255}, 30, PANE_WIDTH, PANE_WIDTH * 2);
    ASSERT_TRUE(MEASURED_L.has_value());
    ASSERT_TRUE(MEASURED_R.has_value());

    EXPECT_NEAR(MEASURED_L->x, (*PREDICTED_L)[0], 1.5);
    EXPECT_NEAR(MEASURED_L->y, (*PREDICTED_L)[1], 1.5);
    EXPECT_NEAR(MEASURED_R->x - PANE_WIDTH, (*PREDICTED_R)[0], 1.5);
    EXPECT_NEAR(MEASURED_R->y, (*PREDICTED_R)[1], 1.5);

    // A parallel rig with a shared frustum: vertical disparity must be zero, and
    // a vertical offset is the thing that makes a stereo pair unfusable.
    EXPECT_NEAR(MEASURED_L->y, MEASURED_R->y, 1.5) << "the two panes disagree vertically; nothing can fuse that";

    // The horizontal disparity is the IPD parallax at the marker's depth, and
    // nothing else: baseline over distance, in tangent, times the pane scale.
    const double DEPTH     = (MARKER - FIX.headPose(static_cast<int>(OUTPUT_RECORD)).pos).length();
    const double BASELINE  = (EYE_L.pos - EYE_R.pos).length();
    const double SCALE     = PANE_WIDTH / L.tanWidth();
    const double PREDICTED = BASELINE / DEPTH * SCALE;

    const double MEASURED_DISPARITY = MEASURED_L->x - (MEASURED_R->x - PANE_WIDTH);
    std::cout << "[measured] presentation stereo: disparity " << MEASURED_DISPARITY << " px, IPD parallax at " << DEPTH << " m predicts " << PREDICTED << " px; frustum term 0 by construction\n";

    EXPECT_GT(MEASURED_DISPARITY, 0.0) << "a point in front of the viewer sits further right in the left eye";
    EXPECT_NEAR(MEASURED_DISPARITY, PREDICTED, std::max(2.0, PREDICTED * 0.1));
}

// A feature at infinity has no parallax, so under the shared frustum it must
// land at the SAME pane coordinate in both eyes. This is the frame-alignment
// property the in-headset viewing found missing: with per-eye recorded frusta
// the same feature sits hundreds of pixels apart, which is a constant disparity
// nobody can fuse and which reads as the two images floating apart.
TEST(EndToEnd, AFeatureAtInfinityLandsAtTheSamePaneCoordinateInBothEyes) {
    const auto& FIX = realFrustumFixture();

    // The checker background draws a world-locked forward mark - a vertical
    // stripe at yaw 0 - in direction space, which is to say at infinity.
    const auto stripeX = [](const SImage& image, int x0, int x1) -> std::optional<double> {
        // vec3(0.34, 0.26, 0.22) written as it looks, i.e. sRGB bytes.
        const auto MARK = findColor(image, {87, 66, 56}, 30, x0, x1);
        if (!MARK || MARK->count < 40)
            return std::nullopt;
        return MARK->x - x0;
    };

    for (const auto& [MODE, NAME] : std::vector<std::pair<eFrustumMode, const char*>>{{eFrustumMode::PRESENTATION, "presentation"}, {eFrustumMode::RECORDED, "recorded"}}) {
        const auto RENDERED = renderCase(
            std::string("infinity-") + NAME,
            [m = MODE](SRenderOptions& o) {
                o.eye        = eEyeSelection::STEREO_SBS;
                o.frustum    = m;
                o.background = eBackgroundChoice::CHECKER;
            },
            FIX, true);

        std::optional<double> worst;
        size_t                frames = 0;
        for (size_t k = 0; k < std::min<size_t>(8, RENDERED.report.frames.size()); ++k) {
            const SImage IMAGE = RENDERED.frame(k);
            const auto   LX    = stripeX(IMAGE, 0, PANE_WIDTH);
            const auto   RX    = stripeX(IMAGE, PANE_WIDTH, PANE_WIDTH * 2);
            if (!LX || !RX)
                continue;
            const double DIFF = std::abs(*LX - *RX);
            worst             = worst ? std::max(*worst, DIFF) : DIFF;
            ++frames;
        }
        // What the two panes' frusta say a feature at infinity must do, which is
        // exact and does not depend on the mark being framed in both panes: with
        // a common orientation, the only thing left is where each frustum starts.
        ASSERT_EQ(RENDERED.report.paneFov.size(), 2u);
        const SFov&  PL       = RENDERED.report.paneFov[0];
        const SFov&  PR       = RENDERED.report.paneFov[1];
        const double ANALYTIC = std::abs(std::tan(PR.l) - std::tan(PL.l)) * (PANE_WIDTH / PL.tanWidth());

        if (MODE == eFrustumMode::PRESENTATION) {
            ASSERT_GT(frames, 0u) << "presentation: the forward mark was never visible in both panes";
            EXPECT_LT(ANALYTIC, 1e-9) << "presentation: the shared frustum must put infinity at the same coordinate by construction";
            EXPECT_LT(*worst, 2.0) << "presentation: a feature at infinity must land at the same pane coordinate in both eyes, but it is " << *worst << " px apart";
            std::cout << "[measured] presentation: infinity lands within " << *worst << " px in both panes (" << frames << " frames)\n";
        } else {
            // The mode this exists to contrast with: the same feature, hundreds
            // of pixels apart, which is exactly what broke fusion in the headset.
            // Measured where the mark is framed in both panes, and always
            // asserted from the frusta - under `recorded` the panes are padded so
            // wide that the mark often falls outside one of them, which is itself
            // a symptom of the frames not agreeing.
            EXPECT_GT(ANALYTIC, 20.0) << "recorded: the per-eye frusta should put infinity in visibly different places; if they do not, this test has stopped contrasting anything";
            if (frames > 0) {
                EXPECT_NEAR(*worst, ANALYTIC, std::max(3.0, ANALYTIC * 0.05));
            }
            std::cout << "[measured] recorded: the panes' frusta put the same infinite feature " << ANALYTIC << " px apart" << (frames > 0 ? std::format(" (measured {:.1f})", *worst) : std::string())
                      << " - the constant disparity that broke fusion\n";
        }
    }
}

// ---------------------------------------------------------------------------
// 6. The assumed-depth knob does what it says.
// ---------------------------------------------------------------------------
TEST(EndToEnd, ChangingTheAssumedBackgroundDepthMovesTheImageByThePredictedParallax) {
    const auto& FIX = fixture();

    const auto AT_TWO = renderCase("depth-2", [](SRenderOptions& o) {
        o.eye             = eEyeSelection::LEFT;
        o.background      = eBackgroundChoice::CAMERA;
        o.backgroundAlign = eBackgroundAlign::RECORDED;
        o.backgroundDepth = 2.0;
    });
    const auto AT_TEN = renderCase("depth-10", [](SRenderOptions& o) {
        o.eye             = eEyeSelection::LEFT;
        o.background      = eBackgroundChoice::CAMERA;
        o.backgroundAlign = eBackgroundAlign::RECORDED;
        o.backgroundDepth = 10.0;
    });

    const size_t K            = 20;
    const SPose  EYE          = FIX.outputCamera(static_cast<int>(FIX.outputRecord(K, AT_TWO.report.fps)), 0);
    const size_t CAMERA_FRAME = predictCameraFrame(FIX, 0, K, AT_TWO.report.fps);
    const SPose  CAMERA_POSE  = FIX.scene.headAt(FIX.cameraHostNs(0, CAMERA_FRAME)).compose(FIX.cameraExtrinsic(0, eBackgroundAlign::RECORDED));
    const auto&  MARKER       = wallMarker(FIX.scene, "green");

    const auto PREDICTED_TWO = predictBackgroundPixel(EYE, AT_TWO.report.paneFov.at(0), PANE_WIDTH, PANE_HEIGHT, CAMERA_POSE, MARKER.world, 2.0);
    const auto PREDICTED_TEN = predictBackgroundPixel(EYE, AT_TWO.report.paneFov.at(0), PANE_WIDTH, PANE_HEIGHT, CAMERA_POSE, MARKER.world, 10.0);
    ASSERT_TRUE(PREDICTED_TWO.has_value());
    ASSERT_TRUE(PREDICTED_TEN.has_value());

    const auto MEASURED_TWO = findColor(AT_TWO.frame(K), MARKER.color, 60, 0, PANE_WIDTH);
    const auto MEASURED_TEN = findColor(AT_TEN.frame(K), MARKER.color, 60, 0, PANE_WIDTH);
    ASSERT_TRUE(MEASURED_TWO.has_value());
    ASSERT_TRUE(MEASURED_TEN.has_value());

    const double PREDICTED_SHIFT = std::hypot((*PREDICTED_TEN)[0] - (*PREDICTED_TWO)[0], (*PREDICTED_TEN)[1] - (*PREDICTED_TWO)[1]);
    const double MEASURED_SHIFT  = std::hypot(MEASURED_TEN->x - MEASURED_TWO->x, MEASURED_TEN->y - MEASURED_TWO->y);
    std::cout << "[measured] wrong-depth shift: " << MEASURED_SHIFT << " px (predicted " << PREDICTED_SHIFT << ")\n";

    EXPECT_GT(PREDICTED_SHIFT, 1.0) << "the test needs the knob to make a visible difference";
    EXPECT_NEAR(MEASURED_SHIFT, PREDICTED_SHIFT, 1.0);
    EXPECT_NEAR(MEASURED_TEN->x, (*PREDICTED_TEN)[0], 2.0);
}

// ---------------------------------------------------------------------------
// 7. Stabilized framing.
//
// The filter's own behaviour is characterized in test_stabilize.cpp; what this
// proves is that the render path uses the smoothed camera, keeps the eye's offset
// from the head, and warps the overlay by direction only when the foreground
// depth is infinite.
// ---------------------------------------------------------------------------
TEST(EndToEnd, StabilizedFramingRendersFromTheSmoothedCamera) {
    const auto& FIX = fixture();

    const int64_t SIGMA    = 200LL * 1000000LL;
    const auto    RENDERED = renderCase("stabilized", [SIGMA](SRenderOptions& o) {
        o.eye              = eEyeSelection::LEFT;
        o.framing          = eFraming::STABILIZED;
        o.stabilizeSigmaNs = SIGMA;
    });

    std::vector<STimedPose> track;
    for (const auto& RECORD : FIX.bundle.telemetry)
        track.push_back({RECORD.tHostNs, RECORD.headPose()});
    const auto SMOOTHED = gaussianSmoothPoses(track, SIGMA);

    const size_t K              = 30;
    const double FPS            = RENDERED.report.fps;
    const size_t OUTPUT_RECORD  = FIX.outputRecord(K, FPS);
    const size_t SOURCE_RECORD  = FIX.overlaySourceRecord(K, FPS);

    const SPose  HEAD   = FIX.bundle.telemetry[OUTPUT_RECORD].headPose();
    const SPose  EYE    = FIX.bundle.telemetry[OUTPUT_RECORD].eyes[0].pose;
    const SPose  SOURCE = FIX.bundle.telemetry[SOURCE_RECORD].eyes[0].pose;
    SPose        OUTPUT = SMOOTHED[OUTPUT_RECORD].compose(HEAD.inverse().compose(EYE));
    // The presentation frustum makes the panes a parallel rig, so the output
    // camera looks along the smoothed head rather than along the eye.
    OUTPUT.rot          = SMOOTHED[OUTPUT_RECORD].rot;
    const SVec3  MARKER = overlayMarkerWorld(FIX.scene, "centre");

    // Infinite foreground depth means the warp preserves directions from the
    // recording eye, so the prediction uses the recorded eye for the direction and
    // the smoothed camera for the frustum.
    const auto PREDICTED = predictOverlayPixel(OUTPUT, RENDERED.report.paneFov.at(0), PANE_WIDTH, PANE_HEIGHT, SOURCE, MARKER);
    ASSERT_TRUE(PREDICTED.has_value());

    const auto MEASURED = findColor(RENDERED.frame(K), {255, 0, 255}, 30, 0, PANE_WIDTH);
    ASSERT_TRUE(MEASURED.has_value());
    EXPECT_NEAR(MEASURED->x, (*PREDICTED)[0], 1.5);
    EXPECT_NEAR(MEASURED->y, (*PREDICTED)[1], 1.5);

    // And the framing really did change: the as-is position must be somewhere else.
    const auto ASIS = predictOverlayPixel(EYE, FIX.scene.eyeFov[0], PANE_WIDTH, PANE_HEIGHT, SOURCE, MARKER);
    ASSERT_TRUE(ASIS.has_value());
    const double MOVED = std::hypot((*PREDICTED)[0] - (*ASIS)[0], (*PREDICTED)[1] - (*ASIS)[1]);
    std::cout << "[measured] stabilization moved the marker " << MOVED << " px at frame " << K << "\n";
    EXPECT_GT(MOVED, 2.0);
}

// ---------------------------------------------------------------------------
// 8. Audio placement across both clock domains.
//
// The app track is stamped in host time, the mic track in device time. Both
// clicks must land on the output timeline at the host instants the ground truth
// records, which means the device-stamped one has been through the clock series.
// ---------------------------------------------------------------------------
TEST(EndToEnd, AudioClicksLandOnTheOutputTimeline) {
    const auto& FIX  = fixture();
    const auto  PATH = scratchRoot() / "render-audio.mkv";

    SRenderOptions options;
    options.take       = FIX.take;
    options.outPath    = PATH.string();
    options.width      = 240;
    options.height     = 180;
    options.eye        = eEyeSelection::LEFT;
    options.background = eBackgroundChoice::SOLID;
    options.noLimiter  = true; // a limiter would reshape the impulses

    SRenderReport report;
    setLogLevel(eLogLevel::WARN);
    ASSERT_EQ(runRender(options, &report), 0);
    setLogLevel(eLogLevel::INFO);
    ASSERT_EQ(report.audio.size(), 2u);

    std::vector<int16_t> pcm;
    int                  sampleRate = 0, channels = 0;
    std::string          error;
    ASSERT_TRUE(decodePcmS16(PATH.string(), pcm, sampleRate, channels, error)) << error;
    ASSERT_EQ(channels, 1);
    ASSERT_EQ(sampleRate, 48000);

    // Anything above 0.4 full scale is a click; the tones sit at 0.15 and 0.10.
    std::vector<int64_t> clicks;
    for (size_t i = 0; i < pcm.size(); ++i) {
        if (std::abs(pcm[i]) < static_cast<int>(0.4 * 32767))
            continue;
        if (clicks.empty() || static_cast<int64_t>(i) - clicks.back() > 100)
            clicks.push_back(static_cast<int64_t>(i));
    }
    ASSERT_EQ(clicks.size(), 2u) << "expected exactly the app and mic clicks";

    const int64_t T0       = FIX.bundle.firstHostNs();
    const int64_t APP_WANT = static_cast<int64_t>(std::llround(static_cast<double>(FIX.scene.app.clickHostNs - T0) * 1e-9 * sampleRate));
    const int64_t MIC_WANT = static_cast<int64_t>(std::llround(static_cast<double>(FIX.scene.mic.clickHostNs - T0) * 1e-9 * sampleRate));

    std::cout << "[measured] app click at sample " << clicks[0] << " (want " << APP_WANT << "), mic click at " << clicks[1] << " (want " << MIC_WANT << ")\n";
    EXPECT_NEAR(clicks[0], APP_WANT, 2);
    EXPECT_NEAR(clicks[1], MIC_WANT, 2);

    // The mic track is device-stamped, so its placement is only right if the clock
    // series was applied; without it the click would be a whole offset late.
    const int64_t IGNORED_CLOCK = static_cast<int64_t>(std::llround(static_cast<double>(FIX.scene.mic.clickTrackNs - T0) * 1e-9 * sampleRate));
    EXPECT_GT(std::abs(IGNORED_CLOCK - MIC_WANT), 1000) << "the fixture must make the two readings distinguishable";
}

// ---------------------------------------------------------------------------
// 9. A host-only take still composes.
// ---------------------------------------------------------------------------
TEST(EndToEnd, AHostOnlyTakeComposesOverTheCheckerBackground) {
    const fs::path TAKE = scratchRoot() / "host-only.hypxrtake";
    if (!fs::exists(TAKE / "manifest.json")) {
        SSynthOptions options;
        options.out     = TAKE;
        options.frames    = 12;
        options.hz        = 60.0;
        options.overlayHz = 60.0;
        options.cameras   = false;
        options.audio   = false;
        options.quiet   = true;
        setLogLevel(eLogLevel::WARN);
        ASSERT_EQ(runSynth(options), 0);
        setLogLevel(eLogLevel::INFO);
    }

    CDiagnostics diags;
    const auto   BUNDLE = SBundle::load(TAKE, diags, {});
    ASSERT_TRUE(BUNDLE.has_value());
    EXPECT_FALSE(diags.hasErrors());
    EXPECT_TRUE(BUNDLE->cameras.empty());

    SRenderOptions options;
    options.take      = TAKE;
    options.outPath   = (scratchRoot() / "host-only.mp4").string();
    options.width     = 240;
    options.height    = 180;
    options.noAudio   = true;
    options.framesDir = (scratchRoot() / "host-only-frames").string();

    SRenderReport report;
    setLogLevel(eLogLevel::WARN);
    ASSERT_EQ(runRender(options, &report), 0);
    setLogLevel(eLogLevel::INFO);
    EXPECT_EQ(report.frames.size(), 12u);
    for (const auto& FRAME : report.frames)
        EXPECT_EQ(FRAME.cameraFrame[0], -1) << "a host-only take must not claim a camera frame";

    // The checker background is a real image, not a flat fill.
    SImage      image;
    std::string error;
    ASSERT_TRUE(loadImage(fs::path(options.framesDir) / "frame_000006.png", image, error)) << error;
    std::set<std::array<int, 4>> colors;
    for (int y = 0; y < image.height; y += 7) {
        for (int x = 0; x < image.width; x += 7)
            colors.insert(image.at(x, y));
    }
    EXPECT_GT(colors.size(), 3u);
}

// ---------------------------------------------------------------------------
// 10. Throughput, reported rather than asserted.
// ---------------------------------------------------------------------------
TEST(EndToEnd, ThroughputIsMeasuredAndReported) {
    const auto RENDERED = renderCase(
        "throughput",
        [](SRenderOptions& o) {
            o.eye        = eEyeSelection::STEREO_SBS;
            o.width      = 960;
            o.height     = 720;
            o.background = eBackgroundChoice::CAMERA;
        },
        fixture(), false);

    const auto& REPORT = RENDERED.report;
    std::cout << "[throughput] " << REPORT.paneWidth * REPORT.paneCount << "x" << REPORT.paneHeight << " on " << REPORT.gpu << ": " << REPORT.framesPerSecond << " fps, "
              << REPORT.megapixelsPerSecond << " Mpix/s (decode " << REPORT.decodeSeconds << " s, gpu " << REPORT.gpuSeconds << " s, encode " << REPORT.encodeSeconds << " s over "
              << REPORT.wallSeconds << " s)\n";

    EXPECT_GT(REPORT.framesPerSecond, 0.0);
    EXPECT_EQ(REPORT.frames.size(), 45u);
    EXPECT_TRUE(fs::exists(RENDERED.video));
}

// ---------------------------------------------------------------------------
// 11. Alpha association and the colour space compositing happens in.
//
// The head-locked HUD layer is stored at 75% alpha over a known solid
// background, so its composited colour is computable in closed form. The
// assertion is discriminating twice over: blending the same pixel in encoded
// space instead of linear light lands ~30 levels away, and reading a
// premultiplied source as though it were straight lands ~30 levels the other
// way. Both wrong answers are checked for explicitly, so this cannot pass by
// accident.
// ---------------------------------------------------------------------------
namespace {

    // Where the head-locked HUD's centre appears, and what colour it must be.
    struct SHudExpectation {
        std::array<double, 2> pixel{};
        std::array<double, 3> color{};
    };

    SHudExpectation expectHud(const SFixture& fix, size_t k, double fps, const SFov& outputFov, const std::array<float, 4>& solidSrgb, bool encodedSpaceInstead) {
        const size_t OUTPUT_RECORD = fix.outputRecord(k, fps);
        const size_t SOURCE_RECORD = fix.overlaySourceRecord(k, fps);

        const SPose OUTPUT_EYE = fix.outputCamera(static_cast<int>(OUTPUT_RECORD), 0);
        const SPose SOURCE_EYE = fix.eyePose(static_cast<int>(SOURCE_RECORD), 0);
        // Head-locked: the layer's world pose follows the head, so its centre at the
        // instant the overlay frame was rendered is head(source) * hudQuad.
        const SVec3 CENTRE     = fix.headPose(static_cast<int>(SOURCE_RECORD)).compose(fix.scene.hudQuad).pos;

        SHudExpectation expectation;
        const auto      PIXEL = predictOverlayPixel(OUTPUT_EYE, outputFov, PANE_WIDTH, PANE_HEIGHT, SOURCE_EYE, CENTRE);
        expectation.pixel     = PIXEL.value_or(std::array<double, 2>{-1.0, -1.0});

        // Alpha survives as a byte, so the prediction uses the quantized value the
        // file actually carries rather than the ideal 0.75.
        const double ALPHA = std::round(fix.scene.hudAlpha * 255.0) / 255.0;
        for (int channel = 0; channel < 3; ++channel) {
            const double BACKGROUND = solidSrgb[static_cast<size_t>(channel)];
            const double FOREGROUND = fix.scene.hudColor[static_cast<size_t>(channel)] / 255.0;
            if (encodedSpaceInstead)
                expectation.color[static_cast<size_t>(channel)] = 255.0 * (BACKGROUND * (1.0 - ALPHA) + FOREGROUND * ALPHA);
            else
                expectation.color[static_cast<size_t>(channel)] = 255.0 * linearToSrgb(srgbToLinear(BACKGROUND) * (1.0 - ALPHA) + srgbToLinear(FOREGROUND) * ALPHA);
        }
        return expectation;
    }

}

TEST(EndToEnd, PartialAlphaCompositesInLinearLightFromAPremultipliedSource) {
    const auto& FIX      = fixture();
    ASSERT_EQ(FIX.scene.overlayAlpha, "premultiplied");
    ASSERT_EQ(FIX.bundle.overlay.alpha, "premultiplied") << "the manifest must say how the file stores colour";

    const auto RENDERED = renderCase("alpha-premultiplied", [](SRenderOptions& o) {
        o.eye        = eEyeSelection::LEFT;
        o.background = eBackgroundChoice::SOLID;
    });

    const SPaneDraw DEFAULTS; // the solid colour the SOLID background paints
    const size_t    K        = 20;
    const auto      EXPECTED = expectHud(FIX, K, RENDERED.report.fps, RENDERED.report.paneFov.at(0), DEFAULTS.solidColor, false);
    const auto      WRONG    = expectHud(FIX, K, RENDERED.report.fps, RENDERED.report.paneFov.at(0), DEFAULTS.solidColor, true);

    const SImage IMAGE    = RENDERED.frame(K);
    const auto   MEASURED = blockColor(IMAGE, EXPECTED.pixel[0], EXPECTED.pixel[1], 4);

    std::cout << "[measured] HUD at 75% alpha composites to (" << MEASURED[0] << ", " << MEASURED[1] << ", " << MEASURED[2] << "); linear-light prediction (" << EXPECTED.color[0] << ", "
              << EXPECTED.color[1] << ", " << EXPECTED.color[2] << "), encoded-space would be (" << WRONG.color[0] << ", " << WRONG.color[1] << ", " << WRONG.color[2] << ")\n";

    for (size_t channel = 0; channel < 3; ++channel)
        EXPECT_NEAR(MEASURED[channel], EXPECTED.color[channel], 3.0) << "channel " << channel;

    // The test is only worth anything if the wrong answer is far away.
    double separation = 0.0;
    for (size_t channel = 0; channel < 3; ++channel)
        separation = std::max(separation, std::abs(EXPECTED.color[channel] - WRONG.color[channel]));
    EXPECT_GT(separation, 15.0) << "the linear and encoded predictions must differ enough to tell apart";
}

TEST(EndToEnd, StraightAndPremultipliedBundlesComposeToTheSamePixels) {
    const auto& PREMULTIPLIED = fixture();
    const auto& STRAIGHT      = straightAlphaFixture();
    ASSERT_EQ(STRAIGHT.scene.overlayAlpha, "straight");
    ASSERT_EQ(STRAIGHT.bundle.overlay.alpha, "straight");

    const auto FROM_PREMULTIPLIED = renderCase("alpha-assoc-premul",
                                               [](SRenderOptions& o) {
                                                   o.eye        = eEyeSelection::LEFT;
                                                   o.background = eBackgroundChoice::SOLID;
                                               },
                                               PREMULTIPLIED);
    const auto FROM_STRAIGHT      = renderCase("alpha-assoc-straight",
                                               [](SRenderOptions& o) {
                                                   o.eye        = eEyeSelection::LEFT;
                                                   o.background = eBackgroundChoice::SOLID;
                                               },
                                               STRAIGHT);

    const SPaneDraw DEFAULTS;
    const size_t    K        = 20;
    const auto      EXPECTED = expectHud(PREMULTIPLIED, K, FROM_PREMULTIPLIED.report.fps, FROM_PREMULTIPLIED.report.paneFov.at(0), DEFAULTS.solidColor, false);

    const auto A = blockColor(FROM_PREMULTIPLIED.frame(K), EXPECTED.pixel[0], EXPECTED.pixel[1], 4);
    const auto B = blockColor(FROM_STRAIGHT.frame(K), EXPECTED.pixel[0], EXPECTED.pixel[1], 4);

    std::cout << "[measured] same HUD pixel from a premultiplied file (" << A[0] << ", " << A[1] << ", " << A[2] << ") and a straight one (" << B[0] << ", " << B[1] << ", " << B[2] << ")\n";

    // Two encodings of the same imagery must compose alike; the only difference
    // allowed is the rounding each encoding costs.
    for (size_t channel = 0; channel < 3; ++channel) {
        EXPECT_NEAR(A[channel], EXPECTED.color[channel], 3.0) << "premultiplied, channel " << channel;
        EXPECT_NEAR(B[channel], EXPECTED.color[channel], 3.0) << "straight, channel " << channel;
        EXPECT_NEAR(A[channel], B[channel], 3.0) << "the two associations disagree on channel " << channel;
    }
}

// ---------------------------------------------------------------------------
// 12. Quad records: v1 does not composite them, but it must carry them intact,
// and the head-relative semantics have to survive the round trip - a v2 that
// reads them as world poses would put every layer in the wrong place.
// ---------------------------------------------------------------------------
TEST(EndToEnd, QuadRecordsRoundTripWithHeadRelativePoses) {
    const auto& FIX = fixture();

    ASSERT_FALSE(FIX.bundle.telemetry.empty());
    for (const auto& RECORD : FIX.bundle.telemetry) {
        ASSERT_TRUE(RECORD.hasQuadsArray);
        ASSERT_EQ(RECORD.quads.size(), 3u);
        EXPECT_EQ(RECORD.quads[0].index, 0);
        EXPECT_EQ(RECORD.quads[1].index, 1);
        EXPECT_EQ(RECORD.quads[2].index, 2);
        EXPECT_FALSE(RECORD.quads[0].name.has_value()) << "the producer sends null names for now";
        EXPECT_FALSE(RECORD.quads[0].viewSpace) << "indices 0 and 1 are the room-anchored monitor's per-eye pair";
        EXPECT_FALSE(RECORD.quads[1].viewSpace);
        EXPECT_TRUE(RECORD.quads[2].viewSpace) << "index 2 is the head-locked HUD";
        EXPECT_TRUE(RECORD.head.has_value()) << "quad poses are relative to this";

        // The pinned reading of `visibility`: an eye mask, not an opacity. The
        // monitor's pair is one quad per eye sharing a pose and splitting a
        // side-by-side swapchain; the HUD goes to both.
        EXPECT_EQ(RECORD.quads[0].eyeVisibility, eEyeVisibility::LEFT);
        EXPECT_EQ(RECORD.quads[1].eyeVisibility, eEyeVisibility::RIGHT);
        EXPECT_EQ(RECORD.quads[2].eyeVisibility, eEyeVisibility::BOTH);
        EXPECT_TRUE(RECORD.quads[0].composedInEye(0));
        EXPECT_FALSE(RECORD.quads[0].composedInEye(1));
        EXPECT_FALSE(RECORD.quads[1].composedInEye(0));
        EXPECT_TRUE(RECORD.quads[1].composedInEye(1));
        EXPECT_TRUE(RECORD.quads[2].composedInEye(0));
        EXPECT_TRUE(RECORD.quads[2].composedInEye(1));
        // A per-eye pair is the same layer twice, so the poses must agree; only
        // the swapchain sub-rect differs.
        EXPECT_LT((RECORD.quads[0].pose.pos - RECORD.quads[1].pose.pos).length(), 1e-12);
        ASSERT_TRUE(RECORD.quads[0].hasRect);
        ASSERT_TRUE(RECORD.quads[1].hasRect);
        EXPECT_LT(RECORD.quads[0].rect[0], RECORD.quads[1].rect[0]) << "the pair takes opposite halves of one swapchain";
    }

    // The room-anchored layer: head * pose must land on the same STAGE pose at
    // every record, however much the head moved. That is the whole content of
    // "quad poses are head-relative", and it fails loudly if they were read as
    // world poses.
    double worstPosition = 0.0;
    double worstAngle    = 0.0;
    for (const auto& RECORD : FIX.bundle.telemetry) {
        const SPose WORLD = RECORD.quads[0].worldPose(RECORD.headPose());
        worstPosition     = std::max(worstPosition, (WORLD.pos - FIX.scene.overlayQuad.pos).length());
        worstAngle        = std::max(worstAngle, (FIX.scene.overlayQuad.rot.inverse() * WORLD.rot).log().length());
    }
    std::cout << "[measured] room-anchored quad re-anchors to within " << worstPosition * 1000.0 << " mm and " << worstAngle * 1000.0 << " mrad over the take\n";
    EXPECT_LT(worstPosition, 1e-6);
    EXPECT_LT(worstAngle, 1e-6);

    // And the head-locked layer is the opposite: constant head-relative, so its
    // STAGE pose must move with the head rather than stay put.
    const SPose FIRST = FIX.bundle.telemetry.front().quads[2].worldPose(FIX.bundle.telemetry.front().headPose());
    const SPose LATER = FIX.bundle.telemetry[45].quads[2].worldPose(FIX.bundle.telemetry[45].headPose());
    for (const auto& RECORD : FIX.bundle.telemetry)
        EXPECT_LT((RECORD.quads[2].pose.pos - FIX.scene.hudQuad.pos).length(), 1e-9) << "the head-locked pose must be recorded unchanged";
    EXPECT_GT((LATER.pos - FIRST.pos).length(), 1e-3) << "a head-locked layer's STAGE pose must follow the head";
}

// ---------------------------------------------------------------------------
// 14. Throughput, and the two claims it rests on.
//
// `validate` counts an overlay's frames from the container's packet index
// rather than by decoding it, and a segmented render composes chunks of the
// timeline in parallel worker processes. Both are only allowed to be faster if
// they are also the same. These tests are where "the same" is checked.
// ---------------------------------------------------------------------------

// Proves: the fast frame count and the slow one agree. `validate` trusts the
// packet count for the alignment rule - the n-th frame is the n-th undropped
// record - so if a packet were ever not a frame, the arbiter would be wrong
// about the one thing it exists to arbitrate.
TEST(EndToEnd, PacketCountedFramesAgreeWithADeepDecode) {
    const auto& FIX = fixture();

    CDiagnostics indexDiags, deepDiags;
    const auto   INDEXED = SBundle::load(FIX.take, indexDiags, SLoadOptions{.probeMedia = true, .probeDepth = eProbeDepth::INDEX, .checksum = false, .probeCache = nullptr});
    const auto   DEEP    = SBundle::load(FIX.take, deepDiags, SLoadOptions{.probeMedia = true, .probeDepth = eProbeDepth::DEEP, .checksum = false, .probeCache = nullptr});

    ASSERT_TRUE(INDEXED.has_value());
    ASSERT_TRUE(DEEP.has_value());
    EXPECT_FALSE(indexDiags.hasErrors());
    EXPECT_FALSE(deepDiags.hasErrors());

    ASSERT_EQ(INDEXED->overlay.videoInfo.size(), DEEP->overlay.videoInfo.size());
    ASSERT_FALSE(INDEXED->overlay.videoInfo.empty());
    for (size_t eye = 0; eye < INDEXED->overlay.videoInfo.size(); ++eye) {
        const auto& FAST = INDEXED->overlay.videoInfo[eye];
        const auto& SLOW = DEEP->overlay.videoInfo[eye];
        EXPECT_TRUE(FAST.intraOnly) << "the overlay encoders in the contract are all intra-only; that is what makes a packet a frame";
        EXPECT_EQ(FAST.ptsNs.size(), SLOW.ptsNs.size()) << "eye " << eye << ": counting packets and counting decoded frames must agree";
        EXPECT_EQ(FAST.ptsNs, SLOW.ptsNs) << "eye " << eye << ": and so must the timestamps";
        // The count the alignment rule is checked against.
        EXPECT_EQ(FAST.ptsNs.size(), INDEXED->overlay.frameTelemetryIndex.size());
    }

    ASSERT_EQ(INDEXED->cameras.size(), DEEP->cameras.size());
    for (size_t i = 0; i < INDEXED->cameras.size(); ++i)
        EXPECT_EQ(INDEXED->cameras[i].video.ptsNs.size(), DEEP->cameras[i].video.ptsNs.size()) << "camera " << INDEXED->cameras[i].key;

    std::cout << "[measured] overlay eye0: " << INDEXED->overlay.videoInfo[0].ptsNs.size() << " frames from the container index, " << DEEP->overlay.videoInfo[0].ptsNs.size()
              << " from a full decode\n";
}

// Proves: a decoder seeked to frame f emits exactly the frames a decoder run
// from the start emits from f onward, pixel for pixel, and numbers them the
// same. This is the whole basis of a segment worker: it starts partway along
// the source and must land on precisely the frame the ordinal rule assigns to
// its first output instant.
TEST(EndToEnd, SeekingToAnOrdinalFrameGivesTheSamePixelsAsDecodingToIt) {
    const auto& FIX = fixture();
    ASSERT_FALSE(FIX.bundle.overlay.videoPaths.empty());

    const std::string PATH   = FIX.bundle.overlay.videoPaths[0];
    const auto&       INFO   = FIX.bundle.overlay.videoInfo[0];
    const int         WIDTH  = FIX.bundle.overlay.width;
    const int         HEIGHT = FIX.bundle.overlay.height;
    ASSERT_GT(INFO.ptsNs.size(), 8u);

    std::string error;
    auto        sequential = CVideoReader::open(PATH, WIDTH, HEIGHT, error);
    ASSERT_TRUE(sequential) << error;

    // Every frame, decoded the slow way, kept for comparison.
    std::vector<std::vector<uint8_t>> reference;
    for (size_t i = 0; i < INFO.ptsNs.size(); ++i) {
        ASSERT_TRUE(sequential->advanceTo(i, error)) << error;
        ASSERT_EQ(sequential->currentIndex().value(), i);
        reference.push_back(sequential->rgba());
    }

    // Every start, and the whole tail from each - not a sample of either.
    //
    // A weaker version of this test passed while the compositor was quietly
    // wrong: checking only the first few frames after a seek missed that
    // ffmpeg's default constant-frame-rate conversion emits one *extra* frame
    // after `-ss` and slips the sequence by one several frames later. A seek is
    // only correct if everything after it is.
    size_t checked = 0;
    for (size_t start = 1; start < INFO.ptsNs.size(); ++start) {
        const auto SEEK = seekSecondsForFrame(INFO, start);
        ASSERT_TRUE(SEEK.has_value()) << "an intra-only source must be seekable to any frame; frame " << start;

        SReaderOptions options;
        options.startFrame  = start;
        options.seekSeconds = *SEEK;
        auto seeked         = CVideoReader::open(PATH, WIDTH, HEIGHT, error, options);
        ASSERT_TRUE(seeked) << error;

        for (size_t i = start; i < INFO.ptsNs.size(); ++i) {
            ASSERT_TRUE(seeked->advanceTo(i, error)) << error;
            ASSERT_EQ(seeked->currentIndex().value(), i) << "a seeked reader must number its frames on the whole-file timeline";
            ASSERT_EQ(seeked->rgba(), reference[i]) << "frame " << i << " reached by seeking to " << start << " differs from the same frame decoded from the start";
            ++checked;
        }
        // And the seek must be a jump, not a silent decode from zero.
        EXPECT_LE(seeked->framesDecoded(), INFO.ptsNs.size() - start) << "seeking to frame " << start << " decoded frames before it";
    }
    std::cout << "[measured] " << checked << " seeked frames across " << INFO.ptsNs.size() - 1 << " start points matched their sequentially decoded originals byte for byte\n";
}

TEST(EndToEnd, SegmentRangesTileTheTimelineExactly) {
    for (size_t frames : {size_t{1}, size_t{7}, size_t{45}, size_t{4137}}) {
        for (int jobs : {1, 2, 3, 4, 8, 12}) {
            size_t next = 0;
            size_t widest = 0, narrowest = frames + 1;
            for (int i = 0; i < jobs; ++i) {
                const auto RANGE = segmentRange(frames, i, jobs);
                EXPECT_EQ(RANGE.first, next) << "frames=" << frames << " jobs=" << jobs << " segment " << i << " must start where the last one ended";
                EXPECT_LE(RANGE.first, RANGE.second);
                next = RANGE.second;
                if (RANGE.second > RANGE.first) {
                    widest    = std::max(widest, RANGE.second - RANGE.first);
                    narrowest = std::min(narrowest, RANGE.second - RANGE.first);
                }
            }
            EXPECT_EQ(next, frames) << "frames=" << frames << " jobs=" << jobs << ": the segments must cover the whole timeline";
            if (narrowest <= frames) {
                EXPECT_LE(widest - narrowest, 1u) << "frames=" << frames << " jobs=" << jobs << ": chunk sizes must differ by at most one";
            }
        }
    }
}

// Proves: --jobs changes how long a render takes and nothing else. The output
// is compared after decoding, because that is where the guarantee lives: K
// encoders each start cold, so the *bitstreams* differ even for a lossless
// codec, while the pixels they carry must not.
TEST(EndToEnd, ASegmentedRenderComposesTheSameFramesAsASingleJob) {
    const auto& FIX = fixture();

    const auto render = [&](const std::string& name, int jobs) {
        SRenderOptions options;
        options.take    = FIX.take;
        options.outPath = (scratchRoot() / (name + ".mkv")).string();
        options.width   = PANE_WIDTH;
        options.height  = PANE_HEIGHT;
        options.noAudio = true;
        // Lossless, so "the same decoded pixels" is a statement about the
        // compositor rather than about two encoders' rate decisions.
        options.videoCodec = "ffv1";
        options.jobs       = jobs;
        // The workers are copies of this program, and this program is the test
        // binary; CMake hands over the real one.
        options.workerBinary = HYPXRCOMPOSE_BINARY;

        SRenderReport report;
        setLogLevel(eLogLevel::WARN);
        const int STATUS = runRender(options, &report);
        setLogLevel(eLogLevel::INFO);
        EXPECT_EQ(STATUS, 0) << name;
        return std::pair{options.outPath, report};
    };

    const auto [SINGLE_PATH, SINGLE] = render("segment-single", 1);
    const auto [MANY_PATH, MANY]     = render("segment-many", 4);

    ASSERT_FALSE(SINGLE.frames.empty());
    ASSERT_EQ(SINGLE.frames.size(), MANY.frames.size()) << "a segmented render must compose every frame the serial one does, and no others";
    EXPECT_EQ(MANY.jobs, 4);

    // 1. The timeline: every output frame drew on the same sources.
    for (size_t k = 0; k < SINGLE.frames.size(); ++k) {
        const auto& A = SINGLE.frames[k];
        const auto& B = MANY.frames[k];
        ASSERT_EQ(A.index, B.index) << "frame " << k;
        EXPECT_EQ(A.tHostNs, B.tHostNs) << "frame " << k << ": the output instant must not depend on which worker computed it";
        EXPECT_EQ(A.telemetryIndex, B.telemetryIndex) << "frame " << k;
        EXPECT_EQ(A.overlayFrame, B.overlayFrame) << "frame " << k << ": a worker that seeked must land on the ordinal rule's frame";
        EXPECT_EQ(A.overlayTelemetryIndex, B.overlayTelemetryIndex) << "frame " << k;
        EXPECT_EQ(A.cameraFrame, B.cameraFrame) << "frame " << k;
    }

    // 2. The pixels.
    std::string error;
    const int   WIDTH  = SINGLE.paneWidth * SINGLE.paneCount;
    const int   HEIGHT = SINGLE.paneHeight;
    auto        single = CVideoReader::open(SINGLE_PATH, WIDTH, HEIGHT, error);
    ASSERT_TRUE(single) << error;
    auto many = CVideoReader::open(MANY_PATH, WIDTH, HEIGHT, error);
    ASSERT_TRUE(many) << error;

    // 2. The pixels.
    //
    // Nearly every frame comes back byte-identical, and the handful that do not
    // differ by an LSB or two on a few pixels out of ~170000. That residue is
    // the GPU's, not the compositor's: the same binary rendering the same range
    // twice is bit-exact (checked while chasing this), so what varies is only
    // how equal two *processes* can be made, and neither uploading through one
    // path, nor allocating every texture up front, nor composing a throwaway
    // frame first moved it. It shows up only where the background is a
    // photographic camera texture; a synthetic flat overlay hides it.
    //
    // So the assertion is a tight numeric bound rather than equality, and the
    // bound is chosen to be far below any real defect. For scale: a genuine
    // off-by-one in the segment seek - a real bug this test caught - missed by
    // ~10000 pixels at 161 LSB. This allows <0.1% of pixels at <=4 LSB, three
    // orders of magnitude tighter, and the frame-by-frame source mapping
    // checked above is exact regardless.
    size_t exact = 0, approximate = 0;
    size_t worstPixels = 0;
    int    worstDelta  = 0;
    for (size_t k = 0; k < SINGLE.frames.size(); ++k) {
        ASSERT_TRUE(single->advanceTo(k, error)) << error;
        ASSERT_TRUE(many->advanceTo(k, error)) << error;
        const auto& A = single->rgba();
        const auto& B = many->rgba();
        ASSERT_EQ(A.size(), B.size());

        if (A == B) {
            ++exact;
            continue;
        }

        size_t differingPixels = 0;
        int    maxDelta        = 0;
        for (size_t i = 0; i < A.size(); i += 4) {
            int delta = 0;
            for (size_t c = 0; c < 4; ++c)
                delta = std::max(delta, std::abs(static_cast<int>(A[i + c]) - static_cast<int>(B[i + c])));
            if (delta > 0) {
                ++differingPixels;
                maxDelta = std::max(maxDelta, delta);
            }
        }
        const size_t PIXELS = A.size() / 4;
        EXPECT_LE(maxDelta, 4) << "output frame " << k << ": --jobs may cost an LSB or two of GPU reproducibility, not " << maxDelta << " - that is a composite that differs, not a rounding";
        EXPECT_LT(static_cast<double>(differingPixels) / static_cast<double>(PIXELS), 0.001)
            << "output frame " << k << ": " << differingPixels << " of " << PIXELS << " pixels differ, far too many to be texture reproducibility";
        worstPixels = std::max(worstPixels, differingPixels);
        worstDelta  = std::max(worstDelta, maxDelta);
        ++approximate;
    }

    EXPECT_EQ(exact + approximate, SINGLE.frames.size());
    std::cout << "[measured] " << exact << " of " << SINGLE.frames.size() << " frames byte-identical between --jobs 1 and --jobs 4; the other " << approximate << " within " << worstDelta
              << " LSB on at most " << worstPixels << " pixel(s) of " << (SINGLE.paneWidth * SINGLE.paneCount * SINGLE.paneHeight) << "\n";
}

// Proves: a side-by-side render says so in the file, and a mono one does not.
//
// Reported from the first real stereo take: the output was a correct SBS pair
// that no player could recognize as one, because nothing in the container or the
// bitstream said "side by side". The XR desktop auto-tags stereo windows off
// exactly these signals, so our own output failed our own pipeline.
TEST(EndToEnd, StereoSideBySideOutputCarriesStereoSignalling) {
    const auto& FIX = fixture();

    const auto renderTo = [&](const std::string& name, eEyeSelection eye) {
        SRenderOptions options;
        options.take        = FIX.take;
        options.outPath     = (scratchRoot() / name).string();
        options.width       = 160;
        options.height      = 120;
        options.eye         = eye;
        options.noAudio     = true;
        options.limitFrames = 6;
        setLogLevel(eLogLevel::WARN);
        const int STATUS = runRender(options);
        setLogLevel(eLogLevel::INFO);
        EXPECT_EQ(STATUS, 0) << name;
        return options.outPath;
    };

    // Matroska carries StereoMode in the container; ffprobe reads it back as a
    // stream tag. This is the signal mpv and the XR desktop's window tagging use.
    const auto matroskaTag = [](const std::string& path) {
        std::string text, error;
        runCapture({"ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream_tags=stereo_mode", "-of", "csv=p=0", path}, text, error);
        while (!text.empty() && (text.back() == '\n' || text.back() == ',' || text.back() == '\r'))
            text.pop_back();
        return text;
    };
    // MP4 has no tag ffmpeg's mov muxer will take, so the signal lives in the
    // bitstream as frame-packing SEI and surfaces as a frame side data.
    const auto hasStereoSideData = [](const std::string& path) {
        std::string text, error;
        runCapture({"ffprobe", "-v", "error", "-select_streams", "v:0", "-read_intervals", "%+#1", "-show_frames", "-of", "default", path}, text, error);
        return text.find("side_data_type=Stereo 3D") != std::string::npos;
    };

    const std::string SBS_MKV  = renderTo("stereo-signal.mkv", eEyeSelection::STEREO_SBS);
    const std::string SBS_MP4  = renderTo("stereo-signal.mp4", eEyeSelection::STEREO_SBS);
    const std::string MONO_MKV = renderTo("mono-signal.mkv", eEyeSelection::LEFT);
    const std::string MONO_MP4 = renderTo("mono-signal.mp4", eEyeSelection::LEFT);

    EXPECT_EQ(matroskaTag(SBS_MKV), "left_right") << "a stereo .mkv must carry StereoMode left_right";
    EXPECT_TRUE(hasStereoSideData(SBS_MKV)) << "and the frame-packing SEI too, since libx264 can write it";
    EXPECT_TRUE(hasStereoSideData(SBS_MP4)) << "a stereo .mp4 must carry frame-packing SEI, arrangement type 3";

    // A wrong stereo flag is worse than none: a mono take tagged side-by-side
    // shows as two half-width pictures in anything that honours the signal.
    EXPECT_EQ(matroskaTag(MONO_MKV), "") << "a mono render must not claim to be stereo";
    EXPECT_FALSE(hasStereoSideData(MONO_MKV));
    EXPECT_FALSE(hasStereoSideData(MONO_MP4));

    std::cout << "[measured] stereo .mkv: StereoMode=" << matroskaTag(SBS_MKV) << " + frame-packing SEI; stereo .mp4: frame-packing SEI; mono: neither\n";
}

// ---------------------------------------------------------------------------
// 15. The output frustum.
//
// A recorded eye frustum is asymmetric and its angular aspect is whatever the
// runtime chose. The pane the user asks for has its own aspect. Handing the
// recorded fov straight to the shader maps one linearly onto the other, which
// stretches the picture by the ratio between them and slides the optical axis
// off where it belongs. These tests are the ones the default fixture could not
// be: its angular aspect happens to sit within 0.12% of the pane's, so every
// stretch cancelled and nothing noticed.
// ---------------------------------------------------------------------------

TEST(EndToEnd, TheOutputFrustumTakesThePanesAspectAndKeepsTheRecordedOpticalAxis) {
    // The reference take's own numbers, eye 0.
    const SFov RECORDED{-0.9425, 0.6981, 0.7679, -0.9599};

    EXPECT_NEAR(RECORDED.angularAspect(), 0.9255, 1e-3);
    EXPECT_NEAR(RECORDED.opticalCentreU(), 0.621, 1e-3) << "the optical axis is nowhere near the middle of the eye buffer";
    EXPECT_NEAR(RECORDED.opticalCentreV(), 0.403, 1e-3);

    // What the old behaviour cost: the recorded frustum mapped onto the pane.
    for (const auto& [W, H, WAS] : std::vector<std::tuple<int, int, double>>{{2064, 2162, 0.9547}, {1920, 1080, 16.0 / 9.0}}) {
        const double STRETCH = WAS / RECORDED.angularAspect();
        const SFov   FITTED  = fitFovToPane(RECORDED, W, H);

        // The whole point: output pixels are square in angle, so the frustum's
        // aspect is the pane's aspect exactly.
        EXPECT_NEAR(FITTED.angularAspect(), static_cast<double>(W) / static_cast<double>(H), 1e-9)
            << W << "x" << H << ": before the fix this was " << RECORDED.angularAspect() << ", a " << STRETCH << "x horizontal stretch";

        // Padded, never cropped: everything the eye recorded is still on screen.
        EXPECT_LE(FITTED.l, RECORDED.l + 1e-12);
        EXPECT_GE(FITTED.r, RECORDED.r - 1e-12);
        EXPECT_GE(FITTED.u, RECORDED.u - 1e-12);
        EXPECT_LE(FITTED.d, RECORDED.d + 1e-12);

        // And the padding is symmetric in tan space, which is what keeps the
        // optical axis pointing exactly where the eye was pointing.
        const auto R = RECORDED.tangents();
        const auto F = FITTED.tangents();
        EXPECT_NEAR(R[0] - F[0], F[1] - R[1], 1e-9) << "horizontal padding must be even, or the picture slides sideways";
        EXPECT_NEAR(F[2] - R[2], R[3] - F[3], 1e-9);

        std::cout << "[measured] " << W << "x" << H << ": recorded aspect " << RECORDED.angularAspect() << " -> output " << FITTED.angularAspect() << "; the old mapping stretched by " << STRETCH
                  << "x, optical centre moved " << (FITTED.opticalCentreU() - RECORDED.opticalCentreU()) * 100.0 << "% of width\n";
    }
}

TEST(EndToEnd, BothEyesOfAStereoPairAreFittedAtOneScale) {
    // Two mirrored frusta, as a headset's are. Fitted independently they would
    // land at two magnifications and stop being a stereo pair.
    const SFov LEFT{-0.9425, 0.6981, 0.7679, -0.9599};
    const SFov RIGHT{-0.6981, 0.9425, 0.7679, -0.9599};

    const double SHARED = std::max(angularPixelForPane(LEFT, PANE_WIDTH, PANE_HEIGHT), angularPixelForPane(RIGHT, PANE_WIDTH, PANE_HEIGHT));
    const SFov   FL     = fitFovToPane(LEFT, PANE_WIDTH, PANE_HEIGHT, SHARED);
    const SFov   FR     = fitFovToPane(RIGHT, PANE_WIDTH, PANE_HEIGHT, SHARED);

    EXPECT_NEAR(FL.tanWidth() / PANE_WIDTH, FR.tanWidth() / PANE_WIDTH, 1e-12) << "one tan unit must be one pixel in both eyes";
    EXPECT_NEAR(FL.tanHeight() / PANE_HEIGHT, FR.tanHeight() / PANE_HEIGHT, 1e-12);
    // Mirrored in, mirrored out.
    EXPECT_NEAR(FL.opticalCentreU(), 1.0 - FR.opticalCentreU(), 1e-9);
}

// The end-to-end one: the output's angular scale must be the same in every
// direction, which is what "a square renders as a square" means.
//
// Measured from marker centroids rather than from one marker's bounding box: the
// markers are a handful of pixels across, so a box would quantize the aspect to
// ~14% and could not see a 1% error. The overlay is the right surface to measure
// on because `asis` reprojects it from the pose it was rendered at, at infinite
// depth - a pure rotation warp, exact - so nothing but the output frustum is
// between the ground truth and the pixels.
TEST(EndToEnd, TheOutputScaleIsIsotropicUnderTheRealFrustum) {
    const auto& FIX = realFrustumFixture();

    // Twice the pane, so the markers are wide enough for a centroid to resolve
    // a one-percent error rather than quantize it away.
    constexpr int WIDE = PANE_WIDTH * 4, TALL = PANE_HEIGHT * 4;
    const auto    RENDERED = renderCase(
        "real-frustum-square",
        [](SRenderOptions& options) {
            options.background = eBackgroundChoice::CHECKER;
            options.width      = WIDE;
            options.height     = TALL;
        },
        FIX, true);

    ASSERT_FALSE(RENDERED.report.paneFov.empty());
    const SFov& OUT = RENDERED.report.paneFov[0];
    EXPECT_NEAR(OUT.angularAspect(), static_cast<double>(WIDE) / static_cast<double>(TALL), 1e-9);

    // What the old mapping would have done to the same picture.
    const double WOULD_HAVE_BEEN = (static_cast<double>(WIDE) / static_cast<double>(TALL)) / FIX.scene.eyeFov[0].angularAspect();

    struct SPoint {
        std::array<double, 2> pixel;
        std::array<double, 2> tan;
        bool                  found = false;
    };

    // Scale is measured per axis from the rectangle's opposite sides, not from
    // the worst of all pairwise distances. A colour-threshold centroid sits a
    // fraction of a pixel off the projection of a marker's centre, and that bias
    // is what a max-over-pairs statistic reports; taking the x component across
    // the two horizontal sides and the y component across the two vertical ones
    // cancels most of it, because the same bias appears at both ends of a side
    // and on both sides of the rectangle.
    size_t frames = 0;
    double worst  = 0.0;
    for (size_t k = 0; k < std::min<size_t>(6, RENDERED.report.frames.size()); ++k) {
        const SImage IMAGE  = RENDERED.frame(k);
        const auto&  RECORD = RENDERED.report.frames[k];
        // The overlay is reprojected at infinite depth, so a marker lands at the
        // direction it had *from the eye that rendered it*, expressed in the
        // *output* camera's orientation. Those are two different records whenever
        // a frame was dropped.
        const SPose OUTPUT_EYE = FIX.eyePose(static_cast<int>(RECORD.telemetryIndex), 0);
        const SPose SOURCE_EYE = FIX.eyePose(static_cast<int>(RECORD.overlayTelemetryIndex[0]), 0);
        const SPose EYE{SOURCE_EYE.pos, OUTPUT_EYE.rot};

        const auto locate = [&](const char* name) {
            SPoint point;
            for (const auto& MARKER : FIX.scene.overlayMarkers) {
                if (MARKER.name != name)
                    continue;
                const auto MEASURED = findColor(IMAGE, MARKER.color, 40, 0, WIDE);
                if (!MEASURED || MEASURED->count < 80)
                    break;
                const SVec3 LOCAL = EYE.dirToLocal(overlayMarkerWorld(FIX.scene, name) - EYE.pos);
                if (!(LOCAL.z < 0.0))
                    break;
                point = {{MEASURED->x, MEASURED->y}, {LOCAL.x / -LOCAL.z, LOCAL.y / -LOCAL.z}, true};
            }
            return point;
        };

        const SPoint TL = locate("geomTL"), TR = locate("geomTR"), BL = locate("geomBL"), BR = locate("geomBR");
        if (!TL.found || !TR.found || !BL.found || !BR.found)
            continue;

        const auto axisScale = [](const SPoint& a, const SPoint& b, int axis) { return std::abs(a.pixel[axis] - b.pixel[axis]) / std::abs(a.tan[axis] - b.tan[axis]); };

        const double SX = 0.5 * (axisScale(TL, TR, 0) + axisScale(BL, BR, 0));
        const double SY = 0.5 * (axisScale(TL, BL, 1) + axisScale(TR, BR, 1));
        const double ASPECT_ERROR = std::abs(SX / SY - 1.0);
        worst                     = std::max(worst, ASPECT_ERROR);
        ++frames;

        EXPECT_LT(ASPECT_ERROR, 0.01) << "frame " << k << ": the output resolves " << SX << " px per tangent horizontally against " << SY
                                      << " vertically, so a square renders " << SX / SY << " times too wide. Mapping the recorded frustum straight onto the pane would make that "
                                      << WOULD_HAVE_BEEN;
    }

    ASSERT_GT(frames, 0u) << "no frame had all four geometry markers; the test proves nothing";
    std::cout << "[measured] " << frames << " frames under the real frustum: worst aspect error " << worst * 100.0 << "%, against the " << (WOULD_HAVE_BEEN - 1.0) * 100.0
              << "% the recorded-fov mapping would have introduced on this pane\n";
}

// And the optical axis: the whole picture must sit where the asymmetry puts it.
TEST(EndToEnd, MarkersLandWherePredictedUnderTheRealFrustum) {
    const auto& FIX = realFrustumFixture();

    const auto RENDERED = renderCase(
        "real-frustum-predict", [](SRenderOptions& options) {
            options.background      = eBackgroundChoice::CAMERA;
            options.backgroundAlign = eBackgroundAlign::RECORDED;
            options.backgroundDepth = 2.0;
        },
        FIX, true);
    ASSERT_FALSE(RENDERED.report.paneFov.empty());
    const SFov& OUT = RENDERED.report.paneFov[0];

    // A wrong optical centre would move everything by the difference between the
    // recorded and fitted centres - about 6% of the width here - so this is a
    // sharp test of the frustum, not just of the pose chain.
    const double CENTRE_SHIFT_PX = std::abs(OUT.opticalCentreU() - FIX.scene.eyeFov[0].opticalCentreU()) * PANE_WIDTH;

    size_t checked = 0;
    double worst   = 0.0;
    for (size_t k = 0; k < std::min<size_t>(4, RENDERED.report.frames.size()); ++k) {
        const SImage IMAGE   = RENDERED.frame(k);
        const auto&  RECORD  = RENDERED.report.frames[k];
        const SPose  EYE     = FIX.eyePose(static_cast<int>(RECORD.telemetryIndex), 0);
        const SPose  CAMERA  = FIX.headPose(static_cast<int>(RECORD.telemetryIndex)).compose(FIX.cameraExtrinsic(0, eBackgroundAlign::RECORDED));

        for (const auto& MARKER : FIX.scene.wallMarkers) {
            const auto PREDICTED = predictBackgroundPixel(EYE, OUT, PANE_WIDTH, PANE_HEIGHT, CAMERA, MARKER.world, 2.0);
            if (!PREDICTED)
                continue;
            if ((*PREDICTED)[0] < 20 || (*PREDICTED)[0] > PANE_WIDTH - 20 || (*PREDICTED)[1] < 20 || (*PREDICTED)[1] > PANE_HEIGHT - 20)
                continue;
            const auto MEASURED = findColor(IMAGE, MARKER.color, 60, 0, PANE_WIDTH);
            if (!MEASURED || MEASURED->count < 40)
                continue;
            const double DX = MEASURED->x - (*PREDICTED)[0];
            const double DY = MEASURED->y - (*PREDICTED)[1];
            const double D  = std::sqrt(DX * DX + DY * DY);
            worst           = std::max(worst, D);
            ++checked;
            EXPECT_LT(D, 2.5) << "frame " << k << ", marker `" << MARKER.name << "` landed " << D << " px from the prediction made against the derived output frustum";
        }
    }
    ASSERT_GT(checked, 0u);
    std::cout << "[measured] " << checked << " markers under the real frustum land within " << worst << " px of prediction; a frustum that ignored the recorded asymmetry would move them "
              << CENTRE_SHIFT_PX << " px\n";
}

// ---------------------------------------------------------------------------
// 16. Camera intrinsics live in the sensor's coordinates, not the image's.
//
// From a live viewing of the first real camera take: "passthrough is not
// covering the full scene - it's 3/4 black, only the top centre has
// passthrough". The convicted term was the principal point: Android states
// intrinsics against the sensor's ACTIVE ARRAY and then delivers a stream
// cropped from it, so cy = 638.6 - dead centre of a 1280x1280 array - was being
// applied to a 1280x960 image, where it means 66.5% down.
// ---------------------------------------------------------------------------

TEST(EndToEnd, IntrinsicsAreRebasedFromTheSensorArrayToTheImage) {
    // The first real camera take's own numbers.
    SCameraIntrinsics intr;
    intr.fx = intr.fy = 867.154175;
    intr.cx           = 641.459106;
    intr.cy           = 638.635071;
    intr.activeArray  = {0, 0, 1280, 1280};

    ASSERT_TRUE(intr.rebaseToImage(1280, 960));
    EXPECT_NEAR(intr.cx, 641.459106, 1e-9) << "no horizontal crop, so cx must not move";
    EXPECT_NEAR(intr.cy, 478.635071, 1e-9) << "the 1280x1280 array is cropped to 1280x960 about its centre, so cy loses 160";
    EXPECT_NEAR(intr.fx, 867.154175, 1e-9) << "a pure crop does not rescale the focal length";
    EXPECT_NEAR(intr.fy, 867.154175, 1e-9);

    // The correction is self-proving: it puts the principal point back at the
    // centre of the image, and makes the vertical field symmetric.
    EXPECT_NEAR(intr.cy / 960.0, 0.5, 0.005);
    const double UP   = std::atan(intr.cy / intr.fy) * 180.0 / M_PI;
    const double DOWN = std::atan((960.0 - intr.cy) / intr.fy) * 180.0 / M_PI;
    EXPECT_NEAR(UP, DOWN, 0.5) << "a camera sees about as far up as down; " << UP << " vs " << DOWN;

    // A pure scale rescales everything; no active array is a no-op.
    SCameraIntrinsics scaled;
    scaled.fx = scaled.fy = 800.0;
    scaled.cx = scaled.cy = 640.0;
    scaled.activeArray    = {0, 0, 1280, 960};
    ASSERT_TRUE(scaled.rebaseToImage(640, 480));
    EXPECT_NEAR(scaled.fx, 400.0, 1e-9);
    EXPECT_NEAR(scaled.cx, 320.0, 1e-9);

    SCameraIntrinsics bare;
    bare.fx = bare.fy = 500.0;
    bare.cx = bare.cy = 320.0;
    EXPECT_FALSE(bare.rebaseToImage(640, 480)) << "no active array declared means the intrinsics are already the image's";
}

// End to end: a bundle whose intrinsics are stated against a padded sensor array
// must compose exactly like one whose intrinsics are already in image
// coordinates. Without the rebase the markers miss by the crop offset.
TEST(EndToEnd, BackgroundMarkersLandWherePredictedWhenIntrinsicsCarryASensorCrop) {
    const auto& FIX = sensorArrayFixture();

    // The bundle declares the crop; the loader must have taken it back out.
    ASSERT_FALSE(FIX.bundle.cameras.empty());
    const auto& CAM = FIX.bundle.cameras.front();
    EXPECT_NEAR(CAM.intrinsics.cy, FIX.scene.cameras.front().intrinsics.cy, 1e-6)
        << "the loader must rebase the principal point back to image coordinates; it is off by the crop offset";
    EXPECT_GT(CAM.intrinsics.activeArray[3], CAM.video.height) << "the fixture is supposed to be carrying a padded active array";

    const double FPS      = FIX.scene.overlayHz;
    const auto   RENDERED = renderCase(
        "sensor-array", [](SRenderOptions& options) {
            options.background      = eBackgroundChoice::CAMERA;
            options.backgroundAlign = eBackgroundAlign::RECORDED;
            options.backgroundDepth = 2.0;
        },
        FIX, true);

    size_t checked = 0;
    double worst   = 0.0;
    for (size_t k : {size_t{10}, size_t{25}}) {
        const SImage IMAGE  = RENDERED.frame(k);
        const SPose  EYE    = FIX.outputCamera(static_cast<int>(FIX.outputRecord(k, FPS)), 0);
        const size_t FRAME  = predictCameraFrame(FIX, 0, k, FPS);
        const SPose  CAMERA = FIX.scene.headAt(FIX.cameraHostNs(0, FRAME)).compose(FIX.cameraExtrinsic(0, eBackgroundAlign::RECORDED));

        for (const auto& MARKER : FIX.scene.wallMarkers) {
            const auto PREDICTED = predictBackgroundPixel(EYE, RENDERED.report.paneFov.at(0), PANE_WIDTH, PANE_HEIGHT, CAMERA, MARKER.world, 2.0);
            if (!PREDICTED || (*PREDICTED)[0] < 20 || (*PREDICTED)[0] > PANE_WIDTH - 20 || (*PREDICTED)[1] < 20 || (*PREDICTED)[1] > PANE_HEIGHT - 20)
                continue;
            const auto MEASURED = findColor(IMAGE, MARKER.color, 60, 0, PANE_WIDTH);
            if (!MEASURED || MEASURED->count < 30)
                continue;
            const double D = std::hypot(MEASURED->x - (*PREDICTED)[0], MEASURED->y - (*PREDICTED)[1]);
            worst          = std::max(worst, D);
            ++checked;
            EXPECT_LT(D, 2.5) << "marker `" << MARKER.name << "` in frame " << k << " landed " << D << " px away; an un-rebased principal point would move it much further";
        }
    }
    ASSERT_GT(checked, 0u);
    std::cout << "[measured] with intrinsics stated against a padded sensor array, markers land within " << worst << " px of prediction\n";
}

// ---------------------------------------------------------------------------
// 17. The background is aimed, and it is reprojected from when it was captured.
// ---------------------------------------------------------------------------

// The camera image must be reprojected from the head pose at the frame's own
// CAPTURE instant, not from the pose at the output instant. A 30 Hz camera
// against a 45 Hz output means the two are up to half a camera period apart, and
// during that the head moves - which is the room lagging or leading the overlay
// that the wearer sees as swim.
TEST(EndToEnd, TheBackgroundIsReprojectedFromTheHeadPoseAtCaptureTime) {
    const auto&  FIX      = briskMotionFixture();
    const double FPS      = FIX.scene.overlayHz;
    const auto   RENDERED = renderCase(
        "capture-time", [](SRenderOptions& options) {
            options.background      = eBackgroundChoice::CAMERA;
            options.backgroundAlign = eBackgroundAlign::RECORDED;
            options.backgroundDepth = 2.0;
        },
        FIX, true);

    // The default alignment aims the camera, so the prediction has to use the
    // same aimed extrinsic - otherwise this measures the aiming, not the timing.
    const SPose extrinsic = FIX.cameraExtrinsic(0, eBackgroundAlign::RECORDED);

    // Scan the take rather than sampling three frames: how far apart the two
    // models are depends on where the head happens to be in its motion, and the
    // frames worth asserting on are the ones where they disagree.
    size_t checked = 0;
    double worstToCapture = 0.0, bestSeparation = 0.0;
    for (size_t k = 4; k < std::min<size_t>(RENDERED.report.frames.size(), 44); ++k) {
        const SImage IMAGE  = RENDERED.frame(k);
        const SPose  EYE    = FIX.outputCamera(static_cast<int>(FIX.outputRecord(k, FPS)), 0);
        const size_t FRAME  = predictCameraFrame(FIX, 0, k, FPS);

        // The two rival models: the head where it was when the shutter opened,
        // and the head where it is now. The compositor must be using the former.
        const SPose CAPTURE = FIX.scene.headAt(FIX.cameraHostNs(0, FRAME)).compose(extrinsic);
        const SPose OUTPUT  = FIX.scene.headAt(FIX.outputHostNs(k, FPS)).compose(extrinsic);

        for (const auto& MARKER : FIX.scene.wallMarkers) {
            const auto AT_CAPTURE = predictBackgroundPixel(EYE, RENDERED.report.paneFov.at(0), PANE_WIDTH, PANE_HEIGHT, CAPTURE, MARKER.world, 2.0);
            const auto AT_OUTPUT  = predictBackgroundPixel(EYE, RENDERED.report.paneFov.at(0), PANE_WIDTH, PANE_HEIGHT, OUTPUT, MARKER.world, 2.0);
            if (!AT_CAPTURE || !AT_OUTPUT)
                continue;
            if ((*AT_CAPTURE)[0] < 20 || (*AT_CAPTURE)[0] > PANE_WIDTH - 20 || (*AT_CAPTURE)[1] < 20 || (*AT_CAPTURE)[1] > PANE_HEIGHT - 20)
                continue;
            const auto MEASURED = findColor(IMAGE, MARKER.color, 60, 0, PANE_WIDTH);
            if (!MEASURED || MEASURED->count < 30)
                continue;

            const double TO_CAPTURE = std::hypot(MEASURED->x - (*AT_CAPTURE)[0], MEASURED->y - (*AT_CAPTURE)[1]);
            const double SEPARATION = std::hypot((*AT_CAPTURE)[0] - (*AT_OUTPUT)[0], (*AT_CAPTURE)[1] - (*AT_OUTPUT)[1]);

            // Only sightings where the two models actually part company can
            // testify. They part by the head's TRANSLATION between capture and
            // output - not its rotation, because the ray from the lens to a world
            // point is the same ray whichever way the lens is turned - so the
            // separation is a couple of pixels even with a briskly moving head,
            // and the gate is set where the measurement can still resolve it.
            if (SEPARATION < 1.5)
                continue;
            bestSeparation = std::max(bestSeparation, SEPARATION);
            worstToCapture = std::max(worstToCapture, TO_CAPTURE);
            ++checked;
            EXPECT_LT(TO_CAPTURE, 2.5) << "marker `" << MARKER.name << "` in frame " << k << " is " << TO_CAPTURE << " px from the capture-time prediction";
            EXPECT_LT(TO_CAPTURE, SEPARATION * 0.75) << "marker `" << MARKER.name << "` in frame " << k << " sits nearer the output-time model than the capture-time one, which is the swim itself";
        }
    }
    ASSERT_GT(checked, 5u) << "no frame had the two models far enough apart to tell them apart; the fixture is not moving enough";
    std::cout << "[measured] over " << checked << " marker sightings where the two models differ by up to " << bestSeparation << " px, the background sits within " << worstToCapture
              << " px of the capture-time one\n";
}

// `--bg-align auto` is a deliberate trade, and this measures both halves of it.
//
// It points each camera's optical axis along the output's forward, so the
// passthrough lands centred instead of sliding off the frame - that is the half
// the wearer asked for. The other half is that it discards the recorded swing,
// so the background is re-registered against the world by exactly that angle.
// On a synthetic take, where the extrinsic IS ground truth, that shows up as a
// registration error; on the real take the recorded swing is measured against
// the IMU rather than the head, so it is wrong to begin with and dropping it is
// the better guess. Both facts belong in the record.
TEST(EndToEnd, BgAlignAutoTradesRegistrationForCoverage) {
    const auto&  FIX = fixture();
    const double FPS = FIX.scene.overlayHz;

    const auto measure = [&](eBackgroundAlign align) {
        const auto RENDERED = renderCase(
            align == eBackgroundAlign::AUTO ? "align-auto" : "align-recorded",
            [align](SRenderOptions& options) {
                options.background      = eBackgroundChoice::CAMERA;
                options.backgroundAlign = align;
                options.backgroundDepth = 2.0;
            },
            FIX, true);

        const size_t K      = 20;
        const size_t RECORD = FIX.outputRecord(K, FPS);
        const SPose  EYE    = FIX.outputCamera(static_cast<int>(RECORD), 0);
        const SFov&  FOV    = RENDERED.report.paneFov.at(0);

        // Where the camera's optical axis lands: the coverage half.
        const SPose CAMERA = FIX.headPose(static_cast<int>(RECORD)).compose(FIX.cameraExtrinsic(0, align));
        double      ax = 0.0, ay = 0.0;
        EXPECT_TRUE(fovProject(FOV, EYE.dirToLocal(CAMERA.dirToWorld({0.0, 0.0, -1.0})), PANE_WIDTH, PANE_HEIGHT, ax, ay));

        // Where a known world feature lands versus the truth: the registration
        // half. predictBackgroundPixel does not depend on the camera's rotation -
        // a ray from the lens to the marker is a ray whichever way the lens is
        // turned - so this prediction is the same for both modes, and any
        // difference in the *measured* position is re-registration.
        const size_t FRAME    = predictCameraFrame(FIX, 0, K, FPS);
        const SPose  TRUE_CAM = FIX.scene.headAt(FIX.cameraHostNs(0, FRAME)).compose(FIX.cameraExtrinsic(0, eBackgroundAlign::RECORDED));
        const auto&  MARKER   = wallMarker(FIX.scene, "green");
        const auto   TRUTH    = predictBackgroundPixel(EYE, FOV, PANE_WIDTH, PANE_HEIGHT, TRUE_CAM, MARKER.world, 2.0);
        const auto   MEASURED = findColor(RENDERED.frame(K), MARKER.color, 60, 0, PANE_WIDTH);
        double       drift    = -1.0;
        if (TRUTH && MEASURED && MEASURED->count > 30)
            drift = std::hypot(MEASURED->x - (*TRUTH)[0], MEASURED->y - (*TRUTH)[1]);

        return std::tuple{std::hypot(ax - PANE_WIDTH * 0.5, ay - PANE_HEIGHT * 0.5), drift};
    };

    const auto [AUTO_OFF, AUTO_DRIFT]         = measure(eBackgroundAlign::AUTO);
    const auto [RECORDED_OFF, RECORDED_DRIFT] = measure(eBackgroundAlign::RECORDED);

    ASSERT_GT(AUTO_DRIFT, 0.0);
    ASSERT_GT(RECORDED_DRIFT, 0.0);

    // Coverage: auto centres the camera, recorded does not.
    EXPECT_LT(AUTO_OFF, 2.0) << "auto must put the optical axis at the pane centre, not " << AUTO_OFF << " px away";
    EXPECT_GT(RECORDED_OFF, AUTO_OFF + 3.0) << "the fixture's extrinsic should be aimed off-axis, or this proves nothing";

    // Registration: recorded is right on a take whose extrinsic is ground truth,
    // and auto is wrong by about the swing it threw away.
    EXPECT_LT(RECORDED_DRIFT, 2.5) << "with a ground-truth extrinsic, `recorded` must land the world where it is";
    EXPECT_GT(AUTO_DRIFT, RECORDED_DRIFT + 2.0) << "auto is supposed to re-register the background; if it does not, it is not doing anything";

    std::cout << "[measured] --bg-align auto: optical axis " << AUTO_OFF << " px from centre (recorded " << RECORDED_OFF << "), world feature drifts " << AUTO_DRIFT
              << " px (recorded " << RECORDED_DRIFT << ") - coverage bought, registration spent\n";
}
