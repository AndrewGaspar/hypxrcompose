// synth -> render -> verify.
//
// Every assertion here is quantitative, because the synthetic bundle's geometry
// is closed-form: the correct output pixel for a known feature is computable, so
// the test compares a measurement against a prediction rather than against a
// stored image. What each one proves is stated at the top of the test.

#include "Ffmpeg.hpp"
#include "Harness.hpp"
#include "Log.hpp"
#include "Render.hpp"
#include "Stabilize.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <iostream>

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

    const SSynthWallMarker& wallMarker(const SSynthScene& scene, const std::string& name) {
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

        const SPose OUTPUT_EYE = FIX.eyePose(static_cast<int>(OUTPUT_RECORD), 0);
        const SPose SOURCE_EYE = FIX.eyePose(static_cast<int>(SOURCE_RECORD), 0);
        const SFov& FOV        = FIX.scene.eyeFov[0];

        for (const auto& MARKER : FIX.scene.overlayMarkers) {
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
        o.eye        = eEyeSelection::LEFT;
        o.background = eBackgroundChoice::CAMERA;
    });

    const double FPS   = RENDERED.report.fps;
    double       worst = 0.0;

    for (size_t k : {size_t{10}, size_t{25}, size_t{40}}) {
        const SImage IMAGE = RENDERED.frame(k);
        const SPose  EYE   = FIX.eyePose(static_cast<int>(FIX.outputRecord(k, FPS)), 0);
        const SFov&  FOV   = FIX.scene.eyeFov[0];

        const size_t CAMERA_FRAME = predictCameraFrame(FIX, 0, k, FPS);
        ASSERT_EQ(RENDERED.report.frames[k].cameraFrame[0], static_cast<int64_t>(CAMERA_FRAME)) << "output frame " << k;

        const SPose CAMERA_POSE = FIX.scene.headAt(FIX.cameraHostNs(0, CAMERA_FRAME)).compose(FIX.camera(0).headToCamera);

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
    const SPose  CAMERA_POSE  = FIX.scene.headAt(FIX.cameraHostNs(0, CAMERA_FRAME)).compose(FIX.camera(0).headToCamera);

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
        o.eye        = eEyeSelection::LEFT;
        o.background = eBackgroundChoice::CAMERA;
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
        const SPose  EYE         = FIX.eyePose(static_cast<int>(FIX.outputRecord(k, FPS)), 0);
        const SPose  CAMERA_POSE = FIX.scene.headAt(FIX.cameraHostNs(0, CHOSEN)).compose(FIX.camera(0).headToCamera);
        const auto   PATCH       = predictBackgroundPixel(EYE, FIX.scene.eyeFov[0], PANE_WIDTH, PANE_HEIGHT, CAMERA_POSE, FIX.scene.codeCentre, 2.0);
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
TEST(EndToEnd, StereoPanesDifferByTheSyntheticIpdParallax) {
    const auto& FIX      = fixture();
    const auto  RENDERED = renderCase("stereo", [](SRenderOptions& o) { o.eye = eEyeSelection::STEREO_SBS; });

    ASSERT_EQ(RENDERED.report.paneCount, 2);

    const size_t K     = 25;
    const SImage IMAGE = RENDERED.frame(K);
    ASSERT_EQ(IMAGE.width, PANE_WIDTH * 2);

    const double FPS            = RENDERED.report.fps;
    const size_t OUTPUT_RECORD  = FIX.outputRecord(K, FPS);
    const size_t SOURCE_RECORD  = FIX.overlaySourceRecord(K, FPS);

    const SVec3 MARKER   = overlayMarkerWorld(FIX.scene, "centre");
    const SPose HEAD     = FIX.headPose(static_cast<int>(OUTPUT_RECORD));
    const SPose EYE_L    = FIX.eyePose(static_cast<int>(OUTPUT_RECORD), 0);
    const SPose EYE_R    = FIX.eyePose(static_cast<int>(OUTPUT_RECORD), 1);
    const SPose SOURCE_L = FIX.eyePose(static_cast<int>(SOURCE_RECORD), 0);
    const SPose SOURCE_R = FIX.eyePose(static_cast<int>(SOURCE_RECORD), 1);

    const auto PREDICTED_L = predictOverlayPixel(EYE_L, FIX.scene.eyeFov[0], PANE_WIDTH, PANE_HEIGHT, SOURCE_L, MARKER);
    const auto PREDICTED_R = predictOverlayPixel(EYE_R, FIX.scene.eyeFov[1], PANE_WIDTH, PANE_HEIGHT, SOURCE_R, MARKER);
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

    // Now isolate the parallax from the frustum asymmetry: predict again with both
    // eyes collapsed onto the head, and take the difference of differences.
    const SPose SOURCE_HEAD = FIX.headPose(static_cast<int>(SOURCE_RECORD));
    const auto  NO_IPD_L    = predictOverlayPixel(HEAD, FIX.scene.eyeFov[0], PANE_WIDTH, PANE_HEIGHT, SOURCE_HEAD, MARKER);
    const auto  NO_IPD_R    = predictOverlayPixel(HEAD, FIX.scene.eyeFov[1], PANE_WIDTH, PANE_HEIGHT, SOURCE_HEAD, MARKER);
    ASSERT_TRUE(NO_IPD_L.has_value());
    ASSERT_TRUE(NO_IPD_R.has_value());

    const double MEASURED_DISPARITY  = MEASURED_L->x - (MEASURED_R->x - PANE_WIDTH);
    const double PREDICTED_DISPARITY = (*PREDICTED_L)[0] - (*PREDICTED_R)[0];
    const double ASYMMETRY_ONLY      = (*NO_IPD_L)[0] - (*NO_IPD_R)[0];
    const double PARALLAX            = PREDICTED_DISPARITY - ASYMMETRY_ONLY;

    std::cout << "[measured] stereo disparity " << MEASURED_DISPARITY << " px (predicted " << PREDICTED_DISPARITY << "), of which " << PARALLAX << " px is IPD parallax\n";

    EXPECT_NEAR(MEASURED_DISPARITY, PREDICTED_DISPARITY, 2.0);
    // A point in front of the viewer sits further right in the left eye's image, so
    // the parallax term must be positive and big enough to be a real signal.
    EXPECT_GT(PARALLAX, 4.0);
    EXPECT_LT(std::abs(MEASURED_DISPARITY - PREDICTED_DISPARITY), std::abs(PARALLAX) * 0.5) << "the measurement must be nearer the with-IPD prediction than the without-IPD one";
}

// ---------------------------------------------------------------------------
// 6. The assumed-depth knob does what it says.
// ---------------------------------------------------------------------------
TEST(EndToEnd, ChangingTheAssumedBackgroundDepthMovesTheImageByThePredictedParallax) {
    const auto& FIX = fixture();

    const auto AT_TWO = renderCase("depth-2", [](SRenderOptions& o) {
        o.eye             = eEyeSelection::LEFT;
        o.background      = eBackgroundChoice::CAMERA;
        o.backgroundDepth = 2.0;
    });
    const auto AT_TEN = renderCase("depth-10", [](SRenderOptions& o) {
        o.eye             = eEyeSelection::LEFT;
        o.background      = eBackgroundChoice::CAMERA;
        o.backgroundDepth = 10.0;
    });

    const size_t K            = 20;
    const SPose  EYE          = FIX.eyePose(static_cast<int>(FIX.outputRecord(K, AT_TWO.report.fps)), 0);
    const size_t CAMERA_FRAME = predictCameraFrame(FIX, 0, K, AT_TWO.report.fps);
    const SPose  CAMERA_POSE  = FIX.scene.headAt(FIX.cameraHostNs(0, CAMERA_FRAME)).compose(FIX.camera(0).headToCamera);
    const auto&  MARKER       = wallMarker(FIX.scene, "green");

    const auto PREDICTED_TWO = predictBackgroundPixel(EYE, FIX.scene.eyeFov[0], PANE_WIDTH, PANE_HEIGHT, CAMERA_POSE, MARKER.world, 2.0);
    const auto PREDICTED_TEN = predictBackgroundPixel(EYE, FIX.scene.eyeFov[0], PANE_WIDTH, PANE_HEIGHT, CAMERA_POSE, MARKER.world, 10.0);
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
    const SPose  OUTPUT = SMOOTHED[OUTPUT_RECORD].compose(HEAD.inverse().compose(EYE));
    const SVec3  MARKER = overlayMarkerWorld(FIX.scene, "centre");

    // Infinite foreground depth means the warp preserves directions from the
    // recording eye, so the prediction uses the recorded eye for the direction and
    // the smoothed camera for the frustum.
    const auto PREDICTED = predictOverlayPixel(OUTPUT, FIX.scene.eyeFov[0], PANE_WIDTH, PANE_HEIGHT, SOURCE, MARKER);
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
