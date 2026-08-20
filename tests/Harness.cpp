#include "Harness.hpp"
#include "Ffmpeg.hpp"
#include "Log.hpp"

#include <cmath>
#include <cstdlib>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <unistd.h>

namespace fs = std::filesystem;

namespace hxctest {

    fs::path scratchRoot() {
        static const fs::path ROOT = [] {
            if (const char* OVERRIDE = std::getenv("HYPXRCOMPOSE_TEST_DIR"))
                return fs::path(OVERRIDE);
            return fs::temp_directory_path() / std::format("hypxrcompose-tests-{}", getpid());
        }();
        std::error_code ec;
        fs::create_directories(ROOT, ec);
        return ROOT;
    }

    bool loadImage(const fs::path& path, SImage& out, std::string& error) {
        return readPng(path.string(), out.rgba, out.width, out.height, error);
    }

    std::optional<SCentroid> findColor(const SImage& image, const std::array<int, 3>& color, int tolerance, int x0, int x1) {
        SCentroid centroid;
        x0 = std::max(0, x0);
        x1 = std::min(image.width, x1);

        for (int y = 0; y < image.height; ++y) {
            for (int x = x0; x < x1; ++x) {
                const auto PIXEL    = image.at(x, y);
                const int  DISTANCE = std::abs(PIXEL[0] - color[0]) + std::abs(PIXEL[1] - color[1]) + std::abs(PIXEL[2] - color[2]);
                if (DISTANCE > tolerance)
                    continue;
                centroid.x += static_cast<double>(x);
                centroid.y += static_cast<double>(y);
                ++centroid.count;
            }
        }

        if (centroid.count == 0)
            return std::nullopt;
        centroid.x /= static_cast<double>(centroid.count);
        centroid.y /= static_cast<double>(centroid.count);
        return centroid;
    }

    SPose SFixture::headPose(int frame) const {
        return scene.headAt(scene.frameHostNs(frame));
    }

    SPose SFixture::eyePose(int frame, int eye) const {
        return headPose(frame).compose({{(eye == 0 ? -0.5 : 0.5) * scene.ipd, 0.0, 0.0}, SQuat::identity()});
    }

    const SSynthCamera& SFixture::camera(int eye) const {
        for (const auto& CAMERA : scene.cameras) {
            if (CAMERA.eye == eye)
                return CAMERA;
        }
        throw std::runtime_error("the fixture has no camera for that eye");
    }

    int64_t SFixture::cameraHostNs(int eye, size_t index) const {
        const auto& CAMERA = camera(eye);
        return bundle.clock.hostFromDevice(CAMERA.deviceNs.at(index) + CAMERA.exposureNs / 2);
    }

    int64_t SFixture::outputHostNs(size_t k, double fps) const {
        return bundle.firstHostNs() + static_cast<int64_t>(std::llround(static_cast<double>(k) * 1e9 / fps));
    }

    size_t SFixture::outputRecord(size_t k, double fps) const {
        return nearestIndex(bundle.telemetryHostNs, outputHostNs(k, fps)).value();
    }

    size_t SFixture::overlaySourceRecord(size_t k, double fps) const {
        // The ordinal rule from the outside: the video's frames are the undropped
        // records in order, so the frame nearest an output instant is found among
        // *those* records' times, and the record it names is the pose the frame was
        // rendered from.
        std::vector<int64_t> times;
        times.reserve(scene.overlayFrames.size());
        for (int record : scene.overlayFrames)
            times.push_back(scene.frameHostNs(record));
        const size_t ORDINAL = nearestIndex(times, outputHostNs(k, fps)).value();
        return static_cast<size_t>(scene.overlayFrames[ORDINAL]);
    }

    SPose SFixture::cameraExtrinsic(int eye, eBackgroundAlign align) const {
        SPose extrinsic = camera(eye).headToCamera;
        if (align == eBackgroundAlign::AUTO)
            extrinsic.rot = twistAbout(extrinsic.rot, {0.0, 0.0, 1.0});
        return extrinsic;
    }

    SPose SFixture::outputCamera(int frame, int eye, eFrustumMode mode) const {
        SPose camera = eyePose(frame, eye);
        if (mode == eFrustumMode::PRESENTATION)
            camera.rot = headPose(frame).rot;
        return camera;
    }

    namespace {

        SFixture buildFixture(const std::string& name, double clockOffsetMs, const std::string& alpha = "premultiplied", const std::optional<SFov>& eyeFov = std::nullopt,
                              bool geometryMarkers = false, int cameraActiveArrayPad = 0, double headSpeed = 1.0, double cameraHz = 0.0, bool legacyMirroredExtrinsics = false) {
            SFixture fixture;
            fixture.take = scratchRoot() / (name + ".hypxrtake");

            fixture.options              = {};
            fixture.options.out          = fixture.take;
            // A 90 Hz session with the overlay at 45 and the cameras at 30: no two
            // of the three rates line up, so no source-selection bug can hide behind
            // an accidental 1:1 index map, and the overlay's `dropped` records are
            // exercised on every run.
            fixture.options.frames       = 90;
            fixture.options.hz           = 90.0;
            fixture.options.overlayHz    = 45.0;
            // 200 ms at 30 camera Hz is exactly six camera frames, which makes the
            // "what would a compositor that ignored the clock have picked?"
            // assertion an exact integer rather than a rounding argument.
            fixture.options.clockOffsetMs = clockOffsetMs;
            fixture.options.alpha         = alpha;
            fixture.options.quiet         = true;
            if (eyeFov)
                fixture.options.eyeFov = *eyeFov;
            fixture.options.geometryMarkers      = geometryMarkers;
            fixture.options.cameraActiveArrayPad = cameraActiveArrayPad;
            fixture.options.headSpeed            = headSpeed;
            if (cameraHz > 0.0)
                fixture.options.cameraHz = cameraHz;
            fixture.options.legacyMirroredExtrinsics = legacyMirroredExtrinsics;

            if (!fs::exists(fixture.take / "manifest.json")) {
                setLogLevel(eLogLevel::WARN);
                if (runSynth(fixture.options) != 0)
                    throw std::runtime_error("the synthetic bundle could not be generated");
                setLogLevel(eLogLevel::INFO);
            }

            std::string error;
            const auto  SCENE = SSynthScene::load(fixture.take, error);
            if (!SCENE)
                throw std::runtime_error("ground truth unreadable: " + error);
            fixture.scene = *SCENE;

            CDiagnostics diags;
            const auto   BUNDLE = SBundle::load(fixture.take, diags, {});
            if (!BUNDLE || diags.hasErrors()) {
                diags.print();
                throw std::runtime_error("the synthetic bundle does not load");
            }
            fixture.bundle = *BUNDLE;
            return fixture;
        }

    }

    const SFixture& fixture() {
        static const SFixture FIXTURE = buildFixture("main", 200.0);
        return FIXTURE;
    }

    const SFixture& zeroOffsetFixture() {
        static const SFixture FIXTURE = buildFixture("zero-offset", 0.0);
        return FIXTURE;
    }

    const SFixture& straightAlphaFixture() {
        static const SFixture FIXTURE = buildFixture("straight-alpha", 200.0, "straight");
        return FIXTURE;
    }

    const SFixture& briskMotionFixture() {
        // A head moving ten times as fast against a 5 Hz camera, so a frame's
        // capture instant and the output instant it is composited at are far
        // enough apart for the two reprojection models to disagree visibly. At
        // rest they agree, and a test that cannot tell them apart is not a test.
        static const SFixture FIXTURE = buildFixture("brisk-motion", 200.0, "premultiplied", std::nullopt, false, 0, 10.0, 5.0);
        return FIXTURE;
    }

    const SFixture& mirroredExtrinsicsFixture() {
        // Stores head_to_camera the way the buggy producer stored it - cant
        // mirrored - while carrying a correct raw pose beside it, which is
        // exactly the shape every take recorded so far has.
        static const SFixture FIXTURE = buildFixture("mirrored-extrinsics", 200.0, "premultiplied", std::nullopt, false, 0, 1.0, 0.0, true);
        return FIXTURE;
    }

    const SFixture& sensorArrayFixture() {
        // 120 px of pad top and bottom, so a loader that ignores the active array
        // puts the principal point 120 px low - the same class of error, and the
        // same direction, as the 160 px the first real take carried.
        static const SFixture FIXTURE = buildFixture("sensor-array", 200.0, "premultiplied", std::nullopt, false, 120);
        return FIXTURE;
    }

    const SFixture& realFrustumFixture() {
        // The frustum the reference take actually recorded, to four places:
        // strongly asymmetric (the optical axis sits at 62.1% of the width, not
        // at half) and with an angular aspect of 0.9255 that matches no pane
        // anybody would ask for. The default fixture cannot catch a framing bug
        // because its angular aspect, 1.3349, is within 0.12% of the 4:3 pane the
        // tests render into - so every stretch cancelled and every test passed.
        static const SFixture FIXTURE = buildFixture("real-frustum", 200.0, "premultiplied", SFov{-0.9425, 0.6981, 0.7679, -0.9599}, true);
        return FIXTURE;
    }

    std::optional<std::array<double, 2>> predictBackgroundPixel(const SPose& outputCamera, const SFov& outputFov, int paneWidth, int paneHeight, const SPose& cameraPose, const SVec3& world,
                                                                double assumedDepth) {
        const SVec3  DIRECTION = (world - cameraPose.pos).normalized();
        const SVec3  OFFSET    = cameraPose.pos - outputCamera.pos;
        const double ALONG     = OFFSET.dot(DIRECTION);
        const double RADICAND  = ALONG * ALONG - OFFSET.dot(OFFSET) + assumedDepth * assumedDepth;
        if (RADICAND < 0.0)
            return std::nullopt;

        const double TRAVEL = -ALONG + std::sqrt(RADICAND);
        if (!(TRAVEL > 0.0))
            return std::nullopt;

        const SVec3 POINT = cameraPose.pos + DIRECTION * TRAVEL;
        double      px = 0.0, py = 0.0;
        if (!fovProject(outputFov, outputCamera.dirToLocal(POINT - outputCamera.pos), paneWidth, paneHeight, px, py))
            return std::nullopt;
        return std::array<double, 2>{px, py};
    }

    std::optional<std::array<double, 2>> predictOverlayPixel(const SPose& outputCamera, const SFov& outputFov, int paneWidth, int paneHeight, const SPose& recordingEye, const SVec3& world) {
        const SVec3 DIRECTION = world - recordingEye.pos;
        double      px = 0.0, py = 0.0;
        if (!fovProject(outputFov, outputCamera.dirToLocal(DIRECTION), paneWidth, paneHeight, px, py))
            return std::nullopt;
        return std::array<double, 2>{px, py};
    }

}
