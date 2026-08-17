#include "Harness.hpp"
#include "Ffmpeg.hpp"
#include "Log.hpp"

#include <cmath>
#include <cstdlib>
#include <mutex>
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

    namespace {

        SFixture buildFixture(const std::string& name, double clockOffsetMs) {
            SFixture fixture;
            fixture.take = scratchRoot() / (name + ".hypxrtake");

            fixture.options              = {};
            fixture.options.out          = fixture.take;
            // 60 Hz telemetry against 30 Hz cameras, so the camera index is never
            // accidentally equal to the output frame index and a source-selection bug
            // cannot hide behind a 1:1 map.
            fixture.options.frames       = 60;
            fixture.options.hz           = 60.0;
            // 200 ms at 30 camera Hz is exactly six camera frames, which makes the
            // "what would a compositor that ignored the clock have picked?"
            // assertion an exact integer rather than a rounding argument.
            fixture.options.clockOffsetMs = clockOffsetMs;
            fixture.options.quiet         = true;

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
