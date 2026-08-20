#pragma once

// Shared machinery for the end-to-end tests: one synthetic bundle per process,
// plus the image measurements the assertions are built out of.

#include "Bundle.hpp"
#include "Math.hpp"
#include "SynthScene.hpp"
#include "Render.hpp"
#include "Synth.hpp"

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace hxctest {

    using namespace hxc;

    std::filesystem::path scratchRoot();

    struct SImage {
        int                  width  = 0;
        int                  height = 0;
        std::vector<uint8_t> rgba;

        std::array<int, 4>   at(int x, int y) const {
            const size_t INDEX = (static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)) * 4;
            return {rgba[INDEX], rgba[INDEX + 1], rgba[INDEX + 2], rgba[INDEX + 3]};
        }
    };

    bool loadImage(const std::filesystem::path& path, SImage& out, std::string& error);

    struct SCentroid {
        double x     = 0.0;
        double y     = 0.0;
        size_t count = 0;
    };

    // Centre of mass of the pixels within `tolerance` (sum of absolute channel
    // differences) of `color`, searched inside [x0, x1) of the image. The blur a
    // bilinear resample puts on a marker's edge is symmetric, so the centroid of
    // the surviving core is an unbiased estimate of the marker's centre.
    std::optional<SCentroid> findColor(const SImage& image, const std::array<int, 3>& color, int tolerance, int x0, int x1);

    // The bundle every end-to-end test shares. Synthesized once, lazily.
    struct SFixture {
        std::filesystem::path take;
        SSynthScene           scene;
        SSynthOptions         options;
        SBundle               bundle;

        // Eye pose at a telemetry frame, straight from the scene's motion model.
        SPose                 eyePose(int frame, int eye) const;
        // The camera the renderer composes that eye's pane from - which is not
        // the same thing. Under the default presentation frustum the panes are a
        // parallel rig: each eye keeps its own position, because that is what
        // carries the stereo parallax, but both look along the head's
        // orientation, so a feature at infinity lands identically in both. Under
        // `recorded` the eye's own pose is used unchanged.
        SPose                 outputCamera(int frame, int eye, eFrustumMode mode = eFrustumMode::PRESENTATION) const;
        SPose                 headPose(int frame) const;
        const SSynthCamera&   camera(int eye) const;
        // The extrinsic the renderer will actually compose with, which under the
        // default `--bg-align auto` is not the one the scene recorded: the swing
        // is dropped so the optical axis lands along the output's forward, and
        // only the roll survives.
        SPose                 cameraExtrinsic(int eye, eBackgroundAlign align = eBackgroundAlign::AUTO) const;
        // Where the generator put camera frame `index` on the host timeline.
        int64_t               cameraHostNs(int eye, size_t index) const;

        // The output timeline, and what each output frame is made of - derived from
        // the ground truth rather than read out of the tool's own report.
        int64_t               outputHostNs(size_t k, double fps) const;
        size_t                outputRecord(size_t k, double fps) const;
        // The telemetry record whose pixels the overlay frame for output frame `k`
        // carries. Differs from outputRecord() wherever a record was dropped.
        size_t                overlaySourceRecord(size_t k, double fps) const;
    };

    const SFixture& fixture();
    // The same scene stored with unassociated alpha instead of premultiplied. Both
    // describe identical imagery, so both must compose to the same pixels.
    const SFixture& straightAlphaFixture();
    // A second bundle whose only difference is a zero host<->device clock offset;
    // used to show what the clock path is worth.
    const SFixture& zeroOffsetFixture();
    // A bundle whose eyes carry the reference take's real frustum: asymmetric,
    // and with an angular aspect that deliberately does not match the test pane.
    const SFixture& realFrustumFixture();
    // A bundle whose camera intrinsics are stated against a sensor active array
    // taller than the delivered image, the way Android reports them.
    const SFixture& sensorArrayFixture();
    // A take with brisk head motion and a slow camera: the only conditions under
    // which "reprojected from capture time" and "reprojected from output time"
    // give visibly different answers.
    const SFixture& briskMotionFixture();
    // A take whose stored head_to_camera carries the producer's mirrored cant,
    // with a correct `extrinsics_android_raw` beside it.
    const SFixture& mirroredExtrinsicsFixture();
    // A take whose cameras see less than its eyes do, so that clipping the
    // output to camera coverage actually clips something.
    const SFixture& narrowCameraFixture();
    // A take whose left camera drops every seventh frame, so the two eyes'
    // capture series differ and a naive pairing would desynchronise them.
    const SFixture& droppedCameraFrameFixture();
    // Brisk head motion with the real rig's 30 Hz camera against a 45 Hz
    // overlay: the conditions under which the two output clocks differ.
    const SFixture& steadyCameraFixture();

    // The v1 background model, evaluated exactly: where must the output pixel be
    // for its assumed-depth ray to sample the camera pixel that shows `world`?
    //
    // The camera's ray through the marker is (camera position -> world); the
    // assumed-depth sphere around the output camera cuts that ray at one point in
    // front of the lens, and that point is what the output pixel is looking at.
    // The distortion model cancels out of this derivation, which is the point: the
    // prediction tests the pose chain and the depth model without borrowing the
    // compositor's projection code.
    std::optional<std::array<double, 2>> predictBackgroundPixel(const SPose& outputCamera, const SFov& outputFov, int paneWidth, int paneHeight, const SPose& cameraPose, const SVec3& world,
                                                                double assumedDepth);

    // The v1 overlay model with an infinite assumed depth: the output pixel whose
    // direction matches the direction `world` had from the recording eye.
    std::optional<std::array<double, 2>> predictOverlayPixel(const SPose& outputCamera, const SFov& outputFov, int paneWidth, int paneHeight, const SPose& recordingEye, const SVec3& world);

}
