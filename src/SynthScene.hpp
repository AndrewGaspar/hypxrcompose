#pragma once

// The closed-form description of the synthetic scene, written into the bundle as
// `synth/ground-truth.json` and read back by the tests.
//
// Keeping this a real serialized structure rather than a set of constants shared
// through a header matters: the tests then predict pixel positions from what the
// bundle *says* the scene was, so a generator change that forgets to update the
// bundle fails the tests instead of silently moving both sides together.

#include "Math.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace hxc {

    struct SSynthWallMarker {
        std::string        name;
        std::array<int, 3> color{};
        SVec3              world;
        double             size = 0.1;
    };

    struct SSynthOverlayMarker {
        std::string        name;
        std::array<int, 3> color{};
        // Position on the quad, in metres from its centre, in quad-local axes.
        double             u    = 0.0;
        double             v    = 0.0;
        double             size = 0.05;
        // Unassociated alpha. A marker below 1 is what makes the composite prove it
        // blends in linear light: the same blend done in encoded space lands
        // dozens of levels away.
        double             alpha = 1.0;

        SVec3              world(const SPose& quad) const {
            return quad.pointToWorld({u, v, 0.0});
        }
    };

    struct SSynthCamera {
        std::string          key;
        int                  eye = 0;
        SPose                headToCamera;
        SCameraIntrinsics    intrinsics;
        int                  width  = 0;
        int                  height = 0;
        double               hz     = 30.0;
        int64_t              exposureNs = 0;
        // Sensor timestamps (start of exposure) in the device clock domain.
        std::vector<int64_t> deviceNs;
    };

    struct SSynthAudio {
        int64_t startNs      = 0; // host domain for `app`, device domain for `mic`
        int64_t clickHostNs  = 0;
        int64_t clickTrackNs = 0; // the click's stamp in the track's own domain
        int     sampleRate   = 48000;
        double  clickAmplitude = 0.0;
    };

    struct SSynthScene {
        int64_t t0HostNs = 0;
        double  hz       = 60.0;
        int     frames   = 60;
        double  overlayHz = 60.0;
        int     overlayWidth  = 0;
        int     overlayHeight = 0;

        // Telemetry record indices that have pixels in the overlay video, in order.
        // The n-th entry is the n-th video frame; every record not listed carries
        // "dropped": true. Decimation to overlayHz and simulated readback-queue
        // losses both land here, which is exactly how the real producer expresses
        // them.
        std::vector<int> overlayFrames;

        // The wall is a plane at z = wallZ spanning [-wallHalfX, wallHalfX] x
        // [-wallHalfY, wallHalfY]. Its planarity is what makes a reprojection at
        // backgroundDepth = -wallZ exact.
        double                        wallZ      = -2.0;
        double                        wallHalfX  = 3.0;
        double                        wallHalfY  = 2.0;
        double                        checkerCell = 0.25;
        std::vector<SSynthWallMarker> wallMarkers;

        // The frame-identity patch: its red channel encodes which camera frame the
        // compositor picked, so an error in the clock path is visible as a colour.
        SVec3 codeCentre;
        double codeSize     = 0.36;
        int    codeBase     = 8;
        int    codeStep     = 8;
        int    codeModulus  = 31;
        int    codeGreen    = 200;
        int    codeBlue     = 40;

        // How the overlay video stores its colour: "premultiplied" (the host
        // producer's default - colour multiplied by alpha in linear light, then
        // sRGB-encoded) or "straight".
        std::string                      overlayAlpha = "premultiplied";

        // The room-anchored "monitor" layer, in STAGE space. Composition index 0.
        SPose                            overlayQuad;
        double                           quadWidth  = 0.70;
        double                           quadHeight = 0.42;
        double                           quadHalo   = 0.04;
        std::vector<SSynthOverlayMarker> overlayMarkers;

        // The head-locked "HUD" layer, recorded head-relative and staying there.
        // Composition index 1, in front of the monitor.
        SPose                            hudQuad;
        double                           hudWidth  = 0.16;
        double                           hudHeight = 0.10;
        std::array<int, 3>               hudColor{255, 200, 0};
        double                           hudAlpha  = 0.75;

        // Multiplies the rate of the head's motion. 1.0 is a person sitting
        // still-ish; larger values are what a test needs when it has to tell
        // "the pose at capture" from "the pose at output" - at rest those two
        // are the same answer and prove nothing.
        double                 motionSpeed = 1.0;

        double                 ipd = 0.063;
        std::array<SFov, 2>    eyeFov{};

        std::vector<SSynthCamera> cameras;
        bool                      hasCameras = false;

        bool                      hasApp = false;
        SSynthAudio               app;
        bool                      hasMic = false;
        SSynthAudio               mic;

        // The head pose the generator used at a host instant. Reproduced exactly by
        // the tests so predictions do not depend on reading telemetry back.
        SPose                     headAt(int64_t tHostNs) const;

        int64_t                   frameHostNs(int frame) const;

        std::string               toJson() const;
        static std::optional<SSynthScene> fromJson(const std::string& text, std::string& error);
        static std::optional<SSynthScene> load(const std::filesystem::path& takeRoot, std::string& error);

        static constexpr const char* RELATIVE_PATH = "synth/ground-truth.json";
    };

}
