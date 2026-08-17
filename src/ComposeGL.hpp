#pragma once

// The v1 reprojection composite, on the GPU.
//
// Back end: EGL_MESA_platform_surfaceless plus an FBO — the same toolbox as the
// portal demo's offscreen renderer in the Hyprland tree, and for the same reason:
// no compositor, no window system, no display, so it runs in CI and over ssh.
//
// One draw per pane. A pane is one output eye; mono output has one, stereo SBS
// has two side by side in a single render target, so a stereo frame is one
// readback and one pipe write rather than two.
//
// The kernel is an inverse warp: for each *output* pixel, build the ray the
// output camera looks along, place a point on it at the assumed depth, and
// project that point into each source's model — the camera's pinhole+distortion
// for the background, the stamped frustum for the overlay. Nothing is forward-
// mapped, so there are no seams or holes to fill. What an assumed depth costs is
// exactly the parallax error research 27 section 5.1 names, and it is a knob:
// an infinite depth degenerates to a pure rotation (no parallax handling at all,
// which is the right answer for a stabilized foreground), and a finite one is
// exact for content at that distance.

#include "Math.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hxc {

    enum class eBackgroundMode {
        SOLID = 0,   // uSolid everywhere
        CHECKER = 1, // a world-locked lat/long checker: no camera, but the framing is legible
        CAMERA = 2,  // the recorded passthrough video, reprojected
    };

    struct SPaneDraw {
        // The camera this pane renders from. In `asis` framing this is the recorded
        // eye pose and fov; in `stabilized` it is the smoothed pose with the eye's
        // own offset re-applied.
        SPose           outputCamera;
        SFov            outputFov;

        eBackgroundMode backgroundMode = eBackgroundMode::CHECKER;
        std::array<float, 4> solidColor = {0.02F, 0.03F, 0.04F, 1.0F};

        // Background camera, when backgroundMode == CAMERA.
        SPose             cameraPose;
        SCameraIntrinsics intrinsics;
        int               backgroundWidth  = 0;
        int               backgroundHeight = 0;
        double            backgroundDepth  = 2.0;

        // Overlay layer.
        bool   hasOverlay          = false;
        SPose  overlayPose;
        SFov   overlayFov;
        double overlayDepth        = 0.0; // <= 0 or non-finite means "infinite"
        bool   premultipliedAlpha  = false;
    };

    class CComposeGL {
      public:
        ~CComposeGL();
        CComposeGL(const CComposeGL&)            = delete;
        CComposeGL& operator=(const CComposeGL&) = delete;

        // `gpuHint` empty picks whatever EGL's default surfaceless display offers.
        // Otherwise devices are enumerated through EGL_EXT_platform_device and the
        // first whose renderer string, vendor string, or DRM node contains the hint
        // (case-insensitive) wins. The special hint "list" fails with the roster in
        // the error string.
        static std::unique_ptr<CComposeGL> create(int paneWidth, int paneHeight, int paneCount, const std::string& gpuHint, std::string& error);

        // Uploads are per pane and per layer so a source frame reused across
        // several output frames costs one upload, not one per output frame.
        bool uploadBackground(int pane, const uint8_t* rgba, int width, int height);
        bool uploadOverlay(int pane, const uint8_t* rgba, int width, int height);

        bool drawPane(int pane, const SPaneDraw& draw, std::string& error);

        // Copies the whole render target out as RGBA8, top row first.
        bool readback(std::vector<uint8_t>& rgba);

        int         width() const;
        int         height() const;
        std::string description() const;

      private:
        CComposeGL() = default;

        struct SState;
        std::unique_ptr<SState> m_state;
    };

}
