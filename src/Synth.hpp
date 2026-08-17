#pragma once

// `hypxrcompose synth <out-take>` — a complete, self-consistent `.hypxrtake`
// bundle with no hardware involved.
//
// This is the reason the compositor is testable at all. Every number in the
// bundle comes from one closed-form scene, so the correct output of `render` is
// *computable*, not merely eyeballable: a test can predict the exact pixel a
// known feature must land on and assert it.
//
// What the generator deliberately makes non-trivial, because each one is a place
// the pipeline can silently be wrong:
//
//   - a nonzero, drifting host<->device clock offset (default +250 ms, +20 ppm),
//     so any code path that forgets to map device time into host time selects
//     visibly wrong frames;
//   - camera extrinsics that are *not* the eye poses: a wider baseline than the
//     IPD, a forward offset, a downward drop, and an outward splay, so a
//     composite that ignores the extrinsic lands the background in the wrong
//     place;
//   - asymmetric, per-eye-mirrored fields of view, so a symmetric-frustum
//     assumption fails;
//   - a principal point off the image centre and real Brown-Conrady distortion,
//     so an undistorted sampler fails;
//   - camera frames on their own cadence with their own phase, so nearest-in-time
//     source selection is exercised rather than an accidental 1:1 index map;
//   - head motion with a deliberate high-frequency jitter component on top of a
//     slow sweep, so stabilization has something to remove and something to keep;
//   - audio clicks at known instants, one on each clock domain.
//
// The scene itself is a checkerboard wall at a known depth carrying uniquely
// coloured markers, plus a floating RGBA "monitor" quad with its own markers and
// a partially transparent halo. Because the wall is planar and its depth is
// known, a background reprojection at the matching assumed depth is exact, and
// any error in the pose chain, the extrinsics, the intrinsics, or the clock moves
// a marker measurably.
//
// A `synth/ground-truth.json` file inside the bundle records every constant the
// tests need. It is not part of the bundle contract; validate ignores unknown
// files, which is the property that lets it exist.

#include <cstdint>
#include <filesystem>
#include <string>

namespace hxc {

    struct SSynthOptions {
        std::filesystem::path out;

        int     frames        = 60;
        double  hz            = 60.0;
        double  overlayHz     = 0.0; // 0 means "same as hz"
        int     overlayWidth  = 640;
        int     overlayHeight = 480;

        bool    cameras       = true;
        int     cameraWidth   = 640;
        int     cameraHeight  = 480;
        double  cameraHz      = 30.0;
        int64_t exposureNs    = 8000000;

        bool    audio         = true;
        bool    mic           = true;

        // device = host + offset; the offset grows by driftPpm microseconds per
        // second, sampled every clockIntervalNs of host time.
        double  clockOffsetMs   = 250.0;
        double  clockDriftPpm   = 20.0;
        int64_t clockIntervalNs = 100000000;

        double  ipd             = 0.063;
        // Deliberately wider than the IPD: this is the parallax error research 27
        // section 5.1 accepts in v1, and making it real here means the test can
        // measure it.
        double  cameraBaseline  = 0.084;
        double  cameraForward   = 0.045;
        double  cameraDrop      = 0.012;
        double  cameraSplayDeg  = 2.0;

        double  wallZ           = -2.0;
        int64_t t0HostNs        = 14400LL * 1000000000LL; // a plausible CLOCK_MONOTONIC: four hours of uptime

        bool    quiet           = false;
    };

    // 0 on success. Overwrites `out` if it already exists.
    int runSynth(const SSynthOptions& options);

}
