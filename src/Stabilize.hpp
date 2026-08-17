#pragma once

// The `--framing stabilized` filter.
//
// Composition is offline, so the filter has the whole pose track in hand and
// there is no reason to accept the lag a causal filter (one-euro, EMA) imposes.
// The filter is therefore a **zero-phase Gaussian**: each output pose is the
// weighted mean of its neighbours with weights exp(-dt^2 / 2 sigma^2), truncated
// at three sigma, with the track's ends clamped (samples past the end repeat the
// endpoint) so the first and last frames are smoothed with a one-sided kernel
// instead of drifting.
//
// Weighting is by *time*, not by sample index, so an irregular telemetry cadence
// (dropped frames, a 45 Hz overlay against a 90 Hz session) does not silently
// change the cutoff.
//
// Positions average componentwise. Rotations average in the tangent space of a
// reference rotation — the kernel-centre sample — via log/exp, then renormalize.
// That is exact for a single rotation and correct to second order in the angular
// spread, which for head motion inside a 200 ms window is a few degrees. It is
// notably better behaved than componentwise quaternion averaging, which is only
// valid for tiny spreads and needs hemisphere fixing to avoid flipping.
//
// The cutoff is a -3 dB frequency of roughly 0.187 / sigma Hz: the 200 ms default
// leaves head motion below about 0.9 Hz intact and removes the high-frequency
// jitter that makes filmed VR unwatchable.

#include "Math.hpp"

#include <cstdint>
#include <vector>

namespace hxc {

    struct STimedPose {
        int64_t tNs = 0;
        SPose   pose;
    };

    // `sigmaNs` <= 0 returns the input untouched.
    std::vector<SPose> gaussianSmoothPoses(const std::vector<STimedPose>& track, int64_t sigmaNs);

    // The -3 dB cutoff of the above, for the log line that tells the user what the
    // filter actually did.
    double gaussianCutoffHz(int64_t sigmaNs);

}
