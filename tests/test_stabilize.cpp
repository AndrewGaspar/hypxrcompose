// The stabilization filter, measured as a filter: what it removes, what it keeps,
// and that it does both without lag.

#include "Stabilize.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace hxc;

namespace {

    constexpr int64_t MS = 1000000;

    // A yaw track: a slow sweep of `slowHz` plus jitter at `fastHz`.
    std::vector<STimedPose> yawTrack(double slowHz, double slowAmplitude, double fastHz, double fastAmplitude, int samples, int64_t intervalNs) {
        std::vector<STimedPose> track;
        track.reserve(static_cast<size_t>(samples));
        for (int i = 0; i < samples; ++i) {
            const int64_t T       = static_cast<int64_t>(i) * intervalNs;
            const double  SECONDS = static_cast<double>(T) * 1e-9;
            const double  YAW     = slowAmplitude * std::sin(2.0 * M_PI * slowHz * SECONDS) + fastAmplitude * std::sin(2.0 * M_PI * fastHz * SECONDS);
            track.push_back({T, SPose{{0.0, 0.0, 0.0}, SQuat::fromAxisAngle({0, 1, 0}, YAW)}});
        }
        return track;
    }

    double yawOf(const SPose& pose) {
        const SVec3 FORWARD = pose.rot.rotate({0, 0, -1});
        return std::atan2(-FORWARD.x, -FORWARD.z);
    }

    struct SComponent {
        double amplitude = 0.0;
        double phase     = 0.0;
    };

    // Amplitude and phase of the component at `hz`, measured over [from, to) by
    // direct correlation. The window must hold a whole number of cycles and must
    // stay clear of the smoother's clamped ends, where the one-sided kernel spreads
    // energy across the spectrum.
    SComponent analyze(const std::vector<double>& values, double hz, double sampleRate, size_t from, size_t to) {
        double real = 0.0, imaginary = 0.0;
        for (size_t i = from; i < to; ++i) {
            const double T = static_cast<double>(i) / sampleRate;
            real += values[i] * std::cos(2.0 * M_PI * hz * T);
            imaginary += values[i] * std::sin(2.0 * M_PI * hz * T);
        }
        const double COUNT = static_cast<double>(to - from);
        return {2.0 * std::hypot(real, imaginary) / COUNT, std::atan2(imaginary, real)};
    }

    // Eight seconds at 100 Hz: long enough that a window from 2 s to 6 s is clear
    // of both clamped ends and holds whole cycles of both test frequencies.
    constexpr int    SAMPLES     = 800;
    constexpr double SAMPLE_RATE = 100.0;
    constexpr size_t WINDOW_FROM = 200;
    constexpr size_t WINDOW_TO   = 600;

}

TEST(Stabilize, ASigmaOfZeroIsTheIdentity) {
    const auto TRACK    = yawTrack(0.5, 0.2, 6.0, 0.05, 64, 10 * MS);
    const auto SMOOTHED = gaussianSmoothPoses(TRACK, 0);
    ASSERT_EQ(SMOOTHED.size(), TRACK.size());
    for (size_t i = 0; i < TRACK.size(); ++i)
        EXPECT_NEAR(yawOf(SMOOTHED[i]), yawOf(TRACK[i].pose), 1e-15);
}

TEST(Stabilize, JitterIsRemovedAndTheSlowSweepSurvives) {
    const double SLOW_HZ = 0.5, SLOW_AMPLITUDE = 0.20;
    const double FAST_HZ = 6.0, FAST_AMPLITUDE = 0.05;
    const auto   TRACK    = yawTrack(SLOW_HZ, SLOW_AMPLITUDE, FAST_HZ, FAST_AMPLITUDE, SAMPLES, 10 * MS);
    const auto   SMOOTHED = gaussianSmoothPoses(TRACK, 200 * MS);

    std::vector<double> before, after;
    for (size_t i = 0; i < TRACK.size(); ++i) {
        before.push_back(yawOf(TRACK[i].pose));
        after.push_back(yawOf(SMOOTHED[i]));
    }

    // The response of a Gaussian of sigma s at frequency f is exp(-2 pi^2 s^2 f^2):
    // 0.905 at 0.5 Hz with sigma 200 ms, and utterly negligible at 6 Hz.
    const double SLOW_GAIN = std::exp(-2.0 * M_PI * M_PI * 0.04 * SLOW_HZ * SLOW_HZ);
    EXPECT_NEAR(analyze(before, SLOW_HZ, SAMPLE_RATE, WINDOW_FROM, WINDOW_TO).amplitude, SLOW_AMPLITUDE, 0.01);
    EXPECT_NEAR(analyze(after, SLOW_HZ, SAMPLE_RATE, WINDOW_FROM, WINDOW_TO).amplitude, SLOW_AMPLITUDE * SLOW_GAIN, 0.01);

    // The jitter must be gone by orders of magnitude, not merely reduced.
    EXPECT_NEAR(analyze(before, FAST_HZ, SAMPLE_RATE, WINDOW_FROM, WINDOW_TO).amplitude, FAST_AMPLITUDE, 0.01);
    EXPECT_LT(analyze(after, FAST_HZ, SAMPLE_RATE, WINDOW_FROM, WINDOW_TO).amplitude, FAST_AMPLITUDE * 0.005);
}

TEST(Stabilize, HasNoPhaseLagWhichIsTheWholeReasonItIsNotOneEuro) {
    // A causal filter delays what it passes. Measure the phase of the surviving
    // sweep before and after: a zero-phase filter must not move it. For scale, a
    // one-euro or EMA filter with the same cutoff would show roughly a radian here.
    const double SLOW_HZ  = 0.5;
    const auto   TRACK    = yawTrack(SLOW_HZ, 0.2, 6.0, 0.05, SAMPLES, 10 * MS);
    const auto   SMOOTHED = gaussianSmoothPoses(TRACK, 200 * MS);

    std::vector<double> before, after, causal;
    double              state = 0.0;
    // A first-order low pass with the same -3 dB point, for comparison.
    const double        ALPHA = 1.0 - std::exp(-2.0 * M_PI * gaussianCutoffHz(200 * MS) / SAMPLE_RATE);
    for (size_t i = 0; i < TRACK.size(); ++i) {
        const double RAW = yawOf(TRACK[i].pose);
        before.push_back(RAW);
        after.push_back(yawOf(SMOOTHED[i]));
        state += ALPHA * (RAW - state);
        causal.push_back(state);
    }

    const double PHASE_BEFORE = analyze(before, SLOW_HZ, SAMPLE_RATE, WINDOW_FROM, WINDOW_TO).phase;
    const double PHASE_AFTER  = analyze(after, SLOW_HZ, SAMPLE_RATE, WINDOW_FROM, WINDOW_TO).phase;
    const double PHASE_CAUSAL = analyze(causal, SLOW_HZ, SAMPLE_RATE, WINDOW_FROM, WINDOW_TO).phase;

    EXPECT_NEAR(PHASE_AFTER, PHASE_BEFORE, 0.01) << "the Gaussian must be phase-neutral";
    EXPECT_GT(std::abs(PHASE_CAUSAL - PHASE_BEFORE), 0.3) << "the comparison filter must actually lag, or this test proves nothing";
}

TEST(Stabilize, PositionsAveragedAndEndpointsClamped) {
    std::vector<STimedPose> track;
    for (int i = 0; i < 40; ++i)
        track.push_back({static_cast<int64_t>(i) * 10 * MS, SPose{{static_cast<double>(i), 0.0, 0.0}, SQuat::identity()}});

    const auto SMOOTHED = gaussianSmoothPoses(track, 50 * MS);

    // A ramp is its own smoothed self in the interior: the kernel is symmetric.
    EXPECT_NEAR(SMOOTHED[20].pos.x, 20.0, 1e-9);
    // At the ends the clamped tail holds the endpoint value, which pulls the
    // smoothed track toward it rather than letting the mean drift inward.
    EXPECT_LT(SMOOTHED.front().pos.x, 2.0);
    EXPECT_GT(SMOOTHED.back().pos.x, 37.0);
}

TEST(Stabilize, IrregularSamplingDoesNotChangeTheCutoff) {
    // Same signal, half the samples in the second half of the track. Weighting by
    // time rather than by index is what keeps the result the same.
    std::vector<STimedPose> dense, sparse;
    for (int i = 0; i < 200; ++i) {
        const int64_t T   = static_cast<int64_t>(i) * 10 * MS;
        const double  YAW = 0.2 * std::sin(2.0 * M_PI * 0.5 * static_cast<double>(T) * 1e-9);
        const SPose   POSE{{0, 0, 0}, SQuat::fromAxisAngle({0, 1, 0}, YAW)};
        dense.push_back({T, POSE});
        if (i < 100 || i % 2 == 0)
            sparse.push_back({T, POSE});
    }

    const auto SMOOTH_DENSE  = gaussianSmoothPoses(dense, 200 * MS);
    const auto SMOOTH_SPARSE = gaussianSmoothPoses(sparse, 200 * MS);

    // Compare at a common instant well inside the sparse region.
    const int64_t WANTED = 150 * 10 * MS;
    size_t        denseIndex = 0, sparseIndex = 0;
    while (dense[denseIndex].tNs < WANTED)
        ++denseIndex;
    while (sparse[sparseIndex].tNs < WANTED)
        ++sparseIndex;

    EXPECT_NEAR(yawOf(SMOOTH_DENSE[denseIndex]), yawOf(SMOOTH_SPARSE[sparseIndex]), 2e-3);
}

TEST(Stabilize, CutoffMatchesTheDocumentedFormula) {
    EXPECT_NEAR(gaussianCutoffHz(200 * MS), 0.9374, 1e-3);
    EXPECT_NEAR(gaussianCutoffHz(100 * MS), 1.8748, 1e-3);
    EXPECT_EQ(gaussianCutoffHz(0), 0.0);
}
