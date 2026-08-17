#include "Stabilize.hpp"

#include <algorithm>
#include <cmath>

namespace hxc {

    double gaussianCutoffHz(int64_t sigmaNs) {
        if (sigmaNs <= 0)
            return 0.0;
        // |H(f)| = exp(-2 pi^2 sigma^2 f^2); solving for |H| = 1/sqrt(2) gives
        // f = sqrt(ln 2 / 2) / (pi sigma).
        const double SIGMA_S = static_cast<double>(sigmaNs) * 1e-9;
        return std::sqrt(std::log(2.0) / 2.0) / (M_PI * SIGMA_S);
    }

    std::vector<SPose> gaussianSmoothPoses(const std::vector<STimedPose>& track, int64_t sigmaNs) {
        std::vector<SPose> out;
        out.reserve(track.size());
        if (track.empty())
            return out;

        if (sigmaNs <= 0) {
            for (const auto& S : track)
                out.push_back(S.pose);
            return out;
        }

        const double  SIGMA   = static_cast<double>(sigmaNs);
        const int64_t SUPPORT = 3 * sigmaNs;

        // The track is time-ordered, so the kernel window only ever moves forward.
        size_t lo = 0, hi = 0;
        for (size_t i = 0; i < track.size(); ++i) {
            const int64_t CENTRE = track[i].tNs;
            while (lo < track.size() && track[lo].tNs < CENTRE - SUPPORT)
                ++lo;
            while (hi < track.size() && track[hi].tNs <= CENTRE + SUPPORT)
                ++hi;

            // Clamped edges: weight that would have fallen off the start or end of
            // the track is handed to the endpoint sample instead of being dropped,
            // so the smoothed track keeps its endpoints rather than easing toward
            // the mean of a truncated window.
            double totalWeight = 0.0;
            SVec3  position{};
            SVec3  rotationSum{};
            const SQuat REFERENCE     = track[i].pose.rot.normalized();
            const SQuat REFERENCE_INV = REFERENCE.inverse();

            const auto accumulate = [&](const SPose& pose, double weight) {
                totalWeight += weight;
                position = position + pose.pos * weight;
                rotationSum = rotationSum + (REFERENCE_INV * pose.rot.normalized()).log() * weight;
            };

            double edgeLow = 0.0, edgeHigh = 0.0;
            for (size_t j = lo; j < hi; ++j) {
                const double DT = static_cast<double>(track[j].tNs - CENTRE) / SIGMA;
                accumulate(track[j].pose, std::exp(-0.5 * DT * DT));
            }

            // Approximate the truncated tails by integrating the kernel over the
            // missing span at the track's mean sample spacing.
            if (track.size() > 1) {
                const double SPACING = static_cast<double>(track.back().tNs - track.front().tNs) / static_cast<double>(track.size() - 1);
                if (SPACING > 0.0) {
                    for (int64_t k = 1;; ++k) {
                        const double DT = (static_cast<double>(track.front().tNs) - SPACING * static_cast<double>(k) - static_cast<double>(CENTRE)) / SIGMA;
                        if (DT < -3.0)
                            break;
                        edgeLow += std::exp(-0.5 * DT * DT);
                    }
                    for (int64_t k = 1;; ++k) {
                        const double DT = (static_cast<double>(track.back().tNs) + SPACING * static_cast<double>(k) - static_cast<double>(CENTRE)) / SIGMA;
                        if (DT > 3.0)
                            break;
                        edgeHigh += std::exp(-0.5 * DT * DT);
                    }
                }
            }
            if (edgeLow > 0.0)
                accumulate(track.front().pose, edgeLow);
            if (edgeHigh > 0.0)
                accumulate(track.back().pose, edgeHigh);

            if (!(totalWeight > 0.0)) {
                out.push_back(track[i].pose);
                continue;
            }

            SPose smoothed;
            smoothed.pos = position * (1.0 / totalWeight);
            smoothed.rot = (REFERENCE * SQuat::exp(rotationSum * (1.0 / totalWeight))).normalized();
            out.push_back(smoothed);
        }

        return out;
    }

}
