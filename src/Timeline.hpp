#pragma once

// One timeline for four sources.
//
// The bundle's clock.jsonl carries samples of the host<->device offset, stamped
// in *host* time, with the sign convention fixed by the contract:
//
//     t_device = t_host + offset
//
// so a device timestamp maps back with t_host = t_device - offset(t_host). The
// right-hand side needs the offset at the host time we are solving for, which is
// why hostFromDevice() iterates: the offset changes by microseconds per second at
// realistic drift, so one correction step already lands well inside a nanosecond
// and two are taken for free.
//
// Between samples the offset is piecewise-linear; outside the sampled span it is
// held constant at the nearest endpoint rather than extrapolated, because a
// linear extrapolation of a noisy offset estimate diverges and a held value is
// wrong by a bounded, understandable amount.

#include <cstdint>
#include <optional>
#include <vector>

namespace hxc {

    struct SClockSample {
        int64_t tHostNs  = 0;
        int64_t offsetNs = 0;
        double  rttUs    = 0.0;
    };

    class CClockMap {
      public:
        CClockMap() = default;
        // Samples are sorted by t_host_ns on construction; duplicates are kept and
        // resolved by taking the first, which keeps the map single-valued.
        explicit CClockMap(std::vector<SClockSample> samples);

        bool    empty() const {
            return m_samples.empty();
        }
        size_t  size() const {
            return m_samples.size();
        }
        int64_t firstHostNs() const;
        int64_t lastHostNs() const;

        // Offset at a host instant: piecewise-linear inside the span, held outside.
        // With no samples at all the offset is zero, which makes the device and host
        // clocks the same timeline — the correct degenerate behaviour for a
        // host-only take.
        int64_t offsetAtHost(int64_t tHostNs) const;

        int64_t deviceFromHost(int64_t tHostNs) const {
            return tHostNs + offsetAtHost(tHostNs);
        }
        int64_t hostFromDevice(int64_t tDeviceNs) const;

        // True when `tHostNs` sits inside the sampled span, so the offset used was
        // interpolated rather than held. Callers surface this as a warning.
        bool covers(int64_t tHostNs) const;

        const std::vector<SClockSample>& samples() const {
            return m_samples;
        }

      private:
        std::vector<SClockSample> m_samples;
    };

    // Index of the entry in a sorted, ascending list whose value is nearest to
    // `needle`. Returns nullopt only for an empty list. Ties go to the lower index.
    std::optional<size_t> nearestIndex(const std::vector<int64_t>& sorted, int64_t needle);

    // Index of the last entry at or before `needle`, or nullopt when `needle`
    // precedes the first entry.
    std::optional<size_t> lastAtOrBefore(const std::vector<int64_t>& sorted, int64_t needle);

}
