#include "Timeline.hpp"

#include <algorithm>
#include <cmath>

namespace hxc {

    CClockMap::CClockMap(std::vector<SClockSample> samples) : m_samples(std::move(samples)) {
        std::stable_sort(m_samples.begin(), m_samples.end(), [](const SClockSample& a, const SClockSample& b) { return a.tHostNs < b.tHostNs; });
    }

    int64_t CClockMap::firstHostNs() const {
        return m_samples.empty() ? 0 : m_samples.front().tHostNs;
    }

    int64_t CClockMap::lastHostNs() const {
        return m_samples.empty() ? 0 : m_samples.back().tHostNs;
    }

    bool CClockMap::covers(int64_t tHostNs) const {
        return !m_samples.empty() && tHostNs >= firstHostNs() && tHostNs <= lastHostNs();
    }

    int64_t CClockMap::offsetAtHost(int64_t tHostNs) const {
        if (m_samples.empty())
            return 0;
        if (tHostNs <= m_samples.front().tHostNs)
            return m_samples.front().offsetNs;
        if (tHostNs >= m_samples.back().tHostNs)
            return m_samples.back().offsetNs;

        const auto IT = std::upper_bound(m_samples.begin(), m_samples.end(), tHostNs, [](int64_t needle, const SClockSample& s) { return needle < s.tHostNs; });
        const auto HI = IT;
        const auto LO = std::prev(IT);

        const int64_t SPAN = HI->tHostNs - LO->tHostNs;
        if (SPAN <= 0)
            return LO->offsetNs;

        const double T = static_cast<double>(tHostNs - LO->tHostNs) / static_cast<double>(SPAN);
        return LO->offsetNs + static_cast<int64_t>(std::llround(T * static_cast<double>(HI->offsetNs - LO->offsetNs)));
    }

    int64_t CClockMap::hostFromDevice(int64_t tDeviceNs) const {
        if (m_samples.empty())
            return tDeviceNs;

        // Seed with the offset sampled at the device instant read as if it were a
        // host instant, then iterate h <- device - offset(h). The map is a
        // contraction whenever the offset changes by less than a nanosecond per
        // nanosecond of host time, which any clock that does not run backwards
        // satisfies; at realistic drift it converges in one step, and the loop is
        // bounded so a pathological series cannot spin.
        int64_t host = tDeviceNs - offsetAtHost(tDeviceNs);
        for (int i = 0; i < 32; ++i) {
            const int64_t NEXT = tDeviceNs - offsetAtHost(host);
            if (NEXT == host)
                return host;
            host = NEXT;
        }
        return host;
    }

    std::optional<size_t> nearestIndex(const std::vector<int64_t>& sorted, int64_t needle) {
        if (sorted.empty())
            return std::nullopt;

        const auto IT = std::lower_bound(sorted.begin(), sorted.end(), needle);
        if (IT == sorted.begin())
            return size_t{0};
        if (IT == sorted.end())
            return sorted.size() - 1;

        const size_t  HI      = static_cast<size_t>(IT - sorted.begin());
        const size_t  LO      = HI - 1;
        const int64_t DIST_LO = needle - sorted[LO];
        const int64_t DIST_HI = sorted[HI] - needle;
        return DIST_HI < DIST_LO ? HI : LO;
    }

    std::optional<size_t> lastAtOrBefore(const std::vector<int64_t>& sorted, int64_t needle) {
        if (sorted.empty() || needle < sorted.front())
            return std::nullopt;
        const auto IT = std::upper_bound(sorted.begin(), sorted.end(), needle);
        return static_cast<size_t>(IT - sorted.begin()) - 1;
    }

}
