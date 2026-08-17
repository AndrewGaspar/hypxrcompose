// The clock map and the nearest-sample helpers: the pieces that decide *which*
// recorded frame a given output instant is made of.

#include "Timeline.hpp"

#include <gtest/gtest.h>

using namespace hxc;

namespace {
    constexpr int64_t MS = 1000000;

    CClockMap threeSamples() {
        return CClockMap({
            {100 * MS, 200 * MS, 1500.0},
            {200 * MS, 210 * MS, 1600.0},
            {300 * MS, 190 * MS, 1400.0},
        });
    }
}

TEST(ClockMap, AnEmptyMapMakesHostAndDeviceTheSameTimeline) {
    const CClockMap MAP;
    EXPECT_TRUE(MAP.empty());
    EXPECT_EQ(MAP.offsetAtHost(12345), 0);
    EXPECT_EQ(MAP.hostFromDevice(12345), 12345);
    EXPECT_EQ(MAP.deviceFromHost(12345), 12345);
}

TEST(ClockMap, OffsetIsExactOnTheSamples) {
    const auto MAP = threeSamples();
    EXPECT_EQ(MAP.offsetAtHost(100 * MS), 200 * MS);
    EXPECT_EQ(MAP.offsetAtHost(200 * MS), 210 * MS);
    EXPECT_EQ(MAP.offsetAtHost(300 * MS), 190 * MS);
}

TEST(ClockMap, OffsetIsLinearBetweenSamples) {
    const auto MAP = threeSamples();
    EXPECT_EQ(MAP.offsetAtHost(150 * MS), 205 * MS);
    EXPECT_EQ(MAP.offsetAtHost(250 * MS), 200 * MS);
    // A quarter of the way through the second span.
    EXPECT_EQ(MAP.offsetAtHost(225 * MS), 205 * MS);
}

TEST(ClockMap, OffsetIsHeldOutsideTheSampledSpanRatherThanExtrapolated) {
    const auto MAP = threeSamples();
    EXPECT_EQ(MAP.offsetAtHost(0), 200 * MS);
    EXPECT_EQ(MAP.offsetAtHost(-5000 * MS), 200 * MS);
    EXPECT_EQ(MAP.offsetAtHost(9999 * MS), 190 * MS);
    EXPECT_FALSE(MAP.covers(50 * MS));
    EXPECT_TRUE(MAP.covers(150 * MS));
}

TEST(ClockMap, DeviceFromHostAppliesTheContractSign) {
    const auto MAP = threeSamples();
    // device = host + offset.
    EXPECT_EQ(MAP.deviceFromHost(150 * MS), 150 * MS + 205 * MS);
}

TEST(ClockMap, HostFromDeviceInvertsDeviceFromHostExactly) {
    const auto MAP = threeSamples();
    for (int64_t host = 90 * MS; host <= 320 * MS; host += 3 * MS) {
        const int64_t DEVICE = MAP.deviceFromHost(host);
        EXPECT_EQ(MAP.hostFromDevice(DEVICE), host) << "at host " << host;
    }
}

TEST(ClockMap, HostFromDeviceHandlesASteeplyDriftingOffset) {
    // A pathological 1000 ppm drift: the seed offset is wrong by a millisecond,
    // which the correction steps must absorb.
    CClockMap map({
        {0, 100 * MS, 0.0},
        {1000 * MS, 101 * MS, 0.0},
        {2000 * MS, 102 * MS, 0.0},
    });
    for (int64_t host = 0; host <= 2000 * MS; host += 37 * MS)
        EXPECT_EQ(map.hostFromDevice(map.deviceFromHost(host)), host) << "at host " << host;
}

TEST(ClockMap, SamplesAreSortedOnConstruction) {
    const CClockMap MAP({
        {300 * MS, 190 * MS, 0.0},
        {100 * MS, 200 * MS, 0.0},
        {200 * MS, 210 * MS, 0.0},
    });
    EXPECT_EQ(MAP.firstHostNs(), 100 * MS);
    EXPECT_EQ(MAP.lastHostNs(), 300 * MS);
    EXPECT_EQ(MAP.offsetAtHost(150 * MS), 205 * MS);
}

TEST(NearestIndex, PicksTheClosestAndBreaksTiesLow) {
    const std::vector<int64_t> SORTED{0, 10, 20, 30};
    EXPECT_EQ(nearestIndex(SORTED, -5).value(), 0u);
    EXPECT_EQ(nearestIndex(SORTED, 4).value(), 0u);
    EXPECT_EQ(nearestIndex(SORTED, 5).value(), 0u); // tie
    EXPECT_EQ(nearestIndex(SORTED, 6).value(), 1u);
    EXPECT_EQ(nearestIndex(SORTED, 20).value(), 2u);
    EXPECT_EQ(nearestIndex(SORTED, 999).value(), 3u);
    EXPECT_FALSE(nearestIndex({}, 0).has_value());
}

TEST(LastAtOrBefore, IsHalfOpenOnTheLowSide) {
    const std::vector<int64_t> SORTED{0, 10, 20, 30};
    EXPECT_FALSE(lastAtOrBefore(SORTED, -1).has_value());
    EXPECT_EQ(lastAtOrBefore(SORTED, 0).value(), 0u);
    EXPECT_EQ(lastAtOrBefore(SORTED, 9).value(), 0u);
    EXPECT_EQ(lastAtOrBefore(SORTED, 10).value(), 1u);
    EXPECT_EQ(lastAtOrBefore(SORTED, 1000).value(), 3u);
}
