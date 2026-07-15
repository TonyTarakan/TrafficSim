#include <gtest/gtest.h>

#include "core/idm.hpp"

using namespace ts::idm;

// clang-format off
constexpr VehicleParams kTestDefault{
    .desired_speed = 13.9f, 
    .max_accel = 2.0f, 
    .comfy_decel = 3.0f,
    .min_gap = 2.0f,
    .time_headway = 1.5f
};
// clang-format on

TEST(Idm, StationaryVehicleAccelerates)
{
    VehicleParams p = kTestDefault;
    float a = acceleration(p, 0.f);
    EXPECT_GT(a, 0.f);
    EXPECT_LE(a, p.max_accel + 0.01f);
}

TEST(Idm, AtDesiredSpeedAccelIsNearZero)
{
    VehicleParams p;
    float a = acceleration(p, p.desired_speed);
    EXPECT_NEAR(a, 0.f, 0.05f);
}

TEST(Idm, AboveDesiredSpeedDecelerates)
{
    VehicleParams p;
    float a = acceleration(p, p.desired_speed * 1.2f);
    EXPECT_LT(a, 0.f);
}

TEST(Idm, LargeGapBehavesLikeFreeFlow)
{
    VehicleParams p;
    float a_free = acceleration(p, 10.f);
    float a_leader = acceleration(p, 10.f, LeaderInfo{.gap = 500.f, .dv = 0.f});
    EXPECT_NEAR(a_free, a_leader, 0.01f);
}

TEST(Idm, SmallGapCausesHardBraking)
{
    VehicleParams p;
    float a = acceleration(p, 10.f, LeaderInfo{.gap = 1.f, .dv = 5.f});
    EXPECT_LT(a, -1.f);
}

TEST(Idm, ApproachingFasterBrakesHarder)
{
    VehicleParams p;
    float a_fast = acceleration(p, 10.f, LeaderInfo{.gap = 10.f, .dv = 8.f});
    float a_slow = acceleration(p, 10.f, LeaderInfo{.gap = 10.f, .dv = 2.f});
    EXPECT_LT(a_fast, a_slow);
}
