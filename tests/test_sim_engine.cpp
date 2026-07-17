#include <gtest/gtest.h>

#include "core/sim_engine.hpp"
#include "core/vehicle_params.hpp"

using namespace ts;

TEST(SimEngine, EmptyWorldDoesNotCrash)
{
    SimEngine engine;
    engine.tick();
    engine.tick();
    EXPECT_GT(engine.sim_time(), 0.0);
}

TEST(SimEngine, SingleVehicleAcceleratesTowardDesiredSpeed)
{
    SimEngine engine;

    Vehicle v{.id = 0,
              .type = VehicleType::Car,
              .idm_params = default_params(VehicleType::Car),
              .speed = 0.f,
              .lane_id = 0,
              .offset = 0.f};
    engine.vehicles().push_back(v);

    for (int i = 0; i < 500; ++i) {  // 10s at 50Hz
        engine.tick();
    }

    const Vehicle& result = engine.vehicles()[0];
    EXPECT_GT(result.speed, 0.f);
    EXPECT_LE(result.speed, v.idm_params.desired_speed + 0.01f);
    EXPECT_GT(result.offset, 0.f);
}

TEST(SimEngine, FollowerNeverPassesSlowerLeader)
{
    SimEngine engine;

    Vehicle leader{.id = 0,
                   .idm_params = default_params(VehicleType::Car),
                   .speed = 5.f,  // running away from a faster follower
                   .lane_id = 0,
                   .offset = 20.f};

    Vehicle follower{.id = 1,
                     .idm_params = default_params(VehicleType::Car),
                     .speed = 15.f,  // approaching a slower leader
                     .lane_id = 0,
                     .offset = 0.f};

    engine.vehicles().push_back(leader);
    engine.vehicles().push_back(follower);

    for (int i = 0; i < 1000; ++i) {  // 20s at 50Hz
        engine.tick();
        const Vehicle& l = engine.vehicles()[0];
        const Vehicle& f = engine.vehicles()[1];
        ASSERT_LT(f.offset, l.offset) << "follower passed leader at tick " << i;
    }
}

TEST(SimEngine, FollowerMatchesGenuinelySlowerLeaderAtSteadyState)
{
    // The leader here has a lower desired_speed, not just a temporarily low
    // current speed — so it stays capped indefinitely instead of also
    // accelerating toward the default 13.9 m/s. That makes this a real
    // steady-state car-following scenario, not a two-vehicle drag race.
    SimEngine engine;

    Vehicle leader{.id = 0,
                   .idm_params = default_params(VehicleType::Car),  // default desired_speed = 15 m/s
                   .speed = 8.f,
                   .lane_id = 0,
                   .offset = 30.f};
    leader.idm_params.desired_speed = 8.f;

    Vehicle follower{.id = 1,
                     .idm_params = default_params(VehicleType::Car),  // default desired_speed = 15 m/s
                     .speed = 8.f,
                     .lane_id = 0,
                     .offset = 0.f};

    engine.vehicles().push_back(leader);
    engine.vehicles().push_back(follower);

    for (int i = 0; i < 1500; ++i) {  // 30s at 50Hz
        engine.tick();
    }

    EXPECT_NEAR(engine.vehicles()[0].speed, 8.f, 0.1f);
    EXPECT_NEAR(engine.vehicles()[1].speed, 8.f, 0.1f);
}

TEST(SimEngine, SimTimeAdvancesByFixedDt)
{
    SimConfig config;
    config.fixed_dt = 0.02f;  // 50Hz
    SimEngine engine(config);

    engine.tick();
    EXPECT_NEAR(engine.sim_time(), 0.02, 1e-6);

    engine.tick();
    EXPECT_NEAR(engine.sim_time(), 0.04, 1e-6);
}
