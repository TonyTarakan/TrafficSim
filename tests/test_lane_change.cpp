#include <gtest/gtest.h>

#include "core/sim_engine.hpp"
#include "core/vehicle_params.hpp"

using namespace ts;

TEST(LaneChange, SwitchesAwayFromSlowBlockerWhenAdjacentLaneIsFree)
{
    SimEngine engine;
    engine.set_lanes({{.id = 0, .from = 0, .to = 1, .length = 500.f, .speed_limit = 20.f, .num_sublanes = 2}});
    std::uint8_t initial_sublane_idx = 0;

    Vehicle truck{.id = 0,
                  .type = VehicleType::Truck,
                  .idm_params = default_params(VehicleType::Truck),
                  .speed = 5.f,
                  .lane_id = 0,
                  .offset = 60.f,
                  .sublane_idx = initial_sublane_idx};
    truck.idm_params.desired_speed = 5.f;  // prevent acceleration

    Vehicle car{.id = 1,
                .idm_params = default_params(VehicleType::Car),
                .speed = 15.f,
                .lane_id = 0,
                .offset = 0.f,
                .sublane_idx = initial_sublane_idx};

    engine.vehicles().push_back(truck);
    engine.vehicles().push_back(car);

    bool switched = false;
    for (int i = 0; i < 500; ++i) {
        engine.tick();
        if (engine.vehicles()[1].sublane_idx != initial_sublane_idx) {
            switched = true;
            break;
        }
    }
    EXPECT_TRUE(switched);
}

TEST(LaneChange, NeverSwitchesOnASingleLaneRoad)
{
    SimEngine engine;
    engine.set_lanes({{.id = 0, .from = 0, .to = 1, .length = 500.f, .speed_limit = 20.f, .num_sublanes = 1}});
    std::uint8_t initial_sublane_idx = 0;

    Vehicle v{.id = 0,
              .idm_params = default_params(VehicleType::Car),
              .speed = 10.f,
              .lane_id = 0,
              .offset = 0.f,
              .sublane_idx = initial_sublane_idx};

    engine.vehicles().push_back(v);

    for (int i = 0; i < 500; ++i) {
        engine.tick();
        ASSERT_EQ(engine.vehicles()[0].sublane_idx, initial_sublane_idx);
    }
}

TEST(LaneChange, StaysPutWhenThereIsNothingToGain)
{
    SimEngine engine;
    engine.set_lanes({{.id = 0, .from = 0, .to = 1, .length = 500.f, .speed_limit = 20.f, .num_sublanes = 2}});
    std::uint8_t initial_sublane_idx = 0;

    Vehicle v{.id = 0,
              .idm_params = default_params(VehicleType::Car),
              .speed = 10.f,
              .lane_id = 0,
              .offset = 0.f,
              .sublane_idx = initial_sublane_idx};

    engine.vehicles().push_back(v);  // alone on the road

    for (int i = 0; i < 500; ++i) {
        engine.tick();
        ASSERT_EQ(engine.vehicles()[0].sublane_idx, initial_sublane_idx);
    }
}

TEST(LaneChange, SafetyCriterionBlocksDangerousMerge)
{
    // A vehicle closing in very fast on the target sublane, right at ego's
    // merge point, should prevent the switch even though the slow blocker
    // ahead would otherwise make it attractive.
    SimEngine engine;
    engine.set_lanes({{.id = 0, .from = 0, .to = 1, .length = 500.f, .speed_limit = 30.f, .num_sublanes = 2}});

    Vehicle slow_blocker{.id = 0,
                         .idm_params = default_params(VehicleType::Car),
                         .speed = 5.f,
                         .lane_id = 0,
                         .offset = 20.f,
                         .sublane_idx = 0};
    slow_blocker.idm_params.desired_speed = 5.f;

    Vehicle ego{.id = 1,
                .idm_params = default_params(VehicleType::Car),
                .speed = 15.f,
                .lane_id = 0,
                .offset = 0.f,
                .sublane_idx = 0};

    Vehicle fast_approacher{.id = 2,
                            .idm_params = default_params(VehicleType::Car),
                            .speed = 30.f,  // closing in fast
                            .lane_id = 0,
                            .offset = -0.5f,  // right beside ego's merge point
                            .sublane_idx = 1};
    fast_approacher.idm_params.desired_speed = 30.f;

    engine.vehicles().push_back(slow_blocker);
    engine.vehicles().push_back(ego);
    engine.vehicles().push_back(fast_approacher);

    engine.tick();

    EXPECT_EQ(engine.vehicles()[1].sublane_idx, 0);
}
