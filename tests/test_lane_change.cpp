#include <gtest/gtest.h>

#include "core/road_graph.hpp"
#include "core/sim_engine.hpp"
#include "core/types.hpp"
#include "core/vehicle_params.hpp"

using namespace ts;

TEST(LaneChange, SwitchesAwayFromSlowBlockerWhenAdjacentLaneIsFree)
{
    SimEngine engine;
    std::vector<Node> nodes = {{.id = NodeId{0}, .pos = {.x = 0.f, .y = 0.f}},
                               {.id = NodeId{1}, .pos = {.x = 500.f, .y = 0.f}}};
    std::vector<Edge> edges = {
        {.id = EdgeId{0}, .from = NodeId{0}, .to = NodeId{1}, .length = 500.f, .speed_limit = 20.f, .lane_count = 2}};

    engine.set_map(nodes, edges);
    std::uint8_t initial_sublane_idx = 0;

    Vehicle truck{.id = VehicleId{0},
                  .type = VehicleType::Truck,
                  .idm_params = default_params(VehicleType::Truck),
                  .speed = 5.f,
                  .edge_id = EdgeId{0},
                  .offset = 60.f,
                  .sublane_idx = initial_sublane_idx};
    truck.idm_params.desired_speed = 5.f;  // prevent acceleration

    Vehicle car{.id = VehicleId{1},
                .idm_params = default_params(VehicleType::Car),
                .speed = 15.f,
                .edge_id = EdgeId{0},
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
    std::vector<Node> nodes = {{.id = NodeId{0}, .pos = {.x = 0.f, .y = 0.f}},
                               {.id = NodeId{1}, .pos = {.x = 500.f, .y = 0.f}}};
    std::vector<Edge> edges = {
        {.id = EdgeId{0}, .from = NodeId{0}, .to = NodeId{1}, .length = 500.f, .speed_limit = 20.f, .lane_count = 2}};
    engine.set_map(nodes, edges);
    std::uint8_t initial_sublane_idx = 0;

    Vehicle v{.id = VehicleId{0},
              .idm_params = default_params(VehicleType::Car),
              .speed = 10.f,
              .edge_id = EdgeId{0},
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
    std::vector<Node> nodes = {{.id = NodeId{0}, .pos = {.x = 0.f, .y = 0.f}},
                               {.id = NodeId{1}, .pos = {.x = 500.f, .y = 0.f}}};
    std::vector<Edge> edges = {
        {.id = EdgeId{0}, .from = NodeId{0}, .to = NodeId{1}, .length = 500.f, .speed_limit = 20.f, .lane_count = 2}};
    engine.set_map(nodes, edges);

    std::uint8_t initial_sublane_idx = 0;

    Vehicle v{.id = VehicleId{0},
              .idm_params = default_params(VehicleType::Car),
              .speed = 10.f,
              .edge_id = EdgeId{0},
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
    std::vector<Node> nodes = {{.id = NodeId{0}, .pos = {.x = 0.f, .y = 0.f}},
                               {.id = NodeId{1}, .pos = {.x = 500.f, .y = 0.f}}};
    std::vector<Edge> edges = {
        {.id = EdgeId{0}, .from = NodeId{0}, .to = NodeId{1}, .length = 500.f, .speed_limit = 20.f, .lane_count = 2}};
    engine.set_map(nodes, edges);

    Vehicle slow_blocker{.id = VehicleId{0},
                         .idm_params = default_params(VehicleType::Car),
                         .speed = 5.f,
                         .edge_id = EdgeId{0},
                         .offset = 20.f,
                         .sublane_idx = 0};
    slow_blocker.idm_params.desired_speed = 5.f;

    Vehicle ego{.id = VehicleId{1},
                .idm_params = default_params(VehicleType::Car),
                .speed = 15.f,
                .edge_id = EdgeId{0},
                .offset = 0.f,
                .sublane_idx = 0};

    Vehicle fast_approacher{.id = VehicleId{2},
                            .idm_params = default_params(VehicleType::Car),
                            .speed = 30.f,  // closing in fast
                            .edge_id = EdgeId{0},
                            .offset = -0.5f,  // right beside ego's merge point
                            .sublane_idx = 1};
    fast_approacher.idm_params.desired_speed = 30.f;

    engine.vehicles().push_back(slow_blocker);
    engine.vehicles().push_back(ego);
    engine.vehicles().push_back(fast_approacher);

    engine.tick();

    EXPECT_EQ(engine.vehicles()[1].sublane_idx, 0);
}