#include <gtest/gtest.h>

#include "core/junction.hpp"
#include "core/sim_engine.hpp"
#include "core/types.hpp"
#include "core/vehicle_params.hpp"

using namespace ts;

TEST(SimEngine, EmptyWorldDoesNotCrash)
{
    SimEngine engine;
    std::vector<Node> nodes = {{.id = NodeId{0}, .pos = {.x = 0, .y = 0}}, {.id = NodeId{1}, .pos = {.x = 10, .y = 0}}};
    std::vector<Edge> edges = {{.id = EdgeId{0}, .from = NodeId{0}, .to = NodeId{1}, .length = 10, .speed_limit = 10}};
    engine.set_map(nodes, edges);
    engine.tick();
    engine.tick();
    EXPECT_GT(engine.sim_time(), 0.0);
}

TEST(SimEngine, SingleVehicleAcceleratesTowardDesiredSpeed)
{
    SimEngine engine;
    std::vector<Node> nodes = {{.id = NodeId{0}, .pos = {.x = 0, .y = 0}}, {.id = NodeId{1}, .pos = {.x = 10, .y = 0}}};
    std::vector<Edge> edges = {{.id = EdgeId{0}, .from = NodeId{0}, .to = NodeId{1}, .length = 10, .speed_limit = 10}};
    engine.set_map(nodes, edges);

    Vehicle v{.id = VehicleId{0},
              .type = VehicleType::Car,
              .idm_params = default_params(VehicleType::Car),
              .speed = 0.f,
              .edge_id = EdgeId{0},
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
    std::vector<Node> nodes = {{.id = NodeId{0}, .pos = {.x = 0, .y = 0}}, {.id = NodeId{1}, .pos = {.x = 10, .y = 0}}};
    std::vector<Edge> edges = {{.id = EdgeId{0}, .from = NodeId{0}, .to = NodeId{1}, .length = 10, .speed_limit = 10}};
    engine.set_map(nodes, edges);

    Vehicle leader{.id = VehicleId{0},
                   .idm_params = default_params(VehicleType::Car),
                   .speed = 5.f,  // running away from a faster follower
                   .edge_id = EdgeId{0},
                   .offset = 20.f};

    Vehicle follower{.id = VehicleId{1},
                     .idm_params = default_params(VehicleType::Car),
                     .speed = 15.f,  // approaching a slower leader
                     .edge_id = EdgeId{0},
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
    std::vector<Node> nodes = {{.id = NodeId{0}, .pos = {.x = 0, .y = 0}}, {.id = NodeId{1}, .pos = {.x = 10, .y = 0}}};
    std::vector<Edge> edges = {{.id = EdgeId{0}, .from = NodeId{0}, .to = NodeId{1}, .length = 10, .speed_limit = 10}};
    engine.set_map(nodes, edges);

    Vehicle leader{.id = VehicleId{0},
                   .idm_params = default_params(VehicleType::Car),  // default desired_speed = 15 m/s
                   .speed = 8.f,
                   .edge_id = EdgeId{0},
                   .offset = 30.f};
    leader.idm_params.desired_speed = 8.f;

    Vehicle follower{.id = VehicleId{1},
                     .idm_params = default_params(VehicleType::Car),  // default desired_speed = 15 m/s
                     .speed = 8.f,
                     .edge_id = EdgeId{0},
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
    SimEngine engine;
    std::vector<Node> nodes = {{.id = NodeId{0}, .pos = {.x = 0, .y = 0}}, {.id = NodeId{1}, .pos = {.x = 10, .y = 0}}};
    std::vector<Edge> edges = {{.id = EdgeId{0}, .from = NodeId{0}, .to = NodeId{1}, .length = 10, .speed_limit = 10}};
    engine.set_map(nodes, edges);

    engine.tick();
    EXPECT_NEAR(engine.sim_time(), 0.02, 1e-6);

    engine.tick();
    EXPECT_NEAR(engine.sim_time(), 0.04, 1e-6);
}

TEST(SimEngine, VehicleStopsAtRedLightJunction)
{
    SimEngine engine;

    std::vector<Node> nodes = {
        {.id = NodeId{0}, .pos = {.x = 0.f, .y = 0.f}},
        {.id = NodeId{1}, .pos = {.x = 100.f, .y = 0.f}},  // junction node
        {.id = NodeId{2}, .pos = {.x = 200.f, .y = 0.f}},
    };
    std::vector<Edge> edges = {
        {.id = EdgeId{0}, .from = NodeId{0}, .to = NodeId{1}, .length = 100.f, .speed_limit = 20.f, .lane_count = 1},
        {.id = EdgeId{1}, .from = NodeId{1}, .to = NodeId{2}, .length = 100.f, .speed_limit = 20.f, .lane_count = 1},
    };
    engine.set_map(nodes, edges);

    // Lane 0's phase never comes up -- an always-red light for this approach.
    Junction junction{.node_id = NodeId{1},
                      .incoming = {EdgeId{0}},
                      .control = TrafficLightControl{.phases = {{.green_edges = {}, .duration = 1000.f}}}};
    engine.set_junctions({junction});

    Vehicle v{.id = VehicleId{0},
              .idm_params = default_params(VehicleType::Car),
              .speed = 15.f,
              .edge_id = EdgeId{0},
              .offset = 40.f};
    engine.vehicles().push_back(v);

    for (int i = 0; i < 1000; ++i) {  // 20s at 50Hz -- plenty of time to reach and stop at the line
        engine.tick();
        ASSERT_LE(engine.vehicles()[0].offset, edges[0].length) << "vehicle ran the red light at tick " << i;
    }

    EXPECT_NEAR(engine.vehicles()[0].speed, 0.f, 0.5f);
    const float expected_gap = default_params(VehicleType::Car).min_gap;
    float actual_gap = edges[0].length - engine.vehicles()[0].offset;
    EXPECT_NEAR(actual_gap, expected_gap, 3.f);
}

TEST(SimEngine, VehicleYieldsToPriorityCrossTraffic)
{
    SimEngine engine;

    std::vector<Node> nodes = {
        {.id = NodeId{0}, .pos = {.x = 0.f, .y = 0.f}},    // minor road start
        {.id = NodeId{1}, .pos = {.x = 50.f, .y = 0.f}},   // junction node
        {.id = NodeId{2}, .pos = {.x = 100.f, .y = 0.f}},  // minor road continues
        {.id = NodeId{3}, .pos = {.x = 0.f, .y = 50.f}},   // main road start
        {.id = NodeId{4}, .pos = {.x = 0.f, .y = -50.f}},  // main road continues
    };
    std::vector<Edge> edges = {
        {.id = EdgeId{0},
         .from = NodeId{0},
         .to = NodeId{1},
         .length = 50.f,
         .speed_limit = 20.f,
         .lane_count = 1},  // minor in
        {.id = EdgeId{1},
         .from = NodeId{1},
         .to = NodeId{2},
         .length = 50.f,
         .speed_limit = 20.f,
         .lane_count = 1},  // minor out
        {.id = EdgeId{2},
         .from = NodeId{3},
         .to = NodeId{1},
         .length = 50.f,
         .speed_limit = 20.f,
         .lane_count = 1},  // main in
        {.id = EdgeId{3},
         .from = NodeId{1},
         .to = NodeId{4},
         .length = 50.f,
         .speed_limit = 20.f,
         .lane_count = 1},  // main out
    };
    engine.set_map(nodes, edges);

    Junction junction{
        .node_id = NodeId{1},
        .incoming = {EdgeId{0}, EdgeId{2}},
        .control = PriorityControl{.yields_to = {{EdgeId{0}, {EdgeId{2}}}}}};  // minor (0) yields to main (2)
    engine.set_junctions({junction});

    // Minor-road vehicle approaching the stop line...
    Vehicle minor{.id = VehicleId{0},
                  .idm_params = default_params(VehicleType::Car),
                  .speed = 10.f,
                  .edge_id = EdgeId{0},
                  .offset = 40.f};
    // ...while a main-road vehicle is mid-crossing, well inside the gap-acceptance window.
    Vehicle main{.id = VehicleId{1},
                 .idm_params = default_params(VehicleType::Car),
                 .speed = 10.f,
                 .edge_id = EdgeId{2},
                 .offset = 40.f};

    engine.vehicles().push_back(minor);
    engine.vehicles().push_back(main);

    for (int i = 0; i < 100; ++i) {  // 2s -- main-road car should already be through, minor should still wait
        engine.tick();
    }

    EXPECT_LE(engine.vehicles()[0].offset, 50.f) << "minor-road vehicle entered the junction while it had to yield";
}