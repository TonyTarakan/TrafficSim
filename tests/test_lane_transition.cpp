#include <gtest/gtest.h>

#include <set>

#include "core/sim_engine.hpp"
#include "core/types.hpp"
#include "core/vehicle_params.hpp"

using namespace ts;

TEST(LaneTransition, VehicleAdvancesToNextLaneOnRoute)
{
    SimEngine engine;
    std::vector<RoadNode> nodes = {
        {.id = NodeId{0}, .pos = {.x = 0.f, .y = 0.f}},
        {.id = NodeId{1}, .pos = {.x = 100.f, .y = 0.f}},
        {.id = NodeId{2}, .pos = {.x = 200.f, .y = 0.f}},
    };
    std::vector<Lane> lanes = {
        {.id = LaneId{0}, .from = NodeId{0}, .to = NodeId{1}, .length = 100.f, .speed_limit = 20.f, .num_sublanes = 1},
        {.id = LaneId{1}, .from = NodeId{1}, .to = NodeId{2}, .length = 100.f, .speed_limit = 20.f, .num_sublanes = 1},
    };
    engine.set_map(nodes, lanes);

    auto route = engine.compute_route(NodeId{0}, NodeId{2});
    ASSERT_TRUE(route.has_value());
    ASSERT_EQ(route->size(), 2u);

    Vehicle v;
    v.id = VehicleId{0};
    v.idm_params = default_params(VehicleType::Car);
    v.lane_id = LaneId{0};
    v.offset = 95.f;  // near the end of lane 0
    v.speed = 15.f;
    v.route = *route;
    engine.vehicles().push_back(v);

    bool transitioned = false;
    for (int i = 0; i < 100 && !transitioned; ++i) {
        engine.tick();
        const auto& r = engine.vehicles()[0];
        if (r.lane_id == LaneId{1}) {
            transitioned = true;
            EXPECT_EQ(r.route_idx, 1u);
            EXPECT_GE(r.offset, 0.f);
            EXPECT_LT(r.offset, 1.f);  // small overflow, not reset or huge
        }
    }
    EXPECT_TRUE(transitioned);
}

TEST(LaneTransition, ForcedMergeClampsSublaneOnNarrowerLane)
{
    SimEngine engine;
    std::vector<RoadNode> nodes = {
        {.id = NodeId{0}, .pos = {.x = 0.f, .y = 0.f}},
        {.id = NodeId{1}, .pos = {.x = 100.f, .y = 0.f}},
        {.id = NodeId{2}, .pos = {.x = 200.f, .y = 0.f}},
    };
    std::vector<Lane> lanes = {
        {.id = LaneId{0}, .from = NodeId{0}, .to = NodeId{1}, .length = 100.f, .speed_limit = 20.f, .num_sublanes = 3},
        {.id = LaneId{1}, .from = NodeId{1}, .to = NodeId{2}, .length = 100.f, .speed_limit = 20.f, .num_sublanes = 1},
    };
    engine.set_map(nodes, lanes);

    auto route = engine.compute_route(NodeId{0}, NodeId{2});
    ASSERT_TRUE(route.has_value());

    Vehicle v;
    v.id = VehicleId{0};
    v.idm_params = default_params(VehicleType::Car);
    v.lane_id = LaneId{0};
    v.sublane_idx = 2;  // leftmost of 3 -- doesn't exist on the bottleneck lane
    v.offset = 95.f;
    v.speed = 15.f;
    v.route = *route;
    engine.vehicles().push_back(v);

    bool merged = false;
    for (int i = 0; i < 100 && !merged; ++i) {
        engine.tick();
        const auto& r = engine.vehicles()[0];
        if (r.lane_id == LaneId{1}) {
            merged = true;
            EXPECT_EQ(r.sublane_idx, 0);  // only valid slot on the 1-sublane road
        }
    }
    EXPECT_TRUE(merged);
}

TEST(LaneTransition, TwoStreamsBothReachSharedLane)
{
    // Y-merge: streams from node0 and node1 both converge onto lane 2
    // before continuing to node3. Verifies routing + merging work
    // together for two independent origins sharing a destination.
    SimEngine engine;
    std::vector<RoadNode> nodes = {
        {.id = NodeId{0}, .pos = {.x = 0.f, .y = -30.f}},
        {.id = NodeId{1}, .pos = {.x = 0.f, .y = 30.f}},
        {.id = NodeId{2}, .pos = {.x = 150.f, .y = 0.f}},
        {.id = NodeId{3}, .pos = {.x = 300.f, .y = 0.f}},
    };
    std::vector<Lane> lanes = {
        {.id = LaneId{0}, .from = NodeId{0}, .to = NodeId{2}, .length = 153.f, .speed_limit = 15.f, .num_sublanes = 2},
        {.id = LaneId{1}, .from = NodeId{1}, .to = NodeId{2}, .length = 153.f, .speed_limit = 15.f, .num_sublanes = 2},
        {.id = LaneId{2}, .from = NodeId{2}, .to = NodeId{3}, .length = 150.f, .speed_limit = 15.f, .num_sublanes = 2},
    };
    engine.set_map(nodes, lanes);

    auto route_a = engine.compute_route(NodeId{0}, NodeId{3});
    auto route_b = engine.compute_route(NodeId{1}, NodeId{3});
    ASSERT_TRUE(route_a.has_value());
    ASSERT_TRUE(route_b.has_value());

    for (int i = 0; i < 4; ++i) {
        Vehicle v;
        v.id = VehicleId{static_cast<uint32_t>(i)};
        v.idm_params = default_params(VehicleType::Car);
        v.lane_id = LaneId{0};
        v.sublane_idx = i % 2;
        v.offset = static_cast<float>(i) * 20.f;
        v.speed = 10.f;
        v.route = *route_a;
        engine.vehicles().push_back(v);
    }
    for (int i = 4; i < 8; ++i) {
        Vehicle v;
        v.id = VehicleId{static_cast<uint32_t>(i)};
        v.idm_params = default_params(VehicleType::Car);
        v.lane_id = LaneId{1};
        v.sublane_idx = i % 2;
        v.offset = static_cast<float>(i - 4) * 20.f;
        v.speed = 10.f;
        v.route = *route_b;
        engine.vehicles().push_back(v);
    }

    std::set<VehicleId> ever_merged;
    for (int tick = 0; tick < 2000; ++tick) {  // 40s
        engine.tick();
        for (const auto& v : engine.vehicles()) {
            if (v.lane_id == LaneId{2}) ever_merged.insert(v.id);
        }
    }

    EXPECT_EQ(ever_merged.size(), 8u);
}