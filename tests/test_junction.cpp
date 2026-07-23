#include <gtest/gtest.h>

#include "core/junction.hpp"

using namespace ts;

namespace {

// A four-way crossroads at node 10, (0, 0): approaches from east, north,
// west and south. World coords are y-down (screen space), so "north" is
// -y and "south" is +y.
//
//              north_in (101, y=-50)
//                    |
//   west_in (104) -- 10 (0,0) -- east_in (100, x=50)
//                    |
//              south_in (105, y=50)
//
// Lane 0 is also authored as PriorityControl's "main road" in some tests.
struct CrossroadsFixture {
    RoadNode junction{.id = 10, .pos = {.x = 0.f, .y = 0.f}};
    RoadNode east_origin{.id = 100, .pos = {.x = 50.f, .y = 0.f}};
    RoadNode north_origin{.id = 101, .pos = {.x = 0.f, .y = -50.f}};
    RoadNode east_dest{.id = 102, .pos = {.x = 100.f, .y = 0.f}};
    RoadNode north_dest{.id = 103, .pos = {.x = 0.f, .y = -100.f}};
    RoadNode west_origin{.id = 104, .pos = {.x = -50.f, .y = 0.f}};
    RoadNode south_origin{.id = 105, .pos = {.x = 0.f, .y = 50.f}};

    Lane east_in{.id = 0, .from = 100, .to = 10, .length = 50.f};   // heading west
    Lane north_in{.id = 1, .from = 101, .to = 10, .length = 50.f};  // heading south
    Lane east_out{.id = 2, .from = 10, .to = 102, .length = 50.f};
    Lane north_out{.id = 3, .from = 10, .to = 103, .length = 50.f};
    Lane west_in{.id = 4, .from = 104, .to = 10, .length = 50.f};   // heading east
    Lane south_in{.id = 5, .from = 105, .to = 10, .length = 50.f};  // heading north

    [[nodiscard]]
    std::vector<Lane> all_lanes() const
    {
        return {east_in, north_in, east_out, north_out, west_in, south_in};
    }

    [[nodiscard]]
    std::vector<RoadNode> all_nodes() const
    {
        return {junction, east_origin, north_origin, east_dest, north_dest, west_origin, south_origin};
    }
};

}  // namespace

TEST(TrafficLightControl, NoPhasesIsAlwaysGreen)
{
    TrafficLightControl light;
    EXPECT_TRUE(light.is_green(0));
}

TEST(TrafficLightControl, RespectsCurrentPhase)
{
    TrafficLightControl light;
    light.phases = {
        {.green_lanes = {0}, .duration = 10.f},
        {.green_lanes = {1}, .duration = 10.f},
    };

    EXPECT_TRUE(light.is_green(0));
    EXPECT_FALSE(light.is_green(1));
}

TEST(TrafficLightControl, AdvanceSignalsCyclesPhases)
{
    Junction j;
    j.node_id = 10;
    j.incoming = {0, 1};
    j.control = TrafficLightControl{.phases = {
                                        {.green_lanes = {0}, .duration = 10.f},
                                        {.green_lanes = {1}, .duration = 10.f},
                                    }};

    JunctionMap map;
    map.rebuild({j});

    map.advance_signals(10.f);  // exactly one phase length

    const auto* light = std::get_if<TrafficLightControl>(&map.find_by_node(10)->control);
    ASSERT_NE(light, nullptr);
    EXPECT_EQ(light->phase_idx, 1u);
    EXPECT_TRUE(light->is_green(1));
    EXPECT_FALSE(light->is_green(0));
}

TEST(JunctionMap, FindByNodeAndLane)
{
    Junction j;
    j.node_id = 10;
    j.incoming = {0, 1};

    JunctionMap map;
    map.rebuild({j});

    EXPECT_EQ(map.find_by_node(10)->node_id, 10u);
    EXPECT_EQ(map.find_by_lane(0)->node_id, 10u);
    EXPECT_EQ(map.find_by_lane(1)->node_id, 10u);
    EXPECT_EQ(map.find_by_node(999), nullptr);
    EXPECT_EQ(map.find_by_lane(999), nullptr);
}

TEST(FindJunctionYield, PriorityLaneWithNoRivalsProceeds)
{
    CrossroadsFixture f;
    Junction j{.node_id = 10,
               .incoming = {0, 1},
               .control = PriorityControl{.yields_to = {{1, {0}}}}};  // lane 1 yields to lane 0
    JunctionMap map;
    map.rebuild({j});

    // Ego is on lane 0, the main road -- not in anyone's yields_to list.
    Vehicle ego{.id = 0, .lane_id = 0, .offset = 45.f};
    auto result = leader_to_yield(ego, f.east_in, map, {}, f.all_lanes());
    EXPECT_FALSE(result.has_value());
}

TEST(FindJunctionYield, MinorLaneYieldsToCloseRival)
{
    CrossroadsFixture f;
    Junction j{.node_id = 10, .incoming = {0, 1}, .control = PriorityControl{.yields_to = {{1, {0}}}}};
    JunctionMap map;
    map.rebuild({j});

    Vehicle ego{.id = 1, .lane_id = 1, .offset = 45.f};                   // on the minor approach, 5m from the line
    Vehicle rival{.id = 0, .speed = 10.f, .lane_id = 0, .offset = 30.f};  // 20m out, 2s to the line

    std::vector<Vehicle> vehicles = {ego, rival};
    auto result = leader_to_yield(ego, f.north_in, map, vehicles, f.all_lanes());

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->gap, 5.f, 1e-3f);
}

TEST(FindJunctionYield, MinorLaneProceedsWhenGapIsWideEnough)
{
    CrossroadsFixture f;
    Junction j{.node_id = 10, .incoming = {0, 1}, .control = PriorityControl{.yields_to = {{1, {0}}}}};
    JunctionMap map;
    map.rebuild({j});

    Vehicle ego{.id = 1, .lane_id = 1, .offset = 45.f};
    // Rival is 40m out doing 5 m/s -- 8s to the line, comfortably over the 4s gap threshold.
    Vehicle rival{.id = 0, .speed = 5.f, .lane_id = 0, .offset = 10.f};

    std::vector<Vehicle> vehicles = {ego, rival};
    auto result = leader_to_yield(ego, f.north_in, map, vehicles, f.all_lanes());
    EXPECT_FALSE(result.has_value());
}

TEST(FindJunctionYield, YieldsWhileJunctionBoxIsOccupied)
{
    CrossroadsFixture f;
    Junction j{.node_id = 10, .incoming = {0, 1}, .control = PriorityControl{.yields_to = {{1, {0}}}}};
    JunctionMap map;
    map.rebuild({j});

    Vehicle ego{.id = 1, .lane_id = 1, .offset = 45.f};
    // Someone is already crossing, freshly out of the junction on the east-out lane.
    Vehicle crossing{.id = 2, .lane_id = 2, .offset = 3.f};

    std::vector<Vehicle> vehicles = {ego, crossing};
    auto result = leader_to_yield(ego, f.north_in, map, vehicles, f.all_lanes());
    EXPECT_TRUE(result.has_value());
}

TEST(FindJunctionYield, RedLightForcesStop)
{
    CrossroadsFixture f;
    Junction j{.node_id = 10,
               .incoming = {0, 1},
               .control = TrafficLightControl{.phases = {
                                                  {.green_lanes = {0}, .duration = 10.f},
                                                  {.green_lanes = {1}, .duration = 10.f},
                                              }}};
    JunctionMap map;
    map.rebuild({j});

    Vehicle ego{.id = 1, .lane_id = 1, .offset = 45.f};  // lane 1 is red in phase 0
    auto result = leader_to_yield(ego, f.north_in, map, {}, f.all_lanes());
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->gap, 5.f, 1e-3f);
}

TEST(FindJunctionYield, GreenLightProceeds)
{
    CrossroadsFixture f;
    Junction j{.node_id = 10,
               .incoming = {0, 1},
               .control = TrafficLightControl{.phases = {
                                                  {.green_lanes = {0}, .duration = 10.f},
                                                  {.green_lanes = {1}, .duration = 10.f},
                                              }}};
    JunctionMap map;
    map.rebuild({j});

    Vehicle ego{.id = 0, .lane_id = 0, .offset = 45.f};  // lane 0 is green in phase 0
    auto result = leader_to_yield(ego, f.east_in, map, {}, f.all_lanes());
    EXPECT_FALSE(result.has_value());
}
