#include <gtest/gtest.h>

#include "core/junction.hpp"
#include "core/road_graph.hpp"
#include "core/types.hpp"

using namespace ts;

namespace {

// Four-way crossroads at node 10, (0, 0): approaches from east, north,
// west and south. World coords are y-down (screen space), so "north" is
// -y and "south" is +y.
//
//              north_in (101, y=-50)
//                    |
//   west_in (104) -- 10 (0,0) -- east_in (100, x=50)
//                    |
//              south_in (105, y=50)
//
struct CrossroadsFixture {
    Node junction{.id = NodeId{10}, .pos = {.x = 0.f, .y = 0.f}};
    Node east_origin{.id = NodeId{100}, .pos = {.x = 50.f, .y = 0.f}};
    Node north_origin{.id = NodeId{101}, .pos = {.x = 0.f, .y = -50.f}};
    Node east_dest{.id = NodeId{102}, .pos = {.x = 100.f, .y = 0.f}};
    Node north_dest{.id = NodeId{103}, .pos = {.x = 0.f, .y = -100.f}};
    Node west_origin{.id = NodeId{104}, .pos = {.x = -50.f, .y = 0.f}};
    Node south_origin{.id = NodeId{105}, .pos = {.x = 0.f, .y = 50.f}};

    Edge east_in{.id = EdgeId{0}, .from = NodeId{100}, .to = NodeId{10}, .length = 50.f};
    Edge north_in{.id = EdgeId{1}, .from = NodeId{101}, .to = NodeId{10}, .length = 50.f};
    Edge east_out{.id = EdgeId{2}, .from = NodeId{10}, .to = NodeId{102}, .length = 50.f};
    Edge north_out{.id = EdgeId{3}, .from = NodeId{10}, .to = NodeId{103}, .length = 50.f};
    Edge west_in{.id = EdgeId{4}, .from = NodeId{104}, .to = NodeId{10}, .length = 50.f};
    Edge south_in{.id = EdgeId{5}, .from = NodeId{105}, .to = NodeId{10}, .length = 50.f};

    [[nodiscard]]
    std::vector<Edge> all_lanes() const
    {
        return {east_in, north_in, east_out, north_out, west_in, south_in};
    }

    [[nodiscard]]
    std::vector<Node> all_nodes() const
    {
        return {junction, east_origin, north_origin, east_dest, north_dest, west_origin, south_origin};
    }

    [[nodiscard]]
    RoadGraph build_graph() const
    {
        return {all_nodes(), all_lanes()};
    }
};

}  // namespace

TEST(TrafficLightControl, NoPhasesIsAlwaysGreen)
{
    TrafficLightControl light;
    EXPECT_TRUE(light.is_green(EdgeId{0}));
}

TEST(TrafficLightControl, RespectsCurrentPhase)
{
    TrafficLightControl light;
    light.phases = {
        {.green_lanes = {EdgeId{0}}, .duration = 10.f},
        {.green_lanes = {EdgeId{1}}, .duration = 10.f},
    };

    EXPECT_TRUE(light.is_green(EdgeId{0}));
    EXPECT_FALSE(light.is_green(EdgeId{1}));
}

TEST(TrafficLightControl, AdvanceSignalsCyclesPhases)
{
    Junction j;
    j.node_id = NodeId{10};
    j.incoming = {EdgeId{0}, EdgeId{1}};
    j.control = TrafficLightControl{.phases = {
                                        {.green_lanes = {EdgeId{0}}, .duration = 10.f},
                                        {.green_lanes = {EdgeId{1}}, .duration = 10.f},
                                    }};

    JunctionMap map;
    RoadGraph empty_graph(std::vector<Node>{}, std::vector<Edge>{});
    map.rebuild({j}, empty_graph);

    map.advance_signals(10.f);  // exactly one phase length

    const auto* light = std::get_if<TrafficLightControl>(&map.find_by_node(NodeId{10})->control);
    ASSERT_NE(light, nullptr);
    EXPECT_EQ(light->phase_idx, 1u);
    EXPECT_TRUE(light->is_green(EdgeId{1}));
    EXPECT_FALSE(light->is_green(EdgeId{0}));
}

TEST(JunctionMap, FindByNodeAndLane)
{
    Junction j;
    j.node_id = NodeId{10};
    j.incoming = {EdgeId{0}, EdgeId{1}};

    JunctionMap map;
    RoadGraph empty_graph(std::vector<Node>{}, std::vector<Edge>{});
    map.rebuild({j}, empty_graph);

    EXPECT_EQ(map.find_by_node(NodeId{10})->node_id, NodeId{10});
    EXPECT_EQ(map.find_by_lane(EdgeId{0})->node_id, NodeId{10});
    EXPECT_EQ(map.find_by_lane(EdgeId{1})->node_id, NodeId{10});
    EXPECT_EQ(map.find_by_node(NodeId{999}), nullptr);
    EXPECT_EQ(map.find_by_lane(EdgeId{999}), nullptr);
}

TEST(JunctionMap, RebuildResolvesGeometryFromNodesAndLanes)
{
    CrossroadsFixture f;
    Junction j{.node_id = NodeId{10}, .incoming = {EdgeId{0}, EdgeId{1}}};

    JunctionMap map;
    auto graph = f.build_graph();
    map.rebuild({j}, graph);

    const Junction* resolved = map.find_by_node(NodeId{10});
    ASSERT_NE(resolved, nullptr);
    EXPECT_FLOAT_EQ(resolved->pos.x, 0.f);
    EXPECT_FLOAT_EQ(resolved->pos.y, 0.f);

    auto east_from = resolved->lines_from.find(EdgeId{0});
    ASSERT_NE(east_from, resolved->lines_from.end());
    EXPECT_FLOAT_EQ(east_from->second.x, 50.f);
    EXPECT_FLOAT_EQ(east_from->second.y, 0.f);

    auto north_from = resolved->lines_from.find(EdgeId{1});
    ASSERT_NE(north_from, resolved->lines_from.end());
    EXPECT_FLOAT_EQ(north_from->second.x, 0.f);
    EXPECT_FLOAT_EQ(north_from->second.y, -50.f);
}

TEST(JunctionMap, RebuildLeavesGeometryUnresolvedForUnknownIds)
{
    Junction j{.node_id = NodeId{999}, .incoming = {EdgeId{123}}};  // nothing in the (empty) map matches

    JunctionMap map;
    RoadGraph empty_graph(std::vector<Node>{}, std::vector<Edge>{});
    map.rebuild({j}, empty_graph);  // no nodes/lanes -- shouldn't crash, just leaves defaults

    const Junction* resolved = map.find_by_node(NodeId{999});
    ASSERT_NE(resolved, nullptr);
    EXPECT_TRUE(resolved->lines_from.empty());
}

TEST(FindJunctionYield, PriorityLaneWithNoRivalsProceeds)
{
    CrossroadsFixture f;
    Junction j{.node_id = NodeId{10},
               .incoming = {EdgeId{0}, EdgeId{1}},
               .control = PriorityControl{.yields_to = {{EdgeId{1}, {EdgeId{0}}}}}};  // lane 1 yields to lane 0
    JunctionMap map;
    auto graph = f.build_graph();
    map.rebuild({j}, graph);

    // Ego is on lane 0, the main road -- not in anyone's yields_to list.
    Vehicle ego{.id = VehicleId{0}, .edge_id = EdgeId{0}, .offset = 45.f};
    auto result = leader_to_yield(ego, f.east_in, map, {}, graph);
    EXPECT_FALSE(result.has_value());
}

TEST(FindJunctionYield, MinorLaneYieldsToCloseRival)
{
    CrossroadsFixture f;
    Junction j{.node_id = NodeId{10},
               .incoming = {EdgeId{0}, EdgeId{1}},
               .control = PriorityControl{.yields_to = {{EdgeId{1}, {EdgeId{0}}}}}};
    JunctionMap map;
    auto graph = f.build_graph();
    map.rebuild({j}, graph);

    Vehicle ego{.id = VehicleId{1}, .edge_id = EdgeId{1}, .offset = 45.f};  // on the minor approach, 5m from the line
    Vehicle rival{.id = VehicleId{0}, .speed = 10.f, .edge_id = EdgeId{0}, .offset = 30.f};  // 20m out, 2s to the line

    std::vector<Vehicle> vehicles = {ego, rival};
    auto result = leader_to_yield(ego, f.north_in, map, vehicles, graph);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->gap, 5.f, 1e-3f);
}

TEST(FindJunctionYield, MinorLaneProceedsWhenGapIsWideEnough)
{
    CrossroadsFixture f;
    Junction j{.node_id = NodeId{10},
               .incoming = {EdgeId{0}, EdgeId{1}},
               .control = PriorityControl{.yields_to = {{EdgeId{1}, {EdgeId{0}}}}}};
    JunctionMap map;
    auto graph = f.build_graph();
    map.rebuild({j}, graph);

    Vehicle ego{.id = VehicleId{1}, .edge_id = EdgeId{1}, .offset = 45.f};
    // Rival is 40m out doing 5 m/s -- 8s to the line, comfortably over the 4s gap threshold.
    Vehicle rival{.id = VehicleId{0}, .speed = 5.f, .edge_id = EdgeId{0}, .offset = 10.f};

    std::vector<Vehicle> vehicles = {ego, rival};
    auto result = leader_to_yield(ego, f.north_in, map, vehicles, graph);
    EXPECT_FALSE(result.has_value());
}

TEST(FindJunctionYield, YieldsWhileJunctionBoxIsOccupied)
{
    CrossroadsFixture f;
    Junction j{.node_id = NodeId{10},
               .incoming = {EdgeId{0}, EdgeId{1}},
               .control = PriorityControl{.yields_to = {{EdgeId{1}, {EdgeId{0}}}}}};
    JunctionMap map;
    auto graph = f.build_graph();
    map.rebuild({j}, graph);

    Vehicle ego{.id = VehicleId{1}, .edge_id = EdgeId{1}, .offset = 45.f};
    // Someone is already crossing, freshly out of the junction on the east-out lane.
    Vehicle crossing{.id = VehicleId{2}, .edge_id = EdgeId{2}, .offset = 3.f};

    std::vector<Vehicle> vehicles = {ego, crossing};
    auto result = leader_to_yield(ego, f.north_in, map, vehicles, graph);
    EXPECT_TRUE(result.has_value());
}

TEST(FindJunctionYield, RedLightForcesStop)
{
    CrossroadsFixture f;
    Junction j{.node_id = NodeId{10},
               .incoming = {EdgeId{0}, EdgeId{1}},
               .control = TrafficLightControl{.phases = {
                                                  {.green_lanes = {EdgeId{0}}, .duration = 10.f},
                                                  {.green_lanes = {EdgeId{1}}, .duration = 10.f},
                                              }}};
    JunctionMap map;
    auto graph = f.build_graph();
    map.rebuild({j}, graph);

    Vehicle ego{.id = VehicleId{1}, .edge_id = EdgeId{1}, .offset = 45.f};  // lane 1 is red in phase 0
    auto result = leader_to_yield(ego, f.north_in, map, {}, graph);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->gap, 5.f, 1e-3f);
}

TEST(FindJunctionYield, GreenLightProceeds)
{
    CrossroadsFixture f;
    Junction j{.node_id = NodeId{10},
               .incoming = {EdgeId{0}, EdgeId{1}},
               .control = TrafficLightControl{.phases = {
                                                  {.green_lanes = {EdgeId{0}}, .duration = 10.f},
                                                  {.green_lanes = {EdgeId{1}}, .duration = 10.f},
                                              }}};
    JunctionMap map;
    auto graph = f.build_graph();
    map.rebuild({j}, graph);

    Vehicle ego{.id = VehicleId{0}, .edge_id = EdgeId{0}, .offset = 45.f};  // lane 0 is green in phase 0
    auto result = leader_to_yield(ego, f.east_in, map, {}, graph);
    EXPECT_FALSE(result.has_value());
}

TEST(FindJunctionYield, UnregulatedYieldsToTrafficOnTheRight)
{
    CrossroadsFixture f;
    Junction j{.node_id = NodeId{10},
               .incoming = {EdgeId{0}, EdgeId{1}, EdgeId{4}, EdgeId{5}},
               .control = UnregulatedControl{}};
    JunctionMap map;
    auto graph = f.build_graph();
    map.rebuild({j}, graph);

    // Facing west (east_in), the right-hand side is north -- ego must
    // yield to a car approaching from the north (north_in), even though
    // it isn't the closest thing around.
    Vehicle ego{.id = VehicleId{0}, .edge_id = EdgeId{0}, .offset = 45.f};
    // 15m out at 5 m/s -- 3s to the line, under the 4s gap threshold.
    Vehicle from_north{.id = VehicleId{1}, .speed = 5.f, .edge_id = EdgeId{1}, .offset = 35.f};

    std::vector<Vehicle> vehicles = {ego, from_north};
    auto result = leader_to_yield(ego, f.east_in, map, vehicles, graph);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->gap, 5.f, 1e-3f);
}

TEST(FindJunctionYield, UnregulatedDoesNotYieldToTrafficOnTheLeft)
{
    CrossroadsFixture f;
    Junction j{.node_id = NodeId{10},
               .incoming = {EdgeId{0}, EdgeId{1}, EdgeId{4}, EdgeId{5}},
               .control = UnregulatedControl{}};
    JunctionMap map;
    auto graph = f.build_graph();
    map.rebuild({j}, graph);

    // Facing west (east_in), south is on ego's left -- no yield owed,
    // even with a rival right on top of the line.
    Vehicle ego{.id = VehicleId{0}, .edge_id = EdgeId{0}, .offset = 45.f};
    Vehicle from_south{.id = VehicleId{1}, .speed = 5.f, .edge_id = EdgeId{5}, .offset = 49.f};

    std::vector<Vehicle> vehicles = {ego, from_south};
    auto result = leader_to_yield(ego, f.east_in, map, vehicles, graph);
    EXPECT_FALSE(result.has_value());
}

TEST(FindJunctionYield, UnregulatedDoesNotYieldToOncomingTraffic)
{
    CrossroadsFixture f;
    Junction j{.node_id = NodeId{10},
               .incoming = {EdgeId{0}, EdgeId{1}, EdgeId{4}, EdgeId{5}},
               .control = UnregulatedControl{}};
    JunctionMap map;
    auto graph = f.build_graph();
    map.rebuild({j}, graph);

    // east_in and west_in are a straight-through opposing pair -- neither
    // owes the other a yield under priority-to-the-right.
    Vehicle ego{.id = VehicleId{0}, .edge_id = EdgeId{0}, .offset = 45.f};
    Vehicle oncoming{.id = VehicleId{1}, .speed = 5.f, .edge_id = EdgeId{4}, .offset = 45.f};

    std::vector<Vehicle> vehicles = {ego, oncoming};
    auto result = leader_to_yield(ego, f.east_in, map, vehicles, graph);
    EXPECT_FALSE(result.has_value());
}

TEST(FindJunctionYield, UnregulatedIsNonReciprocal)
{
    // The car that has the right of way must not also yield to the car
    // that's yielding to it.
    CrossroadsFixture f;
    Junction j{.node_id = NodeId{10},
               .incoming = {EdgeId{0}, EdgeId{1}, EdgeId{4}, EdgeId{5}},
               .control = UnregulatedControl{}};
    JunctionMap map;
    auto graph = f.build_graph();
    map.rebuild({j}, graph);

    Vehicle from_north{.id = VehicleId{1}, .edge_id = EdgeId{1}, .offset = 45.f};
    Vehicle ego{.id = VehicleId{0}, .speed = 5.f, .edge_id = EdgeId{0}, .offset = 20.f};
    // east_in, well within the window

    std::vector<Vehicle> vehicles = {ego, from_north};
    auto result = leader_to_yield(from_north, f.north_in, map, vehicles, graph);
    EXPECT_FALSE(result.has_value());
}