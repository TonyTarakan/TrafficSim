#include <gtest/gtest.h>

#include "core/road_graph.hpp"
#include "core/types.hpp"

using namespace ts;

TEST(RoadNode, DefaultIsInvalid)
{
    RoadNode n;
    EXPECT_EQ(n.id, kInvalidNode);
    EXPECT_FLOAT_EQ(n.z, 0.f);
}

TEST(RoadNode, CanSetFields)
{
    RoadNode n;
    n.id = NodeId{5};
    n.pos = {.x = 10.f, .y = 20.f};
    n.z = 1.f;

    EXPECT_EQ(n.id, NodeId{5});
    EXPECT_FLOAT_EQ(n.pos.x, 10.f);
    EXPECT_FLOAT_EQ(n.pos.y, 20.f);
    EXPECT_FLOAT_EQ(n.z, 1.f);
}

TEST(Lane, DefaultIsInvalid)
{
    Lane l;
    EXPECT_EQ(l.id, kInvalidLane);
    EXPECT_EQ(l.from, kInvalidNode);
    EXPECT_EQ(l.to, kInvalidNode);
}

TEST(Lane, DefaultSpeedLimitIsReasonable)
{
    Lane l;
    // ~60 km/h in m/s — default for a lane.
    EXPECT_NEAR(l.speed_limit, 16.7f, 0.1f);
}

TEST(Lane, DefaultsToSingleSublane)
{
    Lane l;
    EXPECT_EQ(l.num_sublanes, 1);
}

TEST(Lane, CanConfigureMultiLaneSegment)
{
    Lane l;
    l.id = LaneId{0};
    l.from = NodeId{1};
    l.to = NodeId{2};
    l.length = 150.f;
    l.speed_limit = 27.8f;  // ~100 km/h
    l.num_sublanes = 3;

    EXPECT_EQ(l.from, NodeId{1});
    EXPECT_EQ(l.to, NodeId{2});
    EXPECT_FLOAT_EQ(l.length, 150.f);
    EXPECT_EQ(l.num_sublanes, 3);
}

namespace {

// Simple square loop: 0 -> 1 -> 2 -> 3 -> 0, all lanes 100m @ 13.9 m/s.
std::pair<std::vector<RoadNode>, std::vector<Lane>> make_square()
{
    std::vector<RoadNode> nodes = {
        {.id = NodeId{0}, .pos = {.x = 0.f, .y = 0.f}},
        {.id = NodeId{1}, .pos = {.x = 100.f, .y = 0.f}},
        {.id = NodeId{2}, .pos = {.x = 100.f, .y = 100.f}},
        {.id = NodeId{3}, .pos = {.x = 0.f, .y = 100.f}},
    };
    std::vector<Lane> lanes = {
        {.id = LaneId{0}, .from = NodeId{0}, .to = NodeId{1}, .length = 100.f, .speed_limit = 13.9f, .num_sublanes = 1},
        {.id = LaneId{1}, .from = NodeId{1}, .to = NodeId{2}, .length = 100.f, .speed_limit = 13.9f, .num_sublanes = 1},
        {.id = LaneId{2}, .from = NodeId{2}, .to = NodeId{3}, .length = 100.f, .speed_limit = 13.9f, .num_sublanes = 1},
        {.id = LaneId{3}, .from = NodeId{3}, .to = NodeId{0}, .length = 100.f, .speed_limit = 13.9f, .num_sublanes = 1},
    };
    return {nodes, lanes};
}

}  // namespace

TEST(RoadGraph, RebuildPopulatesAdjacency)
{
    auto [nodes, lanes] = make_square();
    RoadGraph g;
    g.rebuild(nodes, lanes);

    auto out = g.outgoing_lanes(NodeId{0});
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0], LaneId{0});  // lane 0 leaves node 0
}

TEST(RoadGraph, SinkNodeHasEmptyOutgoing)
{
    auto [nodes, lanes] = make_square();
    lanes.pop_back();  // remove lane 3 -> 0, so node 3 becomes a dead end
    RoadGraph g;
    g.rebuild(nodes, lanes);

    EXPECT_TRUE(g.outgoing_lanes(NodeId{3}).empty());
}

TEST(RoadGraph, LaneEndNodeIsCorrect)
{
    auto [nodes, lanes] = make_square();
    RoadGraph g;
    g.rebuild(nodes, lanes);

    EXPECT_EQ(g.destination_node(LaneId{0}), NodeId{1});
    EXPECT_EQ(g.destination_node(LaneId{2}), NodeId{3});
    EXPECT_EQ(g.destination_node(LaneId{999}), kInvalidNode);  // unknown lane
}

TEST(RoadGraph, FindRouteSameNodeIsEmptyPath)
{
    auto [nodes, lanes] = make_square();
    RoadGraph g;
    g.rebuild(nodes, lanes);

    auto route = g.find_route(NodeId{1}, NodeId{1});
    ASSERT_TRUE(route.has_value());
    EXPECT_TRUE(route->empty());
}

TEST(RoadGraph, FindRouteDirectNeighbour)
{
    auto [nodes, lanes] = make_square();
    RoadGraph g;
    g.rebuild(nodes, lanes);

    auto route = g.find_route(NodeId{0}, NodeId{1});
    ASSERT_TRUE(route.has_value());
    ASSERT_EQ(route->size(), 1u);
    EXPECT_EQ((*route)[0], LaneId{0});
}

TEST(RoadGraph, FindRouteMultiHop)
{
    auto [nodes, lanes] = make_square();
    RoadGraph g;
    g.rebuild(nodes, lanes);

    // Only path around the loop is 0 -> 1 -> 2 (lanes are one-directional).
    auto route = g.find_route(NodeId{0}, NodeId{2});
    ASSERT_TRUE(route.has_value());
    ASSERT_EQ(route->size(), 2u);
    EXPECT_EQ((*route)[0], LaneId{0});
    EXPECT_EQ((*route)[1], LaneId{1});
}

TEST(RoadGraph, FindRoutePicksFasterPathOverShorterOne)
{
    // Two parallel routes from 0 to 2:
    //   lane A: 0 -> 2 direct, 200m, slow (5 m/s)   -> 40s travel time
    //   lane B: 0 -> 1 -> 2, 2x150m, fast (25 m/s)  -> 12s travel time
    // A* should pick the faster one despite it being geometrically longer.
    std::vector<RoadNode> nodes = {
        {.id = NodeId{0}, .pos = {.x = 0.f, .y = 0.f}},
        {.id = NodeId{1}, .pos = {.x = 50.f, .y = 50.f}},
        {.id = NodeId{2}, .pos = {.x = 100.f, .y = 0.f}},
    };
    std::vector<Lane> lanes = {
        {.id = LaneId{0}, .from = NodeId{0}, .to = NodeId{2}, .length = 200.f, .speed_limit = 5.f, .num_sublanes = 1},
        {.id = LaneId{1}, .from = NodeId{0}, .to = NodeId{1}, .length = 150.f, .speed_limit = 25.f, .num_sublanes = 1},
        {.id = LaneId{2}, .from = NodeId{1}, .to = NodeId{2}, .length = 150.f, .speed_limit = 25.f, .num_sublanes = 1},
    };
    RoadGraph g;
    g.rebuild(nodes, lanes);

    auto route = g.find_route(NodeId{0}, NodeId{2});
    ASSERT_TRUE(route.has_value());
    ASSERT_EQ(route->size(), 2u);
    EXPECT_EQ((*route)[0], LaneId{1});  // via node 1, not the direct slow lane 0
    EXPECT_EQ((*route)[1], LaneId{2});
}

TEST(RoadGraph, FindRouteUnreachableReturnsNullopt)
{
    auto [nodes, lanes] = make_square();
    lanes.pop_back();  // node 3 -> 0 removed, breaks the loop
    RoadGraph g;
    g.rebuild(nodes, lanes);

    // Node 3 is still reachable from 0, but nothing leads back to 0 from 3.
    auto route = g.find_route(NodeId{3}, NodeId{0});
    EXPECT_FALSE(route.has_value());
}