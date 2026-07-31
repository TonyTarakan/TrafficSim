#include <gtest/gtest.h>

#include <stdexcept>

#include "core/road_graph.hpp"
#include "core/types.hpp"

using namespace ts;

TEST(RoadNode, DefaultIsInvalid)
{
    Node n;
    EXPECT_EQ(n.id, kInvalidNode);
    EXPECT_FLOAT_EQ(n.z, 0.f);
}

TEST(RoadNode, CanSetFields)
{
    Node n;
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
    Edge l;
    EXPECT_EQ(l.id, kInvalidEdge);
    EXPECT_EQ(l.from, kInvalidNode);
    EXPECT_EQ(l.to, kInvalidNode);
}

TEST(Lane, DefaultSpeedLimitIsReasonable)
{
    Edge l;
    // ~60 km/h in m/s — default for a lane.
    EXPECT_NEAR(l.speed_limit, 16.7f, 0.1f);
}

TEST(Lane, DefaultsToSingleSublane)
{
    Edge l;
    EXPECT_EQ(l.lane_count, 1);
}

TEST(Lane, CanConfigureMultiLaneSegment)
{
    Edge l;
    l.id = EdgeId{0};
    l.from = NodeId{1};
    l.to = NodeId{2};
    l.length = 150.f;
    l.speed_limit = 27.8f;  // ~100 km/h
    l.lane_count = 3;

    EXPECT_EQ(l.from, NodeId{1});
    EXPECT_EQ(l.to, NodeId{2});
    EXPECT_FLOAT_EQ(l.length, 150.f);
    EXPECT_EQ(l.lane_count, 3);
}

namespace {

// Simple square loop: 0 -> 1 -> 2 -> 3 -> 0, all edges 100m @ 13.9 m/s.
std::pair<std::vector<Node>, std::vector<Edge>> make_square()
{
    std::vector<Node> nodes = {
        {.id = NodeId{0}, .pos = {.x = 0.f, .y = 0.f}},
        {.id = NodeId{1}, .pos = {.x = 100.f, .y = 0.f}},
        {.id = NodeId{2}, .pos = {.x = 100.f, .y = 100.f}},
        {.id = NodeId{3}, .pos = {.x = 0.f, .y = 100.f}},
    };
    std::vector<Edge> edges = {
        {.id = EdgeId{0}, .from = NodeId{0}, .to = NodeId{1}, .length = 100.f, .speed_limit = 13.9f, .lane_count = 1},
        {.id = EdgeId{1}, .from = NodeId{1}, .to = NodeId{2}, .length = 100.f, .speed_limit = 13.9f, .lane_count = 1},
        {.id = EdgeId{2}, .from = NodeId{2}, .to = NodeId{3}, .length = 100.f, .speed_limit = 13.9f, .lane_count = 1},
        {.id = EdgeId{3}, .from = NodeId{3}, .to = NodeId{0}, .length = 100.f, .speed_limit = 13.9f, .lane_count = 1},
    };
    return {nodes, edges};
}

}  // namespace

TEST(RoadGraph, RebuildPopulatesAdjacency)
{
    auto [nodes, edges] = make_square();
    RoadGraph g{nodes, edges};

    auto out = g.outgoing_edges(NodeId{0});
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0], EdgeId{0});  // lane 0 leaves node 0
}

TEST(RoadGraph, SinkNodeHasEmptyOutgoing)
{
    auto [nodes, edges] = make_square();
    edges.pop_back();  // remove lane 3 -> 0, so node 3 becomes a dead end
    RoadGraph g{nodes, edges};

    EXPECT_TRUE(g.outgoing_edges(NodeId{3}).empty());
}

TEST(RoadGraph, LaneEndNodeIsCorrect)
{
    auto [nodes, edges] = make_square();
    RoadGraph g{nodes, edges};

    EXPECT_EQ(g.destination_node(EdgeId{0}), NodeId{1});
    EXPECT_EQ(g.destination_node(EdgeId{2}), NodeId{3});
    EXPECT_EQ(g.destination_node(EdgeId{999}), kInvalidNode);  // unknown lane
}

TEST(RoadGraph, FindRouteSameNodeIsEmptyPath)
{
    auto [nodes, edges] = make_square();
    RoadGraph g{nodes, edges};

    auto route = g.find_route(NodeId{1}, NodeId{1});
    ASSERT_TRUE(route.has_value());
    EXPECT_TRUE(route->empty());
}

TEST(RoadGraph, FindRouteDirectNeighbour)
{
    auto [nodes, edges] = make_square();
    RoadGraph g{nodes, edges};

    auto route = g.find_route(NodeId{0}, NodeId{1});
    ASSERT_TRUE(route.has_value());
    ASSERT_EQ(route->size(), 1u);
    EXPECT_EQ((*route)[0], EdgeId{0});
}

TEST(RoadGraph, FindRouteMultiHop)
{
    auto [nodes, edges] = make_square();
    RoadGraph g{nodes, edges};

    // Only path around the loop is 0 -> 1 -> 2 (edges are one-directional).
    auto route = g.find_route(NodeId{0}, NodeId{2});
    ASSERT_TRUE(route.has_value());
    ASSERT_EQ(route->size(), 2u);
    EXPECT_EQ((*route)[0], EdgeId{0});
    EXPECT_EQ((*route)[1], EdgeId{1});
}

TEST(RoadGraph, FindRoutePicksFasterPathOverShorterOne)
{
    // Two parallel routes from 0 to 2:
    //   lane A: 0 -> 2 direct, 200m, slow (5 m/s)   -> 40s travel time
    //   lane B: 0 -> 1 -> 2, 2x150m, fast (25 m/s)  -> 12s travel time
    // A* should pick the faster one despite it being geometrically longer.
    std::vector<Node> nodes = {
        {.id = NodeId{0}, .pos = {.x = 0.f, .y = 0.f}},
        {.id = NodeId{1}, .pos = {.x = 50.f, .y = 50.f}},
        {.id = NodeId{2}, .pos = {.x = 100.f, .y = 0.f}},
    };
    std::vector<Edge> edges = {
        {.id = EdgeId{0}, .from = NodeId{0}, .to = NodeId{2}, .length = 200.f, .speed_limit = 5.f, .lane_count = 1},
        {.id = EdgeId{1}, .from = NodeId{0}, .to = NodeId{1}, .length = 150.f, .speed_limit = 25.f, .lane_count = 1},
        {.id = EdgeId{2}, .from = NodeId{1}, .to = NodeId{2}, .length = 150.f, .speed_limit = 25.f, .lane_count = 1},
    };
    RoadGraph g{nodes, edges};

    auto route = g.find_route(NodeId{0}, NodeId{2});
    ASSERT_TRUE(route.has_value());
    ASSERT_EQ(route->size(), 2u);
    EXPECT_EQ((*route)[0], EdgeId{1});  // via node 1, not the direct slow lane 0
    EXPECT_EQ((*route)[1], EdgeId{2});
}

TEST(RoadGraph, FindRouteUnreachableReturnsNullopt)
{
    auto [nodes, edges] = make_square();
    edges.pop_back();  // node 3 -> 0 removed, breaks the loop
    RoadGraph g{nodes, edges};

    // Node 3 is still reachable from 0, but nothing leads back to 0 from 3.
    auto route = g.find_route(NodeId{3}, NodeId{0});
    EXPECT_FALSE(route.has_value());
}

TEST(RoadGraph, NonDenseNodeIdThrowsAtConstruction)
{
    // Ids that don't match storage position (0,1,2,...) violate the
    // dense id == index contract RoadGraph relies on for O(1) raw-array
    // lookups. This also covers duplicates and OSM-style sparse ids --
    // any of them will desync id from position somewhere.
    std::vector<Node> nodes = {
        {.id = NodeId{0}, .pos = {.x = 0.f, .y = 0.f}},
        {.id = NodeId{5}, .pos = {.x = 100.f, .y = 0.f}},  // should be NodeId{1}
    };
    EXPECT_THROW(RoadGraph(nodes, {}), std::invalid_argument);
}

TEST(RoadGraph, DuplicateNodeIdThrows)
{
    std::vector<Node> nodes = {
        {.id = NodeId{1}, .pos = {.x = 0.f, .y = 0.f}},
        {.id = NodeId{1}, .pos = {.x = 100.f, .y = 0.f}},  // same id as above, and neither matches its position
    };
    EXPECT_THROW(RoadGraph(nodes, {}), std::invalid_argument);
}

TEST(RoadGraph, NonDenseEdgeIdThrowsAtConstruction)
{
    auto [nodes, edges] = make_square();
    edges[1].id = EdgeId{99};  // no longer matches its position (1)
    EXPECT_THROW(RoadGraph(nodes, edges), std::invalid_argument);
}

TEST(RoadGraph, EdgeToUnknownNodeThrows)
{
    std::vector<Node> nodes = {{.id = NodeId{0}, .pos = {.x = 0.f, .y = 0.f}}};
    std::vector<Edge> edges = {
        {.id = EdgeId{0}, .from = NodeId{0}, .to = NodeId{999}, .length = 10.f, .speed_limit = 10.f},  // no such node
    };
    EXPECT_THROW(RoadGraph(nodes, edges), std::invalid_argument);
}

TEST(RoadGraph, GetNodeThrowsForUnknownId)
{
    auto [nodes, edges] = make_square();
    RoadGraph g{nodes, edges};

    EXPECT_THROW((void)g.get_node(NodeId{999}), std::out_of_range);
}

TEST(RoadGraph, GetEdgeThrowsForUnknownId)
{
    auto [nodes, edges] = make_square();
    RoadGraph g{nodes, edges};

    EXPECT_THROW((void)g.get_edge(EdgeId{999}), std::out_of_range);
}

TEST(RoadGraph, HasNodeAndHasEdgeReflectMembership)
{
    auto [nodes, edges] = make_square();
    RoadGraph g{nodes, edges};

    EXPECT_TRUE(g.has_node(NodeId{0}));
    EXPECT_FALSE(g.has_node(NodeId{999}));
    EXPECT_TRUE(g.has_edge(EdgeId{0}));
    EXPECT_FALSE(g.has_edge(EdgeId{999}));
}