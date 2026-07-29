#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "core/types.hpp"
#include "core/vec2d.hpp"

// Road network primitives: nodes, lanes, and the graph connecting them.
// Supports multi-level interchanges via elevation (z) on nodes.

namespace ts {

struct Node {
    NodeId id{kInvalidNode};
    Vec2D pos{};   // TODO: maybe separate Point2D class?
    float z{0.f};  // elevation
};

// A directed road segment connecting two Nodes.
struct Edge {
    EdgeId id{kInvalidEdge};

    NodeId from{kInvalidNode};
    NodeId to{kInvalidNode};

    float length{0.f};           // metres
    float speed_limit{16.7f};    // m/s, default ~60 km/h
    std::uint8_t lane_count{1};  // parallel lanes in one direction
};

// TODO
//
// Lane (a.k.a sublane) on an Edge(one way road segment)
// struct Lane {
//     LaneId id;
//     EdgeId edge;
//     std::uint8_t index;  // 0..lane_count-1
//     // additional properties (speed, allowed vehicle types, etc)
// };

// Adjacency list over Lane objects.
// A* pathfinding.
// Owns Lane and Node data
class RoadGraph {
public:
    RoadGraph(std::vector<Node> nodes, std::vector<Edge> edges);

    // Read-only access
    [[nodiscard]] const Node& get_node(NodeId id) const;
    [[nodiscard]] const Edge& get_edge(EdgeId id) const;

    // All lanes leaving a given node.
    [[nodiscard]] std::span<const EdgeId> outgoing_lanes(NodeId node) const;

    // The node a edge leads into.
    [[nodiscard]] NodeId destination_node(EdgeId edge) const;

    [[nodiscard]] size_t node_count() const noexcept { return nodes_.size(); }
    [[nodiscard]] size_t edge_count() const noexcept { return edges_.size(); }

    // Shortest path (by travel time = length / speed_limit) from src to dst,
    // Return value:
    //  Sequence of edge ids;
    //  Empty vector means src == dst;
    //  nullopt means dst is unreachable from src.
    [[nodiscard]] std::optional<std::vector<EdgeId>> find_route(NodeId src_id, NodeId dst_id) const;

private:
    // Data
    std::vector<Node> nodes_;
    std::vector<Edge> edges_;

    // Quick access indicies
    std::vector<std::uint32_t> outgoing_offsets_;  // размер node_count+1
    std::vector<EdgeId> outgoing_edges_;           // all

    void build_indices();
};

}  // namespace ts
