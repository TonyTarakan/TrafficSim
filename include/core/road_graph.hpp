#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "core/types.hpp"
#include "core/vec2d.hpp"

// Road network primitives: nodes, edges, lanes and the graph connecting them.
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
// Owns Edge and Node data.
//
// NodeId/EdgeId MUST be dense array indices: nodes[i].id == i and
// edges[i].id == i. That's what lets every lookup below be a raw array
// access instead of a hash lookup -- the only producer of Node/Edge data
// today is hand-authored (main.cpp), which already assigns ids this way.
// The constructor enforces it and throws std::invalid_argument otherwise,
// so a violation fails loud at construction, not silently later.
//
// If a real external source ever needs non-dense ids (OSM import, a map
// editor with deletions), that's the right place to remap to dense ids
// before handing data to RoadGraph -- not something every lookup here
// should pay for until it's actually needed. osm_importer.hpp is parked
// as a far-future stub for exactly this reason; revisit this decision
// once it's real.
class RoadGraph final {
public:
    RoadGraph(std::vector<Node> nodes, std::vector<Edge> edges);

    // Read-only access. Throws std::out_of_range if the id isn't in this graph.
    [[nodiscard]] const Node& get_node(NodeId id) const;
    [[nodiscard]] const Edge& get_edge(EdgeId id) const;

    [[nodiscard]] bool has_node(NodeId id) const noexcept { return id.get() < nodes_.size(); }
    [[nodiscard]] bool has_edge(EdgeId id) const noexcept { return id.get() < edges_.size(); }

    // All edges leaving a given node.
    [[nodiscard]] std::span<const EdgeId> outgoing_edges(NodeId node) const;

    // The node a edge leads into.
    [[nodiscard]] NodeId destination_node(EdgeId edge) const;  // TODO: do we need this func?

    [[nodiscard]] size_t node_count() const noexcept { return nodes_.size(); }
    [[nodiscard]] size_t edge_count() const noexcept { return edges_.size(); }

    // Shortest path (by travel time = length / speed_limit) from src to dst,
    // Return value:
    //  Sequence of edge ids;
    //  Empty vector means src == dst;
    //  nullopt means dst is unreachable from src.
    [[nodiscard]] std::optional<std::vector<EdgeId>> find_route(NodeId src_id, NodeId dst_id) const;

private:
    // Data. Position i MUST hold the entity with id == i (see class comment).
    const std::vector<Node> nodes_;
    const std::vector<Edge> edges_;

    // Quick access indicies
    std::vector<std::uint32_t> outgoing_offsets_;  // размер node_count+1
    std::vector<EdgeId> outgoing_edges_;           // all

    void build_indices();
};

}  // namespace ts
