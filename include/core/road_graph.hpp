#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

#include "core/vec2.hpp"

// Road network primitives: nodes, lanes, and the graph connecting them.
// Supports multi-level interchanges via elevation (z) on nodes.

namespace ts {

// TODO: strong types
using NodeId = std::uint32_t;
using LaneId = std::uint32_t;

inline constexpr NodeId kInvalidNode = static_cast<NodeId>(-1);
inline constexpr LaneId kInvalidLane = static_cast<LaneId>(-1);

// Junction point
struct RoadNode {
    NodeId id{kInvalidNode};
    Vec2D pos{};   // TODO: maybe separate Point2D class?
    float z{0.f};  // elevation
};

// A directed road segment connecting two RoadNodes.
struct Lane {
    LaneId id{kInvalidLane};

    NodeId from{kInvalidNode};
    NodeId to{kInvalidNode};

    float length{0.f};             // metres
    float speed_limit{16.7f};      // m/s, default ~60 km/h
    std::uint8_t num_sublanes{1};  // parallel lanes in one direction
};

// Adjacency list over Lane objects.
// A* pathfinding.
// Keeps its own copies of Lane/RoadNode data
class RoadGraph {
    // TODO: rebuild or construct?
public:
    void rebuild(std::span<const RoadNode> nodes, std::span<const Lane> lanes);

    // All lanes leaving a given node.
    // WARNING: the returned span dangles after the next rebuild()
    [[nodiscard]] std::span<const LaneId> outgoing_lanes(NodeId node) const;

    // The node a lane leads into.
    [[nodiscard]] NodeId destination_node(LaneId lane) const;

    // Shortest path (by travel time = length / speed_limit) from src to dst,
    // Return value:
    //  Sequence of lane ids;
    //  Empty vector means src == dst;
    //  nullopt means dst is unreachable from src.
    [[nodiscard]] std::optional<std::vector<LaneId>> find_route(NodeId src_id, NodeId dst_id) const;

private:
    // TODO: try another containers later
    std::unordered_map<NodeId, std::vector<LaneId>> adjacency_;  // All Lanes 'growing' from the node
    std::unordered_map<LaneId, NodeId> lane_dest_;               // Lane destination
    std::unordered_map<LaneId, Lane> lanes_by_id_;
    std::unordered_map<NodeId, RoadNode> nodes_by_id_;
};

}  // namespace ts
