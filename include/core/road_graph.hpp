#pragma once

#include <cstdint>

#include "core/vec2.hpp"

// Road network primitives: nodes, lanes, and the graph connecting them.
// Supports multi-level interchanges via elevation (z) on nodes.

namespace ts {

using NodeId = std::uint32_t;
using LaneId = std::uint32_t;

inline constexpr NodeId kInvalidNode = static_cast<NodeId>(-1);
inline constexpr LaneId kInvalidLane = static_cast<LaneId>(-1);

// Junction point
// `z` is the elevation
struct RoadNode {
    NodeId id{kInvalidNode};
    Vec2D pos{};
    float z{0.f};
};

// A directed road segment connecting two RoadNodes.
struct Lane {
    LaneId id{kInvalidLane};

    NodeId from{kInvalidNode};
    NodeId to{kInvalidNode};

    float length{0.f};             // metres
    float speed_limit{13.9f};      // m/s, default ~50 km/h
    std::uint8_t num_sublanes{1};  // parallel lanes in one direction
};

}  // namespace ts
