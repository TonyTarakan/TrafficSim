#pragma once

#include <cstdint>

#include "core/vec2.hpp"

namespace ts {

using VehicleId = std::uint32_t;

// Placeholder until core/road_graph.hpp defines the real Lane/RoadNode types.
using LaneId = std::uint32_t;

enum class VehicleType : std::uint8_t {
    Car,
    Truck,
    Bus,
    Motorcycle,
};

// Minimal per-vehicle state. Movement logic (IDM) and lane-changing (MOBIL)
// are separate modules — this struct only holds data.
struct Vehicle {
    VehicleId id{};
    VehicleType type{VehicleType::Car};

    Vec2D position{};
    float speed{0.f};  // m/s, always >= 0

    // Which road segment the vehicle is on
    LaneId lane_id{};
    // Parallel sub-lane within the segment (0 = rightmost).
    // MOBIL works between them
    // IDM works per sublane
    std::uint8_t sublane_idx{0};
};

}  // namespace ts
