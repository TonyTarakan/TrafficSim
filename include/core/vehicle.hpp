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

    Vec2 position{};
    float speed{0.f};  // m/s, always >= 0

    // Which road segment this vehicle is on, and which parallel sub-lane
    // within that segment (0 = rightmost). A Lane with num_sublanes > 1
    // is treated as several independent traffic streams for IDM purposes;
    // lane-changing moves a vehicle between sublane_idx values on the
    // same lane_id. See core/lane_change.hpp for the MOBIL model that
    // will decide when this happens.
    LaneId lane_id{};
    std::uint8_t sublane_idx{0};
};

}  // namespace ts
