#pragma once

#include <cstdint>

#include "core/idm.hpp"
#include "core/road_graph.hpp"

namespace ts {

using VehicleId = std::uint32_t;

enum class VehicleType : std::uint8_t {
    Car,
    Truck,
    Bus,
    Motorcycle,
    // TODO: add trams later
};

// Minimal per-vehicle state. Movement logic (IDM) and lane-changing (MOBIL)
// are separate modules — this struct only holds data.
struct Vehicle {
    VehicleId id{};
    VehicleType type{VehicleType::Car};
    idm::VehicleParams idm_params{};

    Vec2D position{};  // TODO: maybe separate Point2D class?
    float speed{0.f};  // m/s, always >= 0

    // Which road segment the vehicle is on.
    LaneId lane_id{};

    // How far are we from the lane's start.
    float offset{0.f};  // m

    // Parallel sub-lane within the segment (0 = rightmost).
    // MOBIL works between them, IDM works per sublane
    int sublane_idx{0};

    // Seconds remaining before another lane change may be considered.
    // Without this, MOBIL can cause lane change every tick.
    float lane_change_cooldown{1.f};
};

}  // namespace ts
