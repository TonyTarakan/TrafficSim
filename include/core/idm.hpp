#pragma once

#include <optional>

// Intelligent Driver Model — car-following behavior.
// Reference: Treiber, Hennecke, Helbing (2000).

// TODO: use user defined literals e.g. mp-units

namespace ts::idm {

// Per-vehicle-type parameters for different vehicle types (car, truck, bus)
struct VehicleParams {
    float desired_speed{15.0f};  // v0, m/s (54 km/h)
    float max_accel{2.0f};       // a,  m/s^2
    float comfy_decel{3.0f};     // b,  m/s^2
    float min_gap{10.0f};        // s0, m
    float time_headway{1.5f};    // T,  s
};

// Gap and relative speed to the vehicle ahead.
struct LeaderInfo {
    float gap;  // net bumper-to-bumper distance, metres (>= 0)
    float dv;   // vehicle.speed - leader.speed (positive => closing in)
};

// IDM formula:
//
//   a = a_max * [ 1 - (v/v0)^4 - (s*(v,dv) / gap)^2 ]
//
//   s*(v, dv) = s0 + v*T + (v*dv) / (2*sqrt(a_max*b))
//
// nullopt leader_info means free road ahead
[[nodiscard]]
float accelerate(const VehicleParams& p, float curr_speed,
                 std::optional<LeaderInfo> leader_info = std::nullopt) noexcept;

}  // namespace ts::idm
