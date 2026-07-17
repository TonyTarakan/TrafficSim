#include "core/vehicle_params.hpp"

namespace ts {

// TODO: looks like it can be done with constexpr context or templates
idm::VehicleParams default_params(VehicleType type) noexcept
{
    switch (type) {
        case VehicleType::Car:
            return {
                .desired_speed = 18.0f, .max_accel = 2.0f, .comfy_decel = 3.0f, .min_gap = 2.0f, .time_headway = 1.5f};
        case VehicleType::Truck:
            return {
                .desired_speed = 13.0f, .max_accel = 0.8f, .comfy_decel = 2.0f, .min_gap = 4.0f, .time_headway = 2.0f};
        case VehicleType::Bus:
            return {
                .desired_speed = 13.0f, .max_accel = 1.0f, .comfy_decel = 1.5f, .min_gap = 4.0f, .time_headway = 2.0f};
        case VehicleType::Motorcycle:
            return {
                .desired_speed = 20.0f, .max_accel = 3.5f, .comfy_decel = 2.5f, .min_gap = 1.0f, .time_headway = 1.0f};
    }
    return {};
}

}  // namespace ts
