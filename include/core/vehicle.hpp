#pragma once

#include <cstdint>

#include "core/vec2.hpp"

namespace ts {

using VehicleId = std::uint32_t;

enum class VehicleType : std::uint8_t {
    Car,
    Truck,
    Bus,
    Motorcycle,
};

struct Vehicle {
    VehicleId id{};
    VehicleType type{VehicleType::Car};

    Vec2 position{};
    float speed{0.f};  // m/s, always >= 0
};

}  // namespace ts