#pragma once

#include "core/idm.hpp"
#include "core/vehicle.hpp"

// Reasonable default IDM parameters per vehicle type.
// The VehicleParams type itself lives in core/idm.hpp — this module only
// provides sensible factory defaults, it doesn't own the type.

namespace ts {

[[nodiscard]] idm::VehicleParams default_params(VehicleType type) noexcept;

}  // namespace ts
