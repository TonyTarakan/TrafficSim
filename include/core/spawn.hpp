#pragma once

#include <vector>

#include "core/types.hpp"

namespace ts {

struct SpawnPoint {
    NodeId origin;
    std::vector<NodeId> destinations;
    float spawn_rate{1.f};  // vehicles per second or tick counter?
    // TODO(later):
    // - std::vector<???> dst_probability;
    // - VehicleType distribution
    // - start_time;
    // - end_time;
};

// TODO
// - spawn+despawn logic
// - config

}  // namespace ts
