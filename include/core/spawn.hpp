#pragma once

#include <vector>

#include "core/sim_engine.hpp"
#include "core/types.hpp"

namespace ts {

struct Destination {
    NodeId node;
    float weight;  // probability
};

struct SpawnPoint {
    NodeId origin;

    float spawn_rate{1.f};  // vehicles per second

    // TODO(later):
    // - VehicleType distribution
    // - Destination distribution
};

// TODO
class SpawnSystem {
public:
    explicit SpawnSystem(SimEngine& engine);
    void update(float dt);
    void spawn_once(NodeId from, NodeId to, int count = 1);

private:
    std::vector<SpawnPoint> spawn_points_;
};

// TODO
// - spawn+despawn logic
// - config
// - random destination
// - random vehicle

}  // namespace ts
