#pragma once

#include <vector>

#include "core/vehicle.hpp"

// Owns the simulation world and drives it forward one fixed step at a time.
//

namespace ts {

struct SimConfig {
    float fixed_dt{1.f / 50.f};  // seconds per tick (default 50 Hz)
};

class SimEngine {
public:
    explicit SimEngine(SimConfig config = {});

    // Advance the simulation by one fixed_dt step.
    void tick();

    [[nodiscard]] std::vector<Vehicle>& vehicles() & noexcept { return vehicles_; }
    [[nodiscard]] const std::vector<Vehicle>& vehicles() const& noexcept { return vehicles_; }
    [[nodiscard]] double sim_time() const noexcept { return sim_time_; }

private:
    SimConfig config_;
    std::vector<Vehicle> vehicles_;
    double sim_time_{0.0};
};

}  // namespace ts
