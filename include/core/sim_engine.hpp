#pragma once

#include <vector>

#include "concurrency/triple_buffer.hpp"
#include "core/road_graph.hpp"
#include "core/vehicle.hpp"

// Owns the simulation world and drives it forward one fixed step at a time.
//
// Deliberately minimal so far: only Lane::num_sublanes is needed (for
// lane-change bounds checking), not the full RoadGraph — routing across
// lanes comes in a later step.

namespace ts {

struct WorldSnapshot {
    std::vector<Vehicle> vehicles{};
    float sim_time{};
};

struct SimConfig {
    float fixed_dt{1.f / 50.f};  // seconds per tick (default 50 Hz)
};

class SimEngine {
public:
    explicit SimEngine(SimConfig config = {});

    // Advance the simulation by one fixed_dt step.
    void tick();

    void set_lanes(std::vector<Lane> lanes) { lanes_ = std::move(lanes); }

    [[nodiscard]] std::vector<Vehicle>& vehicles() & noexcept { return vehicles_; }
    [[nodiscard]] const std::vector<Vehicle>& vehicles() const& noexcept { return vehicles_; }
    [[nodiscard]] double sim_time() const noexcept { return sim_time_; }
    TripleBuffer<WorldSnapshot>& world_buffer() & noexcept { return world_buffer_; }

private:
    [[nodiscard]] const Lane* find_lane(LaneId id) const;

    SimConfig config_;
    std::vector<Vehicle> vehicles_;
    std::vector<Lane> lanes_;
    double sim_time_{0.0};

    TripleBuffer<WorldSnapshot> world_buffer_;
};

}  // namespace ts
