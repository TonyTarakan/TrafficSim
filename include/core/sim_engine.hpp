#pragma once

#include <optional>
#include <vector>

#include "concurrency/triple_buffer.hpp"
#include "core/junction.hpp"
#include "core/road_graph.hpp"
#include "core/vehicle.hpp"

// Owns the simulation world and drives it forward one fixed step at a time.

namespace ts {

struct WorldSnapshot {
    std::vector<Vehicle> vehicles{};
    float sim_time{};
};

struct SimConfig {
    float fixed_dt{1.f / 50.f};  // seconds per tick (default 50 Hz)
};

// TODO: proactive lane-changing ahead of merges/exits (look-ahead MOBIL bias)

class SimEngine {
public:
    explicit SimEngine(SimConfig config = {});

    // Advance the simulation by one step.
    void tick();

    void set_map(std::vector<RoadNode> nodes, std::vector<Lane> lanes);
    void set_junctions(std::vector<Junction> junctions);

    [[nodiscard]] std::optional<std::vector<LaneId>> compute_route(NodeId src, NodeId dst) const;

    [[nodiscard]] std::vector<Vehicle>& vehicles() & noexcept { return vehicles_; }
    [[nodiscard]] const std::vector<Vehicle>& vehicles() const& noexcept { return vehicles_; }
    [[nodiscard]] double sim_time() const noexcept { return sim_time_; }
    TripleBuffer<WorldSnapshot>& world_buffer() & noexcept { return world_buffer_; }

private:
    [[nodiscard]] const Lane* find_lane(LaneId id) const;

    SimConfig config_;
    std::vector<Vehicle> vehicles_;

    // TODO: do we need lanes_/nodes_ or it can be fully replaced by graph_?
    std::vector<Lane> lanes_;
    std::vector<RoadNode> nodes_;
    RoadGraph graph_;
    JunctionMap junctions_;

    double sim_time_{0.0};

    TripleBuffer<WorldSnapshot> world_buffer_;
};

}  // namespace ts
