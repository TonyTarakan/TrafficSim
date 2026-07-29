#pragma once

#include <optional>
#include <vector>

#include "concurrency/thread_pool.hpp"
#include "concurrency/triple_buffer.hpp"
#include "core/junction.hpp"
#include "core/road_graph.hpp"
#include "core/types.hpp"
#include "core/vehicle.hpp"

// Owns the simulation world and drives it forward one fixed step at a time.

namespace ts {

struct WorldSnapshot {
    std::vector<Vehicle> vehicles{};
    double sim_time{};
};

struct SimConfig {
    float fixed_dt{1.f / 50.f};  // seconds per tick (default 50 Hz)
    std::size_t num_threads{0};  // 0 = auto-detect (hardware_concurrency)
};

// TODO: proactive lane-changing ahead of merges/exits (look-ahead MOBIL bias)

class SimEngine {
public:
    explicit SimEngine(SimConfig config = {}) : config_(config), pool_(config.num_threads) {}

    // Advance the simulation by one step.
    void tick();

    // Take nodes and lanes ownership
    void set_map(std::vector<Node> nodes, std::vector<Edge> edges);

    // TODO: resolve dependencies
    // 'set_junctions' requires set_map() to have been called first.
    // Junction geometry is resolved against the node/lane data.
    void set_junctions(std::vector<Junction> junctions);

    [[nodiscard]] std::optional<std::vector<EdgeId>> compute_route(NodeId src, NodeId dst) const;

    [[nodiscard]] std::vector<Vehicle>& vehicles() & noexcept { return vehicles_; }
    [[nodiscard]] const std::vector<Vehicle>& vehicles() const& noexcept { return vehicles_; }
    [[nodiscard]] double sim_time() const noexcept { return sim_time_; }
    [[nodiscard]] std::size_t worker_count() const noexcept { return pool_.thread_count(); }
    TripleBuffer<WorldSnapshot>& world_buffer() & noexcept { return world_buffer_; }

private:
    [[nodiscard]] const Edge* find_edge(EdgeId id) const;

    SimConfig config_;
    std::vector<Vehicle> vehicles_;

    std::unique_ptr<RoadGraph> graph_;  // main data storage
    JunctionMap junctions_;

    double sim_time_{0.0};

    ThreadPool pool_;
    TripleBuffer<WorldSnapshot> world_buffer_;
};

}  // namespace ts
