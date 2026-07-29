#pragma once

#include <optional>
#include <span>
#include <unordered_map>
#include <variant>
#include <vector>

#include "core/idm.hpp"
#include "core/road_graph.hpp"
#include "core/vehicle.hpp"

// Control policy is a Strategy, selected per node via JunctionControl:
//   - UncontrolledControl: right hand priority
//   - PriorityControl:     explicit road sign priority
//   - TrafficLightControl: fixed-time phased signal

namespace ts {

struct UnregulatedControl {
    // TODO
};

// Sign-based priority.
struct PriorityControl {
    std::unordered_map<EdgeId, std::vector<EdgeId>> yields_to;  // Edge yields to others
};

// Only one edge has green light for duration seconds
struct SignalPhase {
    std::vector<EdgeId> green_edges;
    float duration{30.f};  // seconds
};

// Cyclic green light
struct TrafficLightControl {
    std::vector<SignalPhase> phases;
    std::size_t phase_idx{0};
    float phase_elapsed{0.f};  // seconds into the current phase

    // No phases configured => treat as always-green
    // TODO: add support for flashing yellow (unregulated/priority fallback)
    [[nodiscard]] bool is_green(EdgeId edge) const;
};

using JunctionControl = std::variant<UnregulatedControl, PriorityControl, TrafficLightControl>;

struct Junction {
    NodeId node_id{kInvalidNode};
    std::vector<EdgeId> incoming;
    JunctionControl control{UnregulatedControl{}};

    // Geometry cache, filled in by JunctionMap::rebuild()
    Vec2D pos{};
    std::unordered_map<EdgeId, Vec2D> lines_from{};
};

// Registry of all junctions on the current map.
class JunctionMap {
public:
    void rebuild(std::vector<Junction> junctions, const RoadGraph& graph);

    [[nodiscard]] const Junction* find_by_node(NodeId node_id) const;
    [[nodiscard]] const Junction* find_by_edge(EdgeId incoming_edge_id) const;

    // Advance every tick by dt seconds.
    void advance_signals(float dt);

    [[nodiscard]] std::span<const Junction> all() const noexcept { return junctions_; }

private:
    std::vector<Junction> junctions_;
    std::unordered_map<NodeId, std::size_t> index_by_node_;
    std::unordered_map<EdgeId, std::size_t> index_by_edge_;
};

// 'Ego' looks at the upcoming junction
// If it must yield, 'ego' imagines a leader at the edge's end.
// So IDM brakes for it exactly as if it were a stopped leader.
// nullopt means the way is free.
[[nodiscard]]
std::optional<idm::LeaderInfo> leader_to_yield(const Vehicle& ego, const Edge& ego_edge, const JunctionMap& junctions,
                                               std::span<const Vehicle> all_vehicles, const RoadGraph& graph);

}  // namespace ts
