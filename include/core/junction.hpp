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

using JunctionIdx = std::uint32_t;

struct UnregulatedControl {
    // TODO
};

// Sign-based priority.
struct PriorityControl {
    std::unordered_map<LaneId, std::vector<LaneId>> yields_to;  // Lane yields to others
};

struct TrafficLightControl {
    // TODO
};

using JunctionControl = std::variant<UnregulatedControl, PriorityControl, TrafficLightControl>;

struct Junction {
    NodeId node{kInvalidNode};
    std::vector<LaneId> incoming;
    JunctionControl control{UnregulatedControl{}};
};

// Registry of all junctions on the current map.
class JunctionMap {
public:
    void rebuild(std::vector<Junction> junctions);

    [[nodiscard]] const Junction* find_by_node(NodeId node_id) const;
    [[nodiscard]] const Junction* find_by_lane(LaneId incoming_lane_id) const;

    [[nodiscard]] std::span<const Junction> all() const noexcept { return junctions_; }

private:
    std::vector<Junction> junctions_;
    std::unordered_map<NodeId, std::size_t> index_by_node_;
    std::unordered_map<LaneId, std::size_t> index_by_lane_;
};

// 'Ego' looks at the upcoming junction
// If it must yield, 'ego' imagines a leader at the lane's end.
// So IDM brakes for it exactly as if it were a stopped leader.
// nullopt means the way is free.
[[nodiscard]]
std::optional<idm::LeaderInfo> leader_to_yield(const Vehicle& ego, const Lane& ego_lane, const JunctionMap& junctions,
                                               std::span<const Vehicle> all_vehicles, std::span<const Lane> all_lanes);

}  // namespace ts
