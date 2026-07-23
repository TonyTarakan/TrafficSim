#include "core/junction.hpp"

#include <quill/LogMacros.h>

#include <algorithm>

namespace ts {

void JunctionMap::rebuild(std::vector<Junction> junctions)
{
    junctions_ = std::move(junctions);
    index_by_node_.clear();
    index_by_lane_.clear();

    for (std::size_t i = 0; i < junctions_.size(); ++i) {
        index_by_node_[junctions_[i].node] = i;
        for (LaneId lane : junctions_[i].incoming) {
            index_by_lane_[lane] = i;
        }
    }
}

const Junction* JunctionMap::find_by_node(NodeId node) const
{
    auto it = index_by_node_.find(node);
    if (it == index_by_node_.end()) return nullptr;

    return &junctions_[it->second];
}

const Junction* JunctionMap::find_by_lane(LaneId incoming_lane) const
{
    auto it = index_by_lane_.find(incoming_lane);
    if (it == index_by_lane_.end()) return nullptr;

    return &junctions_[it->second];
}

namespace {

// Simulation-tuning constants.
constexpr float kApproachWindow = 40.f;   // m   - start caring about the junction this far out
constexpr float kClearanceWindow = 10.f;  // m   - safety gap past the junction
constexpr float kCriticalGapS = 4.f;      // s   - minimum accepted gap in higher-priority traffic

// Check if someone is just behind the crossroad/junction
// We don't track turn-specific paths through the node yet
bool junction_is_occupied(NodeId node_id, VehicleId ego_id, std::span<const Vehicle> all_vehicles,
                          std::span<const Lane> all_lanes)
{
    return std::ranges::any_of(all_vehicles, [&](const Vehicle& other) {
        if (other.id == ego_id) return false;

        return std::ranges::any_of(all_lanes, [&](const Lane& l) {
            return (l.id == other.lane_id && l.from == node_id && other.offset < kClearanceWindow);
        });
    });
}

// Check if someone is on 'rival_lane' and close enough
bool someone_is_approaching(const Lane& rival_lane, std::span<const Vehicle> all_vehicles)
{
    for (const auto& other : all_vehicles) {
        if (other.lane_id != rival_lane.id) continue;

        float rival_distance = rival_lane.length - other.offset;
        if (rival_distance < 0.f) continue;              // past the line
        if (rival_distance > kApproachWindow) continue;  // too far to care

        constexpr float kMinSpeed = 0.1f;
        float rival_approach_time = rival_distance / std::max(other.speed, kMinSpeed);
        if (rival_approach_time < kCriticalGapS) {
            return true;
        }
    }
    return false;
}

// Box rule + gap acceptance against an explicit set of rival lanes, shared
// by PriorityControl (rivals from the sign) and UnregulatedControl
// (rivals worked out from geometry).
bool yields_to_rivals(const Junction& junction, VehicleId ego_id, std::span<const LaneId> rival_lanes,
                      std::span<const Vehicle> all_vehicles, std::span<const Lane> all_lanes)
{
    if (rival_lanes.empty()) {
        return false;
    }

    if (junction_is_occupied(junction.node, ego_id, all_vehicles, all_lanes)) {
        return true;
    }

    for (LaneId rival_id : rival_lanes) {
        auto rival_lane_it = std::ranges::find(all_lanes, rival_id, &Lane::id);
        if (rival_lane_it == all_lanes.end()) continue;

        if (someone_is_approaching(*rival_lane_it, all_vehicles)) {
            return true;
        }
    }

    return false;
}

template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};

}  // namespace

std::optional<idm::LeaderInfo> leader_to_yield(const Vehicle& ego, const Lane& ego_lane, const JunctionMap& junctions,
                                               std::span<const Vehicle> all_vehicles, std::span<const Lane> all_lanes)
{
    float distance_to_stop = ego_lane.length - ego.offset;
    if (distance_to_stop < 0.f) return std::nullopt;              // already past the line
    if (distance_to_stop > kApproachWindow) return std::nullopt;  // too far away to care

    const Junction* junction = junctions.find_by_lane(ego_lane.id);
    if (!junction) {
        // TODO: maybe UnregulatedControl as default?
        return std::nullopt;  // this lane doesn't feed a controlled junction
    }

    // clang-format off
    bool must_yield = std::visit(overloaded{

        [&](const UnregulatedControl& ) {
            // TODO: implement
            return false;
        },

        [&](const TrafficLightControl& ) {
            // TODO: implement
            return false;
        },

        [&](const PriorityControl& control) {
            auto it = control.yields_to.find(ego_lane.id);
            if (it == control.yields_to.end()) return false;
            
            return yields_to_rivals(*junction, ego.id, it->second, all_vehicles, all_lanes);
        }

    }, junction->control);
    // clang-format on

    if (must_yield) {
        return idm::LeaderInfo{.gap = distance_to_stop, .dv = ego.speed};
    }

    return std::nullopt;
}

}  // namespace ts
