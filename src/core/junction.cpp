#include "core/junction.hpp"

#include <quill/LogMacros.h>

#include <algorithm>

#include "core/log.hpp"
#include "core/road_graph.hpp"
#include "core/types.hpp"

namespace ts {

bool TrafficLightControl::is_green(LaneId lane) const
{
    if (phases.empty()) return true;

    const SignalPhase& current = phases[phase_idx % phases.size()];
    return (std::ranges::find(current.green_lanes, lane) != current.green_lanes.end());
}

namespace {

const RoadNode* find_node(NodeId id, std::span<const RoadNode> nodes)
{
    auto it = std::ranges::find(nodes, id, &RoadNode::id);
    return (it != nodes.end()) ? &*it : nullptr;
}

const Lane* find_lane(LaneId id, std::span<const Lane> lanes)
{
    auto it = std::ranges::find(lanes, id, &Lane::id);
    return (it != lanes.end()) ? &*it : nullptr;
}

}  // namespace

void JunctionMap::rebuild(std::vector<Junction> junctions, std::span<const RoadNode> nodes, std::span<const Lane> lanes)
{
    junctions_ = std::move(junctions);
    index_by_node_.clear();
    index_by_lane_.clear();

    for (std::size_t i = 0; i < junctions_.size(); ++i) {
        Junction& junction = junctions_[i];
        index_by_node_[junction.node_id] = i;

        if (const RoadNode* node = find_node(junction.node_id, nodes)) {
            junction.pos = node->pos;
        }
        else {
            LOG_WARNING(log::get(), "junction references unknown node {}", junction.node_id.get());
        }

        junction.lines_from.clear();
        for (LaneId lane_id : junction.incoming) {
            index_by_lane_[lane_id] = i;

            const Lane* lane = find_lane(lane_id, lanes);
            const RoadNode* from_node = lane ? find_node(lane->from, nodes) : nullptr;
            if (from_node) {
                junction.lines_from[lane_id] = from_node->pos;
            }
            else {
                LOG_WARNING(log::get(), "junction {}: can't resolve approach geometry for lane {}",
                            junction.node_id.get(), lane_id.get());
            }
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

// TODO: maybe ticks, not seconds?
void JunctionMap::advance_signals(float dt)
{
    for (auto& junction : junctions_) {
        auto* light_ctl = std::get_if<TrafficLightControl>(&junction.control);
        if (!light_ctl) continue;
        if (light_ctl->phases.empty()) continue;

        light_ctl->phase_elapsed += dt;
        const SignalPhase& current = light_ctl->phases[light_ctl->phase_idx % light_ctl->phases.size()];
        if (light_ctl->phase_elapsed >= current.duration) {
            light_ctl->phase_elapsed -= current.duration;
            light_ctl->phase_idx = (light_ctl->phase_idx + 1) % light_ctl->phases.size();
            LOG_INFO(log::get(), "junction {} signal advanced to phase {}", junction.node_id.get(),
                     light_ctl->phase_idx);
        }
    }
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

    if (junction_is_occupied(junction.node_id, ego_id, all_vehicles, all_lanes)) {
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

bool approaches_from_the_right(Vec2D junction_pos, Vec2D ego_from, Vec2D rival_from)
{
    constexpr float kRightHandEpsilon = 1e-3f;  // avoids ties on parallel courses

    Vec2D ego_heading = junction_pos - ego_from;
    Vec2D ego_right{.x = -ego_heading.y, .y = ego_heading.x};

    Vec2D rival_origin_dir = rival_from - junction_pos;  // where the rival is coming from, relative to us
    return ego_right.dot_prod(rival_origin_dir) > kRightHandEpsilon;
}

// Uses cached geometry of lanes.
std::vector<LaneId> right_hand_rivals(const Junction& junction, const Lane& ego_lane)
{
    std::vector<LaneId> rivals;

    auto ego_from_it = junction.lines_from.find(ego_lane.id);
    if (ego_from_it == junction.lines_from.end()) {
        return rivals;  // no cached geometry for this approach, nobody to yield to
    }

    for (LaneId other_id : junction.incoming) {
        if (other_id == ego_lane.id) continue;

        auto rival_from_it = junction.lines_from.find(other_id);
        if (rival_from_it == junction.lines_from.end()) continue;

        if (approaches_from_the_right(junction.pos, ego_from_it->second, rival_from_it->second)) {
            rivals.push_back(other_id);
        }
    }

    return rivals;
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
            auto rivals = right_hand_rivals(*junction, ego_lane);
            return yields_to_rivals(*junction, ego.id, rivals, all_vehicles, all_lanes);
        },

        [&](const TrafficLightControl& control) { 
            return !control.is_green(ego_lane.id); 
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
