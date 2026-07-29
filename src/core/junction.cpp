#include "core/junction.hpp"

#include <quill/LogMacros.h>

#include <algorithm>

#include "core/log.hpp"
#include "core/road_graph.hpp"
#include "core/types.hpp"

namespace ts {

bool TrafficLightControl::is_green(EdgeId lane) const
{
    if (phases.empty()) return true;

    const SignalPhase& current = phases[phase_idx % phases.size()];
    return (std::ranges::find(current.green_lanes, lane) != current.green_lanes.end());
}

void JunctionMap::rebuild(std::vector<Junction> junctions, const RoadGraph& graph)
{
    junctions_ = std::move(junctions);
    index_by_node_.clear();
    index_by_lane_.clear();

    for (std::size_t i = 0; i < junctions_.size(); ++i) {
        Junction& j = junctions_[i];
        index_by_node_[j.node_id] = i;

        if (j.node_id.get() < graph.node_count()) {
            const Node& node = graph.get_node(j.node_id);
            j.pos = node.pos;
        }
        else {
            LOG_WARNING(log::get(), "junction references unknown node {}", j.node_id.get());
        }

        j.lines_from.clear();
        for (EdgeId lane_id : j.incoming) {
            index_by_lane_[lane_id] = i;
            if (lane_id.get() >= graph.edge_count()) {
                LOG_WARNING(log::get(), "junction {}: unknown lane {}", j.node_id.get(), lane_id.get());
                continue;
            }
            const Edge& edge = graph.get_edge(lane_id);
            if (edge.from.get() < graph.node_count()) {
                const Node& from_node = graph.get_node(edge.from);
                j.lines_from[lane_id] = from_node.pos;
            }
            else {
                LOG_WARNING(log::get(), "junction {}: edge {} has invalid from-node {}", j.node_id.get(), lane_id.get(),
                            edge.from.get());
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

const Junction* JunctionMap::find_by_lane(EdgeId incoming_lane) const
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
                          const RoadGraph& graph)
{
    return std::ranges::any_of(all_vehicles, [&](const Vehicle& other) {
        if (other.id == ego_id) return false;
        if (other.edge_id.get() >= graph.edge_count()) return false;
        const Edge& e = graph.get_edge(other.edge_id);
        return (e.from == node_id && other.offset < kClearanceWindow);
    });
}

// Check if someone is on 'rival_lane' and close enough
bool someone_is_approaching(const Edge& rival_lane, std::span<const Vehicle> all_vehicles)
{
    for (const auto& other : all_vehicles) {
        if (other.edge_id != rival_lane.id) continue;

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
bool yields_to_rivals(const Junction& junction, VehicleId ego_id, std::span<const EdgeId> rival_lanes,
                      std::span<const Vehicle> all_vehicles, const RoadGraph& graph)
{
    if (rival_lanes.empty()) {
        return false;
    }

    if (junction_is_occupied(junction.node_id, ego_id, all_vehicles, graph)) {
        return true;
    }

    for (EdgeId rid : rival_lanes) {
        if (rid.get() >= graph.edge_count()) continue;
        const Edge& e = graph.get_edge(rid);
        if (someone_is_approaching(e, all_vehicles)) return true;
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
std::vector<EdgeId> right_hand_rivals(const Junction& junction, const Edge& ego_lane)
{
    std::vector<EdgeId> rivals;

    auto ego_from_it = junction.lines_from.find(ego_lane.id);
    if (ego_from_it == junction.lines_from.end()) {
        return rivals;  // no cached geometry for this approach, nobody to yield to
    }

    for (EdgeId other_id : junction.incoming) {
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

std::optional<idm::LeaderInfo> leader_to_yield(const Vehicle& ego, const Edge& ego_lane, const JunctionMap& junctions,
                                               std::span<const Vehicle> all_vehicles, const RoadGraph& graph)
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
            return yields_to_rivals(*junction, ego.id, rivals, all_vehicles, graph);
        },

        [&](const TrafficLightControl& control) { 
            return !control.is_green(ego_lane.id); 
        },

        [&](const PriorityControl& control) {
            auto it = control.yields_to.find(ego_lane.id);
            if (it == control.yields_to.end()) return false;
            
            return yields_to_rivals(*junction, ego.id, it->second, all_vehicles, graph);
        }

    }, junction->control);
    // clang-format on

    if (must_yield) {
        return idm::LeaderInfo{.gap = distance_to_stop, .dv = ego.speed};
    }

    return std::nullopt;
}

}  // namespace ts
