#include "core/sim_engine.hpp"

#include <algorithm>
#include <limits>

#include "core/idm.hpp"
#include "core/lane_change.hpp"

namespace ts {

namespace {

// Finds the closest vehicle ahead of 'ego' on the same lane and sublane.
// TODO: Take vehicle size into account, centre-to-centre for now
std::optional<idm::LeaderInfo> find_leader(const Vehicle& ego, std::span<const Vehicle> all_vehicles)
{
    std::optional<idm::LeaderInfo> best;
    float best_gap = std::numeric_limits<float>::infinity();

    for (const auto& other : all_vehicles) {
        if (other.id == ego.id) continue;
        if (other.lane_id != ego.lane_id) continue;
        if (other.sublane_idx != ego.sublane_idx) continue;
        if (other.offset <= ego.offset) continue;  // behind us, not a leader

        float gap = other.offset - ego.offset;
        if (gap < best_gap) {
            best_gap = gap;
            best = idm::LeaderInfo{.gap = gap, .dv = ego.speed - other.speed};
        }
    }

    return best;
}

}  // namespace

SimEngine::SimEngine(SimConfig config) : config_(config) {}

void SimEngine::set_map(std::vector<RoadNode> nodes, std::vector<Lane> lanes)
{
    graph_.rebuild(nodes, lanes);
    lanes_ = std::move(lanes);
}

std::optional<std::vector<LaneId>> SimEngine::compute_route(NodeId src, NodeId dst) const
{
    return graph_.find_route(src, dst);
}

const Lane* SimEngine::find_lane(LaneId id) const
{
    auto it = std::ranges::find_if(lanes_, [&](const Lane& l) { return l.id == id; });
    return (it != lanes_.end()) ? &*it : nullptr;
}

void SimEngine::tick()
{
    // --- lane-change decisions, on the pre-tick snapshot ---
    // Must happen before any IDM mutation below: MOBIL needs to see the
    // same consistent "world" for every vehicle, same reasoning as the
    // leader snapshot further down. Only sublane_idx is mutated here —
    // speed/offset stay untouched until the IDM pass, so that pass's own
    // snapshot-then-apply logic is unaffected.
    constexpr float kLaneChangeCooldownS = 3.f;  // seconds before re-evaluating // TODO: make random
    using namespace lane_change;

    std::vector<Turn> sublane_deltas(vehicles_.size(), Turn::NONE);
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        if (vehicles_[i].lane_change_cooldown > 0.f) {
            continue;  // still cooling down from a recent switch — skip decide()
        }
        const Lane* lane = find_lane(vehicles_[i].lane_id);
        std::uint8_t num_sublanes = lane ? lane->num_sublanes : 1;
        sublane_deltas[i] = decide(vehicles_[i], vehicles_, num_sublanes);
    }
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        Vehicle& v = vehicles_[i];
        if (sublane_deltas[i] != Turn::NONE) {
            int new_sublane = static_cast<int>(v.sublane_idx) + static_cast<int>(sublane_deltas[i]);
            v.sublane_idx = static_cast<std::uint8_t>(new_sublane);
            v.lane_change_cooldown = kLaneChangeCooldownS;
        }
        else {
            v.lane_change_cooldown = std::max(0.f, v.lane_change_cooldown - config_.fixed_dt);
        }
    }

    // --- car-following (IDM), on the (now lane-change-applied) snapshot ---
    // Snapshot leaders before mutating anything
    // TODO: this is O(N*N). Optimize!
    std::vector<std::optional<idm::LeaderInfo>> leaders(vehicles_.size());
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        leaders[i] = find_leader(vehicles_[i], vehicles_);
    }

    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        Vehicle& v = vehicles_[i];

        float accel = idm::accelerate(v.idm_params, v.speed, leaders[i]);
        float new_speed = std::max(0.f, v.speed + accel * config_.fixed_dt);  // speed >= 0

        v.offset += 0.5f * (v.speed + new_speed) * config_.fixed_dt;  // linear accel distance
        v.speed = new_speed;
    }

    // --- lane transitions: advance along the route when a lane ends ---
    // Runs after IDM integration (needs this tick's updated offset to know
    // whether we've actually run off the end of the current lane).
    for (auto& v : vehicles_) {
        const Lane* cur_lane = find_lane(v.lane_id);
        if (!cur_lane || v.offset <= cur_lane->length) {
            continue;  // still within the current lane, nothing to do
        }

        float overflow = v.offset - cur_lane->length;

        if (v.route_idx + 1 >= v.route.size()) {
            // Workaround. No despawn logic yet -- loop back to the
            // start so demo vehicles keep driving indefinitely.
            v.route_idx = 0;
        }
        else {
            ++v.route_idx;
        }

        if (v.route.empty()) {
            continue;  // no route at all -- stay put at the lane's end
        }

        v.lane_id = v.route[v.route_idx];
        v.offset = overflow;

        // Forced merge: if the new lane has fewer sublanes than our
        // current index allows, clamp into range. This is a hard merge,
        // not a negotiated one -- MOBIL doesn't yet look ahead to an
        // upcoming lane-count reduction, so vehicles don't proactively
        // merge early. That's a natural follow-up, not this step.
        const Lane* new_lane = find_lane(v.lane_id);
        int max_sublane = new_lane ? static_cast<int>(new_lane->num_sublanes) - 1 : 0;
        if (v.sublane_idx > max_sublane) {
            v.sublane_idx = max_sublane;
        }
    }

    sim_time_ += config_.fixed_dt;

    world_buffer_.back().vehicles = vehicles_;
    world_buffer_.back().sim_time = sim_time_;

    world_buffer_.publish();
}

}  // namespace ts
