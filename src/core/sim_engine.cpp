#include "core/sim_engine.hpp"

#include <algorithm>
#include <limits>

#include "core/idm.hpp"

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

void SimEngine::tick()
{
    // Snapshot leaders before mutating anything
    // TODO: this is O(N*N). Optimize!
    std::vector<std::optional<idm::LeaderInfo>> leaders(vehicles_.size());
    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        leaders[i] = find_leader(vehicles_[i], vehicles_);
    }

    for (std::size_t i = 0; i < vehicles_.size(); ++i) {
        Vehicle& v = vehicles_[i];

        float accel = idm::acceleration(v.idm_params, v.speed, leaders[i]);
        float new_speed = std::max(0.f, v.speed + accel * config_.fixed_dt);  // speed >= 0

        v.offset += 0.5f * (v.speed + new_speed) * config_.fixed_dt;  // linear accel distance
        v.speed = new_speed;

        // TODO: if offset exceeds the current lane's length
        // move the vehicle onto its next lane on the route.
    }

    sim_time_ += config_.fixed_dt;
}

}  // namespace ts
