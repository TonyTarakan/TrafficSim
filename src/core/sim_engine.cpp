#include "core/sim_engine.hpp"

#include <quill/LogMacros.h>

#include <algorithm>
#include <limits>
#include <random>

#include "core/idm.hpp"
#include "core/junction.hpp"
#include "core/lane_change.hpp"
#include "core/log.hpp"
#include "core/types.hpp"

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

SimEngine::SimEngine(SimConfig config) : config_(config), pool_(config.num_threads) {}

void SimEngine::set_map(std::vector<RoadNode> nodes, std::vector<Lane> lanes)
{
    LOG_INFO(log::get(), "map loaded: {} nodes, {} lanes", nodes.size(), lanes.size());
    graph_.rebuild(nodes, lanes);
    lanes_ = std::move(lanes);
    nodes_ = std::move(nodes);
}

void SimEngine::set_junctions(std::vector<Junction> junctions)
{
    LOG_INFO(log::get(), "{} junction(s) loaded", junctions.size());
    junctions_.rebuild(std::move(junctions), nodes_, lanes_);
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

namespace {

// TODO: remove code duplication
float generate_rand(float from, float to)
{
    thread_local std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> dist{from, to};

    return dist(rng);
}

}  // namespace

void SimEngine::tick()
{
    junctions_.advance_signals(config_.fixed_dt);

    // --- lane-change decisions, on the pre-tick snapshot ---
    // Must happen before any IDM mutation below: MOBIL needs to see the
    // same consistent "world" for every vehicle, same reasoning as the
    // leader snapshot further down. Only sublane_idx is mutated here —
    // speed/offset stay untouched until the IDM pass, so that pass's own
    // snapshot-then-apply logic is unaffected.

    using namespace lane_change;

    std::vector<Turn> sublane_deltas(vehicles_.size(), Turn::NONE);
    pool_.parallel_for(
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                if (vehicles_[i].lane_change_cooldown > 0.f) {
                    continue;  // still cooling down from a recent switch — skip decide()
                }
                const Lane* lane = find_lane(vehicles_[i].lane_id);
                std::uint8_t num_sublanes = lane ? lane->num_sublanes : 1;
                sublane_deltas[i] = decide(vehicles_[i], vehicles_, num_sublanes);
            }
        },
        vehicles_.size());

    pool_.parallel_for(
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                Vehicle& v = vehicles_[i];
                if (sublane_deltas[i] != Turn::NONE) {
                    int new_sublane = static_cast<int>(v.sublane_idx) + static_cast<int>(sublane_deltas[i]);
                    v.sublane_idx = static_cast<std::uint8_t>(new_sublane);
                    v.lane_change_cooldown = generate_rand(3.0f, 4.0f);
                }
                else {
                    v.lane_change_cooldown = std::max(0.f, v.lane_change_cooldown - config_.fixed_dt);
                }
            }
        },
        vehicles_.size());

    // --- car-following (IDM), on the (now lane-change-applied) snapshot ---
    // Snapshot leaders before mutating anything
    // TODO: this is O(N*N). Optimize!
    std::vector<std::optional<idm::LeaderInfo>> leaders(vehicles_.size());
    pool_.parallel_for(
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                leaders[i] = find_leader(vehicles_[i], vehicles_);
            }
        },
        vehicles_.size());

    pool_.parallel_for(
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                Vehicle& v = vehicles_[i];

                // check real leaders
                float accel = idm::accelerate(v.idm_params, v.speed, leaders[i]);

                // A junction the current lane feeds into
                // acts as a second, independent obstacle (virtual leader).
                const Lane* cur_lane = find_lane(v.lane_id);
                if (cur_lane) {  // TODO: is it OK when the vehicle is out of lane?

                    auto virt_leader = leader_to_yield(v, *cur_lane, junctions_, vehicles_, lanes_);
                    if (virt_leader) {
                        float junction_accel = idm::accelerate(v.idm_params, v.speed, virt_leader);
                        if (junction_accel < accel) {
                            LOG_TRACE_L1(log::get(), "vehicle {} yields at junction, {:.1f}m to the line", v.id.get(),
                                         virt_leader->gap);
                        }

                        accel = std::min(accel, junction_accel);  // if we had a real leader, more restrictive wins
                    }
                }

                float new_speed = std::max(0.f, v.speed + accel * config_.fixed_dt);  // speed >= 0

                v.offset += 0.5f * (v.speed + new_speed) * config_.fixed_dt;  // linear accel distance
                v.speed = new_speed;
            }
        },
        vehicles_.size());

    // --- lane transitions: advance along the route when a lane ends ---
    // Runs after IDM integration (needs this tick's updated offset to know
    // whether we've actually run off the end of the current lane).
    pool_.parallel_for(
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                Vehicle& v = vehicles_[i];
                const Lane* cur_lane = find_lane(v.lane_id);
                if (!cur_lane || v.offset <= cur_lane->length) {
                    continue;  // still within the current lane, nothing to do
                }

                float overflow = v.offset - cur_lane->length;

                if (v.route_idx + 1 >= v.route.size()) {
                    // Workaround. No despawn logic yet, loop back to the start
                    v.route_idx = 0;
                }
                else {
                    ++v.route_idx;
                }

                if (v.route.empty()) continue;  // stay put at the lane's end

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
        },
        vehicles_.size());

    sim_time_ += config_.fixed_dt;

    world_buffer_.back().vehicles = vehicles_;
    world_buffer_.back().sim_time = sim_time_;

    world_buffer_.publish();
}

}  // namespace ts
