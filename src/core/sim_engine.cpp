#include "core/sim_engine.hpp"

#include <quill/LogMacros.h>

#include <algorithm>
#include <limits>

#include "core/idm.hpp"
#include "core/junction.hpp"
#include "core/lane_change.hpp"
#include "core/log.hpp"
#include "core/random.hpp"
#include "core/types.hpp"

namespace ts {

namespace {

// Finds the closest vehicle ahead of 'ego' on the same edge and sublane.
// TODO: Take vehicle size into account, centre-to-centre for now
std::optional<idm::LeaderInfo> find_leader(const Vehicle& ego, std::span<const Vehicle> all_vehicles)
{
    std::optional<idm::LeaderInfo> best;
    float best_gap = std::numeric_limits<float>::infinity();

    for (const auto& other : all_vehicles) {
        if (other.id == ego.id) continue;
        if (other.edge_id != ego.edge_id) continue;
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

void SimEngine::set_map(std::vector<Node> nodes, std::vector<Edge> edges)
{
    LOG_INFO(log::get(), "map loaded: {} nodes, {} edges", nodes.size(), edges.size());
    graph_ = std::make_unique<RoadGraph>(std::move(nodes), std::move(edges));
    // Junctions are invalid now
    junctions_ = JunctionMap{};
}

void SimEngine::set_junctions(std::vector<Junction> junctions)
{
    if (!graph_) {
        LOG_ERROR(log::get(), "set_junctions called before set_map");
        return;
    }
    LOG_INFO(log::get(), "{} junction(s) loaded", junctions.size());
    junctions_.rebuild(std::move(junctions), *graph_);
}

std::optional<std::vector<EdgeId>> SimEngine::compute_route(NodeId src, NodeId dst) const
{
    if (!graph_) return std::nullopt;
    return graph_->find_route(src, dst);
}

const Edge* SimEngine::find_edge(EdgeId id) const
{
    if (!graph_) return nullptr;
    if (id.get() >= graph_->edge_count()) return nullptr;
    return &graph_->get_edge(id);
}

void SimEngine::tick()
{
    if (!graph_) {
        LOG_WARNING(log::get(), "tick called with no map loaded");
        return;
    }

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
                Vehicle& v = vehicles_[i];
                if (v.lane_change_cooldown > 0.f) continue;
                const Edge* edge = find_edge(v.edge_id);
                std::uint8_t num_sublanes = edge ? edge->lane_count : 1;
                sublane_deltas[i] = decide(v, vehicles_, num_sublanes);
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

    // Snapshot junction-yield decisions too, for the same reason: this
    // scans ALL of vehicles_ (any vehicle on a rival lane can block ego),
    // not just index i. Computing it in the same pass that mutates
    // vehicles_[i].speed/offset would race
    std::vector<std::optional<idm::LeaderInfo>> junction_leaders(vehicles_.size());
    pool_.parallel_for(
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                const Edge* cur_edge = find_edge(vehicles_[i].edge_id);
                if (cur_edge) {  // TODO: is it OK when the vehicle is out of lane?
                    junction_leaders[i] = leader_to_yield(vehicles_[i], *cur_edge, junctions_, vehicles_, *graph_);
                }
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
                if (junction_leaders[i]) {
                    float junction_accel = idm::accelerate(v.idm_params, v.speed, junction_leaders[i]);
                    if (junction_accel < accel) {
                        LOG_TRACE_L1(log::get(), "vehicle {} yields at junction, {:.1f}m to the line", v.id.get(),
                                     junction_leaders[i]->gap);
                    }

                    accel = std::min(accel, junction_accel);  // if we had a real leader, more restrictive wins
                }

                float new_speed = std::max(0.f, v.speed + accel * config_.fixed_dt);  // speed >= 0

                v.offset += 0.5f * (v.speed + new_speed) * config_.fixed_dt;  // linear accel distance
                v.speed = new_speed;
            }
        },
        vehicles_.size());

    // --- edge transitions: advance along the route when a edge ends ---
    // Runs after IDM integration (needs this tick's updated offset to know
    // whether we've actually run off the end of the current edge).
    pool_.parallel_for(
        [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                Vehicle& v = vehicles_[i];
                const Edge* cur_edge = find_edge(v.edge_id);
                if (!cur_edge || v.offset <= cur_edge->length) {
                    continue;  // still within the current edge, nothing to do
                }

                float overflow = v.offset - cur_edge->length;

                if (v.route_idx + 1 >= v.route.size()) {
                    // Workaround. No despawn logic yet, loop back to the start
                    v.route_idx = 0;
                }
                else {
                    ++v.route_idx;
                }

                if (v.route.empty()) continue;  // stay put at the edge's end

                v.edge_id = v.route[v.route_idx];
                v.offset = overflow;

                // Forced merge: if the new edge has fewer sublanes than our
                // current index allows, clamp into range. This is a hard merge,
                // not a negotiated one -- MOBIL doesn't yet look ahead to an
                // upcoming lane-count reduction, so vehicles don't proactively
                // merge early. That's a natural follow-up, not this step.
                const Edge* new_edge = find_edge(v.edge_id);
                int max_sublane = new_edge ? static_cast<int>(new_edge->lane_count) - 1 : 0;
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
