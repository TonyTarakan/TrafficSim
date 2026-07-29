#include "core/lane_change.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace ts::lane_change {

namespace {

// The closest vehicle ahead 'ref_offset' on the given lane/sublane
const Vehicle* find_ahead(const Vehicle& ego, std::span<const Vehicle> all_vehicles, int sublane_idx)
{
    const Vehicle* best = nullptr;
    float best_offset = std::numeric_limits<float>::infinity();

    for (const auto& other : all_vehicles) {
        if (other.id == ego.id) continue;
        if (other.edge_id != ego.edge_id) continue;
        if (other.sublane_idx != sublane_idx) continue;
        if (other.offset <= ego.offset) continue;

        if (other.offset < best_offset) {
            best_offset = other.offset;
            best = &other;
        }
    }

    return best;
}

// The closest vehicle behind 'ref_offset' on the given lane/sublane
// TODO: remove the code duplication
const Vehicle* find_behind(const Vehicle& ego, std::span<const Vehicle> all_vehicles, int sublane_idx)
{
    const Vehicle* best = nullptr;
    float best_offset = -std::numeric_limits<float>::infinity();

    for (const auto& other : all_vehicles) {
        if (other.id == ego.id) continue;
        if (other.edge_id != ego.edge_id) continue;
        if (other.sublane_idx != sublane_idx) continue;
        if (other.offset >= ego.offset) continue;

        if (other.offset > best_offset) {
            best_offset = other.offset;
            best = &other;
        }
    }

    return best;
}

// idm::LeaderInfo as seen by 'from', if 'leader' exists.
std::optional<idm::LeaderInfo> gap_info(const Vehicle& from, const Vehicle* leader)
{
    if (!leader) return std::nullopt;
    return idm::LeaderInfo{.gap = leader->offset - from.offset, .dv = from.speed - leader->speed};
}

// Evaluates one candidate sublane (ego.sublane_idx + delta).
// Return:
//  - the MOBIL metrics if the move is safe and legal,
//  - nullopt if it's out of bounds or would force someone to brake hard.
std::optional<float> evaluate_candidate(const Vehicle& ego, std::span<const Vehicle> all_vehicles, int num_sublanes,
                                        Turn delta, const MobilParams& params)
{
    int target_sublane_idx = static_cast<int>(ego.sublane_idx) + static_cast<int>(delta);
    if (target_sublane_idx < 0 || target_sublane_idx >= static_cast<int>(num_sublanes)) {
        return std::nullopt;  // no such sublane on this road
    }

    const Vehicle* old_lead = find_ahead(ego, all_vehicles, ego.sublane_idx);
    const Vehicle* old_flwr = find_behind(ego, all_vehicles, ego.sublane_idx);
    const Vehicle* new_lead = find_ahead(ego, all_vehicles, target_sublane_idx);
    const Vehicle* new_flwr = find_behind(ego, all_vehicles, target_sublane_idx);

    float ego_acc_curr = idm::accelerate(ego.idm_params, ego.speed, gap_info(ego, old_lead));
    float ego_acc_futr = idm::accelerate(ego.idm_params, ego.speed, gap_info(ego, new_lead));

    // Safety: would our move force a potential new follower to brake too hard?
    if (new_flwr) {
        float new_flwr_acc_futr = idm::accelerate(new_flwr->idm_params, new_flwr->speed, gap_info(*new_flwr, &ego));
        if (new_flwr_acc_futr < -params.max_safe_decel) {
            return std::nullopt;  // unsafe move
        }
    }

    // 'Politeness' metrics means "How will our move affect to road neighbours"
    float politeness_term = 0.f;

    if (new_flwr) {
        float new_flwr_acc_curr = idm::accelerate(new_flwr->idm_params, new_flwr->speed, gap_info(*new_flwr, new_lead));
        float new_flwr_acc_futr = idm::accelerate(new_flwr->idm_params, new_flwr->speed, gap_info(*new_flwr, &ego));
        politeness_term += new_flwr_acc_futr - new_flwr_acc_curr;
    }

    if (old_flwr) {
        float old_flwr_acc_curr = idm::accelerate(old_flwr->idm_params, old_flwr->speed, gap_info(*old_flwr, &ego));
        float old_flwr_acc_futr = idm::accelerate(old_flwr->idm_params, old_flwr->speed, gap_info(*old_flwr, old_lead));
        politeness_term += old_flwr_acc_futr - old_flwr_acc_curr;
    }

    float motivation = (ego_acc_futr - ego_acc_curr) + params.politeness * politeness_term;
    if (motivation > params.switch_thresh) {
        return motivation;
    }

    return std::nullopt;
}

}  // namespace

Turn decide(const Vehicle& ego, std::span<const Vehicle> all_vehicles, int num_sublanes, const MobilParams& params)
{
    auto right = evaluate_candidate(ego, all_vehicles, num_sublanes, Turn::RIGHT, params);
    auto left = evaluate_candidate(ego, all_vehicles, num_sublanes, Turn::LEFT, params);

    if (right && left) {
        return (*left > *right) ? Turn::LEFT : Turn::RIGHT;  // pick whichever gives the bigger improvement
    }

    if (right) return Turn::RIGHT;
    if (left) return Turn::LEFT;

    return Turn::NONE;
}

}  // namespace ts::lane_change
