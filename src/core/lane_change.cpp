#include "core/lane_change.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

namespace ts::lane_change {

namespace {

enum LaneChange : std::int8_t { NONE = 0, RIGHT = -1, LEFT = +1 };

// Nearest vehicle strictly ahead of 'ref_offset' on the given lane/sublane
const Vehicle* find_ahead(const Vehicle& ego, std::span<const Vehicle> all_vehicles, LaneId lane_id, int sublane_idx,
                          float ref_offset)
{
    const Vehicle* best = nullptr;
    float best_offset = std::numeric_limits<float>::infinity();

    for (const auto& other : all_vehicles) {
        if (other.id == ego.id) continue;
        if (other.lane_id != lane_id) continue;
        if (other.sublane_idx != sublane_idx) continue;
        if (other.offset <= ref_offset) continue;

        if (other.offset < best_offset) {
            best_offset = other.offset;
            best = &other;
        }
    }

    return best;
}

// Nearest vehicle strictly behind 'ref_offset' on the given lane/sublane
const Vehicle* find_behind(const Vehicle& ego, std::span<const Vehicle> all_vehicles, LaneId lane_id, int sublane_idx,
                           float ref_offset)
{
    const Vehicle* best = nullptr;
    float best_offset = -std::numeric_limits<float>::infinity();

    for (const auto& other : all_vehicles) {
        if (other.id == ego.id) continue;
        if (other.lane_id != lane_id) continue;
        if (other.sublane_idx != sublane_idx) continue;
        if (other.offset >= ref_offset) continue;

        if (other.offset > best_offset) {
            best_offset = other.offset;
            best = &other;
        }
    }

    return best;
}

// idm::LeaderInfo as seen by 'from', if 'leader' exists.
std::optional<idm::LeaderInfo> gap_to(const Vehicle& from, const Vehicle* leader)
{
    if (!leader) return std::nullopt;
    return idm::LeaderInfo{.gap = leader->offset - from.offset, .dv = from.speed - leader->speed};
}

// Evaluates one candidate sublane (ego.sublane_idx + delta).
// Returns the MOBIL incentive score if the move is safe and legal,
// or nullopt if it's out of bounds or would force someone to brake hard.
std::optional<float> evaluate_candidate(const Vehicle& ego, std::span<const Vehicle> all_vehicles, int num_sublanes,
                                        int delta, const MobilParams& params)
{
    int target = static_cast<int>(ego.sublane_idx) + delta;
    if (target < 0 || target >= static_cast<int>(num_sublanes)) {
        return std::nullopt;  // no such sublane on this road
    }

    const Vehicle* old_lead = find_ahead(ego, all_vehicles, ego.lane_id, ego.sublane_idx, ego.offset);
    const Vehicle* old_flwr = find_behind(ego, all_vehicles, ego.lane_id, ego.sublane_idx, ego.offset);
    const Vehicle* new_lead = find_ahead(ego, all_vehicles, ego.lane_id, target, ego.offset);
    const Vehicle* new_flwr = find_behind(ego, all_vehicles, ego.lane_id, target, ego.offset);

    float ego_acc_curr = idm::acceleration(ego.idm_params, ego.speed, gap_to(ego, old_lead));
    float ego_acc_futr = idm::acceleration(ego.idm_params, ego.speed, gap_to(ego, new_lead));

    // Safety: would inserting ego force new_follower to brake too hard?
    if (new_flwr) {
        float new_flwr_acc_futr = idm::acceleration(new_flwr->idm_params, new_flwr->speed, gap_to(*new_flwr, &ego));
        if (new_flwr_acc_futr < -params.max_safe_decel) {
            return std::nullopt;  // unsafe — would slam the brakes on someone
        }
    }

    // Politeness means "How will our move affect to road neighbours"
    float politeness_term = 0.f;

    if (new_flwr) {
        float new_flwr_acc_curr = idm::acceleration(new_flwr->idm_params, new_flwr->speed, gap_to(*new_flwr, new_lead));
        float new_flwr_acc_futr = idm::acceleration(new_flwr->idm_params, new_flwr->speed, gap_to(*new_flwr, &ego));
        politeness_term += new_flwr_acc_futr - new_flwr_acc_curr;
    }

    if (old_flwr) {
        float old_flwr_acc_curr = idm::acceleration(old_flwr->idm_params, old_flwr->speed, gap_to(*old_flwr, &ego));
        float old_flwr_acc_futr = idm::acceleration(old_flwr->idm_params, old_flwr->speed, gap_to(*old_flwr, old_lead));
        politeness_term += old_flwr_acc_futr - old_flwr_acc_curr;
    }

    float motivation = (ego_acc_futr - ego_acc_curr) + params.politeness * politeness_term;
    if (motivation > params.switch_thresh) {
        return motivation;
    }

    return std::nullopt;
}

}  // namespace

int decide(const Vehicle& ego, std::span<const Vehicle> all_vehicles, int num_sublanes, const MobilParams& params)
{
    auto right = evaluate_candidate(ego, all_vehicles, num_sublanes, LaneChange::RIGHT, params);
    auto left = evaluate_candidate(ego, all_vehicles, num_sublanes, LaneChange::LEFT, params);

    if (right && left) {
        return (*left > *right) ? LaneChange::LEFT : LaneChange::RIGHT;  // pick whichever gives the bigger improvement
    }

    if (right) return LaneChange::RIGHT;
    if (left) return LaneChange::LEFT;

    return LaneChange::NONE;
}

}  // namespace ts::lane_change
