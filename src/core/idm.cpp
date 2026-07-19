#include "core/idm.hpp"

#include <algorithm>
#include <cmath>

namespace ts::idm {

float accelerate(const VehicleParams& p, float curr_speed, std::optional<LeaderInfo> leader_info) noexcept
{
    const float speed_ratio = curr_speed / p.desired_speed;
    const float free_road_cf = 1.f - std::pow(speed_ratio, 4.0f);

    if (!leader_info) {
        return p.max_accel * free_road_cf;
    }

    // Avoid division by zero if two vehicles are (almost) touching.
    const float gap = std::max(leader_info->gap, 0.001f);
    const float dv = leader_info->dv;

    const float sqrt_amb = std::sqrt(p.max_accel * p.comfy_decel);
    const float s_star = p.min_gap + p.time_headway * curr_speed + (curr_speed * dv) / (2.f * sqrt_amb);

    const float interaction_cf = std::pow((s_star / gap), 2.0f);

    return p.max_accel * (free_road_cf - interaction_cf);
}

}  // namespace ts::idm
