#pragma once

#include <span>

#include "core/vehicle.hpp"

// Lane-changing decisions on multi-lane roads.
//
// Model: MOBIL (Minimizing Overall Braking Induced by Lane changes),
// Kesting, Treiber, Helbing (2007) — the standard companion to IDM.
//
// Two components per candidate lane change:
//   - motivation:         would MY acceleration improve enough after switching?
//   - safety/politeness:  would the vehicle behind me on the target lane
//                         be forced to brake unacceptably hard?

namespace ts::lane_change {

struct MobilParams {
    float politeness{0.3f};     // p — weight given to affected neighbors' comfort
    float switch_thresh{0.2f};  // Delta_a_th, m/s^2 — minimum gain to bother switching
    float max_safe_decel{4.f};  // b_safe, m/s^2 — never force a follower to brake harder
};

// Lane change or turn
enum class Turn : std::int8_t { NONE = 0, RIGHT = -1, LEFT = +1 };

// Returns the sublane_idx delta 'ego' should apply this tick: -1, 0, or +1.
// 'num_sublanes' bounds which deltas are even legal on ego's current lane.
[[nodiscard]]
Turn decide(const Vehicle& ego, std::span<const Vehicle> all_vehicles, int num_sublanes,
            const MobilParams& params = {});

}  // namespace ts::lane_change
