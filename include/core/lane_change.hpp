#pragma once

#include <cstdint>
#include <span>

#include "core/vehicle.hpp"

// Lane-changing decisions on multi-lane roads.
//
// Model: MOBIL (Minimizing Overall Braking Induced by Lane changes),
// Kesting, Treiber, Helbing (2007) — the standard companion to IDM.
// Runs on top of IDM: for each vehicle, evaluates whether switching to
// an adjacent sublane would improve its own acceleration enough to be
// worth it, while not forcing a following vehicle on the target sublane
// to brake harder than a "politeness" threshold allows.
//
// Two components per candidate lane change:
//   - incentive:  would MY acceleration improve enough after switching?
//   - safety/politeness: would the vehicle behind me on the target lane
//     be forced to brake unacceptably hard?
//
// Operates purely on already-known IDM accelerations (before/after gap
// calculations) — no separate physics model, just a decision layer.
// Reads a vehicle snapshot only, never mutates — same "decide on last
// tick's state, apply after" pattern as IDM itself.

namespace ts::lane_change {

struct MobilParams {
    float politeness{0.3f};     // p — weight given to affected neighbors' comfort
    float switch_thresh{0.2f};  // Delta_a_th, m/s^2 — minimum gain to bother switching
    float max_safe_decel{4.f};  // b_safe, m/s^2 — never force a follower to brake harder
};

// Returns the sublane_idx delta `ego` should apply this tick: -1, 0, or +1.
// `num_sublanes` bounds which deltas are even legal on ego's current lane.
[[nodiscard]]
int decide(const Vehicle& ego, std::span<const Vehicle> all_vehicles, int num_sublanes, const MobilParams& params = {});

}  // namespace ts::lane_change
