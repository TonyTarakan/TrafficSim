#pragma once

#include <optional>

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

namespace ts {

struct Vehicle;  // core/vehicle.hpp

namespace lane_change {

// TODO: struct MobilParams — politeness factor, changing threshold,
//       max safe deceleration for the follower on the target lane.

// TODO: std::optional<int> decide(...) — returns -1 / 0 / +1 sublane
//       delta for one vehicle, given its current/left/right neighbours'
//       IDM state. Called once per vehicle per tick, independent of
//       other vehicles' decisions that same tick (evaluated on last
//       tick's snapshot, applied atomically — same pattern as IDM itself,
//       so it parallelizes the same way across the thread pool).

}  // namespace lane_change
}  // namespace ts
