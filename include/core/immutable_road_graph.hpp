#pragma once

namespace ts {

// TODO: graph optimization, refactor structs for SoA/DOD later
//
// PROPOSAL:
//
// struct Node {
//     double x, y, z;
// };
//
// struct Edge {
//     NodeId from;
//     NodeId to;
//     float length;
//     float max_speed;
//     uint8_t lane_count;
// };
//
// struct Lane {
//     EdgeId edge_id;
//     uint8_t index;
// };

class ImmutableRoadGraph {
    // TODO
};

}  // namespace ts