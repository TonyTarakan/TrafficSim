#include "core/road_graph.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

#include "core/types.hpp"

namespace ts {

void RoadGraph::rebuild(std::span<const RoadNode> nodes, std::span<const Lane> lanes)
{
    adjacency_.clear();
    lane_dest_.clear();
    lanes_by_id_.clear();
    nodes_by_id_.clear();

    for (const auto& n : nodes) {
        nodes_by_id_[n.id] = n;
        adjacency_.try_emplace(n.id);  // micro optimization
    }
    for (const auto& l : lanes) {
        adjacency_[l.from].emplace_back(l.id);
        lane_dest_[l.id] = l.to;
        lanes_by_id_[l.id] = l;
    }
}

std::span<const LaneId> RoadGraph::outgoing_lanes(NodeId node) const
{
    auto it = adjacency_.find(node);
    if (it == adjacency_.end()) {
        return {};
    }
    return it->second;
}

NodeId RoadGraph::destination_node(LaneId lane) const
{
    auto it = lane_dest_.find(lane);
    return (it != lane_dest_.end()) ? it->second : kInvalidNode;
}

// A* over the lane graph
//
// Costs are travel times, not distances
//
// g = cost from start (real)
// h = heuristic to goal (estimated remaining cost)
// f = total cost to go (estimated)
// f = g + h
//
std::optional<std::vector<LaneId>> RoadGraph::find_route(NodeId src_id, NodeId dst_id) const
{
    if (src_id == dst_id) {
        return std::vector<LaneId>{};
    }

    // Heuristic cheapest possible cost
    // (Straight distance to the dst node) / (magic, kinda maximum speed)
    // Like "you can't reach the dst faster then this time"
    auto curr_h_cost = [&](NodeId node_id) -> float {
        constexpr float kOptimisticTopSpeed = 33.f;  // TODO: maybe default param?

        auto it_n = nodes_by_id_.find(node_id);
        auto it_d = nodes_by_id_.find(dst_id);
        if (it_n == nodes_by_id_.end() || it_d == nodes_by_id_.end()) {
            return 0.f;
        }

        Vec2D d = it_n->second.pos - it_d->second.pos;
        return std::sqrt(d.length_sq()) / kOptimisticTopSpeed;
    };

    // Frontier aka Open aka Candidates to process.
    struct OpenNode {
        float f_cost;
        NodeId id;
    };
    struct OpenNodeGreater {
        bool operator()(const OpenNode& a, const OpenNode& b) const { return a.f_cost > b.f_cost; }
    };
    using CostMinHeap = std::priority_queue<OpenNode, std::vector<OpenNode>, OpenNodeGreater>;

    CostMinHeap frontier;

    std::unordered_map<NodeId, float> best_costs;
    std::unordered_map<NodeId, LaneId> came_via;
    std::unordered_map<NodeId, NodeId> came_from;

    best_costs[src_id] = 0.f;
    frontier.emplace(curr_h_cost(src_id), src_id);

    while (!frontier.empty()) {
        auto [curr_f_cost, curr_id] = frontier.top();
        frontier.pop();

        if (curr_id == dst_id) {
            std::vector<LaneId> path;
            NodeId node_id = dst_id;
            while (node_id != src_id) {
                path.push_back(came_via.at(node_id));
                node_id = came_from.at(node_id);
            }
            std::ranges::reverse(path);
            return path;
        }

        // We can push a node more than once with different costs.
        // Just ignore if it's not the best.
        constexpr float kCostEpsilon = 1e-4f;
        float best_g_so_far =
            best_costs.contains(curr_id) ? best_costs[curr_id] : std::numeric_limits<float>::infinity();
        if (curr_f_cost > best_g_so_far + curr_h_cost(curr_id) + kCostEpsilon) {
            continue;
        }

        for (LaneId lane_id : outgoing_lanes(curr_id)) {
            auto it = lanes_by_id_.find(lane_id);
            if (it == lanes_by_id_.end()) {
                continue;
            }

            const Lane& lane = it->second;
            NodeId next_id = lane.to;
            constexpr float kMinSpeed = 0.1f;
            float edge_cost = lane.length / std::max(lane.speed_limit, kMinSpeed);
            float best_cost_next = best_g_so_far + edge_cost;

            // Found a cheaper path to next.
            if (!best_costs.contains(next_id) || best_cost_next < best_costs[next_id]) {
                best_costs[next_id] = best_cost_next;
                came_via[next_id] = lane_id;
                came_from[next_id] = curr_id;
                frontier.emplace(best_cost_next + curr_h_cost(next_id), next_id);
            }
        }
    }

    return std::nullopt;  // dst unreachable from src
}

}  // namespace ts