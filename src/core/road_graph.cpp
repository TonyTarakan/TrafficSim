#include "core/road_graph.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

namespace ts {

void RoadGraph::rebuild(const std::span<RoadNode> nodes, const std::span<Lane> lanes)
{
    adjacency_.clear();
    lane_dest_.clear();
    lane_info_.clear();
    node_info_.clear();

    for (const auto& n : nodes) {
        node_info_[n.id] = n;
        adjacency_.try_emplace(n.id);  // micro optimization
    }
    for (const auto& l : lanes) {
        adjacency_[l.from].emplace_back(l.id);
        lane_dest_[l.id] = l.to;
        lane_info_[l.id] = l;
    }
}

std::span<const LaneId> RoadGraph::node_lanes_out(NodeId node) const
{
    auto it = adjacency_.find(node);
    if (it == adjacency_.end()) {
        return {};
    }
    return it->second;
}

NodeId RoadGraph::lane_dest(LaneId lane) const
{
    auto it = lane_dest_.find(lane);
    return (it != lane_dest_.end()) ? it->second : kInvalidNode;
}

// A* over the lane graph.
// Edge cost = travel time (length / speed_limit).
// Heuristic = straight-line distance / an optimistic top speed — admissible
// as long as no lane is faster than that, so it never overestimates.
std::optional<std::vector<LaneId>> RoadGraph::find_route(NodeId src_id, NodeId dst_id) const
{
    if (src_id == dst_id) {
        return std::vector<LaneId>{};
    }

    // Cheapest possible cost
    // (Straight distance to the dst node) / (magic, kinda maximum speed)
    // Like "you can't reach the dst faster then this time"
    auto heuristic = [&](NodeId n) -> float {
        constexpr float kOptimisticTopSpeed = 33.f;  // ~120 km/h, m/s

        auto it_n = node_info_.find(n);
        auto it_d = node_info_.find(dst_id);
        if (it_n == node_info_.end() || it_d == node_info_.end()) {
            return 0.f;
        }

        Vec2D d = it_n->second.pos - it_d->second.pos;
        return std::sqrt(d.length_sq()) / kOptimisticTopSpeed;
    };

    using CostEntry = std::pair<float, NodeId>;
    using CostMinHeap = std::priority_queue<CostEntry, std::vector<CostEntry>, std::greater<>>;

    CostMinHeap costs_to_neighs;  // TODO: OPT: reserve

    std::unordered_map<NodeId, float> best_costs;
    std::unordered_map<NodeId, LaneId> came_via;
    std::unordered_map<NodeId, NodeId> came_from;

    best_costs[src_id] = 0.f;
    costs_to_neighs.emplace(heuristic(src_id), src_id);

    while (!costs_to_neighs.empty()) {
        auto [neigh_cost, neigh_id] = costs_to_neighs.top();
        costs_to_neighs.pop();

        if (neigh_id == dst_id) {
            std::vector<LaneId> path;
            NodeId node = dst_id;
            while (node != src_id) {
                path.push_back(came_via.at(node));
                node = came_from.at(node);
            }
            std::ranges::reverse(path);
            return path;
        }

        // We can push a node more than once with different costs.
        // Just ignore if it's not the best.
        // TODO: fix magic nums
        float best_cost_known = best_costs.contains(neigh_id) ? best_costs[neigh_id] : 1e9f;
        if (neigh_cost > best_cost_known + heuristic(neigh_id) + 1e-4f) {
            continue;
        }

        for (LaneId lane_id : node_lanes_out(neigh_id)) {
            auto it = lane_info_.find(lane_id);
            if (it == lane_info_.end()) {
                continue;
            }

            const Lane& lane = it->second;
            NodeId next = lane.to;
            float edge_cost = lane.length / std::max(lane.speed_limit, 0.1f);
            float best_cost_next = best_cost_known + edge_cost;

            // Better way found
            if (!best_costs.contains(next) || best_cost_next < best_costs[next]) {
                best_costs[next] = best_cost_next;
                came_via[next] = lane_id;
                came_from[next] = neigh_id;
                costs_to_neighs.emplace(best_cost_next + heuristic(next), next);
            }
        }
    }

    return std::nullopt;  // dst unreachable from src
}

}  // namespace ts