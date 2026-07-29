#include "core/road_graph.hpp"

#include <quill/LogMacros.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <format>
#include <queue>

#include "core/types.hpp"

namespace ts {

RoadGraph::RoadGraph(std::vector<Node> nodes, std::vector<Edge> edges)
    : nodes_(std::move(nodes)), edges_(std::move(edges))
{
    build_indices();
}

void RoadGraph::build_indices()
{
    const std::size_t node_cnt = nodes_.size();

    // Подсчёт исходящих сегментов для каждого узла
    std::vector<uint32_t> out_degree(node_cnt, 0);
    for (const auto& lane : edges_) {
        out_degree[lane.from.get()]++;
    }

    // Префиксные суммы для offsets
    outgoing_offsets_.resize(node_cnt + 1);
    outgoing_offsets_[0] = 0;
    for (size_t i = 0; i < node_cnt; ++i) {
        outgoing_offsets_[i + 1] = outgoing_offsets_[i] + out_degree[i];
    }

    // Заполнение списка исходящих сегментов
    outgoing_edges_.resize(outgoing_offsets_[node_cnt]);
    std::vector<uint32_t> cursor = outgoing_offsets_;  // копия
    for (const auto& lane : edges_) {
        uint32_t pos = cursor[lane.from.get()]++;
        outgoing_edges_[pos] = lane.id;
    }
}

// TODO: handle or throw?
const Node& RoadGraph::get_node(NodeId id) const
{
    if (id.get() >= nodes_.size()) {
        throw std::out_of_range(std::format("Invalid NodeId in RoadGraph::get_node {} {} ", id.get(), nodes_.size()));
    }
    return nodes_[id.get()];
}

// TODO: handle or throw?
const Edge& RoadGraph::get_edge(EdgeId id) const
{
    if (id.get() >= edges_.size()) {
        throw std::out_of_range(std::format("Invalid EdgeId in RoadGraph::get_edge {} {} ", id.get(), nodes_.size()));
    }
    return edges_[id.get()];
}

std::span<const EdgeId> RoadGraph::outgoing_edges(NodeId node) const
{
    if (node.get() >= nodes_.size()) return {};

    uint32_t start = outgoing_offsets_[node.get()];
    uint32_t count = outgoing_offsets_[node.get() + 1] - start;

    return {outgoing_edges_.data() + start, count};
}

NodeId RoadGraph::destination_node(EdgeId edge) const
{
    if (edge.get() >= edges_.size()) return kInvalidNode;

    return edges_[edge.get()].to;
}

// A* over the graph
//
// Costs are travel times, not distances
//
// g = cost from start (real)
// h = heuristic to goal (estimated remaining cost)
// f = total cost to go (estimated)
// f = g + h
//
std::optional<std::vector<EdgeId>> RoadGraph::find_route(NodeId src_id, NodeId dst_id) const
{
    if (src_id == dst_id) {
        return std::vector<EdgeId>{};
    }

    // Heuristic cheapest possible cost
    // (Straight distance to the dst node) / (magic, kinda maximum speed)
    // Like "you can't reach the dst faster then this time"
    auto curr_h_cost = [&](NodeId node_id) -> float {
        constexpr float kOptimisticTopSpeed = 33.f;  // TODO: maybe default param?

        // Check both exist
        if (node_id.get() >= nodes_.size() || dst_id.get() >= nodes_.size()) {
            return 0.f;
        }

        const Node& n = nodes_[node_id.get()];
        const Node& d = nodes_[dst_id.get()];
        Vec2D delta = n.pos - d.pos;
        return std::sqrt(delta.length_sq()) / kOptimisticTopSpeed;
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
    std::unordered_map<NodeId, EdgeId> came_via;
    std::unordered_map<NodeId, NodeId> came_from;

    best_costs[src_id] = 0.f;
    frontier.emplace(curr_h_cost(src_id), src_id);

    while (!frontier.empty()) {
        auto [curr_f_cost, curr_id] = frontier.top();
        frontier.pop();

        if (curr_id == dst_id) {
            std::vector<EdgeId> path;
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

        for (EdgeId edge_id : outgoing_edges(curr_id)) {
            const Edge& edge = edges_[edge_id.get()];
            NodeId next_id = edge.to;

            constexpr float kMinSpeed = 0.1f;
            float edge_cost = edge.length / std::max(edge.speed_limit, kMinSpeed);
            float best_cost_next = best_g_so_far + edge_cost;

            // Found a cheaper path to next.
            if (!best_costs.contains(next_id) || best_cost_next < best_costs[next_id]) {
                best_costs[next_id] = best_cost_next;
                came_via[next_id] = edge_id;
                came_from[next_id] = curr_id;
                frontier.emplace(best_cost_next + curr_h_cost(next_id), next_id);
            }
        }
    }

    return std::nullopt;  // dst unreachable from src
}

}  // namespace ts