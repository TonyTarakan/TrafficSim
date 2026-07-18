#include "render/renderer.hpp"

#include <algorithm>

namespace ts {

namespace {

const RoadNode* find_node(std::span<const RoadNode> nodes, NodeId id)
{
    // TODO: make universal with ADL
    auto it = std::ranges::find_if(nodes, [&](const RoadNode& n) { return n.id == id; });
    return (it != nodes.end()) ? &*it : nullptr;
}

const Lane* find_lane(std::span<const Lane> lanes, LaneId id)
{
    // TODO: make universal with ADL
    auto it = std::ranges::find_if(lanes, [&](const Lane& l) { return l.id == id; });
    return (it != lanes.end()) ? &*it : nullptr;
}

}  // namespace

void Renderer::draw_lanes(std::span<const RoadNode> nodes, std::span<const Lane> lanes, const Camera& camera)
{
    SDL_SetRenderDrawColor(sdl_renderer_, 90, 90, 90, 255);

    for (const auto& lane : lanes) {
        const RoadNode* from = find_node(nodes, lane.from);
        const RoadNode* to = find_node(nodes, lane.to);
        if (!from || !to) {
            continue;
        }

        Vec2D p1 = camera.to_screen(from->pos);
        Vec2D p2 = camera.to_screen(to->pos);
        SDL_RenderLine(sdl_renderer_, p1.x, p1.y, p2.x, p2.y);
    }
}

void Renderer::draw_vehicles(std::span<const Vehicle> vehicles, std::span<const RoadNode> nodes,
                             std::span<const Lane> lanes, const Camera& camera)
{
    SDL_SetRenderDrawColor(sdl_renderer_, 220, 180, 60, 255);

    constexpr float kVehicleSizePx = 8.f;

    for (const auto& v : vehicles) {
        const Lane* lane = find_lane(lanes, v.lane_id);
        if (!lane) {
            continue;
        }
        const RoadNode* from = find_node(nodes, lane->from);
        const RoadNode* to = find_node(nodes, lane->to);
        if (!from || !to) {
            continue;
        }

        // Workaround
        float t = (lane->length > 0.f) ? std::clamp(v.offset / lane->length, 0.f, 1.f) : 0.f;
        Vec2D world_pos = from->pos + (to->pos - from->pos) * t;
        Vec2D screen_pos = camera.to_screen(world_pos);

        SDL_FRect rect{.x = screen_pos.x - kVehicleSizePx / 2.f,
                       .y = screen_pos.y - kVehicleSizePx / 2.f,
                       .w = kVehicleSizePx,
                       .h = kVehicleSizePx};
        SDL_RenderFillRect(sdl_renderer_, &rect);
    }
}

}  // namespace ts
