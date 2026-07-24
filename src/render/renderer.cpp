#include "render/renderer.hpp"

#include <algorithm>
#include <cmath>

namespace ts {

namespace {

constexpr float kSublaneWidthM = 4.0f;  // typical lane width, metres

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

// Unit vector perpendicular to the from->to direction, for offsetting
// sublanes sideways. Returns {0,0} for a degenerate (zero-length) lane.
Vec2D lane_perpendicular(Vec2D from, Vec2D to)
{
    Vec2D dir = to - from;
    float len = std::sqrt(dir.length_sq());
    if (len < 1e-6f) {
        return {.x = 0.f, .y = 0.f};
    }
    return {.x = -dir.y / len, .y = dir.x / len};
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

        Vec2D perp = lane_perpendicular(from->pos, to->pos);

        // One line per sublane, so a multi-lane road actually looks like one.
        // TODO: fix offsets for overlapping lanes
        for (std::uint8_t sub = 1; sub < lane.num_sublanes + 1; ++sub) {
            Vec2D lane_offset = perp * (static_cast<float>(sub) * kSublaneWidthM);
            Vec2D p1 = camera.to_screen(from->pos + lane_offset);
            Vec2D p2 = camera.to_screen(to->pos + lane_offset);
            SDL_RenderLine(sdl_renderer_, p1.x, p1.y, p2.x, p2.y);
        }
    }
}

void Renderer::draw_vehicles(std::span<const Vehicle> vehicles, std::span<const RoadNode> nodes,
                             std::span<const Lane> lanes, const Camera& camera)
{
    SDL_SetRenderDrawColor(sdl_renderer_, 220, 180, 60, 255);

    constexpr float kVehicleSizePx = 8.f;  // TODO: meters?

    for (const auto& v : vehicles) {
        const Lane* lane = find_lane(lanes, v.lane_id);
        if (!lane) continue;

        const RoadNode* from = find_node(nodes, lane->from);
        const RoadNode* to = find_node(nodes, lane->to);
        if (!from || !to) continue;

        // Workaround
        float t = (lane->length > 0.f) ? std::clamp(v.offset / lane->length, 0.f, 1.f) : 0.f;
        Vec2D lane_pos = from->pos + (to->pos - from->pos) * t;

        // Offset (parallelogram sum)
        Vec2D perp = lane_perpendicular(from->pos, to->pos);
        Vec2D world_pos = lane_pos + perp * (static_cast<float>(v.sublane_idx + 1) * kSublaneWidthM);

        Vec2D screen_pos = camera.to_screen(world_pos);

        SDL_FRect rect{.x = screen_pos.x - kVehicleSizePx / 2.f,
                       .y = screen_pos.y - kVehicleSizePx / 2.f,
                       .w = kVehicleSizePx,
                       .h = kVehicleSizePx};
        SDL_RenderFillRect(sdl_renderer_, &rect);
    }
}

}  // namespace ts
