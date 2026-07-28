#include "render/renderer.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

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

// World position of the stop line.
// TODO: make it look good
std::optional<Vec2D> lane_stop_line(std::span<const RoadNode> nodes, std::span<const Lane> lanes, LaneId lane_id)
{
    constexpr float kPullbackM = 15.0f;

    const Lane* lane = find_lane(lanes, lane_id);
    if (!lane) return std::nullopt;

    const RoadNode* from = find_node(nodes, lane->from);
    const RoadNode* to = find_node(nodes, lane->to);
    if (!from || !to) return std::nullopt;

    Vec2D dir = to->pos - from->pos;
    float len = std::sqrt(dir.length_sq());
    if (len < 1e-6f) return to->pos;

    Vec2D unit = dir * (1.f / len);
    Vec2D perp = lane_perpendicular(from->pos, to->pos);

    auto pos = to->pos - unit * kPullbackM + perp * kSublaneWidthM;

    constexpr float kSideOffsetM = 7.0f;
    pos = pos + perp * kSideOffsetM;

    return pos;
}

// Like a "give way" sign.
// TODO: add orientation, refactor
void draw_yield_marker(SDL_Renderer* sdl_renderer, Vec2D screen_pos)
{
    constexpr float kHalfWidthPx = 9.f;
    constexpr float kHalfHeightPx = 6.f;

    SDL_SetRenderDrawColor(sdl_renderer, 200, 0, 0, 255);

    float x = screen_pos.x;
    float y = screen_pos.y;
    SDL_RenderLine(sdl_renderer, x - kHalfWidthPx, y - kHalfHeightPx, x + kHalfWidthPx, y - kHalfHeightPx);
    SDL_RenderLine(sdl_renderer, x + kHalfWidthPx, y - kHalfHeightPx, x, y + kHalfHeightPx);
    SDL_RenderLine(sdl_renderer, x, y + kHalfHeightPx, x - kHalfWidthPx, y - kHalfHeightPx);
}

// Draw a primitive traffic light icon (SDL3 version).
// Uses float coordinates with SDL_FRect.
void draw_traffic_light(SDL_Renderer* sdl_renderer, ts::Vec2D screen_pos)
{
    // Dimensions
    constexpr float kBodyWidth = 6.f;
    constexpr float kBodyHeight = 16.f;
    constexpr float kLightSize = 4.f;  // size of each signal (square)
    constexpr float kSpacing = 1.f;    // gap between signals

    // Top-left corner of the body
    float x = screen_pos.x - kBodyWidth / 2.f;
    float y = screen_pos.y - kBodyHeight / 2.f;

    // 1. Draw the body (dark gray background + black border)
    SDL_SetRenderDrawColor(sdl_renderer, 50, 50, 50, 255);
    SDL_FRect body_rect = {.x = x, .y = y, .w = kBodyWidth, .h = kBodyHeight};
    SDL_RenderFillRect(sdl_renderer, &body_rect);  // fill body

    SDL_SetRenderDrawColor(sdl_renderer, 0, 0, 0, 255);
    SDL_RenderRect(sdl_renderer, &body_rect);  // outline

    // 2. Draw the three signals
    float light_x = screen_pos.x - kLightSize / 2.f;
    float light_y = y + kSpacing;

    SDL_SetRenderDrawColor(sdl_renderer, 255, 0, 0, 255);
    SDL_FRect red_rect = {.x = light_x, .y = light_y, .w = kLightSize, .h = kLightSize};
    SDL_RenderFillRect(sdl_renderer, &red_rect);

    light_y += kLightSize + kSpacing;
    SDL_SetRenderDrawColor(sdl_renderer, 255, 255, 0, 255);
    SDL_FRect yellow_rect = {.x = light_x, .y = light_y, .w = kLightSize, .h = kLightSize};
    SDL_RenderFillRect(sdl_renderer, &yellow_rect);

    light_y += kLightSize + kSpacing;
    SDL_SetRenderDrawColor(sdl_renderer, 0, 255, 0, 255);
    SDL_FRect green_rect = {.x = light_x, .y = light_y, .w = kLightSize, .h = kLightSize};
    SDL_RenderFillRect(sdl_renderer, &green_rect);
}

template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};

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

void Renderer::draw_junctions(std::span<const RoadNode> nodes, std::span<const Lane> lanes,
                              std::span<const Junction> junctions, const Camera& camera)
{
    for (const auto& junction : junctions) {
        // clang-format off
        std::visit(overloaded{
            
            [](const UnregulatedControl& ) {
                // Nothing
            },
            
            [&](const PriorityControl& control) {
                for (const auto& entry : control.yields_to) {
                    auto pos = lane_stop_line(nodes, lanes, entry.first);
                    if (pos) {
                        draw_yield_marker(sdl_renderer_, camera.to_screen(*pos));
                    }
                }
            },

            [&](const TrafficLightControl& ) {
                // TODO: draw live signals???
                for (LaneId lane_id : junction.incoming) {
                    auto stop_pos = lane_stop_line(nodes, lanes, lane_id);
                    if (stop_pos) {
                        draw_traffic_light(sdl_renderer_, camera.to_screen(*stop_pos));
                    }
                }
            },
        },
        junction.control);
        // clang-format on
    }
}

}  // namespace ts
