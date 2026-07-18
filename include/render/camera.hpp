#pragma once

#include "core/vec2.hpp"

// World <-> screen coordinate conversion, pan and zoom.

namespace ts {

struct Camera {
    Vec2D offset{};    // world position shown at screen (0, 0)
    float zoom{10.f};  // pixels per metre

    [[nodiscard]]
    Vec2D to_screen(Vec2D world) const noexcept
    {
        return {.x = (world.x - offset.x) * zoom, .y = (world.y - offset.y) * zoom};
    }

    [[nodiscard]]
    Vec2D to_world(Vec2D screen) const noexcept
    {
        return {.x = screen.x / zoom + offset.x, .y = screen.y / zoom + offset.y};
    }

    void pan(float dx_px, float dy_px) noexcept
    {
        offset.x -= dx_px / zoom;
        offset.y -= dy_px / zoom;
    }
};

}  // namespace ts
