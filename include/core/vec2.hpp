#pragma once

namespace ts {

struct Vec2 {
    float x{};
    float y{};

    Vec2 operator+(Vec2 o) const noexcept { return {.x = x + o.x, .y = y + o.y}; }
    Vec2 operator-(Vec2 o) const noexcept { return {.x = x - o.x, .y = y - o.y}; }
    Vec2 operator*(float s) const noexcept { return {.x = x * s, .y = y * s}; }

    [[nodiscard]] float dot(Vec2 o) const noexcept { return x * o.x + y * o.y; }
    [[nodiscard]] float length_sq() const noexcept { return dot(*this); }
};

}  // namespace ts
