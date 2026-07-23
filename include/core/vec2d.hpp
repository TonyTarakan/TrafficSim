#pragma once

namespace ts {

struct Vec2D {
    float x{};
    float y{};

    Vec2D operator+(Vec2D other) const noexcept { return {.x = x + other.x, .y = y + other.y}; }
    Vec2D operator-(Vec2D other) const noexcept { return {.x = x - other.x, .y = y - other.y}; }
    Vec2D operator*(float s) const noexcept { return {.x = x * s, .y = y * s}; }

    [[nodiscard]] float dot_prod(Vec2D other) const noexcept { return x * other.x + y * other.y; }
    [[nodiscard]] float length_sq() const noexcept { return dot_prod(*this); }
};

// TODO: separate Point2D with cast from/to Vec2D

}  // namespace ts
