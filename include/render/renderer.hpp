#pragma once

#include <SDL3/SDL.h>

#include <span>

#include "core/junction.hpp"
#include "core/road_graph.hpp"
#include "core/vehicle.hpp"
#include "render/camera.hpp"

// Draws the world (lanes, vehicles, signals) using the SDL3 renderer.

namespace ts {

class Renderer {
public:
    explicit Renderer(SDL_Renderer* sdl_renderer) : sdl_renderer_(sdl_renderer) {}

    void draw_lanes(std::span<const RoadNode> nodes, std::span<const Lane> lanes, const Camera& camera);

    void draw_vehicles(std::span<const Vehicle> vehicles, std::span<const RoadNode> nodes, std::span<const Lane> lanes,
                       const Camera& camera);

    void draw_junctions(std::span<const RoadNode> nodes, std::span<const Lane> lanes,
                        std::span<const Junction> junctions, const Camera& camera);

private:
    SDL_Renderer* sdl_renderer_;
};

}  // namespace ts
