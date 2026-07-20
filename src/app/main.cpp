#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include <print>
#include <random>
#include <thread>
#include <vector>

#include "core/road_graph.hpp"
#include "core/sim_engine.hpp"
#include "core/vehicle_params.hpp"
#include "render/camera.hpp"
#include "render/renderer.hpp"

//
// Generic TODOs:
//
// - Try graph libs(MELON)
// - Optimize IDM and MOBIL algos
// - This is a project for fun and study,
//   but check https://eclipse.dev/sumo/ for ideas
//

namespace {

// A single straight demo lane with a handful of cars on it, just to see
// SimEngine + Renderer working end to end. Real map loading comes later.
std::vector<ts::RoadNode> make_demo_nodes()
{
    return {
        {.id = 0, .pos = {.x = 0.f, .y = 0.f}},
        {.id = 1, .pos = {.x = 200.f, .y = 0.f}},
    };
}

std::vector<ts::Lane> make_demo_lanes()
{
    return {
        {.id = 0, .from = 0, .to = 1, .length = 200.f, .speed_limit = 15.0f, .num_sublanes = 4},
    };
}

float generate_rand(float from, float to)
{
    static std::random_device rd;
    static std::mt19937 rng{rd()};  // генератор
    static std::uniform_real_distribution<float> dist{from, to};

    return dist(rng);
}

void spawn_demo_vehicles(ts::SimEngine& engine)
{
    for (int i = 1; i < 16; ++i) {
        float random_speed = generate_rand(5.0f, 10.0f);

        ts::Vehicle v{
            .id = static_cast<ts::VehicleId>(i),
            .type = ts::VehicleType::Car,
            .idm_params = ts::default_params(ts::VehicleType::Car),
            .speed = random_speed,
            .lane_id = 0,
            .offset = static_cast<float>(i) * 10.f,  // spread along the lane
            .sublane_idx = i % 2,
        };
        v.idm_params.desired_speed = random_speed;

        engine.vehicles().push_back(v);
    }
}

}  // namespace

int main(int /*argc*/, char** /*argv*/)
{
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::println("SDL_Init: {}\n", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("TrafficSim", 1280, 720, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* sdl_renderer = SDL_CreateRenderer(window, nullptr);
    SDL_SetRenderVSync(sdl_renderer, 1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplSDL3_InitForSDLRenderer(window, sdl_renderer);
    ImGui_ImplSDLRenderer3_Init(sdl_renderer);
    ImGui::StyleColorsDark();

    // --- demo scene setup ---
    std::vector<ts::RoadNode> nodes = make_demo_nodes();
    std::vector<ts::Lane> lanes = make_demo_lanes();

    ts::SimEngine engine;
    engine.set_lanes(lanes);
    spawn_demo_vehicles(engine);

    ts::Renderer renderer{sdl_renderer};
    ts::Camera camera;
    camera.offset = {.x = -50.f, .y = -50.f};  // leave a little margin around the lane
    camera.zoom = 5.f;

    std::atomic<bool> running = true;

    std::jthread sim_thread([&] {
        using namespace std::chrono;

        constexpr auto dt = 20ms;

        while (running) {
            auto start = steady_clock::now();

            engine.tick();

            std::this_thread::sleep_until(start + dt);
        }
    });

    while (running) {
        engine.world_buffer().consume();
        const auto& snapshot = engine.world_buffer().front();

        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            ImGui_ImplSDL3_ProcessEvent(&ev);
            if (ev.type == SDL_EVENT_QUIT) running = false;
        }

        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("TrafficSim");
        ImGui::Text("sim time: %.1f s", snapshot.sim_time);
        ImGui::Text("vehicles: %zu", snapshot.vehicles.size());
        ImGui::End();

        SDL_SetRenderDrawColor(sdl_renderer, 30, 30, 30, 255);
        SDL_RenderClear(sdl_renderer);

        renderer.draw_lanes(nodes, lanes, camera);
        renderer.draw_vehicles(snapshot.vehicles, nodes, lanes, camera);

        ImGui::Render();
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), sdl_renderer);
        SDL_RenderPresent(sdl_renderer);
    }

    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(sdl_renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
