#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>
#include <quill/LogMacros.h>

#include <print>
#include <random>
#include <thread>
#include <utility>
#include <vector>

#include "core/junction.hpp"
#include "core/log.hpp"
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

// Two streams merging into one, then continuing to a shared destination:
//
//   node0 (stream A) --lane0--
//                              --> node2 --lane2(shared)--> node3
//   node1 (stream B) --lane1--
//
// Demonstrates route-following (vehicles advance lane0/1 -> lane2 when they reach the end)
// and forced merging
// (if the shared lane ever has fewer sublanes than an incoming one, SimEngine::tick() clamps sublane_idx on
// transition).
std::vector<ts::RoadNode> make_demo_nodes()
{
    return {
        {.id = 0, .pos = {.x = 0.f, .y = -30.f}},  // stream A origin
        {.id = 1, .pos = {.x = 0.f, .y = 30.f}},   // stream B origin
        {.id = 2, .pos = {.x = 150.f, .y = 0.f}},  // merge point
        {.id = 3, .pos = {.x = 300.f, .y = 0.f}},  // shared destination
    };
}

std::vector<ts::Lane> make_demo_lanes()
{
    return {
        {.id = 0, .from = 0, .to = 2, .length = 153.f, .speed_limit = 15.f, .num_sublanes = 1},
        {.id = 1, .from = 1, .to = 2, .length = 153.f, .speed_limit = 15.f, .num_sublanes = 1},
        {.id = 2, .from = 2, .to = 3, .length = 150.f, .speed_limit = 15.f, .num_sublanes = 1},
    };
}

std::vector<ts::Junction> make_demo_junctions()
{
    ts::Junction j{};
    j.node = 2;
    j.incoming = {0, 1};
    j.control = ts::PriorityControl{.yields_to = {{1, {0}}}};
    return {j};
}

float generate_rand(float from, float to)
{
    static std::random_device rd;
    static std::mt19937 rng{rd()};  // генератор
    static std::uniform_real_distribution<float> dist{from, to};

    return dist(rng);
}

void spawn_stream(ts::SimEngine& engine, ts::LaneId origin_lane, ts::NodeId origin_node, ts::VehicleId id_start)
{
    auto route = engine.compute_route(origin_node, 3);
    if (!route) {
        return;  // shouldn't happen with this demo network, but don't crash if it does
    }

    for (int i = 0; i < 6; ++i) {
        float random_speed = generate_rand(5.0f, 10.0f);

        ts::Vehicle v{
            .id = id_start + static_cast<ts::VehicleId>(i),
            .type = ts::VehicleType::Car,
            .idm_params = ts::default_params(ts::VehicleType::Car),
            .speed = random_speed,
            .lane_id = origin_lane,
            .offset = static_cast<float>(i) * 10.f,  // spread along the lane
            .sublane_idx = 0,                        // i % 2,
            .route = *route,
        };
        v.idm_params.desired_speed = random_speed;

        engine.vehicles().push_back(v);
    }
}

void spawn_demo_vehicles(ts::SimEngine& engine)
{
    spawn_stream(engine, /*origin_lane=*/0, /*origin_node=*/0, /*id_start=*/0);
    spawn_stream(engine, /*origin_lane=*/1, /*origin_node=*/1, /*id_start=*/100);
}

}  // namespace

int main(int /*argc*/, char** /*argv*/)
{
    ts::log::init();
    LOG_INFO(ts::log::get(), "TrafficSim starting up");

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        LOG_CRITICAL(ts::log::get(), "SDL_Init failed: {}", SDL_GetError());
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
    std::vector<ts::Junction> junctions = make_demo_junctions();

    ts::SimEngine engine;
    engine.set_map(nodes, lanes);
    engine.set_junctions(std::move(junctions));
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

    LOG_INFO(ts::log::get(), "TrafficSim shutting down");

    return 0;
}
