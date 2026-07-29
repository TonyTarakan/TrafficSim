#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_video.h>
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
#include "core/types.hpp"
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
// - Spawn points + despawn (OD matrix by nodes)
// - Junction stop lane fix
// - Multi-level junctions
//   (RoadNode::z — make it not just a field, but a working feature in the A* heuristics and rendering)
// - Circles, curves - ??
// - Map editor (maybe there's a separate one for OSM or something like that) (editor_ui.cpp)
//   Saving/loading maps + OSM (map_serializer.cpp)
// - Proactive merge via MOBIL and smooth lane changes
// - Live traffic lights?
// - REPEAT/CHECK latch, barrier, lock-free, cv, queues, sanitizers, ...
// - Benchmarks (bench/, perf/eBPF, lock-free, AoS→SoA comparison, DOD)
// - Improved rendering, UI and refactoring, strong types, UDL units
// - transition to oneTBB and SIMD
//
// - Immutable Map as a single object.
// - SpawnSystem / DespawnSystem / VehicleSystem.
// - Job System (parallel_for) instead of the universal ThreadPool.
// - Indexes (LaneId → Lane*, NodeId → Node*) instead of linear searches.
// - Lane occupancy cache (search for the leader not across all cars, but only across cars in the desired lane).
//

namespace {

// Node ids for the demo network -- named so the map/junction/spawn code
// below can't drift out of sync with itself (see the incoming={2}-instead-
// of-{0,1} bug from earlier: hardcoded ids with no shared source of truth
// are exactly how that happens).
constexpr ts::NodeId kNodeW0{0};  // main road, west entry/exit
constexpr ts::NodeId kNodeJ1{1};  // unregulated crossroads (right-hand rule)
constexpr ts::NodeId kNodeJ2{2};  // priority-controlled crossroads (signposted main road)
constexpr ts::NodeId kNodeJ3{3};  // traffic-light crossroads
constexpr ts::NodeId kNodeE0{4};  // main road, east entry/exit
constexpr ts::NodeId kNodeN1{5};
constexpr ts::NodeId kNodeS1{6};
constexpr ts::NodeId kNodeN2{7};
constexpr ts::NodeId kNodeS2{8};
constexpr ts::NodeId kNodeN3{9};
constexpr ts::NodeId kNodeS3{10};

// Lane ids, grouped by road segment. Every road is two one-way lanes
// (a proper pair), not a single bidirectional one.
constexpr ts::EdgeId kEdgeW0_J1{0};
constexpr ts::EdgeId kEdgeJ1_J2{1};
constexpr ts::EdgeId kEdgeJ2_J3{2};
constexpr ts::EdgeId kEdgeJ3_E0{3};
constexpr ts::EdgeId kEdgeE0_J3{4};
constexpr ts::EdgeId kEdgeJ3_J2{5};
constexpr ts::EdgeId kEdgeJ2_J1{6};
constexpr ts::EdgeId kEdgeJ1_W0{7};
constexpr ts::EdgeId kEdgeN1_J1{8};
constexpr ts::EdgeId kEdgeJ1_S1{9};
constexpr ts::EdgeId kEdgeS1_J1{10};
constexpr ts::EdgeId kEdgeJ1_N1{11};
constexpr ts::EdgeId kEdgeN2_J2{12};
constexpr ts::EdgeId kEdgeJ2_S2{13};
constexpr ts::EdgeId kEdgeS2_J2{14};
constexpr ts::EdgeId kEdgeJ2_N2{15};
constexpr ts::EdgeId kEdgeN3_J3{16};
constexpr ts::EdgeId kEdgeJ3_S3{17};
constexpr ts::EdgeId kEdgeS3_J3{18};
constexpr ts::EdgeId kEdgeJ3_N3{19};

//                         N1              N2              N3
//                         |               |               |
//   W0 === J1 =========== J2 ============ J3 ============ E0
//                         |               |               |
//                         S1              S2              S3
//
// A main road running W0 -> E0 through three crossroads, each demonstrating
// a different JunctionControl strategy, plus a two-way cross street at
// every one of them so through traffic and cross traffic actually
// collide:
//
//   J1: UnregulatedControl -- right-hand rule
//   J2: PriorityControl    -- main road is signposted, cross street yields both ways
//   J3: TrafficLightControl -- 8s phases, alternating main road / cross street
//
// Every road is a pair of one-way lanes, so this is real two-way traffic,
// not a single lane pretending to be bidirectional.
std::vector<ts::Node> make_demo_nodes()
{
    return {
        {.id = kNodeW0, .pos = {.x = -150.f, .y = 300.f}}, {.id = kNodeJ1, .pos = {.x = 0.f, .y = 300.f}},
        {.id = kNodeJ2, .pos = {.x = 300.f, .y = 300.f}},  {.id = kNodeJ3, .pos = {.x = 600.f, .y = 300.f}},
        {.id = kNodeE0, .pos = {.x = 750.f, .y = 300.f}},  {.id = kNodeN1, .pos = {.x = 0.f, .y = 120.f}},
        {.id = kNodeS1, .pos = {.x = 0.f, .y = 480.f}},    {.id = kNodeN2, .pos = {.x = 300.f, .y = 120.f}},
        {.id = kNodeS2, .pos = {.x = 300.f, .y = 480.f}},  {.id = kNodeN3, .pos = {.x = 600.f, .y = 120.f}},
        {.id = kNodeS3, .pos = {.x = 600.f, .y = 480.f}},
    };
}

std::vector<ts::Edge> make_demo_lanes()
{
    constexpr float kMainSpeed = 30.f;
    constexpr float kCrossSpeed = 20.f;

    return {
        // Main road, both directions, straight through J1/J2/J3.
        {.id = kEdgeW0_J1, .from = kNodeW0, .to = kNodeJ1, .length = 150.f, .speed_limit = kMainSpeed, .lane_count = 2},
        {.id = kEdgeJ1_J2, .from = kNodeJ1, .to = kNodeJ2, .length = 300.f, .speed_limit = kMainSpeed, .lane_count = 2},
        {.id = kEdgeJ2_J3, .from = kNodeJ2, .to = kNodeJ3, .length = 300.f, .speed_limit = kMainSpeed, .lane_count = 2},
        {.id = kEdgeJ3_E0, .from = kNodeJ3, .to = kNodeE0, .length = 150.f, .speed_limit = kMainSpeed, .lane_count = 2},
        {.id = kEdgeE0_J3, .from = kNodeE0, .to = kNodeJ3, .length = 150.f, .speed_limit = kMainSpeed, .lane_count = 2},
        {.id = kEdgeJ3_J2, .from = kNodeJ3, .to = kNodeJ2, .length = 300.f, .speed_limit = kMainSpeed, .lane_count = 2},
        {.id = kEdgeJ2_J1, .from = kNodeJ2, .to = kNodeJ1, .length = 300.f, .speed_limit = kMainSpeed, .lane_count = 2},
        {.id = kEdgeJ1_W0, .from = kNodeJ1, .to = kNodeW0, .length = 150.f, .speed_limit = kMainSpeed, .lane_count = 2},

        // Cross street at J1 -- unregulated.
        {.id = kEdgeN1_J1, .from = kNodeN1, .to = kNodeJ1, .length = 180.f, .speed_limit = kCrossSpeed},
        {.id = kEdgeJ1_S1, .from = kNodeJ1, .to = kNodeS1, .length = 180.f, .speed_limit = kCrossSpeed},
        {.id = kEdgeS1_J1, .from = kNodeS1, .to = kNodeJ1, .length = 180.f, .speed_limit = kCrossSpeed},
        {.id = kEdgeJ1_N1, .from = kNodeJ1, .to = kNodeN1, .length = 180.f, .speed_limit = kCrossSpeed},

        // Cross street at J2 -- yields to the main road.
        {.id = kEdgeN2_J2, .from = kNodeN2, .to = kNodeJ2, .length = 180.f, .speed_limit = kCrossSpeed},
        {.id = kEdgeJ2_S2, .from = kNodeJ2, .to = kNodeS2, .length = 180.f, .speed_limit = kCrossSpeed},
        {.id = kEdgeS2_J2, .from = kNodeS2, .to = kNodeJ2, .length = 180.f, .speed_limit = kCrossSpeed},
        {.id = kEdgeJ2_N2, .from = kNodeJ2, .to = kNodeN2, .length = 180.f, .speed_limit = kCrossSpeed},

        // Cross street at J3 -- traffic light.
        {.id = kEdgeN3_J3, .from = kNodeN3, .to = kNodeJ3, .length = 180.f, .speed_limit = kCrossSpeed},
        {.id = kEdgeJ3_S3, .from = kNodeJ3, .to = kNodeS3, .length = 180.f, .speed_limit = kCrossSpeed},
        {.id = kEdgeS3_J3, .from = kNodeS3, .to = kNodeJ3, .length = 180.f, .speed_limit = kCrossSpeed},
        {.id = kEdgeJ3_N3, .from = kNodeJ3, .to = kNodeN3, .length = 180.f, .speed_limit = kCrossSpeed},
    };
}

std::vector<ts::Junction> make_demo_junctions()
{
    ts::Junction j1{};
    j1.node_id = kNodeJ1;
    j1.incoming = {kEdgeW0_J1, kEdgeJ2_J1, kEdgeN1_J1, kEdgeS1_J1};
    j1.control = ts::UnregulatedControl{};

    ts::Junction j2{};
    j2.node_id = kNodeJ2;
    j2.incoming = {kEdgeJ1_J2, kEdgeJ3_J2, kEdgeN2_J2, kEdgeS2_J2};
    // Main-road through traffic (from J1 or from J3) has priority; the
    // cross street yields to both directions of it.
    j2.control = ts::PriorityControl{.yields_to = {
                                         {kEdgeN2_J2, {kEdgeJ1_J2, kEdgeJ3_J2}},
                                         {kEdgeS2_J2, {kEdgeJ1_J2, kEdgeJ3_J2}},
                                     }};

    ts::Junction j3{};
    j3.node_id = kNodeJ3;
    j3.incoming = {kEdgeJ2_J3, kEdgeE0_J3, kEdgeN3_J3, kEdgeS3_J3};
    j3.control = ts::TrafficLightControl{.phases = {
                                             {.green_lanes = {kEdgeJ2_J3, kEdgeE0_J3}, .duration = 8.f},
                                             {.green_lanes = {kEdgeN3_J3, kEdgeS3_J3}, .duration = 8.f},
                                         }};

    return {j1, j2, j3};
}

// TODO: remove code duplication
float generate_rand(float from, float to)
{
    static std::random_device rd;
    static std::mt19937 rng{rd()};  // генератор
    std::uniform_real_distribution<float> dist{from, to};

    return dist(rng);
}

// Spawns 'count' vehicles at the start of the route from 'origin_node' to
// 'dest_node', spread out nose-to-tail so they don't start overlapping.
void spawn_stream(ts::SimEngine& engine, ts::NodeId origin_node, ts::NodeId dest_node, ts::VehicleId id_start,
                  int count)
{
    auto route = engine.compute_route(origin_node, dest_node);
    if (!route || route->empty()) {
        LOG_WARNING(ts::log::get(), "no route from node {} to node {}, skipping stream", origin_node.get(),
                    dest_node.get());
        return;
    }

    for (int i = 0; i < count; ++i) {
        float random_speed = generate_rand(5.0f, 10.0f);

        ts::Vehicle v{
            .id = static_cast<ts::VehicleId>(id_start.get() + i),
            .type = ts::VehicleType::Car,
            .idm_params = ts::default_params(ts::VehicleType::Car),
            .speed = random_speed,
            .edge_id = route->front(),
            .offset = static_cast<float>(i) * 12.f,  // spread along the lane
            .route = *route,
        };
        v.idm_params.desired_speed = random_speed;

        engine.vehicles().push_back(v);
    }
}

void spawn_demo_vehicles(ts::SimEngine& engine)
{
    // Main road through traffic, both directions -- crosses all three junctions.
    spawn_stream(engine, kNodeW0, kNodeE0, ts::VehicleId{0}, 10);
    spawn_stream(engine, kNodeE0, kNodeW0, ts::VehicleId{1000}, 10);

    // Cross traffic at J1 (unregulated).
    spawn_stream(engine, kNodeN1, kNodeS1, ts::VehicleId{2000}, 3);
    spawn_stream(engine, kNodeS1, kNodeN1, ts::VehicleId{3000}, 3);

    // Cross traffic at J2 (priority signs -- yields to the main road).
    spawn_stream(engine, kNodeN2, kNodeS2, ts::VehicleId{4000}, 3);
    spawn_stream(engine, kNodeS2, kNodeN2, ts::VehicleId{5000}, 3);

    // Cross traffic at J3 (traffic light).
    spawn_stream(engine, kNodeN3, kNodeS3, ts::VehicleId{6000}, 4);
    spawn_stream(engine, kNodeS3, kNodeN3, ts::VehicleId{7000}, 4);
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

    SDL_Window* window = SDL_CreateWindow("TrafficSim", 1280, 720, SDL_WINDOW_RESIZABLE | SDL_WINDOW_MAXIMIZED);
    SDL_Renderer* sdl_renderer = SDL_CreateRenderer(window, nullptr);
    SDL_SetRenderVSync(sdl_renderer, 1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplSDL3_InitForSDLRenderer(window, sdl_renderer);
    ImGui_ImplSDLRenderer3_Init(sdl_renderer);
    ImGui::StyleColorsDark();

    // --- demo scene setup ---
    std::vector<ts::Node> nodes = make_demo_nodes();
    std::vector<ts::Edge> lanes = make_demo_lanes();
    std::vector<ts::Junction> junctions = make_demo_junctions();
    const std::size_t junction_count = junctions.size();
    const std::vector<ts::Junction> junctions_for_render = junctions;

    ts::SimEngine engine;
    engine.set_map(nodes, lanes);
    engine.set_junctions(std::move(junctions));
    spawn_demo_vehicles(engine);

    ts::Renderer renderer{sdl_renderer};
    ts::Camera camera;
    camera.offset = {.x = -170.f, .y = 0.f};  // fit the W..E / N..S extent with some margin
    camera.zoom = 2.0f;

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
        ImGui::Text("sim time: %.1lf s", snapshot.sim_time);
        ImGui::Text("vehicles: %zu", snapshot.vehicles.size());
        ImGui::Text("junctions: %zu (unregulated / priority / traffic light)", junction_count);
        ImGui::End();

        SDL_SetRenderDrawColor(sdl_renderer, 30, 30, 30, 255);
        SDL_RenderClear(sdl_renderer);

        renderer.draw_edges(nodes, lanes, camera);
        renderer.draw_junctions(nodes, lanes, junctions_for_render, camera);
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
