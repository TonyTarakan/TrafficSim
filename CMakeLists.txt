cmake_minimum_required(VERSION 3.25)
project(trafficsim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

# sanitizers
if(TS_SANITIZE)
    add_compile_options(-fsanitize=${TS_SANITIZE} -fno-omit-frame-pointer)
    add_link_options   (-fsanitize=${TS_SANITIZE})
endif()

# warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# deps
find_package(SDL3          REQUIRED)

find_package(GTest         REQUIRED)
find_package(benchmark     REQUIRED)

# imgui and nlohmann json (workaround)
include(FetchContent)
 
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.92.8
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)
 
add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer3.cpp
)
target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC SDL3::SDL3)

FetchContent_Declare(nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.12.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

FetchContent_Declare(quill
    GIT_REPOSITORY https://github.com/odygrd/quill.git
    GIT_TAG        v12.1.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(quill)

# main lib
add_library(trafficsim_lib STATIC
    src/core/road_graph.cpp
    src/core/idm.cpp
    src/core/sim_engine.cpp
    src/core/traffic_rules.cpp
    src/core/vehicle_params.cpp
    src/core/lane_change.cpp  
    src/core/log.cpp
    src/concurrency/thread_pool.cpp
    src/concurrency/triple_buffer.cpp
    src/concurrency/work_queue.cpp
    src/render/renderer.cpp
    src/render/camera.cpp
    src/render/editor_ui.cpp
    src/render/debug_overlay.cpp
    src/map_io/map_serializer.cpp
)
target_include_directories(trafficsim_lib PUBLIC include)
target_link_libraries(trafficsim_lib PUBLIC quill::quill)


# main executable
add_executable(trafficsim src/app/main.cpp)
target_link_libraries(trafficsim PRIVATE trafficsim_lib)

target_link_libraries(trafficsim PRIVATE
    SDL3::SDL3
    imgui
    nlohmann_json::nlohmann_json
)

# tests
add_executable(tests
    tests/main_test.cpp
    tests/test_vec2d.cpp
    tests/test_vehicle.cpp
    tests/test_idm.cpp
    tests/test_road_graph.cpp
    tests/test_sim_engine.cpp
    tests/test_lane_change.cpp
    tests/test_lane_transition.cpp
)
target_link_libraries(tests PRIVATE trafficsim_lib GTest::gtest_main)

include(CTest)
enable_testing()

gtest_discover_tests(tests)
