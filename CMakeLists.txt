cmake_minimum_required(VERSION 3.25)
project(trafficsim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── sanitizers ────────────────────────────────────────────────────────────────
if(TS_SANITIZE)
    add_compile_options(-fsanitize=${TS_SANITIZE} -fno-omit-frame-pointer)
    add_link_options   (-fsanitize=${TS_SANITIZE})
endif()

# ── warnings ──────────────────────────────────────────────────────────────────
add_compile_options(-Wall -Wextra -Wpedantic)

# ── deps ____________________──────────────────────────────────────────────────
find_package(SDL3          REQUIRED)

find_package(GTest         REQUIRED)
find_package(benchmark     REQUIRED)

# ── imgui и nlohmann json через FetchContent ───────────────────────────────────
include(FetchContent)
 
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.92.8
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(imgui)
 
# imgui не имеет CMakeLists.txt — собираем вручную
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

# ── main executable ───────────────────────────────────────────────────────────
add_executable(trafficsim src/main.cpp)

target_link_libraries(trafficsim PRIVATE
    SDL3::SDL3
    imgui
    nlohmann_json::nlohmann_json
)

# ── tests ─────────────────────────────────────────────────────────────────────
enable_testing()

add_executable(tests tests/main_test.cpp)
target_link_libraries(tests PRIVATE GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(tests)
