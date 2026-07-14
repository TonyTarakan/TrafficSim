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

# ── deps (injected by Conan) ──────────────────────────────────────────────────
find_package(SDL3          REQUIRED)
find_package(imgui         REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(GTest         REQUIRED)
find_package(benchmark     REQUIRED)

# ── main executable ───────────────────────────────────────────────────────────
add_executable(trafficsim src/main.cpp)

target_link_libraries(trafficsim PRIVATE
    SDL3::SDL3
    imgui::imgui
    nlohmann_json::nlohmann_json
)

# ── tests ─────────────────────────────────────────────────────────────────────
enable_testing()

add_executable(tests tests/main_test.cpp)
target_link_libraries(tests PRIVATE GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(tests)
