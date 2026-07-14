from conan import ConanFile
from conan.tools.cmake import cmake_layout


class TrafficSim(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"

    def requirements(self):
        self.requires("sdl/3.2.10")
        self.requires("imgui/1.91.9-docking")
        self.requires("nlohmann_json/3.11.3")
        self.requires("gtest/1.15.0")
        self.requires("benchmark/1.9.1")

    def layout(self):
        cmake_layout(self)
