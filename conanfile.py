from conan import ConanFile
from conan.tools.cmake import cmake_layout


class TrafficSim(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"

    def requirements(self):
        self.requires("sdl/3.4.8")
        self.requires("imgui/1.92.8")
        self.requires("nlohmann_json/3.12.0")
        self.requires("gtest/1.17.0")
        self.requires("benchmark/1.9.5")
        self.requires("quill/12.0.0")

    def layout(self):
        # build_type берётся из -s build_type=Debug|Release и т.д.
        # папка будет build/debug, build/release — совпадает с binaryDir в пресетах
        bt = str(self.settings.build_type).lower()
        self.folders.build = f"build/{bt}"
        self.folders.generators = f"build/{bt}/generators"