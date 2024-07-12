from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout, CMakeDeps
import os


class SC_3D_collision_helpersRecipe(ConanFile):
    name = "3d_collision_helpers"
    version = "3.22.1"
    package_type = "library"

    # Optional metadata
    license = "MIT"
    author = "Stephen Seo stephen@seodisparate.com"
    url = "https://git.seodisparate.com/stephenseo/3d_collision_helpers"
    description = "A library with helpers for 3d space"
    topics = ("3D", "collision", "matrix", "vector")

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "CMakeLists.txt", "src/*"

    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")

    def configure(self):
        if self.options.shared:
            self.options.rm_safe("fPIC")

    def layout(self):
        cmake_layout(self)
    
    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        if not self.conf.get("tools.build:skip_test", default=False):
            self.run(os.path.join(self.build_folder, "UnitTest"))

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["SC_3D_CollisionDetectionHelpers"]

