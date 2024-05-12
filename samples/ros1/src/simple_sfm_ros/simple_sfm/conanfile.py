from conans import CMake, ConanFile

class SimpleSFMConan(ConanFile):
    name = "simple_sfm"
    version = "1.0.0"

    settings = "os", "compiler", "build_type", "arch"
    options = {
        "with_tests": [True, False],
    }
    default_options = {
        "with_tests": False,
        "boost:without_test": True,
        "boost:without_graph": True,
        "boost:shared": True,
        "gtsam:with_TBB": False
    }
    generators = "cmake", "cmake_find_package"

    _cmake = None

    def requirements(self):
        self.requires("gtsam/4.0.3")
        self.requires("boost/1.71.0")

    def _configure_cmake(self):
        if self._cmake:
            return self._cmake

        self._cmake = CMake(self)
        self._cmake.definitions["BUILD_TESTING"] = self.options.with_tests
        self._cmake.configure()
        return self._cmake

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()
