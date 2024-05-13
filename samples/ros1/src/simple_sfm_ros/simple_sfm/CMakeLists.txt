cmake_minimum_required(VERSION 3.10)
project(simple_sfm)

set(CMAKE_CXX_STANDARD 14)

include(cmake/conan.cmake)
conan_cmake_configure(REQUIRES
                      "boost/1.71.0"
                      "catch2/2.13.9"
                      "gtsam/4.0.3"
                      OPTIONS
                      "boost:shared=True"
                      "boost:without_graph=True"
                      "boost:without_test=True"
                      "catch2:with_main=True"
                      "gtsam:with_TBB=False"
                      GENERATORS
                      cmake_find_package
)
conan_cmake_autodetect(settings)
conan_cmake_install(PATH_OR_REFERENCE
                    ${CMAKE_CURRENT_BINARY_DIR}
                    BUILD
                    missing
                    SETTINGS
                    ${settings}
)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_BINARY_DIR})
find_package(GTSAM REQUIRED)

add_library(simple_sfm SHARED src/simple_sfm.cpp src/utils.cpp)
target_include_directories(simple_sfm PUBLIC include)
target_link_libraries(simple_sfm PRIVATE GTSAM::gtsam)

if(BUILD_TESTING)
  find_package(Catch2 2.13.9 REQUIRED)
  add_subdirectory(tests)
endif()
