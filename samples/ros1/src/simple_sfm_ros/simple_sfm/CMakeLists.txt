cmake_minimum_required(VERSION 3.16)
project(simple_sfm)

add_library(simple_sfm SHARED src/simple_sfm.cpp)
target_include_directories(simple_sfm PUBLIC include)
