cmake_minimum_required(VERSION 3.19)
project(simple_sfm)

set(REQUIRED_CATKIN_PACKAGES roscpp)

find_package(catkin REQUIRED COMPONENTS ${REQUIRED_CATKIN_PACKAGES})

catkin_package(CATKIN_DEPENDS ${REQUIRED_CATKIN_PACKAGES})
