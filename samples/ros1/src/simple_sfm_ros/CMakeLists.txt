cmake_minimum_required(VERSION 3.10)
project(simple_sfm_ros)

set(REQUIRED_CATKIN_PACKAGES pcl_conversions roscpp sensor_msgs visualization_msgs)
find_package(catkin REQUIRED COMPONENTS ${REQUIRED_CATKIN_PACKAGES})

catkin_package(CATKIN_DEPENDS ${REQUIRED_CATKIN_PACKAGES})

# simple_sfm library
add_subdirectory(simple_sfm)

# simple_sfm_node
add_executable(simple_sfm_node src/simple_sfm_node.cpp)
add_dependencies(simple_sfm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(simple_sfm_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(simple_sfm_node PRIVATE ${catkin_LIBRARIES} simple_sfm)

install(TARGETS simple_sfm_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch data rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
