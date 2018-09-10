cmake_minimum_required(VERSION 2.8.3)
project(lidar2d_object_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs obstacle_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  obstacle_detector
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar2d_object_tracker
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)

add_executable(lidar2d_object_tracker
  src/lidar2d_object_tracker.cpp
)
add_dependencies(lidar2d_object_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar2d_object_tracker
  ${catkin_LIBRARIES}
)