cmake_minimum_required(VERSION 2.8.3)
project(healbot_odometry)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
)


catkin_package(
  CATKIN_DEPENDS nav_msgs tf
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)


add_executable(diff_odometry
  src/diff_odometry.cpp
)
add_dependencies(diff_odometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(diff_odometry
  ${catkin_LIBRARIES}
)