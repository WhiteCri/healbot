cmake_minimum_required(VERSION 2.8.3)
project(motor_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamixel_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS dynamixel_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)



add_executable(motor_controller
  src/motor_controller.cpp
)
add_dependencies(motor_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_controller
  ${catkin_LIBRARIES}
)

add_executable(fake_controller
  src/fake_controller.cpp
)
add_dependencies(fake_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fake_controller
  ${catkin_LIBRARIES}
)

add_executable(cmd_vel_diff_processor
  src/cmd_vel_diff_processor.cpp
)
add_dependencies(cmd_vel_diff_processor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_vel_diff_processor
  ${catkin_LIBRARIES}
)