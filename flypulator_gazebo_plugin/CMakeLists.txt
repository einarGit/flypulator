cmake_minimum_required(VERSION 2.8.3)
project(flypulator_gazebo_plugin)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  genmsg
  dynamic_reconfigure
  gazebo_msgs
  controller_manager
  robot_state_publisher
  effort_controllers
  controller_interface
  joint_state_controller
  hardware_interface
  position_controllers
  std_srvs
  flypulator_common_msgs
)

catkin_package()

include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(aero_plugin SHARED src/aero_plugin.cpp)
target_link_libraries(aero_plugin ${GAZEBO_libraries} ${catkin_LIBRARIES})

add_library(control_plugin SHARED src/control_plugin.cpp)
target_link_libraries(control_plugin ${GAZEBO_libraries} ${catkin_LIBRARIES})
