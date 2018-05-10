cmake_minimum_required(VERSION 2.8.3)
project(flypulator_teleoperation)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(joystick src/joystick.cpp)
target_link_libraries(joystick ${catkin_LIBRARIES} )
