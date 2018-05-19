cmake_minimum_required(VERSION 2.8.3)
project(flypulator_common_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(FILES
  RotorVelStamped.msg 
  Vector6dMsg.msg
  UavStateStamped.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
