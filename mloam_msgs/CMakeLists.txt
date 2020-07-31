cmake_minimum_required(VERSION 2.8.3)
project(mloam_msgs)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  nav_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  Extrinsics.msg
  Keyframes.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  message_generation
  geometry_msgs
  std_msgs
  nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
