cmake_minimum_required(VERSION 2.6)
project(motor_test)

add_definitions(-std=c++11)

include_directories(
  include/dynamixel_sdk
)
link_libraries(dxl_x64_cpp)

add_executable(get_position src/get_position.cpp)
add_executable(set_position src/set_position.cpp)
add_executable(set_position_speed src/set_position_speed.cpp)