cmake_minimum_required(VERSION 2.6)
project(pcl_tools)

# add_definitions(-std=c++11)

find_package(PCL 1.2 REQUIRED)
find_package(Eigen3 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(icp src/icp.cpp)
target_link_libraries (icp ${PCL_LIBRARIES})

