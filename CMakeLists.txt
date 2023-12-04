cmake_minimum_required(VERSION 3.0.2)
project(nano_dbscan)

find_package(PCL 1.11.1 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(test src/test.cpp)
target_link_libraries(test pthread ${PCL_LIBRARIES})