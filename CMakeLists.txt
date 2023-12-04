cmake_minimum_required(VERSION 3.0.2)
project(nano_dbscan)

find_package(PCL 1.11.1 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(test2d src/test2d.cpp)
target_link_libraries(test2d pthread ${PCL_LIBRARIES})


add_executable(test3d src/test3d.cpp)
target_link_libraries(test3d pthread ${PCL_LIBRARIES})