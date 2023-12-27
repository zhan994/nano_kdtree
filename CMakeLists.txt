cmake_minimum_required(VERSION 3.0.2)
project(nano_dbscan)

find_package(PCL 1.11.1 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(test_nano_dbscan_2d src/test_nano_dbscan_2d.cpp)
target_link_libraries(test_nano_dbscan_2d pthread ${PCL_LIBRARIES})


add_executable(test_nano_dbscan_3d src/test_nano_dbscan_3d.cpp)
target_link_libraries(test_nano_dbscan_3d pthread ${PCL_LIBRARIES})