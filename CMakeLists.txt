cmake_minimum_required(VERSION 3.0.2)
project(nano_dbscan)

include_directories(include)

add_executable(test src/test.cpp)
target_link_libraries(test pthread)