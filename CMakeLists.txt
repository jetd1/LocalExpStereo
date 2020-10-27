cmake_minimum_required(VERSION 2.8)
project(LocalExp)

add_definitions("-march=native -std=c++17 -Ofast -funroll-loops")

find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/LocalExpansionStereo/)

include_directories(${INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
add_executable(stereo_match LocalExpansionStereo/main.cpp maxflow/maxflow.cpp maxflow/graph.cpp)
target_link_libraries(stereo_match ${OpenCV_LIBRARIES})
