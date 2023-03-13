# cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=1
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(camera_fusion)

find_package(OpenCV 4.1 REQUIRED)

# include Eigen library
include_directories("/usr/include/eigen3")

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executable for create matrix exercise
add_executable(compute_ttc_lidar src/compute_ttc_lidar.cpp src/structIO.cpp)

# add_executable(compute_ttc_lidar src/compute_ttc_lidar_ca.cpp src/structIO.cpp)
target_link_libraries(compute_ttc_lidar ${OpenCV_LIBRARIES})
