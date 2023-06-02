cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE Release)

project(GaussNewton)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(gaussNewton gaussNewton.cpp)

target_link_libraries(gaussNewton ${OpenCV_LIBS})