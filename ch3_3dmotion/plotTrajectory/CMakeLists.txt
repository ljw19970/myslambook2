cmake_minimum_required(VERSION 2.8)

project(PlotTrajectory)

find_package(Pangolin REQUIRED)
include_directories("/usr/include/eigen3")  # 之后用find_package替代

add_executable(plotTrajectory plotTrajectory.cpp)

target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})
