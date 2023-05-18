cmake_minimum_required(VERSION 2.8)

project(UseGeometry)

add_executable(useGeometry useGeometry.cpp)

include_directories("/usr/include/eigen3")  # 之后用find_package替代