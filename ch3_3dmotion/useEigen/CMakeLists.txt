cmake_minimum_required(VERSION 2.8)

project(UseEigen)

add_executable(eigenMatrix eigenMatrix.cpp)

include_directories("/usr/include/eigen3")  # 之后用find_package替代