cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE Release)

project(UseSophus)

find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)

# include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(useSophus useSophus.cpp)

target_link_libraries(useSophus Sophus::Sophus)