cmake_minimum_required(VERSION 2.8)

# 只有当编译模式为debug, C++代码中的assert才能起作用
set(CMAKE_BUILD_TYPE "Debug")

project(TrajectoryError)

find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
# Ubuntu 20.04需要使用fmt库
find_package(fmt REQUIRED)
set(FMT_LIBRARIES fmt::fmt)

include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(trajectoryError trajectoryError.cpp)

target_link_libraries(trajectoryError ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} ${FMT_LIBRARIES})