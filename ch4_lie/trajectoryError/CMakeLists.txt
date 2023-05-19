cmake_minimum_required(VERSION 2.8)

project(UseSophus)

find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(fmt REQUIRED)
set(FMT_LIBRARIES fmt::fmt)

# include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(trajectoryError trajectoryError.cpp)

# target_link_libraries(trajectoryError Sophus::Sophus ${Pangolin_LIBRARIES})
target_link_libraries(trajectoryError ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} ${FMT_LIBRARIES})