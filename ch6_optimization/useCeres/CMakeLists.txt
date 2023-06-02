cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE Release)

project(UseCeres)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

add_executable(useCeres useCeres.cpp)

target_link_libraries(useCeres ${OpenCV_LIBS} ${CERES_LIBRARIES})