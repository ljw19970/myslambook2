cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)  # g2o库需要用c++17标准编译

project(UseG2o)

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# g2o
# 将当前路径下的cmake_modules文件夹添加到$CMAKE_MODULE_PATH中
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear
 g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse 
 g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension 
 g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg
 g2o_types_data g2o_types_sim3  g2o_solver_cholmod cxsparse)


add_executable(useG2o useG2o.cpp)

target_link_libraries(useG2o ${OpenCV_LIBS} ${G2O_LIBS})
