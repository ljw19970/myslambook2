# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 2.8)

# 声明 cmake 工程
project(HelloSLAM)

# 设置编译模式 (Debug vs. Release)
set(CMAKE_BUILD_TYPE "Debug")

# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable(helloSLAM helloSLAM.cpp)
add_executable(useHello useHello.cpp)
add_executable(gdbTest gdbTest.cpp)

# 添加库
# add_library( hello libHelloSLAM.cpp ) # 添加静态hello库
add_library(hello_shared SHARED libHelloSLAM.cpp) # 添加共享hello库

# 将库文件链接到可执行程序上
target_link_libraries(useHello hello_shared)