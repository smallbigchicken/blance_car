# 1. 设置 CMake 最低版本要求
cmake_minimum_required(VERSION 3.10)

# 2. 项目名称
project(MotorControl)

# 3. 设置 C++ 标准 (推荐 C++11 或更高)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 4. 包含头文件目录 (如果有 .h 文件放在 include 文件夹下的话)
# include_directories(include)

# 5. 生成可执行文件
# 格式: add_executable(程序名 源码文件)
add_executable(motor_ctrl src/main.cpp)

# SocketCAN 依赖的是 Linux 内核头文件，通常不需要链接额外的库
# 如果以后用到多线程，可能需要加: target_link_libraries(motor_ctrl pthread)