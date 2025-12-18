cmake_minimum_required(VERSION 3.10)


project(BalanceCar VERSION 1.0 LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
find_package(Threads REQUIRED)

# 添加 src 子目录，CMake 会去 src 下找另一个 CMakeLists.txt
add_subdirectory(src)