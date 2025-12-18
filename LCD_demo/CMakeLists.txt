cmake_minimum_required(VERSION 3.10)

# 项目只使用 CXX (C++)，不需要 C 了
project(LCD_demo CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含头文件路径
include_directories(inc src/font)

# 定义源文件 (注意这里全都是 .cpp 了)
set(SOURCE_FILES
    src/adxl345.cpp
    src/LCD_demo.cpp
    src/LCD_driver.cpp
    src/LCD_gui.cpp
    
    # 字体文件也改成了 cpp
    src/font/font8.cpp
    src/font/font12.cpp
    src/font/font16.cpp
    src/font/font20.cpp
    src/font/font24.cpp
)

add_executable(LCD_demo ${SOURCE_FILES})