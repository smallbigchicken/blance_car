# 1. 指定CMake最小版本（参考实验一CMakeLists例程，需与开发板环境兼容）
cmake_minimum_required(VERSION 3.10)

# 2. 定义项目名称（自定义，需与实验功能关联）
project(KeyLEDControl)

# 3. 交叉编译器配置（关键：匹配Atlas200开发板aarch架构，参考实验一3-399段）
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # 开发板本地编译：使用系统默认gcc（Ascend310处理器为aarch64架构）
    set(CMAKE_C_COMPILER gcc)
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    # x86虚拟机交叉编译：指定昇腾交叉编译器路径（参考实验一交叉编译逻辑）
    set(CMAKE_C_COMPILER "/opt/compiler/bin/aarch64-linux-gnu-gcc")
else ()
    message(FATAL_ERROR "Unsupported architecture! Only aarch64 (Atlas200) and x86_64 (cross-compile) are allowed.")
endif()

# 4. 设置可执行文件输出目录（参考实验一3-400段，统一输出到out目录，便于管理）
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/out")

# 5. 配置编译选项（参考实验一Makefile的-Wall警告输出，确保代码规范性）
set(CMAKE_C_FLAGS "-Wall -O2")  # -Wall：输出所有警告；-O2：优化编译
set(CMAKE_EXE_LINKER_FLAGS "-lrt")  # 链接rt库（解决clock()函数依赖问题，参考实验代码编译需求）

# 6. 指定源文件（按键控制实验的核心代码文件）
set(SOURCE_FILES key_led_task.c)

# 7. 生成可执行文件（可执行文件名：key_led_task，与源文件对应）
add_executable(key_led_task ${SOURCE_FILES})

# 8. 链接依赖库（此处仅需系统rt库，已在链接选项中指定，若后续扩展可添加其他库）
target_link_libraries(key_led_task rt)