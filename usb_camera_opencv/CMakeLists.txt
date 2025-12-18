cmake_minimum_required(VERSION 3.10)
project(usb_camera_opencv)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(usb_camera src/main.cpp)

target_link_libraries(usb_camera ${OpenCV_LIBS})
