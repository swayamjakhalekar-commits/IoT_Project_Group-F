cmake_minimum_required(VERSION 3.16)
project(AutonomousCar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=armv8-a -mtune=cortex-a72")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(autonomous_car main.cpp)
target_link_libraries(autonomous_car ${OpenCV_LIBS} pthread)
