cmake_minimum_required(VERSION 3.10)
project(ipie_autonomous_car)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE SOURCES src/*.cpp)
add_executable(ipie_car ${SOURCES})
target_link_libraries(ipie_car ${OpenCV_LIBS})
