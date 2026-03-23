cmake_minimum_required(VERSION 3.16)
project(AutonomousCar)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=armv8-a -mtune=cortex-a72")

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

# Add executable (UPDATED PATH)
add_executable(autonomous_car src/main.cpp)

# Link libraries
target_link_libraries(autonomous_car ${OpenCV_LIBS} pthread)