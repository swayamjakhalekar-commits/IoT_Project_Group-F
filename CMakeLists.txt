cmake_minimum_required(VERSION 3.10)
project(ipie_autonomous_car)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

file(GLOB_RECURSE SOURCES src/*.cpp)

add_executable(ipie_car ${SOURCES})

target_include_directories(ipie_car PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ipie_car
    ${OpenCV_LIBS}
    Threads::Threads
)
