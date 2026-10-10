cmake_minimum_required(VERSION 3.10)
project(abinaraproject)
include_directories(include)
add_library(robot_lib 
    src/robot.cpp
    src/robot_sar.cpp
    src/robot_delivery.cpp
)
add_executable(abinara_app src/main.cpp)
target_link_libraries(abinara_app robot_lib)