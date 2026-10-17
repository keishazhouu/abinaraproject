cmake_minimum_required(VERSION 3.16)
project(abinaraproject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(abinara_sim src/main.cpp)

target_include_directories(abinara_sim PRIVATE
    "$ENV{HOME}/mujoco/include"
)

target_link_directories(abinara_sim PRIVATE
    "$ENV{HOME}/mujoco/build/lib"
)

target_link_libraries(abinara_sim PRIVATE
    mujoco
    glfw
)