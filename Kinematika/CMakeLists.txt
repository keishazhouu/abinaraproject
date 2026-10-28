cmake_minimum_required(VERSION 3.10)
project(kpp_ros_mujoco)

add_compile_options(-std=c++17 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

file(GLOB _mujoco_guess "$ENV{HOME}/.mujoco/mujoco-*")
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
  HINTS $ENV{MUJOCO_DIR} ${_mujoco_guess} PATH_SUFFIXES include)
find_library(MUJOCO_LIB mujoco
  HINTS $ENV{MUJOCO_DIR} ${_mujoco_guess} PATH_SUFFIXES lib)
if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIB)
  message(FATAL_ERROR "MuJoCo tidak ditemukan. Jalankan: export MUJOCO_DIR=/path/ke/mujoco-x.y.z")
endif()

catkin_package(CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${MUJOCO_INCLUDE_DIR})

add_executable(mujoco_sim_node
  src/mujoco_sim.cpp
  src/sim.cpp
  src/callback.cpp
  src/control.cpp
  src/ros_bridge.cpp)
target_link_libraries(mujoco_sim_node ${catkin_LIBRARIES} ${MUJOCO_LIB} glfw ${OPENGL_LIBRARIES})

add_executable(triangle_trajectory_node
  src/triangle_trajectory_node.cpp
  src/arm_kinematics.cpp)
target_link_libraries(triangle_trajectory_node ${catkin_LIBRARIES})

add_executable(sync_node src/sync_node.cpp)
target_link_libraries(sync_node ${catkin_LIBRARIES})

add_executable(ik_command_node
  src/ik_command_node.cpp
  src/inverse_kinematics.cpp
  src/arm_kinematics.cpp)
target_link_libraries(ik_command_node ${catkin_LIBRARIES})

install(TARGETS mujoco_sim_node triangle_trajectory_node sync_node ik_command_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
