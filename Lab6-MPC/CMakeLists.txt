cmake_minimum_required(VERSION 3.5)

project(SDCND-Lab6)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(global_kinematic_model)
add_subdirectory(polyfit)
add_subdirectory(mpc_to_line)
