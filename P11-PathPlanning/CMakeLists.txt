project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  include_directories(/usr/local/include)
  include_directories(/usr/local/opt/openssl/include)
  link_directories(/usr/local/lib)
  link_directories(/usr/local/opt/openssl/lib)
  link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(src/Eigen-3.3)
set(sources
  README.md
  data/highway_map.csv
  src/p11_helper.h
  src/p11_helper.cpp
  src/trajectory_planner.h
  src/trajectory_planner.cpp
  src/behaviour.h
  src/behaviour_planner.h
  src/behaviour_planner.cpp
  src/waypoint.h
  src/waypoint.cpp
  src/vehicle.h
  src/vehicle.cpp)
add_executable(path_planning src/main.cpp ${sources})
target_link_libraries(path_planning z ssl uv uWS)

add_executable(test_poly src/test_poly.cpp ${sources})
