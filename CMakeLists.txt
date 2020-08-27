cmake_minimum_required(VERSION 3.5)
project(nav_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#add_executable(avoid_obs src/AvoidObs.cpp src/PotentialFields.cpp)
#add_executable(astar src/Astar.cpp)
ament_auto_add_executable(nav_states src/NavStates.cpp)

ament_auto_package()
