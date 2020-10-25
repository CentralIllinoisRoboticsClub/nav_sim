cmake_minimum_required(VERSION 3.5)
project(nav_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(nav_sim/)

ament_auto_find_build_dependencies()

ament_auto_add_executable(avoid_obs src/AvoidObs.cpp src/PotentialFields.cpp)
ament_auto_add_executable(astar src/Astar.cpp)
ament_auto_add_executable(nav_states src/NavStates.cpp)

install(DIRECTORY
  launch
  config
  
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  nav_sim/sim_bot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
