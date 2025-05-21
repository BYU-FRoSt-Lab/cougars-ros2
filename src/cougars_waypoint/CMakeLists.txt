cmake_minimum_required(VERSION 3.8)
project(cougars_waypoint) # CHANGED

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}) # Uses 'cougars_waypoint'

install(
    DIRECTORY launch config resource cougars_waypoint # CHANGED python package dir name
    DESTINATION share/${PROJECT_NAME} # Uses 'cougars_waypoint'
)

ament_package()