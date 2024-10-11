cmake_minimum_required(VERSION 3.8)
project(cougars_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(frost_interfaces REQUIRED)
find_package(seatrac_interfaces REQUIRED)
find_package(dvl_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(MOOS 10 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include/${PROJECT_NAME})

add_executable(coug_controls src/coug_controls.cpp src/pid.cpp)
ament_target_dependencies(coug_controls rclcpp std_msgs frost_interfaces seatrac_interfaces geometry_msgs)

add_executable(coug_kinematics src/coug_kinematics.cpp)
ament_target_dependencies(coug_kinematics rclcpp std_msgs frost_interfaces)

add_executable(moos_bridge src/moos_bridge.cpp)
ament_target_dependencies(moos_bridge rclcpp std_msgs frost_interfaces MOOS nav_msgs)

# Link libraries
target_link_libraries(coug_controls)
target_link_libraries(coug_kinematics)
target_link_libraries(moos_bridge)

# Install executables
install(TARGETS
  coug_controls
  coug_kinematics
  moos_bridge
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/battery_sub.py
  scripts/leak_sub.py
  scripts/manual_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#Install Launch file
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
