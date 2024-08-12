cmake_minimum_required(VERSION 3.8)
project(cougars_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(frost_interfaces REQUIRED)
find_package(dvl_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(moos_bridge src/moos_bridge.cpp)
ament_target_dependencies(moos_bridge rclcpp std_msgs frost_interfaces)

add_executable(fin_control src/fin_control.cpp)
ament_target_dependencies(fin_control rclcpp std_msgs frost_interfaces)

add_executable(dvl_convertor src/dvl_convertor.cpp)
ament_target_dependencies(dvl_convertor rclcpp std_msgs frost_interfaces geometry_msgs dvl_msgs)

add_executable(depth_convertor src/depth_convertor.cpp)
ament_target_dependencies(depth_convertor rclcpp std_msgs frost_interfaces geometry_msgs)

# Link libraries
target_link_libraries(moos_bridge)
target_link_libraries(fin_control)
target_link_libraries(dvl_convertor)
target_link_libraries(depth_convertor)
install(TARGETS
  moos_bridge
  fin_control
  dvl_convertor
  depth_convertor
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
