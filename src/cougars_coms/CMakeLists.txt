cmake_minimum_required(VERSION 3.8)
project(cougars_coms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(seatrac_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(cougars_coms src/cougars_coms.cpp)
ament_target_dependencies(cougars_coms rclcpp std_srvs seatrac_interfaces geometry_msgs)

install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME})

install(TARGETS
  cougars_coms
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()