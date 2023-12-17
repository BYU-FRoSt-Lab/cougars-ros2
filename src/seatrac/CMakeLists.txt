cmake_minimum_required(VERSION 3.8)
project(seatrac)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(seatrac_driver REQUIRED)

add_executable(modem_pub src/modem_pub.cpp)
ament_target_dependencies(modem_pub rclcpp std_msgs)
target_link_libraries(modem_pub
    seatrac_driver
)

install(TARGETS
  modem_pub
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()