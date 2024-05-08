cmake_minimum_required(VERSION 3.8)
project(seatrac)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(frost_interfaces REQUIRED)

#find_package(seatrac_driver REQUIRED)
if(NOT TARGET seatrac_driver)
    find_package(seatrac_driver QUIET)
    if(NOT TARGET seatrac_driver)
        include(FetchContent)
        FetchContent_Declare(seatrac_driver
            GIT_REPOSITORY https://Clayton314@bitbucket.org/frostlab/seatrac_driver.git
            GIT_TAG        main
        )
        FetchContent_MakeAvailable(seatrac_driver)
    endif()
endif()

add_executable(modem src/modem_ros_node.cpp)
ament_target_dependencies(modem rclcpp std_msgs frost_interfaces)
target_link_libraries(modem
seatrac_driver
)

add_executable(calibrate_modem src/calibrate_modem.cpp)
# ament_target_dependencies(calibrate_modem rclcpp std_msgs frost_interfaces)
target_link_libraries(calibrate_modem
seatrac_driver
)

install(TARGETS
  modem
  calibrate_modem
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()