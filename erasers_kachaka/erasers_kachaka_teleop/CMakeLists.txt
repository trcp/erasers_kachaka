cmake_minimum_required(VERSION 3.8)
project(erasers_kachaka_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop rclcpp geometry_msgs sensor_msgs)

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop rclcpp geometry_msgs)

# install nodes
install(TARGETS
  teleop
  keyboard_teleop
  DESTINATION lib/${PROJECT_NAME}
)

# include dirs
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
