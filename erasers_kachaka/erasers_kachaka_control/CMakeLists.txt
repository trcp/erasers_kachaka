cmake_minimum_required(VERSION 3.8)
project(erasers_kachaka_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# teleop joy setup
add_executable(teleop_joy src/teleop_joy.cpp)
ament_target_dependencies(teleop_joy rclcpp geometry_msgs sensor_msgs)
# emc_joy
add_executable(emc_joy src/emc_joy.cpp)
ament_target_dependencies(emc_joy rclcpp sensor_msgs std_srvs)

# install nodes
install(TARGETS
  teleop_joy
  emc_joy
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
