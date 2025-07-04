cmake_minimum_required(VERSION 3.8)
project(erasers_kachaka_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

# add executable
add_executable(pot_fields_node src/pot_fields_node.cpp)
ament_target_dependencies(pot_fields_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)
add_executable(leg_finder_node src/leg_finder_node.cpp)
ament_target_dependencies(leg_finder_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
)

# install targets
install(TARGETS
  pot_fields_node
  leg_finder_node
  DESTINATION lib/${PROJECT_NAME}
)

# install directories
install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
