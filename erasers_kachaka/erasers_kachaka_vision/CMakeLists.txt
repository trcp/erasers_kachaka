cmake_minimum_required(VERSION 3.8)
project(erasers_kachaka_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(kachaka_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(human_track src/human_track.cpp)
ament_target_dependencies(human_track
  rclcpp
  std_srvs
  geometry_msgs
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  message_filters
  kachaka_interfaces
  OpenCV
)

install(TARGETS
  human_track
  DESTINATION lib/${PROJECT_NAME}
)

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
