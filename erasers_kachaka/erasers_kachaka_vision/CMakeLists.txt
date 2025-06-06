cmake_minimum_required(VERSION 3.8)
project(erasers_kachaka_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kachaka_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(
  object_detection_visualizer
  src/object_detection_visualizer.cpp
)


ament_target_dependencies(
  object_detection_visualizer
  rclcpp
  sensor_msgs
  kachaka_interfaces
  cv_bridge
)


target_link_libraries(
  object_detection_visualizer 
  ${OpenCV_LIBRARIES}
)


install(TARGETS
  object_detection_visualizer
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