cmake_minimum_required(VERSION 3.8)
project(erasers_kachaka_cartographer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(auto_map_saver src/auto_map_saver.cpp)
ament_target_dependencies(auto_map_saver rclcpp)

install(TARGETS
  auto_map_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  map
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
