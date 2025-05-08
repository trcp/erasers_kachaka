cmake_minimum_required(VERSION 3.8)
project(erasers_kachaka_cartographer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# map_saver
add_executable(map_saver src/map_saver.cpp)
ament_target_dependencies(map_saver rclcpp)

# occupancy_grid_map_creator
add_library(occupancy_grid_map_creator SHARED src/occupancy_grid_map_creator.cpp)
target_include_directories(occupancy_grid_map_creator PUBLIC
  include
)
ament_target_dependencies(occupancy_grid_map_creator rclcpp nav_msgs)

# map_providor
add_executable(map_providor src/map_providor.cpp)
target_include_directories(map_providor PRIVATE
  include
)
target_link_libraries(map_providor
  occupancy_grid_map_creator
)
ament_target_dependencies(map_providor rclcpp nav_msgs)

# インストール設定
install(TARGETS
  map_saver
  occupancy_grid_map_creator
  map_providor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  map
  config
  params
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  include
  DESTINATION include
)

# テスト設定
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()