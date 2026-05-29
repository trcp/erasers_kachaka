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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rviz_common REQUIRED)

# Qt5 Settings
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

# Include Directories
include_directories(
  include
)

# teleop
add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop rclcpp geometry_msgs sensor_msgs)

# teleop joy
add_executable(keyboard_teleop src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop rclcpp geometry_msgs)

# teleop rviz
set(QT_MOC_HEADERS
  include/erasers_kachaka_teleop/joystick_panel.hpp
)
add_library(joystick_panel SHARED
  src/joystick_panel.cpp
  ${QT_MOC_HEADERS}
)
target_link_libraries(joystick_panel
  Qt5::Widgets
)
ament_target_dependencies(joystick_panel
  rclcpp
  rviz_common
  geometry_msgs
)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# install nodes
install(TARGETS
  teleop
  keyboard_teleop
  joystick_panel
  DESTINATION lib/${PROJECT_NAME}
)

# include dirs
install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
