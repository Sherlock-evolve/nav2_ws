cmake_minimum_required(VERSION 3.8)
project(nav2_lab_behaviors)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(nav2_lab_observe_spin_behavior SHARED
  plugins/observe_spin.cpp
)

target_include_directories(nav2_lab_observe_spin_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav2_lab_observe_spin_behavior
  nav2_core
  nav2_behaviors
  nav2_util
  nav2_costmap_2d
  nav2_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

pluginlib_export_plugin_description_file(nav2_core nav2_lab_behaviors.xml)

install(
  TARGETS nav2_lab_observe_spin_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES nav2_lab_behaviors.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
