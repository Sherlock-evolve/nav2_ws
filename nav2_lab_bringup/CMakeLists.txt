cmake_minimum_required(VERSION 3.8)
project(nav2_lab_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY behavior_trees launch maps params rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
