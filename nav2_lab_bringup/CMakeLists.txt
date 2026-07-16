cmake_minimum_required(VERSION 3.8)
project(nav2_lab_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY behavior_trees launch maps params rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_plugin_selection test/test_plugin_selection.py)
endif()

ament_package()
