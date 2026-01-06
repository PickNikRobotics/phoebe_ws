cmake_minimum_required(VERSION 3.22)
project(dual_arm_mobile_hw)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    assets
    config
    description
    models 
    objectives
    launch
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/odom_offset_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
