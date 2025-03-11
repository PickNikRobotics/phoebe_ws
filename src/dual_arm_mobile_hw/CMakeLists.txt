cmake_minimum_required(VERSION 3.22)
project(dual_arm_mobile_hw)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    objectives
    launch
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/odometry_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
