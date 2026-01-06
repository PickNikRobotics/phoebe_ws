cmake_minimum_required(VERSION 3.22)
project(dual_arm_mobile_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
  meshes
  xacro
  DESTINATION
  share/${PROJECT_NAME}
  )

ament_package()
