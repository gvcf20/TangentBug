cmake_minimum_required(VERSION 3.8)
project(nav_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf worlds config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()