cmake_minimum_required(VERSION 3.5)
project(elite_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  meshes
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}/)

ament_package()