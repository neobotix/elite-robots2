cmake_minimum_required(VERSION 3.5)
project(elite_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}/)

ament_package()