cmake_minimum_required(VERSION 3.5)
project(elite_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  config
  launch
  meshes
  urdf
  world
  DESTINATION share/${PROJECT_NAME}/)

ament_package()