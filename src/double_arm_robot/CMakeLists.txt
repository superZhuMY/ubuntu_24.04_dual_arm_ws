cmake_minimum_required(VERSION 3.8)
project(double_arm_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    textures
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()