cmake_minimum_required(VERSION 3.8)
project(double_arm_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY textures DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
