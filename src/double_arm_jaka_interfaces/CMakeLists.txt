cmake_minimum_required(VERSION 3.8)
project(double_arm_jaka_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Command.msg"
  "msg/JakaIo.msg"
  "msg/Zitai.msg"
  "msg/Zitai1.msg"
  "action/ObjectPose.action"
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    trajectory_msgs
    control_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()