cmake_minimum_required(VERSION 3.8)
project(double_arm_harvest_execution)

find_package(ament_cmake REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(double_arm_harvest_interfaces REQUIRED)

set(HARVEST_DEPS
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_msgs
  control_msgs
  trajectory_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  double_arm_harvest_interfaces
)

add_library(harvest_core STATIC
  src/pose_utils.cpp
  src/failure_policy.cpp
  src/stage_barrier.cpp
  src/trajectory_dispatcher.cpp
  src/dual_harvest_executor.cpp
)
target_include_directories(harvest_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(harvest_core ${HARVEST_DEPS})

add_executable(dual_arm_harvest_executor src/main.cpp)
target_link_libraries(dual_arm_harvest_executor harvest_core)
ament_target_dependencies(dual_arm_harvest_executor rclcpp rclcpp_action)

install(TARGETS dual_arm_harvest_executor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/send_manual_harvest_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pose_utils test/test_pose_utils.cpp)
  target_include_directories(test_pose_utils PRIVATE include)
  target_link_libraries(test_pose_utils harvest_core)

  ament_add_gtest(test_stage_barrier test/test_stage_barrier.cpp)
  target_include_directories(test_stage_barrier PRIVATE include)
  target_link_libraries(test_stage_barrier harvest_core)

  ament_add_gtest(test_sync_stamp test/test_sync_stamp.cpp)
  target_include_directories(test_sync_stamp PRIVATE include)
  target_link_libraries(test_sync_stamp harvest_core)
  ament_target_dependencies(test_sync_stamp rclcpp trajectory_msgs)

  ament_add_gtest(test_failure_policy test/test_failure_policy.cpp)
  target_include_directories(test_failure_policy PRIVATE include)
  target_link_libraries(test_failure_policy harvest_core)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_fake_harvest_integration.py TIMEOUT "600")
endif()

ament_package()
