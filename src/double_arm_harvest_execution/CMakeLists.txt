cmake_minimum_required(VERSION 3.8)
project(double_arm_harvest_execution)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

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
  src/task_request.cpp
  src/task_runtime.cpp
  src/task_behavior.cpp
  src/task_workflow.cpp
  src/task_skills.cpp
  src/motion_skill.cpp
  src/gripper_skill.cpp
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
ament_python_install_package(double_arm_harvest_workflows)
install(PROGRAMS
  scripts/send_manual_harvest_goal.py
  scripts/harvest_workflow_worker.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_pose_utils test/test_pose_utils.cpp)
  target_include_directories(test_pose_utils PRIVATE include)
  target_link_libraries(test_pose_utils harvest_core)

  ament_add_gtest(test_stage_barrier test/test_stage_barrier.cpp)
  target_include_directories(test_stage_barrier PRIVATE include)
  target_link_libraries(test_stage_barrier harvest_core)

  ament_add_gtest(test_trajectory_dispatcher test/test_trajectory_dispatcher.cpp)
  target_include_directories(test_trajectory_dispatcher PRIVATE include)
  target_link_libraries(test_trajectory_dispatcher harvest_core)

  ament_add_gtest(test_failure_policy test/test_failure_policy.cpp)
  target_include_directories(test_failure_policy PRIVATE include)
  target_link_libraries(test_failure_policy harvest_core)

  ament_add_gtest(test_execute_task_schema test/test_execute_task_schema.cpp)
  ament_target_dependencies(test_execute_task_schema double_arm_harvest_interfaces)

  ament_add_gtest(test_task_request test/test_task_request.cpp)
  target_include_directories(test_task_request PRIVATE include)
  target_link_libraries(test_task_request harvest_core)

  ament_add_gtest(test_task_runtime test/test_task_runtime.cpp)
  target_include_directories(test_task_runtime PRIVATE include)
  target_link_libraries(test_task_runtime harvest_core)

  ament_add_gtest(test_task_behavior test/test_task_behavior.cpp)
  target_include_directories(test_task_behavior PRIVATE include)
  target_link_libraries(test_task_behavior harvest_core)

  ament_add_pytest_test(test_manual_target_conversion test/test_manual_target_conversion.py)
  ament_add_pytest_test(test_manual_task_sender test/test_manual_task_sender.py)
  ament_add_pytest_test(test_python_workflows test/test_python_workflows.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})

  ament_add_gtest(test_gripper_skill test/test_gripper_skill.cpp)
  target_include_directories(test_gripper_skill PRIVATE include)
  target_link_libraries(test_gripper_skill harvest_core)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_executable(test_task_executor_submit_fault test/test_task_executor_submit_fault.cpp)
  target_link_libraries(test_task_executor_submit_fault harvest_core)
  ament_target_dependencies(test_task_executor_submit_fault rclcpp rclcpp_action)
  add_launch_test(test/test_task_partial_send_integration.py TIMEOUT "180"
    ARGS "fault_executable:=$<TARGET_FILE:test_task_executor_submit_fault>")
  add_launch_test(test/test_fake_harvest_integration.py TIMEOUT "600")
  add_launch_test(test/test_fake_harvest_leaf_integration.py TIMEOUT "600")
  add_launch_test(test/test_task_recovery_integration.py TIMEOUT "600")
  add_launch_test(test/test_fake_harvest_pause.py TIMEOUT "180")
  add_launch_test(test/test_fake_harvest_leaf_integration.py TIMEOUT "600"
    TARGET test_python_workflow_fake_integration ARGS "workflow_backend:=python")
  add_launch_test(test/test_task_recovery_integration.py TIMEOUT "600"
    TARGET test_python_workflow_recovery_integration ARGS "workflow_backend:=python")
  add_launch_test(test/test_task_partial_send_integration.py TIMEOUT "180"
    TARGET test_python_workflow_partial_send_integration
    ARGS "fault_executable:=$<TARGET_FILE:test_task_executor_submit_fault>"
    "workflow_backend:=python")
  add_launch_test(test/test_python_workflow_faults.py TIMEOUT "180")
endif()

ament_package()
