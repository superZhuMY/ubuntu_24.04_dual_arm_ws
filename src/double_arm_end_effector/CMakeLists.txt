cmake_minimum_required(VERSION 3.8)
project(double_arm_end_effector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(double_arm_harvest_interfaces REQUIRED)

add_library(serial_bus_servo_protocol STATIC
  src/serial_bus_servo_protocol.cpp
  src/serial_bus.cpp
)
target_include_directories(serial_bus_servo_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(dual_gripper_controller src/dual_gripper_controller.cpp)
target_link_libraries(dual_gripper_controller serial_bus_servo_protocol)
ament_target_dependencies(dual_gripper_controller
  rclcpp
  rclcpp_action
  double_arm_harvest_interfaces
)

install(TARGETS
  serial_bus_servo_protocol
  dual_gripper_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_serial_bus_servo_protocol
    test/test_serial_bus_servo_protocol.cpp)
  target_link_libraries(test_serial_bus_servo_protocol serial_bus_servo_protocol)
endif()

ament_export_include_directories(include)
ament_package()
