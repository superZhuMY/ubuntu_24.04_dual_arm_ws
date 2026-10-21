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
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED IMPORTED_TARGET libmodbus)

add_library(gripper_modbus STATIC
  src/gripper_mapping.cpp
  src/modbus_rtu_bus.cpp
)
target_include_directories(gripper_modbus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gripper_modbus PUBLIC PkgConfig::MODBUS)

add_executable(dual_gripper_controller src/dual_gripper_controller.cpp)
target_link_libraries(dual_gripper_controller gripper_modbus)
ament_target_dependencies(dual_gripper_controller
  rclcpp
  rclcpp_action
  double_arm_harvest_interfaces
)

install(TARGETS
  gripper_modbus
  dual_gripper_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gripper_calibration_gui.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gripper_mapping test/test_gripper_mapping.cpp)
  target_link_libraries(test_gripper_mapping gripper_modbus)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gripper_raw_action test/test_gripper_raw_action.py
    TIMEOUT 60
    ENV
      "GRIPPER_CONTROLLER_EXECUTABLE=$<TARGET_FILE:dual_gripper_controller>"
      "ROS_DOMAIN_ID=174"
      "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
  )
endif()

ament_export_include_directories(include)
ament_package()
