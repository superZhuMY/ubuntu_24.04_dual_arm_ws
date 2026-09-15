cmake_minimum_required(VERSION 3.8)
project(double_arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# libmodbus (optional — RealTransport can compile without it)
find_library(MODBUS_LIB modbus)
if(MODBUS_LIB)
  message(STATUS "libmodbus found: ${MODBUS_LIB}")
else()
  message(STATUS "libmodbus NOT found — RealTransport Modbus backend will be stub")
endif()

# ── Shared library: ArmSystemHardware plugin ──────────────────────
add_library(${PROJECT_NAME} SHARED
  src/arm_system_hardware.cpp
  src/stm32_system_hardware.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# ── Readonly diagnostic tool (E.2) ──────────────────────────────
add_executable(readonly_check src/readonly_check.cpp)
target_include_directories(readonly_check PRIVATE include)
target_link_libraries(readonly_check ${MODBUS_LIB})
ament_target_dependencies(readonly_check rclcpp)
install(TARGETS readonly_check DESTINATION lib/${PROJECT_NAME})

# Plugin description
pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # ── motor_converter tests ──────────────────────────────────────
  ament_add_gtest(test_motor_converter
    test/test_motor_converter.cpp
  )
  target_include_directories(test_motor_converter PRIVATE include)

  # ── serial_protocol tests ──────────────────────────────────────
  ament_add_gtest(test_serial_protocol
    test/test_serial_protocol.cpp
  )
  target_include_directories(test_serial_protocol PRIVATE include)

  # ── modbus_protocol tests ──────────────────────────────────────
  ament_add_gtest(test_modbus_protocol
    test/test_modbus_protocol.cpp
  )
  target_include_directories(test_modbus_protocol PRIVATE include)

  # ── ArmSystemHardware plugin tests ─────────────────────────────
  ament_add_gtest(test_arm_system_hardware
    test/test_arm_system_hardware.cpp
  )
  target_include_directories(test_arm_system_hardware PRIVATE include)
  target_link_libraries(test_arm_system_hardware ${PROJECT_NAME})
  ament_target_dependencies(test_arm_system_hardware
    hardware_interface
    rclcpp
    rclcpp_lifecycle
  )

  # ── Legacy communication sequence tests ────────────────────────
  ament_add_gtest(test_legacy_communication_sequence
    test/test_legacy_communication_sequence.cpp
  )
  target_include_directories(test_legacy_communication_sequence PRIVATE include)
  target_link_libraries(test_legacy_communication_sequence ${PROJECT_NAME})
  ament_target_dependencies(test_legacy_communication_sequence
    hardware_interface
    rclcpp
    rclcpp_lifecycle
  )

  # ── RealTransport tests ────────────────────────────────────────
  ament_add_gtest(test_real_transport
    test/test_real_transport.cpp
  )
  target_include_directories(test_real_transport PRIVATE include)
  target_link_libraries(test_real_transport ${PROJECT_NAME})
  ament_target_dependencies(test_real_transport
    hardware_interface
    rclcpp
    rclcpp_lifecycle
  )

  # ── E.2 readonly check Mock safety test ────────────────────────
  ament_add_gtest(test_readonly_check
    test/test_readonly_check.cpp
  )
  target_include_directories(test_readonly_check PRIVATE include)
  target_link_libraries(test_readonly_check ${MODBUS_LIB})
  ament_target_dependencies(test_readonly_check rclcpp)

  # ── E.3 safe-hold gate tests (offline, mock backends only) ────
  ament_add_gtest(test_e3_safe_hold
    test/test_e3_safe_hold.cpp
  )
  target_include_directories(test_e3_safe_hold PRIVATE include)
  target_link_libraries(test_e3_safe_hold ${PROJECT_NAME})
  ament_target_dependencies(test_e3_safe_hold
    hardware_interface
    rclcpp
    rclcpp_lifecycle
  )

  # ── E.4 step-wise homing tests (offline, mock backends only) ───
  ament_add_gtest(test_e4_step_home
    test/test_e4_step_home.cpp
  )
  target_include_directories(test_e4_step_home PRIVATE include)
  target_link_libraries(test_e4_step_home ${PROJECT_NAME})
  ament_target_dependencies(test_e4_step_home
    hardware_interface
    rclcpp
    rclcpp_lifecycle
  )

  # ── STM32 protocol codec tests (F.1, header-only, offline) ─────
  ament_add_gtest(test_stm32_protocol
    test/test_stm32_protocol.cpp
  )
  target_include_directories(test_stm32_protocol PRIVATE include)

  # ── STM32 stream parser tests (F.1, header-only, offline) ──────
  ament_add_gtest(test_stm32_stream_parser
    test/test_stm32_stream_parser.cpp
  )
  target_include_directories(test_stm32_stream_parser PRIVATE include)

  # ── STM32 transport tests (F.1, mock serial, offline) ──────────
  ament_add_gtest(test_stm32_transport
    test/test_stm32_transport.cpp
  )
  target_include_directories(test_stm32_transport PRIVATE include)

  # ── Stm32Backend tests (F.1, mock serial, offline) ─────────────
  ament_add_gtest(test_stm32_backend
    test/test_stm32_backend.cpp
  )
  target_include_directories(test_stm32_backend PRIVATE include)

  # ── Stm32SystemHardware plugin tests (F.1, mock serial, offline) ─
  ament_add_gtest(test_stm32_system_hardware
    test/test_stm32_system_hardware.cpp
  )
  target_include_directories(test_stm32_system_hardware PRIVATE include)
  target_link_libraries(test_stm32_system_hardware ${PROJECT_NAME})
  ament_target_dependencies(test_stm32_system_hardware
    hardware_interface
    rclcpp
    rclcpp_lifecycle
  )

  # ── Xacro / launch static tests (F.1: STM32 backend selection) ──
  ament_add_gtest(test_stm32_xacro
    test/test_stm32_xacro.cpp
  )
  ament_target_dependencies(test_stm32_xacro rclcpp)
endif()

# Link libmodbus (required for RealModbusBackend)
if(NOT MODBUS_LIB)
  message(FATAL_ERROR "libmodbus is required. Install: sudo apt install libmodbus-dev")
endif()
target_link_libraries(${PROJECT_NAME} ${MODBUS_LIB})

ament_package()
