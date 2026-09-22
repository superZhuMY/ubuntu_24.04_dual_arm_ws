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

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/send_manual_harvest_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
