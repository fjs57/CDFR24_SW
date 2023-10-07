cmake_minimum_required(VERSION 3.8)
project(debug_tools_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hw_support_interfaces_pkg REQUIRED)

add_executable(debugger_motors src/DebuggerMotors.cpp)
ament_target_dependencies(debugger_motors rclcpp hw_support_interfaces_pkg)

install(TARGETS
  debugger_motors
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
