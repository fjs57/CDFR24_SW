cmake_minimum_required(VERSION 3.8)
project(hw_support_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hw_support_interfaces_pkg REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include/hw_support_pkg)

add_executable(can_support_node src/CAN_support.cpp)
ament_target_dependencies(can_support_node rclcpp hw_support_interfaces_pkg)

add_executable(actuator_board_node src/ActuatorBoard.cpp src/GenericBoard.cpp)
ament_target_dependencies(actuator_board_node rclcpp hw_support_interfaces_pkg)

add_executable(motor_board_node src/MotorBoard.cpp src/GenericBoard.cpp)
ament_target_dependencies(motor_board_node rclcpp hw_support_interfaces_pkg)

add_executable(distribution_board_node src/DistributionBoard.cpp src/GenericBoard.cpp)
ament_target_dependencies(distribution_board_node rclcpp hw_support_interfaces_pkg)

add_library(steering_hardware_interface src/DistributionBoard.cpp)

ament_target_dependencies(
  steering_hardware_interface
  controller_manager
  hardware_interface
  rclcpp
  hw_support_interfaces_pkg
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

install(
  TARGETS steering_hardware_interface
  DESTINATION lib
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  can_support_node
  # generic_board_node
  actuator_board_node
  motor_board_node
  distribution_board_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(
  steering_hardware_interface
)

ament_package()
