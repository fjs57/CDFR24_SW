cmake_minimum_required(VERSION 3.8)
project(hw_support_pkg)

# set(CMAKE_STATIC_LIBRARY_PREFIX "")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hw_support_interfaces_pkg REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(include/hw_support_pkg)

add_executable(can_support_node src/CAN_support.cpp)
ament_target_dependencies(can_support_node rclcpp hw_support_interfaces_pkg)

add_executable(actuator_board_node src/ActuatorBoard.cpp src/GenericBoard.cpp)
ament_target_dependencies(actuator_board_node rclcpp hw_support_interfaces_pkg)

add_executable(motor_board_node src/MotorBoard.cpp src/GenericBoard.cpp)
ament_target_dependencies(motor_board_node rclcpp hw_support_interfaces_pkg std_msgs)

add_executable(distribution_board_node src/DistributionBoard.cpp src/GenericBoard.cpp)
ament_target_dependencies(distribution_board_node rclcpp hw_support_interfaces_pkg)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
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

ament_package()
