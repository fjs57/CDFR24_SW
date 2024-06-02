cmake_minimum_required(VERSION 3.8)
project(robot_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hw_support_interfaces_pkg REQUIRED)

install(DIRECTORY
  launch
  config
  description
  DESTINATION share/${PROJECT_NAME}
)

# install(TARGETS

#   DESTINATION lib/${PROJECT_NAME}
# )

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  hw_support_interfaces_pkg
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE

include_directories(hardware/include/robot_pkg)

add_library(
  robot_pkg
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(robot_pkg PUBLIC cxx_std_17)
target_include_directories(robot_pkg PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/robot_pkg>
)
ament_target_dependencies(
  robot_pkg PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "robot_pkg_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface robot_pkg_plugin.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/robot_pkg
)

install(TARGETS robot_pkg
  EXPORT export_robot_pkg
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


## EXPORTS
ament_export_targets(export_robot_pkg HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


ament_package()
