cmake_minimum_required(VERSION 3.14)
project(autoware_autonomous_emergency_braking)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(AEB_NODE ${PROJECT_NAME}_node)
ament_auto_add_library(${AEB_NODE} SHARED
  include/autoware/autonomous_emergency_braking/node.hpp
  src/node.cpp
)
rclcpp_components_register_node(${AEB_NODE}
  PLUGIN "autoware::motion::control::autonomous_emergency_braking::AEB"
  EXECUTABLE ${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
