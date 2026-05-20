cmake_minimum_required(VERSION 3.8)
project(dodge_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pb_rm_interfaces REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  pb_rm_interfaces
  auto_aim_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
)

include_directories(include)

add_executable(dodge_manager_node src/dodge_manager_node.cpp)
ament_target_dependencies(dodge_manager_node ${DEPENDENCIES})

install(TARGETS
  dodge_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
