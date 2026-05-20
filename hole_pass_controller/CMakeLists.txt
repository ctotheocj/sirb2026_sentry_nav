cmake_minimum_required(VERSION 3.8)
project(hole_pass_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sentry_nav_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/hole_pass_controller.cpp
  src/cmd_vel_mux_node.cpp
  src/navigation_mode_manager_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sentry_nav_interfaces
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "hole_pass_controller::HolePassController"
  EXECUTABLE hole_pass_controller_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "hole_pass_controller::CmdVelMuxNode"
  EXECUTABLE cmd_vel_mux_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "hole_pass_controller::NavigationModeManagerNode"
  EXECUTABLE navigation_mode_manager_node
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS hole_pass_controller_node cmd_vel_mux_node navigation_mode_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
