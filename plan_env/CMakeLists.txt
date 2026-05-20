cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)

project(plan_env)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED)

add_library(plan_env SHARED
  src/grid_map.cpp
  src/raycast.cpp
)

target_include_directories(plan_env PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(plan_env
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(plan_env
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

add_executable(grid_map_node src/grid_map_node.cpp)

target_link_libraries(grid_map_node plan_env)
ament_target_dependencies(grid_map_node rclcpp)

add_library(grid_map_component SHARED src/grid_map_component.cpp)
target_link_libraries(grid_map_component plan_env)
ament_target_dependencies(grid_map_component rclcpp rclcpp_components)
rclcpp_components_register_node(grid_map_component
  PLUGIN "plan_env::GridMapComponent"
  EXECUTABLE grid_map_component_node)

install(
  TARGETS plan_env
  EXPORT plan_envTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS grid_map_component DESTINATION lib)
install(TARGETS grid_map_component_node DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS grid_map_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(plan_envTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  Eigen3
)

ament_package()
