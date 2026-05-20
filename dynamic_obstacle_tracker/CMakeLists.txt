cmake_minimum_required(VERSION 3.8)
project(dynamic_obstacle_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search segmentation)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sentry_nav_interfaces REQUIRED)

add_executable(obstacle_tracker_node
  src/obstacle_tracker_node.cpp
)

target_include_directories(obstacle_tracker_node
  PRIVATE ${PCL_INCLUDE_DIRS} include
)

target_link_libraries(obstacle_tracker_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(obstacle_tracker_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  sentry_nav_interfaces
)

install(TARGETS obstacle_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)


ament_package()
