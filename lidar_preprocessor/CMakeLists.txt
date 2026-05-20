cmake_minimum_required(VERSION 3.8)
project(lidar_preprocessor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED)

add_executable(lidar_preprocessor_node
  src/lidar_preprocessor_node.cpp
)

target_include_directories(lidar_preprocessor_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${PCL_INCLUDE_DIRS}
)

target_link_libraries(lidar_preprocessor_node ${PCL_LIBRARIES})

ament_target_dependencies(lidar_preprocessor_node
  rclcpp sensor_msgs pcl_conversions Eigen3
)

install(TARGETS lidar_preprocessor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
