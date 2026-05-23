cmake_minimum_required(VERSION 3.10)
project(ndt_omp_relocalization)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
find_package(OpenMP REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Find the ndt_omp static library from ndt_omp_ros2 install
find_library(NDT_OMP_LIBRARY
  NAMES ndt_omp
  PATHS ${ndt_omp_ros2_DIR}/../../../lib
  NO_DEFAULT_PATH
)

if(NOT NDT_OMP_LIBRARY)
  message(FATAL_ERROR "Could not find libndt_omp library from ndt_omp_ros2 package")
endif()

message(STATUS "Found ndt_omp library: ${NDT_OMP_LIBRARY}")

add_library(${PROJECT_NAME} SHARED
  src/ndt_omp_relocalization.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
  ${NDT_OMP_LIBRARY}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_eigen
  tf2_ros
  pcl_conversions
  ndt_omp_ros2
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ndt_omp_relocalization::NdtOmpRelocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_eigen
  tf2_ros
  pcl_conversions
  ndt_omp_ros2
)

ament_package()
