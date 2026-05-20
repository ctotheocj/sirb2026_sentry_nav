// Copyright 2025 Pan — Apache-2.0

#include "plan_env/grid_map.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("grid_map_node");
  auto map = std::make_shared<plan_env::GridMap>();
  map->initMap(node);
  plan_env::GridMapRegistry::set(map);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
