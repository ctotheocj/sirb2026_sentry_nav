#include "plan_env/grid_map.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace plan_env
{

class GridMapComponent : public rclcpp::Node
{
public:
  explicit GridMapComponent(const rclcpp::NodeOptions & options)
  : rclcpp::Node("grid_map_node", options)
  {
    // Defer initMap: shared_from_this() is invalid inside the constructor
    init_timer_ = create_wall_timer(std::chrono::milliseconds(0), [this]() {
      init_timer_->cancel();
      map_ = std::make_shared<GridMap>();
      map_->initMap(shared_from_this());
      GridMapRegistry::set(map_);
      RCLCPP_INFO(get_logger(), "GridMapComponent initialized, ESDF registry set");
    });
  }

private:
  GridMap::Ptr map_;
  rclcpp::TimerBase::SharedPtr init_timer_;
};

}  // namespace plan_env

RCLCPP_COMPONENTS_REGISTER_NODE(plan_env::GridMapComponent)
