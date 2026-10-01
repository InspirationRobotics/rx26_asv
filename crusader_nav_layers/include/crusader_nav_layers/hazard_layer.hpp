// HazardLayer - the known hazards, drawn LETHAL, from /crsd/nav/hazards.
//
// docs/nav2_avoidance_spec.md section 3.3. STATELESS: it keeps the last
// HazardArray and redraws whatever falls inside the bounds each costmap cycle.
// Each message REPLACES the previous set, so a hazard that is gone disappears
// with no delete message; its old footprint is re-dirtied so the cells it
// covered are cleared by the master grid reset and not redrawn.
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "crusader_msgs/msg/hazard_array.hpp"
#include "crusader_nav_layers/raster.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crusader_nav_layers
{

class HazardLayer : public nav2_costmap_2d::Layer
{
public:
  HazardLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
    int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}
  void onFootprintChanged() override {}
  void matchSize() override {}

private:
  void onMsg(crusader_msgs::msg::HazardArray::ConstSharedPtr msg);

  rclcpp::Subscription<crusader_msgs::msg::HazardArray>::SharedPtr sub_;
  double max_age_s_ = 5.0;

  // Everything below is shared between the subscription thread and the
  // costmap's update thread.
  std::mutex mtx_;
  std::string frame_;                 // header.frame_id of the stored set
  std::vector<Shape> shapes_;         // the stored set, validated
  std::vector<Aabb> removed_;         // footprints of sets that were replaced
  bool have_msg_ = false;
  bool dirty_ = false;
  rclcpp::Time rx_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace crusader_nav_layers
