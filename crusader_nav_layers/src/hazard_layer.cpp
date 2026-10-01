#include "crusader_nav_layers/hazard_layer.hpp"

#include <algorithm>
#include <functional>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace crusader_nav_layers
{

namespace
{

/// Collapse `boxes` to their union once there are more than this many. A layer
/// that is never asked to update (disabled) must not grow without bound at 2 Hz.
constexpr std::size_t kMaxRemovedBoxes = 64;

void expand(const Aabb & b, double * min_x, double * min_y, double * max_x, double * max_y)
{
  *min_x = std::min(*min_x, b.x0);
  *min_y = std::min(*min_y, b.y0);
  *max_x = std::max(*max_x, b.x1);
  *max_y = std::max(*max_y, b.y1);
}

}  // namespace

void HazardLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"HazardLayer: failed to lock the costmap node"};
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/crsd/nav/hazards")));
  declareParameter("max_age_s", rclcpp::ParameterValue(5.0));
  std::string topic;
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic);
  node->get_parameter(name_ + ".max_age_s", max_age_s_);
  current_ = false;     // nothing received yet: not current, so nothing plans blind

  // Reliable + transient_local + depth 1 is the publisher's profile: a layer
  // that starts after bt_runner still gets the last whole set.
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();
  // The costmap's executor spins ONLY callback_group_, never the node's
  // default group, so a subscription made without it is never serviced.
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = callback_group_;
  sub_ = node->create_subscription<crusader_msgs::msg::HazardArray>(
    topic, qos, std::bind(&HazardLayer::onMsg, this, std::placeholders::_1), opts);
  RCLCPP_INFO(
    logger_, "%s: hazards from %s (max_age_s %.1f)", name_.c_str(), topic.c_str(), max_age_s_);
}

void HazardLayer::onMsg(crusader_msgs::msg::HazardArray::ConstSharedPtr msg)
{
  std::vector<Shape> shapes;
  shapes.reserve(msg->hazards.size());
  int dropped = 0;
  std::string why;
  for (const auto & h : msg->hazards) {
    Shape s;
    if (makeShape(
        h.kind, h.x, h.y, h.radius_m, h.polygon_x, h.polygon_y, h.keepout_m, &s, &why))
    {
      shapes.push_back(std::move(s));
    } else {
      ++dropped;
      RCLCPP_ERROR_THROTTLE(
        logger_, *clock_, 5000, "%s: hazard id %d dropped: %s", name_.c_str(), h.id, why.c_str());
    }
  }
  if (dropped > 0) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000, "%s: %d of %zu hazards could not be placed and are NOT drawn",
      name_.c_str(), dropped, msg->hazards.size());
  }

  const double res = layered_costmap_->getCostmap()->getResolution();
  std::lock_guard<std::mutex> lock(mtx_);
  for (const Shape & old : shapes_) {removed_.push_back(shapeBounds(old, res));}
  if (removed_.size() > kMaxRemovedBoxes) {
    Aabb u = removed_.front();
    for (const Aabb & b : removed_) {expand(b, &u.x0, &u.y0, &u.x1, &u.y1);}
    removed_.assign(1, u);
  }
  shapes_ = std::move(shapes);
  frame_ = msg->header.frame_id;
  have_msg_ = true;
  dirty_ = true;
  rx_time_ = clock_->now();
}

void HazardLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {return;}
  const double res = layered_costmap_->getCostmap()->getResolution();
  std::lock_guard<std::mutex> lock(mtx_);

  // A dead bt_runner makes the costmap non-current, which stops plans rather
  // than planning around hazards that may have moved.
  current_ = have_msg_ && (clock_->now() - rx_time_).seconds() <= max_age_s_;
  if (have_msg_ && frame_ != layered_costmap_->getGlobalFrameID()) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000, "%s: hazards are in frame '%s' but the costmap is in '%s': IGNORED",
      name_.c_str(), frame_.c_str(), layered_costmap_->getGlobalFrameID().c_str());
    current_ = false;
    return;
  }
  if (!have_msg_) {return;}

  if (layered_costmap_->isRolling() || dirty_) {
    for (const Shape & s : shapes_) {
      expand(shapeBounds(s, res), min_x, min_y, max_x, max_y);
    }
    for (const Aabb & b : removed_) {expand(b, min_x, min_y, max_x, max_y);}
    dirty_ = false;
    removed_.clear();
  }
}

void HazardLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {return;}
  std::lock_guard<std::mutex> lock(mtx_);
  if (!have_msg_ || frame_ != layered_costmap_->getGlobalFrameID()) {return;}
  const double res = master_grid.getResolution();
  const double ox = master_grid.getOriginX(), oy = master_grid.getOriginY();
  const int nx = static_cast<int>(master_grid.getSizeInCellsX());
  const int ny = static_cast<int>(master_grid.getSizeInCellsY());
  // Stateless: whatever falls inside the cycle's rectangle is redrawn, because
  // the master grid was reset inside it.
  for (const Shape & s : shapes_) {
    rasterShape(
      s, res, ox, oy, nx, ny, min_i, min_j, max_i, max_j, [&](int i, int j) {
        master_grid.setCost(
          static_cast<unsigned int>(i), static_cast<unsigned int>(j),
          nav2_costmap_2d::LETHAL_OBSTACLE);
      });
  }
}

void HazardLayer::reset()
{
  // clear_entirely resets EVERY layer. Keep the set and redraw it on the next
  // cycle: the planner must not lose a known buoy because someone cleared the
  // LiDAR's marks.
  std::lock_guard<std::mutex> lock(mtx_);
  dirty_ = true;
}

}  // namespace crusader_nav_layers

PLUGINLIB_EXPORT_CLASS(crusader_nav_layers::HazardLayer, nav2_costmap_2d::Layer)
