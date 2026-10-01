// ros_planner_port.hpp — the ROS side of path::PlannerPort: Nav2's planner_server.
//
// Only a factory is declared, and it is declared ALWAYS, so bt_runner_node.cpp has
// one code path whether or not the image has Nav2. src/ros_planner_port.cpp holds
// the implementation behind CRSD_HAVE_NAV2 (set by CMakeLists.txt when nav2_msgs is
// found) and a stub that returns nullptr otherwise: the runner then refuses to start
// with nav_mode other than "off". The off-ROS runner never includes this file.
//
// Spec: docs/nav2_avoidance_spec.md section 5.4.
#ifndef CRUSADER_BT__ROS_PLANNER_PORT_HPP_
#define CRUSADER_BT__ROS_PLANNER_PORT_HPP_

#include <memory>
#include <string>

#include "crusader_bt/planner_port.hpp"

namespace rclcpp
{
class Node;
}

namespace crusader_bt
{

/// The planner port over `planner_server`'s ComputePathToPose action, IsPathValid
/// service and the costmap's ClearEntireCostmap service, all in `map_frame`.
///
/// nullptr when this build has no Nav2 (nav2_msgs absent). Everything the port
/// creates hangs off `node`, which must outlive it; the clients live in their own
/// MutuallyExclusive callback group, so their callbacks run on the executor's
/// threads and never touch the tick thread.
std::shared_ptr<path::PlannerPort> makeRosPlannerPort(
  rclcpp::Node * node, const std::string & map_frame, const std::string & planner_id,
  const std::string & action, const std::string & valid_srv, const std::string & clear_srv);

}  // namespace crusader_bt

#endif  // CRUSADER_BT__ROS_PLANNER_PORT_HPP_
