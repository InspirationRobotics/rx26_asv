// ros_planner_port.cpp — path::PlannerPort over Nav2's planner_server.
//
// WHAT THE LEG SEES. A non-blocking port (planner_port.hpp): requestPlan() sends a
// ComputePathToPose goal and returns a sequence number at once; lastPlan() later
// answers for THAT number or for a newer one, never for an older one. A new request
// cancels the one in flight, and whatever the old one eventually says is dropped by
// seq here as well as in the leg, so a late reply can never be adopted as the answer
// to a later question. The leg runs on the tick thread; every callback in this file
// runs on an executor thread in this port's own callback group; the two meet only
// through `mu_`, which is this port's own and is never the Context's.
//
// WHAT NAV2 DOES NOT TELL US. Humble's ComputePathToPose has no error code: a path
// that cannot be planned is just ABORTED, whatever the reason (no path, a start in
// lethal space, a costmap that is not current), and a costmap that is not current
// may not answer at all. The leg times a request out at plan_timeout_s itself.
//
// Spec: docs/nav2_avoidance_spec.md section 5.4.
//
// Without Nav2 (CRSD_HAVE_NAV2 undefined) this file is a stub that returns nullptr.
#include "crusader_bt/ros_planner_port.hpp"

#ifdef CRSD_HAVE_NAV2

#include <cmath>
#include <cstdint>
#include <mutex>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace crusader_bt
{
namespace
{

using nav::Vec2;
using path::Reply;
using ComputePath = nav2_msgs::action::ComputePathToPose;
using PlanHandle = rclcpp_action::ClientGoalHandle<ComputePath>;
using IsValid = nav2_msgs::srv::IsPathValid;
using ClearAll = nav2_msgs::srv::ClearEntireCostmap;

class RosPlannerPort : public path::PlannerPort
{
public:
  RosPlannerPort(
    rclcpp::Node * node, std::string map_frame, std::string planner_id,
    const std::string & action, const std::string & valid_srv, const std::string & clear_srv)
  : node_(node), frame_(std::move(map_frame)), planner_id_(std::move(planner_id))
  {
    // ONE group for all three clients: MutuallyExclusive, so their callbacks never
    // run at the same time as each other, and the node's MultiThreadedExecutor runs
    // them on its own threads rather than the tick thread (worker_).
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    plan_client_ = rclcpp_action::create_client<ComputePath>(node_, action, group_);
    valid_client_ = node_->create_client<IsValid>(
      valid_srv, rmw_qos_profile_services_default, group_);
    clear_client_ = node_->create_client<ClearAll>(
      clear_srv, rmw_qos_profile_services_default, group_);
  }

  bool ready() override
  {
    return plan_client_->action_server_is_ready() && valid_client_->service_is_ready();
  }

  std::uint64_t requestPlan(Vec2 start, Vec2 goal) override
  {
    if (!plan_client_->action_server_is_ready()) {return 0;}
    ComputePath::Goal g;
    g.goal = pose(goal);
    g.start = pose(start);
    g.use_start = true;
    g.planner_id = planner_id_;

    std::uint64_t seq;
    {
      std::lock_guard<std::mutex> lk(mu_);
      seq = ++plan_seq_;
      setPlan(seq, Reply::Pending);
      cancelActive();                       // supersede: the old goal's reply is dropped by seq
    }
    rclcpp_action::Client<ComputePath>::SendGoalOptions o;
    o.goal_response_callback = [this, seq](PlanHandle::SharedPtr h) {onGoalResponse(seq, h);};
    o.result_callback = [this, seq](const PlanHandle::WrappedResult & r) {onResult(seq, r);};
    plan_client_->async_send_goal(g, o);
    return seq;
  }

  path::PlanReply lastPlan() override
  {
    std::lock_guard<std::mutex> lk(mu_);
    return plan_;
  }

  std::uint64_t requestCheck(const std::vector<Vec2> & p) override
  {
    if (p.empty() || !valid_client_->service_is_ready()) {return 0;}
    auto req = std::make_shared<IsValid::Request>();
    req->path.header.frame_id = frame_;
    req->path.poses.reserve(p.size());
    for (const Vec2 & v : p) {req->path.poses.push_back(pose(v));}

    std::uint64_t seq;
    {
      std::lock_guard<std::mutex> lk(mu_);
      seq = ++check_seq_;
      check_ = path::CheckReply{};
      check_.seq = seq;
      check_.status = Reply::Pending;
    }
    valid_client_->async_send_request(
      req, [this, seq](rclcpp::Client<IsValid>::SharedFuture f) {
        path::CheckReply cr;
        cr.seq = seq;
        try {
          cr.valid = f.get()->is_valid;
          cr.status = Reply::Ok;
        } catch (const std::exception &) {
          cr.status = Reply::Failed;
        }
        std::lock_guard<std::mutex> lk(mu_);
        if (seq == check_seq_) {check_ = cr;}      // a stale reply is dropped
      });
    return seq;
  }

  path::CheckReply lastCheck() override
  {
    std::lock_guard<std::mutex> lk(mu_);
    return check_;
  }

  void clearCostmap() override
  {
    if (!clear_client_->service_is_ready()) {
      RCLCPP_WARN(
        node_->get_logger(), "costmap clear: %s is not available", clear_client_->get_service_name());
      return;
    }
    clear_client_->async_send_request(
      std::make_shared<ClearAll::Request>(), [this](rclcpp::Client<ClearAll>::SharedFuture) {
        RCLCPP_INFO(node_->get_logger(), "global costmap cleared (planner_server answered)");
      });
  }

private:
  geometry_msgs::msg::PoseStamped pose(Vec2 p) const
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame_;                  // stamp 0 = "the latest transform"
    ps.pose.position.x = p.x;                     // map IS the BT's local frame (spec 2)
    ps.pose.position.y = p.y;
    ps.pose.orientation.w = 1.0;                  // SmacPlanner2D ignores orientation
    return ps;
  }

  /// CALL UNDER mu_. Cancel the goal in flight, if its handle has arrived.
  void cancelActive()
  {
    if (active_) {
      plan_client_->async_cancel_goal(active_);
      active_.reset();
    }
  }

  /// CALL UNDER mu_. A blank reply for `seq` in state `st`: nothing else of an older
  /// reply survives into it.
  void setPlan(std::uint64_t seq, Reply st, const std::string & why = "")
  {
    plan_ = path::PlanReply{};
    plan_.seq = seq;
    plan_.status = st;
    plan_.why = why;
  }

  void failed(std::uint64_t seq, const std::string & why) {setPlan(seq, Reply::Failed, why);}

  void onGoalResponse(std::uint64_t seq, const PlanHandle::SharedPtr & h)
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (!h) {
      if (seq == plan_seq_) {failed(seq, "planner_server rejected the goal");}
      return;
    }
    if (seq != plan_seq_) {                 // superseded before it was accepted: stop it now
      plan_client_->async_cancel_goal(h);
      return;
    }
    active_ = h;
  }

  void onResult(std::uint64_t seq, const PlanHandle::WrappedResult & r)
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (seq != plan_seq_) {return;}         // a stale reply: ignored
    active_.reset();
    switch (r.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        succeeded(seq, *r.result);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        failed(seq, "planner aborted (no path, start in lethal space, or costmap)");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        failed(seq, "planner goal canceled");
        break;
      default:
        failed(seq, "planner returned an unknown result");
    }
  }

  /// CALL UNDER mu_.
  void succeeded(std::uint64_t seq, const ComputePath::Result & res)
  {
    const auto & poses = res.path.poses;
    if (!res.path.header.frame_id.empty() && res.path.header.frame_id != frame_) {
      failed(seq, "planner returned a path in frame '" + res.path.header.frame_id +
        "', not '" + frame_ + "'");
      return;
    }
    if (poses.size() < 2) {
      failed(seq, "planner returned a path of " + std::to_string(poses.size()) + " pose(s)");
      return;
    }
    setPlan(seq, Reply::Ok);
    plan_.path.reserve(poses.size());
    for (const auto & ps : poses) {plan_.path.push_back({ps.pose.position.x, ps.pose.position.y});}
    plan_.planning_s = res.planning_time.sec + res.planning_time.nanosec * 1e-9;
  }

  rclcpp::Node * node_;
  std::string frame_, planner_id_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp_action::Client<ComputePath>::SharedPtr plan_client_;
  rclcpp::Client<IsValid>::SharedPtr valid_client_;
  rclcpp::Client<ClearAll>::SharedPtr clear_client_;

  std::mutex mu_;                           // this port's own; never the Context's
  std::uint64_t plan_seq_ = 0, check_seq_ = 0;
  path::PlanReply plan_;
  path::CheckReply check_;
  PlanHandle::SharedPtr active_;            // the goal in flight, once accepted
};

}  // namespace

std::shared_ptr<path::PlannerPort> makeRosPlannerPort(
  rclcpp::Node * node, const std::string & map_frame, const std::string & planner_id,
  const std::string & action, const std::string & valid_srv, const std::string & clear_srv)
{
  return std::make_shared<RosPlannerPort>(
    node, map_frame, planner_id, action, valid_srv, clear_srv);
}

}  // namespace crusader_bt

#else  // no Nav2 in this image

namespace crusader_bt
{

std::shared_ptr<path::PlannerPort> makeRosPlannerPort(
  rclcpp::Node *, const std::string &, const std::string &, const std::string &,
  const std::string &, const std::string &)
{
  return nullptr;
}

}  // namespace crusader_bt

#endif  // CRSD_HAVE_NAV2
