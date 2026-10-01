// planner_port.hpp — the seam between the planned leg and whoever plans.
//
// Pure: no ROS, no Nav2. PlannedLeg (planned_leg.hpp) talks to a PlannerPort and
// never to rclcpp, so the whole state machine runs under g++ on a laptop with a
// scripted fake, and the ROS implementation (ros_planner_port.cpp, WP2, compiled
// only when nav2_msgs exists) is a thin adapter over Nav2's actions and services.
//
// Spec: docs/nav2_avoidance_spec.md section 5.3 (exact).
//
// THE CONTRACT, which the ROS side must keep:
//   * NON-BLOCKING. Every call returns at once; the tick thread never waits.
//   * Implementations lock their OWN mutex and never the Context's `mu`: the leg
//     is called with `mu` NOT held, and a port that took it would deadlock a
//     callback against the tick.
//   * A new request SUPERSEDES any in flight. Its reply is dropped by seq - the
//     leg compares seq, never "the latest reply", because a late answer to the
//     request before last would otherwise be adopted as the answer to this one.
//   * seq 0 is "nothing". requestPlan/requestCheck return 0 when nothing was sent.
#ifndef CRUSADER_BT__PLANNER_PORT_HPP_
#define CRUSADER_BT__PLANNER_PORT_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "crusader_bt/path_math.hpp"

namespace crusader_bt
{
namespace path
{

/// None = no reply yet at all (the initial state); Pending = sent, not answered;
/// Unavailable = the server or service was not there (a BLIND planner, which the
/// leg treats worse than a planner that merely found no path).
enum class Reply {None, Pending, Ok, Failed, Unavailable};

struct PlanReply
{
  std::uint64_t seq = 0;
  Reply status = Reply::None;
  std::vector<Vec2> path;
  double planning_s = -1.0;     ///< -1 = unknown; a blank, not a zero
  std::string why;
};

struct CheckReply
{
  std::uint64_t seq = 0;
  Reply status = Reply::None;
  bool valid = false;
};

/// Non-blocking. Called from the tick thread; implementations lock their OWN mutex and
/// never ctx->mu. A new request supersedes any in flight (its reply is dropped by seq).
class PlannerPort
{
public:
  virtual ~PlannerPort() = default;
  virtual bool ready() = 0;                                          // server + service up
  virtual std::uint64_t requestPlan(Vec2 start, Vec2 goal) = 0;      // 0 = not sent
  virtual PlanReply lastPlan() = 0;
  virtual std::uint64_t requestCheck(const std::vector<Vec2> & path) = 0;
  virtual CheckReply lastCheck() = 0;
  virtual void clearCostmap() = 0;                                   // fire and forget
};

/// offros and tests: instant straight [start, goal] path; checks always valid.
///
/// Not a planner: the leg's own "plan crosses a known hazard" rejection is what
/// makes it refuse a line through the dock. Its job is to drive the REAL
/// PlannedLeg state machine and the real leaves with no Nav2 around.
class StraightPlannerPort : public PlannerPort
{
public:
  bool ready() override {return true;}

  std::uint64_t requestPlan(Vec2 start, Vec2 goal) override
  {
    plan_.seq = ++plan_seq_;
    plan_.status = Reply::Ok;
    plan_.path = {start, goal};
    plan_.planning_s = 0.0;
    plan_.why.clear();
    return plan_.seq;
  }
  PlanReply lastPlan() override {return plan_;}

  std::uint64_t requestCheck(const std::vector<Vec2> &) override
  {
    check_.seq = ++check_seq_;
    check_.status = Reply::Ok;
    check_.valid = true;
    return check_.seq;
  }
  CheckReply lastCheck() override {return check_;}

  void clearCostmap() override {}

private:
  PlanReply plan_;
  CheckReply check_;
  std::uint64_t plan_seq_ = 0, check_seq_ = 0;
};

}  // namespace path
}  // namespace crusader_bt

#endif  // CRUSADER_BT__PLANNER_PORT_HPP_
