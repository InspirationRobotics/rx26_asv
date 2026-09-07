// context.hpp — the one shared object every leaf reads, and the leaf base class.
//
// WHY A CONTEXT AND NOT BLACKBOARD PORTS FOR EVERYTHING.
//
// BehaviorTree.CPP can carry any type through a port, but every non-trivial one
// needs a convertFromString specialisation before it can appear in the XML, and
// a missing one is a RUNTIME error at tree-load — on the boat, at the dock. So
// ports here carry only strings and numbers, which the library handles natively,
// and world state (pose, buoys, entry/exit, the local origin) travels in this
// struct instead. It is the blackboard in the sense that matters: one object,
// written by the runner, read by every leaf.
//
// THE LEAVES NEVER SUBSCRIBE. The runner owns every subscription and refreshes
// this struct once per tick. A leaf that opened its own subscription could not
// be reasoned about off-boat, would double the DDS traffic for a topic already
// being read, and would see a different instant of the world than its siblings.
//
// LOCKING. The runner writes under `mu` from ROS callbacks; leaves read under
// it from the tick thread. Every access goes through a lock_guard — the tick
// runs on the executor thread and the callbacks may not.
#ifndef CRUSADER_BT__CONTEXT_HPP_
#define CRUSADER_BT__CONTEXT_HPP_

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

#include "crusader_bt/nav_math.hpp"

namespace crusader_bt
{

/// World state, refreshed by the runner once per tick. Read-only to leaves
/// except for `waypoint` and `consumed`, which NextWaypoint writes.
struct Context
{
  std::mutex mu;
  rclcpp::Node * node = nullptr;              ///< borrowed; the runner owns it

  // ---- fixed for the whole mission ----
  nav::LatLon origin;                         ///< local frame origin, set ONCE
  bool origin_set = false;
  double approach_lat = 0.0;
  double approach_lon = 0.0;
  bool has_approach = false;

  // ---- refreshed every tick ----
  bool autonomous = false;                    ///< mode in shared.autonomous_modes
  bool pose_fresh = false;
  nav::Vec2 boat;
  double heading_deg = std::nan("");          ///< NaN when GPS yaw is unresolved
  std::vector<nav::Buoy> buoys;
  bool have_entry = false;
  bool have_exit = false;
  nav::Vec2 entry;
  nav::Vec2 exitp;                            ///< `exit` is a libc function

  /// Where the boat was when the goal was accepted. "Go home" in a mission
  /// means "back to where this attempt started", not the autopilot's HOME —
  /// those differ whenever the boat was driven out manually first.
  nav::Vec2 home;
  bool have_home = false;

  // ---- written by NextWaypoint, read by NavigateTo ----
  nav::Vec2 waypoint;
  bool have_waypoint = false;
  int waypoint_buoy_id = -1;

  // ---- posture ----
  bool publish_setpoints = false;             ///< false = this tree cannot move the boat

  // ---- outputs the runner wires up ----
  std::function<void(nav::LatLon)> send_setpoint;
  std::function<void(const std::string &)> set_task;
  std::function<void()> publish_report;
  std::function<void(int)> consume_buoy;      ///< mark a buoy dealt with
};

using ContextPtr = std::shared_ptr<Context>;

/// A place to go, passed BETWEEN LEAVES on the blackboard.
///
/// This is the channel perception uses to tell an action where to drive. A
/// compute leaf writes one to an output port, an action leaf reads it from an
/// input port, and the XML shows the wire:
///
///     <NextWaypoint buoys="{buoys}" out="{goal}"/>
///     <NavigateTo   goal="{goal}"/>
///
/// NO convertFromString SPECIALISATION IS NEEDED, and that is worth knowing
/// because the fear of needing one is why the first version of this package
/// smuggled every position through the Context struct instead. A specialisation
/// is only required to parse a LITERAL out of the XML (goal="1.28;103.85"). A
/// value that only ever travels between leaves as {ref} is moved as the real
/// type and never touches a string.
///
/// `why` is carried so a log line, a feedback message or a viewer can say WHICH
/// buoy this is and why we are steering there — a bare lat/lon in a log tells
/// you where the boat went and nothing about what it thought it was doing.
struct Waypoint
{
  double lat = 0.0;
  double lon = 0.0;
  std::string why;
};

/// Fetch the context from the blackboard, or throw with a message that names
/// the cause. A leaf constructed without it is a wiring bug in the runner, and
/// failing loudly at tree-load beats a null dereference on the water.
inline ContextPtr contextFrom(const BT::NodeConfig & cfg, const std::string & who)
{
  ContextPtr ctx;
  if (!cfg.blackboard || !cfg.blackboard->get("ctx", ctx) || !ctx) {
    throw BT::RuntimeError(
      who + ": no 'ctx' on the blackboard. "
      "If this node sits inside a <SubTree>, that is the cause: a subtree gets "
      "its OWN blackboard and does not inherit the parent's entries. Add "
      "_autoremap=\"true\" to the SubTree tag, e.g. "
      "<SubTree ID=\"Whatever\" _autoremap=\"true\"/>. "
      "Verified against BT.CPP 4.9 on 2026-09-07: without it every leaf in the "
      "subtree throws this at tree-load; with it the subtree runs. "
      "Otherwise the runner failed to set it before creating the tree.");
  }
  return ctx;
}

/// Base for leaves that take more than one tick (publish, then poll).
class CrusaderAction : public BT::StatefulActionNode
{
public:
  CrusaderAction(const std::string & name, const BT::NodeConfig & cfg)
  : BT::StatefulActionNode(name, cfg), ctx_(contextFrom(cfg, name)) {}

  /// Default: nothing to undo. Leaves that latch state override this.
  void onHalted() override {}

protected:
  ContextPtr ctx_;

  rclcpp::Logger log() const
  {
    return ctx_->node ? ctx_->node->get_logger() : rclcpp::get_logger("crusader_bt");
  }
};

/// Base for a leaf that DOES something but finishes within one tick.
///
/// The distinction from CrusaderCondition is not decoration. BT.CPP's
/// SyncActionNode::executeTick THROWS if the leaf returns RUNNING, which is the
/// right guard for a leaf that is supposed to be instantaneous, and the two
/// report different NodeType — so Groot, and tools/bt_view.py, draw a thing that
/// publishes a topic differently from a thing that answers a question.
///
/// If it publishes, writes a port, or changes anything: this one.
/// If it only reads state and answers yes/no: CrusaderCondition.
class CrusaderSyncAction : public BT::SyncActionNode
{
public:
  CrusaderSyncAction(const std::string & name, const BT::NodeConfig & cfg)
  : BT::SyncActionNode(name, cfg), ctx_(contextFrom(cfg, name)) {}

protected:
  ContextPtr ctx_;

  rclcpp::Logger log() const
  {
    return ctx_->node ? ctx_->node->get_logger() : rclcpp::get_logger("crusader_bt");
  }
};

/// Base for leaves that answer in one tick and have NO side effects.
class CrusaderCondition : public BT::ConditionNode
{
public:
  CrusaderCondition(const std::string & name, const BT::NodeConfig & cfg)
  : BT::ConditionNode(name, cfg), ctx_(contextFrom(cfg, name)) {}

protected:
  ContextPtr ctx_;

  rclcpp::Logger log() const
  {
    return ctx_->node ? ctx_->node->get_logger() : rclcpp::get_logger("crusader_bt");
  }
};

/// Register every Crusader leaf with `factory`.
///
/// Registered DIRECTLY rather than loaded as plugins with registerFromPlugin().
/// Plugins buy hot-swapping a behaviour without relinking, which we will never
/// do mid-competition, and cost a shared-library search path that is wrong in
/// exactly one environment — the container — and fails at tree-load with a
/// message about a missing .so rather than about the tree.
void registerCrusaderNodes(BT::BehaviorTreeFactory & factory);

}  // namespace crusader_bt

#endif  // CRUSADER_BT__CONTEXT_HPP_
