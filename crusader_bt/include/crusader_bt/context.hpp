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
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

#include "crusader_bt/dock_math.hpp"
#include "crusader_bt/nav_math.hpp"

namespace crusader_bt
{

/// World state, refreshed by the runner once per tick. Read-only to leaves
/// except for `waypoint` and `consumed`, which NextWaypoint writes, the gate
/// bookkeeping the Disruptive Task 1 leaves keep, and the Task 3 commitments
/// (chosen bay, berth, request) below `dock`.
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
  std::vector<nav::Buoy> buoys;               ///< FUSED: plan + tracker
  bool have_entry = false;
  bool have_exit = false;
  nav::Vec2 entry;
  nav::Vec2 exitp;                            ///< `exit` is a libc function

  // ---- the UAV's passage plan, above Core tier ----
  //
  // `buoys` above is the FUSION of this and the tracker's targets, and the
  // beacon state in it always came from here. See nav::fusePassage.
  //
  // entry/exitp come from the plan too, not from findBeacon over the boat's own
  // targets: the EXIT sits 92 m out in a full-size field, far past the camera,
  // and a transit that waits to SEE it never starts.
  std::vector<nav::PlanBuoy> plan;
  std::uint32_t plan_version = 0;             ///< bumps on every new plan
  bool plan_fresh = false;                    ///< aged like pose_fresh
  double plan_age_s = 0.0;                    ///< seconds since the last plan
  std::vector<nav::Buoy> obstacles;           ///< tracked, but not in the plan

  // ---- the passage the BOAT planned ----
  //
  // The aircraft sends ten buoys and their colours. Which pair is a gate, and
  // what order to drive them in, is worked out here by nav::planPassage over
  // `buoys` -- the FUSED field, so the aircraft's colours and the tracker's
  // positions. Re-planned every tick, because a confirmation can bring new
  // colours and the answer must follow them.
  nav::Passage passage;

  /// Gates already driven, as (red_id, green_id).
  ///
  /// BY BUOY IDS, NOT BY COUNT. A confirmation can recolour the field, the plan
  /// re-runs, and the order can change underneath the boat; "I have done two,
  /// start at index two" then skips a gate that was never driven. Ids survive
  /// a reorder and survive the pair itself swapping colours.
  std::vector<std::pair<int, int>> cleared_gates;

  // ---- the confirmation handshake ----
  //
  // Having crossed a gate, the boat publishes gate_reached(gate_seq) and waits
  // for an acknowledgement carrying the SAME seq before driving the next one.
  // The question it is asking is "is the next pair still where you said it
  // was" -- the aircraft answers by acking and retransmitting the whole field.
  //
  // Matching on the sequence is what makes a retransmission on a lossy radio
  // distinguishable from the answer to the next request.
  std::uint8_t gate_seq = 0;
  int gate_red_id = -1;                       ///< the gate being driven now
  int gate_green_id = -1;
  bool have_confirmation = false;             ///< an ack for gate_seq arrived
  /// Every gate in `passage` is in `cleared_gates`: go to the exit.
  ///
  /// THE BOAT DECIDES THIS NOW, from its own plan -- it holds the whole field,
  /// so it can count. The aircraft used to end the transit with a 255/255
  /// answer, which it can no longer do because it no longer assigns gates.
  ///
  /// Proximity to the exit is still not a substitute, for the reason it never
  /// was: the boat must clear every gate first, and the exit buoy can sit well
  /// inside any sane arrival radius of the last one.
  bool passage_complete = false;
  /// Did the boat finish the gate it last asked about?
  ///
  /// RequestConfirmation advances the sequence ONLY when this is set, so a leg
  /// that failed re-asks under the SAME seq and the aircraft's idempotent
  /// answer gives the same reply. Without it every failure inside the leg --
  /// a confirmation timeout, an unusable pair, a PlanChanged re-task -- burns
  /// a sequence number, and back when the sequence number chose the gate that
  /// also SKIPPED one. SITL 2026-09-13.
  ///
  /// It is no longer what stops a gate being skipped -- cleared_gates is, and
  /// it is keyed on the buoys rather than on a counter -- but it still keeps
  /// the sequence numbering honest.
  bool gate_cleared = false;

  // ---- Task 3: the docking bays ----
  //
  // `dock` is written by the runner on every DockObservation, through
  // ingestDockObservation() below - the ONE function both runners call, so
  // the ROS node and the off-ROS sim runner cannot disagree about what a frame
  // means. Everything after it is written by the Task 3 leaves.
  int tier = 0;                               ///< goal tier: 0 Core, 1 Adv, 2 Disr
  dock::DockBook dock;                        ///< the bays, world-anchored
  dock::Mount cam_mount;                      ///< camera_link in base_link
  double dock_obs_age_s = 1e9;                ///< since the last DockObservation
  std::uint32_t dock_seq = 0;                 ///< bumps on every DockObservation
  double dock_t = 0.0;                        ///< that frame's stamp, seconds
  // The timing layer's verdict on the latest frame, for the bay it tracks.
  std::string dock_pattern;                   ///< "", steady, flash, code, ...
  std::vector<dock::Colour> dock_colours;
  int dock_target_window = -1;                ///< DockWindow.index, -1 = none
  double dock_fps = 0.0;                      ///< the timing layer's observed rate
  /// "hit" events seen so far. COUNTED, because DockObservation.last_event is
  /// set on one frame only and a leaf ticking at 10 Hz can miss that frame.
  int dock_hits = 0;

  /// How much indicator evidence makes a verdict, and how many sightings make
  /// a bay. HERE rather than on each leaf's ports because SafeBayKnown and
  /// CommitSafeBay must apply the SAME rule - a condition that passes on
  /// looser numbers than the commit that follows it is a tree that stalls.
  dock::VoteParams dock_votes;
  int dock_min_obs = 5;

  std::string task3_phase;                    ///< for feedback and the viewers
  int survey_attempt = 0;                     ///< PickVantage calls, all of them
  int survey_looks = 0;                       ///< ... of which placed from seen bays
  int chosen_track = -1;                      ///< committed by CommitSafeBay
  int chosen_bay = 0;                         ///< its 1-based number
  dock::Berth berth;                          ///< for the chosen bay
  int fired_window = -1;                      ///< DockWindow.index we put out
  /// RoboCommand's ReadinessConfirm for the docking report, from
  /// /crsd/ocs_command. The light is what the leaves wait on; this only stops
  /// the report being re-sent.
  bool readiness_confirmed = false;
  dock::Request request;                      ///< decoded, held, not yet sent
  bool have_request = false;

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
  /// Tell the aircraft a gate is cleared and ask for the next pair. Publishes
  /// std_msgs/UInt8 on crsd/gate_reached; rxl_link_node puts it on the air.
  std::function<void(std::uint8_t)> report_gate_reached;

  // ---- Task 3 outputs ----
  //
  // The three RoboCommand reports go out as JSON for the OCS to relay, like
  // the Task 1 report. Field names are rx_reports.proto's.
  std::function<void(int)> report_docking;          ///< DockingReport.bay_id
  std::function<void(int)> report_firefighting;     ///< FirefightingReport.window_id
  /// ResourceDeliveryRequest to RoboCommand.
  std::function<void(const dock::Request &)> report_request;
  /// The same request to the UAV, over the radio.
  std::function<void(const dock::Request &)> relay_request;
  /// The water cannon: fire or not, aimed at a point in camera_link.
  std::function<void(bool, double, double, double)> cannon;
};

using ContextPtr = std::shared_ptr<Context>;

/// Fold one DockObservation into the context. CALL UNDER ctx.mu.
///
/// Both runners convert their input (a crusader_msgs/DockObservation, or a
/// JSON line from the sim) into a dock::Frame and hand it here; nothing about
/// what a frame MEANS is decided in either runner.
///
/// The bays are placed only with a fresh pose and a resolved heading - a
/// sighting placed from a stale pose is a bay in the wrong place, forever. The
/// timing layer's verdict does not depend on the pose, so it is always taken.
inline void ingestDockObservation(Context & c, const dock::Frame & f)
{
  if (c.origin_set && c.pose_fresh && std::isfinite(c.heading_deg)) {
    c.dock.ingest(f, c.boat, c.heading_deg, c.cam_mount);
  }
  ++c.dock_seq;
  c.dock_t = f.t;
  c.dock_pattern = f.target_pattern;
  c.dock_colours = f.target_colours;
  c.dock_target_window = f.target_window_index;
  c.dock_fps = f.observed_fps;
  if (f.last_event == "hit") {++c.dock_hits;}
}

/// Forget everything Task 3 learned. Called at goal start: a second attempt
/// must not inherit the first one's bays, votes or commitments.
inline void resetTask3(Context & c)
{
  c.dock = dock::DockBook{};
  c.dock_hits = 0;
  c.task3_phase.clear();
  c.fired_window = -1;
  c.survey_attempt = 0;
  c.survey_looks = 0;
  c.chosen_track = -1;
  c.chosen_bay = 0;
  c.berth = dock::Berth{};
  c.readiness_confirmed = false;
  c.request = dock::Request{};
  c.have_request = false;
}

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

/// The Task 3 leaves, from src/task3_leaves.cpp. Called by
/// registerCrusaderNodes, so a runner registers everything with one call.
void registerTask3Nodes(BT::BehaviorTreeFactory & factory);

}  // namespace crusader_bt

#endif  // CRUSADER_BT__CONTEXT_HPP_
