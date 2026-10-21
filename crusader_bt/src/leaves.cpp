// leaves.cpp — the Task 1 Core leaves and the shared ones; registers every leaf.
//
// No leaf here should be interesting: geometry that can be silently wrong lives in
// nav_math.hpp, tested on a laptop (test/test_nav_math.cpp). What is left is: read a
// port, call nav_math, publish, poll. A leaf that grows a formula: move it there and test it.
//
// LOAD-BEARING, in the reactive guard band re-ticked at 10 Hz:
//   ResolveBuoyStates and ReportSafePassage MUST NEVER RETURN FAILURE: it reaches
//   the root and ends the mission, so one dropped camera frame would abort the run.
//   "Saw nothing this tick" is SUCCESS; EntryResolved gates on actually knowing.
//   NextWaypoint's FAILURE is not an error: it ends the transit loop.
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

#include "crusader_bt/context.hpp"
#include "crusader_bt/nav_math.hpp"
#include "crusader_bt/path_math.hpp"
#include "crusader_bt/planned_leg.hpp"

// PORTS: BT::InputPort has (name, description) and (name, default, description), NO
// (name, default). That binds to the first and fails with a template error that buries
// the cause, so a port with a default MUST also carry a description.

namespace crusader_bt
{
namespace
{

using nav::Vec2;

// ------------------------------------------------- setpoints and planned legs
//
// NavigateTo and CircleBuoy share one PlannedLeg driver: the one place a leg's output
// becomes a setpoint, a log line and a status for the map. sendSetpoint() is in
// context.hpp so the whole-field leaves (global_leaves.cpp) send the same way.

/// The world a PlannedLeg needs this tick, read under ONE lock (hazards: the same
/// knownHazards() the costmap is drawn from).
path::LegInputs legInputs(Context & c, bool goal_ok, Vec2 goal)
{
  std::lock_guard<std::mutex> lk(c.mu);
  path::LegInputs in;
  in.now_s = c.now_s;
  in.pose_fresh = c.pose_fresh;
  in.boat = c.boat;
  in.heading_deg = c.heading_deg;
  in.goal_ok = goal_ok;
  in.goal = goal;
  in.datum_ok = !c.datum_mismatch;
  in.hazards = knownHazards(c);
  return in;
}

/// The LegConfig fields NavigateTo and CircleBuoy share, from their same-named ports.
path::LegConfig legConfigFromPorts(const BT::TreeNode & node)
{
  path::LegConfig cfg;
  cfg.avoid = node.getInput<bool>("avoid").value_or(true);
  cfg.tolerance = node.getInput<double>("tolerance").value_or(2.0);
  cfg.blocked_timeout_s = node.getInput<double>("blocked_timeout_s").value_or(15.0);
  return cfg;
}

/// One PlannedLeg and its output's setpoint, log lines and map status. NavigateTo
/// keeps one for its whole life; CircleBuoy restarts it per hop.
class LegDriver
{
public:
  LegDriver(Context & c, std::string leaf, std::string name)
  : c_(c), leaf_(std::move(leaf)), name_(std::move(name)) {}

  /// A new leg towards `goal`. `label` prefixes the setpoint log lines ("NavigateTo
  /// port", "  orbit 3/9"). hop/hops are for CircleBuoy's status (-1 = not an orbit).
  BT::NodeStatus start(
    const path::LegConfig & cfg, Vec2 goal, const std::string & label,
    const rclcpp::Logger & lg, int hop = -1, int hops = 0)
  {
    cfg_ = cfg;
    goal_ = goal;
    hop_ = hop;
    hops_ = hops;
    sent_ = false;
    leg_ = std::make_unique<path::PlannedLeg>(c_.nav, c_.nav_mode, c_.planner.get());
    return apply(leg_->start(cfg, legInputs(c_, true, goal)), label, lg);
  }

  /// One tick. `goal_ok` false = the goal could not be re-resolved: the leg keeps the last.
  BT::NodeStatus step(bool goal_ok, Vec2 goal, const std::string & label, const rclcpp::Logger & lg)
  {
    if (!leg_) {return BT::NodeStatus::FAILURE;}
    if (goal_ok) {goal_ = goal;}
    return apply(leg_->step(legInputs(c_, goal_ok, goal_)), label, lg);
  }

  /// Forget the leg and tell the map there is none. Safe at any time.
  void halt()
  {
    leg_.reset();
    std::lock_guard<std::mutex> lk(c_.mu);
    clearLeg(c_);
  }

private:
  /// The suffix that says what this setpoint IS. Shadow sends the legacy goal and
  /// never holds, so it names neither a hold nor a carrot; Off holds (its legs are
  /// guarded straight ones) but has no carrot; On names both.
  std::string sendLabel(const path::LegOutput & out, const std::string & label)
  {
    const char * how = "";
    if (c_.nav_mode != path::Mode::Shadow &&
      (out.state == path::LegState::Blocked || out.state == path::LegState::Degraded))
    {
      how = " hold";
    } else if (c_.nav_mode == path::Mode::On && cfg_.avoid) {
      how = " carrot";
    } else if (sent_) {
      how = " moved";
    }
    sent_ = true;
    return label + how + " ->";
  }

  BT::NodeStatus apply(
    const path::LegOutput & out, const std::string & label, const rclcpp::Logger & lg)
  {
    if (out.send) {sendSetpoint(c_, lg, out.setpoint, sendLabel(out, label));}
    for (const std::string & line : out.log) {RCLCPP_INFO(lg, "nav: %s", line.c_str());}
    writeStatus(out);
    if (out.result == path::Result::Failure) {
      RCLCPP_WARN(lg, "nav: leg FAILED: %s", out.why.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return out.result == path::Result::Success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  void writeStatus(const path::LegOutput & out)
  {
    std::lock_guard<std::mutex> lk(c_.mu);
    Context::LegStatus & s = c_.leg;
    s.leaf = leaf_;
    s.name = name_;
    s.state = path::legStateName(out.state);
    s.why = out.why;
    s.avoid = cfg_.avoid;
    s.blocked_s = out.blocked_s;
    s.have_goal = true;
    s.goal = goal_;
    s.have_target = leg_->hasTarget();
    s.target = leg_->target();
    s.path = leg_->path();
    s.plan_ms = leg_->planMs();
    s.hop = hop_;
    s.hops = hops_;
    ++c_.leg_seq;
  }

  Context & c_;
  std::string leaf_, name_;
  std::unique_ptr<path::PlannedLeg> leg_;
  path::LegConfig cfg_;
  Vec2 goal_;
  int hop_ = -1, hops_ = 0;
  bool sent_ = false;          ///< a setpoint has gone out on this leg already
};

// ---------------------------------------------------------------- conditions

/// The autonomy latch: a CONDITION, first in the guard band, because a tree that ticks
/// while the pilot has the boat fights the pilot. FAILURE halts everything within a tick.
class IsAutonomous : public CrusaderCondition
{
public:
  IsAutonomous(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->autonomous ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class EntryResolved : public CrusaderCondition
{
public:
  EntryResolved(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->have_entry ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class ExitResolved : public CrusaderCondition
{
public:
  ExitResolved(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->have_exit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

/// Close enough to the exit buoy to stop transiting? `radius` MUST exceed CircleBuoy's,
/// or the transit ends inside the orbit and the circle starts from the wrong place
/// (the tree: 8 m against a 6 m orbit).
class NearExit : public CrusaderCondition
{
public:
  NearExit(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("radius", 8.0, "metres; keep > orbit radius")};
  }

  BT::NodeStatus tick() override
  {
    const double r = getInput<double>("radius").value_or(8.0);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->have_exit || !ctx_->pose_fresh) {
      return BT::NodeStatus::FAILURE;      // unknown is not "arrived"
    }
    return nav::norm(ctx_->exitp - ctx_->boat) <= r ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// -------------------------------------------------------- world-model update

/// Refreshes nothing: the runner's subscriptions fill the context. It exists so the
/// tree SHOWS where the world update happens, and gives a future ledger (flash
/// discrimination, colour-change tracking) a home already ticked at the right rate.
/// ALWAYS SUCCESS (file header).
class ResolveBuoyStates : public CrusaderSyncAction
{
public:
  ResolveBuoyStates(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (ctx_->buoys.size() != last_n_) {
      last_n_ = ctx_->buoys.size();
      RCLCPP_INFO(
        log(), "world: %zu buoys, entry %s, exit %s", last_n_,
        ctx_->have_entry ? "yes" : "--", ctx_->have_exit ? "yes" : "--");
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::size_t last_n_ = static_cast<std::size_t>(-1);
};

/// SafePassageReport to /crsd/robocommand/report, at most every min_period_s: the guard band
/// ticks at 10 Hz and the handbook wants an update when the mapped field changes. ALWAYS SUCCESS.
class ReportSafePassage : public CrusaderSyncAction
{
public:
  ReportSafePassage(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("min_period_s", 1.0, "floor between reports")};
  }

  BT::NodeStatus tick() override
  {
    const double period = getInput<double>("min_period_s").value_or(1.0);
    const auto now = std::chrono::steady_clock::now();
    if (last_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now - last_).count() < period)
    {
      return BT::NodeStatus::SUCCESS;
    }
    last_ = now;
    std::string body;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!ctx_->origin_set) {return BT::NodeStatus::SUCCESS;}
      body = safePassageReportJson(*ctx_);
    }
    sendReport(*ctx_, "safe_passage", body);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::chrono::steady_clock::time_point last_{};
};

// ------------------------------------------------------------------- actions

/// One task attempt as the heartbeat reports it (handbook 3.4.11): `token` on the first tick,
/// TASK_NONE when the child finishes or this is halted, so every attempt is NONE -> task -> NONE.
/// Returns the child's status.
///
/// TASK_NONE is held `none_dwell_s` first (RUNNING, child not ticked): back to back it would
/// last one 100 ms tick and the 2 Hz heartbeat would never carry it.
class TaskAttempt : public BT::DecoratorNode
{
public:
  TaskAttempt(const std::string & n, const BT::NodeConfig & c)
  : BT::DecoratorNode(n, c), ctx_(contextFrom(c, n)) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("token", "TASK_SAFE_PASSAGE", "an RxTask name"),
      BT::InputPort<double>("none_dwell_s", 1.5,
        "TASK_NONE is held this long before a new attempt is claimed, so the 2 Hz heartbeat "
        "carries it")};
  }

  BT::NodeStatus tick() override
  {
    setStatus(BT::NodeStatus::RUNNING);
    if (!active_) {
      double none_age_s;
      {
        std::lock_guard<std::mutex> lk(ctx_->mu);
        none_age_s = ctx_->now_s - ctx_->none_since_s;
      }
      const double dwell_s = getInput<double>("none_dwell_s").value_or(1.5);
      if (none_age_s < dwell_s) {return BT::NodeStatus::RUNNING;}
      active_ = true;
      announceTask(*ctx_, getInput<std::string>("token").value_or("TASK_SAFE_PASSAGE"));
    }
    const BT::NodeStatus st = child_node_->executeTick();
    if (st != BT::NodeStatus::RUNNING) {
      end();
      resetChild();
    }
    return st;
  }

  void halt() override
  {
    end();
    BT::DecoratorNode::halt();
  }

private:
  void end()
  {
    if (!active_) {return;}
    active_ = false;
    announceTask(*ctx_, "TASK_NONE");
  }

  ContextPtr ctx_;
  bool active_ = false;
};

/// Holds station until RoboCommand's RunStart has reached the boat (handbook 3.4.10: position-hold
/// after switching to autonomous, begin once the run has started). A RunStart that arrived up to
/// `max_age_s` before this node started still counts, so one that beat the START button is not
/// lost. FAILURE only after `timeout_s` > 0.
class WaitForRunStart : public CrusaderAction
{
public:
  WaitForRunStart(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_age_s", 30.0,
        "a run_start received up to this long before the node started still counts"),
      BT::InputPort<double>("timeout_s", 0.0, "0 = wait forever; >0 = FAILURE after this long"),
      BT::InputPort<bool>("hold", true, "hold station while waiting"),
      BT::OutputPort<int>("run_id", "RoboCommand's run id; -1 when the message carried none")};
  }

  BT::NodeStatus onStart() override
  {
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      since_s_ = ctx_->now_s;
    }
    held_ = false;
    const BT::NodeStatus st = poll();
    if (st == BT::NodeStatus::RUNNING) {
      RCLCPP_INFO(
        log(), "WaitForRunStart: waiting for RoboCommand's RunStart (max_age %.0f s, timeout %.0f s)",
        maxAgeS(), timeoutS());
    }
    return st;
  }

  BT::NodeStatus onRunning() override {return poll();}

  void onHalted() override {}

private:
  double maxAgeS() const {return getInput<double>("max_age_s").value_or(30.0);}
  double timeoutS() const {return getInput<double>("timeout_s").value_or(0.0);}

  BT::NodeStatus poll()
  {
    double now_s;
    runstart::Latch latch;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      now_s = ctx_->now_s;
      latch = ctx_->run_start;
    }
    if (latch.counts(since_s_, maxAgeS())) {
      (void)setOutput("run_id", latch.id);
      RCLCPP_INFO(
        log(), "WaitForRunStart: RunStart accepted (run_id %d, received %.1f s ago)", latch.id,
        std::max(0.0, now_s - latch.t_s));
      return BT::NodeStatus::SUCCESS;
    }
    if (timeoutS() > 0.0 && now_s - since_s_ >= timeoutS()) {
      RCLCPP_WARN(log(), "WaitForRunStart: no RunStart within %.0f s", timeoutS());
      return BT::NodeStatus::FAILURE;
    }
    // Once, like HoldStation; retried only until the first send, because the wait can begin
    // before the first pose.
    if (!held_ && getInput<bool>("hold").value_or(true)) {
      held_ = sendHold(*ctx_, log(), "WaitForRunStart hold at");
    }
    return BT::NodeStatus::RUNNING;
  }

  double since_s_ = 0.0;      ///< Context::now_s when the wait began
  bool held_ = false;
};

/// Reads a named target out of the context, drives there, and polls for arrival.
///
/// The driving is path::PlannedLeg, picked once at start from nav_mode and `avoid`
/// (docs/nav2_avoidance_spec.md 5.3):
///   * shadow, avoid=false: THE LEGACY LEG. Publish the goal, re-send it when it
///     moves, arrive on `tolerance`.
///   * off (any `avoid`), or on with avoid=false: that, plus a guard that holds while
///     the straight line to the goal crosses a known hazard, resumes once it has stayed
///     clear for nav_unblock_reset_s, FAILS after `blocked_timeout_s`. No planner, no
///     Nav2: off (the boat default) holds in front of a hazard, never drives through it
///     (spec 5.5). With avoid=false this is also the gate crossing and the Task 3
///     predock and berth, where the hazard IS the thing driven past.
///   * on, avoid=true (default): plan round every known hazard through Nav2, follow a
///     carrot 3-5 m ahead. Cannot plan = HOLD, FAILURE after `blocked_timeout_s`;
///     arrival is judged on the true goal.
///   * shadow, avoid=true: the planner runs and the map shows it, but the legacy leg
///     drives. Never holds, never fails.
///
/// `target` is a string rather than a typed port so the XML needs no custom
/// convertFromString (see context.hpp). "waypoint" is NextWaypoint's; "approach" is
/// the team-provided lat/lon from the action goal.
///
/// WITH publish_setpoints FALSE it still polls arrival without publishing: the read-only
/// water-test posture, where a human drives and the tree tracks whether the boat reached
/// where the mission wanted. Faked arrival would prove nothing.
class NavigateTo : public CrusaderAction
{
public:
  NavigateTo(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c), leg_(*ctx_, "NavigateTo", n) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target", "waypoint",
                                 "port | waypoint | approach | exit | home | fix"),
      // THE PERCEPTION WIRE: with target="port" the goal is a Waypoint an upstream
      // leaf wrote, so the XML shows where it came from:
      //     <SomeDetector out="{goal}"/>
      //     <NavigateTo target="port" goal="{goal}"/>
      BT::InputPort<Waypoint>("goal", "a Waypoint from an upstream leaf"),
      BT::InputPort<double>("lat", 0.0, "with target=fix: latitude"),
      BT::InputPort<double>("lon", 0.0, "with target=fix: longitude"),
      BT::InputPort<double>("tolerance", 2.0, "arrival radius, metres"),
      BT::InputPort<double>("resend_m", 1.5,
        "re-send the setpoint when the goal moves further than this"),
      BT::InputPort<bool>("avoid", true,
        "plan around known hazards (needs nav_mode shadow or on; in off every leg is a "
        "guarded straight one); false = a straight leg"),
      BT::InputPort<std::string>("exempt", "",
        "with avoid=false only: dock, a hazard this leg drives past"),
      BT::InputPort<double>("blocked_timeout_s", 15.0,
        "FAILURE after being blocked this long")};
  }

  BT::NodeStatus onStart() override
  {
    const std::string which = getInput<std::string>("target").value_or("waypoint");
    if (!resolve(which, goal_)) {
      RCLCPP_WARN(log(), "NavigateTo: target '%s' is not available", which.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return leg_.start(legConfig(), goal_, "NavigateTo " + which, log());
  }

  BT::NodeStatus onRunning() override
  {
    // THE GOAL IS ALLOWED TO MOVE: a tree rewrites {goal} as the mission learns more.
    // The leg re-sends only past resend_m: a setpoint republished at 10 Hz is a stream,
    // which ArduRover treats as a different control mode, and the deadband keeps this
    // leaf a leaf rather than a controller. resend_m 0 freezes the goal at its start.
    // resolve() takes ctx_->mu itself, so it is called outside the lock.
    const std::string which = getInput<std::string>("target").value_or("waypoint");
    const bool follow = getInput<double>("resend_m").value_or(1.5) > 0.0;
    Vec2 want;
    const bool have = follow && resolve(which, want);
    const BT::NodeStatus st = leg_.step(have, want, "NavigateTo " + which, log());
    if (st == BT::NodeStatus::SUCCESS) {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (ctx_->consume_buoy && ctx_->waypoint_buoy_id >= 0) {
        ctx_->consume_buoy(ctx_->waypoint_buoy_id);
      }
    }
    return st;
  }

  void onHalted() override {leg_.halt();}

private:
  bool resolve(const std::string & which, Vec2 & out)
  {
    if (which == "port") {
      const auto w = getInput<Waypoint>("goal");
      if (!w) {
        RCLCPP_WARN(
          log(), "NavigateTo target=port but no goal on the blackboard: %s",
          w.error().c_str());
        return false;
      }
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!ctx_->origin_set) {return false;}
      why_ = w.value().why;
      out = nav::toLocal({w.value().lat, w.value().lon}, ctx_->origin);
      return true;
    }
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (which == "waypoint") {
      if (!ctx_->have_waypoint) {return false;}
      out = ctx_->waypoint;
      return true;
    }
    if (which == "exit") {
      if (!ctx_->have_exit) {return false;}
      out = ctx_->exitp;
      return true;
    }
    if (which == "approach") {
      if (!ctx_->has_approach || !ctx_->origin_set) {return false;}
      out = nav::toLocal({ctx_->approach_lat, ctx_->approach_lon}, ctx_->origin);
      return true;
    }
    if (which == "home") {
      if (!ctx_->have_home) {return false;}
      out = ctx_->home;
      return true;
    }
    if (which == "fix") {
      // Literal XML coordinates: a tree driven with no perception at all (the
      // waypoint-tour demo exercises every phase transition without a buoy).
      if (!ctx_->origin_set) {return false;}
      const auto la = getInput<double>("lat");
      const auto lo = getInput<double>("lon");
      if (!la || !lo) {return false;}
      out = nav::toLocal({la.value(), lo.value()}, ctx_->origin);
      return true;
    }
    return false;
  }

  /// The leg's settings from the ports. `exempt` (gate | dock) only makes sense for a
  /// straight leg: a planned one treats every known object as a hazard.
  path::LegConfig legConfig()
  {
    path::LegConfig cfg = legConfigFromPorts(*this);
    cfg.resend_m = getInput<double>("resend_m").value_or(1.5);
    const std::string exempt = getInput<std::string>("exempt").value_or("");
    if (exempt.empty()) {return cfg;}
    if (cfg.avoid) {
      RCLCPP_WARN(
        log(), "NavigateTo: exempt='%s' is ignored with avoid=true (set avoid=false for a "
        "straight leg)", exempt.c_str());
      return cfg;
    }
    std::size_t at = 0;
    while (at <= exempt.size()) {
      std::size_t comma = exempt.find(',', at);
      if (comma == std::string::npos) {comma = exempt.size();}
      std::string tok = exempt.substr(at, comma - at);
      tok.erase(std::remove(tok.begin(), tok.end(), ' '), tok.end());
      at = comma + 1;
      if (tok == "dock") {
        cfg.exempt_dock = true;
      } else if (!tok.empty()) {
        RCLCPP_WARN(log(), "NavigateTo: unknown exempt '%s' (dock): ignored", tok.c_str());
      }
    }
    return cfg;
  }

  Vec2 goal_;
  std::string why_;
  LegDriver leg_;
};

/// Drives once around a buoy: one hop onto the ring, `points` waypoints round it, then
/// the overshoot. Core Tier: ENTRY clockwise before the transit, EXIT counterclockwise.
///
/// THE RING STARTS ON THE BOAT'S OWN BEARING (path::orbitRing): ring[0] is the hop onto
/// the circle and the sweep over ring[1..] is +-(360 + overshoot_deg) whatever the start
/// (measured from the boat's position instead, a far start read as a 313 degree circle).
/// Each hop is one PlannedLeg, so in every nav_mode it is at least a straight one (in off,
/// guarded) and the orbit needs no Nav2. A ring point within orbit_clear_m of a known
/// hazard is pushed outward (path::adjustRing) or dropped, and the log says how many.
///
/// THE OVERSHOOT. A hop arrives within `tolerance` (2 m, ~19 degrees on a 6 m ring), so a
/// ring stopping at exactly 360 closes ~19 degrees short, more if the approach enters off
/// ring[0] (the independent referee scored 328 where 330 is needed). So the ring runs
/// `overshoot_deg` past the start, the same way round, and is complete only at the LAST point.
///
/// A hop that cannot be driven FAILS the leaf after blocked_timeout_s; the tree's
/// RetryUntilSuccessful restarts the whole orbit.
class CircleBuoy : public CrusaderAction
{
public:
  CircleBuoy(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c), leg_(*ctx_, "CircleBuoy", n) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("anchor", "entry", "entry | exit | fix"),
      BT::InputPort<double>("lat", 0.0, "with anchor=fix: latitude"),
      BT::InputPort<double>("lon", 0.0, "with anchor=fix: longitude"),
      BT::InputPort<double>("radius", "orbit radius, metres (absent: nav_orbit_radius_m)"),
      BT::InputPort<int>("points", "waypoints around the circle (absent: nav_orbit_points)"),
      BT::InputPort<double>("overshoot_deg", kOvershootDeg,
        "degrees to keep going past one full turn, the same way round, so a boat that "
        "takes each hop early (tolerance 2 m is ~19 deg at 6 m) still sweeps >= 360; "
        "45 = one extra point at 8 points; 0 = stop at exactly 360; clamped to 0..360"),
      BT::InputPort<std::string>("direction", "cw", "cw | ccw"),
      BT::InputPort<double>("tolerance", "hop arrival radius, metres (absent: nav_orbit_tolerance_m)"),
      BT::InputPort<bool>("avoid", true,
        "plan each hop around known hazards (needs nav_mode shadow or on)"),
      BT::InputPort<double>("blocked_timeout_s", 15.0,
        "FAILURE after a hop has been blocked this long")};
  }

  BT::NodeStatus onStart() override
  {
    const std::string anchor = getInput<std::string>("anchor").value_or("entry");
    const bool cw = getInput<std::string>("direction").value_or("cw") != "ccw";
    const double radius = getInput<double>("radius").value_or(ctx_->nav.orbit_radius_m);
    const int points = getInput<int>("points").value_or(ctx_->nav.orbit_points);
    const double overshoot = getInput<double>("overshoot_deg").value_or(kOvershootDeg);

    Vec2 a, from;
    std::vector<path::Hazard> hazards;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!ctx_->pose_fresh || !ctx_->origin_set) {
        RCLCPP_WARN(log(), "CircleBuoy: no fresh pose");
        return BT::NodeStatus::FAILURE;
      }
      if (anchor == "fix") {
        const auto la = getInput<double>("lat");
        const auto lo = getInput<double>("lon");
        if (!la || !lo) {
          RCLCPP_WARN(log(), "CircleBuoy: anchor=fix needs lat and lon");
          return BT::NodeStatus::FAILURE;
        }
        a = nav::toLocal({la.value(), lo.value()}, ctx_->origin);
      } else {
        const bool have = (anchor == "exit") ? ctx_->have_exit : ctx_->have_entry;
        if (!have) {
          RCLCPP_WARN(log(), "CircleBuoy: %s buoy not available", anchor.c_str());
          return BT::NodeStatus::FAILURE;
        }
        a = (anchor == "exit") ? ctx_->exitp : ctx_->entry;
      }
      from = ctx_->boat;
      hazards = knownHazards(*ctx_);
    }
    const std::vector<Vec2> raw = path::orbitRing(a, from, radius, points, cw, overshoot);
    if (raw.empty()) {
      RCLCPP_WARN(log(), "CircleBuoy: points=%d makes no ring (needs at least 1)", points);
      return BT::NodeStatus::FAILURE;
    }
    int dropped = 0;
    ring_ = path::adjustRing(raw, a, hazards, ctx_->nav, &dropped);
    i_ = 0;
    RCLCPP_INFO(
      log(), "CircleBuoy %s: %d waypoints at %.1f m, %s (sweep %.0f deg with %.0f deg "
      "overshoot, %d point(s) dropped for a known hazard)",
      anchor.c_str(), points, radius, cw ? "CW" : "CCW",
      nav::sweepDeg(a, std::vector<Vec2>(ring_.begin() + 1, ring_.end()), ring_.front()),
      overshoot, dropped);
    cfg_ = legConfigFromPorts(*this);
    if (!getInput<double>("tolerance")) {cfg_.tolerance = ctx_->nav.orbit_tolerance_m;}
    return startHop();
  }

  BT::NodeStatus onRunning() override
  {
    const BT::NodeStatus st = leg_.step(true, ring_[i_], label(), log());
    if (st != BT::NodeStatus::SUCCESS) {return st;}
    ++i_;
    return startHop();
  }

  void onHalted() override
  {
    ring_.clear();
    i_ = 0;
    leg_.halt();
  }

private:
  /// The default overshoot_deg: one extra point at the default 8 points.
  static constexpr double kOvershootDeg = 45.0;

  std::string label() const
  {
    return "  orbit " + std::to_string(i_ + 1) + "/" + std::to_string(ring_.size());
  }

  /// Start hop i_ (0 = the hop onto the ring), or finish the orbit when none is left.
  BT::NodeStatus startHop()
  {
    if (i_ >= ring_.size()) {
      RCLCPP_INFO(log(), "CircleBuoy: circle complete");
      leg_.halt();
      return BT::NodeStatus::SUCCESS;
    }
    // The goal rule (path::clearPoint) against the field as it is NOW: adjustRing never
    // moves ring[0], and a track can appear after the ring was built; either can leave
    // a hop beside a buoy where it never arrives.
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const Vec2 p = path::clearPoint(
        ring_[i_], knownHazards(*ctx_), ctx_->nav.orbit_clear_m, ctx_->nav.goal_max_move_m);
      if (nav::norm(p - ring_[i_]) > 1e-9) {
        RCLCPP_INFO(log(), "CircleBuoy: hop %zu moved %.2f m clear of a known hazard", i_,
          nav::norm(p - ring_[i_]));
        ring_[i_] = p;
      }
    }
    return leg_.start(cfg_, ring_[i_], label(), log(), static_cast<int>(i_),
        static_cast<int>(ring_.size()));
  }

  std::vector<Vec2> ring_;
  std::size_t i_ = 0;
  path::LegConfig cfg_;
  LegDriver leg_;
};

/// Publishes the current position as the setpoint and never succeeds: whatever wraps it
/// (a Timeout, a sibling condition in a ReactiveFallback) ends it, so the give-up policy
/// stays visible in the tree instead of hidden in the leaf.
class HoldStation : public CrusaderAction
{
public:
  HoldStation(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override
  {
    sendHold(*ctx_, log(), "HoldStation at");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {return BT::NodeStatus::RUNNING;}
};

/// The next place to steer during the transit; the decision is nav::nextWaypoint(),
/// tested off the boat.
///
/// FAILURE ("nothing sensible left to steer to") is the transit loop's secondary exit.
/// The PRIMARY exit is NearExit: ending on an empty candidate list would let one missed
/// detection stop the boat mid-field.
class NextWaypoint : public CrusaderSyncAction
{
public:
  NextWaypoint(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("offset", 4.0, "metres to clear the buoy by")};
  }

  BT::NodeStatus tick() override
  {
    const double offset = getInput<double>("offset").value_or(4.0);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->pose_fresh) {return BT::NodeStatus::FAILURE;}

    const nav::NextWaypoint r = nav::nextWaypoint(
      ctx_->buoys, ctx_->boat, ctx_->heading_deg, ctx_->have_exit, ctx_->exitp,
      offset);
    ctx_->have_waypoint = r.ok;
    if (!r.ok) {
      RCLCPP_INFO(log(), "NextWaypoint: nothing left to steer to");
      return BT::NodeStatus::FAILURE;
    }
    ctx_->waypoint = r.wp;
    ctx_->waypoint_buoy_id = r.buoy_id;
    RCLCPP_INFO(
      log(), "NextWaypoint: %s at %.0f m",
      r.buoy_id < 0 ? "the exit" : ("buoy " + std::to_string(r.buoy_id)).c_str(),
      nav::norm(r.wp - ctx_->boat));
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace

void registerCrusaderNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<IsAutonomous>("IsAutonomous");
  factory.registerNodeType<EntryResolved>("EntryResolved");
  factory.registerNodeType<ExitResolved>("ExitResolved");
  factory.registerNodeType<NearExit>("NearExit");
  factory.registerNodeType<ResolveBuoyStates>("ResolveBuoyStates");
  factory.registerNodeType<ReportSafePassage>("ReportSafePassage");
  factory.registerNodeType<TaskAttempt>("TaskAttempt");
  factory.registerNodeType<WaitForRunStart>("WaitForRunStart");
  factory.registerNodeType<NavigateTo>("NavigateTo");
  factory.registerNodeType<CircleBuoy>("CircleBuoy");
  factory.registerNodeType<HoldStation>("HoldStation");
  factory.registerNodeType<NextWaypoint>("NextWaypoint");
  // Task 3, in their own file (src/task3_leaves.cpp).
  registerTask3Nodes(factory);
  registerFireNodes(factory);
  // Task 1 above Core, planned over the whole field (src/global_leaves.cpp).
  registerGlobalPassageNodes(factory);
}

}  // namespace crusader_bt
