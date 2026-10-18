// leaves.cpp — the per-gate Task 1 leaves and the shared ones; registers every leaf.
//
// No leaf here should be interesting: geometry that can be silently wrong lives in
// nav_math.hpp, tested on a laptop (test/test_nav_math.cpp). What is left is: read a
// port, call nav_math, publish, poll. A leaf that grows a formula: move it there and test it.
//
// LOAD-BEARING, in the reactive guard band re-ticked at 10 Hz:
//   ResolveBuoyStates and PublishSafePassageReport MUST NEVER RETURN FAILURE: it reaches
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

/// Sends SafePassageReport, rate-limited HERE: the guard band ticks at 10 Hz and the
/// handbook wants an update "when new buoys are detected or the mapped buoy state
/// changes", not 10 a second. ALWAYS SUCCESS (file header).
class PublishSafePassageReport : public CrusaderSyncAction
{
public:
  PublishSafePassageReport(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("min_period_s", 1.0, "floor between reports")};
  }

  BT::NodeStatus tick() override
  {
    const double period = getInput<double>("min_period_s").value_or(1.0);
    const auto now = std::chrono::steady_clock::now();
    if (last_.time_since_epoch().count() != 0) {
      const double dt = std::chrono::duration<double>(now - last_).count();
      if (dt < period) {return BT::NodeStatus::SUCCESS;}
    }
    last_ = now;
    if (ctx_->publish_report) {ctx_->publish_report();}
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::chrono::steady_clock::time_point last_{};
};

// ------------------------------------------------------------------- actions

/// Publishes the task token. This is not bookkeeping: the course activates its
/// light beacons only once a system reports it is attempting the passage
/// (handbook 3.4.11), so nothing on the water is detectable before this runs.
class SetTask : public CrusaderSyncAction
{
public:
  SetTask(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("token", "TASK_SAFE_PASSAGE", "an RxTask name")};
  }

  BT::NodeStatus tick() override
  {
    announceTask(*ctx_, getInput<std::string>("token").value_or("TASK_NONE"));
    return BT::NodeStatus::SUCCESS;
  }
};

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

/// Sets the tier TierIs and TierAtLeast read, for a mission that runs its tasks at different tiers.
class SetTier : public CrusaderSyncAction
{
public:
  SetTier(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("tier", "core | advanced | disruptive")};
  }

  BT::NodeStatus tick() override
  {
    const auto want = getInput<std::string>("tier");
    int tier = 0;
    if (!want || !parseTier(want.value(), &tier)) {
      RCLCPP_ERROR(
        log(), "SetTier: '%s' is not a tier (core | advanced | disruptive)",
        want ? want.value().c_str() : "<no tier port>");
      return BT::NodeStatus::FAILURE;
    }
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->tier = tier;
    }
    RCLCPP_INFO(log(), "SetTier: tier -> %s", tierName(tier));
    return BT::NodeStatus::SUCCESS;
  }
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
        "with avoid=false only: comma list of gate | dock, hazards this leg drives past"),
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
    std::lock_guard<std::mutex> lk(ctx_->mu);
    std::size_t at = 0;
    while (at <= exempt.size()) {
      std::size_t comma = exempt.find(',', at);
      if (comma == std::string::npos) {comma = exempt.size();}
      std::string tok = exempt.substr(at, comma - at);
      tok.erase(std::remove(tok.begin(), tok.end(), ' '), tok.end());
      at = comma + 1;
      if (tok == "gate") {
        for (const int id : {ctx_->gate_red_id, ctx_->gate_green_id}) {
          if (id >= 0) {cfg.exempt_buoys.push_back(id);}
        }
      } else if (tok == "dock") {
        cfg.exempt_dock = true;
      } else if (!tok.empty()) {
        RCLCPP_WARN(log(), "NavigateTo: unknown exempt '%s' (gate | dock): ignored", tok.c_str());
      }
    }
    return cfg;
  }

  Vec2 goal_;
  std::string why_;
  LegDriver leg_;
};

/// A stand-in for a detector: turns "something 50 m off the bow to starboard" into a
/// Waypoint on the blackboard, so the compute-leaf -> action-leaf wire can be exercised
/// in SITL with no camera. A real perception leaf has this shape but reads ctx_->buoys
/// (filled from /crsd/world_targets).
///
/// Named for what it does: a "DetectBuoy" that invented a buoy would be a lie in the tree.
class TargetAhead : public CrusaderSyncAction
{
public:
  TargetAhead(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("range", 50.0, "metres from the boat"),
      BT::InputPort<double>("bearing", 0.0, "degrees relative to the bow, cw+"),
      BT::InputPort<std::string>("why", "target", "what this is, for the log"),
      BT::OutputPort<Waypoint>("out", "where the next action should drive")};
  }

  BT::NodeStatus tick() override
  {
    const double range = getInput<double>("range").value_or(50.0);
    const double rel = getInput<double>("bearing").value_or(0.0);
    const std::string why = getInput<std::string>("why").value_or("target");

    Waypoint w;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!ctx_->pose_fresh || !ctx_->origin_set) {
        RCLCPP_WARN(log(), "TargetAhead: no fresh pose");
        return BT::NodeStatus::FAILURE;
      }
      // Bow-relative: with a NaN heading the bearing is a guess, so refuse
      // rather than steer somewhere plausible.
      if (!std::isfinite(ctx_->heading_deg)) {
        RCLCPP_WARN(log(), "TargetAhead: heading is NaN, refusing to guess");
        return BT::NodeStatus::FAILURE;
      }
      const Vec2 dir = nav::headingVec(ctx_->heading_deg + rel);
      const nav::LatLon ll = nav::toLatLon(ctx_->boat + dir * range, ctx_->origin);
      w.lat = ll.lat;
      w.lon = ll.lon;
      w.why = why;
    }
    setOutput("out", w);
    RCLCPP_INFO(
      log(), "TargetAhead: %s at %.0f m brg %+.0f -> %.7f, %.7f",
      w.why.c_str(), range, rel, w.lat, w.lon);
    return BT::NodeStatus::SUCCESS;
  }
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

// ============================================================== Disruptive tier
//
// The per-gate UAV handshake. NONE is a ROS action client, and only AwaitConfirmation
// can be RUNNING, so it is the only one that needs onHalted(). The runner owns every
// subscription; these read the Context its /crsd/passage_plan and /crsd/next_gate fill.

/// Has the aircraft reported, recently enough to still believe?
///
/// THE HANDBOOK GATE: above Core the UAV must overfly and report before the USV may
/// transit the field, so this sits in the guard band and its FAILURE halts the mission.
///
/// Also the dead-radio stop: rxl_link_node never repeats the last plan when the link
/// goes quiet, so the age here climbs and the boat stops instead of driving a passage
/// nobody can still confirm.
class PassagePlanFresh : public CrusaderCondition
{
public:
  PassagePlanFresh(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("max_age_s", 15.0,
      "seconds; the plan arrives at ~0.2 Hz so this is not the pose timeout")};
  }

  BT::NodeStatus tick() override
  {
    const double max_age = getInput<double>("max_age_s").value_or(15.0);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (ctx_->plan_version == 0) {
      warnOnce("no passage plan yet: the UAV has not reported");
      return BT::NodeStatus::FAILURE;
    }
    if (!ctx_->plan_fresh || ctx_->plan_age_s > max_age) {
      warnOnce("passage plan is stale");
      return BT::NodeStatus::FAILURE;
    }
    warned_ = false;
    return BT::NodeStatus::SUCCESS;
  }

private:
  /// The guard band re-ticks at 10 Hz: un-gated, this is ten copies a second.
  void warnOnce(const char * why)
  {
    if (warned_) {return;}
    warned_ = true;
    RCLCPP_WARN(log(), "%s (age %.1fs, v%u) - not transiting", why,
      ctx_->plan_age_s, static_cast<unsigned>(ctx_->plan_version));
  }
  bool warned_ = false;
};

/// Did the UAV supersede the plan since we last looked? The Disruptive trigger: in a
/// reactive branch, a new plan halts an in-flight NavigateTo and the leg restarts on it.
///
/// PRIMES ON ITS FIRST TICK rather than comparing against zero: the tree is rebuilt per
/// goal, so a plan that arrived BEFORE the goal would read as a change and cancel the
/// first leg before it started.
class PlanChanged : public CrusaderCondition
{
public:
  PlanChanged(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const std::uint32_t v = ctx_->plan_version;
    if (!primed_) {
      primed_ = true;
      seen_ = v;
      return BT::NodeStatus::FAILURE;          // "no change", by construction
    }
    if (v == seen_) {return BT::NodeStatus::FAILURE;}
    RCLCPP_INFO(log(), "plan v%u supersedes v%u - re-planning this leg",
      static_cast<unsigned>(v), static_cast<unsigned>(seen_));
    seen_ = v;            // consumed: report a change ONCE, not every tick
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool primed_ = false;
  std::uint32_t seen_ = 0;
};

/// Has the boat cleared every gate in its own plan?
///
/// THE BOAT DECIDES, not the aircraft: it holds all ten buoys, so it knows the red-green
/// pairs and which it has driven. Not a geometric test: the exit buoy can sit well inside
/// any sane arrival radius of the last gate. The aircraft can still SHORTEN a passage: a
/// confirmation that recolours a pair away leaves one fewer gate to drive.
class PassageComplete : public CrusaderCondition
{
public:
  PassageComplete(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const bool done = ctx_->passage.valid &&
      nav::nextGate(ctx_->passage, ctx_->cleared_gates) == nullptr;
    ctx_->passage_complete = done;               // for the report, not for logic
    return done ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

/// The gate order from the fused field, re-planned every tick: cheap (a handful of
/// distances), and a cached plan would keep driving the old field after a confirmation
/// brought new colours or positions, while the report showed the new one.
class PlanPassage : public CrusaderSyncAction
{
public:
  PlanPassage(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_width", 20.0, "widest a real gate can be, m"),
      BT::InputPort<double>("min_width", 2.0, "narrowest a real gate can be, m"),
      BT::InputPort<double>("max_cross_angle", 60.0,
        "a pair is a gate only if crossing it red-to-starboard is within this many degrees "
        "of the entry -> exit axis; otherwise both buoys are singles")};
  }

  BT::NodeStatus tick() override
  {
    const double wmax = getInput<double>("max_width").value_or(20.0);
    const double wmin = getInput<double>("min_width").value_or(2.0);
    const double wang = getInput<double>("max_cross_angle").value_or(60.0);

    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->have_entry || !ctx_->have_exit) {
      RCLCPP_WARN_THROTTLE(
        log(), *ctx_->node->get_clock(), 5000,
        "cannot plan the passage: %s missing from the aircraft's field",
        !ctx_->have_entry ? "ENTRY" : "EXIT");
      return BT::NodeStatus::FAILURE;
    }

    ctx_->passage = nav::planPassage(ctx_->buoys, ctx_->entry, ctx_->exitp, wmax, wmin, wang);
    if (!ctx_->passage.valid) {
      RCLCPP_WARN_THROTTLE(
        log(), *ctx_->node->get_clock(), 5000,
        "cannot plan the passage: %s", ctx_->passage.why.c_str());
      return BT::NodeStatus::FAILURE;
    }

    // Logged only when it changes, or a 10 Hz tick buries everything else.
    const std::size_t n = ctx_->passage.gates.size();
    if (n != last_n_ || ctx_->passage.unpaired.size() != last_unpaired_) {
      last_n_ = n;
      last_unpaired_ = ctx_->passage.unpaired.size();
      std::string line;
      for (const auto & g : ctx_->passage.gates) {
        line += " (" + std::to_string(g.red_id) + "," + std::to_string(g.green_id) + ")";
      }
      RCLCPP_INFO(
        log(), "passage planned: %zu gate(s)%s, %zu buoy(s) unpaired (singles, held to their "
        "side by fences in nav_mode on)",
        n, line.c_str(), ctx_->passage.unpaired.size());
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::size_t last_n_ = static_cast<std::size_t>(-1);
  std::size_t last_unpaired_ = static_cast<std::size_t>(-1);
};

/// DEPRECATED PASS-THROUGH: `out` = `in`, SUCCESS. NavigateTo avoids by itself
/// (docs/nav2_avoidance_spec.md 5.5). Registered ONLY so an old XML that still names it
/// loads and runs; `clearance` and `margin` are ignored. It says so once per run.
class AvoidObstacles : public CrusaderSyncAction
{
public:
  AvoidObstacles(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Waypoint>("in", "the goal an upstream leaf chose"),
      BT::OutputPort<Waypoint>("out", "the same goal"),
      BT::InputPort<double>("clearance", 5.0, "ignored"),
      BT::InputPort<double>("margin", 2.0, "ignored")};
  }

  BT::NodeStatus tick() override
  {
    if (!warned_) {
      warned_ = true;
      RCLCPP_WARN(
        log(), "AvoidObstacles is deprecated and does nothing: NavigateTo avoids by itself");
    }
    const auto in = getInput<Waypoint>("in");
    if (!in) {return BT::NodeStatus::FAILURE;}
    setOutput("out", in.value());
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool warned_ = false;
};

/// Point gate_red_id / gate_green_id at the first gate not yet cleared. FAILURE = the
/// passage is done: the loop exit, not an error (PassageComplete asks the same one level up).
class SelectNextGate : public CrusaderSyncAction
{
public:
  SelectNextGate(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const nav::PlannedGate * g = nav::nextGate(ctx_->passage, ctx_->cleared_gates);
    if (g == nullptr) {return BT::NodeStatus::FAILURE;}

    if (g->red_id != ctx_->gate_red_id || g->green_id != ctx_->gate_green_id) {
      RCLCPP_INFO(
        log(), "driving gate red %d / green %d, %.1f m wide, %.0f m along",
        g->red_id, g->green_id, g->width_m, g->along_m);
    }
    ctx_->gate_red_id = g->red_id;
    ctx_->gate_green_id = g->green_id;
    return BT::NodeStatus::SUCCESS;
  }
};

/// Tell the aircraft a gate is done and ask "is the rest of the field still where you
/// said?". It acks this sequence number and retransmits all ten buoys; the boat re-plans
/// from that. One publish, SUCCESS in the same tick; the waiting is AwaitConfirmation's.
///
/// The sequence advances only when the previous gate was cleared, so a failed leg
/// re-asks under the SAME number and gets the aircraft's idempotent repeat.
class RequestConfirmation : public CrusaderSyncAction
{
public:
  RequestConfirmation(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    // FOR THE LOG ONLY: seq does not count gates (the entry orbit asks first, so seq 2
    // is gate 1), and "gate 1" logged at the entry sends a reader looking for the wrong one.
    return {BT::InputPort<std::string>("what", "gate",
      "what the boat just finished, for the log line")};
  }

  BT::NodeStatus tick() override
  {
    std::uint8_t seq;
    bool retry;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      retry = !ctx_->gate_cleared && ctx_->gate_seq != 0;
      if (!retry) {
        ctx_->gate_seq = static_cast<std::uint8_t>(ctx_->gate_seq + 1);
      }
      ctx_->gate_cleared = false;
      ctx_->have_confirmation = false;
      seq = ctx_->gate_seq;
    }
    if (ctx_->report_gate_reached) {ctx_->report_gate_reached(seq);}
    const std::string what = getInput<std::string>("what").value_or("gate");
    RCLCPP_INFO(
      log(), "checkpoint %u (%s): %s", static_cast<unsigned>(seq), what.c_str(),
      retry ? "did not finish - asking the aircraft to confirm again"
            : "done, asking the aircraft to confirm the field");
    return BT::NodeStatus::SUCCESS;
  }
};

/// The boat is through this gate; recorded by its two buoy ids. Last step of the leg, so
/// anything failing earlier leaves the gate un-cleared and it is driven again. Keyed on
/// the BUOYS, not a counter, so a recolour that reshuffles the order cannot make the boat
/// skip one it never drove.
class MarkGateCleared : public CrusaderSyncAction
{
public:
  MarkGateCleared(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const int r = ctx_->gate_red_id, g = ctx_->gate_green_id;
    if (r == nav::kNoBuoy || g == nav::kNoBuoy || r < 0 || g < 0) {
      return BT::NodeStatus::FAILURE;
    }
    if (!nav::gateCleared(ctx_->cleared_gates, r, g)) {
      ctx_->cleared_gates.push_back({r, g});
    }
    ctx_->gate_cleared = true;
    RCLCPP_INFO(
      log(), "through gate red %d / green %d (%zu of %zu cleared)", r, g,
      ctx_->cleared_gates.size(), ctx_->passage.gates.size());
    return BT::NodeStatus::SUCCESS;
  }
};

/// Wait for the aircraft to confirm the field before driving the next gate.
///
/// KEEPS ASKING, THEN CARRIES ON: re-requests every `retry_s` cover a lost reply;
/// FAILURE at `timeout_s` lets the tree drive on with the last field rather than park
/// forever. A deliberate trade, and the WARN keeps it visible: otherwise proceeding on
/// an unconfirmed field looks identical to a confirmed one.
///
/// NOTHING HOLDS STATION HERE, nor needs to: under GUIDED the autopilot is still holding
/// the gate's `through` setpoint and nothing below publishes a new one.
class AwaitConfirmation : public CrusaderAction
{
public:
  AwaitConfirmation(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout_s", 120.0,
        "give up waiting and drive on with the last field"),
      BT::InputPort<double>("retry_s", 3.0,
        "how often to re-send the request while waiting"),
      BT::InputPort<bool>("release_on_change", false,
        "true: a CHANGED field also ends the wait - the aircraft re-tasked instead of acking, "
        "and the tree re-plans against it. Changed = a colour, a buoy, or a move beyond "
        "retask_move_m (gp::fieldChange); the aircraft's own re-report scatter is not one"),
      BT::InputPort<double>("retask_move_m", 2.0,
        "with release_on_change: a buoy that moved less than this has not changed")};
  }

  BT::NodeStatus onStart() override
  {
    t0_ = std::chrono::steady_clock::now();
    last_ask_ = t0_;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      field0_ = gp::fieldSig(ctx_->plan, ctx_->entry, ctx_->exitp);
    }
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    std::uint8_t seq;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (ctx_->have_confirmation) {
        RCLCPP_INFO(
          log(), "checkpoint %u confirmed by the aircraft",
          static_cast<unsigned>(ctx_->gate_seq));
        return BT::NodeStatus::SUCCESS;
      }
      if (getInput<bool>("release_on_change").value_or(false)) {
        const std::string change = gp::fieldChange(
          field0_, gp::fieldSig(ctx_->plan, ctx_->entry, ctx_->exitp),
          getInput<double>("retask_move_m").value_or(2.0));
        if (!change.empty()) {
          RCLCPP_INFO(
            log(), "checkpoint %u: the aircraft changed the field (%s) - re-planning",
            static_cast<unsigned>(ctx_->gate_seq), change.c_str());
          return BT::NodeStatus::SUCCESS;
        }
      }
      seq = ctx_->gate_seq;
    }

    const auto now = std::chrono::steady_clock::now();
    const double waited = std::chrono::duration<double>(now - t0_).count();
    const double since_ask = std::chrono::duration<double>(now - last_ask_).count();

    if (since_ask >= getInput<double>("retry_s").value_or(3.0)) {
      last_ask_ = now;
      if (ctx_->report_gate_reached) {ctx_->report_gate_reached(seq);}
      RCLCPP_INFO(
        log(), "checkpoint %u: re-asking for confirmation (%.0fs)",
        static_cast<unsigned>(seq), waited);
    }

    if (waited >= getInput<double>("timeout_s").value_or(120.0)) {
      RCLCPP_WARN(
        log(),
        "checkpoint %u NOT confirmed after %.0fs - driving on with the last field "
        "the aircraft sent. The passage may have moved since.",
        static_cast<unsigned>(seq), waited);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  std::chrono::steady_clock::time_point t0_;
  std::chrono::steady_clock::time_point last_ask_;
  gp::FieldSig field0_;          ///< the aircraft's field when the wait began
};

class GateWaypoint : public CrusaderSyncAction
{
public:
  GateWaypoint(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("point", "through", "approach | through"),
      BT::InputPort<double>("standoff", "metres past the middle (absent: nav_gate_standoff_m)"),
      BT::InputPort<double>("approach", "metres short of the middle (absent: nav_gate_approach_m)"),
      BT::InputPort<double>("offset", 4.0, "metres to clear a LONE buoy by"),
      BT::OutputPort<Waypoint>("out", "where the next action should drive")};
  }

  BT::NodeStatus tick() override
  {
    const std::string which = getInput<std::string>("point").value_or("through");
    const double standoff = getInput<double>("standoff").value_or(ctx_->nav.gate_standoff_m);
    const double approach = getInput<double>("approach").value_or(ctx_->nav.gate_approach_m);
    const double offset = getInput<double>("offset").value_or(4.0);

    nav::Vec2 target;
    std::string why;
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->origin_set) {return BT::NodeStatus::FAILURE;}

    const int r = ctx_->gate_red_id;
    const int g = ctx_->gate_green_id;
    const nav::Gate plain = nav::gateFromIds(ctx_->buoys, r, g, standoff, approach);
    const std::vector<path::Hazard> hz = knownHazards(*ctx_);

    if (plain.valid) {
      // Move the two points off anything on the straight crossing: the same hazard set and
      // exemption the crossing leg checks (exempt="gate").
      bool tight = false;
      const nav::Gate gate = path::clearGateWaypoints(
        nav::findById(ctx_->buoys, r)->p, nav::findById(ctx_->buoys, g)->p, standoff, approach,
        path::exempt(hz, {r, g}, false, ctx_->nav.exempt_radius_m), ctx_->nav, nullptr, &tight);
      target = (which == "approach") ? gate.approach : gate.through;
      const nav::Vec2 was = (which == "approach") ? plain.approach : plain.through;
      why = "gate " + std::to_string(static_cast<int>(ctx_->gate_seq)) + " " +
        which + " (red " + std::to_string(r) + ", green " + std::to_string(g) + ")" +
        (nav::norm(target - was) > 0.01 ? (tight ? ", moved clear (tight)" : ", moved clear") : "");
    } else {
      const nav::Buoy * lone = nav::findById(ctx_->buoys, r);
      if (lone == nullptr) {lone = nav::findById(ctx_->buoys, g);}
      if (lone == nullptr || !ctx_->have_exit) {
        RCLCPP_WARN(log(), "gate %u unusable: %s",
          static_cast<unsigned>(ctx_->gate_seq), plain.why);
        return BT::NodeStatus::FAILURE;
      }
      const nav::Vec2 travel = ctx_->exitp - ctx_->boat;
      target = nav::sideWaypoint(lone->p, travel, lone->state, offset);
      why = "lone buoy " + std::to_string(lone->id) + " (" + plain.why + ")";
      RCLCPP_WARN(log(), "gate %u: %s - steering past the one we have",
        static_cast<unsigned>(ctx_->gate_seq), plain.why);
    }

    // the goal rule, whatever branch chose the point: a no-op for a gate point that
    // clearGateWaypoints placed (at least the tight clearance), a last resort when nothing fitted
    const nav::Vec2 clear =
      path::clearPoint(target, hz, ctx_->nav.gate_tight_clear_m, ctx_->nav.goal_max_move_m);
    if (nav::norm(clear - target) > 1e-9) {
      why += ", nudged " + std::to_string(static_cast<int>(nav::norm(clear - target) * 100.0)) +
        " cm off a hazard";
      target = clear;
    }

    const nav::LatLon ll = nav::toLatLon(target, ctx_->origin);
    setOutput("out", Waypoint{ll.lat, ll.lon, why});
    return BT::NodeStatus::SUCCESS;
  }
};

void registerCrusaderNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<IsAutonomous>("IsAutonomous");
  factory.registerNodeType<EntryResolved>("EntryResolved");
  factory.registerNodeType<ExitResolved>("ExitResolved");
  factory.registerNodeType<NearExit>("NearExit");
  factory.registerNodeType<ResolveBuoyStates>("ResolveBuoyStates");
  factory.registerNodeType<PublishSafePassageReport>("PublishSafePassageReport");
  factory.registerNodeType<SetTask>("SetTask");
  factory.registerNodeType<TaskAttempt>("TaskAttempt");
  factory.registerNodeType<WaitForRunStart>("WaitForRunStart");
  factory.registerNodeType<SetTier>("SetTier");
  factory.registerNodeType<NavigateTo>("NavigateTo");
  factory.registerNodeType<CircleBuoy>("CircleBuoy");
  factory.registerNodeType<HoldStation>("HoldStation");
  factory.registerNodeType<NextWaypoint>("NextWaypoint");
  factory.registerNodeType<TargetAhead>("TargetAhead");
  // Disruptive tier: the UAV handshake.
  factory.registerNodeType<PassagePlanFresh>("PassagePlanFresh");
  factory.registerNodeType<PlanChanged>("PlanChanged");
  factory.registerNodeType<PassageComplete>("PassageComplete");
  factory.registerNodeType<PlanPassage>("PlanPassage");
  factory.registerNodeType<SelectNextGate>("SelectNextGate");
  factory.registerNodeType<AvoidObstacles>("AvoidObstacles");
  factory.registerNodeType<RequestConfirmation>("RequestConfirmation");
  factory.registerNodeType<AwaitConfirmation>("AwaitConfirmation");
  factory.registerNodeType<GateWaypoint>("GateWaypoint");
  factory.registerNodeType<MarkGateCleared>("MarkGateCleared");
  // Task 3, in their own file (src/task3_leaves.cpp).
  registerTask3Nodes(factory);
  registerFireNodes(factory);
  // Task 1 above Core, planned over the whole field (src/global_leaves.cpp).
  registerGlobalPassageNodes(factory);
}

}  // namespace crusader_bt
