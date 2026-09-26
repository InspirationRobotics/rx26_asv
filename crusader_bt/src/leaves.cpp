// leaves.cpp — every Task 1 leaf. Eleven of them, and none is interesting.
//
// That is the design working. All the geometry that can be silently wrong lives
// in nav_math.hpp, which compiles and tests on a laptop in a second
// (test/test_nav_math.cpp, 57 checks). What is left here is: read a port, call
// nav_math, publish, poll. If a leaf in this file grows a formula, move the
// formula to nav_math and test it.
//
// TWO CONTRACTS THAT ARE LOAD-BEARING, both in the reactive guard band that is
// re-ticked ten times a second:
//
//   ResolveBuoyStates and PublishSafePassageReport MUST NEVER RETURN FAILURE.
//   A FAILURE there propagates to the root and ends the mission, so a single
//   dropped camera frame would abort the run. "I saw nothing this tick" is
//   SUCCESS; EntryResolved is what gates on actually knowing something.
//
//   NextWaypoint's FAILURE is not an error — it is how the transit loop ends
//   when there is nothing sensible left to steer to.
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

#include "crusader_bt/context.hpp"
#include "crusader_bt/nav_math.hpp"

// NOTE ON PORTS. BT::InputPort has two overloads: (name, description) and
// (name, default_value, description). There is NO (name, default_value) form —
// it binds to the first, fails to convert the default into a StringView, and
// emits a template error long enough to bury what it is telling you. A port
// with a default MUST also carry a description.

namespace crusader_bt
{
namespace
{

using nav::Vec2;

// ---------------------------------------------------------------- conditions

/// The autonomy latch, at the top of the guard band.
///
/// Deliberately a CONDITION and deliberately first: a tree that ticks missions
/// while the pilot has the boat is a tree that fights the pilot. Its FAILURE
/// halts every descendant within one tick.
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

/// Are we close enough to the exit buoy to stop transiting?
///
/// `radius` MUST be larger than CircleBuoy's radius, or the transit would end
/// inside the orbit and the circle would start from the wrong place. The tree
/// uses 8 m against a 6 m orbit.
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

/// Refreshes nothing by itself — the runner's subscriptions already filled the
/// context. This leaf exists so the tree SHOWS where the world update happens,
/// and so that a future ledger (flash discrimination, colour-change tracking)
/// has an obvious home that is already ticked at the right rate.
///
/// ALWAYS SUCCESS. See the file header.
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

/// Sends SafePassageReport. Rate-limited HERE rather than by the tree, because
/// the guard band ticks at 10 Hz and the handbook wants an update "when new
/// buoys are detected or the mapped buoy state changes" — not 10 a second.
///
/// ALWAYS SUCCESS. See the file header.
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
    const std::string t = getInput<std::string>("token").value_or("TASK_NONE");
    if (ctx_->set_task) {ctx_->set_task(t);}
    RCLCPP_INFO(log(), "current_task -> %s", t.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

/// Reads a named target out of the context, publishes it, and polls for arrival.
///
/// `target` selects WHICH position, as a string rather than a typed port, so the
/// XML needs no custom convertFromString (see context.hpp):
///     "waypoint"  the one NextWaypoint just wrote
///     "approach"  the team-provided lat/lon from the action goal
///     "exit"      the exit buoy
///
/// WITH publish_setpoints FALSE it does not publish but still polls arrival.
/// That is the read-only posture for a water test: a human drives, and the tree
/// tracks whether the boat reached where the mission wanted it. A version that
/// faked arrival would prove nothing.
class NavigateTo : public CrusaderAction
{
public:
  NavigateTo(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target", "waypoint",
                                 "port | waypoint | approach | exit | home | fix"),
      // THE PERCEPTION WIRE. With target="port" this reads a Waypoint written
      // by an upstream compute leaf, so the XML shows where the goal came from:
      //     <SomeDetector out="{goal}"/>
      //     <NavigateTo target="port" goal="{goal}"/>
      BT::InputPort<Waypoint>("goal", "a Waypoint from an upstream leaf"),
      BT::InputPort<double>("lat", 0.0, "with target=fix: latitude"),
      BT::InputPort<double>("lon", 0.0, "with target=fix: longitude"),
      BT::InputPort<double>("tolerance", 2.0, "arrival radius, metres"),
      BT::InputPort<double>("resend_m", 1.5,
        "re-send the setpoint when the goal moves further than this")};
  }

  BT::NodeStatus onStart() override
  {
    const std::string which = getInput<std::string>("target").value_or("waypoint");
    if (!resolve(which, goal_)) {
      RCLCPP_WARN(log(), "NavigateTo: target '%s' is not available", which.c_str());
      return BT::NodeStatus::FAILURE;
    }
    nav::LatLon ll;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ll = nav::toLatLon(goal_, ctx_->origin);
    }
    if (ctx_->publish_setpoints && ctx_->send_setpoint) {
      ctx_->send_setpoint(ll);
      RCLCPP_INFO(log(), "NavigateTo %s -> %.7f, %.7f", which.c_str(), ll.lat, ll.lon);
    } else {
      RCLCPP_INFO(
        log(), "NavigateTo %s -> %.7f, %.7f  [NOT SENT: publish_setpoints is false]",
        which.c_str(), ll.lat, ll.lon);
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const double tol = getInput<double>("tolerance").value_or(2.0);

    // THE GOAL IS ALLOWED TO MOVE. AvoidObstacles rewrites {goal} as the boat
    // closes on a blocker, and a leaf that published once in onStart would keep
    // driving the original line straight through it.
    //
    // Re-sent only when the point has actually MOVED further than resend_m. A
    // setpoint republished at 10 Hz is a different control mode from this one -
    // it is a stream, ArduRover treats it as one, and the deadband is what
    // keeps this leaf a leaf rather than a controller.
    //
    // resolve() is called outside the lock, as onStart does: it touches ctx_
    // directly and takes no lock of its own.
    const double resend = getInput<double>("resend_m").value_or(1.5);
    const std::string which = getInput<std::string>("target").value_or("waypoint");
    nav::Vec2 want;
    if (resend > 0.0 && resolve(which, want) && nav::norm(want - goal_) > resend) {
      goal_ = want;
      nav::LatLon ll;
      {
        std::lock_guard<std::mutex> lk2(ctx_->mu);
        ll = nav::toLatLon(goal_, ctx_->origin);
      }
      if (ctx_->publish_setpoints && ctx_->send_setpoint) {
        ctx_->send_setpoint(ll);
        RCLCPP_INFO(
          log(), "NavigateTo %s moved -> %.7f, %.7f", which.c_str(), ll.lat, ll.lon);
      }
    }

    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->pose_fresh) {return BT::NodeStatus::RUNNING;}
    if (nav::norm(goal_ - ctx_->boat) <= tol) {
      if (ctx_->consume_buoy && ctx_->waypoint_buoy_id >= 0) {
        ctx_->consume_buoy(ctx_->waypoint_buoy_id);
      }
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

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
      // Literal coordinates from the XML. This is what lets a tree be written
      // and driven with no perception at all -- the whole point of the
      // waypoint-tour demo, which exercises every phase transition the real
      // mission uses without needing a single buoy.
      if (!ctx_->origin_set) {return false;}
      const auto la = getInput<double>("lat");
      const auto lo = getInput<double>("lon");
      if (!la || !lo) {return false;}
      out = nav::toLocal({la.value(), lo.value()}, ctx_->origin);
      return true;
    }
    return false;
  }

  Vec2 goal_;
  std::string why_;
};

/// A stand-in for a detector: turns "something is 50 m off the bow to
/// starboard" into a Waypoint on the blackboard.
///
/// THIS IS THE SHAPE EVERY PERCEPTION LEAF HAS. A real one would read
/// ctx_->buoys (which the runner fills from /crsd/world_targets), pick the
/// target it cares about and write its position. This one computes the position
/// from a range and bearing relative to the boat, so the wire from a compute
/// leaf to an action leaf can be exercised in SITL with no camera at all.
///
/// It is named for what it does rather than for what it stands in for. A leaf
/// called "DetectBuoy" that invented a buoy would be a lie in the tree.
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
      // Relative to the BOW, so it needs the heading. NaN heading means we do
      // not know which way we are pointing, and a bearing off an unknown datum
      // is a guess -- refuse rather than steer somewhere plausible.
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

/// Drives `points` waypoints once around a buoy. Core Tier: ENTRY clockwise
/// before the transit, EXIT counterclockwise to complete the task.
class CircleBuoy : public CrusaderAction
{
public:
  CircleBuoy(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("anchor", "entry", "entry | exit | fix"),
      BT::InputPort<double>("lat", 0.0, "with anchor=fix: latitude"),
      BT::InputPort<double>("lon", 0.0, "with anchor=fix: longitude"),
      BT::InputPort<double>("radius", 6.0, "orbit radius, metres"),
      BT::InputPort<int>("points", 5, "waypoints around the circle"),
      BT::InputPort<std::string>("direction", "cw", "cw | ccw"),
      BT::InputPort<double>("tolerance", 2.0, "arrival radius, metres")};
  }

  BT::NodeStatus onStart() override
  {
    const std::string anchor = getInput<std::string>("anchor").value_or("entry");
    const bool cw = getInput<std::string>("direction").value_or("cw") != "ccw";
    const double radius = getInput<double>("radius").value_or(6.0);
    const int points = getInput<int>("points").value_or(5);

    Vec2 a, from;
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
    }
    ring_ = nav::orbit(a, from, radius, points, cw);
    i_ = 0;
    RCLCPP_INFO(
      log(), "CircleBuoy %s: %d waypoints at %.1f m, %s (sweep %.0f deg)",
      anchor.c_str(), points, radius, cw ? "CW" : "CCW",
      nav::sweepDeg(a, ring_, from));
    send();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const double tol = getInput<double>("tolerance").value_or(2.0);
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!ctx_->pose_fresh) {return BT::NodeStatus::RUNNING;}
      if (nav::norm(ring_[i_] - ctx_->boat) > tol) {return BT::NodeStatus::RUNNING;}
    }
    if (++i_ >= ring_.size()) {
      RCLCPP_INFO(log(), "CircleBuoy: circle complete");
      return BT::NodeStatus::SUCCESS;
    }
    send();
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {ring_.clear(); i_ = 0;}

private:
  void send()
  {
    if (i_ >= ring_.size()) {return;}
    nav::LatLon ll;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ll = nav::toLatLon(ring_[i_], ctx_->origin);
    }
    if (ctx_->publish_setpoints && ctx_->send_setpoint) {
      ctx_->send_setpoint(ll);
    }
    RCLCPP_INFO(
      log(), "  orbit %zu/%zu -> %.7f, %.7f%s", i_ + 1, ring_.size(), ll.lat, ll.lon,
      ctx_->publish_setpoints ? "" : "  [NOT SENT]");
  }

  std::vector<Vec2> ring_;
  std::size_t i_ = 0;
};

/// Publishes the current position as the setpoint, and never succeeds. Whatever
/// wraps it — a Timeout, or a sibling condition in a ReactiveFallback — is what
/// ends it. That is deliberate: a HoldStation that timed out on its own would
/// put the give-up policy in the leaf instead of in the tree, where it is
/// visible.
class HoldStation : public CrusaderAction
{
public:
  HoldStation(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->pose_fresh) {return BT::NodeStatus::RUNNING;}
    const nav::LatLon ll = nav::toLatLon(ctx_->boat, ctx_->origin);
    if (ctx_->publish_setpoints && ctx_->send_setpoint) {ctx_->send_setpoint(ll);}
    RCLCPP_INFO(log(), "HoldStation at %.7f, %.7f", ll.lat, ll.lon);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {return BT::NodeStatus::RUNNING;}
};

/// Works out the next place to steer during the transit.
///
/// The whole decision is nav::nextWaypoint(), which is tested off-boat. This
/// leaf reads the context, calls it, and writes the answer back.
///
/// FAILURE means "nothing sensible left to steer to", which is the transit
/// loop's secondary exit. The PRIMARY exit is NearExit — arriving at the exit
/// buoy, not running out of buoys. Ending the transit on an empty candidate
/// list would make a single missed detection stop the boat mid-field.
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
// Six leaves the UAV handshake needs. NONE of them is a ROS action client:
// four decide in one tick and return, one publishes once and returns, and only
// AwaitConfirmation can be RUNNING -- so it is the only one that needs onHalted().
//
// The runner owns every subscription, as always. These read the Context fields
// its /crsd/passage_plan and /crsd/next_gate callbacks filled in.

/// Has the aircraft reported, and recently enough to still believe?
///
/// THE HANDBOOK GATE. Above Core tier the UAV must complete its overfly and
/// report before the USV may transit into the field, so this sits in the guard
/// band and its FAILURE halts the whole mission in one tick.
///
/// It is also the dead-radio stop. rxl_link_node publishes NOTHING when the
/// link goes quiet -- it never repeats the last plan -- precisely so the age
/// here keeps climbing and the boat stops instead of driving a passage nobody
/// can still confirm.
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
  /// The guard band re-ticks at 10 Hz, so an un-gated log line is ten copies of
  /// the same sentence every second.
  void warnOnce(const char * why)
  {
    if (warned_) {return;}
    warned_ = true;
    RCLCPP_WARN(log(), "%s (age %.1fs, v%u) - not transiting", why,
      ctx_->plan_age_s, static_cast<unsigned>(ctx_->plan_version));
  }
  bool warned_ = false;
};

/// Did the UAV supersede the plan since we last looked?
///
/// The Disruptive trigger, and the whole reason the tier needs no new
/// machinery: this sits in a reactive branch, so a new plan halts an in-flight
/// NavigateTo and the leg restarts against the new one.
///
/// PRIMES ON ITS FIRST TICK rather than comparing against zero. The tree is
/// rebuilt per goal, so a plan that arrived BEFORE the goal would otherwise
/// read as a change on tick one and cancel the first leg before it started.
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

/// Has the aircraft said there are no more gates?
///
/// The ONLY thing that ends the transit loop. Set when a pair arrives as
/// 255/255. Deliberately not a geometric test: the boat must clear every gate
/// before the exit, and the exit buoy can sit well inside any sane arrival
/// radius of the last one.
/// Has the boat cleared every gate in its own plan?
///
/// THE BOAT DECIDES, not the aircraft. It holds all ten buoys, so it knows how
/// many red-green pairs there are and which it has driven. The aircraft used to
/// end the transit by answering 255/255; it no longer assigns gates, so it can
/// no longer end them either. It can still SHORTEN a passage -- a confirmation
/// carrying a field with one pair recoloured away leaves one fewer gate to
/// drive -- which is the honest way for it to influence this.
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

/// Work out the gate order from the fused field. One call, every tick.
///
/// Cheap enough to re-run rather than cache: two pairs out of ten buoys is a
/// handful of distances. Re-running is the POINT -- a confirmation can bring
/// new colours or new positions, and a cached plan would keep driving the old
/// field while the report showed the new one.
class PlanPassage : public CrusaderSyncAction
{
public:
  PlanPassage(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_width", 20.0, "widest a real gate can be, m"),
      BT::InputPort<double>("min_width", 2.0, "narrowest a real gate can be, m")};
  }

  BT::NodeStatus tick() override
  {
    const double wmax = getInput<double>("max_width").value_or(20.0);
    const double wmin = getInput<double>("min_width").value_or(2.0);

    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->have_entry || !ctx_->have_exit) {
      RCLCPP_WARN_THROTTLE(
        log(), *ctx_->node->get_clock(), 5000,
        "cannot plan the passage: %s missing from the aircraft's field",
        !ctx_->have_entry ? "ENTRY" : "EXIT");
      return BT::NodeStatus::FAILURE;
    }

    ctx_->passage = nav::planPassage(ctx_->buoys, ctx_->entry, ctx_->exitp, wmax, wmin);
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
        log(), "passage planned: %zu gate(s)%s, %zu buoy(s) unpaired",
        n, line.c_str(), ctx_->passage.unpaired.size());
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::size_t last_n_ = static_cast<std::size_t>(-1);
  std::size_t last_unpaired_ = static_cast<std::size_t>(-1);
};

/// Rewrite {goal} to steer around whatever is in the way.
///
/// WHY THE TREE AND NOT THE AUTOPILOT. ArduRover's OA_TYPE avoidance, fed
/// OBSTACLE_DISTANCE by proximity_bridge, is a black box: it cannot be watched
/// on the tree view, it cannot be exercised in a pool, and when it does
/// something surprising nothing in our logs says why. Here the detour is an
/// ordinary waypoint, logged in the same line as every other waypoint, and
/// NavigateTo re-sends the setpoint when it moves.
///
/// WHAT COUNTS AS AN OBSTACLE. Two things, and the second is easy to forget:
///
///   * ctx_->obstacles - tracked contacts that matched no buoy in the
///     aircraft's field. This is where the LiDAR clustering ends up:
///     lidar_cluster_node -> target_tracker -> world_targets -> fusePassage.
///
///   * every PASSAGE buoy that is not one of the two in the gate being driven.
///     Gate 1's pair is still floating there while the boat runs to gate 2, and
///     it will happily drive through it otherwise. Only the current red and
///     green are exempt, because driving between those two IS the task.
class AvoidObstacles : public CrusaderSyncAction
{
public:
  AvoidObstacles(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Waypoint>("in", "the goal an upstream leaf chose"),
      BT::OutputPort<Waypoint>("out", "the same goal, or a detour short of it"),
      BT::InputPort<double>("clearance", 5.0, "how near a thing may come, m"),
      BT::InputPort<double>("margin", 2.0, "extra push beyond clearance, m")};
  }

  BT::NodeStatus tick() override
  {
    const auto in = getInput<Waypoint>("in");
    if (!in) {return BT::NodeStatus::FAILURE;}
    const double clearance = getInput<double>("clearance").value_or(5.0);
    const double margin = getInput<double>("margin").value_or(2.0);

    Waypoint w = in.value();
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->origin_set || !ctx_->pose_fresh) {
      setOutput("out", w);                  // no pose: cannot reason, do not guess
      return BT::NodeStatus::SUCCESS;
    }

    std::vector<nav::Buoy> hazards = ctx_->obstacles;
    for (const nav::Buoy & b : ctx_->buoys) {
      if (b.id == ctx_->gate_red_id || b.id == ctx_->gate_green_id) {continue;}
      hazards.push_back(b);
    }

    const nav::Vec2 goal = nav::toLocal({w.lat, w.lon}, ctx_->origin);
    const nav::Detour d =
      nav::avoidObstacles(ctx_->boat, goal, hazards, clearance, margin);
    if (d.detoured) {
      const nav::LatLon ll = nav::toLatLon(d.wp, ctx_->origin);
      w.lat = ll.lat;
      w.lon = ll.lon;
      w.why = "around buoy " + std::to_string(d.around_id) + " (" +
        std::to_string(static_cast<int>(d.miss_m + 0.5)) + " m off track), then " +
        in.value().why;
      if (d.around_id != last_) {
        last_ = d.around_id;
        RCLCPP_INFO(
          log(), "steering around buoy %d, %.1f m off the direct line",
          d.around_id, d.miss_m);
      }
    } else if (last_ != nav::kNoBuoy) {
      last_ = nav::kNoBuoy;
      RCLCPP_INFO(log(), "track is clear again");
    }
    setOutput("out", w);
    return BT::NodeStatus::SUCCESS;
  }

private:
  int last_ = nav::kNoBuoy;
};

/// Point gate_red_id / gate_green_id at the first gate not yet cleared.
///
/// FAILURE means the passage is done. That is not an error and the tree treats
/// it as the loop exit -- PassageComplete asks the same question one level up.
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

/// Tell the aircraft a gate is done and ask it to confirm the field.
///
/// One publish, SUCCESS in the same tick. The WAITING is AwaitConfirmation, and
/// splitting the two is what keeps this one free.
///
/// WHAT IS BEING ASKED HAS CHANGED. This used to mean "assign me the next
/// pair". It now means "I am through a gate -- is the rest of the field still
/// where you said it was?". The aircraft answers by acking this sequence number
/// and retransmitting all ten buoys; the boat re-plans from that and drives on.
///
/// It advances the sequence only when the previous gate was actually cleared,
/// so a leg that failed re-asks under the SAME number and gets the aircraft's
/// idempotent repeat rather than burning a sequence.
class RequestConfirmation : public CrusaderSyncAction
{
public:
  RequestConfirmation(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    // FOR THE LOG ONLY, and it earns its place: the sequence number no longer
    // counts gates. The entry orbit asks first, so seq 1 is the entry and seq 2
    // is gate 1, and a log line reading "gate 1" at the entry sends whoever is
    // reading it looking for a gate the boat has not reached yet.
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

/// The boat is through this gate. Records it by the ids of its two buoys.
///
/// Last step of the leg, so anything that fails before it leaves the gate
/// un-cleared and the boat drives it again. Keyed on the BUOYS rather than on a
/// counter, so a confirmation that recolours the field and reshuffles the order
/// cannot make the boat skip one it never drove.
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
/// KEEPS ASKING, THEN GIVES UP AND CARRIES ON. Re-requesting every `retry_s`
/// covers a lost reply on a lossy radio; FAILURE at `timeout_s` hands the
/// decision back to the tree, which drives on with the last field it was given
/// rather than parking on the course forever. That is a deliberate trade and
/// the WARN is how it stays visible -- a boat that quietly proceeds on an
/// unconfirmed field looks identical to one that was confirmed.
///
/// NOTHING HOLDS STATION HERE and nothing needs to: the boat arrived at this
/// gate's `through` waypoint under GUIDED, the autopilot is still holding that
/// setpoint, and no leaf below is publishing a new one.
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
        "how often to re-send the request while waiting")};
  }

  BT::NodeStatus onStart() override
  {
    t0_ = std::chrono::steady_clock::now();
    last_ask_ = t0_;
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
      BT::InputPort<double>("standoff", 6.0, "metres past the middle"),
      BT::InputPort<double>("approach", 8.0, "metres short of the middle"),
      BT::InputPort<double>("offset", 4.0, "metres to clear a LONE buoy by"),
      BT::OutputPort<Waypoint>("out", "where the next action should drive")};
  }

  BT::NodeStatus tick() override
  {
    const std::string which = getInput<std::string>("point").value_or("through");
    const double standoff = getInput<double>("standoff").value_or(6.0);
    const double approach = getInput<double>("approach").value_or(8.0);
    const double offset = getInput<double>("offset").value_or(4.0);

    nav::Vec2 target;
    std::string why;
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->origin_set) {return BT::NodeStatus::FAILURE;}

    const int r = ctx_->gate_red_id;
    const int g = ctx_->gate_green_id;
    const nav::Gate gate = nav::gateFromIds(ctx_->buoys, r, g, standoff, approach);

    if (gate.valid) {
      target = (which == "approach") ? gate.approach : gate.through;
      why = "gate " + std::to_string(static_cast<int>(ctx_->gate_seq)) + " " +
        which + " (red " + std::to_string(r) + ", green " + std::to_string(g) + ")";
    } else {
      const nav::Buoy * lone = nav::findById(ctx_->buoys, r);
      if (lone == nullptr) {lone = nav::findById(ctx_->buoys, g);}
      if (lone == nullptr || !ctx_->have_exit) {
        RCLCPP_WARN(log(), "gate %u unusable: %s",
          static_cast<unsigned>(ctx_->gate_seq), gate.why);
        return BT::NodeStatus::FAILURE;
      }
      const nav::Vec2 travel = ctx_->exitp - ctx_->boat;
      target = nav::sideWaypoint(lone->p, travel, lone->state, offset);
      why = "lone buoy " + std::to_string(lone->id) + " (" + gate.why + ")";
      RCLCPP_WARN(log(), "gate %u: %s - steering past the one we have",
        static_cast<unsigned>(ctx_->gate_seq), gate.why);
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
}

}  // namespace crusader_bt
