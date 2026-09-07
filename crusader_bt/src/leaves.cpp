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
      BT::InputPort<double>("tolerance", 2.0, "arrival radius, metres")};
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
}

}  // namespace crusader_bt
