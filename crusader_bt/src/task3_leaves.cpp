// task3_leaves.cpp — the Task 3 (Coordinated Logistics) leaves.
//
// Same rules as leaves.cpp, and for the same reasons:
//
//   * Every calculation is in dock_math.hpp, which is tested off-ROS
//     (test/test_dock_math.cpp). A formula growing here belongs there.
//   * The leaves never subscribe. The runner folds every DockObservation into
//     the Context through ingestDockObservation(), and a leaf reads the result.
//   * Outputs (reports, the cannon, setpoints) are called OUTSIDE ctx_->mu: the
//     runner's publish callbacks may take it, and std::mutex does not recurse.
//
// The mission these serve, handbook 3.3.4, in the order the tree runs them:
//
//   find the safe bay   PickVantage -> NavigateTo -> HoldStation, until
//                       SafeBayKnown: exactly one bay reads GREEN
//   commit and dock     CommitSafeBay, DockWaypoint predock/berth, guarded by
//                       ChosenBaySafe; DockedInBay; ReportDocking
//   put the fire out    AwaitFireTarget (the RED window), SprayUntilHit,
//                       ReportFirefighting
//   read the request    DecodeResourceRequest (the colour code), then
//                       ReportResourceRequest to RoboCommand and the UAV
//
// ONE CONTRACT THAT IS LOAD-BEARING: UpdateDockBook sits in the guard band,
// re-ticked ten times a second, and MUST NEVER RETURN FAILURE - "no bay in
// view this tick" is not a reason to end the mission.
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

#include "crusader_bt/context.hpp"
#include "crusader_bt/dock_math.hpp"
#include "crusader_bt/nav_math.hpp"

namespace crusader_bt
{
namespace
{

using nav::Vec2;
using Clock = std::chrono::steady_clock;

double since(Clock::time_point t0)
{
  return std::chrono::duration<double>(Clock::now() - t0).count();
}

/// The chosen bay's latest window reading for `index`, or nullptr. CALL UNDER mu.
const dock::WindowSighting * chosenWindow(const Context & c, int index)
{
  const dock::BayTrack * t = c.dock.find(c.chosen_track);
  if (t == nullptr || index < 0) {return nullptr;}
  for (const auto & w : t->windows) {
    if (w.index == index) {return &w;}
  }
  return nullptr;
}

// ---------------------------------------------------------------- conditions

/// Is the dock detector alive? It publishes EVERY frame, bays or not, so
/// silence means the node is dead rather than that the bays are out of view.
/// Blind, the boat must not keep driving at a dock.
class DockCameraAlive : public CrusaderCondition
{
public:
  DockCameraAlive(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("max_age_s", 3.0, "seconds without a DockObservation")};
  }

  BT::NodeStatus tick() override
  {
    const double max_age = getInput<double>("max_age_s").value_or(3.0);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->dock_obs_age_s <= max_age ? BT::NodeStatus::SUCCESS :
           BT::NodeStatus::FAILURE;
  }
};

/// Exactly one bay reads GREEN (see dock::chooseSafeBay). With
/// need_others_red, every other bay must also have read RED.
class SafeBayKnown : public CrusaderCondition
{
public:
  SafeBayKnown(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("need_others_red", true,
                                "also require every other bay to read RED")};
  }

  BT::NodeStatus tick() override
  {
    const bool strict = getInput<bool>("need_others_red").value_or(true);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const dock::DockLayout L = dock::layout(ctx_->dock, ctx_->dock_min_obs);
    return dock::chooseSafeBay(ctx_->dock, L, ctx_->dock_votes, strict).ok ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

/// Does the committed bay still read GREEN on RECENT evidence? Guards the
/// berthing legs: the indicator is read best close in, which is exactly when
/// the boat is committing to the approach.
class ChosenBaySafe : public CrusaderCondition
{
public:
  ChosenBaySafe(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("min_ema", 0.3, "recent GREEN share below which we abort")};
  }

  BT::NodeStatus tick() override
  {
    const double min_ema = getInput<double>("min_ema").value_or(0.3);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const dock::BayTrack * t = ctx_->dock.find(ctx_->chosen_track);
    return (t != nullptr && dock::stillSafe(*t, min_ema)) ? BT::NodeStatus::SUCCESS :
           BT::NodeStatus::FAILURE;
  }
};

/// In the berth: close enough in, on the centreline, bow in - AND the whole
/// hull inside the slip.
///
/// The hull test is what makes this honest. Centre-point tolerances alone
/// once passed a hull whose corner was over a finger: the tree reported
/// "docked", a strict RoboCommand said otherwise, and the fire never lit (sim,
/// 2026-09-24). The slip is taken from the bay pitch the boat MEASURED
/// (dock::bayPitch) less half a finger (finger_margin), not a number typed in
/// for a course nobody has measured.
class DockedInBay : public CrusaderCondition
{
public:
  DockedInBay(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("along_tol", 0.25, "metres either side of the berth depth"),
      BT::InputPort<double>("lateral_tol", 0.3, "metres off the centreline"),
      BT::InputPort<double>("heading_tol_deg", 15.0, "bow off straight-in"),
      BT::InputPort<double>("hull_length", 1.0, "m, the boat (~1.0 x 0.6)"),
      BT::InputPort<double>("hull_beam", 0.6, "m, the boat"),
      BT::InputPort<double>("finger_margin", 0.35,
        "m in from the bay pitch's edge: half a 0.5 m finger, plus 0.1 m clear")};
  }

  BT::NodeStatus tick() override
  {
    const double at = getInput<double>("along_tol").value_or(0.25);
    const double lt = getInput<double>("lateral_tol").value_or(0.3);
    const double ht = getInput<double>("heading_tol_deg").value_or(15.0);
    const double hl = getInput<double>("hull_length").value_or(1.0);
    const double hb = getInput<double>("hull_beam").value_or(0.6);
    const double fm = getInput<double>("finger_margin").value_or(0.35);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->pose_fresh || !ctx_->berth.ok) {return BT::NodeStatus::FAILURE;}
    const dock::DockedCheck d = dock::dockedIn(
      ctx_->berth, ctx_->boat, ctx_->heading_deg, ctx_->berth.berth_m, at, lt, ht);
    if (!d.docked) {return BT::NodeStatus::FAILURE;}
    const double pitch = dock::bayPitch(ctx_->dock, dock::layout(ctx_->dock, ctx_->dock_min_obs));
    if (!std::isfinite(pitch)) {return BT::NodeStatus::SUCCESS;}   // nothing to test against
    const double reach = dock::hullHalfWidthUsed(ctx_->berth, ctx_->boat, ctx_->heading_deg, hl, hb);
    if (reach > pitch / 2.0 - fm) {
      // Throttled: the tree polls this while waiting for the boat to settle.
      RCLCPP_WARN_THROTTLE(log(), *ctx_->node->get_clock(), 2000,
        "not docked yet: hull reaches %.2f m off the centreline of a %.2f m slip (%s)",
        reach, pitch, d.why.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

/// Already on the committed bay's centreline, outside the line-up point, bow
/// in (dock::linedUp). The tree skips the lead-in leg when this holds.
class LinedUp : public CrusaderCondition
{
public:
  LinedUp(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_along_m", 2.7, "at least this far out from the face"),
      BT::InputPort<double>("lateral_tol", 0.4, "metres off the centreline"),
      BT::InputPort<double>("heading_tol_deg", 20.0, "bow off straight-in")};
  }

  BT::NodeStatus tick() override
  {
    const double a = getInput<double>("min_along_m").value_or(2.7);
    const double lt = getInput<double>("lateral_tol").value_or(0.4);
    const double ht = getInput<double>("heading_tol_deg").value_or(20.0);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const dock::BayTrack * t = ctx_->dock.find(ctx_->chosen_track);
    if (t == nullptr || !ctx_->pose_fresh) {return BT::NodeStatus::FAILURE;}
    // Only the face and its normal matter here; the distances are placeholders.
    const dock::Berth b = dock::berthFor(
      *t, dock::layout(ctx_->dock, ctx_->dock_min_obs), 3.0, 1.25);
    return dock::linedUp(b, ctx_->boat, ctx_->heading_deg, a, lt, ht) ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

/// The goal's tier is at least `tier` (0 Core, 1 Advanced, 2 Disruptive), so
/// one tree serves all three and Core simply stops after the fire is out.
class TierAtLeast : public CrusaderCondition
{
public:
  TierAtLeast(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("tier", 1, "0 Core, 1 Advanced, 2 Disruptive")};
  }

  BT::NodeStatus tick() override
  {
    const int want = getInput<int>("tier").value_or(1);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->tier >= want ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// ------------------------------------------------------ world-model update

/// Says what the boat believes about the bays, when that belief changes.
///
/// The runner has already folded every frame in; this leaf exists so the tree
/// SHOWS where the world update is, and so the log carries one line per
/// change of verdict rather than one per frame.
///
/// ALWAYS SUCCESS. See the file header.
class UpdateDockBook : public CrusaderSyncAction
{
public:
  UpdateDockBook(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::string key, detail;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const dock::DockLayout L = dock::layout(ctx_->dock, ctx_->dock_min_obs);
      if (L.ok) {
        for (std::size_t i = 0; i < L.ids.size(); ++i) {
          const dock::BayTrack * t = ctx_->dock.find(L.ids[i]);
          key += std::to_string(i + 1) + ":" +
            dock::verdictChar(dock::verdict(*t, ctx_->dock_votes)) + " ";
        }
        const dock::Choice c = dock::chooseSafeBay(ctx_->dock, L, ctx_->dock_votes, true);
        key += c.ok ? "safe=" + std::to_string(c.bay_number) : "safe=?";
        detail = c.why;
      } else {
        key = std::to_string(ctx_->dock.tracks.size()) + " tracks";
        detail = L.why;
      }
    }
    if (key != last_) {
      last_ = key;
      RCLCPP_INFO(log(), "dock: %s  (%s)", key.c_str(), detail.c_str());
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::string last_;
};

// ------------------------------------------------------------ finding the bay

/// Where to look from next: see dock::vantage. Each call is one more attempt,
/// so under a RetryUntilSuccessful the vantage moves on every retry.
class PickVantage : public CrusaderSyncAction
{
public:
  PickVantage(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("standoff", 5.0,
        "m in front of the faces; MUST clear the fingers: finger length + half a hull"),
      BT::InputPort<double>("lead_m", 2.0, "drive here first, further out, to arrive facing in"),
      BT::OutputPort<Waypoint>("out", "where to look from"),
      BT::OutputPort<Waypoint>("lead", "where to drive first")};
  }

  BT::NodeStatus tick() override
  {
    const double standoff = getInput<double>("standoff").value_or(5.0);
    const double lead_m = getInput<double>("lead_m").value_or(2.0);
    Waypoint w, lead;
    int attempt = 0;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!ctx_->origin_set) {return BT::NodeStatus::FAILURE;}
      attempt = ctx_->survey_attempt++;
      const Vec2 a = ctx_->has_approach ?
        nav::toLocal({ctx_->approach_lat, ctx_->approach_lon}, ctx_->origin) : Vec2{};
      const int blind = attempt - ctx_->survey_looks;     // looks that saw nothing
      const dock::Vantage v = dock::vantage(
        ctx_->dock, ctx_->has_approach, a, blind, ctx_->survey_looks, standoff,
        ctx_->dock_min_obs, lead_m);
      if (!v.ok) {
        RCLCPP_WARN(log(), "PickVantage: %s", v.why.c_str());
        return BT::NodeStatus::FAILURE;
      }
      if (v.from_bays) {++ctx_->survey_looks;}
      const nav::LatLon ll = nav::toLatLon(v.p, ctx_->origin);
      const nav::LatLon ld = nav::toLatLon(v.lead, ctx_->origin);
      w = Waypoint{ll.lat, ll.lon, v.why};
      lead = Waypoint{ld.lat, ld.lon, "lead-in: " + v.why};
      ctx_->task3_phase = "SURVEY";
    }
    setOutput("out", w);
    setOutput("lead", lead);
    RCLCPP_INFO(log(), "survey look %d: %s", attempt + 1, w.why.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

/// Commit to the one GREEN bay. From here on the berthing legs steer to THIS
/// bay even if a later frame's votes wobble; ChosenBaySafe is what can undo it.
class CommitSafeBay : public CrusaderSyncAction
{
public:
  CommitSafeBay(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("need_others_red", false,
                                "the strict rule; the tree has already applied it")};
  }

  BT::NodeStatus tick() override
  {
    const bool strict = getInput<bool>("need_others_red").value_or(false);
    dock::Choice c;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const dock::DockLayout L = dock::layout(ctx_->dock, ctx_->dock_min_obs);
      c = dock::chooseSafeBay(ctx_->dock, L, ctx_->dock_votes, strict);
      if (c.ok) {
        ctx_->chosen_track = c.track_id;
        ctx_->chosen_bay = c.bay_number;
        ctx_->berth = dock::Berth{};
        ctx_->task3_phase = "DOCKING";
      }
    }
    if (!c.ok) {
      RCLCPP_WARN(log(), "CommitSafeBay: %s", c.why.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(log(), "committed to bay %d  [%s]", c.bay_number, c.why.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

// ------------------------------------------------------------------ docking

/// The line-up point or the berth for the committed bay, recomputed from the
/// bay's CURRENT estimate on every tick. Inside a ReactiveSequence with
/// NavigateTo, the berth follows the estimate as close-range sightings refine
/// it - which is where the estimate gets good.
class DockWaypoint : public CrusaderSyncAction
{
public:
  DockWaypoint(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("point", "berth", "lead | predock | berth"),
      BT::InputPort<double>("predock_m", 3.0, "line-up point, m out from the face"),
      BT::InputPort<double>("berth_m", 1.25,
        "face to BODY ORIGIN when docked: bow offset plus clearance"),
      BT::InputPort<double>("lead_m", 2.0, "lead-in, m beyond the line-up point"),
      BT::OutputPort<Waypoint>("out", "where the next action should drive")};
  }

  BT::NodeStatus tick() override
  {
    const std::string which = getInput<std::string>("point").value_or("berth");
    const double pre = getInput<double>("predock_m").value_or(3.0);
    const double bm = getInput<double>("berth_m").value_or(1.25);
    const double lead = getInput<double>("lead_m").value_or(2.0);
    Waypoint w;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const dock::BayTrack * t = ctx_->dock.find(ctx_->chosen_track);
      if (t == nullptr || !ctx_->origin_set) {return BT::NodeStatus::FAILURE;}
      const dock::DockLayout L = dock::layout(ctx_->dock, ctx_->dock_min_obs);
      ctx_->berth = dock::berthFor(*t, L, pre, bm, lead);
      if (!ctx_->berth.ok) {return BT::NodeStatus::FAILURE;}
      const Vec2 p = which == "lead" ? ctx_->berth.lead :
        (which == "predock" ? ctx_->berth.predock : ctx_->berth.berth);
      const nav::LatLon ll = nav::toLatLon(p, ctx_->origin);
      w = Waypoint{ll.lat, ll.lon, "bay " + std::to_string(ctx_->chosen_bay) + " " + which};
    }
    setOutput("out", w);
    return BT::NodeStatus::SUCCESS;
  }
};

/// DockingReport(bay_id). RoboCommand activates the fire on this.
class ReportDocking : public CrusaderSyncAction
{
public:
  ReportDocking(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    int bay = 0;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      bay = ctx_->chosen_bay;
      ctx_->readiness_confirmed = false;
      ctx_->task3_phase = "DOCKED";
    }
    if (bay <= 0) {return BT::NodeStatus::FAILURE;}
    if (ctx_->report_docking) {ctx_->report_docking(bay);}
    RCLCPP_INFO(log(), "REPORTED: docked in bay %d", bay);
    return BT::NodeStatus::SUCCESS;
  }
};

// ------------------------------------------------------------- firefighting

/// Wait for the fire: the timing layer calling one window of our bay steady
/// RED. Re-sends the docking report every `resend_s` until RoboCommand
/// confirms it, because a lost report means no fire, ever.
///
/// NOTHING HOLDS STATION HERE and nothing needs to: the berth is the last
/// setpoint, and GUIDED holds it.
class AwaitFireTarget : public CrusaderAction
{
public:
  AwaitFireTarget(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout_s", 60.0, "give up waiting for the light"),
      BT::InputPort<double>("resend_s", 10.0, "re-send the docking report this often")};
  }

  BT::NodeStatus onStart() override
  {
    t0_ = Clock::now();
    last_ = t0_;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->task3_phase = "AWAIT FIRE";
    }
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    int bay = 0;
    bool resend = false;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const bool red = ctx_->dock_pattern == "steady" && ctx_->dock_colours.size() == 1 &&
        ctx_->dock_colours[0] == dock::Colour::Red;
      if (red && ctx_->dock_target_window >= 0) {
        RCLCPP_INFO(log(), "FIRE: window %d is steady RED", ctx_->dock_target_window);
        return BT::NodeStatus::SUCCESS;
      }
      bay = ctx_->chosen_bay;
      resend = !ctx_->readiness_confirmed &&
        since(last_) >= getInput<double>("resend_s").value_or(10.0);
    }
    if (resend) {
      last_ = Clock::now();
      if (ctx_->report_docking) {ctx_->report_docking(bay);}
      RCLCPP_INFO(log(), "no light and no confirmation yet: re-sent docking report (bay %d)", bay);
    }
    if (since(t0_) >= getInput<double>("timeout_s").value_or(60.0)) {
      RCLCPP_WARN(log(), "AwaitFireTarget: no RED window after %.0fs", since(t0_));
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

private:
  Clock::time_point t0_, last_;
};

/// Spray the target window until it turns GREEN.
///
/// Aimed EVERY TICK at the CV's latest aim point for that window, so drift in
/// the berth is followed rather than sprayed past. With no fresh aim point it
/// STOPS spraying rather than spraying where the window used to be.
///
/// Two independent "it's out" signals, either one ends it: the timing layer's
/// `hit` event (counted by the runner, since it lives on one frame), and the
/// window's own state held GREEN for min_green_s (dock::HitWatch).
///
/// The cannon is switched off on EVERY exit - success, timeout and halt. A
/// halted leaf that left the pump running would keep spraying through
/// whatever the tree did next.
class SprayUntilHit : public CrusaderAction
{
public:
  SprayUntilHit(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout_s", 60.0, "give up"),
      BT::InputPort<double>("min_green_s", 0.4, "GREEN this long after RED is a hit"),
      BT::InputPort<double>("max_aim_age_s", 1.0, "do not fire at an older aim point")};
  }

  BT::NodeStatus onStart() override
  {
    t0_ = Clock::now();
    watch_.reset();
    firing_ = false;
    std::lock_guard<std::mutex> lk(ctx_->mu);
    hits0_ = ctx_->dock_hits;
    window_ = ctx_->dock_target_window;
    seq_ = ctx_->dock_seq;
    ctx_->task3_phase = "FIREFIGHTING";
    if (window_ < 0) {return BT::NodeStatus::FAILURE;}
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const double min_green = getInput<double>("min_green_s").value_or(0.4);
    const double max_age = getInput<double>("max_aim_age_s").value_or(1.0);
    bool hit = false, aim_ok = false;
    double x = 0, y = 0, z = 0;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const dock::WindowSighting * w = chosenWindow(*ctx_, window_);
      const dock::BayTrack * t = ctx_->dock.find(ctx_->chosen_track);
      // A NEW frame in which OUR bay was seen. The track keeps its last
      // windows when the bay drops out of a frame, and feeding that stale
      // state again would make a stale GREEN look like one held for seconds.
      if (w != nullptr && ctx_->dock_seq != seq_ && t->windows_t == ctx_->dock_t) {
        seq_ = ctx_->dock_seq;
        watch_.update(w->state, ctx_->dock_t, min_green);
      }
      hit = watch_.hit || ctx_->dock_hits > hits0_;
      if (w != nullptr && t != nullptr && w->has_position &&
        ctx_->dock_t - t->windows_t <= max_age)
      {
        aim_ok = true;
        x = w->x;
        y = w->y;
        z = w->z;
      }
      if (hit) {ctx_->fired_window = window_;}
    }
    if (hit) {
      stop();
      RCLCPP_INFO(log(), "HIT: window %d is GREEN after %.1fs of spray", window_, since(t0_));
      return BT::NodeStatus::SUCCESS;
    }
    if (since(t0_) >= getInput<double>("timeout_s").value_or(60.0)) {
      stop();
      RCLCPP_WARN(log(), "SprayUntilHit: window %d still not GREEN after %.0fs",
        window_, since(t0_));
      return BT::NodeStatus::FAILURE;
    }
    if (aim_ok) {
      if (ctx_->cannon) {ctx_->cannon(true, x, y, z);}
      if (!firing_) {
        RCLCPP_INFO(log(), "spraying window %d at (%.2f, %.2f, %.2f) camera_link",
          window_, x, y, z);
      }
      firing_ = true;
    } else if (firing_) {
      stop();
      RCLCPP_WARN(log(), "lost the aim point for window %d: holding fire", window_);
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {stop();}

private:
  void stop()
  {
    if (ctx_->cannon) {ctx_->cannon(false, 0.0, 0.0, 0.0);}
    firing_ = false;
  }

  Clock::time_point t0_;
  dock::HitWatch watch_;
  int hits0_ = 0;
  int window_ = -1;
  std::uint32_t seq_ = 0;
  bool firing_ = false;
};

/// FirefightingReport(window_id). window_id = DockWindow.index + base.
class ReportFirefighting : public CrusaderSyncAction
{
public:
  ReportFirefighting(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("window_id_base", 1,
        "RoboCommand's window_id for DockWindow.index 0 - CONFIRM with RoboNation")};
  }

  BT::NodeStatus tick() override
  {
    const int base = getInput<int>("window_id_base").value_or(1);
    int w = -1;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      w = ctx_->fired_window;
      ctx_->task3_phase = "FIRE OUT";
    }
    if (w < 0) {return BT::NodeStatus::FAILURE;}
    if (ctx_->report_firefighting) {ctx_->report_firefighting(w + base);}
    RCLCPP_INFO(log(), "REPORTED: fire out in window_id %d (index %d)", w + base, w);
    return BT::NodeStatus::SUCCESS;
  }
};

// ------------------------------------------------------- the resource request

/// Read the colour code off the target window, and hold it still.
///
/// The code starts GREEN 5 s + off 1 s after the hit and repeats for 60 s. The
/// timing layer needs two full cycles (~10 s) before it says "code"; this then
/// wants the SAME answer for hold_s more (dock::RequestHold) before anything is
/// reported, because a report cannot be taken back.
class DecodeResourceRequest : public CrusaderAction
{
public:
  DecodeResourceRequest(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout_s", 80.0, "the code runs for 60 s after the hit"),
      BT::InputPort<double>("hold_s", 5.0, "the same answer for this long"),
      BT::InputPort<int>("min_frames", 20, "and across this many new frames")};
  }

  BT::NodeStatus onStart() override
  {
    t0_ = Clock::now();
    hold_.reset();
    std::lock_guard<std::mutex> lk(ctx_->mu);
    seq_ = ctx_->dock_seq;
    ctx_->have_request = false;
    ctx_->task3_phase = "DECODING";
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const double hold = getInput<double>("hold_s").value_or(5.0);
    const int frames = getInput<int>("min_frames").value_or(20);
    dock::Request got;
    bool done = false;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      // Below ~4 fps a 1 s on / 1 s off light cannot be resolved (Nyquist);
      // the timing layer says "unresolved" and this will time out. Say WHY
      // while it is happening rather than leave a bare timeout to explain.
      if (ctx_->dock_fps > 0.0 && ctx_->dock_fps < 4.0) {
        RCLCPP_WARN_THROTTLE(log(), *ctx_->node->get_clock(), 5000,
          "camera at %.1f fps: the code cannot be read below 4", ctx_->dock_fps);
      }
      if (ctx_->dock_seq != seq_) {
        seq_ = ctx_->dock_seq;
        const dock::Request r = dock::requestFrom(ctx_->dock_pattern, ctx_->dock_colours);
        done = hold_.update(r, ctx_->dock_t, hold, frames);
        if (done) {
          ctx_->request = hold_.cand;
          ctx_->have_request = true;
          got = hold_.cand;
        }
      }
    }
    if (done) {
      RCLCPP_INFO(log(), "decoded: %s (held %d frames)", got.why.c_str(), hold_.frames);
      return BT::NodeStatus::SUCCESS;
    }
    if (since(t0_) >= getInput<double>("timeout_s").value_or(80.0)) {
      RCLCPP_WARN(log(), "DecodeResourceRequest: no steady code after %.0fs (last: %s)",
        since(t0_), hold_.cand.ok ? hold_.cand.why.c_str() : "nothing");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

private:
  Clock::time_point t0_;
  dock::RequestHold hold_;
  std::uint32_t seq_ = 0;
};

/// ResourceDeliveryRequest to RoboCommand, and the same request to the UAV.
class ReportResourceRequest : public CrusaderSyncAction
{
public:
  ReportResourceRequest(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    dock::Request r;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!ctx_->have_request) {return BT::NodeStatus::FAILURE;}
      r = ctx_->request;
      ctx_->task3_phase = "REPORTED";
    }
    if (ctx_->report_request) {ctx_->report_request(r);}
    if (ctx_->relay_request) {ctx_->relay_request(r);}
    RCLCPP_INFO(log(), "REPORTED + RELAYED: %s", r.why.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace

void registerTask3Nodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<DockCameraAlive>("DockCameraAlive");
  factory.registerNodeType<SafeBayKnown>("SafeBayKnown");
  factory.registerNodeType<ChosenBaySafe>("ChosenBaySafe");
  factory.registerNodeType<DockedInBay>("DockedInBay");
  factory.registerNodeType<LinedUp>("LinedUp");
  factory.registerNodeType<TierAtLeast>("TierAtLeast");
  factory.registerNodeType<UpdateDockBook>("UpdateDockBook");
  factory.registerNodeType<PickVantage>("PickVantage");
  factory.registerNodeType<CommitSafeBay>("CommitSafeBay");
  factory.registerNodeType<DockWaypoint>("DockWaypoint");
  factory.registerNodeType<ReportDocking>("ReportDocking");
  factory.registerNodeType<AwaitFireTarget>("AwaitFireTarget");
  factory.registerNodeType<SprayUntilHit>("SprayUntilHit");
  factory.registerNodeType<ReportFirefighting>("ReportFirefighting");
  factory.registerNodeType<DecodeResourceRequest>("DecodeResourceRequest");
  factory.registerNodeType<ReportResourceRequest>("ReportResourceRequest");
}

}  // namespace crusader_bt
