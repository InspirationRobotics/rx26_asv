// task3_leaves.cpp — the Task 3 (Coordinated Logistics) leaves.
//
// Same rules as leaves.cpp:
//
//   * Every calculation is in dock_math.hpp, tested off-ROS
//     (test/test_dock_math.cpp). A formula growing here belongs there.
//   * The leaves never subscribe: the runner folds every DockObservation into
//     the Context through ingestDockObservation().
//   * Outputs (reports, the cannon, setpoints) are called OUTSIDE ctx_->mu: the
//     runner's publish callbacks may take it, and std::mutex does not recurse.
//
// The mission these serve (handbook 3.3.4), in tree order: find the safe bay, commit
// and dock, put the fire out, read the colour code and report the request.
// behavior_trees/task3_*.xml holds the tree.
//
// ONE CONTRACT THAT IS LOAD-BEARING: UpdateDockBook sits in the guard band,
// re-ticked ten times a second, and MUST NEVER RETURN FAILURE - "no bay in
// view this tick" is not a reason to end the mission.
#include <chrono>
#include <cmath>
#include <cstdio>
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

/// The GREEN bay's latest window reading for `index`, or nullptr. CALL UNDER mu.
const dock::WindowSighting * greenWindow(const Context & c, int index)
{
  if (index < 0) {return nullptr;}
  for (const auto & w : c.green.windows) {
    if (w.index == index) {return &w;}
  }
  return nullptr;
}

/// The GREEN bay's windows if the newest frame saw them and `seq` has not had that frame
/// yet (now it has), else nullptr. CALL UNDER mu. Context::green keeps its last windows
/// when the bay drops out of a frame, and feeding that stale state again would make a stale
/// GREEN look like one held for seconds.
const std::vector<dock::WindowSighting> * newGreenWindows(
  const Context & c, std::uint32_t & seq)
{
  if (c.dock_seq == seq || c.green.windows_t != c.dock_t) {return nullptr;}
  seq = c.dock_seq;
  return &c.green.windows;
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

/// Put camera_node on `model` (its live parameter, as the GCS's Sensors tab does) and
/// wait until it is on it. For "dock" that means a DockObservation newer than the ask:
/// the model running, not just the parameter accepted. Already on the dock model:
/// SUCCESS at once, nothing asked. FAILURE when camera_node is not up, refuses, or
/// timeout_s passes. A camera restart comes back on its YAML model (buoy), which is
/// blind to the dock: this is what stops a Task 3 run sliding past it.
class SetCameraModel : public CrusaderAction
{
public:
  SetCameraModel(const std::string & n, const BT::NodeConfig & c) : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("model", "dock", "none | buoy | dock"),
      BT::InputPort<double>("timeout_s", 15.0, "FAILURE when not on it after this")};
  }

  BT::NodeStatus onStart() override
  {
    model_ = getInput<std::string>("model").value_or("dock");
    bool there = false;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      t0_ = ctx_->now_s;
      seq0_ = ctx_->dock_seq;
      there = model_ == "dock" && ctx_->dock_obs_age_s <= kFreshS;
      ctx_->camera_model_ack = 0;
      ctx_->camera_model_why.clear();
    }
    if (there) {
      RCLCPP_INFO(log(), "camera already on the dock model");
      return BT::NodeStatus::SUCCESS;
    }
    std::string why = "bt_runner_node has no way to ask camera_node";
    if (!ctx_->set_camera_model || !ctx_->set_camera_model(model_, why)) {
      return fail(why);
    }
    RCLCPP_INFO(log(), "camera_node model -> %s", model_.c_str());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    double waited = 0.0;
    int ack = 0;
    bool there = false;
    std::string why;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      waited = ctx_->now_s - t0_;
      ack = ctx_->camera_model_ack;
      why = ctx_->camera_model_why;
      there = model_ == "dock" ?
        ctx_->dock_seq != seq0_ && ctx_->dock_obs_age_s <= kFreshS : ack > 0;
    }
    if (ack < 0) {
      return fail("camera_node refused the " + model_ + " model: " + why);
    }
    if (there) {
      RCLCPP_INFO(log(), "camera on the %s model after %.1f s", model_.c_str(), waited);
      return BT::NodeStatus::SUCCESS;
    }
    if (waited > getInput<double>("timeout_s").value_or(15.0)) {
      char buf[160];
      std::snprintf(buf, sizeof buf, "camera not on the %s model %.0f s after asking (%s)",
        model_.c_str(), waited, ack > 0 ? "accepted, no dock frames since" : "no answer");
      return fail(buf);
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  /// The reason goes in camera_model_why too: bt_runner_node's FAILURE detail, the GCS's
  /// Tree box, quotes it.
  BT::NodeStatus fail(const std::string & why)
  {
    RCLCPP_WARN(log(), "SetCameraModel: %s", why.c_str());
    std::lock_guard<std::mutex> lk(ctx_->mu);
    ctx_->camera_model_why = why;
    return BT::NodeStatus::FAILURE;
  }

  static constexpr double kFreshS = 1.0;
  std::string model_;
  double t0_ = 0.0;
  std::uint32_t seq0_ = 0;
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

/// In the berth: close enough in, on the centreline, bow in - AND the whole hull
/// inside the slip. The hull test is what makes this honest (centre-point tolerances
/// alone can pass a hull whose corner is over a finger, which a strict RoboCommand
/// will not call docked); the slip comes from the bay pitch the boat MEASURED
/// (dock::bayPitch) less half a finger (finger_margin).
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

// ------------------------------------------------------ world-model update

/// Says what the boat believes about the bays, when that belief changes. The
/// runner has already folded every frame in; this leaf SHOWS where the world
/// update is, and logs one line per change of verdict rather than one per frame.
/// ALWAYS SUCCESS (see the file header).
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

/// A bay is committed in this run (CommitSafeBay).
class BayChosen : public CrusaderCondition
{
public:
  BayChosen(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->chosen_bay > 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

/// Commit to the FIRST bay that reads GREEN with confidence, without waiting to see
/// every bay (the scan stops here): any confirmed track whose votes say GREEN
/// (dock::verdict, min_votes reads, min_share of them green, recent ones agreeing);
/// the most-seen if several. The bay NUMBER (DockingReport, bays numbered left to
/// right facing them) is the layout's when all the bays are known, else its place
/// among the bays seen so far, else 1, the last two logged as guesses. FAILURE until
/// a GREEN bay is in hand.
class CommitFirstGreen : public CrusaderSyncAction
{
public:
  CommitFirstGreen(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("min_votes", 8, "GREEN/RED reads before a bay has a verdict"),
      BT::InputPort<double>("min_share", 0.8, "share of them GREEN"),
      BT::InputPort<int>("min_obs", 5, "sightings before a track is a bay"),
      BT::InputPort<double>("max_turn_deg", 0.0,
        "> 0: not while the bow is further than this off the dock heading (still turning)"),
      BT::OutputPort<int>("bay", "the committed bay's number, 1-based left to right")};
  }

  BT::NodeStatus tick() override
  {
    dock::VoteParams v;
    const int min_obs = getInput<int>("min_obs").value_or(5);
    const double max_turn = getInput<double>("max_turn_deg").value_or(0.0);
    int bay = 0, seen = 0, red = 0, green = 0, id = -1;
    bool guess = false;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      v = ctx_->dock_votes;
      v.min_votes = getInput<int>("min_votes").value_or(v.min_votes);
      v.min_share = getInput<double>("min_share").value_or(v.min_share);
      // mid-turn the camera sweeps the dock: wait until the bow is on the dock heading
      const double dock_h = dockHeading(*ctx_, fire::kNaN);
      if (max_turn > 0.0 && std::isfinite(dock_h) &&
        !(std::fabs(fire::wrap180(dock_h - ctx_->heading_deg)) <= max_turn))
      {
        return BT::NodeStatus::FAILURE;
      }
      const dock::BayTrack * best = nullptr;
      for (const auto & t : ctx_->dock.tracks) {
        if (t.n < min_obs || dock::verdict(t, v) != dock::Verdict::Green ||
          !inView(*ctx_, t))
        {
          continue;                 // only a GREEN bay the camera sees NOW
        }
        if (best == nullptr || t.n > best->n) {best = &t;}
      }
      if (best == nullptr) {return BT::NodeStatus::FAILURE;}
      const dock::DockLayout L = dock::layout(ctx_->dock, ctx_->dock_min_obs);
      bay = L.ok ? L.numberOf(best->id) : 0;
      if (bay <= 0) {             // its place among the bays seen so far, left to right
        guess = true;
        const nav::Vec2 right = nav::starboardOf(best->outward() * -1.0);
        std::vector<double> along;   // the other confirmed bays, metres right of the GREEN one
        for (const auto & t : ctx_->dock.tracks) {
          if (t.n < ctx_->dock_min_obs || t.id == best->id) {continue;}
          const double d = nav::dot(t.p - best->p, right);
          bool same = std::fabs(d) < 0.6;
          for (double o : along) {same = same || std::fabs(o - d) < 0.6;}
          if (!same) {along.push_back(d);}
        }
        bay = 1;
        for (double d : along) {if (d < 0.0) {++bay;}}
        seen = static_cast<int>(along.size()) + 1;
      }
      ctx_->chosen_track = best->id;
      ctx_->chosen_bay = bay;
      ctx_->berth = dock::Berth{};
      ctx_->task3_phase = "DOCKING";
      id = best->id;
      red = best->red;
      green = best->green;
    }
    setOutput("bay", bay);
    if (guess) {
      RCLCPP_INFO(log(), "GREEN bay found: track %d (R%d/G%d), bay %d (guess: %d of %d bays seen)",
        id, red, green, bay, bay, seen);
    } else {
      RCLCPP_INFO(log(), "GREEN bay found: track %d (R%d/G%d), bay %d", id, red, green, bay);
    }
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
    return {
      BT::InputPort<bool>("need_others_red", false,
                          "the strict rule; the tree has already applied it"),
      BT::OutputPort<int>("bay", "the committed bay's number, 1-based left to right")};
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
    setOutput("bay", c.bay_number);
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
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("bay", "the bay docked in (CommitSafeBay's)")};
  }

  BT::NodeStatus tick() override
  {
    const int bay = getInput<int>("bay").value_or(0);
    if (bay <= 0) {return BT::NodeStatus::FAILURE;}
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->readiness_confirmed = false;
      ctx_->docking_report_s = ctx_->now_s;
      ctx_->task3_phase = "DOCKED";
    }
    sendReport(*ctx_, "docking", dock::dockingReportJson(bay));
    RCLCPP_INFO(log(), "REPORTED: docked in bay %d", bay);
    return BT::NodeStatus::SUCCESS;
  }
};

// ------------------------------------------------------------- firefighting

/// Wait for the fire: a window of OUR bay lit RED. Two ways to see it, the first wins:
///   * the windows' own states on the frames that saw our bay (dock::RedWatch), with
///     min_red_s >= 0: RED for min_red_s (a misread frame or two is not a fire), or for
///     refire_red_s once a fire is out this run (longer than the resource code's 1 s
///     RED flashes, which follow a hit). A window missing from a frame or two keeps its
///     run. This sees the light within a few frames, docked or not.
///   * the timing layer calling one window steady RED (the only way with min_red_s < 0).
/// `target` is the window to spray.
///
/// Re-sends the docking report every `resend_s` until RoboCommand confirms it, because a
/// lost report means no fire, ever - but only once ReportDocking has sent it: beside a
/// docking still under way, it must not report docking early.
class AwaitFireTarget : public CrusaderAction
{
public:
  AwaitFireTarget(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("bay", "the bay docked in, for re-sending the docking report"),
      BT::InputPort<double>("timeout_s", 60.0, "give up waiting for the light"),
      BT::InputPort<double>("resend_s", 10.0, "re-send the docking report this often"),
      BT::InputPort<double>("min_red_s", -1.0,
        ">= 0: fire on our bay's own window reading RED this long; < 0: the timing layer only"),
      BT::InputPort<double>("refire_red_s", 1.5,
        "... once a fire is out: this long (longer than the code's 1 s flashes)"),
      BT::OutputPort<int>("target", "DockWindow.index of the RED window (SprayUntilHit's)")};
  }

  BT::NodeStatus onStart() override
  {
    t0_ = Clock::now();
    red_.reset();
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      seq_ = ctx_->dock_seq;
      ctx_->task3_phase = "AWAIT FIRE";
    }
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    const int bay = getInput<int>("bay").value_or(0);
    const double min_red = getInput<double>("min_red_s").value_or(-1.0);
    bool resend = false;
    int target = -1;
    const char * how = "";
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const bool red = ctx_->dock_pattern == "steady" && ctx_->dock_colours.size() == 1 &&
        ctx_->dock_colours[0] == dock::Colour::Red;
      if (red && ctx_->dock_target_window >= 0) {
        target = ctx_->dock_target_window;
        how = "the timing layer: steady RED";
      }
      const auto * ws = target < 0 && min_red >= 0.0 ? newGreenWindows(*ctx_, seq_) : nullptr;
      if (ws != nullptr) {
        red_.update(*ws, ctx_->dock_t);
        const double need = ctx_->fires_out > 0 ?
          std::max(min_red, getInput<double>("refire_red_s").value_or(1.5)) : min_red;
        target = red_.lit(ctx_->dock_t, need);
        if (target >= 0) {how = "our bay's window reads RED";}
      }
      if (target < 0) {
        resend = bay > 0 && ctx_->docking_report_s >= 0.0 && !ctx_->readiness_confirmed &&
          ctx_->now_s - ctx_->docking_report_s >= getInput<double>("resend_s").value_or(10.0);
        if (resend) {ctx_->docking_report_s = ctx_->now_s;}
      }
    }
    if (target >= 0) {
      setOutput("target", target);
      RCLCPP_INFO(log(), "FIRE: window %d (%s)", target, how);
      return BT::NodeStatus::SUCCESS;
    }
    if (resend) {
      sendReport(*ctx_, "docking", dock::dockingReportJson(bay));
      RCLCPP_INFO(log(), "no light and no confirmation yet: re-sent docking report (bay %d)", bay);
    }
    if (since(t0_) >= getInput<double>("timeout_s").value_or(60.0)) {
      RCLCPP_WARN(log(), "AwaitFireTarget: no RED window after %.0fs", since(t0_));
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

private:
  Clock::time_point t0_;
  dock::RedWatch red_;
  std::uint32_t seq_ = 0;
};

/// Spray the target window until it turns GREEN.
///
/// The window is `target` (AwaitFireTarget's), else the timing layer's. Aimed EVERY TICK
/// at the CV's latest aim point for it, so the boat moving is followed. When the window
/// drops out (a glitch, a few lost frames, the other window seen alone) it keeps spraying
/// at the last aim point for max_aim_age_s - water on a miss does no harm, and stopping
/// on every lost frame wastes the shot - and past that holds fire until it is seen again.
/// With keep_after_docking, once the docking report is out it never holds fire for a lost
/// window: it sprays the last aim point until the window is seen again (and re-aims there)
/// or the fire is out.
/// If another window of our bay reads RED while ours has not for retarget_s (counted from
/// the trigger, which saw ours RED), and this spray never saw ours RED, it switches to that
/// one: the trigger was a misread.
///
/// Two independent "it's out" signals, either ends it: the timing layer's `hit` event
/// (counted by the runner, since it lives on one frame) and the window's own state
/// held GREEN for min_green_s (dock::HitWatch). The cannon is switched off on EVERY
/// exit (success, timeout, halt): a halted leaf leaving the pump running would keep
/// spraying through whatever the tree did next.
class SprayUntilHit : public CrusaderAction
{
public:
  SprayUntilHit(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("target", -1,
        "the window to spray (AwaitFireTarget's); -1 = the timing layer's"),
      BT::OutputPort<int>("window", "DockWindow.index of the window put out"),
      BT::InputPort<double>("timeout_s", 60.0, "give up"),
      BT::InputPort<double>("min_green_s", 0.4, "GREEN this long after RED is a hit"),
      BT::InputPort<double>("max_aim_age_s", 1.0,
        "keep spraying at the last aim point this long after the window was last seen"),
      BT::InputPort<bool>("keep_after_docking", false,
        "once the docking report is out, spray the last aim point however long it is lost"),
      BT::InputPort<double>("retarget_s", 2.0,
        "another window RED while ours has not been for this long: switch to it")};
  }

  BT::NodeStatus onStart() override
  {
    t0_ = Clock::now();
    watch_.reset();
    red_.reset();
    firing_ = held_ = false;
    aim_t_ = aim_seen_s_ = -1e18;
    const int target = getInput<int>("target").value_or(-1);
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      hits0_ = ctx_->dock_hits;
      window_ = target >= 0 ? target : ctx_->dock_target_window;
      seq_ = ctx_->dock_seq;
      red0_t_ = ctx_->dock_t;
      ctx_->task3_phase = "FIREFIGHTING";
    }
    if (window_ < 0) {return BT::NodeStatus::FAILURE;}
    return onRunning();                      // the window is lit now: aim on this tick
  }

  BT::NodeStatus onRunning() override
  {
    const double min_green = getInput<double>("min_green_s").value_or(0.4);
    const double max_age = getInput<double>("max_aim_age_s").value_or(1.0);
    const double retarget_s = getInput<double>("retarget_s").value_or(2.0);
    const bool keep = getInput<bool>("keep_after_docking").value_or(false);
    bool hit = false, aim_ok = false, held = false;
    int switched_from = -1;
    double x = 0, y = 0, z = 0, aim_age = 0.0;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (const auto * ws = newGreenWindows(*ctx_, seq_)) {
        red_.update(*ws, ctx_->dock_t);
        // ours was RED at the trigger (red0_t_), so it counts as RED from then on
        const int other = red_.lit(ctx_->dock_t, 0.0);
        const double ours = red_.lastRed(window_);
        const double ours_t = std::isfinite(ours) ? std::max(ours, red0_t_) : red0_t_;
        if (other >= 0 && other != window_ && !watch_.armed &&
          ctx_->dock_t - ours_t > retarget_s)
        {
          switched_from = window_;
          window_ = other;
          watch_.reset();
          aim_t_ = aim_seen_s_ = -1e18;
        }
        const dock::WindowSighting * w = greenWindow(*ctx_, window_);
        if (w != nullptr) {watch_.update(w->state, ctx_->dock_t, min_green);}
      }
      hit = watch_.hit || ctx_->dock_hits > hits0_;
      const dock::WindowSighting * w = greenWindow(*ctx_, window_);
      if (w != nullptr && w->has_position && ctx_->green.windows_t > aim_t_) {
        aim_t_ = ctx_->green.windows_t;        // a new aim point: keep it
        aim_seen_s_ = ctx_->now_s;
        ax_ = w->x;
        ay_ = w->y;
        az_ = w->z;
      }
      aim_age = ctx_->now_s - aim_seen_s_;
      held = keep && ctx_->docking_report_s >= 0.0 && aim_seen_s_ > -1e17 && aim_age > max_age;
      if (aim_age <= max_age || held) {
        aim_ok = true;
        x = ax_;
        y = ay_;
        z = az_;
      }
      if (hit) {++ctx_->fires_out;}
    }
    if (switched_from >= 0) {
      RCLCPP_WARN(log(), "window %d reads RED and window %d has not for %.0f s: spraying %d",
        window_, switched_from, retarget_s, window_);
    }
    if (hit) {
      setOutput("window", window_);
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
    if (held && !held_) {
      RCLCPP_INFO(log(), "window %d not seen for %.1f s: docked, so spraying on at its last "
        "aim point", window_, aim_age);
    }
    held_ = held;
    if (aim_ok) {
      if (ctx_->cannon) {ctx_->cannon(true, x, y, z);}
      if (!firing_) {
        RCLCPP_INFO(log(), "spraying window %d at (%.2f, %.2f, %.2f) camera_link",
          window_, x, y, z);
      }
      firing_ = true;
    } else if (firing_) {
      stop();
      RCLCPP_WARN(log(), "window %d not seen for %.1f s: holding fire until it is", window_,
        aim_age);
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
  dock::RedWatch red_;
  int hits0_ = 0;
  int window_ = -1;
  std::uint32_t seq_ = 0;
  bool firing_ = false, held_ = false;             // held_: spraying a lost window, docked
  double aim_t_ = -1e18, aim_seen_s_ = -1e18;     // the kept aim point's stamp, and now_s then
  double red0_t_ = 0.0;                            // dock_t at the trigger
  double ax_ = 0.0, ay_ = 0.0, az_ = 0.0;
};

/// FirefightingReport(window_id). window_id = DockWindow.index + base.
class ReportFirefighting : public CrusaderSyncAction
{
public:
  ReportFirefighting(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("window", "DockWindow.index put out (SprayUntilHit's)"),
      BT::InputPort<int>("window_id_base", 1,
        "RoboCommand's window_id for DockWindow.index 0 - CONFIRM with RoboNation")};
  }

  BT::NodeStatus tick() override
  {
    const int base = getInput<int>("window_id_base").value_or(1);
    const int w = getInput<int>("window").value_or(-1);
    if (w < 0) {return BT::NodeStatus::FAILURE;}
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->task3_phase = "FIRE OUT";
    }
    sendReport(*ctx_, "firefighting", dock::firefightingReportJson(w + base));
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
      BT::OutputPort<dock::Request>("request", "the decoded request"),
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
        if (done) {got = hold_.cand;}
      }
    }
    if (done) {
      setOutput("request", got);
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

/// ResourceDeliveryRequest to RoboCommand.
class ReportResourceDelivery : public CrusaderSyncAction
{
public:
  ReportResourceDelivery(const std::string & n, const BT::NodeConfig & c)
  : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<dock::Request>("request", "the decoded request (DecodeResourceRequest's)")};
  }

  BT::NodeStatus tick() override
  {
    const auto r = getInput<dock::Request>("request");
    if (!r || !r.value().ok) {return BT::NodeStatus::FAILURE;}
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->task3_phase = "REPORTED";
    }
    sendReport(*ctx_, "resource_delivery", dock::resourceRequestJson(r.value()));
    RCLCPP_INFO(log(), "REPORTED: %s", r.value().why.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace

void registerTask3Nodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<DockCameraAlive>("DockCameraAlive");
  factory.registerNodeType<SetCameraModel>("SetCameraModel");
  factory.registerNodeType<SafeBayKnown>("SafeBayKnown");
  factory.registerNodeType<ChosenBaySafe>("ChosenBaySafe");
  factory.registerNodeType<DockedInBay>("DockedInBay");
  factory.registerNodeType<LinedUp>("LinedUp");
  factory.registerNodeType<UpdateDockBook>("UpdateDockBook");
  factory.registerNodeType<PickVantage>("PickVantage");
  factory.registerNodeType<CommitSafeBay>("CommitSafeBay");
  factory.registerNodeType<CommitFirstGreen>("CommitFirstGreen");
  factory.registerNodeType<BayChosen>("BayChosen");
  factory.registerNodeType<DockWaypoint>("DockWaypoint");
  factory.registerNodeType<ReportDocking>("ReportDocking");
  factory.registerNodeType<AwaitFireTarget>("AwaitFireTarget");
  factory.registerNodeType<SprayUntilHit>("SprayUntilHit");
  factory.registerNodeType<ReportFirefighting>("ReportFirefighting");
  factory.registerNodeType<DecodeResourceRequest>("DecodeResourceRequest");
  factory.registerNodeType<ReportResourceDelivery>("ReportResourceDelivery");
}

}  // namespace crusader_bt
