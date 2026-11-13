// global_leaves.cpp — the leaves of the whole-field Task 1 trees (behavior_trees/lib/common/task1_plan_drive.xml).
//
// Nothing here is geometry: the plan is global_passage.hpp and the driving path_follower.hpp,
// both pinned off the boat by test/test_global_passage.cpp (closed loop included).
//
// THE MODEL. One plan covers the rest of the mission from the boat: approach, entry orbit
// (clockwise), transit (every red to starboard, every green to port), exit orbit (counter-
// clockwise); ctx.global_passage.phase says which is being driven. A plan is KEPT while good
// and re-made from the boat when:
//   * there is none, or the aircraft's field changed (a re-task);
//   * (Core tier, no aircraft) the boat's own camera/LiDAR field changed IN A WAY THAT MATTERS: a
//     red/green would now be passed on the wrong side, or the ENTRY/EXIT moved before its orbit.
//     Settled for sensor_settle_s, at most one per sensor_replan_min_s; a change that does not
//     matter is ABSORBED (the plan stays), so the boat does not lurch at every tracker flicker;
//   * the boat is not near the start of what is left of the leg it is about to drive;
//   * a hazard now sits closer to what is left than the plan allowed (fusion moved a buoy,
//     or a LiDAR/camera track appeared on it).
// That is the whole re-tasking mechanism.
//
// NO NAV2: the boat follows its own plan with GUIDED setpoints, so these behave the same in
// every nav_mode.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

#include "crusader_bt/context.hpp"
#include "crusader_bt/global_passage.hpp"
#include "crusader_bt/hold_heading.hpp"
#include "crusader_bt/path_follower.hpp"

namespace crusader_bt
{
namespace
{

using nav::Vec2;
using gp::Phase;

constexpr double kInf = std::numeric_limits<double>::infinity();

/// Everything the plan keeps clear of: knownHazards() without side fences, which belong to
/// the per-gate tree's Nav2 legs (the parity holds sides here). CALL UNDER ctx.mu.
std::vector<path::Hazard> planHazards(const Context & c)
{
  std::vector<path::Hazard> hz = knownHazards(c);
  hz.erase(std::remove_if(hz.begin(), hz.end(), [](const path::Hazard & h) {
      return h.source == path::HazardSource::Fence;
    }), hz.end());
  return hz;
}

/// What `phase`'s leg is checked against: planHazards(), except that an ORBIT is not stopped by
/// a track too vague to trust (gp::weakTrack) - the planner fitted the rings without it
/// (gp::orbitHazards). The approach and the transit keep clear of every track. CALL UNDER ctx.mu.
std::vector<path::Hazard> legHazards(const Context & c, Phase phase)
{
  std::vector<path::Hazard> hz = planHazards(c);
  if (!gp::isOrbit(phase)) {return hz;}
  return gp::orbitHazards(hz, c.buoys, c.global_passage.params.weak_sd_m);
}

/// Enough to plan with: ENTRY, EXIT, a frame and a boat. CALL UNDER ctx.mu.
bool fieldKnown(const Context & c)
{
  return c.origin_set && c.have_entry && c.have_exit && c.pose_fresh &&
         std::isfinite(c.boat.x) && std::isfinite(c.boat.y);
}

/// What is left of `phase`'s leg: from the last checkpoint stop for the transit, all of it
/// otherwise. CALL UNDER ctx.mu.
std::vector<Vec2> legLeft(const Context & c, Phase phase)
{
  const Context::GlobalPassage & g = c.global_passage;
  const std::vector<Vec2> & leg = g.plan.legs[static_cast<int>(phase)];
  if (phase != Phase::Transit || leg.size() < 2) {return leg;}
  return gp::slice(leg, g.transit_s, gp::arcLengths(leg).back());
}

/// No aircraft report: the boat's own tracker is the only source of buoys (Core tier). The same
/// test bt_runner_node's refuseLocked() uses. CALL UNDER ctx.mu.
bool coreMode(const Context & c) {return c.plan.empty();}

/// The latest report, as a plan records it: the aircraft's, or in Core tier the tracker's
/// confirmed RED and GREEN buoys (id, position, colour) and the ENTRY/EXIT it chose.
/// CALL UNDER ctx.mu.
gp::FieldSig currentField(const Context & c)
{
  if (!coreMode(c)) {return gp::fieldSig(c.plan, c.entry, c.exitp);}
  std::vector<nav::PlanBuoy> seen;
  for (const nav::Buoy & b : c.buoys) {
    if (nav::isSideConstrained(b.state) && std::isfinite(b.p.x) && std::isfinite(b.p.y)) {
      seen.push_back({b.id, b.p, b.state});
    }
  }
  // The ENTRY/EXIT as the planner anchors them (gp::detail::anchorOf), not the raw points: a
  // buoy first seen near a MANUAL or remembered point moves the orbit onto it, and that has to
  // reach the sensor check even though the buoy has no colour yet.
  const double snap = c.global_passage.params.anchor_snap_m;
  return gp::fieldSig(std::move(seen),
      gp::detail::anchorOf(c.buoys, c.entry, nav::Beacon::FlashingBlue, snap).first,
      gp::detail::anchorOf(c.buoys, c.exitp, nav::Beacon::SteadyBlue, snap).first);
}

/// What changed in the field since the plan was made ("" = nothing): colours, buoys, or a move
/// beyond retask_move_m (the aircraft's scatter) / sensor_move_m (the tracker's). RAW: in Core
/// tier it says nothing about whether the plan still serves (sensorNeedsReplan). CALL UNDER ctx.mu.
std::string retasked(const Context & c)
{
  const Context::GlobalPassage & g = c.global_passage;
  if (!g.plan.ok) {return "";}
  const double move = coreMode(c) ? g.params.sensor_move_m : g.params.retask_move_m;
  return gp::fieldChange(g.field, currentField(c), move);
}

/// The path the plan is judged on: the transit leg before the transit starts; the track driven
/// plus what is left of the leg during it. CALL UNDER ctx.mu.
std::vector<Vec2> judgedPath(const Context & c, Phase phase)
{
  const Context::GlobalPassage & g = c.global_passage;
  if (phase <= Phase::EntryOrbit) {return g.plan.legs[static_cast<int>(Phase::Transit)];}
  if (phase != Phase::Transit) {return {};}
  std::vector<Vec2> judged = g.traj;
  judged.push_back(c.boat);
  const std::vector<Vec2> left = legLeft(c, Phase::Transit);
  judged.insert(judged.end(), left.begin(), left.end());
  return judged;
}

/// Core tier: "" when the raw change in the boat's own field does not matter for the plan in
/// hand, in which case it is ABSORBED (g.field := now, so it stops reporting) and logged once;
/// otherwise why it does matter (gp::sensorChangeMatters). CALL UNDER ctx.mu.
std::string sensorNeedsReplan(
  Context & c, Phase phase, const rclcpp::Logger & lg, const std::string & raw)
{
  Context::GlobalPassage & g = c.global_passage;
  const std::vector<Vec2> & ring = g.plan.entry_ring.empty() ? g.entry_ring : g.plan.entry_ring;
  const std::string why = gp::sensorChangeMatters(g.plan, phase, c.buoys, c.entry, c.exitp, ring,
      judgedPath(c, phase), g.params.sensor_move_m, g.params.anchor_snap_m);
  if (!why.empty()) {return why;}
  g.field = currentField(c);
  RCLCPP_INFO(lg, "the boat's own field changed (%s); the plan in hand still serves: kept",
    raw.c_str());
  return "";
}

/// "" when the plan in hand can still drive `phase`; otherwise why not. Core tier replans for a
/// field change only when it matters (and absorbs it otherwise). CALL UNDER ctx.mu.
std::string planUnusable(Context & c, Phase phase, const rclcpp::Logger & lg)
{
  const Context::GlobalPassage & g = c.global_passage;
  if (!g.plan.ok) {return "no plan yet";}
  const std::string change = retasked(c);
  if (!change.empty()) {
    if (!coreMode(c)) {return "the aircraft's field changed: " + change;}
    const std::string why = sensorNeedsReplan(c, phase, lg, change);
    if (!why.empty()) {return "the boat's own field: " + why;}
  }
  const std::vector<Vec2> left = legLeft(c, phase);
  if (left.empty()) {return "the plan has no " + std::string(gp::phaseName(phase)) + " leg";}
  // near the start of what is left (a pass-through hand-over ends ~2 m short of it)
  const std::vector<Vec2> head = gp::slice(left, 0.0, 3.0);
  double d = kInf;
  for (const Vec2 & q : head) {d = std::min(d, nav::norm(q - c.boat));}
  if (d > 2.5) {return "the boat is " + gp::detail::fmt(d, 1) + " m off the plan";}
  const double clear = gp::polylineClearance(left, legHazards(c, phase));
  if (clear < g.plan.hard_used - 0.05) {
    return "a hazard is now " + gp::detail::fmt(clear, 2) + " m from the plan";
  }
  return "";
}

/// Plan the mission from the boat, phase `from` on, and adopt it if it worked. The search
/// (tens to hundreds of ms) runs WITHOUT the lock; inputs and result cross under it.
bool replan(Context & c, Phase from, const rclcpp::Logger & lg, const std::string & why)
{
  gp::Request rq;
  gp::Params p;
  gp::FieldSig field;
  {
    std::lock_guard<std::mutex> lk(c.mu);
    if (!fieldKnown(c)) {return false;}
    const Context::GlobalPassage & g = c.global_passage;
    rq.from = from;
    rq.boat = c.boat;
    rq.buoys = c.buoys;
    rq.hazards = planHazards(c);
    rq.entry = c.entry;
    rq.exitp = c.exitp;
    rq.exit_guessed = c.exit_guessed;
    rq.cleared = c.cleared_gates;
    if (from >= Phase::Transit) {
      rq.entry_ring_done = g.entry_ring;
      rq.traj = g.traj;
      for (const gp::Ray & r : g.plan.rays) {rq.prev_rays.push_back({r.id, r.red});}
    }
    p = g.params;
    field = currentField(c);
  }
  const auto t0 = std::chrono::steady_clock::now();
  gp::Plan pl = gp::plan(rq, p);
  const double ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  if (pl.ok) {
    RCLCPP_INFO(lg, "global plan (%s; %.0f ms): %s", why.c_str(), ms, gp::summary(pl).c_str());
  } else {
    RCLCPP_WARN(lg, "global plan (%s; %.0f ms): %s", why.c_str(), ms, gp::summary(pl).c_str());
  }
  for (const std::string & n : pl.notes) {
    if (pl.relaxed || n.rfind("WARNING", 0) == 0) {
      RCLCPP_WARN(lg, "  %s", n.c_str());
    } else {
      RCLCPP_INFO(lg, "  %s", n.c_str());
    }
  }
  for (const gp::Checkpoint & cp : pl.checkpoints) {
    RCLCPP_INFO(lg, "  checkpoint after gate red %d / green %d, %.1f m into the transit",
      cp.red_id, cp.green_id, cp.s);
  }
  if (!pl.ok) {return false;}
  std::lock_guard<std::mutex> lk(c.mu);
  Context::GlobalPassage & g = c.global_passage;
  g.plan = std::move(pl);
  g.field = std::move(field);
  g.transit_s = 0.0;                         // a new transit leg starts at the boat
  if (from <= Phase::EntryOrbit) {g.entry_ring = g.plan.entry_ring;}
  ++g.plans;
  return true;
}

/// Hold where the boat is.
void holdHere(Context & c, const rclcpp::Logger & lg, const char * why)
{
  sendHold(c, lg, std::string("hold (") + why + ") at");
}

/// Task 4: a moving object's keep-out within 20 m of the boat. TAKES ctx.mu ITSELF.
bool invaderNearBoat(Context & c)
{
  std::lock_guard<std::mutex> lk(c.mu);
  return invaderNear(c, {c.boat}, 20.0);
}

/// Compass degrees, from "0,-45,45". Unparsable entries are skipped; none at all = {0}.
std::vector<double> parseDegrees(const std::string & text)
{
  std::vector<double> out;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ',')) {
    try {
      std::size_t used = 0;
      const double v = std::stod(item, &used);
      if (used > 0 && std::isfinite(v)) {out.push_back(v);}
    } catch (const std::exception &) {
    }
  }
  if (out.empty()) {out.push_back(0.0);}
  return out;
}

/// -180..180
double wrap180(double d)
{
  double w = std::fmod(d + 180.0, 360.0);
  if (w < 0.0) {w += 360.0;}
  return w - 180.0;
}

// ---------------------------------------------------------------- conditions

/// The transit is behind the boat (every gate and side driven).
class TransitComplete : public CrusaderCondition
{
public:
  TransitComplete(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->global_passage.phase > Phase::Transit ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// ------------------------------------------------------------------- actions

/// RUNNING until fieldKnown(). No timeout of its own: the mission timeout is the backstop,
/// and handbook 3.3.2 says the boat may not transit before the aircraft has reported.
class WaitForField : public CrusaderAction
{
public:
  WaitForField(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override
  {
    said_ = false;
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    std::size_t n;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (fieldKnown(*ctx_)) {
        n = ctx_->buoys.size();
      } else {
        if (!said_) {
          said_ = true;
          RCLCPP_INFO(log(), "waiting for the aircraft's field (entry %s, exit %s, pose %s)",
            ctx_->have_entry ? "yes" : "--", ctx_->have_exit ? "yes" : "--",
            ctx_->pose_fresh ? "fresh" : "stale");
        }
        return BT::NodeStatus::RUNNING;
      }
    }
    RCLCPP_INFO(log(), "field known: %zu buoys", n);
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool said_ = false;
};

/// Ensures a plan for the rest of the mission from the current phase: keeps the one in hand
/// while planUnusable() allows, else re-plans from the boat. Its ports set the planner's
/// parameters for every later replan too.
///
/// Cannot plan: HOLDS, retries every retry_s (the field may change, fusion may move a buoy),
/// FAILURE after timeout_s. Never drives on a guess.
class PlanGlobalPassage : public CrusaderAction
{
public:
  PlanGlobalPassage(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("orbit_radius_m", "orbit radius wanted (absent: nav_orbit_radius_m); "
        "shrunk round a buoy until its neighbours keep nav_orbit_clear_m"),
      BT::InputPort<double>("orbit_min_m", 1.5, "smallest orbit normally accepted"),
      BT::InputPort<double>("overshoot_deg", 45.0, "orbit past one full turn, at least"),
      BT::InputPort<double>("soft_m", 2.0, "clearance beyond which more water buys nothing"),
      BT::InputPort<double>("w_clear", 4.0, "a metre at nav_hard_m costs 1 + w_clear metres"),
      BT::InputPort<double>("relax1_m", 0.7,
        "clearance tried when nothing fits at nav_hard_m (= nav_hard_m: never relax)"),
      BT::InputPort<double>("relax2_m", 0.5,
        "... and then this (0.5 threads the 1.48 m gate of the real Singapore field)"),
      BT::InputPort<double>("res_m", 0.1, "planning grid, metres"),
      BT::InputPort<double>("retask_move_m", 2.0,
        "a re-reported buoy that moved less than this is the aircraft's scatter, not a re-task"),
      BT::InputPort<double>("anchor_snap_m", 1.5,
        "a blue buoy this close to the ENTRY/EXIT point anchors its orbit (neighbours are 1.9-3.5 m "
        "apart on the real field: a 3 m snap can reach the wrong one)"),
      BT::InputPort<double>("weak_sd_m", 0.6,
        "a track with no colour and more position spread than this does not shape the ENTRY/EXIT "
        "orbits (0 = every track does); the approach and transit still keep clear of it"),
      BT::InputPort<double>("retry_s", 1.0, "between tries while no plan fits"),
      BT::InputPort<double>("timeout_s", 60.0, "FAILURE after this long without a plan")};
  }

  BT::NodeStatus onStart() override
  {
    gp::Params p;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const path::NavParams & np = ctx_->nav;
      p.hard_m = np.hard_m;
      p.orbit_clear_m = np.orbit_clear_m;
      p.orbit_r_pref_m = getInput<double>("orbit_radius_m").value_or(np.orbit_radius_m);
      p.buoy_radius_m = np.buoy_radius_m;
    }
    p.orbit_r_min_m = getInput<double>("orbit_min_m").value_or(1.5);
    p.overshoot_deg = getInput<double>("overshoot_deg").value_or(45.0);
    p.soft_m = getInput<double>("soft_m").value_or(2.0);
    p.w_clear = getInput<double>("w_clear").value_or(4.0);
    p.relax1_m = getInput<double>("relax1_m").value_or(0.7);
    p.relax2_m = getInput<double>("relax2_m").value_or(0.5);
    p.res_m = getInput<double>("res_m").value_or(0.1);
    p.retask_move_m = getInput<double>("retask_move_m").value_or(2.0);
    p.anchor_snap_m = getInput<double>("anchor_snap_m").value_or(1.5);
    p.weak_sd_m = getInput<double>("weak_sd_m").value_or(0.6);
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->global_passage.params = p;
    }
    t0_ = std::chrono::steady_clock::now();
    last_try_ = t0_ - std::chrono::hours(1);
    held_ = false;
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    Phase phase;
    std::string why;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      phase = ctx_->global_passage.phase;
      if (phase == Phase::Done) {return BT::NodeStatus::SUCCESS;}
      if (!fieldKnown(*ctx_)) {return BT::NodeStatus::RUNNING;}
      why = planUnusable(*ctx_, phase, log());
    }
    if (why.empty()) {return BT::NodeStatus::SUCCESS;}
    const auto now = std::chrono::steady_clock::now();
    const double since = std::chrono::duration<double>(now - last_try_).count();
    if (since >= getInput<double>("retry_s").value_or(1.0)) {
      last_try_ = now;
      if (replan(*ctx_, phase, log(), why)) {return BT::NodeStatus::SUCCESS;}
      if (!held_) {
        held_ = true;
        holdHere(*ctx_, log(), "no plan fits");
      }
    }
    // Task 4: a moving object's keep-out near the boat is a wait for it to pass, never a timeout
    if (invaderNearBoat(*ctx_)) {t0_ = now;}
    if (std::chrono::duration<double>(now - t0_).count() >= getInput<double>("timeout_s").value_or(60.0)) {
      RCLCPP_ERROR(log(), "no plan for the %s in %.0f s: giving up", gp::phaseName(phase),
        std::chrono::duration<double>(now - t0_).count());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

private:
  std::chrono::steady_clock::time_point t0_, last_try_;
  bool held_ = false;
};

/// Drives one phase of the plan.
///
///   * Already past this phase: SUCCESS at once, so a Retry round the whole sequence resumes
///     where the mission is. Not yet at it: FAILURE (a wiring error).
///   * The transit's track is recorded: its parity is what a mid-transit replan starts from.
///   * The aircraft's field changes during the approach or transit: re-plans from
///     the boat AT ONCE and carries on. An orbit carries on (it does not depend on colours)
///     and the next phase picks the change up.
///   * A hazard closer to the path ahead than the plan allowed: re-plans and carries on; if
///     the path ahead is BLOCKED (closer than hard - local_check_tol), HOLDS, re-plans every
///     second, resumes once clear for nav_unblock_reset_s, FAILS after blocked_timeout_s.
///   * Arrives: approach, entry orbit and transit hand over while moving unless stop_at_end;
///     the exit orbit stops on the point.
///   * Stale pose or a changed datum: holds once and waits; never fails for it.
class DrivePlanPhase : public CrusaderAction
{
public:
  DrivePlanPhase(const std::string & n, const BT::NodeConfig & c)
  : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("phase", "transit", "approach | entry_orbit | transit | exit_orbit"),
      BT::InputPort<bool>("stop_at_end", false,
        "stop on the leg's last point instead of handing over to the next leg while moving"),
      BT::InputPort<double>("blocked_timeout_s", 15.0, "FAILURE after being blocked this long"),
      BT::InputPort<double>("lookahead_m", 1.5, "pure pursuit lookahead along the path"),
      BT::InputPort<double>("resend_m", 0.3, "re-send the setpoint once it has moved this far"),
      BT::InputPort<double>("end_tol_m", 0.8, "arrival radius at a stop"),
      BT::InputPort<double>("sensor_settle_s", 1.0,
        "Core tier: a change in the boat's own field must persist this long before it is acted on"),
      BT::InputPort<double>("sensor_replan_min_s", 3.0,
        "Core tier: least time between two sensor-triggered replans"),
      BT::InputPort<double>("pause_every_m", 0.0,
        "transit only: stop and look every this far along it (0 = never)"),
      BT::InputPort<double>("pause_s", 5.0, "... for this long (the tracker needs ~4 s)"),
      BT::InputPort<double>("pause_unknown_m", 0.0,
        "approach, entry orbit and transit: stop and look at a buoy with no colour yet this near "
        "ahead, once per buoy, up to pause_s (0 = never)"),
      BT::InputPort<double>("unknown_fov_deg", 70.0,
        "... that counts as ahead: within half this angle of the bow either side")};
  }

  BT::NodeStatus onStart() override
  {
    const std::string ph = getInput<std::string>("phase").value_or("transit");
    if (!gp::parsePhase(ph, phase_)) {
      RCLCPP_ERROR(log(), "DrivePlanPhase: unknown phase '%s'", ph.c_str());
      return BT::NodeStatus::FAILURE;
    }
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const Phase cur = ctx_->global_passage.phase;
      if (cur > phase_) {return BT::NodeStatus::SUCCESS;}
      if (cur < phase_) {
        RCLCPP_ERROR(log(), "DrivePlanPhase %s: the mission is still in its %s", ph.c_str(),
          gp::phaseName(cur));
        return BT::NodeStatus::FAILURE;
      }
      last_now_ = ctx_->now_s;
    }
    blocked_ = false;
    blocked_s_ = 0.0;
    clear_since_ = -1.0;
    degraded_ = false;
    last_replan_ = -kInf;
    last_log_ = -kInf;
    raw_since_ = -1.0;
    last_sensor_replan_ = -kInf;
    sensor_failed_ = false;
    pausing_ = false;
    look_id_ = -1;
    pause_every_ = phase_ == Phase::Transit ?
      std::max(0.0, getInput<double>("pause_every_m").value_or(0.0)) : 0.0;
    pause_s_ = std::max(0.0, getInput<double>("pause_s").value_or(5.0));
    pause_unknown_ = phase_ == Phase::ExitOrbit ? 0.0 :
      std::max(0.0, getInput<double>("pause_unknown_m").value_or(0.0));
    unknown_half_fov_ = 0.5 * std::clamp(getInput<double>("unknown_fov_deg").value_or(70.0),
        0.0, 360.0);
    follower_.reset({});
    rest_.clear();
    if (!startLeg(true)) {
      enterBlocked("no plan for this leg");
      last_replan_ = last_now_;
    } else {
      RCLCPP_INFO(log(), "%s: %.1f m to drive", gp::phaseName(phase_), follower_.total());
    }
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    Vec2 boat;
    bool fresh, datum_ok;
    double now, reset_s;
    std::vector<path::Hazard> hz;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      boat = ctx_->boat;
      fresh = ctx_->pose_fresh;
      datum_ok = !ctx_->datum_mismatch;
      now = ctx_->now_s;
      reset_s = ctx_->nav.unblock_reset_s;
      hz = legHazards(*ctx_, phase_);
      if (fresh && phase_ == Phase::Transit) {recordTrack(boat);}
    }
    const double dt = std::max(0.0, now - last_now_);
    last_now_ = now;

    if (!fresh || !datum_ok) {
      if (!degraded_) {
        degraded_ = true;
        if (fresh) {holdHere(*ctx_, log(), "datum changed");}
        RCLCPP_WARN(log(), "%s: %s - holding", gp::phaseName(phase_),
          fresh ? "the datum changed (restart bt_runner)" : "stale pose");
        follower_.forgetSent();
      }
      return BT::NodeStatus::RUNNING;
    }
    degraded_ = false;
    const bool replannable = phase_ == Phase::Approach || phase_ == Phase::Transit;

    // 0. The plan was replaced under this leg (a replan whose leg would not load, or another
    //    leaf's): never finish a phase on a superseded leg - its end is the OLD plan's.
    if (!follower_.empty() && planGeneration() != leg_gen_ && !startLeg(true)) {
      follower_.reset({});
    }

    // 1. The field changed: re-plan the approach or the transit from here. The aircraft's
    //    re-task acts at once (1 Hz); in Core tier the boat's own field is debounced, checked for
    //    whether it matters, and rate-limited (sensorStep).
    const bool core = replannable && inCoreMode();
    if (core) {
      sensorStep(now);
    } else if (replannable && now - last_replan_ >= 1.0) {
      const std::string change = fieldChanged();
      if (!change.empty()) {replanHere(now, "the aircraft's field changed: " + change);}
    }
    // 2. No leg at all (no plan fitted at the start): keep trying.
    if (follower_.empty() && now - last_replan_ >= 1.0) {
      last_replan_ = now;
      startLeg(true);
    }

    gp::FollowOut o;
    if (!follower_.empty()) {o = follower_.step(boat, hz);}
    const char * stuck = follower_.empty() ? "no plan for this leg" :
      (core ? sensor_failed_ : (replannable && !fieldChanged().empty())) ?
      "no plan fits the changed field" :
      o.blocked ? "a hazard is on the path ahead" : nullptr;

    // 3. Stuck: hold, re-plan every second, FAILURE at blocked_timeout_s. Blocked by a Task 4
    //    object's keep-out it is WAITING for the object to pass: that time never counts.
    if (stuck != nullptr) {
      const bool invader = invaderAhead(boat);
      if (invader) {stuck = "a Task 4 object's keep-out is on the path ahead: waiting for it to pass";}
      if (!blocked_) {enterBlocked(stuck);}
      if (!invader) {blocked_s_ += dt;}
      clear_since_ = -1.0;
      if (now - last_replan_ >= 1.0) {replanHere(now, stuck);}
      writeStatus(o, "BLOCKED", stuck);
      if (timedOut()) {
        RCLCPP_WARN(log(), "%s: blocked for %.0f s (%s) - leg FAILED", gp::phaseName(phase_),
          blocked_s_, stuck);
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    // 4. Clear again: drive on only once it has STAYED clear (a flickering track must not
    //    make the boat lurch at it), counting toward the timeout meanwhile.
    if (blocked_) {
      if (!invaderAhead(boat)) {blocked_s_ += dt;}
      if (clear_since_ < 0.0) {clear_since_ = now;}
      if (now - clear_since_ + 1e-6 < reset_s) {
        writeStatus(o, "BLOCKED", "clear; holding until it has stayed clear");
        return timedOut() ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
      }
      RCLCPP_INFO(log(), "%s: clear again after %.1f s - driving on", gp::phaseName(phase_),
        blocked_s_);
      blocked_ = false;
      blocked_s_ = 0.0;
      follower_.forgetSent();
      o = follower_.step(boat, hz);
    }
    if (o.arrived) {return finish(o);}

    // 4b. Core tier's LOOKS: hold the spot up to pause_s so the tracker can call the lights of
    //     the buoys now near (it needs ~4 s), then drive on. Two causes: every pause_every_m
    //     along the transit, and a buoy with no colour yet coming into view ahead
    //     (pause_unknown_m, once per buoy). A replan meanwhile (sensorStep, above) is taken as
    //     usual. Pause time is never blocked time (blocked_s_ only grows in steps 3 and 4).
    if (pausing_) {
      if (!lookOver(now)) {
        writeStatus(o, "PAUSED", pause_why_);
        return BT::NodeStatus::RUNNING;
      }
      endLook(o.s);
      o = follower_.step(boat, hz);
    } else if (const int id = unnamedAhead(); id >= 0) {
      beginLook(now, "a buoy with no colour yet", "a buoy with no colour yet: looking", id);
      writeStatus(o, "PAUSED", pause_why_);
      return BT::NodeStatus::RUNNING;
    } else if (pause_every_ > 0.0 && offset_ + o.s >= next_pause_ &&
      o.total - o.s > 0.5 * pause_every_)
    {
      beginLook(now, "a look at the field", "looking at the buoys near the boat", -1);
      RCLCPP_INFO(log(), "%s: %.1f m in, %.1f m to go: pausing %.0f s to look",
        gp::phaseName(phase_), offset_ + o.s, o.total - o.s, pause_s_);
      writeStatus(o, "PAUSED", pause_why_);
      return BT::NodeStatus::RUNNING;
    }

    // 5. Not blocked, but the water ahead shrank since planning (the boat's camera moved a
    //    buoy from the aircraft's ~1 m fix, or a track appeared): re-plan without stopping,
    //    so the path re-centres on the better positions before it is nearly blocked.
    if (replannable && now - last_replan_ >= 3.0) {
      const std::string why = shrunk(hz);
      if (!why.empty() && replanHere(now, why)) {o = follower_.step(boat, hz);}
    }

    if (o.send) {
      sendSetpoint(*ctx_, log(), o.setpoint, "");
      if (now - last_log_ >= 5.0) {
        last_log_ = now;
        RCLCPP_INFO(log(), "%s: %.1f / %.1f m, %.2f m off the path, %.2f m of water ahead",
          gp::phaseName(phase_), o.s, o.total, o.offpath, o.clear_ahead);
      }
    }
    markGatesPassed();
    writeStatus(o, "FOLLOWING", "");
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    clearLeg(*ctx_);
  }

private:
  /// Load what is left of this phase's leg into the follower, re-planning first if the plan
  /// in hand will not do and `allow_replan`. False = no usable leg.
  bool startLeg(bool allow_replan)
  {
    std::string why;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      why = planUnusable(*ctx_, phase_, log());
    }
    if (!why.empty()) {
      if (!allow_replan || !replan(*ctx_, phase_, log(), why)) {return false;}
    }
    const bool stop_end = getInput<bool>("stop_at_end").value_or(false);
    std::vector<Vec2> path;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      Context::GlobalPassage & g = ctx_->global_passage;
      const std::vector<Vec2> & leg = g.plan.legs[static_cast<int>(phase_)];
      if (leg.size() < 1) {return false;}
      plan_hard_ = g.plan.hard_used;
      leg_gen_ = g.plans;
      offset_ = phase_ == Phase::Transit ? g.transit_s : 0.0;
      if (phase_ == Phase::Transit && g.traj.empty()) {g.traj.push_back(leg.front());}
      path = leg.size() < 2 ? leg : gp::slice(leg, offset_, gp::arcLengths(leg).back());
      stop_ = stop_end || phase_ == Phase::ExitOrbit;
      rest_.clear();
      for (int k = static_cast<int>(phase_) + 1; k < 4; ++k) {
        const std::vector<Vec2> & l = g.plan.legs[k];
        rest_.insert(rest_.end(), l.begin(), l.end());
      }
      fp_.lookahead_m = getInput<double>("lookahead_m").value_or(1.5);
      fp_.resend_m = getInput<double>("resend_m").value_or(0.3);
      fp_.end_tol_m = getInput<double>("end_tol_m").value_or(0.8);
      fp_.pass_early_m = stop_ ? 0.0 : fp_.lookahead_m + 0.5;
      fp_.check_clear_m = std::min(ctx_->nav.hard_m - ctx_->nav.local_check_tol_m, plan_hard_ - 0.1);
    }
    follower_.setParams(fp_);
    follower_.reset(std::move(path));
    next_pause_ = pause_every_ > 0.0 ? offset_ + pause_every_ : kInf;
    // the water every vertex had when this leg was loaded: shrunk() compares against it
    std::vector<path::Hazard> hz;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      hz = legHazards(*ctx_, phase_);
      soft_ = ctx_->global_passage.params.soft_m;
    }
    cum_ = gp::arcLengths(follower_.path());
    planned_clear_.clear();
    for (const Vec2 & v : follower_.path()) {planned_clear_.push_back(path::minClearance(hz, v));}
    return true;
  }

  /// "" unless, somewhere in the next check_ahead_m of the leg, a hazard is now 0.3 m closer
  /// than when the leg was loaded (and inside soft_m), or closer than the plan's clearance.
  std::string shrunk(const std::vector<path::Hazard> & hz) const
  {
    const std::vector<Vec2> & pl = follower_.path();
    const double s0 = follower_.s();
    for (std::size_t i = 0; i < pl.size() && i < planned_clear_.size(); ++i) {
      if (cum_[i] < s0) {continue;}
      if (cum_[i] > s0 + fp_.check_ahead_m) {break;}
      const double now_c = path::minClearance(hz, pl[i]);
      if (now_c < plan_hard_ - 0.05) {
        return "a hazard is " + gp::detail::fmt(now_c, 2) + " m from the path ahead";
      }
      if (now_c < std::min(soft_, planned_clear_[i]) - 0.3) {
        return "a buoy moved " + gp::detail::fmt(planned_clear_[i] - now_c, 1) +
          " m toward the path ahead";
      }
    }
    return "";
  }

  /// replan() now and load this phase's new leg. True = the follower has one.
  bool replanHere(double now, const std::string & why)
  {
    last_replan_ = now;
    const bool ok = replan(*ctx_, phase_, log(), why) && startLeg(false);
    if (ok) {sensor_failed_ = false;}
    return ok;
  }

  /// Core tier, once per tick: a raw change in the boat's own field must persist
  /// sensor_settle_s (colour votes flicker, a flashing light has an off-phase); then it is
  /// absorbed if it does not matter for the plan in hand, else the plan is remade from the boat,
  /// no sooner than sensor_replan_min_s after the last sensor replan. A FAILED replan sets
  /// sensor_failed_, which holds the boat and hands the retries to the blocked path (step 3).
  void sensorStep(double now)
  {
    std::string raw, why;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      raw = retasked(*ctx_);
      if (raw.empty()) {
        raw_since_ = -1.0;
        sensor_failed_ = false;
        return;
      }
      if (raw_since_ < 0.0) {raw_since_ = now;}
      if (now - raw_since_ < getInput<double>("sensor_settle_s").value_or(1.0)) {return;}
      why = sensorNeedsReplan(*ctx_, phase_, log(), raw);
    }
    if (why.empty()) {raw_since_ = -1.0; return;}       // absorbed
    if (sensor_failed_ ||
      now - last_sensor_replan_ < getInput<double>("sensor_replan_min_s").value_or(3.0))
    {
      return;
    }
    last_sensor_replan_ = now;
    if (replanHere(now, "the boat's own field: " + why)) {
      raw_since_ = -1.0;
    } else {
      sensor_failed_ = true;
    }
  }

  /// A Task 4 keep-out within 1 m of the next 12 m of this leg (or of the boat).
  bool invaderAhead(Vec2 boat)
  {
    std::vector<Vec2> ahead{boat};
    const std::vector<Vec2> rem = follower_.remaining();
    double run = 0.0;
    for (std::size_t i = 0; i < rem.size() && run < 12.0; ++i) {
      if (i > 0) {run += nav::norm(rem[i] - rem[i - 1]);}
      ahead.push_back(rem[i]);
    }
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return invaderNear(*ctx_, ahead, 1.0);
  }

  bool inCoreMode()
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return coreMode(*ctx_);
  }

  bool timedOut()
  {
    return blocked_s_ + 1e-6 >= getInput<double>("blocked_timeout_s").value_or(15.0);
  }

  int planGeneration()
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->global_passage.plans;
  }

  /// retasked(), under the lock.
  std::string fieldChanged()
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return retasked(*ctx_);
  }

  /// Stop on the spot: hold, and forget what was sent so the first carrot after is sent again.
  void holdAndForget(const char * why)
  {
    holdHere(*ctx_, log(), why);
    follower_.forgetSent();
  }

  /// Start a look: hold for up to pause_s. `buoy_id` >= 0 is the unnamed buoy being looked at
  /// (the look ends early once it has a colour or its track is gone), -1 a timed look.
  void beginLook(double now, const char * hold_why, const char * status_why, int buoy_id)
  {
    pausing_ = true;
    pause_until_ = now + pause_s_;
    look_id_ = buoy_id;
    pause_why_ = status_why;
    holdAndForget(hold_why);
  }

  /// The look is over: pause_s has passed, or the buoy looked at has a colour (or no track) now.
  bool lookOver(double now)
  {
    if (now >= pause_until_) {return true;}
    if (look_id_ < 0) {return false;}
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const nav::Buoy * b = nav::findById(ctx_->buoys, look_id_);
    return b == nullptr || b->state != nav::Beacon::Unknown;
  }

  /// Finish a look at transit arc `s`: reschedule the periodic look from here, mark every unnamed
  /// buoy now in view as looked at (one look per buoy, whatever it ended up called), drive on.
  void endLook(double s)
  {
    pausing_ = false;
    next_pause_ = offset_ + s + pause_every_;
    if (pause_unknown_ > 0.0) {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      std::set<int> & looked = ctx_->looked_unnamed;
      if (look_id_ >= 0) {looked.insert(look_id_);}
      for (int id; (id = nav::unnamedInView(ctx_->buoys, ctx_->boat, ctx_->heading_deg,
        pause_unknown_, unknown_half_fov_, looked)) >= 0;)
      {
        looked.insert(id);
      }
    }
    look_id_ = -1;
    follower_.forgetSent();
    RCLCPP_INFO(log(), "%s: look done, driving on", gp::phaseName(phase_));
  }

  /// The nearest buoy with no colour yet, in view ahead and not looked at before: its id, and
  /// the pause is logged here. -1 when pause_unknown_m is off or there is none.
  int unnamedAhead()
  {
    if (pause_unknown_ <= 0.0) {return -1;}
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const int id = nav::unnamedInView(ctx_->buoys, ctx_->boat, ctx_->heading_deg, pause_unknown_,
        unknown_half_fov_, ctx_->looked_unnamed);
    if (id >= 0) {
      const nav::Buoy * b = nav::findById(ctx_->buoys, id);
      RCLCPP_INFO(log(), "%s: buoy %d %.1f m ahead has no colour yet - pausing up to %.0f s to "
        "look", gp::phaseName(phase_), id, b != nullptr ? nav::norm(b->p - ctx_->boat) : 0.0,
        pause_s_);
    }
    return id;
  }

  void enterBlocked(const char * why)
  {
    blocked_ = true;
    clear_since_ = -1.0;
    holdAndForget(why);
    RCLCPP_WARN(log(), "%s: %s - holding, re-planning every second", gp::phaseName(phase_), why);
  }

  /// The transit as driven, every 0.2 m. CALL UNDER ctx.mu.
  void recordTrack(Vec2 boat)
  {
    std::vector<Vec2> & tr = ctx_->global_passage.traj;
    if (tr.empty() || nav::norm(boat - tr.back()) >= 0.2) {tr.push_back(boat);}
  }

  /// Gates the boat has driven through count as cleared: a replan never puts one behind the boat.
  void markGatesPassed()
  {
    if (phase_ != Phase::Transit) {return;}
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const double at = offset_ + follower_.s();
    for (const gp::Checkpoint & cp : ctx_->global_passage.plan.checkpoints) {
      if (cp.s > at) {continue;}
      if (!nav::gateCleared(ctx_->cleared_gates, cp.red_id, cp.green_id)) {
        ctx_->cleared_gates.push_back({cp.red_id, cp.green_id});
        RCLCPP_INFO(log(), "through gate red %d / green %d", cp.red_id, cp.green_id);
      }
    }
  }

  BT::NodeStatus finish(const gp::FollowOut &)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    Context::GlobalPassage & g = ctx_->global_passage;
    if (phase_ == Phase::EntryOrbit) {g.entry_ring = g.plan.entry_ring;}
    g.phase = static_cast<Phase>(static_cast<int>(phase_) + 1);
    RCLCPP_INFO(log(), "%s complete -> %s", gp::phaseName(phase_), gp::phaseName(g.phase));
    return BT::NodeStatus::SUCCESS;
  }

  void writeStatus(const gp::FollowOut & o, const char * state, const std::string & why)
  {
    std::vector<Vec2> shown = follower_.remaining();
    std::lock_guard<std::mutex> lk(ctx_->mu);
    shown.insert(shown.end(), rest_.begin(), rest_.end());
    Context::LegStatus & s = ctx_->leg;
    s.leaf = "DrivePlanPhase";
    s.name = gp::phaseName(phase_);
    s.state = state;
    s.why = why;
    s.avoid = true;
    s.blocked_s = blocked_s_;
    s.have_goal = !follower_.empty();
    s.goal = follower_.empty() ? Vec2{} : follower_.path().back();
    s.have_target = !blocked_;
    s.target = o.target;
    s.path = std::move(shown);
    s.plan_ms = -1.0;
    s.hop = -1;
    s.hops = 0;
    ++ctx_->leg_seq;
  }

  Phase phase_ = Phase::Transit;
  gp::Follower follower_;
  gp::FollowParams fp_;
  std::vector<Vec2> rest_;          ///< the rest of the plan, for the map
  double offset_ = 0.0;             ///< arc of the follower's start on the transit leg
  double plan_hard_ = 0.8;
  int leg_gen_ = -1;                ///< global_passage.plans when the leg was loaded
  double soft_ = 2.0;
  std::vector<double> cum_, planned_clear_;   ///< per follower vertex: arc, clearance when loaded
  bool stop_ = false;
  bool blocked_ = false, degraded_ = false;
  double blocked_s_ = 0.0, clear_since_ = -1.0;
  double last_now_ = 0.0, last_replan_ = -kInf, last_log_ = -kInf;
  // Core tier's sensor replans (sensorStep)
  double raw_since_ = -1.0;                   ///< when the current raw change was first seen
  double last_sensor_replan_ = -kInf;
  bool sensor_failed_ = false;                ///< a needed sensor replan found no plan
  // the transit's looks (pause_every_m)
  double pause_every_ = 0.0, pause_s_ = 5.0;
  double next_pause_ = kInf;                  ///< transit arc of the next look
  bool pausing_ = false;
  double pause_until_ = 0.0;
  std::string pause_why_;                     ///< the PAUSED status line
  // the looks at unnamed buoys (pause_unknown_m)
  double pause_unknown_ = 0.0;                ///< range, m; 0 = off (and always off in the exit orbit)
  double unknown_half_fov_ = 35.0;            ///< deg either side of the bow
  int look_id_ = -1;                          ///< the unnamed buoy being looked at, -1 = a timed look
};

// ------------------------------------------------- Core tier: looking for the field

/// The ENTRY / EXIT / both were SEEN this run by the boat (or given by the GCS MANUAL field):
/// not remembered from an earlier run, not guessed (Context::entry_seen, exit_seen).
class AnchorSeen : public CrusaderCondition
{
public:
  AnchorSeen(const std::string & n, const BT::NodeConfig & c) : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("which", "entry", "entry | exit | both")};
  }

  BT::NodeStatus tick() override
  {
    const std::string w = getInput<std::string>("which").value_or("entry");
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const bool ok = w == "exit" ? ctx_->exit_seen :
      w == "both" ? (ctx_->entry_seen && ctx_->exit_seen) : ctx_->entry_seen;
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

/// The passage is under way: a plan was made, or the mission is past its approach. A Task 4
/// resume (AvoidInvaders restarts its child) then skips the search and goes straight to planning.
class PassageStarted : public CrusaderCondition
{
public:
  PassageStarted(const std::string & n, const BT::NodeConfig & c) : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const Context::GlobalPassage & g = ctx_->global_passage;
    return (g.plans > 0 || g.phase > Phase::Approach) ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

/// Stand still and LOOK (Core tier: the boat finds the field itself).
///
/// Keeps the spot it started on (HoldHeading's rule: the heading+speed command, back to the spot
/// when it drifts past radius_m) and turns through the looks: headings offsets_deg from the base
/// `face`, staying dwell_s at each once within settle_deg of it (the tracker needs ~4 s to call a
/// buoy's light; a turn that takes longer than turn_timeout_s looks from where it got to).
///   face: field = toward the operator's field centre (the GCS Run panel), exit / entry = toward
///         that buoy (the EXIT may be the guess), heading = where the bow points now. Any of them
///         unknown: the bow's heading.
/// SUCCESS after the last look, or as soon as `until` (entry | exit | both) is SEEN; never
/// FAILURE for what it did not see (the tree decides). FAILURE only on a pose stale for 5 s.
/// GUIDED only; sends nothing when publish_setpoints is false.
class ScanForBuoys : public CrusaderAction
{
public:
  ScanForBuoys(const std::string & n, const BT::NodeConfig & c) : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("face", "field", "field | exit | entry | heading"),
      BT::InputPort<std::string>("offsets_deg", "0,-45,45", "the looks, from the base heading"),
      BT::InputPort<double>("dwell_s", 5.0, "each look, once on the heading"),
      BT::InputPort<double>("settle_deg", 12.0, "on the heading within this"),
      BT::InputPort<double>("turn_timeout_s", 15.0, "a turn taking longer looks from there"),
      BT::InputPort<double>("radius_m", 1.5, "drift from the spot that triggers a return"),
      BT::InputPort<std::string>("until", "", "end early once SEEN: entry | exit | both | field (ENTRY seen, EXIT seen or "
        "guessed) | (never)")};
  }

  BT::NodeStatus onStart() override
  {
    looks_ = parseDegrees(getInput<std::string>("offsets_deg").value_or("0,-45,45"));
    dwell_s_ = std::max(0.5, getInput<double>("dwell_s").value_or(5.0));
    settle_deg_ = getInput<double>("settle_deg").value_or(12.0);
    turn_timeout_s_ = getInput<double>("turn_timeout_s").value_or(15.0);
    prm_.radius_m = std::max(0.5, getInput<double>("radius_m").value_or(1.5));
    prm_.return_speed_mps = 0.3;
    until_ = getInput<std::string>("until").value_or("");
    const std::string face = getInput<std::string>("face").value_or("field");
    std::string base_from = "the bow";
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!ctx_->pose_fresh || !ctx_->origin_set) {
        RCLCPP_ERROR(log(), "ScanForBuoys: no fresh pose, cannot take the spot");
        return BT::NodeStatus::FAILURE;
      }
      anchor_ = ctx_->boat;
      start_s_ = turn_s_ = ctx_->now_s;
      base_ = std::isfinite(ctx_->heading_deg) ? ctx_->heading_deg : 0.0;
      Vec2 toward;
      bool have = false;
      if (face == "field") {
        have = fieldCentre(*ctx_, &toward);
      } else if (face == "exit") {
        have = ctx_->have_exit;
        toward = ctx_->exitp;
        if (!have) {have = fieldCentre(*ctx_, &toward);}
      } else if (face == "entry") {
        have = ctx_->have_entry;
        toward = ctx_->entry;
        if (!have) {have = fieldCentre(*ctx_, &toward);}
      }
      if (have && nav::norm(toward - anchor_) > 1.0) {
        base_ = nav::bearingDeg(anchor_, toward);
        base_from = face == "field" ? "the field centre" : "the " + face;
      }
    }
    look_ = 0;
    dwelling_since_ = -1.0;
    returning_ = false;
    stale_ = false;
    RCLCPP_INFO(log(), "scan: %zu looks of %.0f s round %03.0f deg (toward %s)%s", looks_.size(),
      dwell_s_, holdheading::wrap360(base_), base_from.c_str(),
      ctx_->publish_setpoints ? "" : "  [NOT SENT: publish_setpoints is false]");
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    Vec2 boat;
    double now, hdg_now;
    bool fresh, done_early;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      boat = ctx_->boat;
      now = ctx_->now_s;
      hdg_now = ctx_->heading_deg;
      fresh = ctx_->pose_fresh;
      done_early = until_ == "entry" ? ctx_->entry_seen :
        until_ == "exit" ? ctx_->exit_seen :
        until_ == "both" ? (ctx_->entry_seen && ctx_->exit_seen) :
        until_ == "field" ? fieldKnown(*ctx_) : false;
    }
    if (done_early) {
      RCLCPP_INFO(log(), "scan: %s seen after %.0f s - done", until_.c_str(), now - start_s_);
      return BT::NodeStatus::SUCCESS;
    }
    if (!fresh) {
      if (!stale_) {
        stale_ = true;
        stale_s_ = now;
        RCLCPP_WARN(log(), "scan: pose stale, not commanding");
      }
      if (now - stale_s_ > 5.0) {
        RCLCPP_ERROR(log(), "scan: pose stale for 5 s, giving up");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    stale_ = false;
    prm_.heading_deg = holdheading::wrap360(base_ + looks_[look_]);
    const holdheading::Command cmd = holdheading::decide(boat, anchor_, returning_, prm_);
    returning_ = cmd.returning;
    const bool on_heading = !cmd.returning && std::isfinite(hdg_now) &&
      std::fabs(wrap180(prm_.heading_deg - hdg_now)) <= settle_deg_;
    if (dwelling_since_ < 0.0 && (on_heading || now - turn_s_ > turn_timeout_s_)) {
      dwelling_since_ = now;
      RCLCPP_INFO(log(), "scan: look %d/%zu at %03.0f deg%s", look_ + 1, looks_.size(),
        prm_.heading_deg, on_heading ? "" : " (turn timed out: looking from here)");
    }
    if (dwelling_since_ >= 0.0 && now - dwelling_since_ >= dwell_s_) {
      if (++look_ >= static_cast<int>(looks_.size())) {
        RCLCPP_INFO(log(), "scan: done after %.0f s", now - start_s_);
        return BT::NodeStatus::SUCCESS;
      }
      dwelling_since_ = -1.0;
      turn_s_ = now;
      prm_.heading_deg = holdheading::wrap360(base_ + looks_[look_]);
    }
    if (ctx_->publish_setpoints && ctx_->heading_speed) {
      ctx_->heading_speed(cmd.returning ? cmd.heading_deg : prm_.heading_deg,
        cmd.returning ? cmd.speed_mps : 0.0);
    }
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      Context::LegStatus & s = ctx_->leg;
      s = Context::LegStatus{};
      s.leaf = "ScanForBuoys";
      s.name = "look " + std::to_string(look_ + 1) + "/" + std::to_string(looks_.size());
      s.state = dwelling_since_ >= 0.0 ? "LOOKING" : (returning_ ? "RETURNING" : "TURNING");
      s.why = "heading " + std::to_string(static_cast<int>(std::lround(prm_.heading_deg)));
      s.have_goal = true;
      s.goal = anchor_;
      ++ctx_->leg_seq;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    clearLeg(*ctx_);
  }

private:
  std::vector<double> looks_;
  holdheading::Params prm_;
  Vec2 anchor_;
  double base_ = 0.0, dwell_s_ = 5.0, settle_deg_ = 12.0, turn_timeout_s_ = 15.0;
  double start_s_ = 0.0, turn_s_ = 0.0, dwelling_since_ = -1.0, stale_s_ = 0.0;
  int look_ = 0;
  bool returning_ = false, stale_ = false;
  std::string until_;
};

/// Core tier, the CENTRE SCAN (task1_core_centre.xml): stand on the spot (the operator's field
/// centre, or wherever the drive there ended) and turn through the whole circle one camera view
/// at a time, staying on each SECTOR until every close buoy in view has a colour.
///
///   sectors  n = ceil(360 / (hfov_deg - overlap_deg)), evenly spaced (360 / n apart), the first
///            on the bow's heading when the scan starts, then clockwise round to it again.
///   a sector is DONE, after at least min_s on its heading, once every buoy within range_m of the
///            boat and inside the view (+-(hfov_deg / 2 - edge_deg) of the heading) has a colour
///            (any state but unknown: off/black counts) that has held for stable_s. After max_s on
///            it the scan moves on anyway, and says which buoys in view still had none.
///   Buoys beyond range_m wait for nobody: the operator's "close" (t1_scan_range_m).
///   A buoy still uncoloured after colour_wait_s IN VIEW (summed over the whole scan) is waited on
///            no more: a black buoy never gets a colour (target_tracker reports a track with only
///            dark votes as unknown, since a flashing light is dark half the time), so without
///            this every sector holding one ran to max_s (2026-11-13 sim: scans of 70-120 s).
/// HoldHeading's rule keeps the spot (back to it when the boat drifts past radius_m), as
/// ScanForBuoys does; a turn taking longer than turn_timeout_s looks from where it got to.
/// SUCCESS after the last sector; never FAILURE for what it did not see (the tree decides, with
/// FieldKnown). FAILURE only on a pose stale for 5 s. GUIDED only; sends nothing when
/// publish_setpoints is false.
class SectorScan : public CrusaderAction
{
public:
  SectorScan(const std::string & n, const BT::NodeConfig & c) : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("hfov_deg", 82.0, "the camera's horizontal field of view"),
      BT::InputPort<double>("overlap_deg", 15.0, "each sector overlaps the next by this"),
      BT::InputPort<double>("edge_deg", 5.0, "a buoy this near the edge of the view is not waited on"),
      BT::InputPort<double>("range_m", 15.0, "wait for the colours of buoys within this of the boat"),
      BT::InputPort<double>("min_s", 4.0, "at least this long on each sector"),
      BT::InputPort<double>("max_s", 15.0, "at most this long on each sector"),
      BT::InputPort<double>("stable_s", 1.0, "a colour must hold this long to count"),
      BT::InputPort<double>("colour_wait_s", 6.0,
        "stop waiting on a buoy still uncoloured after this long in view (0 = wait to max_s)"),
      BT::InputPort<double>("settle_deg", 12.0, "on the heading within this"),
      BT::InputPort<double>("turn_timeout_s", 15.0, "a turn taking longer looks from there"),
      BT::InputPort<double>("radius_m", 0.8, "drift from the spot that triggers a return"),
      BT::InputPort<double>("spot_search_m", 1.5,
        "the spot is the point this near the boat farthest from every known buoy (0 = here)")};
  }

  /// Distance from p to the nearest known buoy or obstacle (inf: none). CALL UNDER ctx.mu.
  static double nearestBuoy(const Context & c, Vec2 p)
  {
    double d = kInf;
    for (const nav::Buoy & b : c.buoys) {d = std::min(d, nav::norm(b.p - p));}
    for (const nav::Buoy & b : c.obstacles) {d = std::min(d, nav::norm(b.p - p));}
    return d;
  }

  /// The point within r of `here` (a 0.25 m grid) with the most room to the nearest buoy, a
  /// tie going to the nearer one: turning on the spot sweeps the hull round, and a buoy 1.5 m
  /// off is touched by a 1 m boat that drifts. CALL UNDER ctx.mu.
  static Vec2 clearestSpot(const Context & c, Vec2 here, double r)
  {
    Vec2 best = here;
    double best_d = nearestBuoy(c, here);
    if (r <= 0.0 || !std::isfinite(best_d)) {return here;}
    for (double dx = -r; dx <= r + 1e-9; dx += 0.25) {
      for (double dy = -r; dy <= r + 1e-9; dy += 0.25) {
        if (dx * dx + dy * dy > r * r) {continue;}
        const Vec2 q{here.x + dx, here.y + dy};
        const double d = nearestBuoy(c, q) - 0.05 * std::hypot(dx, dy);   // prefer staying put
        if (d > best_d + 1e-6) {
          best_d = d;
          best = q;
        }
      }
    }
    return best;
  }

  BT::NodeStatus onStart() override
  {
    const double hfov =std::clamp(getInput<double>("hfov_deg").value_or(82.0), 20.0, 180.0);
    const double overlap = std::clamp(getInput<double>("overlap_deg").value_or(15.0), 0.0, hfov - 10.0);
    half_view_deg_ = std::max(5.0, hfov / 2.0 - std::max(0.0, getInput<double>("edge_deg").value_or(5.0)));
    range_m_ = std::max(1.0, getInput<double>("range_m").value_or(15.0));
    min_s_ = std::max(0.5, getInput<double>("min_s").value_or(4.0));
    max_s_ = std::max(min_s_, getInput<double>("max_s").value_or(15.0));
    stable_s_ = std::max(0.0, getInput<double>("stable_s").value_or(1.0));
    colour_wait_s_ = std::max(0.0, getInput<double>("colour_wait_s").value_or(6.0));
    settle_deg_ = getInput<double>("settle_deg").value_or(12.0);
    turn_timeout_s_ = getInput<double>("turn_timeout_s").value_or(15.0);
    prm_.radius_m = std::max(0.3, getInput<double>("radius_m").value_or(0.8));
    prm_.return_speed_mps = 0.3;
    n_ = std::max(1, static_cast<int>(std::ceil(360.0 / (hfov - overlap) - 1e-9)));
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!ctx_->pose_fresh || !ctx_->origin_set) {
        RCLCPP_ERROR(log(), "SectorScan: no fresh pose, cannot take the spot");
        return BT::NodeStatus::FAILURE;
      }
      anchor_ = clearestSpot(*ctx_, ctx_->boat, std::max(0.0, getInput<double>("spot_search_m").value_or(1.5)));
      start_s_ = turn_s_ = ctx_->now_s;
      base_ = std::isfinite(ctx_->heading_deg) ? ctx_->heading_deg : 0.0;
      const double moved = nav::norm(anchor_ - ctx_->boat);
      if (moved > 0.2) {
        RCLCPP_INFO(log(), "sector scan: the spot moves %.1f m, to %.1f m from the nearest buoy",
          moved, nearestBuoy(*ctx_, anchor_));
      }
    }
    sector_ = 0;
    dwelling_since_ = -1.0;
    returning_ = false;
    stale_ = false;
    seen_.clear();
    waited_.clear();
    last_tick_s_ = -1.0;
    RCLCPP_INFO(log(), "sector scan: %d sectors of %.0f deg from %03.0f (camera %.0f deg, overlap "
      "%.0f), %.0f-%.0f s each, waiting for colours within %.0f m (at most %.0f s in view each)%s",
      n_, 360.0 / n_, holdheading::wrap360(base_), hfov, overlap, min_s_, max_s_, range_m_,
      colour_wait_s_, ctx_->publish_setpoints ? "" : "  [NOT SENT: publish_setpoints is false]");
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    Vec2 boat;
    double now, hdg_now;
    bool fresh;
    std::vector<nav::Buoy> buoys;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      boat = ctx_->boat;
      now = ctx_->now_s;
      hdg_now = ctx_->heading_deg;
      fresh = ctx_->pose_fresh;
      buoys = ctx_->buoys;
    }
    // how long each buoy's colour has held (a change starts it again)
    for (const nav::Buoy & b : buoys) {
      auto it = seen_.find(b.id);
      if (it == seen_.end() || it->second.first != b.state) {seen_[b.id] = {b.state, now};}
    }
    if (!fresh) {
      if (!stale_) {
        stale_ = true;
        stale_s_ = now;
        RCLCPP_WARN(log(), "sector scan: pose stale, not commanding");
      }
      if (now - stale_s_ > 5.0) {
        RCLCPP_ERROR(log(), "sector scan: pose stale for 5 s, giving up");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
    stale_ = false;
    prm_.heading_deg = holdheading::wrap360(base_ + sector_ * (360.0 / n_));
    const holdheading::Command cmd = holdheading::decide(boat, anchor_, returning_, prm_);
    returning_ = cmd.returning;
    const bool on_heading = !cmd.returning && std::isfinite(hdg_now) &&
      std::fabs(wrap180(prm_.heading_deg - hdg_now)) <= settle_deg_;
    if (dwelling_since_ < 0.0 && (on_heading || now - turn_s_ > turn_timeout_s_)) {
      dwelling_since_ = now;
      RCLCPP_INFO(log(), "sector scan: sector %d/%d at %03.0f deg%s", sector_ + 1, n_,
        prm_.heading_deg, on_heading ? "" : " (turn timed out: looking from here)");
    }
    int in_view = 0;
    std::string waiting;
    const double dt = last_tick_s_ < 0.0 ? 0.0 : std::clamp(now - last_tick_s_, 0.0, 1.0);
    last_tick_s_ = now;
    if (dwelling_since_ >= 0.0) {
      for (const nav::Buoy & b : buoys) {
        const double d = nav::norm(b.p - boat);
        if (d > range_m_ || d < 0.3) {continue;}
        if (std::fabs(wrap180(nav::bearingDeg(boat, b.p) - prm_.heading_deg)) > half_view_deg_) {
          continue;
        }
        ++in_view;
        const auto & s = seen_[b.id];
        if (b.state == nav::Beacon::Unknown) {
          double & w = waited_[b.id];
          const bool was_waiting = colour_wait_s_ <= 0.0 || w < colour_wait_s_;
          w += dt;
          if (colour_wait_s_ > 0.0 && w >= colour_wait_s_) {
            if (was_waiting) {
              RCLCPP_INFO(log(), "sector scan: track %d at %.1f m: no colour after %.0f s in view "
                "(a black buoy, or hidden); not waiting on it", b.id, d, w);
            }
            continue;
          }
        }
        if (b.state == nav::Beacon::Unknown || now - s.second < stable_s_) {
          waiting += (waiting.empty() ? "" : ", ") + std::to_string(b.id) + " at " +
            gp::detail::fmt(d, 1) + " m";
        }
      }
      const double on = now - dwelling_since_;
      const bool clear = on >= min_s_ && waiting.empty();
      if (clear || on >= max_s_) {
        if (clear) {
          RCLCPP_INFO(log(), "sector scan: sector %d/%d done after %.1f s: %d buoy(s) within %.0f m "
            "in view, every one coloured", sector_ + 1, n_, on, in_view, range_m_);
        } else {
          RCLCPP_WARN(log(), "sector scan: sector %d/%d: %.0f s up, moving on; still no colour: %s",
            sector_ + 1, n_, on, waiting.c_str());
        }
        if (++sector_ >= n_) {
          summary(buoys, boat, now);
          return BT::NodeStatus::SUCCESS;
        }
        dwelling_since_ = -1.0;
        turn_s_ = now;
        prm_.heading_deg = holdheading::wrap360(base_ + sector_ * (360.0 / n_));
      }
    }
    if (ctx_->publish_setpoints && ctx_->heading_speed) {
      ctx_->heading_speed(cmd.returning ? cmd.heading_deg : prm_.heading_deg,
        cmd.returning ? cmd.speed_mps : 0.0);
    }
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      Context::LegStatus & s = ctx_->leg;
      s = Context::LegStatus{};
      s.leaf = "SectorScan";
      s.name = "sector " + std::to_string(sector_ + 1) + "/" + std::to_string(n_);
      s.state = dwelling_since_ >= 0.0 ? (waiting.empty() ? "LOOKING" : "WAITING FOR COLOURS") :
        (returning_ ? "RETURNING" : "TURNING");
      s.why = "heading " + std::to_string(static_cast<int>(std::lround(prm_.heading_deg))) +
        (waiting.empty() ? std::string() : "; no colour yet: " + waiting);
      s.have_goal = true;
      s.goal = anchor_;
      ++ctx_->leg_seq;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    clearLeg(*ctx_);
  }

private:
  /// One line for the whole scan: what is mapped now, within range and beyond.
  void summary(const std::vector<nav::Buoy> & buoys, Vec2 boat, double now)
  {
    int near = 0, near_col = 0, far = 0;
    for (const nav::Buoy & b : buoys) {
      if (nav::norm(b.p - boat) <= range_m_) {
        ++near;
        if (b.state != nav::Beacon::Unknown) {++near_col;}
      } else {
        ++far;
      }
    }
    bool entry, exit_seen, exit_guessed;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      entry = ctx_->entry_seen;
      exit_seen = ctx_->exit_seen;
      exit_guessed = ctx_->exit_guessed;
    }
    RCLCPP_INFO(log(), "sector scan: done after %.0f s: %d/%d buoys within %.0f m coloured, %d "
      "beyond; ENTRY %s, EXIT %s", now - start_s_, near_col, near, range_m_, far,
      entry ? "seen" : "NOT SEEN", exit_seen ? "seen" : (exit_guessed ? "guessed" : "NOT SEEN"));
  }

  holdheading::Params prm_;
  Vec2 anchor_;
  std::map<int, std::pair<nav::Beacon, double>> seen_;   ///< id -> colour, since when
  std::map<int, double> waited_;   ///< id -> seconds in view with no colour, this scan
  double base_ = 0.0, half_view_deg_ = 36.0, range_m_ = 15.0, min_s_ = 4.0, max_s_ = 15.0;
  double stable_s_ = 1.0, colour_wait_s_ = 6.0, settle_deg_ = 12.0, turn_timeout_s_ = 15.0;
  double last_tick_s_ = -1.0;
  double start_s_ = 0.0, turn_s_ = 0.0, dwelling_since_ = -1.0, stale_s_ = 0.0;
  int n_ = 6, sector_ = 0;
  bool returning_ = false, stale_ = false;
};

/// The next vantage point round the operator's field centre, for NavigateTo target="port":
/// radius_m from the centre, the first on the line from the centre to where the boat is when
/// this first runs, the next ones offsets_deg round from it (in order; then round again).
/// Each call moves on one. FAILURE without a field centre.
class PickFieldVantage : public CrusaderSyncAction
{
public:
  PickFieldVantage(const std::string & n, const BT::NodeConfig & c) : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("radius_m", 12.0, "from the field centre (the field is ~8 x 13 m)"),
      BT::InputPort<std::string>("offsets_deg", "0,60,120,180,240,300",
        "round the centre, from the boat's side, in the order visited"),
      BT::OutputPort<Waypoint>("goal", "the vantage point")};
  }

  BT::NodeStatus tick() override
  {
    const double r = getInput<double>("radius_m").value_or(12.0);
    const std::vector<double> offs =
      parseDegrees(getInput<std::string>("offsets_deg").value_or("0,60,120,180,240,300"));
    Waypoint w;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      Vec2 c;
      if (!fieldCentre(*ctx_, &c) || !ctx_->pose_fresh) {
        RCLCPP_WARN(log(), "PickFieldVantage: %s", ctx_->pose_fresh ?
          "no field centre (set it on the GCS Run panel): cannot search round the field" :
          "no fresh pose");
        return BT::NodeStatus::FAILURE;
      }
      if (!have_base_) {
        base_ = nav::norm(ctx_->boat - c) > 1.0 ? nav::bearingDeg(c, ctx_->boat) : 180.0;
        have_base_ = true;
      }
      const double brg = base_ + offs[next_ % offs.size()];
      const Vec2 p = c + nav::headingVec(brg) * r;
      const nav::LatLon ll = nav::toLatLon(p, ctx_->origin);
      w.lat = ll.lat;
      w.lon = ll.lon;
      w.why = "vantage " + std::to_string(next_ + 1) + ": " + std::to_string(static_cast<int>(r)) +
        " m from the field centre, bearing " +
        std::to_string(static_cast<int>(std::lround(holdheading::wrap360(brg)))) + " from it";
    }
    ++next_;
    RCLCPP_INFO(log(), "search: %s", w.why.c_str());
    setOutput("goal", w);
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool have_base_ = false;
  double base_ = 0.0;
  std::size_t next_ = 0;
};

/// The ENTRY and the EXIT are known (seen, given, or the EXIT guessed from the GCS field centre):
/// WaitForField would return at once.
class FieldKnown : public CrusaderCondition
{
public:
  FieldKnown(const std::string & n, const BT::NodeConfig & c) : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return fieldKnown(*ctx_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

/// lat/lon hold a position: finite, on the globe, and not the 0, 0 a blank GCS field leaves.
/// Guards a NavigateTo target="fix" whose numbers come from the operator (it would drive to 0, 0).
class FixGiven : public CrusaderCondition
{
public:
  FixGiven(const std::string & n, const BT::NodeConfig & c) : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("lat", 0.0, "latitude"),
      BT::InputPort<double>("lon", 0.0, "longitude")};
  }

  BT::NodeStatus tick() override
  {
    const double la = getInput<double>("lat").value_or(0.0);
    const double lo = getInput<double>("lon").value_or(0.0);
    return latLonGiven(la, lo) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

/// Round the fixed field on the way to `target`. With nav_mode off every leg is a guarded straight
/// one, and a straight line through the Task 1 buoy field holds in front of the first buoy and
/// FAILS (Task 1 EXIT -> Task 3 approach point). t4::detour picks one point beside the field.
///
/// SUCCESS: drive to {via} first (the shortest boat -> via -> target that keeps margin_m from
///          every known buoy, fence and the dock, else 2 m, else nav_hard_m; inside the course).
/// FAILURE: go straight: the straight line already keeps that, or the target is not available,
///          or no point within 40 m clears the field (logged: the straight leg then holds in
///          front of it). Use it as ForceSuccess(Sequence(PickDetour, NavigateTo port {via})).
/// Task 4's moving keep-outs are not part of the field: AvoidInvaders handles them.
class PickDetour : public CrusaderSyncAction
{
public:
  PickDetour(const std::string & n, const BT::NodeConfig & c) : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target", "approach", "approach | fix | port"),
      BT::InputPort<double>("lat", 0.0, "with target=fix: latitude"),
      BT::InputPort<double>("lon", 0.0, "with target=fix: longitude"),
      BT::InputPort<Waypoint>("goal", "with target=port: a Waypoint from an upstream leaf"),
      BT::InputPort<double>("margin_m", 3.0, "clearance wanted from every buoy and the dock"),
      BT::OutputPort<Waypoint>("via", "the point to drive to first")};
  }

  BT::NodeStatus tick() override
  {
    const std::string which = getInput<std::string>("target").value_or("approach");
    const double margin = std::clamp(getInput<double>("margin_m").value_or(3.0), 0.5, 20.0);
    const double la = getInput<double>("lat").value_or(0.0);
    const double lo = getInput<double>("lon").value_or(0.0);
    const auto port = getInput<Waypoint>("goal");
    Vec2 boat, goal;
    std::vector<path::Hazard> field;
    std::vector<Vec2> course;
    std::vector<double> clears{margin};
    nav::LatLon origin;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!ctx_->origin_set || !ctx_->pose_fresh) {
        RCLCPP_WARN(log(), "PickDetour: no fresh pose: straight on");
        return BT::NodeStatus::FAILURE;
      }
      bool have = false;
      if (which == "approach" && ctx_->has_approach) {
        goal = nav::toLocal({ctx_->approach_lat, ctx_->approach_lon}, ctx_->origin);
        have = true;
      } else if (which == "fix" && latLonGiven(la, lo)) {
        goal = nav::toLocal({la, lo}, ctx_->origin);
        have = true;
      } else if (which == "port" && port) {
        goal = nav::toLocal({port.value().lat, port.value().lon}, ctx_->origin);
        have = true;
      }
      if (!have) {
        RCLCPP_WARN(log(), "PickDetour: target '%s' is not available", which.c_str());
        return BT::NodeStatus::FAILURE;
      }
      boat = ctx_->boat;
      origin = ctx_->origin;
      for (const path::Hazard & h : knownHazards(*ctx_)) {
        if (h.source != path::HazardSource::Keepout) {field.push_back(h);}
      }
      for (const nav::LatLon & ll : ctx_->course_corners) {
        course.push_back(nav::toLocal(ll, ctx_->origin));
      }
      if (margin > 2.0) {clears.push_back(2.0);}
      if (ctx_->nav.hard_m < clears.back()) {clears.push_back(ctx_->nav.hard_m);}
    }
    const t4::Detour d = t4::detour(boat, goal, field, clears, course);
    if (d.route == t4::Route::Straight) {
      RCLCPP_INFO(log(), "PickDetour %s: straight (%.0f m, keeps %.1f m of the field)",
        which.c_str(), d.len_m, d.clear_m);
      return BT::NodeStatus::FAILURE;
    }
    if (d.route == t4::Route::None) {
      RCLCPP_WARN(log(), "PickDetour %s: no point within 40 m takes the leg round the field "
        "(%zu hazards): straight on, and the leg holds in front of it", which.c_str(),
        field.size());
      return BT::NodeStatus::FAILURE;
    }
    const nav::LatLon ll = nav::toLatLon(d.via, origin);
    Waypoint w;
    w.lat = ll.lat;
    w.lon = ll.lon;
    char why[160];
    std::snprintf(why, sizeof(why), "round the field to %s: %.0f m instead of %.0f, %.1f m clear",
      which.c_str(), d.len_m, nav::norm(goal - boat), d.clear_m);
    w.why = why;
    RCLCPP_INFO(log(), "PickDetour: %s", why);
    setOutput("via", w);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace

void registerGlobalPassageNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<TransitComplete>("TransitComplete");
  factory.registerNodeType<WaitForField>("WaitForField");
  factory.registerNodeType<PlanGlobalPassage>("PlanGlobalPassage");
  factory.registerNodeType<DrivePlanPhase>("DrivePlanPhase");
  factory.registerNodeType<AnchorSeen>("AnchorSeen");
  factory.registerNodeType<PassageStarted>("PassageStarted");
  factory.registerNodeType<ScanForBuoys>("ScanForBuoys");
  factory.registerNodeType<SectorScan>("SectorScan");
  factory.registerNodeType<PickFieldVantage>("PickFieldVantage");
  factory.registerNodeType<FieldKnown>("FieldKnown");
  factory.registerNodeType<FixGiven>("FixGiven");
  factory.registerNodeType<PickDetour>("PickDetour");
}

}  // namespace crusader_bt
