// global_leaves.cpp — the leaves of the whole-field Task 1 trees (behavior_trees/task1_global.xml).
//
// Nothing here is geometry. The plan is global_passage.hpp, the driving is path_follower.hpp,
// both pinned off the boat by test/test_global_passage.cpp (closed loop included). This file
// reads the context, calls them, sends setpoints, keeps the mission's phase and says what it
// did.
//
// THE MODEL. One plan covers the rest of the mission from the boat: approach, entry orbit
// (clockwise), transit (every red to starboard, every green to port), exit orbit (counter-
// clockwise). ctx.global_passage.phase says which of the four is being driven. A plan is KEPT
// while it is still good, and re-made from the boat when it is not:
//   * there is none, or the field changed (a new plan_version: the aircraft re-tasked);
//   * the boat is not near the start of what is left of the leg it is about to drive;
//   * a hazard now sits closer to what is left than the plan allowed (fusion moved a buoy,
//     or a LiDAR/camera track appeared on it).
// That is the whole re-tasking mechanism, Advanced and Disruptive alike. Advanced plans once
// and keeps it; Disruptive also stops at checkpoints and asks, and replans on any change.
//
// NO NAV2. These leaves never call the planner_server: the boat follows its own plan with
// GUIDED setpoints, so they behave the same in every nav_mode (off on the boat today).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

#include "crusader_bt/context.hpp"
#include "crusader_bt/global_passage.hpp"
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

/// The field is there to plan with: the aircraft's ENTRY and EXIT, a frame and a boat.
/// CALL UNDER ctx.mu.
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

/// The aircraft's latest report, as a plan records it. CALL UNDER ctx.mu.
gp::FieldSig currentField(const Context & c)
{
  return gp::fieldSig(c.plan, c.entry, c.exitp);
}

/// "" while the aircraft's field is the one the plan was made against (gp::fieldChange:
/// colours, buoys, or a move beyond retask_move_m); otherwise what changed. CALL UNDER ctx.mu.
std::string retasked(const Context & c)
{
  const Context::GlobalPassage & g = c.global_passage;
  if (!g.plan.ok) {return "";}
  return gp::fieldChange(g.field, currentField(c), g.params.retask_move_m);
}

/// "" when the plan in hand can still drive `phase`; otherwise why not. CALL UNDER ctx.mu.
std::string planUnusable(const Context & c, Phase phase)
{
  const Context::GlobalPassage & g = c.global_passage;
  if (!g.plan.ok) {return "no plan yet";}
  const std::string change = retasked(c);
  if (!change.empty()) {return "the aircraft's field changed: " + change;}
  const std::vector<Vec2> left = legLeft(c, phase);
  if (left.empty()) {return "the plan has no " + std::string(gp::phaseName(phase)) + " leg";}
  // near the start of what is left (a pass-through hand-over ends ~2 m short of it)
  const std::vector<Vec2> head = gp::slice(left, 0.0, 3.0);
  double d = kInf;
  for (const Vec2 & q : head) {d = std::min(d, nav::norm(q - c.boat));}
  if (d > 2.5) {return "the boat is " + gp::detail::fmt(d, 1) + " m off the plan";}
  const double clear = gp::polylineClearance(left, planHazards(c));
  if (clear < g.plan.hard_used - 0.05) {
    return "a hazard is now " + gp::detail::fmt(clear, 2) + " m from the plan";
  }
  return "";
}

/// Plan the rest of the mission from the boat, from `from` on, and adopt it if it worked.
/// The search runs WITHOUT the lock (tens to a few hundred ms); its inputs and its result
/// cross under it.
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

/// Hold where the boat is: its own position as the setpoint, once per hold.
void holdHere(Context & c, const rclcpp::Logger & lg, const char * why)
{
  Vec2 here;
  {
    std::lock_guard<std::mutex> lk(c.mu);
    if (!c.pose_fresh) {return;}
    here = c.boat;
  }
  sendSetpoint(c, lg, here, std::string("hold (") + why + ") at");
}

// ---------------------------------------------------------------- conditions

/// The goal's tier: "core" | "advanced" | "disruptive" (or 0 | 1 | 2).
class TierIs : public CrusaderCondition
{
public:
  TierIs(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("tier", "disruptive", "core | advanced | disruptive")};
  }

  BT::NodeStatus tick() override
  {
    const std::string t = getInput<std::string>("tier").value_or("disruptive");
    const int want = (t == "core" || t == "0") ? 0 : (t == "advanced" || t == "1") ? 1 : 2;
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->tier == want ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

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

/// The last transit stretch stopped at a gate's checkpoint (not at the end of the transit).
class AtCheckpoint : public CrusaderCondition
{
public:
  AtCheckpoint(const std::string & n, const BT::NodeConfig & c)
  : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->global_passage.at_checkpoint ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// ------------------------------------------------------------------- actions

/// Waits (RUNNING) until there is a field to plan with: the aircraft's ENTRY and EXIT, a
/// frame and a fresh pose. No timeout of its own: the mission timeout is the backstop, and
/// handbook 3.3.2 says the boat may not transit before the aircraft has reported.
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

/// Makes sure there is a plan for the rest of the mission from the current phase: keeps the
/// one in hand while it is good (planUnusable), re-plans from the boat when it is not. Sets the
/// planner's parameters for every later replan, too.
///
/// Cannot plan: HOLDS the boat where it is, tries again every retry_s (the field may change,
/// fusion may move a buoy), FAILURE after timeout_s. Never drives on a guess.
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
      BT::InputPort<double>("relax2_m", 0.6, "... and then this"),
      BT::InputPort<double>("res_m", 0.1, "planning grid, metres"),
      BT::InputPort<double>("retask_move_m", 2.0,
        "a re-reported buoy that moved less than this is the aircraft's scatter, not a re-task"),
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
    p.relax2_m = getInput<double>("relax2_m").value_or(0.6);
    p.res_m = getInput<double>("res_m").value_or(0.1);
    p.retask_move_m = getInput<double>("retask_move_m").value_or(2.0);
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
      why = planUnusable(*ctx_, phase);
    }
    if (why.empty()) {return BT::NodeStatus::SUCCESS;}           // the plan in hand is good
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

/// Drives one phase of the plan (or, for the transit with stop_at_checkpoints, the stretch up
/// to the next gate's checkpoint).
///
///   * Already past this phase: SUCCESS at once (so a Retry round the whole sequence resumes
///     where the mission is). Not yet at it: FAILURE (a wiring error).
///   * Starts from the plan in hand if it is still usable for this phase, else re-plans first.
///   * Every tick: the follower's setpoint; the transit's track is recorded (its parity is
///     what a mid-transit replan starts from).
///   * The field changes (Disruptive re-task) during the approach or the transit: re-plans
///     from the boat AT ONCE and carries on. During an orbit: the orbit carries on (it does
///     not depend on colours) and the next phase picks the change up.
///   * A hazard sits closer to the path ahead than the plan allowed: re-plans from the boat
///     (at most every 2 s) and carries on; if the path ahead is BLOCKED (closer than
///     hard - local_check_tol), HOLDS, re-plans every second, resumes once a clear plan has
///     stayed clear for nav_unblock_reset_s, FAILS after blocked_timeout_s.
///   * Arrives: approach and entry orbit hand over without stopping (pass-through) unless
///     stop_at_end; the transit's checkpoint stretches and the exit orbit stop on the point.
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
      BT::InputPort<bool>("stop_at_checkpoints", false,
        "transit only: end each stretch at the next gate's checkpoint (Disruptive)"),
      BT::InputPort<bool>("stop_at_end", false,
        "stop on the leg's last point instead of handing over to the next leg while moving"),
      BT::InputPort<double>("blocked_timeout_s", 15.0, "FAILURE after being blocked this long"),
      BT::InputPort<double>("lookahead_m", 1.5, "pure pursuit lookahead along the path"),
      BT::InputPort<double>("resend_m", 0.3, "re-send the setpoint once it has moved this far"),
      BT::InputPort<double>("end_tol_m", 0.8, "arrival radius at a stop")};
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
      ctx_->global_passage.at_checkpoint = false;
      last_now_ = ctx_->now_s;
    }
    blocked_ = false;
    blocked_s_ = 0.0;
    clear_since_ = -1.0;
    degraded_ = false;
    last_replan_ = -kInf;
    last_log_ = -kInf;
    follower_.reset({});
    rest_.clear();
    cp_red_ = cp_green_ = -1;
    if (!startLeg(true)) {
      enterBlocked("no plan for this leg");
      last_replan_ = last_now_;
    } else {
      const std::string to = cp_red_ < 0 ? std::string() : " to the checkpoint after gate red " +
        std::to_string(cp_red_) + " / green " + std::to_string(cp_green_);
      RCLCPP_INFO(log(), "%s: %.1f m to drive%s", gp::phaseName(phase_), follower_.total(), to.c_str());
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
      hz = planHazards(*ctx_);
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

    // 1. The aircraft re-tasked: re-plan the approach or the transit from here, now. An orbit
    //    carries on (it does not depend on colours); the next phase picks the change up.
    if (replannable && now - last_replan_ >= 1.0) {
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
      (replannable && !fieldChanged().empty()) ? "no plan fits the changed field" :
      o.blocked ? "a hazard is on the path ahead" : nullptr;

    // 3. Stuck: hold, re-plan every second, FAILURE at blocked_timeout_s.
    if (stuck != nullptr) {
      if (!blocked_) {enterBlocked(stuck);}
      blocked_s_ += dt;
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
    // 4. Was stuck, clear now: drive on only once it has STAYED clear (a flickering track
    //    must not make the boat lurch at it), still counting toward the timeout meanwhile.
    if (blocked_) {
      blocked_s_ += dt;
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

    // 5. Not blocked, but the water ahead shrank since this path was planned: the boat's own
    //    camera has moved a buoy from where the aircraft put it (its fix is good to ~1 m),
    //    or a track appeared. Re-plan from here, without stopping, so the path re-centres on
    //    the better positions instead of waiting until it is nearly blocked.
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
  /// Load what is left of this phase's leg into the follower (re-planning first when the plan
  /// in hand will not do and `allow_replan`). False = no usable leg.
  bool startLeg(bool allow_replan)
  {
    std::string why;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      why = planUnusable(*ctx_, phase_);
    }
    if (!why.empty()) {
      if (!allow_replan || !replan(*ctx_, phase_, log(), why)) {return false;}
    }
    const bool stop_cp = getInput<bool>("stop_at_checkpoints").value_or(false);
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
      const double total = gp::arcLengths(leg).back();
      double end = total;
      cp_red_ = cp_green_ = -1;
      if (phase_ == Phase::Transit) {
        if (g.traj.empty()) {g.traj.push_back(leg.front());}
        for (const gp::Checkpoint & cp : g.plan.checkpoints) {
          if (cp.s <= offset_ + 0.5) {continue;}
          if (stop_cp) {
            end = cp.s;
            cp_red_ = cp.red_id;
            cp_green_ = cp.green_id;
          }
          break;
        }
      }
      path = leg.size() < 2 ? leg : gp::slice(leg, offset_, end);
      stop_ = stop_end || cp_red_ >= 0 || phase_ == Phase::ExitOrbit;
      rest_.clear();
      for (int k = static_cast<int>(phase_) + 1; k < 4; ++k) {
        const std::vector<Vec2> & l = g.plan.legs[k];
        rest_.insert(rest_.end(), l.begin(), l.end());
      }
      if (end < total && leg.size() >= 2) {
        const std::vector<Vec2> tail = gp::slice(leg, end, total);
        rest_.insert(rest_.begin(), tail.begin(), tail.end());
      }
      fp_.lookahead_m = getInput<double>("lookahead_m").value_or(1.5);
      fp_.resend_m = getInput<double>("resend_m").value_or(0.3);
      fp_.end_tol_m = getInput<double>("end_tol_m").value_or(0.8);
      fp_.pass_early_m = stop_ ? 0.0 : fp_.lookahead_m + 0.5;
      fp_.check_clear_m = std::min(ctx_->nav.hard_m - ctx_->nav.local_check_tol_m, plan_hard_ - 0.1);
    }
    follower_.setParams(fp_);
    follower_.reset(std::move(path));
    // the water every vertex had when this leg was loaded: shrunk() compares against it
    std::vector<path::Hazard> hz;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      hz = planHazards(*ctx_);
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

  /// Re-plan the rest of the mission from the boat now and load this phase's new leg.
  /// True = the follower has a new leg.
  bool replanHere(double now, const std::string & why)
  {
    last_replan_ = now;
    return replan(*ctx_, phase_, log(), why) && startLeg(false);
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

  /// What changed in the aircraft's field since the plan in hand was made ("" = nothing).
  std::string fieldChanged()
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return retasked(*ctx_);
  }

  void enterBlocked(const char * why)
  {
    blocked_ = true;
    clear_since_ = -1.0;
    holdHere(*ctx_, log(), why);
    follower_.forgetSent();
    RCLCPP_WARN(log(), "%s: %s - holding, re-planning every second", gp::phaseName(phase_), why);
  }

  /// The transit as driven, every 0.2 m. CALL UNDER ctx.mu.
  void recordTrack(Vec2 boat)
  {
    std::vector<Vec2> & tr = ctx_->global_passage.traj;
    if (tr.empty() || nav::norm(boat - tr.back()) >= 0.2) {tr.push_back(boat);}
  }

  /// Gates the boat has driven through count as cleared, stop or no stop (the report's
  /// gates_cleared, and a replan never puts a checkpoint behind the boat).
  void markGatesPassed()
  {
    if (phase_ != Phase::Transit) {return;}
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const double at = offset_ + follower_.s();
    for (const gp::Checkpoint & cp : ctx_->global_passage.plan.checkpoints) {
      if (cp.s > at || (cp.red_id == cp_red_ && cp.green_id == cp_green_)) {continue;}
      if (!nav::gateCleared(ctx_->cleared_gates, cp.red_id, cp.green_id)) {
        ctx_->cleared_gates.push_back({cp.red_id, cp.green_id});
        RCLCPP_INFO(log(), "through gate red %d / green %d", cp.red_id, cp.green_id);
      }
    }
  }

  BT::NodeStatus finish(const gp::FollowOut & o)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    Context::GlobalPassage & g = ctx_->global_passage;
    if (phase_ == Phase::Transit && cp_red_ >= 0) {
      g.transit_s = offset_ + o.total;
      g.at_checkpoint = true;
      if (!nav::gateCleared(ctx_->cleared_gates, cp_red_, cp_green_)) {
        ctx_->cleared_gates.push_back({cp_red_, cp_green_});
      }
      ctx_->gate_red_id = cp_red_;
      ctx_->gate_green_id = cp_green_;
      ctx_->gate_cleared = true;
      RCLCPP_INFO(log(), "through gate red %d / green %d: at its checkpoint (%zu cleared)",
        cp_red_, cp_green_, ctx_->cleared_gates.size());
      return BT::NodeStatus::SUCCESS;
    }
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
  int cp_red_ = -1, cp_green_ = -1;
  bool stop_ = false;
  bool blocked_ = false, degraded_ = false;
  double blocked_s_ = 0.0, clear_since_ = -1.0;
  double last_now_ = 0.0, last_replan_ = -kInf, last_log_ = -kInf;
};

}  // namespace

void registerGlobalPassageNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<TierIs>("TierIs");
  factory.registerNodeType<TransitComplete>("TransitComplete");
  factory.registerNodeType<AtCheckpoint>("AtCheckpoint");
  factory.registerNodeType<WaitForField>("WaitForField");
  factory.registerNodeType<PlanGlobalPassage>("PlanGlobalPassage");
  factory.registerNodeType<DrivePlanPhase>("DrivePlanPhase");
}

}  // namespace crusader_bt
