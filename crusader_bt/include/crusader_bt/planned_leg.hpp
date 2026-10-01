// planned_leg.hpp — one leg of a mission, driven through a planner or straight.
//
// Pure: no ROS, no Nav2, no clock of its own. The leaf feeds it the world each
// tick (LegInputs) and applies what it says (LegOutput); everything that decides
// WHAT the boat is told to do lives here, so it can be pinned by test_planned_leg
// with a scripted fake planner and a manual clock. Spec: docs/nav2_avoidance_spec.md
// section 5.3 ("Leg behaviour" is the contract the tests pin).
//
// FOUR BEHAVIOURS, picked once at start() from (Mode, LegConfig::avoid):
//
//   A  Shadow with avoid=false           the legacy NavigateTo: send the goal,
//                                        resend when it moves, arrive on tolerance.
//   B  Off (avoid or not), or            A, plus a guard: hold if the straight
//      On with avoid=false               segment crosses a known hazard (minus the
//                                        exemptions); resume only once the line has
//                                        stayed clear for unblock_reset_s. No
//                                        planner calls, ever.
//   C  On, avoid=true                    plan, follow a carrot, re-plan, hold when
//                                        blocked, fail only after blocked_timeout_s.
//   D  Shadow, avoid=true                C run completely, every port call real, but
//                                        what is SENT is A's. Never a hold, never a
//                                        Failure: it shows what C would have done.
//
// OFF IS B, NOT A (2026-10-01, spec 5.5). Off is the boat's default until the asv
// container has Nav2, and the old run-in detour is gone, so an Off leg that merely
// drove the straight line would go through any black buoy or track on it. Off has
// no planner to go round a hazard with, so it HOLDS in front of one and FAILS after
// blocked_timeout_s: it never drives through. It needs no Nav2.
//
// THE RULES THAT MATTER MOST, because they are the ones that keep the boat off a
// buoy when something upstream is wrong:
//
//   * A stale pose, a NaN heading or a changed datum is DEGRADED: hold once, ask for
//     nothing, never FAIL. Nav2 itself would carry on with the last pose (spec 2:
//     isPathValid returns true when the pose lookup fails), so the BT is the only
//     thing that notices. The mission timeout and the guard band are the backstop.
//   * A planner that does not answer is BLIND, which is worse than one that found no
//     path. Blind -> BLOCKED even with a path in hand; "no path" -> keep following
//     a path that is still valid.
//   * ARRIVAL is judged on the TRUE goal, never on the carrot or the path's end: the
//     planner's goal was pushed out of a hazard and clipped into the window.
//
// Time is the caller's clock (LegInputs::now_s); `dt` is the gap between steps.
// Times are compared with a 1 us slack so "after 5.0 s" does not depend on whether
// fifty additions of 0.1 landed a hair either side of 5.
#ifndef CRUSADER_BT__PLANNED_LEG_HPP_
#define CRUSADER_BT__PLANNED_LEG_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "crusader_bt/path_math.hpp"
#include "crusader_bt/planner_port.hpp"

namespace crusader_bt
{
namespace path
{

enum class Mode {Off, Shadow, On};
enum class LegState {Idle, Straight, Planning, Following, Blocked, Degraded, Arrived, Failed};

inline const char * legStateName(LegState s)
{
  switch (s) {
    case LegState::Idle: return "IDLE";
    case LegState::Straight: return "STRAIGHT";
    case LegState::Planning: return "PLANNING";
    case LegState::Following: return "FOLLOWING";
    case LegState::Blocked: return "BLOCKED";
    case LegState::Degraded: return "DEGRADED";
    case LegState::Arrived: return "ARRIVED";
    case LegState::Failed: return "FAILED";
  }
  return "IDLE";
}

struct LegConfig
{
  bool avoid = true;
  double tolerance = 2.0;
  double resend_m = 1.5;
  double blocked_timeout_s = 15.0;
  std::vector<int> exempt_buoys;
  bool exempt_dock = false;
};

struct LegInputs
{
  double now_s = 0.0;
  bool pose_fresh = false;
  Vec2 boat;
  double heading_deg = std::nan("");
  bool goal_ok = false;
  Vec2 goal;
  std::vector<Hazard> hazards;
  bool datum_ok = true;
};

enum class Result {Running, Success, Failure};

struct LegOutput
{
  Result result = Result::Running;
  bool send = false;
  Vec2 setpoint;
  LegState state = LegState::Idle;
  std::string why;
  double blocked_s = 0.0;
  std::vector<std::string> log;
};

class PlannedLeg
{
public:
  PlannedLeg(const NavParams & p, Mode m, PlannerPort * port)    // port may be null (Off)
  : p_(p), mode_(m), port_(port) {}

  /// = onStart. Sends the goal (A, B) or issues the first plan request (C, D); never
  /// finishes the leg itself, as the legacy onStart never did.
  LegOutput start(const LegConfig & c, const LegInputs & in)
  {
    r_ = Run{};
    r_.cfg = c;
    r_.kind = classify(c);
    r_.shadow = mode_ == Mode::Shadow;
    r_.state = r_.kind == Kind::Planned ? LegState::Planning : LegState::Straight;
    r_.active = true;
    r_.last_now = r_.plan_wait_start = in.now_s;
    return tick(in, true);
  }

  /// = onRunning, each tick.
  LegOutput step(const LegInputs & in)
  {
    return r_.active ? tick(in, false) : LegOutput{};
  }

  /// Forget everything. In-flight replies are then ignored: no seq is pending.
  void halt() {r_ = Run{};}

  const std::vector<Vec2> & path() const {return r_.path;}
  bool hasTarget() const {return r_.have_target;}
  Vec2 target() const {return r_.target;}
  LegState state() const {return r_.state;}
  double planMs() const {return r_.plan_ms;}

private:
  enum class Kind {Straight, Guard, Planned};

  /// What one tick has found out so far: the clock, and the plan failures that
  /// the state step at the end of the tick has to act on.
  struct Tick
  {
    double now = 0.0, dt = 0.0;
    bool first = false, adopted = false;
    bool soft = false, blind = false;
    std::string fail_why;
  };

  struct Run
  {
    bool active = false, shadow = false;
    Kind kind = Kind::Straight;
    LegConfig cfg;
    LegState state = LegState::Idle;
    double last_now = 0.0, plan_wait_start = 0.0;

    Vec2 goal;                 // the true goal, re-resolved every tick
    Vec2 drive_goal;           // what the legacy send last named
    Vec2 last_fresh;           // the last position seen with a fresh pose
    Vec2 last_sent, target;
    bool have_fresh = false, have_sent = false, have_target = false, degraded = false;

    std::vector<Vec2> path;
    Vec2 path_goal;            // the true goal the ADOPTED path was planned for
    std::size_t closest = 0;
    bool path_clipped = false;
    bool escape = false;
    Vec2 escape_p;

    std::uint64_t plan_seq = 0;
    double plan_since = 0.0, last_req = -std::numeric_limits<double>::infinity();
    bool have_req = false, need_replan = false, req_clipped = false, req_escape = false;
    Vec2 req_goal, req_escape_p;

    std::uint64_t check_seq = 0;
    double check_since = 0.0, last_check = -std::numeric_limits<double>::infinity();
    int invalid_streak = 0;

    double blocked_s = 0.0, follow_s = 0.0, plan_ms = -1.0;
    double clear_since = 0.0;                 // a guarded leg: when its line was first seen clear
    bool have_clear = false;
    bool cleared = false;
    std::string why, last_fail_logged;
  };

  static bool reached(double x, double limit) {return x + 1e-6 >= limit;}

  /// A reply that answers the request we are waiting on. None and Pending are "no
  /// answer yet", and a reply for any other seq is somebody else's, however fresh.
  static bool answered(std::uint64_t pending, std::uint64_t seq, Reply st)
  {
    return seq == pending && st != Reply::Pending && st != Reply::None;
  }

  /// How far the last-sent point may sit from the end of the path before the end is sent.
  static constexpr double kEndResendM = 0.3;

  /// Which of A, B, C, D (see the header). Off has no planner, so `avoid` cannot ask
  /// it for anything and every Off leg is the guarded straight one (B).
  Kind classify(const LegConfig & c) const
  {
    if (mode_ == Mode::Off) {return Kind::Guard;}
    if (c.avoid) {return Kind::Planned;}
    return mode_ == Mode::On ? Kind::Guard : Kind::Straight;      // Shadow: A, guard logged only
  }

  // ------------------------------------------------------------- one tick

  LegOutput tick(const LegInputs & in, bool first)
  {
    Run & r = r_;
    Tick t;
    t.now = in.now_s;
    t.first = first;
    t.dt = first ? 0.0 : std::max(0.0, in.now_s - r.last_now);
    r.last_now = in.now_s;
    const LegState before = r.state;

    LegOutput o;
    if (r.state == LegState::Arrived) {
      o.result = Result::Success;                     // terminal states are sticky
    } else if (r.state == LegState::Failed) {
      o.result = Result::Failure;
    } else if (first && !in.goal_ok) {
      finish(o, LegState::Failed, Result::Failure, "goal not available");
    } else {
      if (in.goal_ok) {r.goal = in.goal;}
      if (r.kind == Kind::Planned) {plannedTick(in, t, o);} else {straightTick(in, t, o);}
    }

    o.state = r.state;
    o.blocked_s = r.blocked_s;
    o.why = r.shadow && r.kind == Kind::Planned &&
      (r.state == LegState::Blocked || r.state == LegState::Degraded) ?
      "shadow: would HOLD (" + r.why + ")" : r.why;
    if (r.state != before) {
      o.log.push_back(
        std::string("leg ") + legStateName(r.state) + (o.why.empty() ? "" : ": " + o.why));
    }
    return o;
  }

  // ----------------------------------------------------- shared small pieces

  void finish(LegOutput & o, LegState s, Result res, const std::string & why)
  {
    r_.state = s;
    r_.why = why;
    o.result = res;
  }

  void sendTo(LegOutput & o, Vec2 p)
  {
    o.send = true;
    o.setpoint = p;
    r_.last_sent = r_.target = p;
    r_.have_sent = r_.have_target = true;
  }

  /// A hold is the boat's own position, sent ONCE per episode. It clears the
  /// "already sent" memory so the first carrot after it is always sent, and the
  /// target: a hold has no target, and a stale one on the status line would lie.
  void holdAt(LegOutput & o, Vec2 at)
  {
    o.send = true;
    o.setpoint = at;
    r_.have_sent = r_.have_target = false;
  }

  void enterBlocked(LegOutput & o, Vec2 at, const std::string & why)
  {
    r_.state = LegState::Blocked;
    r_.why = why;
    r_.follow_s = 0.0;
    holdAt(o, at);
  }

  /// BLOCKED with the path given up on: the planner cannot vouch for it (blind) or
  /// the field says it is no good. A replan is wanted at once.
  void blockAndReplan(LegOutput & o, Vec2 at, const std::string & why)
  {
    dropPath();
    r_.need_replan = true;
    enterBlocked(o, at, why);
  }

  bool arrivedAt(Vec2 goal, const LegInputs & in) const
  {
    return in.pose_fresh && nav::norm(goal - in.boat) <= r_.cfg.tolerance;
  }

  /// The legacy goal rule (NavigateTo onStart/onRunning before WP2): the first
  /// call names the goal, later ones re-name it only when it moved more than
  /// resend_m. A setpoint republished every tick is a stream, not a leaf.
  bool legacyGoal(const LegInputs & in, bool first)
  {
    if (!in.goal_ok) {return false;}
    if (first || (r_.cfg.resend_m > 0.0 && nav::norm(in.goal - r_.drive_goal) > r_.cfg.resend_m)) {
      r_.drive_goal = in.goal;
      return true;
    }
    return false;
  }

  // ------------------------------------------------------------ A and B

  static constexpr const char * kLineBlocked = "a known hazard is on the line to the goal";

  /// The straight segment to the goal crosses a known hazard (minus the exemptions).
  bool lineBlocked(const LegInputs & in) const
  {
    return !segmentClear(
      in.boat, r_.drive_goal,
      exempt(in.hazards, r_.cfg.exempt_buoys, r_.cfg.exempt_dock, p_.exempt_radius_m),
      p_.hard_m - p_.local_check_tol_m);
  }

  void straightTick(const LegInputs & in, Tick & t, LegOutput & o)
  {
    Run & r = r_;
    const bool due = legacyGoal(in, t.first);
    if (!t.first && arrivedAt(r.drive_goal, in)) {
      finish(o, LegState::Arrived, Result::Success, "");
      return;
    }
    if (!in.pose_fresh) {
      r.have_clear = false;               // a stale pose has seen nothing: no clear time banked
    } else if (r.kind == Kind::Guard) {
      if (guardHolds(in, t, o, lineBlocked(in))) {return;}
    } else if (r.shadow) {                // Shadow: say so, never act
      r.why = lineBlocked(in) ? std::string("shadow: would HOLD (") + kLineBlocked + ")" : "";
    }
    if (due && r.state != LegState::Blocked) {sendTo(o, r.drive_goal);}
  }

  /// B's guard, once per tick with a fresh pose. True = the leg is held (or has
  /// failed) this tick and sends nothing more.
  ///
  /// A held leg resumes only when the line has stayed clear for unblock_reset_s, the
  /// same rule a planned leg lives by: a track that flickers on and off the line must
  /// not make the boat lurch at it on every clear tick. blocked_s counts for as long
  /// as the leg is BLOCKED, the line clear or not (the boat is held either way), and
  /// only the resume zeroes it, so a flicker still ends in FAILURE at blocked_timeout_s.
  bool guardHolds(const LegInputs & in, const Tick & t, LegOutput & o, bool blocked)
  {
    Run & r = r_;
    if (blocked) {
      r.have_clear = false;
      if (r.state != LegState::Blocked) {
        enterBlocked(o, in.boat, kLineBlocked);
      } else {
        r.blocked_s += t.dt;
        r.why = kLineBlocked;
      }
    } else if (r.state == LegState::Blocked) {
      if (!r.have_clear) {
        r.have_clear = true;
        r.clear_since = t.now;
      }
      r.blocked_s += t.dt;
      if (reached(t.now - r.clear_since, p_.unblock_reset_s)) {     // clear long enough: carry on
        r.state = LegState::Straight;
        r.why.clear();
        r.blocked_s = 0.0;
        r.have_clear = false;
        sendTo(o, r.drive_goal);
        return true;
      }
      r.why = "the line is clear; holding until it has stayed clear";
    } else {
      return false;                       // clear, and not held
    }
    if (reached(r.blocked_s, r.cfg.blocked_timeout_s)) {
      finish(o, LegState::Failed, Result::Failure, std::string("blocked: ") + kLineBlocked);
    }
    return true;
  }

  // ---------------------------------------------------------------- C and D

  /// D wraps C: C runs completely so the plans, checks and states are real, but
  /// the OUTPUT is the legacy one. Nothing C would have sent reaches the boat.
  void plannedTick(const LegInputs & in, Tick & t, LegOutput & o)
  {
    if (!r_.shadow) {planMachine(in, t, o); return;}
    LegOutput c;
    planMachine(in, t, c);
    o.log = c.log;
    const bool due = legacyGoal(in, t.first);
    if (!t.first && arrivedAt(r_.drive_goal, in)) {
      finish(o, LegState::Arrived, Result::Success, "");
    } else if (due) {
      o.send = true;
      o.setpoint = r_.drive_goal;
    }
  }

  const char * degradedReason(const LegInputs & in) const
  {
    if (!in.datum_ok) {return "datum changed: restart bt_runner";}
    if (!in.pose_fresh) {return "stale pose";}
    if (!std::isfinite(in.heading_deg)) {return "heading unknown (NaN)";}
    return nullptr;
  }

  void planMachine(const LegInputs & in, Tick & t, LegOutput & o)
  {
    Run & r = r_;
    if (in.pose_fresh && detail::finite(in.boat)) {
      r.have_fresh = true;
      r.last_fresh = in.boat;
    }

    // 0. Degraded: hold on entry only, ask for nothing, never fail.
    if (const char * why = degradedReason(in)) {
      if (!r.degraded) {
        r.degraded = true;
        if (r.have_fresh) {holdAt(o, r.last_fresh);}
      }
      r.state = LegState::Degraded;
      r.why = why;
      return;
    }
    if (r.degraded) {recover(t);}

    // 1. Arrival, on the TRUE goal. (Shadow judges it in plannedTick, the legacy way.)
    if (!r.shadow && !t.first && arrivedAt(r.goal, in)) {
      finish(o, LegState::Arrived, Result::Success, "");
      return;
    }

    // blocked_s grows only while BLOCKED, and the tick that ends the spell counts:
    // the boat WAS held for that dt. (DEGRADED returned above, so it never grows there.)
    if (r.state == LegState::Blocked) {r.blocked_s += t.dt;}
    readPlanReply(in, t);                                       // 2. replies
    if (r.state == LegState::Following) {checkPath(in, t, o);}  // 3. validity
    maybeRequest(in, t);                                        // 4. request
    stateStep(in, t, o);                                        // 5. states
  }

  /// Back from DEGRADED: the pose may have jumped and the path is stale, so forget
  /// it and plan again from PLANNING, with a fresh first_plan_wait.
  void recover(const Tick & t)
  {
    Run & r = r_;
    r.degraded = false;
    dropPath();
    r.plan_seq = 0;
    r.need_replan = true;
    r.state = LegState::Planning;
    r.why.clear();
    r.plan_wait_start = t.now;
    r.follow_s = 0.0;
    r.have_sent = false;
  }

  void dropPath()
  {
    Run & r = r_;
    r.path.clear();
    r.closest = 0;
    r.path_clipped = false;
    r.escape = false;
    r.invalid_streak = 0;
    r.check_seq = 0;
  }

  /// Keep the first failure of the strongest class: blind outranks soft.
  static void failPlan(Tick & t, bool blind, const std::string & why)
  {
    if (blind) {
      if (!t.blind) {t.blind = true; t.fail_why = why;}
    } else if (!t.blind && !t.soft) {
      t.soft = true;
      t.fail_why = why;
    }
  }

  double tol() const {return p_.hard_m - p_.local_check_tol_m;}

  /// The path still in front of the boat. While an escape is active the boat is
  /// INSIDE the hard band and the path starts at the escape point, so the whole
  /// path counts: projecting the boat onto it would start the check in the very
  /// hazard we are escaping and block the leg forever.
  std::vector<Vec2> ahead(const LegInputs & in) const
  {
    return r_.escape ? r_.path : detail::remainingPath(r_.path, in.boat, r_.closest);
  }

  // 2. ---------------------------------------------------------- replies

  void readPlanReply(const LegInputs & in, Tick & t)
  {
    Run & r = r_;
    if (r.plan_seq == 0 || port_ == nullptr) {return;}
    const PlanReply pr = port_->lastPlan();
    if (answered(r.plan_seq, pr.seq, pr.status)) {
      r.plan_seq = 0;
      r.plan_ms = pr.planning_s >= 0.0 ? pr.planning_s * 1000.0 : -1.0;
      if (pr.status == Reply::Ok) {
        adoptOrReject(pr, in, t);
      } else {
        failPlan(t, pr.status == Reply::Unavailable, pr.why.empty() ? "planner failed" : pr.why);
      }
    } else if (reached(t.now - r.plan_since, p_.plan_timeout_s)) {
      r.plan_seq = 0;                           // a late reply is dropped by seq from here on
      failPlan(t, true, "planner timeout (costmap not current?)");
    }
  }

  /// How far the goal may move from the one the adopted path was planned for before
  /// that path stops being "the path to the goal". The SMALLER of goal_replan_m and the
  /// tolerance: a path that ends within the tolerance of the goal still arrives, and
  /// one that does not must be replaced however the hysteresis scores the two.
  double retargetM() const {return std::min(p_.goal_replan_m, r_.cfg.tolerance);}

  /// The path in hand is worth keeping: valid against the known field, planned for the
  /// goal we have NOW, and not a window-clipped path that is nearly used up. THE
  /// SECOND AND THIRD TESTS ARE NOT IN THE SPEC AND ARE NEEDED.
  /// Goal: preferNew measures both paths to the CURRENT goal, and an old-goal path
  /// followed to its end scores |old end - goal| = d against a fresh plan's ~d, so the
  /// new plan would never win and the leg would sit at the old end, d from a goal it
  /// can never arrive at, until the mission timeout. Hysteresis is there to stop a good
  /// path to THIS goal being swapped for a marginally better one, nothing more.
  /// Window: for a goal beyond clip_radius, the plan after the boat has advanced is
  /// exactly as long (path + |end - goal|) as the one it replaces, so preferNew would
  /// never take it and the boat would stop at the end of the first window. A spent
  /// window path is replaced by the next plan, whatever it measures.
  bool pathWorthKeeping(const LegInputs & in) const
  {
    const Run & r = r_;
    if (r.path.empty()) {return false;}
    if (nav::norm(r.path_goal - r.goal) > retargetM()) {return false;}
    const std::vector<Vec2> rest = ahead(in);
    if (detail::pathBlocked(rest, in.hazards, tol())) {return false;}
    return !(r.path_clipped && pathLength(rest) < p_.lookahead_m);
  }

  void adoptOrReject(const PlanReply & pr, const LegInputs & in, Tick & t)
  {
    Run & r = r_;
    std::vector<Vec2> cand = pr.path;
    if (cand.empty()) {failPlan(t, false, "planner returned an empty path"); return;}
    // [escape?] + reply.path: the plan STARTED at the escape point, so the path
    // the boat follows must start there too (unless the planner kept it, to within
    // a costmap cell).
    if (r.req_escape && nav::norm(cand.front() - r.req_escape_p) > 0.05) {
      cand.insert(cand.begin(), r.req_escape_p);
    }
    if (detail::pathBlocked(cand, in.hazards, tol())) {
      failPlan(t, false, "plan crosses a known hazard");      // the costmap lagged the field
      return;
    }
    if (pathWorthKeeping(in)) {
      const std::vector<Vec2> rest = ahead(in);
      const double cur = pathLength(rest) + nav::norm(rest.back() - r.goal);
      const double now = pathLength(cand) + nav::norm(cand.back() - r.goal);
      if (!preferNew(true, cur, now, p_.hysteresis_frac, p_.hysteresis_m)) {return;}
    }
    r.path = std::move(cand);
    r.path_goal = r.req_goal;
    r.closest = 0;
    r.path_clipped = r.req_clipped;
    r.escape = r.req_escape;
    r.escape_p = r.req_escape_p;
    r.invalid_streak = 0;
    r.check_seq = 0;
    t.adopted = true;
    if (r.state == LegState::Planning || r.state == LegState::Blocked) {
      r.state = LegState::Following;
      r.why.clear();
      r.follow_s = 0.0;
    }
  }

  // 3. ------------------------------------------------ validity of the path

  void checkPath(const LegInputs & in, Tick & t, LegOutput & o)
  {
    Run & r = r_;
    r.closest = closestIndex(r.path, in.boat, r.closest);
    const std::vector<Vec2> rest = ahead(in);
    const char * bad = nullptr;
    if (detail::pathBlocked(rest, in.hazards, tol())) {bad = "a known hazard is on the path";}

    if (r.check_seq != 0 && port_ != nullptr) {
      const CheckReply cr = port_->lastCheck();
      if (answered(r.check_seq, cr.seq, cr.status)) {
        r.check_seq = 0;
        if (cr.status == Reply::Ok) {r.invalid_streak = cr.valid ? 0 : r.invalid_streak + 1;}
      } else if (reached(t.now - r.check_since, p_.plan_timeout_s)) {
        r.check_seq = 0;                        // a lost reply must not stop checking for good
      }
    }
    if (bad == nullptr && r.invalid_streak >= std::max(1, p_.invalid_confirm)) {
      bad = "the planner says the path is no longer valid";
    }
    if (bad != nullptr) {
      blockAndReplan(o, in.boat, bad);
      return;
    }
    if (r.check_seq == 0 && port_ != nullptr && reached(t.now - r.last_check, p_.check_period_s)) {
      r.last_check = r.check_since = t.now;
      r.check_seq = port_->requestCheck(rest);
    }
  }

  // 4. ---------------------------------------------------------- requests

  void maybeRequest(const LegInputs & in, Tick & t)
  {
    Run & r = r_;
    if (r.have_req && nav::norm(r.goal - r.req_goal) > p_.goal_replan_m) {r.need_replan = true;}
    if (r.plan_seq != 0) {return;}
    const bool want = r.need_replan || r.path.empty() ||
      reached(t.now - r.last_req, p_.replan_period_s);
    if (!want || !reached(t.now - r.last_req, p_.min_request_gap_s)) {return;}

    r.last_req = t.now;
    if (port_ == nullptr || !port_->ready()) {
      failPlan(t, true, "planner_server not available");
      return;
    }
    const Moved m = pushGoalOut(r.goal, in.boat, in.hazards, p_.hard_m, p_.goal_margin_m);
    const Vec2 g = clipToWindow(m.p, in.boat, p_.clip_radius_m);
    const Escape e = escapeStart(in.boat, g, in.hazards, p_);
    const bool use_escape = e.needed && e.ok;
    const std::uint64_t seq = port_->requestPlan(use_escape ? e.p : in.boat, g);
    if (seq == 0) {
      failPlan(t, true, "plan request not sent");
    } else {
      r.plan_seq = seq;
      r.plan_since = t.now;
      r.have_req = true;
      r.req_goal = r.goal;
      r.need_replan = false;
      r.req_clipped = nav::norm(m.p - in.boat) > p_.clip_radius_m;
      r.req_escape = use_escape;
      r.req_escape_p = e.p;
    }
    if (e.needed && !e.ok) {failPlan(t, false, "boxed in");}
  }

  // 5. ------------------------------------------------------------- states

  void stateStep(const LegInputs & in, Tick & t, LegOutput & o)
  {
    Run & r = r_;
    if (r.state == LegState::Planning) {
      if (t.blind || t.soft) {
        enterBlocked(o, in.boat, t.fail_why);
      } else if (reached(t.now - r.plan_wait_start, p_.first_plan_wait_s)) {
        enterBlocked(o, in.boat, "no plan within first_plan_wait_s");
      }
    } else if (r.state == LegState::Following) {
      if (t.blind) {
        blockAndReplan(o, in.boat, t.fail_why);     // a blind planner cannot vouch for our path
      } else {
        if (t.soft && t.fail_why != r.last_fail_logged) {
          o.log.push_back("plan failed: " + t.fail_why + " (keeping the current path)");
        }
        follow(in, t, o);
      }
    }
    if (t.soft || t.blind) {r.last_fail_logged = t.fail_why;}
    if (r.state == LegState::Blocked) {blockedStep(o);}
  }

  /// blocked_s counts from the tick AFTER BLOCKED was entered (planMachine adds dt);
  /// a plan arriving does not reset it, only unblock_reset_s of continuous
  /// FOLLOWING does, so a flip-flop between the two still times out.
  void blockedStep(LegOutput & o)
  {
    Run & r = r_;
    if (!r.cleared && reached(r.blocked_s, p_.clear_after_s)) {
      r.cleared = true;
      if (port_ != nullptr) {port_->clearCostmap();}
      o.log.push_back(
        "costmap cleared: blocked for " + std::to_string(std::lround(r.blocked_s)) + " s");
    }
    if (!r.shadow && reached(r.blocked_s, r.cfg.blocked_timeout_s)) {
      finish(o, LegState::Failed, Result::Failure, "blocked: " + r.why);
    }
  }

  void follow(const LegInputs & in, const Tick & t, LegOutput & o)
  {
    Run & r = r_;
    if (!t.adopted) {r.follow_s += t.dt;}
    if (reached(r.follow_s, p_.unblock_reset_s)) {
      r.blocked_s = 0.0;
      r.cleared = false;
    }
    if (r.escape) {
      if (minClearance(in.hazards, in.boat) < p_.hard_m) {
        if (!r.have_sent || nav::norm(r.escape_p - r.last_sent) > 1e-6) {sendTo(o, r.escape_p);}
        return;                  // wp_radius + 0.5 away: ArduRover does not count it as reached
      }
      r.escape = false;
    }
    const Carrot c = carrot(r.path, in.boat, r.closest, in.hazards, p_);
    // The third clause: an intermediate carrot the boat is about to reach would be
    // counted as reached by ArduRover (WP_RADIUS) and the boat would loiter there.
    // The fourth is NOT in the spec and is needed: the end of the path is the
    // pushed-out goal, up to ~1.4 m short of the true one, and the true goal is what
    // arrival is judged on. Behind a resend_m deadband the last intermediate carrot
    // can sit 1.1 m short of the end, the boat parks on it 2.5 m from the true goal,
    // and the leg waits forever. The end is therefore always sent, to within 0.3 m.
    if (!r.have_sent || nav::norm(c.p - r.last_sent) > r.cfg.resend_m ||
      (!c.is_end && nav::norm(in.boat - r.last_sent) < p_.wp_radius_m + 0.5) ||
      (c.is_end && nav::norm(c.p - r.last_sent) > kEndResendM))
    {
      sendTo(o, c.p);
    }
  }

  NavParams p_;
  Mode mode_;
  PlannerPort * port_;
  Run r_;
};

}  // namespace path
}  // namespace crusader_bt

#endif  // CRUSADER_BT__PLANNED_LEG_HPP_
