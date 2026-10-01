// test_planned_leg — prove the leg state machine with a scripted planner and a manual clock.
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include -o /tmp/t test/test_planned_leg.cpp && /tmp/t
//
// No ROS, no Nav2, no real time. FakePlannerPort answers (or does not) exactly as
// the test says, and Rig owns the clock: a tick is 100 ms unless a test says otherwise,
// kept as integer milliseconds so "at 5.0 s" is not a question about floating point.
// The numbered sections are docs/nav2_avoidance_spec.md section 5.9's list for this
// file, in its order; 15+ are the edges that list leaves to the implementation:
// 17 a moved goal is re-targeted (not hysteresis'd away), 18 a guarded straight leg
// needs its line to stay CLEAR before it resumes, 19 Off mode is guarded too
// (spec 5.5, 2026-10-01).
//
// What these pin is what keeps the boat off a buoy when something upstream is wrong:
// a planner that never answers, a pose that goes stale, a heading that goes NaN, a
// plan that crosses a hazard the costmap has not heard of yet.
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "crusader_bt/planned_leg.hpp"

using namespace crusader_bt;         // NOLINT(build/namespaces) — a test
using namespace crusader_bt::path;   // NOLINT(build/namespaces)
using nav::Vec2;

static int g_fails = 0;
static int g_checks = 0;

static void chk(const std::string & name, bool pass)
{
  ++g_checks;
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s\n", pass ? "ok" : "FAIL", name.c_str());
}

static void chk_near(const std::string & name, double got, double want, double tol)
{
  const bool pass = std::fabs(got - want) <= tol;
  ++g_checks;
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s\n", pass ? "ok" : "FAIL", name.c_str());
  if (!pass) {std::printf("   (got %.6f, want %.6f)\n", got, want);}
}

static bool same(Vec2 a, Vec2 b) {return a.x == b.x && a.y == b.y;}
static bool near2(Vec2 a, Vec2 b, double tol = 1e-6) {return nav::norm(a - b) <= tol;}

static Hazard buoy(int id, double x, double y, HazardSource src = HazardSource::PlanBuoy)
{
  Hazard h;
  h.kind = HazardKind::Circle;
  h.source = src;
  h.id = id;
  h.c = {x, y};
  h.r = 0.30;
  return h;
}

// --------------------------------------------------------------- the fakes

/// Answers when the test says so. auto_plan = reply Ok at once with a straight
/// [start, goal]; otherwise the request stays Pending until the test edits `plan`.
class FakePlannerPort : public PlannerPort
{
public:
  bool is_ready = true;
  bool auto_plan = true;
  bool auto_check = true;
  bool check_valid = true;
  std::uint64_t nplan = 0, ncheck = 0;
  int nclear = 0, ready_calls = 0;
  std::vector<std::pair<Vec2, Vec2>> reqs;        // (start, goal) of every plan request
  std::vector<std::vector<Vec2>> checks;          // the path of every validity request
  PlanReply plan;
  CheckReply check;

  bool ready() override {++ready_calls; return is_ready;}

  std::uint64_t requestPlan(Vec2 s, Vec2 g) override
  {
    reqs.push_back({s, g});
    plan = PlanReply{};
    plan.seq = ++nplan;
    plan.planning_s = 0.02;
    plan.status = auto_plan ? Reply::Ok : Reply::Pending;
    if (auto_plan) {plan.path = {s, g};}
    return plan.seq;
  }
  PlanReply lastPlan() override {return plan;}

  std::uint64_t requestCheck(const std::vector<Vec2> & p) override
  {
    checks.push_back(p);
    check = CheckReply{};
    check.seq = ++ncheck;
    check.status = auto_check ? Reply::Ok : Reply::Pending;
    check.valid = check_valid;
    return check.seq;
  }
  CheckReply lastCheck() override {return check;}

  void clearCostmap() override {++nclear;}

  /// Answer the request now pending, as the planner would.
  void answer(Reply st, std::vector<Vec2> path = {}, const std::string & why = "")
  {
    plan.status = st;
    plan.path = std::move(path);
    plan.why = why;
  }
};

struct Sent
{
  int ms;
  Vec2 p;
  bool hold;
};

/// A leg, its fake planner and a clock. `in` is the world the leaf would hand it.
class Rig
{
public:
  FakePlannerPort port;
  PlannedLeg leg;
  LegInputs in;
  LegOutput out;
  std::vector<Sent> sends;
  int ms = 0;
  int failures = 0;
  double max_blocked = 0.0;
  static constexpr double kT0 = 100.0;

  explicit Rig(Mode m, const NavParams & p = NavParams{}) : leg(p, m, &port)
  {
    in.now_s = kT0;
    in.pose_fresh = true;
    in.heading_deg = 90.0;
    in.goal_ok = true;
    in.boat = {0, 0};
    last_fresh_ = in.boat;
  }

  void note()
  {
    if (out.result == Result::Failure) {++failures;}
    max_blocked = std::max(max_blocked, out.blocked_s);
    if (out.send) {sends.push_back({ms, out.setpoint, same(out.setpoint, last_fresh_)});}
  }
  const LegOutput & start(const LegConfig & c)
  {
    if (in.pose_fresh) {last_fresh_ = in.boat;}
    out = leg.start(c, in);
    note();
    return out;
  }
  const LegOutput & tick(int dt_ms = 100)
  {
    ms += dt_ms;
    in.now_s = kT0 + ms / 1000.0;
    if (in.pose_fresh) {last_fresh_ = in.boat;}
    out = leg.step(in);
    note();
    return out;
  }
  /// Tick until t = `seconds` after the start, calling `each` after every tick.
  void runTo(double seconds, const std::function<void()> & each = nullptr)
  {
    while (ms + 50 < static_cast<int>(std::lround(seconds * 1000.0))) {
      tick();
      if (each) {each();}
    }
  }
  int holds() const
  {
    int n = 0;
    for (const Sent & s : sends) {n += s.hold ? 1 : 0;}
    return n;
  }
  /// The boat flies toward the last non-hold setpoint at `speed` m/s.
  void fly(double speed)
  {
    for (auto it = sends.rbegin(); it != sends.rend(); ++it) {
      if (it->hold) {return;}
      const Vec2 d = it->p - in.boat;
      const double step = speed * 0.1;
      in.boat = nav::norm(d) <= step ? it->p : in.boat + nav::unit(d) * step;
      return;
    }
  }
  /// One tick, then the boat flies on.
  void advance(double speed = 3.0)
  {
    tick();
    fly(speed);
  }
  /// Advance until `done()` (checked before each tick) or `max_ticks`; `before(i)`, if
  /// given, edits the world ahead of tick i, as a tree rewriting {goal} would.
  void driveUntil(
    const std::function<bool()> & done, int max_ticks,
    const std::function<void(int)> & before = nullptr, double speed = 3.0)
  {
    for (int i = 0; i < max_ticks && !done(); ++i) {
      if (before) {before(i);}
      advance(speed);
    }
  }

private:
  Vec2 last_fresh_;
};

int main()
{
  // ------------------------------------------- 1. Off: the legacy send sequence
  std::printf("1. Off mode, on a clear line, reproduces the legacy NavigateTo send sequence\n");
  {
    Rig r(Mode::Off);
    r.in.goal = {50, 0};
    const LegConfig c;       // avoid defaults to true: meaningless in Off
    LegOutput o = r.start(c);
    chk("start sends the goal", o.send && same(o.setpoint, {50, 0}) && o.result == Result::Running);
    chk("the state is STRAIGHT", o.state == LegState::Straight && std::string(legStateName(o.state)) == "STRAIGHT");
    r.in.goal = {51, 0};
    chk("a 1.0 m goal move does not resend (resend_m 1.5)", !r.tick().send);
    r.in.goal = {52, 0};
    o = r.tick();
    chk("a 2.0 m move does", o.send && same(o.setpoint, {52, 0}));
    chk("... and nothing is resent after it", !r.tick().send);
    r.in.boat = {51, 0};
    o = r.tick();
    chk("arrival within tolerance: Success", o.result == Result::Success);
    chk("... reported as ARRIVED", o.state == LegState::Arrived);
    chk("Off mode never touches the planner", r.port.reqs.empty() && r.port.ready_calls == 0 &&
      r.port.ncheck == 0 && r.port.nclear == 0);

    Rig stale(Mode::Off);
    stale.in.goal = {50, 0};
    stale.start(c);
    stale.in.boat = {50, 0};
    stale.in.pose_fresh = false;
    chk("a stale pose never arrives (Running)", stale.tick().result == Result::Running);

    Rig haz(Mode::Off);
    haz.in.goal = {50, 0};
    haz.in.hazards = {buoy(1, 25, 5.0)};         // well off the line (section 19: ON the line)
    haz.start(c);
    haz.runTo(5.0);
    chk("Off mode: a hazard well off the line changes nothing, only the goal is sent",
      haz.sends.size() == 1 && haz.holds() == 0 && haz.out.state == LegState::Straight);
    chk("a Straight leg in Off never fails on its own", haz.failures == 0);
  }

  // ------------------------------------- 2. On: plan, follow, resend, true goal
  std::printf("2. On: a plan arrives, FOLLOWING, the carrot 5 m ahead\n");
  {
    Rig r(Mode::On);
    r.in.goal = {30, 0};
    const LegConfig c;
    LegOutput o = r.start(c);
    chk("start: PLANNING, one request out, nothing sent yet",
      o.state == LegState::Planning && r.port.reqs.size() == 1 && !o.send);
    chk("the request: start = the boat, goal = the goal",
      same(r.port.reqs[0].first, {0, 0}) && same(r.port.reqs[0].second, {30, 0}));
    o = r.tick();
    chk("the next tick adopts the plan: FOLLOWING", o.state == LegState::Following);
    chk("... and sends the carrot 5 m ahead", o.send && near2(o.setpoint, {5, 0}));
    chk("... which is a target the status line can show", r.leg.hasTarget() && near2(r.leg.target(), {5, 0}));
    chk("... and the path is the plan", r.leg.path().size() == 2);
    chk_near("planMs comes from the reply", r.leg.planMs(), 20.0, 1e-9);

    for (int i = 1; i <= 50; ++i) {
      r.in.boat.x = 0.5 * i;
      r.tick();
    }
    bool spaced = r.sends.size() > 2;
    for (std::size_t i = 1; i + 1 < r.sends.size(); ++i) {       // the last one is the END (below)
      spaced = spaced && r.sends[i].p.x - r.sends[i - 1].p.x > 1.5 - 1e-9;
    }
    chk("as the boat advances, carrots are resent only when they moved > resend_m", spaced);
    chk("... far fewer sends than ticks", r.sends.size() < 20);
    chk("... and the last is the END of the path (is_end)", near2(r.sends.back().p, {30, 0}));
    const std::size_t n_sends = r.sends.size();
    for (int i = 0; i < 20; ++i) {r.tick();}
    chk("... after which nothing more is sent while the boat sits there", r.sends.size() == n_sends);

    // The other resend rule: an intermediate carrot the boat is about to reach.
    Rig q(Mode::On);
    q.in.goal = {30, 0};
    LegConfig huge;
    huge.resend_m = 100.0;                  // the distance rule can never fire
    q.start(huge);
    q.tick();
    chk("(resend_m = 100) the first carrot is at 5 m", q.sends.size() == 1 && near2(q.sends[0].p, {5, 0}));
    int resent_at = -1;
    for (int i = 1; i <= 20 && resent_at < 0; ++i) {
      q.in.boat.x = 0.5 * i;
      q.tick();
      if (q.sends.size() > 1) {resent_at = i;}
    }
    chk("... resent when the boat comes within wp_radius + 0.5 of it (boat past 2.5 m)",
      resent_at > 0 && q.in.boat.x > 2.5 && q.in.boat.x <= 3.0 + 1e-9);
    chk("... with the carrot 5 m past the boat again", near2(q.sends[1].p, {q.in.boat.x + 5.0, 0}, 1e-6));

    // A goal ON a buoy: pushed out for the planner, judged on the TRUE goal.
    Rig g(Mode::On);
    g.in.goal = {30, 0};
    g.in.hazards = {buoy(1, 30, 0)};
    g.start(c);
    chk("the planner is asked for a goal pushed out of the buoy, not the buoy's centre",
      g.port.reqs.size() == 1 && g.port.reqs[0].second.x < 29.0 && g.port.reqs[0].second.x > 28.0);
    bool done = false;
    for (int i = 0; i < 400 && !done; ++i) {
      g.tick();
      g.fly(3.0);
      done = g.out.result == Result::Success;
    }
    chk("the leg SUCCEEDS on the true-goal tolerance", done);
    chk("... with the boat still short of the pushed-out end of the path",
      nav::norm(g.in.boat - Vec2{30, 0}) <= 2.0 && g.leg.path().back().x < 29.0);
    chk("... reported ARRIVED", g.out.state == LegState::Arrived);
    chk("... and no hold was sent on the way", g.holds() == 0);
  }

  // ------------------------------------ 3. No plan ever: hold, clear, FAILURE
  std::printf("3. No plan ever arrives\n");
  {
    Rig r(Mode::On);
    r.port.auto_plan = false;
    r.in.goal = {30, 0};
    const LegConfig c;
    r.start(c);
    r.runTo(0.9);
    chk("PLANNING while the first plan may still come (0.9 s)",
      r.out.state == LegState::Planning && r.sends.empty());
    r.tick();
    chk("BLOCKED at 1.0 s", r.out.state == LegState::Blocked);
    chk("... with exactly one hold, on the boat", r.holds() == 1 && r.sends.size() == 1 &&
      same(r.sends[0].p, {0, 0}));
    chk("... and the reason says why", r.out.why == "planner timeout (costmap not current?)");
    r.runTo(5.9);
    chk("blocked_s counts from entering BLOCKED; no costmap clear before 5 s of it", r.port.nclear == 0);
    r.runTo(6.0);
    chk("the costmap is cleared once at blocked_s = 5 s (t = 6.0)", r.port.nclear == 1);
    r.runTo(15.9);
    chk("... only once per episode, and no FAILURE yet", r.port.nclear == 1 && r.failures == 0);
    r.runTo(16.0);
    chk("FAILURE at blocked_s = 15 s (t = 16.0)", r.out.result == Result::Failure && r.out.state == LegState::Failed);
    chk("... the hold stands: still exactly one send", r.sends.size() == 1);
    chk("... and the planner was kept asked all along", r.port.reqs.size() >= 15);
    chk("a finished leg stays finished", r.tick().result == Result::Failure);

    // first_plan_wait on its own: the planner is slow but not failed.
    NavParams slow;
    slow.plan_timeout_s = 5.0;
    Rig w(Mode::On, slow);
    w.port.auto_plan = false;
    w.in.goal = {30, 0};
    w.start(c);
    w.runTo(1.0);
    chk("(timeout 5 s) still BLOCKED at first_plan_wait_s, by the wait, not a failure",
      w.out.state == LegState::Blocked && w.out.why == "no plan within first_plan_wait_s" && w.holds() == 1);
  }

  // ----------------------- 4. Blocked, a valid plan at 8 s, and the flip-flop
  std::printf("4. Blocked, then a valid plan; blocked_s held until 3 s of FOLLOWING\n");
  {
    Rig r(Mode::On);
    r.port.auto_plan = false;
    r.in.goal = {30, 0};
    const LegConfig c;
    r.start(c);
    r.runTo(7.9);
    chk("blocked since t = 1.0: blocked_s is 6.9 at t = 7.9", std::fabs(r.out.blocked_s - 6.9) < 1e-6);
    r.port.answer(Reply::Ok, {{0, 0}, {30, 0}});          // the pending request is answered
    r.port.auto_plan = true;                              // and the planner is back for what follows
    r.tick();
    chk("a valid plan at 8.0 s: FOLLOWING", r.out.state == LegState::Following);
    chk("... the first send after the hold is the carrot", r.sends.back().p.x > 4.0 && !r.sends.back().hold);
    chk_near("blocked_s is HELD (7.0), not reset by the plan", r.out.blocked_s, 7.0, 1e-6);
    r.runTo(10.9);
    chk("... still held after 2.9 s of FOLLOWING", std::fabs(r.out.blocked_s - 7.0) < 1e-6);
    r.runTo(11.0);
    chk_near("... reset to 0 after 3 s of continuous FOLLOWING", r.out.blocked_s, 0.0, 1e-9);
    chk("the costmap was cleared once, in the first episode", r.port.nclear == 1);

    // A flip-flop: a hazard on the path 2 s on, 2 s off. Each BLOCKED spell is
    // followed by FOLLOWING for < 3 s, so blocked_s is never reset and it fails.
    Rig f(Mode::On);
    f.in.goal = {30, 0};
    f.start(c);
    bool never_decreased = true;
    double prev = 0.0;
    f.in.now_s = Rig::kT0;
    for (int i = 0; i < 600 && f.failures == 0; ++i) {
      const double t = (f.ms + 100) / 1000.0;
      const bool present = t >= 2.0 && std::fmod(std::floor((t - 2.0) / 2.0), 2.0) == 0.0;
      f.in.hazards = present ? std::vector<Hazard>{buoy(1, 15, 0)} : std::vector<Hazard>{};
      f.tick();
      never_decreased = never_decreased && f.out.blocked_s >= prev - 1e-12;
      prev = f.out.blocked_s;
    }
    chk("a flip-flop every 2 s still FAILS", f.failures == 1);
    chk("... blocked_s never went back down on the way (no reset below 3 s of FOLLOWING)", never_decreased);
    chk("... and it failed by blocked_s >= 15, not by luck", f.out.blocked_s >= 15.0 - 1e-6);
  }

  // ----------------------------------- 5. IsPathValid: once is nothing, twice is
  std::printf("5. IsPathValid: invalid once does nothing; twice gives BLOCKED\n");
  {
    NavParams quiet;
    quiet.replan_period_s = 100.0;           // so only a confirmed-invalid path asks for a plan
    Rig r(Mode::On, quiet);
    r.port.auto_check = false;               // the test answers the checks
    r.in.goal = {30, 0};
    const LegConfig c;
    r.start(c);
    r.tick();                                // adopt; the first check goes out
    chk("FOLLOWING, and a validity check is in flight", r.out.state == LegState::Following && r.port.ncheck == 1);
    chk("... of the path from the boat's projection on", r.port.checks[0].size() >= 1 && near2(r.port.checks[0][0], {0, 0}));
    const std::size_t holds0 = static_cast<std::size_t>(r.holds());
    r.port.check.status = Reply::Ok;
    r.port.check.valid = false;              // invalid, once
    r.tick();
    chk("invalid ONCE does nothing: still FOLLOWING, no hold, no replan",
      r.out.state == LegState::Following && static_cast<std::size_t>(r.holds()) == holds0 && r.port.reqs.size() == 1);
    chk("... and the next check goes out", r.port.ncheck == 2);
    r.port.check.status = Reply::Ok;
    r.port.check.valid = false;              // invalid, twice
    const std::size_t reqs0 = r.port.reqs.size();
    r.tick();
    chk("invalid TWICE: BLOCKED", r.out.state == LegState::Blocked);
    chk("... with one hold", r.holds() == 1);
    chk("... and an immediate plan request in the same tick", r.port.reqs.size() == reqs0 + 1);
    chk("... the path is dropped", r.leg.path().empty());

    // Invalid, valid, invalid: the streak is consecutive.
    Rig s(Mode::On, quiet);
    s.port.auto_check = false;
    s.in.goal = {30, 0};
    s.start(c);
    s.tick();
    s.port.check.status = Reply::Ok; s.port.check.valid = false; s.tick();
    s.port.check.status = Reply::Ok; s.port.check.valid = true; s.tick();
    s.port.check.status = Reply::Ok; s.port.check.valid = false; s.tick();
    chk("invalid, valid, invalid is not two in a row", s.out.state == LegState::Following);
    // A check that never answers must not stop checking for good.
    Rig lost(Mode::On, quiet);
    lost.port.auto_check = false;
    lost.in.goal = {30, 0};
    lost.start(c);
    lost.runTo(3.0);
    chk("a lost check reply times out and the next check goes out", lost.port.ncheck >= 2 &&
      lost.out.state == LegState::Following);
  }

  // ------------------------------------- 6. A new known hazard; a plan across one
  std::printf("6. A new known hazard on the path is BLOCKED in the same tick\n");
  {
    Rig r(Mode::On);
    r.in.goal = {30, 0};
    const LegConfig c;
    r.start(c);
    r.tick();
    chk("FOLLOWING on a clear field", r.out.state == LegState::Following);
    r.in.hazards = {buoy(1, 15, 0)};         // mid-way along a TWO-vertex path
    r.tick();
    chk("a hazard appears on the path: BLOCKED in that same tick", r.out.state == LegState::Blocked);
    chk("... with the hold sent in that tick", r.out.send && same(r.out.setpoint, {0, 0}) && r.holds() == 1);
    chk("... and the path is dropped", r.leg.path().empty() && !r.leg.hasTarget());
    chk("... the reason names it", r.out.why == "a known hazard is on the path");

    // A plan that crosses a known hazard is rejected, however it reaches us.
    Rig x(Mode::On);
    x.in.goal = {30, 0};
    x.in.hazards = {buoy(1, 15, 0)};
    x.start(c);
    x.tick();
    chk("a straight plan through a buoy is REJECTED: no path adopted", x.leg.path().empty());
    chk("... a plan failure sends PLANNING to BLOCKED at once, with that reason",
      x.out.state == LegState::Blocked && x.out.why == "plan crosses a known hazard");
    x.runTo(5.0);
    chk("... and never follows it", x.leg.path().empty() && x.holds() == 1);

    Rig d(Mode::On);
    d.port.auto_plan = false;
    d.in.goal = {30, 0};
    d.in.hazards = {buoy(1, 15, 0)};
    d.start(c);
    std::vector<Vec2> dense;
    for (int i = 0; i <= 300; ++i) {dense.push_back({0.1 * i, 0.0});}
    d.port.answer(Reply::Ok, dense);
    d.tick();
    chk("a DENSE plan with a vertex on the buoy is rejected too", d.leg.path().empty());

    // The costmap lags the field: a path that clears the hazard by less than the
    // hard band minus the tolerance is rejected; one that clears it is taken.
    Rig n(Mode::On);
    n.port.auto_plan = false;
    n.in.goal = {30, 0};
    n.in.hazards = {buoy(1, 15, 0.9)};       // 0.6 m off the line (surface to line)
    n.start(c);
    n.port.answer(Reply::Ok, {{0, 0}, {30, 0}});
    n.tick();
    chk("a plan 0.6 m off a buoy surface is rejected (needs hard - tol = 0.7)", n.leg.path().empty());
    Rig m(Mode::On);
    m.port.auto_plan = false;
    m.in.goal = {30, 0};
    m.in.hazards = {buoy(1, 15, 1.2)};       // 0.9 m off the line
    m.start(c);
    m.port.answer(Reply::Ok, {{0, 0}, {30, 0}});
    m.tick();
    chk("... and one 0.9 m off is adopted", m.out.state == LegState::Following);
  }

  // ----------------------------------------------------- 7. NaN heading: DEGRADED
  std::printf("7. NaN heading: DEGRADED\n");
  {
    Rig r(Mode::On);
    r.in.goal = {30, 0};
    r.in.heading_deg = std::nan("");
    const LegConfig c;
    r.start(c);
    chk("DEGRADED at once", r.out.state == LegState::Degraded && r.out.why == "heading unknown (NaN)");
    chk("... with one hold, on the boat", r.holds() == 1 && same(r.sends[0].p, {0, 0}));
    r.runTo(60.0);
    chk("60 s later: still one hold", r.holds() == 1 && r.sends.size() == 1);
    chk("... zero plan requests", r.port.reqs.empty() && r.port.ready_calls == 0);
    chk("... blocked_s stayed 0", r.max_blocked == 0.0 && r.out.blocked_s == 0.0);
    chk("... and no FAILURE", r.failures == 0 && r.out.state == LegState::Degraded);
    r.in.heading_deg = 90.0;
    r.tick();
    chk("the heading comes back: PLANNING", r.out.state == LegState::Planning);
    chk("... and a plan is requested at once", r.port.reqs.size() == 1);
    r.tick();
    chk("... and it is followed", r.out.state == LegState::Following);

    Rig d(Mode::On);
    d.in.goal = {30, 0};
    d.in.datum_ok = false;
    d.start(c);
    chk("a changed datum is DEGRADED with its own reason",
      d.out.state == LegState::Degraded && d.out.why == "datum changed: restart bt_runner" && d.holds() == 1);
    d.runTo(20.0);
    chk("... and never fails", d.failures == 0 && d.port.reqs.empty());
  }

  // ------------------------------------------- 8. A stale pose holds where it was
  std::printf("8. A stale pose holds at the last fresh pose, with no Failure\n");
  {
    Rig r(Mode::On);
    r.in.goal = {30, 0};
    const LegConfig c;
    r.start(c);
    for (int i = 0; i < 5; ++i) {r.tick();}
    r.in.boat = {3, 0};
    r.tick();
    chk("FOLLOWING before the pose goes stale", r.out.state == LegState::Following);
    const std::size_t reqs0 = r.port.reqs.size();
    const std::size_t sends0 = r.sends.size();
    r.in.pose_fresh = false;
    r.in.boat = {999, 999};                  // garbage: must never be used
    r.tick();
    chk("stale: DEGRADED", r.out.state == LegState::Degraded && r.out.why == "stale pose");
    chk("... one hold, at the LAST FRESH position (3, 0), not the garbage",
      r.sends.size() == sends0 + 1 && same(r.sends.back().p, {3, 0}));
    r.runTo(70.0);
    chk("60 s later: no further send", r.sends.size() == sends0 + 1);
    chk("... no request, no FAILURE", r.port.reqs.size() == reqs0 && r.failures == 0);
    r.in.pose_fresh = true;
    r.in.boat = {3, 0};
    r.tick();
    chk("a fresh pose again: PLANNING, replanning from where the boat is",
      r.out.state == LegState::Planning && r.port.reqs.size() == reqs0 + 1 &&
      same(r.port.reqs.back().first, {3, 0}));
  }

  // -------------------------------------------- 9. planner_server not available
  std::printf("9. ready() == false\n");
  {
    Rig r(Mode::On);
    r.port.is_ready = false;
    r.in.goal = {30, 0};
    const LegConfig c;
    r.start(c);
    chk("BLOCKED at once, with the reason", r.out.state == LegState::Blocked &&
      r.out.why == "planner_server not available");
    chk("... one hold", r.holds() == 1);
    chk("... and no request was sent", r.port.reqs.empty());
    r.runTo(14.9);
    chk("not failed at 14.9 s", r.failures == 0);
    r.runTo(15.0);
    chk("FAILURE at 15 s, reason intact", r.out.result == Result::Failure &&
      r.out.why == "blocked: planner_server not available");
    chk("... still exactly one hold, and the costmap clear still went out once", r.holds() == 1 && r.port.nclear == 1);

    // The planner goes away DURING a leg: blind, so hold even with a valid path.
    Rig b(Mode::On);
    b.in.goal = {30, 0};
    b.start(c);
    b.tick();
    chk("FOLLOWING first", b.out.state == LegState::Following);
    b.port.is_ready = false;
    b.runTo(1.0);
    chk("the planner goes away: BLOCKED, holding, path dropped",
      b.out.state == LegState::Blocked && b.holds() == 1 && b.leg.path().empty());
    b.port.is_ready = true;
    b.tick();
    b.tick();
    chk("... and it comes back: FOLLOWING again", b.out.state == LegState::Following);
  }

  // ------------------------------------------------------ 10. timeout; late reply
  std::printf("10. A timeout at 1.0 s; the late reply is ignored by seq\n");
  {
    Rig r(Mode::On);
    r.port.auto_plan = false;
    r.in.goal = {30, 0};
    const LegConfig c;
    r.start(c);
    const std::uint64_t first = r.port.plan.seq;
    r.runTo(0.9);
    chk("no timeout at 0.9 s", r.out.state == LegState::Planning && r.port.reqs.size() == 1);
    r.tick();
    chk("timeout at 1.0 s: BLOCKED with the planner-timeout reason",
      r.out.state == LegState::Blocked && r.out.why == "planner timeout (costmap not current?)");
    chk("... and the leg asks again (a new seq)", r.port.reqs.size() == 2 && r.port.plan.seq != first);
    PlanReply late;
    late.seq = first;
    late.status = Reply::Ok;
    late.path = {{0, 0}, {30, 0}};
    const std::uint64_t second = r.port.plan.seq;
    r.port.plan = late;                      // the OLD request's answer, arriving late
    r.tick();
    chk("the late reply for the OLD seq is ignored: no path, still BLOCKED",
      r.leg.path().empty() && r.out.state == LegState::Blocked);
    PlanReply fresh = late;
    fresh.seq = second;
    r.port.plan = fresh;                     // the answer to the CURRENT request
    r.tick();
    chk("the reply for the current seq is taken", r.out.state == LegState::Following);

    // A reply that is still Pending does not time out early.
    Rig p(Mode::On);
    p.port.auto_plan = false;
    p.in.goal = {30, 0};
    p.start(c);
    p.runTo(0.8);
    chk("Pending is not a reply", p.leg.path().empty() && p.out.state == LegState::Planning);
    // Failed and Unavailable.
    Rig fl(Mode::On);
    fl.port.auto_plan = false;
    fl.in.goal = {30, 0};
    fl.start(c);
    fl.port.answer(Reply::Failed, {}, "planner aborted (no path)");
    fl.tick();
    chk("Failed with no path: BLOCKED at once, with the planner's own reason",
      fl.out.state == LegState::Blocked && fl.out.why == "planner aborted (no path)");
  }

  // ------------------------------------------------------------------ 11. Shadow
  std::printf("11. Shadow: legacy sends, real state, never a Failure\n");
  {
    Rig r(Mode::Shadow);
    r.port.auto_plan = false;                // the planner never answers
    r.in.goal = {30, 0};
    const LegConfig c;                       // avoid = true
    r.start(c);
    chk("the start sends the legacy goal", r.out.send && same(r.out.setpoint, {30, 0}));
    r.runTo(2.0);
    chk("the state shows BLOCKED", r.out.state == LegState::Blocked);
    chk("... and says it is only a shadow", r.out.why.rfind("shadow: would HOLD (", 0) == 0);
    r.runTo(40.0);
    chk("40 s blocked: never a Failure", r.failures == 0 && r.out.result == Result::Running);
    chk("the only sends are the goal: no hold, no carrot", r.sends.size() == 1 && r.holds() == 0 &&
      same(r.sends[0].p, {30, 0}));
    chk("... but every port call was real (requests, and the costmap clear)",
      r.port.reqs.size() > 10 && r.port.nclear == 1);
    r.in.goal = {33, 0};
    r.tick();
    chk("a moved goal is resent the legacy way", r.sends.size() == 2 && same(r.sends[1].p, {33, 0}));
    r.in.boat = {32, 0};
    r.tick();
    chk("arrival is judged the legacy way: Success", r.out.result == Result::Success);

    Rig g(Mode::Shadow);
    g.in.goal = {30, 0};
    g.start(c);
    g.runTo(3.0);
    chk("with a planner that answers, the state is FOLLOWING and the sends are STILL only the goal",
      g.out.state == LegState::Following && g.sends.size() == 1 && g.holds() == 0);
    g.in.heading_deg = std::nan("");
    g.runTo(6.0);
    chk("a NaN heading shows DEGRADED, but nothing is held", g.out.state == LegState::Degraded &&
      g.holds() == 0 && g.sends.size() == 1 && g.failures == 0);

    // Straight, in Shadow: the guard is logged, never acted on.
    Rig s(Mode::Shadow);
    s.in.goal = {30, 0};
    s.in.hazards = {buoy(1, 15, 0)};
    LegConfig straight;
    straight.avoid = false;
    s.start(straight);
    s.runTo(20.0);
    chk("avoid=false in Shadow: STRAIGHT, would-hold logged, nothing sent but the goal",
      s.out.state == LegState::Straight && s.out.why.rfind("shadow: would HOLD", 0) == 0 &&
      s.sends.size() == 1 && s.holds() == 0 && s.failures == 0);
    chk("... and no planner call", s.port.reqs.empty());
  }

  // -------------------- 12. avoid=false, exempt gate: straight, with a guard
  std::printf("12. avoid=false with exempt=gate\n");
  {
    LegConfig c;
    c.avoid = false;
    c.exempt_buoys = {1, 2};
    const std::vector<Hazard> pair{buoy(1, 10, 0.9), buoy(2, 10, -0.9)};   // 0.6 m off the line

    Rig r(Mode::On);
    r.in.goal = {20, 0};
    r.in.hazards = pair;
    r.start(c);
    chk("the own pair 0.9 m off the segment is ignored: STRAIGHT, the goal sent",
      r.out.state == LegState::Straight && r.out.send && same(r.out.setpoint, {20, 0}));
    r.runTo(5.0);
    chk("... and stays so", r.out.state == LegState::Straight && r.holds() == 0);

    // A third buoy on the line, from t = 1 s.
    std::vector<Hazard> with_third = pair;
    with_third.push_back(buoy(3, 15, 0));
    r.in.hazards = with_third;
    r.tick();
    chk("a THIRD buoy on the line: BLOCKED, one hold", r.out.state == LegState::Blocked && r.holds() == 1 &&
      same(r.out.setpoint, {0, 0}));
    r.runTo(10.0);
    chk("... still one hold after 4 s", r.holds() == 1 && r.failures == 0);
    r.in.hazards = pair;
    r.tick();
    chk("removing it does not resume at once: the line must stay clear (section 18)",
      r.out.state == LegState::Blocked && !r.out.send && r.holds() == 1);
    r.runTo(13.0);                       // clear since 10.1 s: 2.9 s of it
    chk("... still held after 2.9 s of clear line", r.out.state == LegState::Blocked && !r.out.send);
    r.tick();
    chk("... and after unblock_reset_s (3 s): STRAIGHT, with the goal RESENT",
      r.out.state == LegState::Straight && r.out.send && same(r.out.setpoint, {20, 0}));
    chk("... blocked_s is back to 0", r.out.blocked_s == 0.0);
    chk("A straight leg makes no planner calls and no costmap clears",
      r.port.reqs.empty() && r.port.ready_calls == 0 && r.port.nclear == 0 && r.port.ncheck == 0);

    Rig f(Mode::On);
    f.in.goal = {20, 0};
    f.in.hazards = with_third;
    f.start(c);
    chk("a blocked straight leg holds at the start", f.out.state == LegState::Blocked && f.holds() == 1 &&
      f.sends.size() == 1);
    f.runTo(14.9);
    chk("not failed at 14.9 s", f.failures == 0);
    f.runTo(15.0);
    chk("FAILURE after blocked_timeout_s (15 s)", f.out.result == Result::Failure);
    chk("... one hold only, and still no planner call", f.holds() == 1 && f.port.reqs.empty() && f.port.nclear == 0);

    LegConfig quick = c;
    quick.blocked_timeout_s = 3.0;
    Rig q(Mode::On);
    q.in.goal = {20, 0};
    q.in.hazards = with_third;
    q.start(quick);
    q.runTo(3.0);
    chk("blocked_timeout_s is a port: 3 s fails at 3 s", q.out.result == Result::Failure);

    // The dock: exempt=dock.
    Hazard fing;
    fing.kind = HazardKind::Polygon;
    fing.source = HazardSource::Dock;
    fing.id = 7;
    fing.poly = {{9, -0.25}, {11, -0.25}, {11, 0.25}, {9, 0.25}};
    LegConfig dockc;
    dockc.avoid = false;
    Rig dk(Mode::On);
    dk.in.goal = {20, 0};
    dk.in.hazards = {fing};
    dk.start(dockc);
    chk("a dock finger on the line blocks a straight leg", dk.out.state == LegState::Blocked);
    dockc.exempt_dock = true;
    Rig dx(Mode::On);
    dx.in.goal = {20, 0};
    dx.in.hazards = {fing};
    dx.start(dockc);
    dx.runTo(3.0);
    chk("... unless exempt=dock", dx.out.state == LegState::Straight && dx.holds() == 0);

    // A stale pose is Running, as legacy: no guard, no failure, no growth.
    Rig st(Mode::On);
    st.in.goal = {20, 0};
    st.in.hazards = with_third;
    st.start(c);
    st.in.pose_fresh = false;
    st.runTo(40.0);
    chk("a stale pose on a blocked straight leg: Running, no growth, no Failure",
      st.failures == 0 && st.out.blocked_s == 0.0 && st.out.result == Result::Running);
    // And arrival.
    Rig ar(Mode::On);
    ar.in.goal = {20, 0};
    ar.start(c);
    ar.in.boat = {19, 0};
    chk("a straight leg arrives on tolerance", ar.tick().result == Result::Success);
  }

  // -------------------------------------------------------------- 13. A 100 m goal
  std::printf("13. A 100 m goal: the requested goal sits 35 m out and advances\n");
  {
    Rig r(Mode::On);
    r.in.goal = {100, 0};
    const LegConfig c;
    r.start(c);
    chk("the first request names a goal 35 m out, not 100", near2(r.port.reqs[0].second, {35, 0}));
    double reached_x = 0.0;
    bool stalled = false;
    for (int i = 0; i < 600 && r.out.result == Result::Running; ++i) {
      r.tick();
      r.fly(3.0);
      reached_x = std::max(reached_x, r.in.boat.x);
      if (r.in.boat.x > 20.0 && r.in.boat.x < 30.0 && r.port.reqs.back().second.x < 50.0) {
        stalled = true;
      }
    }
    chk("as the boat advances the requested goal advances with it (boat + 35 m)",
      near2(r.port.reqs.back().second, {r.in.boat.x + 35.0, 0}, 40.0) &&
      r.port.reqs.back().second.x > 90.0 && !stalled);
    chk("the boat does NOT stop at the end of the first window (x > 60)", reached_x > 60.0);
    chk("... and goes the whole way: Success on the true goal", r.out.result == Result::Success);
    chk("... every request stayed inside the window (<= 35 m from the boat)", [&] {
        for (const auto & q : r.port.reqs) {
          if (q.second.x - q.first.x > 35.0 + 1e-6) {return false;}
        }
        return true;
      }());
    chk("... with no hold on the way", r.holds() == 0);
  }

  // ------------------------------------------------------ 14. Start inside the band
  std::printf("14. A start inside the hard zone: the escape point first\n");
  {
    Rig r(Mode::On);
    r.in.goal = {40, 0};
    r.in.hazards = {buoy(1, 10, 0)};
    r.in.boat = {10.8, 0};                    // 0.5 m off the buoy's surface: inside hard = 0.8
    const LegConfig c;
    r.start(c);
    const NavParams P;
    const Escape want = escapeStart({10.8, 0}, {40, 0}, r.in.hazards, P);
    chk("an escape is needed and found", want.needed && want.ok);
    chk("the plan START is the escape point, not the boat", r.port.reqs.size() == 1 &&
      near2(r.port.reqs[0].first, want.p) && nav::norm(r.port.reqs[0].first - r.in.boat) >= 2.5 - 1e-9);
    chk("... and the goal is the goal", same(r.port.reqs[0].second, {40, 0}));
    r.tick();
    chk("the FIRST thing sent is the escape point", r.out.send && near2(r.out.setpoint, want.p) &&
      r.out.state == LegState::Following);
    r.tick();
    chk("... and it is not resent while the boat is still in the band", r.sends.size() == 1);
    chk("the path the boat follows starts at the escape point", near2(r.leg.path().front(), want.p));
    r.in.boat = want.p;                       // out in the clear
    r.tick();
    chk("once clear of the band the leg follows the path: a carrot, past the escape point",
      r.sends.size() == 2 && r.sends[1].p.x > want.p.x + 3.0 && !r.out.why.size());

    Rig b(Mode::On);
    b.in.goal = {20, 0};
    b.in.hazards = {buoy(1, -1, 0), buoy(2, 1, 0), buoy(3, 0, 1), buoy(4, 0, -1)};
    b.start(c);
    chk("boxed in: the request still goes out, from the boat", b.port.reqs.size() == 1 &&
      same(b.port.reqs[0].first, {0, 0}));
    chk("... and the leg is BLOCKED with the reason `boxed in`", b.out.state == LegState::Blocked &&
      b.out.why == "boxed in" && b.holds() == 1);
  }

  // ------------------------------------------------- 15. The edges of the machine
  std::printf("15. Halt, no goal, the stub port, the state names\n");
  {
    Rig r(Mode::On);
    r.port.auto_plan = false;
    r.in.goal = {30, 0};
    const LegConfig c;
    r.start(c);
    r.leg.halt();
    chk("halt: IDLE, no path, no target", r.leg.state() == LegState::Idle && r.leg.path().empty() &&
      !r.leg.hasTarget());
    const std::size_t reqs0 = r.port.reqs.size();
    r.port.answer(Reply::Ok, {{0, 0}, {30, 0}});          // the in-flight reply lands after halt
    chk("a step after halt does nothing", r.leg.step(r.in).state == LegState::Idle && r.port.reqs.size() == reqs0);
    chk("... and the in-flight reply is not adopted", r.leg.path().empty());
    r.leg.start(c, r.in);
    chk("a leg can be started again after halt", r.leg.state() == LegState::Planning &&
      r.port.reqs.size() == reqs0 + 1);

    Rig n(Mode::On);
    n.in.goal_ok = false;
    const LegOutput o = n.start(LegConfig{});
    chk("start with no goal: Failure, and nothing sent", o.result == Result::Failure && !o.send);

    // A goal that cannot be resolved later keeps the last one.
    Rig k(Mode::Off);
    k.in.goal = {50, 0};
    k.start(LegConfig{});
    k.in.goal_ok = false;
    k.in.goal = {500, 0};
    k.tick();
    chk("a goal that goes missing mid-leg is ignored, not obeyed", !k.out.send && k.out.result == Result::Running);

    chk("state names", std::string(legStateName(LegState::Idle)) == "IDLE" &&
      std::string(legStateName(LegState::Planning)) == "PLANNING" &&
      std::string(legStateName(LegState::Following)) == "FOLLOWING" &&
      std::string(legStateName(LegState::Blocked)) == "BLOCKED" &&
      std::string(legStateName(LegState::Degraded)) == "DEGRADED" &&
      std::string(legStateName(LegState::Arrived)) == "ARRIVED" &&
      std::string(legStateName(LegState::Failed)) == "FAILED");

    // The offros stub: instant straight path, checks always valid.
    StraightPlannerPort sp;
    chk("StraightPlannerPort is ready", sp.ready());
    chk("... nothing planned yet: seq 0, status None", sp.lastPlan().seq == 0 && sp.lastPlan().status == Reply::None);
    const std::uint64_t s1 = sp.requestPlan({1, 2}, {3, 4});
    const PlanReply pr = sp.lastPlan();
    chk("... an instant [start, goal] path", s1 == 1 && pr.seq == 1 && pr.status == Reply::Ok &&
      pr.path.size() == 2 && same(pr.path[0], {1, 2}) && same(pr.path[1], {3, 4}));
    chk("... seqs increase", sp.requestPlan({0, 0}, {1, 1}) == 2);
    const std::uint64_t c1 = sp.requestCheck({{0, 0}, {1, 1}});
    chk("... checks are always valid", c1 == 1 && sp.lastCheck().status == Reply::Ok && sp.lastCheck().valid);
    sp.clearCostmap();

    PlannedLeg real(NavParams{}, Mode::On, &sp);
    LegInputs in;
    in.now_s = 10.0;
    in.pose_fresh = true;
    in.heading_deg = 0.0;
    in.goal_ok = true;
    in.goal = {30, 0};
    in.hazards = {buoy(1, 15, 0)};
    real.start(LegConfig{}, in);
    in.now_s = 10.1;
    LegOutput so = real.step(in);
    chk("PlannedLeg + the stub: a line through a known hazard is rejected by the leg itself",
      so.state == LegState::Blocked && so.why == "plan crosses a known hazard" && real.path().empty());
    PlannedLeg clear(NavParams{}, Mode::On, &sp);
    in.hazards.clear();
    in.now_s = 20.0;
    clear.start(LegConfig{}, in);
    in.now_s = 20.1;
    so = clear.step(in);
    chk("... and a free line is followed", so.state == LegState::Following && so.send &&
      near2(so.setpoint, {5, 0}));

    // A null port is Off-mode safe, and On-mode "not available", never a crash.
    PlannedLeg nul(NavParams{}, Mode::On, nullptr);
    in.now_s = 30.0;
    const LegOutput no = nul.start(LegConfig{}, in);
    chk("a null port in On mode: BLOCKED, planner_server not available",
      no.state == LegState::Blocked && no.why == "planner_server not available");
    PlannedLeg off(NavParams{}, Mode::Off, nullptr);
    chk("a null port in Off mode is the legacy leg", off.start(LegConfig{}, in).send);
  }

  // ------------------------------------------ 16. Hysteresis, cadence, goal moves
  std::printf("16. Replans: hysteresis, cadence, a moving goal, a failure while FOLLOWING\n");
  {
    const LegConfig c;
    // A detour in hand (34.0 m), a straight line on offer (30 m): 12 % < 20 %, so KEPT.
    Rig h(Mode::On);
    h.port.auto_plan = false;
    h.in.goal = {30, 0};
    h.start(c);
    h.port.answer(Reply::Ok, {{0, 0}, {15, 8}, {30, 0}});
    h.tick();
    chk("the first plan is adopted (a detour)", h.out.state == LegState::Following && h.leg.path().size() == 3);
    h.runTo(1.0);
    h.port.answer(Reply::Ok, {{0, 0}, {30, 0}});          // a plan that is only ~12 % shorter
    h.tick();
    chk("a replan 12 % shorter is NOT adopted (hysteresis): the path is still the detour",
      h.leg.path().size() == 3);

    // 42.4 m in hand against 30 m on offer: 29 % and 12 m shorter: ADOPTED.
    Rig a(Mode::On);
    a.port.auto_plan = false;
    a.in.goal = {30, 0};
    a.start(c);
    a.port.answer(Reply::Ok, {{0, 0}, {15, 15}, {30, 0}});
    a.tick();
    a.runTo(1.0);
    a.port.answer(Reply::Ok, {{0, 0}, {30, 0}});
    a.tick();
    chk("a replan 29 % and 12 m shorter IS adopted", a.leg.path().size() == 2 && a.out.state == LegState::Following);

    // Cadence: about 2 Hz while FOLLOWING, never closer than min_request_gap.
    Rig r(Mode::On);
    r.in.goal = {30, 0};
    r.start(c);
    r.runTo(10.0);
    chk("steady state: about 2 requests a second (replan_period 0.5 s)",
      r.port.reqs.size() >= 19 && r.port.reqs.size() <= 22);
    chk("... and a validity check about every 0.1 s", r.port.ncheck >= 80 && r.port.ncheck <= 101);

    // The goal moving more than goal_replan_m asks for a plan NOW, not at the next period.
    NavParams slow;
    slow.replan_period_s = 100.0;
    Rig m(Mode::On, slow);
    m.in.goal = {30, 0};
    m.start(c);
    m.runTo(2.0);
    chk("(replan_period 100 s) no periodic replans", m.port.reqs.size() == 1);
    m.in.goal = {30.8, 0};
    m.tick();
    chk("a goal move under goal_replan_m (1.0 m) asks for nothing", m.port.reqs.size() == 1);
    m.in.goal = {31.5, 0};
    m.tick();
    chk("a goal move over it asks for a plan in that tick", m.port.reqs.size() == 2 &&
      same(m.port.reqs.back().second, {31.5, 0}));

    // A planner that says "no path" while FOLLOWING does not stop a path that is still valid ...
    Rig f(Mode::On);
    f.in.goal = {30, 0};
    f.start(c);
    f.tick();
    f.port.auto_plan = false;
    f.runTo(0.6);                            // the next replan goes out, unanswered
    f.port.answer(Reply::Failed, {}, "planner aborted (no path)");
    f.tick();
    chk("Failed with a valid path in hand: keep FOLLOWING, no hold", f.out.state == LegState::Following &&
      f.holds() == 0 && !f.leg.path().empty());
    bool logged = false;
    for (const std::string & l : f.out.log) {logged = logged || l.find("planner aborted") != std::string::npos;}
    chk("... and say so in the log", logged);

    // ... but a planner that does not ANSWER is blind: hold, even with a path in hand.
    Rig t(Mode::On);
    t.in.goal = {30, 0};
    t.start(c);
    t.tick();
    t.port.auto_plan = false;
    t.runTo(2.0);
    chk("a timeout while FOLLOWING: BLOCKED, one hold, the path dropped", t.out.state == LegState::Blocked &&
      t.holds() == 1 && t.leg.path().empty() && t.out.why == "planner timeout (costmap not current?)");
    chk("... and not before the 1.0 s timeout (carrots were sent until then)", t.sends.size() >= 2);

    // Unavailable (the action server vanished mid-request) is blind too.
    Rig u(Mode::On);
    u.in.goal = {30, 0};
    u.start(c);
    u.tick();
    u.port.auto_plan = false;
    u.runTo(0.6);
    u.port.answer(Reply::Unavailable, {}, "action server gone");
    u.tick();
    chk("Unavailable while FOLLOWING: BLOCKED", u.out.state == LegState::Blocked && u.out.why == "action server gone");

    // A new start() begins a fresh episode.
    Rig n(Mode::On);
    n.port.auto_plan = false;
    n.in.goal = {30, 0};
    n.start(c);
    n.runTo(8.0);
    n.leg.halt();
    n.leg.start(c, n.in);
    chk("start() after halt() begins a fresh episode: blocked_s 0", n.leg.step(n.in).blocked_s == 0.0);
  }

  // ---------------- 17. A goal that moved is re-targeted, never hysteresis'd away
  std::printf("17. A planned leg re-targets a goal that moved while FOLLOWING\n");
  {
    LegConfig c;
    c.tolerance = 1.0;                       // the Task 3 lead-in (task3_disruptive.xml)

    // The boat is 3 m short of the end of a path to (30, 0) when the goal moves 5 m
    // sideways. The old path scored 3 + 5 = 8 m against the fresh plan's 5.8 m: 27 % but
    // only 2.2 m shorter, so hysteresis refused it, and following the old path to its end
    // made the two EQUAL (5 m each). A new plan was never adopted and the boat parked
    // 5 m from a goal it could not arrive at.
    Rig r(Mode::On);
    r.in.goal = {30, 0};
    r.start(c);
    r.driveUntil([&] {return r.in.boat.x >= 27.0;}, 400);
    chk("(setup) following the plan to (30, 0), the boat 3 m short of its end",
      r.out.state == LegState::Following && r.in.boat.x >= 27.0 && r.in.boat.x < 28.0);
    r.in.goal = {30, 5};
    r.driveUntil([&] {return r.out.result != Result::Running;}, 400);
    chk("the goal moved 5 m: the leg ARRIVES at the new goal", r.out.result == Result::Success);
    chk("... on a path that was planned to the NEW goal",
      !r.leg.path().empty() && near2(r.leg.path().back(), {30, 5}));
    chk("... and the boat is within the tolerance of it",
      nav::norm(r.in.boat - Vec2{30, 5}) <= 1.0 + 1e-9);

    // The same thing as it happens in the lead-in: DockWaypoint rewrites {goal} every
    // tick while the bay estimate settles, here 0.5 m/s sideways for 10 s.
    Rig d(Mode::On);
    d.in.goal = {30, 0};
    d.start(c);
    d.driveUntil(
      [&] {return d.out.result != Result::Running;}, 600,
      [&](int i) {d.in.goal.y = std::min(5.0, 0.05 * i);});
    chk("a goal drifting 5 m sideways over 10 s: the leg still ARRIVES",
      d.out.result == Result::Success && nav::norm(d.in.boat - Vec2{30, 5}) <= 1.0 + 1e-9);

    // Jitter below the threshold: the path in hand is KEPT, whatever the planner offers.
    Rig j(Mode::On);
    j.in.goal = {30, 0};
    j.start(c);
    const auto jitter = [&](int i) {j.in.goal = {30, (i % 2 == 0) ? 0.4 : -0.4};};
    j.driveUntil([] {return false;}, 60, jitter);          // the boat is at x = 18 after 6 s
    chk("goal jitter of +-0.4 m (under goal_replan_m 1.0 and the tolerance): still FOLLOWING the "
      "first path, planned from (0, 0)",
      j.out.state == LegState::Following && near2(j.leg.path().front(), {0, 0}) &&
      j.leg.path().size() == 2 && j.in.boat.x > 10.0);
    chk("... replans kept coming (this is not a leg that stopped asking)", j.port.reqs.size() > 5);
    j.driveUntil([&] {return j.out.result != Result::Running;}, 200, jitter);
    chk("... and the leg arrives", j.out.result == Result::Success);

    // The threshold is the SMALLER of goal_replan_m and the tolerance: a goal that moved
    // out of a tight tolerance is not the goal the path was planned for, even though it is
    // under goal_replan_m and would not ask for a plan by itself.
    LegConfig tight;
    tight.tolerance = 0.4;
    Rig k(Mode::On);
    k.in.goal = {30, 0};
    k.start(tight);
    k.driveUntil([&] {return k.in.boat.x >= 10.0;}, 400);
    k.in.goal = {30, 0.6};                   // > tolerance 0.4, < goal_replan_m 1.0
    k.driveUntil([] {return false;}, 15);
    chk("(tolerance 0.4) a 0.6 m goal move is followed by a path to the new goal",
      !k.leg.path().empty() && near2(k.leg.path().back(), {30, 0.6}));
    k.driveUntil([&] {return k.out.result != Result::Running;}, 200);
    chk("... and the leg ARRIVES", k.out.result == Result::Success);
  }

  // ------- 18. A guarded straight leg resumes only after a CLEAR line stays clear
  std::printf("18. A guarded straight leg needs unblock_reset_s of clear line to resume\n");
  {
    LegConfig c;
    c.avoid = false;
    const std::vector<Hazard> on{buoy(1, 10, 0)};
    const std::vector<Hazard> none;

    // A track that flickers: on the line for 1 s, off it for 1 s, and so on.
    auto flicker = [&](Rig & r) {r.in.hazards = (((r.ms + 100) / 1000) % 2 == 0) ? on : none;};
    for (const Mode m : {Mode::On, Mode::Off}) {
      const std::string tag = m == Mode::On ? "(On, avoid=false) " : "(Off) ";
      LegConfig lc = c;
      lc.avoid = m == Mode::On ? false : true;      // Off: the default avoid, which must not matter
      Rig f(m);
      f.in.goal = {20, 0};
      f.in.hazards = on;
      f.start(lc);
      f.runTo(14.9, [&] {flicker(f);});
      chk(tag + "a track flickering on the line: held the whole time, ONE hold, no resume",
        f.out.state == LegState::Blocked && f.holds() == 1 && f.sends.size() == 1);
      chk_near(tag + "... and blocked_s kept counting through the clear half-seconds",
        f.out.blocked_s, 14.9, 0.05);
      f.tick();
      chk(tag + "FAILURE at blocked_timeout_s (15 s) all the same",
        f.out.result == Result::Failure && f.ms == 15000);
    }

    // A track that leaves for good: the boat resumes 3.0 s after the line first CLEARED.
    Rig g(Mode::On);
    g.in.goal = {20, 0};
    g.in.hazards = on;
    g.start(c);
    g.runTo(5.0);
    g.in.hazards = none;
    g.tick();                                // 5.1 s: the line is clear for the first time
    chk("the line clears: still BLOCKED, nothing sent", g.out.state == LegState::Blocked && !g.out.send);
    g.runTo(8.0);                            // 2.9 s of clear line
    chk("2.9 s clear: still held, still one send (the hold)", g.out.state == LegState::Blocked &&
      g.sends.size() == 1 && g.out.blocked_s > 0.0);
    g.tick();                                // 8.1 s: 3.0 s clear
    chk("3.0 s clear: STRAIGHT again, with the goal RESENT",
      g.out.state == LegState::Straight && g.out.send && same(g.out.setpoint, {20, 0}));
    chk("... blocked_s is back to 0", g.out.blocked_s == 0.0);

    // A clear spell that is cut short starts the window over.
    Rig h(Mode::On);
    h.in.goal = {20, 0};
    h.in.hazards = on;
    h.start(c);
    h.runTo(1.0);
    h.in.hazards = none;
    h.runTo(3.0);                            // clear from 1.1 s: 1.9 s of it
    h.in.hazards = on;
    h.tick();                                // 3.1 s: blocked again
    h.in.hazards = none;
    h.runTo(5.0);                            // clear from 3.2 s: 1.8 s, 3.9 s after the FIRST clear
    chk("a clear spell cut short does not count towards the next one", h.out.state == LegState::Blocked &&
      h.sends.size() == 1);
    h.runTo(6.1);
    chk("... the window is 3.0 s from the SECOND clear (3.2 + 3.0)", h.out.state == LegState::Blocked);
    h.tick();
    chk("... and the leg resumes then", h.out.state == LegState::Straight && h.out.send && h.ms == 6200);
  }

  // ------------- 19. Off: every leg is a guarded straight leg, avoid or not
  std::printf("19. Off mode: a leg never drives through a known hazard\n");
  {
    for (const bool avoid : {true, false}) {
      const std::string tag = avoid ? "(Off, avoid=true) " : "(Off, avoid=false) ";
      LegConfig c;
      c.avoid = avoid;
      Rig r(Mode::Off);
      r.in.goal = {50, 0};
      r.in.hazards = {buoy(1, 25, 0)};
      r.start(c);
      chk(tag + "a hazard on the line: BLOCKED, one hold on the boat, the goal NOT sent",
        r.out.state == LegState::Blocked && r.sends.size() == 1 && r.holds() == 1 &&
        same(r.sends[0].p, {0, 0}));
      chk(tag + "... the status says BLOCKED and why, never STRAIGHT",
        std::string(legStateName(r.out.state)) == "BLOCKED" &&
        r.out.why == "a known hazard is on the line to the goal");
      r.runTo(14.9);
      chk(tag + "... still one hold at 14.9 s, no Failure yet", r.holds() == 1 &&
        r.sends.size() == 1 && r.failures == 0);
      r.tick();
      chk(tag + "FAILURE at blocked_timeout_s (15 s)", r.out.result == Result::Failure &&
        r.out.state == LegState::Failed);
      chk(tag + "... with no planner call and no costmap clear (no Nav2 needed)",
        r.port.reqs.empty() && r.port.ready_calls == 0 && r.port.nclear == 0 && r.port.ncheck == 0);
    }

    // A clear line: the legacy send sequence, exactly.
    for (const bool avoid : {true, false}) {
      LegConfig c;
      c.avoid = avoid;
      Rig r(Mode::Off);
      r.in.goal = {50, 0};
      r.in.hazards = {buoy(1, 25, 3.0)};         // 3 m off the line: outside hard
      LegOutput o = r.start(c);
      chk("(Off) a hazard off the line: STRAIGHT, and the start sends the goal",
        o.state == LegState::Straight && o.send && same(o.setpoint, {50, 0}));
      r.in.goal = {52, 0};
      o = r.tick();
      chk("(Off) ... a 2.0 m goal move is resent; nothing else is", o.send && same(o.setpoint, {52, 0}) &&
        r.sends.size() == 2 && r.holds() == 0);
    }

    // Exemptions still apply to the guard (a gate crossing driven in Off).
    LegConfig gate;
    gate.avoid = false;
    gate.exempt_buoys = {1, 2};
    Rig x(Mode::Off);
    x.in.goal = {20, 0};
    x.in.hazards = {buoy(1, 10, 0.9), buoy(2, 10, -0.9)};
    x.start(gate);
    x.runTo(3.0);
    chk("(Off, exempt gate) the leg's own pair is ignored", x.out.state == LegState::Straight && x.holds() == 0);
    x.in.hazards.push_back(buoy(3, 15, 0));
    x.tick();
    chk("(Off, exempt gate) ... a third buoy on the line holds it", x.out.state == LegState::Blocked && x.holds() == 1);

    // A hazard that appears mid-leg holds the boat where it is, and the leg resumes by itself.
    Rig m(Mode::Off);
    m.in.goal = {50, 0};
    m.start(LegConfig{});
    m.in.boat = {8, 0};
    m.in.hazards = {buoy(1, 25, 0)};
    m.tick();
    chk("(Off) a hazard appears mid-leg: one hold, on the boat", m.out.state == LegState::Blocked &&
      m.sends.size() == 2 && m.holds() == 1 && same(m.sends.back().p, {8, 0}));
    m.in.hazards.clear();
    m.runTo(3.1);
    chk("(Off) ... the line clears: not resumed before 3 s of it", m.out.state == LegState::Blocked && m.sends.size() == 2);
    m.tick();
    chk("(Off) ... then resumed with the goal resent", m.out.state == LegState::Straight &&
      same(m.sends.back().p, {50, 0}) && m.sends.size() == 3);

    // A stale pose is Running, as legacy: nothing to judge the line from.
    Rig st(Mode::Off);
    st.in.goal = {50, 0};
    st.in.hazards = {buoy(1, 25, 0)};
    st.start(LegConfig{});
    st.in.pose_fresh = false;
    st.runTo(40.0);
    chk("(Off) a stale pose: Running, no growth, no Failure", st.failures == 0 && st.out.blocked_s == 0.0 &&
      st.out.result == Result::Running);
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  std::printf("%s\n", g_fails == 0 ? "PASS" : "FAIL");
  return g_fails == 0 ? 0 : 1;
}
