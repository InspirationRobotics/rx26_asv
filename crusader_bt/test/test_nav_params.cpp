// test_nav_params — the nav_* planner knobs as a table: declared once, checked, diffed.
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include -o /tmp/t test/test_nav_params.cpp && /tmp/t
//
// bt_runner_node re-reads these into ctx_->nav when a goal is accepted and refuses a bad value in
// its set-callback; both go through the functions pinned here, so the node itself needs no ROS to
// be right about the rules.
#include <cmath>
#include <cstdio>
#include <limits>
#include <set>
#include <string>

#include "crusader_bt/nav_params.hpp"

using namespace crusader_bt::path;   // NOLINT(build/namespaces) -- a test

static int g_fails = 0;
static int g_checks = 0;

static void chk(const std::string & name, bool pass)
{
  ++g_checks;
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s\n", pass ? "ok" : "FAIL", name.c_str());
}

static bool refused(const std::string & name, double v) {return !checkNavValue(name, v).empty();}

int main()
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  std::printf("1. the table covers NavParams\n");
  {
    // Every double and int knob of the struct, in a table: add a field to NavParams and forget the
    // table and this is what fails. (Doubles are 8-aligned and the two ints each take an 8-byte
    // slot between doubles on every 64-bit target this builds for.)
    static_assert(sizeof(void *) != 8 ||
      sizeof(NavParams) == std::size(kNavReals) * sizeof(double) + 2 * sizeof(double),
      "NavParams has a field that is not in kNavReals / kNavWholes (nav_params.hpp)");
    chk("47 keys: 45 real + 2 whole", kNavKeyCount == 47 && std::size(kNavWholes) == 2);

    std::set<std::string> names;
    bool all_nav = true;
    for (const NavReal & k : kNavReals) {names.insert(k.name); all_nav &= std::string(k.name).rfind("nav_", 0) == 0;}
    for (const NavWhole & k : kNavWholes) {names.insert(k.name); all_nav &= std::string(k.name).rfind("nav_", 0) == 0;}
    chk("every name is unique and starts with nav_", names.size() == kNavKeyCount && all_nav);

    // Poke every knob to its own value: if two entries pointed at one member, fewer than all differ.
    NavParams poked;
    double v = 100.0;
    for (const NavReal & k : kNavReals) {poked.*(k.field) = v; v += 1.0;}
    for (const NavWhole & k : kNavWholes) {poked.*(k.field) = static_cast<int>(v); v += 1.0;}
    chk("every table entry reaches its own member", diffNavParams(NavParams{}, poked).size() == kNavKeyCount);
  }

  std::printf("2. the defaults are all acceptable (so a boat on defaults starts)\n");
  chk("checkNavParams(NavParams{}) is empty", checkNavParams(NavParams{}).empty());

  std::printf("3. non-finite values are refused for every knob\n");
  {
    bool all = true;
    for (const NavReal & k : kNavReals) {
      all &= refused(k.name, nan) && refused(k.name, inf) && refused(k.name, -inf);
    }
    for (const NavWhole & k : kNavWholes) {all &= refused(k.name, nan) && refused(k.name, inf);}
    chk("NaN, +inf and -inf refused everywhere", all);
    const std::string why = checkNavValue("nav_hard_m", nan);
    chk("the reason names the key and says finite",
      why.find("nav_hard_m") != std::string::npos && why.find("finite") != std::string::npos);
  }

  std::printf("4. the bounds, per kind\n");
  {
    chk("Positive: 0 refused", refused("nav_gate_step_m", 0.0));
    chk("Positive: negative refused", refused("nav_plan_timeout_s", -1.0));
    chk("Positive: tiny accepted", !refused("nav_gate_step_m", 0.01));
    chk("Positive: the reason says > 0",
      checkNavValue("nav_hard_m", 0.0).find("> 0") != std::string::npos);
    chk("NonNegative: 0 accepted (a margin of none)", !refused("nav_goal_margin_m", 0.0));
    chk("NonNegative: negative refused", refused("nav_goal_margin_m", -0.1));
    chk("NonNegative: the reason says >= 0",
      checkNavValue("nav_hysteresis_m", -1.0).find(">= 0") != std::string::npos);
    chk("Any: nav_fence_len_m <= 0 is the off switch and is accepted",
      !refused("nav_fence_len_m", 0.0) && !refused("nav_fence_len_m", -1.0));
    chk("whole: nav_invalid_confirm 0 refused, 1 accepted",
      refused("nav_invalid_confirm", 0.0) && !refused("nav_invalid_confirm", 1.0));
    chk("whole: nav_orbit_points 0 and -3 refused, 12 accepted",
      refused("nav_orbit_points", 0.0) && refused("nav_orbit_points", -3.0) &&
      !refused("nav_orbit_points", 12.0));
    chk("a structural nav_* key is not ours to judge", !refused("nav_mode", nan) && !refused("nav_hazard_rate_hz", -5.0));
    chk("a non-nav key is not ours either", !refused("tick_hz", -1.0));
  }

  std::printf("5. checkNavParams names the offender\n");
  {
    NavParams p;
    p.orbit_radius_m = 0.0;
    const std::string why = checkNavParams(p);
    chk("a zero orbit radius is found by name", why.find("nav_orbit_radius_m") != std::string::npos);
    p = NavParams{};
    p.orbit_points = 0;
    chk("a zero ring is found by name", checkNavParams(p).find("nav_orbit_points") != std::string::npos);
  }

  std::printf("6. the diff\n");
  {
    NavParams a, b;
    chk("identical -> no change", diffNavParams(a, b).empty() && describeNavChanges({}).empty());
    b.hard_m = 1.0;
    b.orbit_points = 12;
    b.gate_approach_m = 3.0;
    const std::vector<NavChange> d = diffNavParams(a, b);
    chk("three differences found", d.size() == 3);
    chk("in table order: reals first, then ints",
      d.size() == 3 && d[0].name == "nav_hard_m" && d[1].name == "nav_gate_approach_m" &&
      d[2].name == "nav_orbit_points");
    chk("before and after carried",
      d.size() == 3 && d[0].before == 0.8 && d[0].after == 1.0 && d[2].before == 8.0 && d[2].after == 12.0);
    chk("the log line", describeNavChanges(d) ==
      "nav_hard_m 0.8 -> 1, nav_gate_approach_m 8 -> 3, nav_orbit_points 8 -> 12");
    NavParams n1, n2;
    n1.soft_m = nan;
    n2.soft_m = nan;
    chk("NaN against NaN is a change (never silently 'equal')", diffNavParams(n1, n2).size() == 1);
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  std::printf("%s\n", g_fails == 0 ? "PASS" : "FAIL");
  return g_fails == 0 ? 0 : 1;
}
