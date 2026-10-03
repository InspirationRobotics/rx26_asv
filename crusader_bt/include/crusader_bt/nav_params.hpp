// nav_params.hpp — bt_runner_node's nav_* planner knobs as DATA: one table that declares
// them, re-reads them, checks them and diffs them.
//
// Header-only, stdlib + path_math.hpp. No ROS, so the whole of it is tested off-boat:
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include test/test_nav_params.cpp
//
// WHY THIS EXISTS. path::NavParams was filled ONCE, at construction, from the node's
// parameters. A later `ros2 param set /bt_runner_node nav_hard_m 1.2` (or the ground
// station's Tuning tab, which does exactly that) changed the PARAMETER and left the
// planner on the old number: the set succeeded, the number on the page changed, and
// nothing about the boat did. That is the posture crusader_common.param_utils calls the
// worst one available - a declared-but-ignored parameter.
//
// So bt_runner_node now re-reads every key below into ctx_->nav when a mission goal is
// ACCEPTED (never mid-mission), and refuses a value the planner cannot work with at the
// moment it is set, in words. Four consumers (declare, read, check, diff), one list of
// names. A key added to NavParams and not to this table is caught by test_nav_params.
#ifndef CRUSADER_BT__NAV_PARAMS_HPP_
#define CRUSADER_BT__NAV_PARAMS_HPP_

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <iterator>
#include <string>
#include <vector>

#include "crusader_bt/path_math.hpp"

namespace crusader_bt
{
namespace path
{

/// What a REAL knob's value must satisfy on top of being finite. Chosen per key from how the
/// planner USES it, not from what sounds sensible: a step of 0 m never advances (gate_step_m makes
/// every gate search fail), a 0 s plan timeout times out every request before the planner can
/// answer, a 0 m radius is not a buoy - those are Positive. A margin or an inflation of 0 is a
/// legitimate "none" - NonNegative. fence_len_m <= 0 is the documented off switch - Any.
enum class NavBound {Positive, NonNegative, Any};

/// One planner knob: a double or an int member of NavParams, by its ROS parameter name.
struct NavKnob
{
  const char * name;
  double NavParams::* real;   ///< exactly one of real / whole is set
  int NavParams::* whole;
  NavBound bound;             ///< reals only
  int min;                    ///< wholes only: the smallest accepted value

  bool isWhole() const {return whole != nullptr;}
  double get(const NavParams & p) const
  {
    return real != nullptr ? p.*real : static_cast<double>(p.*whole);
  }
  void set(NavParams & p, double v) const
  {
    if (real != nullptr) {p.*real = v;} else {p.*whole = static_cast<int>(v);}
  }
};

constexpr NavKnob realKnob(const char * name, double NavParams::* m, NavBound b)
{
  return {name, m, nullptr, b, 0};
}
constexpr NavKnob wholeKnob(const char * name, int NavParams::* m, int min)
{
  return {name, nullptr, m, NavBound::Any, min};
}

/// Every knob of NavParams, in the order of the struct (reals first, then the two ints), which
/// is the order a diff reports.
inline constexpr NavKnob kNavKnobs[] = {
  realKnob("nav_hard_m", &NavParams::hard_m, NavBound::Positive),
  realKnob("nav_soft_m", &NavParams::soft_m, NavBound::Positive),
  realKnob("nav_buoy_radius_m", &NavParams::buoy_radius_m, NavBound::Positive),
  realKnob("nav_track_radius_m", &NavParams::track_radius_m, NavBound::Positive),
  realKnob("nav_exempt_radius_m", &NavParams::exempt_radius_m, NavBound::NonNegative),
  realKnob("nav_lookahead_m", &NavParams::lookahead_m, NavBound::Positive),
  realKnob("nav_lookahead_min_m", &NavParams::lookahead_min_m, NavBound::NonNegative),
  realKnob("nav_max_chord_dev_m", &NavParams::max_chord_dev_m, NavBound::NonNegative),
  realKnob("nav_wp_radius_m", &NavParams::wp_radius_m, NavBound::Positive),
  realKnob("nav_replan_period_s", &NavParams::replan_period_s, NavBound::Positive),
  realKnob("nav_min_request_gap_s", &NavParams::min_request_gap_s, NavBound::NonNegative),
  realKnob("nav_check_period_s", &NavParams::check_period_s, NavBound::Positive),
  realKnob("nav_plan_timeout_s", &NavParams::plan_timeout_s, NavBound::Positive),
  realKnob("nav_first_plan_wait_s", &NavParams::first_plan_wait_s, NavBound::Positive),
  realKnob("nav_hysteresis_frac", &NavParams::hysteresis_frac, NavBound::NonNegative),
  realKnob("nav_hysteresis_m", &NavParams::hysteresis_m, NavBound::NonNegative),
  realKnob("nav_goal_replan_m", &NavParams::goal_replan_m, NavBound::NonNegative),
  realKnob("nav_clip_radius_m", &NavParams::clip_radius_m, NavBound::Positive),
  realKnob("nav_clear_after_s", &NavParams::clear_after_s, NavBound::NonNegative),
  realKnob("nav_unblock_reset_s", &NavParams::unblock_reset_s, NavBound::NonNegative),
  realKnob("nav_escape_margin_m", &NavParams::escape_margin_m, NavBound::NonNegative),
  realKnob("nav_goal_margin_m", &NavParams::goal_margin_m, NavBound::NonNegative),
  realKnob("nav_local_check_tol_m", &NavParams::local_check_tol_m, NavBound::NonNegative),
  realKnob("nav_orbit_max_push_m", &NavParams::orbit_max_push_m, NavBound::NonNegative),
  realKnob("nav_orbit_clear_m", &NavParams::orbit_clear_m, NavBound::NonNegative),
  realKnob("nav_dock_finger_len_m", &NavParams::dock_finger_len_m, NavBound::Positive),
  realKnob("nav_dock_finger_w_m", &NavParams::dock_finger_w_m, NavBound::Positive),
  realKnob("nav_dock_slip_w_m", &NavParams::dock_slip_w_m, NavBound::Positive),
  realKnob("nav_dock_deck_depth_m", &NavParams::dock_deck_depth_m, NavBound::Positive),
  realKnob("nav_fence_len_m", &NavParams::fence_len_m, NavBound::Any),   // <= 0 switches fences off
  realKnob("nav_fence_spacing_m", &NavParams::fence_spacing_m, NavBound::Positive),
  realKnob("nav_fence_radius_m", &NavParams::fence_radius_m, NavBound::Positive),
  realKnob("nav_fence_clear_m", &NavParams::fence_clear_m, NavBound::NonNegative),
  realKnob("nav_gate_clear_m", &NavParams::gate_clear_m, NavBound::NonNegative),
  realKnob("nav_gate_min_standoff_m", &NavParams::gate_min_standoff_m, NavBound::NonNegative),
  realKnob("nav_gate_min_approach_m", &NavParams::gate_min_approach_m, NavBound::NonNegative),
  realKnob("nav_gate_margin_m", &NavParams::gate_margin_m, NavBound::NonNegative),
  realKnob("nav_gate_step_m", &NavParams::gate_step_m, NavBound::Positive),
  realKnob("nav_gate_tight_clear_m", &NavParams::gate_tight_clear_m, NavBound::NonNegative),
  realKnob("nav_gate_tight_margin_m", &NavParams::gate_tight_margin_m, NavBound::NonNegative),
  realKnob("nav_orbit_radius_m", &NavParams::orbit_radius_m, NavBound::Positive),
  realKnob("nav_orbit_tolerance_m", &NavParams::orbit_tolerance_m, NavBound::Positive),
  realKnob("nav_gate_standoff_m", &NavParams::gate_standoff_m, NavBound::Positive),
  realKnob("nav_gate_approach_m", &NavParams::gate_approach_m, NavBound::Positive),
  realKnob("nav_goal_max_move_m", &NavParams::goal_max_move_m, NavBound::NonNegative),
  // invalid_confirm is clamped to >= 1 where it is used, and orbitRing needs at least one point:
  // below that the knob is not "off", it is a mission that fails.
  wholeKnob("nav_invalid_confirm", &NavParams::invalid_confirm, 1),
  wholeKnob("nav_orbit_points", &NavParams::orbit_points, 1),
};

/// The knob called `name`, or nullptr (a structural nav_* key, or no nav_ key at all).
inline const NavKnob * findNavKnob(const std::string & name)
{
  for (const NavKnob & k : kNavKnobs) {
    if (name == k.name) {return &k;}
  }
  return nullptr;
}

namespace detail
{

inline std::string num(double v)
{
  char buf[40];
  std::snprintf(buf, sizeof buf, "%.10g", v);
  return buf;
}

}  // namespace detail

/// A knob's accepted range in words, for its parameter description.
inline std::string navRangeText(const NavKnob & k)
{
  if (k.isWhole()) {return ">= " + std::to_string(k.min);}
  switch (k.bound) {
    case NavBound::Positive: return "> 0";
    case NavBound::NonNegative: return ">= 0";
    default: return "any finite number";
  }
}

/// "" when `value` is acceptable for the planner knob `name`, or `name` is not a planner knob
/// (other nav_* keys are structural and not ours to judge); otherwise WHY not, as a sentence
/// for the node's set-callback to hand back. A NaN or an infinity is refused for every knob.
inline std::string checkNavValue(const std::string & name, double value)
{
  const NavKnob * k = findNavKnob(name);
  if (k == nullptr) {return "";}
  auto refuse = [&](const std::string & need) {
      return name + "=" + detail::num(value) + " is refused: " + need;
    };
  if (!std::isfinite(value)) {
    return refuse(k->isWhole() ? "it must be a whole number" : "it must be a finite number");
  }
  if (k->isWhole()) {
    return value < k->min ? refuse("it must be >= " + std::to_string(k->min)) : "";
  }
  if (k->bound == NavBound::Positive && !(value > 0.0)) {
    return refuse("it must be > 0 (a zero or negative size or period breaks the planner)");
  }
  if (k->bound == NavBound::NonNegative && !(value >= 0.0)) {return refuse("it must be >= 0");}
  return "";
}

/// The first knob of `p` that checkNavValue refuses, or "" when every one passes. For startup:
/// a YAML value the planner cannot use should stop the node, not wait for a mission to find it.
inline std::string checkNavParams(const NavParams & p)
{
  for (const NavKnob & k : kNavKnobs) {
    const std::string why = checkNavValue(k.name, k.get(p));
    if (!why.empty()) {return why;}
  }
  return "";
}

/// One knob whose value differs between two NavParams.
struct NavChange
{
  std::string name;
  double before = 0.0, after = 0.0;
};

/// Every knob whose value differs, in table order. NaN != NaN counts as a change.
inline std::vector<NavChange> diffNavParams(const NavParams & before, const NavParams & after)
{
  std::vector<NavChange> out;
  for (const NavKnob & k : kNavKnobs) {
    const double a = k.get(before), b = k.get(after);
    if (!(a == b)) {out.push_back({k.name, a, b});}
  }
  return out;
}

/// "nav_hard_m 0.8 -> 1, nav_soft_m 2 -> 1.2": the log line's body.
inline std::string describeNavChanges(const std::vector<NavChange> & changes)
{
  std::string s;
  for (const NavChange & c : changes) {
    if (!s.empty()) {s += ", ";}
    s += c.name + " " + detail::num(c.before) + " -> " + detail::num(c.after);
  }
  return s;
}

}  // namespace path
}  // namespace crusader_bt

#endif  // CRUSADER_BT__NAV_PARAMS_HPP_
