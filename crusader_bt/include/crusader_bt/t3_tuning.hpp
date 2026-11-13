// t3_tuning.hpp — Task 3's control knobs, set per run from the GCS Run panel ("Task 3 control").
//
// Each knob is a bt_runner_node parameter of the same name, read at Start (with the rest of the
// Run panel) and put on the blackboard, where the Task 3 trees use it as a port: ApproachBay's
// kp_lat="{t3_app_kp_lat}", StrafeKeep's, StrafeSearch's, and the hold distance. So a value set
// in the GCS applies from the next Start, with no rebuild, and every GOAL log line is followed
// by the values that run used.
//
// THE SAME LIST lives in two more places, which test_operator_settings.py checks against this
// one: crusader_groundstation's operator_settings.py (kept across a bt_runner restart) and the
// laptop page's RUN_INPUTS (the rows). Add a knob in all three, and give its port a {t3_...}
// in every Task 3 tree that has the leaf.
//
// The defaults are the 2026-11-13 retune after the boat hit the left finger of the bay: more
// sideways authority above the FC's 30 us RC dead zone (RC4_DZ, 2026-10-22 dump), more damping
// to brake the search's slide, and a slower slide.
//
// Pure: no ROS, no BT.CPP.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace crusader_bt
{
namespace t3tune
{

struct Knob
{
  const char * key;   ///< the bt_runner_node parameter and the blackboard key
  double def, lo, hi;
  const char * what;  ///< the parameter's description (units first)
};

inline constexpr Knob kKnobs[] = {
  // ---- where to dock
  {"t3_hold_m", 1.7, 0.8, 6.0, "m: the docked hold, camera to the bay's face, while firing"},
  {"t3_lat_offset_m", 0.0, -0.6, 0.6,
    "m: where to sit across the bay, + = LEFT of its centre facing the dock; 0 = centred"},
  {"t3_handover_m", 2.0, 1.0, 6.0, "m: the approach hands over to the hold this far from the face"},
  // ---- the search slide (StrafeSearch)
  {"t3_search_lat_us", 80.0, 40.0, 200.0, "us: the sideways stick while sliding along the dock"},
  // ---- the approach: line up on the bay, then drive in (ApproachBay)
  {"t3_app_kp_lat", 250.0, 0.0, 1000.0, "us per m off the bay's centre"},
  {"t3_app_kd_lat", 150.0, 0.0, 1000.0, "us per m/s sideways: brakes the slide"},
  {"t3_app_min_us", 45.0, 0.0, 150.0,
    "us: the smallest correction; the FC's 30 us dead zone plus what moves the hull"},
  {"t3_app_strafe_us", 100.0, 20.0, 300.0, "us: the sideways stick cap"},
  {"t3_app_fwd_us", 60.0, 0.0, 200.0, "us: the forward stick driving in"},
  {"t3_app_centre_tol_m", 0.08, 0.02, 1.0, "m: centred = this close to the bay's centre ..."},
  {"t3_app_centred_s", 1.5, 0.0, 10.0, "s: ... for this long, then drive in"},
  {"t3_app_recentre_m", 0.20, 0.05, 2.0, "m: driving in, stop and re-centre past this"},
  {"t3_app_coast_s", 1.5, 0.0, 5.0,
    "s: no sideways stick while the offset would close within this anyway; 0 = always steer"},
  {"t3_app_kp_yaw", 4.0, 0.0, 50.0, "us per deg off the dock heading"},
  {"t3_app_kd_yaw", 3.0, 0.0, 50.0, "us per deg/s of turn"},
  // ---- the docked hold (StrafeKeep)
  {"t3_hold_kp_lat", 200.0, 0.0, 1000.0, "us per m off the bay's centre"},
  {"t3_hold_kd_lat", 200.0, 0.0, 1000.0, "us per m/s sideways"},
  {"t3_hold_ki_lat", 20.0, 0.0, 200.0, "us per m.s: holds against a steady set"},
  {"t3_hold_kp_fwd", 90.0, 0.0, 1000.0, "us per m off the hold distance"},
  {"t3_hold_kd_fwd", 60.0, 0.0, 1000.0, "us per m/s in or out"},
  {"t3_hold_ki_fwd", 40.0, 0.0, 200.0, "us per m.s: holds against a steady set"},
  {"t3_hold_kp_yaw", 5.0, 0.0, 50.0, "us per deg off square"},
  {"t3_hold_kd_yaw", 2.0, 0.0, 50.0, "us per deg/s of turn"},
  {"t3_hold_min_us", 45.0, 0.0, 150.0, "us: the smallest correction"},
  {"t3_hold_max_us", 120.0, 20.0, 300.0, "us: the stick cap, every axis"},
};

using Values = std::map<std::string, double>;

/// Every knob at its default.
inline Values defaults()
{
  Values v;
  for (const Knob & k : kKnobs) {v[k.key] = k.def;}
  return v;
}

/// `x` within the knob's range; not a number = its default.
inline double clampTo(const Knob & k, double x)
{
  return std::isfinite(x) ? std::clamp(x, k.lo, k.hi) : k.def;
}

/// `v[key]`, else the knob's default (no such knob: NaN).
inline double get(const Values & v, const std::string & key)
{
  const auto it = v.find(key);
  if (it != v.end()) {return it->second;}
  for (const Knob & k : kKnobs) {
    if (key == k.key) {return k.def;}
  }
  return NAN;
}

/// For the log: "hold_m=1.7 lat_offset_m=0 ...", every knob, the t3_ prefix dropped; a value
/// away from its default is starred.
inline std::string text(const Values & v)
{
  std::string s;
  char b[64];
  for (const Knob & k : kKnobs) {
    const double x = get(v, k.key);
    std::snprintf(b, sizeof(b), "%s%s=%g%s", s.empty() ? "" : " ", k.key + 3, x,
      x == k.def ? "" : "*");
    s += b;
  }
  return s;
}

/// Settings that fight each other, in words; empty = none.
inline std::vector<std::string> warnings(const Values & v)
{
  std::vector<std::string> w;
  if (get(v, "t3_handover_m") < get(v, "t3_hold_m")) {
    w.push_back("t3_handover_m is inside t3_hold_m: the hold backs out after the hand-over");
  }
  if (get(v, "t3_app_recentre_m") < get(v, "t3_app_centre_tol_m")) {
    w.push_back("t3_app_recentre_m is under t3_app_centre_tol_m: it stops driving in as soon as "
      "it starts");
  }
  if (get(v, "t3_app_min_us") >= get(v, "t3_app_strafe_us")) {
    w.push_back("t3_app_min_us is at or over t3_app_strafe_us: every sideways correction is "
      "the cap");
  }
  if (get(v, "t3_hold_min_us") >= get(v, "t3_hold_max_us")) {
    w.push_back("t3_hold_min_us is at or over t3_hold_max_us: every hold correction is the cap");
  }
  return w;
}

}  // namespace t3tune
}  // namespace crusader_bt
