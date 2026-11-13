// task4_math.hpp — Task 4 Disruptive (handbook 3.3.5): RoboCommand's virtual moving object.
// Header only, stdlib + nav_math + path_math, so test/test_task4_math.cpp runs off the boat.
//
// THE RULE. "All uncrewed systems operating in the impacted domains must avoid approaching
// within 10m of this object (as visible on the tracking display) based on its current and
// projected travel." RoboCommand's MovingObjectAlert gives its position, heading, speed, the
// domains it affects and a keep-out radius; nothing ends it (no All Clear in Disruptive).
//
// THE MODEL. Each alert is dead-reckoned from the moment it arrived: straight on, at its speed.
// A new alert within same_object_m of an object's predicted position is that object again (its
// position, course and speed replaced); farther away it is another object. The keep-out kept is
// its radius (the alert's, else fallback_radius_m) plus margin_m.
//
// WHAT THE BOAT DOES WITH IT (all of it only for objects whose domains include the USV):
//   * sweptHazard(): its keep-out swept over the next sweep_s is a hazard (Hazard.SRC_KEEPOUT) in
//     knownHazards(), so every leg that checks hazards already avoids it: the Task 1 planner
//     re-plans round it, a guarded straight leg holds rather than cross it.
//   * assess(): THREAT when, the boat staying where it is, the object's projected travel over
//     horizon_s comes inside the keep-out. The AvoidInvaders decorator then steps the boat aside
//     to pickEscape()'s point and waits there until it is clear again.
#ifndef CRUSADER_BT__TASK4_MATH_HPP_
#define CRUSADER_BT__TASK4_MATH_HPP_

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "crusader_bt/nav_math.hpp"
#include "crusader_bt/path_math.hpp"

namespace crusader_bt
{
namespace t4
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

/// The operator's settings: bt_runner_node t4_* (the GCS Run panel sets the first four).
struct Settings
{
  bool enabled = true;              ///< false: alerts are tracked and shown, never acted on
  double margin_m = 3.0;            ///< kept beyond the object's keep-out radius
  double fallback_radius_m = 10.0;  ///< the keep-out when an alert carries none (3.3.5: 10 m)
  double horizon_s = 40.0;          ///< how far ahead its projected travel counts as a threat
  double sweep_s = 20.0;            ///< the hazard the planners avoid: its travel over this long
  double expire_s = 900.0;          ///< an object not re-reported for this long is dropped
  double same_object_m = 30.0;      ///< a new alert this near an object's prediction updates it
  double boat_speed_mps = 0.5;      ///< the boat's GUIDED speed, for timing an escape
  double clear_hyst_m = 2.0;        ///< a THREAT ends only this far outside the keep-out
};

/// One reported object, dead-reckoned from t0.
struct Invader
{
  int id = 0;                       ///< ours, 1.. in order of first report
  nav::Vec2 p0;                     ///< where the alert put it, local metres
  nav::LatLon ll0;                  ///< ... as RoboCommand sent it
  double t0 = 0.0;                  ///< when that alert arrived (Context::now_s's clock)
  double first_t = 0.0;             ///< when it was first reported
  double heading_deg = 0.0;         ///< its course, deg true
  double speed_mps = 0.0;
  double radius_m = 10.0;           ///< its keep-out radius: the alert's, else the fallback
  bool radius_given = false;        ///< the alert carried a radius
  bool affects_usv = true;          ///< the alert's domains include the USV (or name none)
  std::string domains;              ///< as sent, e.g. "USV UAV", for the GCS
  int reports = 1;                  ///< alerts that updated it
};

/// One MovingObjectAlert as received (fields absent from the JSON are protobuf defaults: 0).
struct Alert
{
  double lat = kNaN, lon = kNaN;
  double heading_deg = 0.0;
  double speed_mps = 0.0;
  double radius_m = 0.0;            ///< 0 = none given
  std::vector<std::string> domains; ///< "USV", "UUV", "UAV" (TYPE_ prefix dropped)
  double seq = 0.0;                 ///< RxCommand.seq of the message that carried it; 0 = none
};

inline nav::Vec2 velocity(const Invader & v)
{
  return nav::headingVec(v.heading_deg) * std::max(0.0, v.speed_mps);
}

/// Where it is predicted to be at time t (t >= t0 projects forward; earlier is clamped to t0).
inline nav::Vec2 at(const Invader & v, double t)
{
  return v.p0 + velocity(v) * std::max(0.0, t - v.t0);
}

/// The distance to keep from its predicted position: its radius plus the operator's margin.
inline double keepout(const Invader & v, const Settings & s)
{
  return std::max(0.0, v.radius_m) + std::max(0.0, s.margin_m);
}

/// Closest approach of the object to a FIXED point q over [t, t + h]: the distance and when
/// (seconds after t).
struct Cpa
{
  double d = kInf;
  double dt = 0.0;
};

inline Cpa cpaFixed(const Invader & v, nav::Vec2 q, double t, double h)
{
  const nav::Vec2 p = at(v, t);
  const nav::Vec2 vel = velocity(v);
  const double v2 = nav::dot(vel, vel);
  double tau = 0.0;
  if (v2 > 1e-12 && h > 0.0) {tau = std::clamp(-nav::dot(p - q, vel) / v2, 0.0, h);}
  return {nav::norm(p + vel * tau - q), tau};
}

/// Closest approach while the boat drives straight from a to b at `speed`, starting at t, then
/// waits at b until t + h_total: the least distance between the two over that time.
inline Cpa cpaLeg(
  const Invader & v, nav::Vec2 a, nav::Vec2 b, double speed, double t, double h_total)
{
  const double len = nav::norm(b - a);
  const double ta = speed > 1e-6 ? len / speed : 0.0;
  Cpa best;
  // the drive, sampled every 0.25 s of travel or 0.5 m, whichever is finer
  const int n = std::max(1, static_cast<int>(std::ceil(std::max(ta / 0.25, len / 0.5))));
  for (int i = 0; i <= n; ++i) {
    const double f = static_cast<double>(i) / n;
    const nav::Vec2 q = a + (b - a) * f;
    const double tau = ta * f;
    const double d = nav::norm(at(v, t + tau) - q);
    if (d < best.d) {best = {d, tau};}
  }
  if (h_total > ta) {
    const Cpa w = cpaFixed(v, b, t + ta, h_total - ta);
    if (w.d < best.d) {best = {w.d, ta + w.dt};}
  }
  return best;
}

/// A convex stadium round the segment a-b, radius r, counter-clockwise; `arc` segments per end.
/// The ends' polygons are drawn OUTSIDE the true circles (their chords touch them), so nothing
/// within r of the segment is ever outside it.
inline std::vector<nav::Vec2> capsule(nav::Vec2 a, nav::Vec2 b, double r, int arc = 7)
{
  const nav::Vec2 d = nav::unit(b - a);
  const double base = std::atan2(d.y, d.x);
  r /= std::cos(nav::kPi / (2.0 * arc));
  std::vector<nav::Vec2> poly;
  poly.reserve(2 * (arc + 1));
  // round b: from -90 deg to +90 deg of the direction of travel, then round a: +90 to +270
  for (int i = 0; i <= arc; ++i) {
    const double ang = base - nav::kPi / 2.0 + nav::kPi * i / arc;
    poly.push_back(b + nav::Vec2{std::cos(ang), std::sin(ang)} * r);
  }
  for (int i = 0; i <= arc; ++i) {
    const double ang = base + nav::kPi / 2.0 + nav::kPi * i / arc;
    poly.push_back(a + nav::Vec2{std::cos(ang), std::sin(ang)} * r);
  }
  return poly;
}

/// The keep-out swept over [t, t + sweep_s], as a hazard the planners avoid: a circle while the
/// object stands still (moves under 0.5 m in that time), else a stadium along its course.
inline path::Hazard sweptHazard(const Invader & v, const Settings & s, double t)
{
  const nav::Vec2 a = at(v, t);
  const nav::Vec2 b = at(v, t + std::max(0.0, s.sweep_s));
  const double r = keepout(v, s);
  if (nav::norm(b - a) < 0.5) {
    return path::detail::circleHazard(path::HazardSource::Keepout, v.id, a, r);
  }
  return path::detail::polygonHazard(path::HazardSource::Keepout, v.id, capsule(a, b, r));
}

/// Objects that count: those affecting the USV, while Task 4 is enabled.
inline bool counts(const Invader & v, const Settings & s) {return s.enabled && v.affects_usv;}

/// Every counted object's swept keep-out, for knownHazards().
inline std::vector<path::Hazard> hazards(
  const std::vector<Invader> & objs, const Settings & s, double t)
{
  std::vector<path::Hazard> out;
  for (const Invader & v : objs) {
    if (counts(v, s) && path::detail::finite(v.p0)) {out.push_back(sweptHazard(v, s, t));}
  }
  return out;
}

// ------------------------------------------------------------------ alerts

/// The alert's domains include the USV (an alert naming none counts for every vehicle).
inline bool affectsUsv(const Alert & a)
{
  return a.domains.empty() ||
    std::find(a.domains.begin(), a.domains.end(), "USV") != a.domains.end();
}

/// Acknowledge this alert to RoboCommand? Once per object: the alert that made a NEW object,
/// when it affects the USV and carries the RxCommand.seq the acknowledgment must quote.
/// Scoring (section 4, Dynamic Incident Response, Disruptive): 25 points for "acknowledging
/// receipt"; IncidentAck is the protocol's only acknowledgment (3.4 names none for Disruptive).
inline bool shouldAck(bool new_object, const Alert & a)
{
  return new_object && affectsUsv(a) && a.seq > 0.0;
}

/// The IncidentAck report body for the alert carried by RxCommand `seq` (rx_reports.proto:
/// IncidentAck.command_seq = the command it answers).
inline std::string incidentAckJson(double seq)
{
  return "{\"command_seq\":" + std::to_string(static_cast<unsigned long long>(seq)) + "}";
}

/// Fold one alert in: the object it re-reports (nearest prediction within same_object_m) is
/// replaced, else it is a new one. `p` = the alert's position in local metres. Returns its id.
inline int ingest(
  std::vector<Invader> & objs, int & next_id, const Alert & a, nav::Vec2 p, double t,
  const Settings & s)
{
  Invader * same = nullptr;
  double best = s.same_object_m;
  for (Invader & v : objs) {
    const double d = nav::norm(at(v, t) - p);
    if (d <= best) {
      best = d;
      same = &v;
    }
  }
  Invader v;
  if (same != nullptr) {
    v = *same;
    ++v.reports;
  } else {
    v.id = next_id++;
    v.first_t = t;
  }
  v.p0 = p;
  v.ll0 = {a.lat, a.lon};
  v.t0 = t;
  v.heading_deg = std::isfinite(a.heading_deg) ? a.heading_deg : 0.0;
  v.speed_mps = std::isfinite(a.speed_mps) ? std::max(0.0, a.speed_mps) : 0.0;
  v.radius_given = std::isfinite(a.radius_m) && a.radius_m > 0.0;
  v.radius_m = v.radius_given ? a.radius_m : s.fallback_radius_m;
  v.affects_usv = affectsUsv(a);
  v.domains.clear();
  for (const std::string & d : a.domains) {v.domains += (v.domains.empty() ? "" : " ") + d;}
  if (same != nullptr) {
    *same = v;
  } else {
    objs.push_back(v);
  }
  return v.id;
}

/// Should this alert be folded in? A re-delivered or late one carries a seq no newer than the
/// last one applied (RxCommand.seq only goes up); applying it would put the object back where it
/// was. `last_seq` = the newest applied, updated here. seq 0 (absent) is always applied.
inline bool acceptSeq(double seq, double & last_seq)
{
  if (seq <= 0.0) {return true;}
  if (seq <= last_seq) {return false;}
  last_seq = seq;
  return true;
}

/// Drop objects not re-reported for expire_s. Returns how many went.
inline int expire(std::vector<Invader> & objs, double t, const Settings & s)
{
  const std::size_t n = objs.size();
  objs.erase(std::remove_if(objs.begin(), objs.end(), [&](const Invader & v) {
      return t - v.t0 > s.expire_s;
    }), objs.end());
  return static_cast<int>(n - objs.size());
}

// ---------------------------------------------------------------- the verdict

/// For one tick: is any counted object's projected travel coming inside its keep-out of the
/// boat, the boat staying put? `evading` widens the line by clear_hyst_m (it ends only clearly).
struct Assessment
{
  bool threat = false;
  int id = -1;                ///< the object that decides it (the nearest approach), -1 none
  double dist_m = kNaN;       ///< boat to that object's predicted position now
  double cpa_m = kNaN;        ///< its closest approach over horizon_s
  double cpa_s = kNaN;        ///< ... in this many seconds
  double keepout_m = kNaN;    ///< what it must stay outside
};

inline Assessment assess(
  const std::vector<Invader> & objs, const Settings & s, nav::Vec2 boat, double t, bool evading)
{
  Assessment out;
  double worst = kInf;        // the least (cpa - keepout): the tightest object decides
  for (const Invader & v : objs) {
    if (!counts(v, s) || !path::detail::finite(v.p0)) {continue;}
    const Cpa c = cpaFixed(v, boat, t, s.horizon_s);
    const double ko = keepout(v, s);
    const double slack = c.d - ko;
    if (slack < worst) {
      worst = slack;
      out.id = v.id;
      out.dist_m = nav::norm(at(v, t) - boat);
      out.cpa_m = c.d;
      out.cpa_s = c.dt;
      out.keepout_m = ko;
    }
  }
  out.threat = out.id >= 0 && worst < (evading ? s.clear_hyst_m : 0.0);
  return out;
}

// ------------------------------------------------------------------ the escape

/// Where to step aside to. `margin_m` = the least (approach - keepout) over every counted
/// object, driving there and then waiting until horizon_s past arrival; `end_m` = the same at the
/// point itself, from the moment the boat arrives. `safe` = margin_m >= 0 AND end_m >= the
/// hysteresis (Settings::clear_hyst_m): the threat is over there, not merely just outside the line.
struct Escape
{
  bool ok = false;            ///< a point was found (safe or the best there is)
  bool safe = false;          ///< the point is clear of every object's projected travel
  nav::Vec2 p;
  double margin_m = -kInf;
  double end_m = -kInf;
  double dist_m = 0.0;
};

/// The least of f(v) over every counted object (kInf when none count).
template<typename F>
inline double minOverCounted(const std::vector<Invader> & objs, const Settings & s, F f)
{
  double m = kInf;
  for (const Invader & v : objs) {
    if (counts(v, s) && path::detail::finite(v.p0)) {m = std::min(m, f(v));}
  }
  return m;
}

/// The worst approach of any counted object to a boat driving boat -> e (then waiting there):
/// approach minus keep-out, in metres.
inline double legMargin(
  const std::vector<Invader> & objs, const Settings & s, nav::Vec2 boat, nav::Vec2 e, double t)
{
  const double speed = std::max(0.1, s.boat_speed_mps);
  const double h_total = nav::norm(e - boat) / speed + s.horizon_s;
  return minOverCounted(objs, s, [&](const Invader & v) {
      return cpaLeg(v, boat, e, speed, t, h_total).d - keepout(v, s);
    });
}

/// The worst approach to the boat once it sits at e: from its arrival, horizon_s ahead, approach
/// minus keep-out. A leg's margin cannot tell a point beside a parked object from one away from
/// it (the start of the leg decides both); this can.
inline double endMargin(
  const std::vector<Invader> & objs, const Settings & s, nav::Vec2 boat, nav::Vec2 e, double t)
{
  const double arrive = t + nav::norm(e - boat) / std::max(0.1, s.boat_speed_mps);
  return minOverCounted(objs, s, [&](const Invader & v) {
      return cpaFixed(v, e, arrive, s.horizon_s).d - keepout(v, s);
    });
}

/// Driving to e, then waiting: the margins and whether e is safe (see Escape).
inline Escape evalEscape(
  const std::vector<Invader> & objs, const Settings & s, nav::Vec2 boat, nav::Vec2 e, double t)
{
  Escape x;
  x.ok = true;
  x.p = e;
  x.dist_m = nav::norm(e - boat);
  x.margin_m = legMargin(objs, s, boat, e, t);
  x.end_m = endMargin(objs, s, boat, e, t);
  x.safe = x.margin_m >= 0.0 && x.end_m >= s.clear_hyst_m;
  return x;
}

/// The fixed field may not be crossed: every point of a->b keeps hard_m from it, or - for a
/// boat that starts closer than that (docked) - never gets closer than where it started.
inline bool legClearOfField(
  nav::Vec2 a, nav::Vec2 b, const std::vector<path::Hazard> & field, double hard_m)
{
  if (field.empty()) {return true;}
  const double need = std::min(hard_m, path::minClearance(field, a) - 0.05);
  const double len = nav::norm(b - a);
  const int n = std::max(1, static_cast<int>(std::ceil(len / 0.25)));
  for (int i = 1; i <= n; ++i) {
    if (path::minClearance(field, a + (b - a) * (static_cast<double>(i) / n)) < need) {
      return false;
    }
  }
  return true;
}

/// Inside the course polygon (local metres, any winding); an empty or degenerate one is
/// "everywhere".
inline bool insideCourse(const std::vector<nav::Vec2> & course, nav::Vec2 p)
{
  if (course.size() < 3) {return true;}
  bool in = false;
  for (std::size_t i = 0, j = course.size() - 1; i < course.size(); j = i++) {
    const nav::Vec2 a = course[i], b = course[j];
    if ((a.y > p.y) != (b.y > p.y) &&
      p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)
    {
      in = !in;
    }
  }
  return in;
}

/// The nearest SAFE point (24 bearings, 3 to 30 m out) the boat can drive to without crossing
/// the fixed field (buoys, dock: `field`, hard_m) or leaving the course; none safe: the one with
/// the most margin. Candidates rank on the leg margin, then (equal margin: a boat already inside
/// a keep-out gets the start's own distance from every way that does not close) on the end
/// margin, so the boat goes straight away from a parked or receding object, not 3 m sideways, and
/// never settles just outside the line to wait forever. ok = false only when every candidate
/// crosses the field or leaves the course.
inline Escape pickEscape(
  const std::vector<Invader> & objs, const Settings & s, nav::Vec2 boat, double t,
  const std::vector<path::Hazard> & field, double hard_m,
  const std::vector<nav::Vec2> & course = {})
{
  static const double kRings[] = {3.0, 5.0, 7.0, 9.0, 12.0, 15.0, 18.0, 22.0, 26.0, 30.0};
  // x ranks above y: more leg margin, else (exactly equal) more end margin
  auto better = [](const Escape & x, const Escape & y) {
      return x.margin_m > y.margin_m || (x.margin_m == y.margin_m && x.end_m > y.end_m);
    };
  Escape best;
  for (double r : kRings) {
    for (int k = 0; k < 24; ++k) {
      const nav::Vec2 e = boat + nav::headingVec(k * 15.0) * r;
      if (!insideCourse(course, e) || !legClearOfField(boat, e, field, hard_m)) {continue;}
      Escape c = evalEscape(objs, s, boat, e, t);
      c.dist_m = r;
      // safe beats unsafe; among safe the nearest (rings go outward, so the first ring that has
      // one), the best on that ring; among unsafe the best of all
      if (best.safe) {
        if (c.safe && r <= best.dist_m + 1e-9 && better(c, best)) {best = c;}
        continue;
      }
      if (c.safe || !best.ok || better(c, best)) {best = c;}
    }
  }
  return best;
}

// ------------------------------------------------------------------ the detour
//
// Not Task 4's own, but the same geometry: a leg BETWEEN tasks (the Task 1 EXIT -> the Task 3
// approach point) is a guarded straight one with nav_mode off, and a straight line through the
// buoy field holds in front of the first buoy and FAILS. One point beside the field turns it
// into two straight legs that clear it.

/// The leg p -> q keeps clear_m of every hazard in `field`, except one that a RELAXED end is
/// already closer to: that one only as close as the end is now, minus 5 cm (the boat ends its
/// EXIT orbit ~1.5 m off the buoy; the Task 3 approach point may sit near the dock).
inline bool legKeeps(
  nav::Vec2 p, nav::Vec2 q, const std::vector<path::Hazard> & field, double clear_m,
  bool relax_p, bool relax_q)
{
  if (field.empty()) {return true;}
  std::vector<double> need(field.size(), clear_m);
  for (std::size_t i = 0; i < field.size(); ++i) {
    if (relax_p) {need[i] = std::min(need[i], path::clearance(field[i], p) - 0.05);}
    if (relax_q) {need[i] = std::min(need[i], path::clearance(field[i], q) - 0.05);}
  }
  const int n = std::max(1, static_cast<int>(std::ceil(nav::norm(q - p) / 0.25)));
  for (int k = 0; k <= n; ++k) {
    const nav::Vec2 x = p + (q - p) * (static_cast<double>(k) / n);
    for (std::size_t i = 0; i < field.size(); ++i) {
      if (path::clearance(field[i], x) < need[i]) {return false;}
    }
  }
  return true;
}

enum class Route {Straight, Via, None};

struct Detour
{
  Route route = Route::None;
  nav::Vec2 via;             ///< Via: drive here first
  double clear_m = 0.0;      ///< what the route keeps from the field (Straight, Via)
  double len_m = 0.0;        ///< a -> (via) -> b
};

/// From a to b round the fixed field, for each clearance of `clears_m` in turn (the first that
/// works wins): Straight when the straight leg keeps it; else Via the point v with the shortest
/// a -> v -> b whose two legs keep it (a and b relaxed as legKeeps says; v is not: it is open
/// water), among points round the leg's midpoint 2..max_m out every 10 deg, inside the course.
/// None: no single point within max_m does it at any of them.
inline Detour detour(
  nav::Vec2 a, nav::Vec2 b, const std::vector<path::Hazard> & field,
  const std::vector<double> & clears_m, const std::vector<nav::Vec2> & course = {},
  double max_m = 40.0)
{
  struct Cand {nav::Vec2 v; double len;};
  std::vector<Cand> cands;
  const nav::Vec2 mid = (a + b) * 0.5;
  for (double r = 2.0; r <= max_m + 1e-9; r += 2.0) {
    for (int k = 0; k < 36; ++k) {
      const nav::Vec2 v = mid + nav::headingVec(k * 10.0) * r;
      if (insideCourse(course, v)) {cands.push_back({v, nav::norm(v - a) + nav::norm(b - v)});}
    }
  }
  std::stable_sort(cands.begin(), cands.end(),
    [](const Cand & x, const Cand & y) {return x.len < y.len;});
  Detour d;
  for (double c : clears_m) {
    if (legKeeps(a, b, field, c, true, true)) {
      d.route = Route::Straight;
      d.clear_m = c;
      d.len_m = nav::norm(b - a);
      return d;
    }
    for (const Cand & k : cands) {
      if (legKeeps(a, k.v, field, c, true, false) && legKeeps(k.v, b, field, c, false, true)) {
        d.route = Route::Via;
        d.via = k.v;
        d.clear_m = c;
        d.len_m = k.len;
        return d;
      }
    }
  }
  return d;
}

// ------------------------------------------------------------------ the JSON

namespace detail
{
/// The object (balanced braces) that follows `"key":` anywhere in `s`, or "" if none.
inline std::string objectAfter(const std::string & s, const std::string & key)
{
  const std::size_t k = s.find("\"" + key + "\"");
  if (k == std::string::npos) {return "";}
  const std::size_t open = s.find('{', k + key.size() + 2);
  if (open == std::string::npos) {return "";}
  int depth = 0;
  bool quoted = false;
  for (std::size_t i = open; i < s.size(); ++i) {
    const char ch = s[i];
    if (ch == '"' && (i == 0 || s[i - 1] != '\\')) {quoted = !quoted;}
    if (quoted) {continue;}
    if (ch == '{') {++depth;}
    if (ch == '}' && --depth == 0) {return s.substr(open, i - open + 1);}
  }
  return "";
}

/// The number after `"key":` in `s` (a quoted "NaN"/"Infinity" or anything unparsable: NaN);
/// `found` says whether the key was there at all (protobuf JSON omits zero values).
inline double numberAfter(const std::string & s, const std::string & key, bool * found)
{
  *found = false;
  const std::size_t k = s.find("\"" + key + "\"");
  if (k == std::string::npos) {return kNaN;}
  std::size_t i = s.find(':', k + key.size() + 2);
  if (i == std::string::npos) {return kNaN;}
  *found = true;
  ++i;
  while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) {++i;}
  if (i < s.size() && s[i] == '"') {++i;}       // protobuf quotes int64 and non-finite values
  const char * start = s.c_str() + i;
  char * end = nullptr;
  const double v = std::strtod(start, &end);
  return end == start ? kNaN : v;
}

/// Either spelling of a field: proto (snake_case, what the OCS sends) or JSON (lowerCamel).
inline double numberEither(const std::string & s, const char * snake, const char * camel, bool * found)
{
  double v = numberAfter(s, snake, found);
  if (!*found) {v = numberAfter(s, camel, found);}
  return v;
}
}  // namespace detail

/// A MovingObjectAlert in one /crsd/ocs_command message (the OCS relays RoboCommand's RxCommand
/// as protobuf JSON, proto field names). False when the message is not one or has no position.
/// Absent numbers are protobuf's zero (heading 0, speed 0, radius 0 = none given).
/// affected_vehicle_types: names ("TYPE_USV") or numbers (1 USV, 2 UUV, 3 UAV).
inline bool parseAlert(const std::string & json, Alert * out)
{
  std::string obj = detail::objectAfter(json, "moving_object_alert");
  if (obj.empty()) {obj = detail::objectAfter(json, "movingObjectAlert");}
  if (obj.empty()) {return false;}
  Alert a;
  bool f = false;
  a.lat = detail::numberAfter(obj, "latitude", &f);
  a.lon = detail::numberAfter(obj, "longitude", &f);
  if (!std::isfinite(a.lat) || !std::isfinite(a.lon) || std::fabs(a.lat) > 90.0 ||
    std::fabs(a.lon) > 180.0 || (a.lat == 0.0 && a.lon == 0.0))
  {
    return false;
  }
  a.seq = detail::numberAfter(json, "seq", &f);         // RxCommand.seq: the top-level key
  if (!f || !std::isfinite(a.seq) || a.seq < 0.0) {a.seq = 0.0;}
  a.heading_deg = detail::numberEither(obj, "heading_deg", "headingDeg", &f);
  if (!f) {a.heading_deg = 0.0;}
  a.speed_mps = detail::numberEither(obj, "speed_mps", "speedMps", &f);
  if (!f) {a.speed_mps = 0.0;}
  a.radius_m = detail::numberEither(obj, "keep_out_radius_m", "keepOutRadiusM", &f);
  if (!f) {a.radius_m = 0.0;}
  std::size_t k = obj.find("\"affected_vehicle_types\"");
  if (k == std::string::npos) {k = obj.find("\"affectedVehicleTypes\"");}
  if (k != std::string::npos) {
    const std::size_t open = obj.find('[', k);
    const std::size_t close = open == std::string::npos ? open : obj.find(']', open);
    if (close != std::string::npos) {
      const std::string list = obj.substr(open + 1, close - open - 1);
      auto add = [&a](const std::string & d) {
          if (std::find(a.domains.begin(), a.domains.end(), d) == a.domains.end()) {
            a.domains.push_back(d);
          }
        };
      if (list.find("USV") != std::string::npos) {add("USV");}
      if (list.find("UUV") != std::string::npos) {add("UUV");}
      if (list.find("UAV") != std::string::npos) {add("UAV");}
      for (std::size_t i = 0; i < list.size(); ++i) {       // numeric enums
        if (list[i] >= '1' && list[i] <= '3' && (i == 0 || !std::isalnum(
            static_cast<unsigned char>(list[i - 1]))) &&
          (i + 1 >= list.size() || !std::isalnum(static_cast<unsigned char>(list[i + 1]))))
        {
          add(list[i] == '1' ? "USV" : list[i] == '2' ? "UUV" : "UAV");
        }
      }
    }
  }
  *out = a;
  return true;
}

/// An RxCommand that ends the alerts: AllClear (not part of Disruptive, but harmless to obey).
inline bool isAllClear(const std::string & json)
{
  return json.find("\"all_clear\"") != std::string::npos ||
         json.find("\"allClear\"") != std::string::npos;
}

}  // namespace t4
}  // namespace crusader_bt

#endif  // CRUSADER_BT__TASK4_MATH_HPP_
