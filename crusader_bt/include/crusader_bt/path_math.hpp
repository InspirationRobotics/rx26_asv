// path_math.hpp — the geometry the planned legs and the costmap share, and nothing else.
//
// Header-only, stdlib + nav_math.hpp + dock_math.hpp. No ROS, no Nav2, no
// BehaviorTree.CPP. Same contract as nav_math.hpp, for the same reason: this is
// where a sign error or an off-by-one MIRRORS THE WORLD and shows up first on the
// water, and it is the part that compiles and runs on a laptop in a second:
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include test/test_path_math.cpp
//
// Spec: docs/nav2_avoidance_spec.md section 5.2 (this API is exact; WP2 codes
// against it). planned_leg.hpp is the state machine built on top.
//
// THE FRAME. Vec2 is the BT's local ENU plane (x east, y north). The TF frame
// `map`, ctx->origin and every HazardArray coordinate are THE SAME plane (spec
// section 2), so a hazard here is drawn by the costmap's HazardLayer at the same
// metres with no conversion. Nothing in this file projects lat/lon.
//
// THE ONE IDEA. A hazard is a shape with a signed distance (clearance()) to its
// LETHAL boundary. "Hard" (0.8 m) and "soft" (2.0 m) clearance are not baked
// into the shapes: hazards are drawn at PHYSICAL size (+ an optional keepout) and
// every check asks "is the clearance at least X". That is what lets the BT's
// local checks and Nav2's inflated costmap agree about the same field while the
// costmap does the inflation and this file does the arithmetic.
#ifndef CRUSADER_BT__PATH_MATH_HPP_
#define CRUSADER_BT__PATH_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "crusader_bt/dock_math.hpp"
#include "crusader_bt/nav_math.hpp"

namespace crusader_bt
{
namespace path
{

using nav::Vec2;

enum class HazardKind {Circle = 0, Polygon = 1};                              // == Hazard.msg kind
enum class HazardSource {PlanBuoy = 0, Track = 1, Dock = 2, Keepout = 3};     // == Hazard.msg SRC_*

/// One known hazard. A circle uses c and r; a polygon uses poly (convex, vertices
/// counter-clockwise, as Hazard.msg says). `keepout` pads either beyond the
/// physical surface.
struct Hazard
{
  HazardKind kind = HazardKind::Circle;
  HazardSource source = HazardSource::PlanBuoy;
  int id = -1;
  Vec2 c;
  double r = 0.0;
  std::vector<Vec2> poly;
  double keepout = 0.0;
};

/// Defaults == the `nav_*` keys of crusader_params.yaml bt_runner_node (spec 5.8).
/// Keep the two in step: check_config cross-checks several of them.
struct NavParams
{
  double hard_m = 0.8, soft_m = 2.0;
  double buoy_radius_m = 0.30, track_radius_m = 0.30, exempt_radius_m = 1.0;
  double lookahead_m = 5.0, lookahead_min_m = 3.0, max_chord_dev_m = 0.3, wp_radius_m = 2.0;
  double replan_period_s = 0.5, min_request_gap_s = 0.2, check_period_s = 0.1;
  double plan_timeout_s = 1.0, first_plan_wait_s = 1.0;
  int invalid_confirm = 2;
  double hysteresis_frac = 0.2, hysteresis_m = 3.0, goal_replan_m = 1.0, clip_radius_m = 35.0;
  double clear_after_s = 5.0, unblock_reset_s = 3.0, escape_margin_m = 0.5, goal_margin_m = 0.3;
  double local_check_tol_m = 0.1, orbit_max_push_m = 3.0, orbit_clear_m = 1.4;
  double dock_finger_len_m = 2.0, dock_finger_w_m = 0.5, dock_slip_w_m = 1.5,
    dock_deck_depth_m = 1.0;
};

namespace detail
{

constexpr double kInf = std::numeric_limits<double>::infinity();

inline bool finite(Vec2 v) {return std::isfinite(v.x) && std::isfinite(v.y);}

/// Distance from p to the segment a-b. a == b is a point.
inline double distToSegment(Vec2 p, Vec2 a, Vec2 b)
{
  const Vec2 ab = b - a;
  const double l2 = nav::dot(ab, ab);
  if (l2 < 1e-18) {return nav::norm(p - a);}
  const double t = std::clamp(nav::dot(p - a, ab) / l2, 0.0, 1.0);
  return nav::norm(p - (a + ab * t));
}

/// Signed distance from p to a convex polygon: positive outside, negative inside.
/// Winding does not matter (Hazard.msg says counter-clockwise, but a clockwise
/// polygon must not silently become "everything is outside"). Fewer than three
/// vertices degrade to a segment or a point; none is +inf (nothing to hit).
inline double polygonSignedDistance(const std::vector<Vec2> & poly, Vec2 p)
{
  const std::size_t n = poly.size();
  if (n == 0) {return kInf;}
  double d = kInf;
  bool all_left = true, all_right = true;
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 a = poly[i], b = poly[(i + 1) % n];
    d = std::min(d, distToSegment(p, a, b));
    const double cr = nav::cross(b - a, p - a);
    if (cr < 0.0) {all_left = false;}
    if (cr > 0.0) {all_right = false;}
  }
  return (n >= 3 && (all_left || all_right)) ? -d : d;
}

/// f(point) at a, then every step_m along a->b, then b: n equal steps of length
/// <= step_m, so b itself is always sampled. Stops and returns false at the first
/// f that returns false. The one place the sampling loop lives: segmentClear and
/// escapeStart both walk a segment the same way, and a step that skipped b would
/// miss exactly the hazard sitting at the goal.
template<typename F>
bool walkSegment(Vec2 a, Vec2 b, double step_m, F f)
{
  const double len = nav::norm(b - a);
  const double step = step_m > 1e-6 ? step_m : 0.1;
  const int n = len < 1e-12 ? 0 : static_cast<int>(std::ceil(len / step));
  for (int k = 0; k <= n; ++k) {
    const double t = n == 0 ? 0.0 : static_cast<double>(k) / n;
    if (!f(a + (b - a) * t)) {return false;}
  }
  return true;
}

/// An axis-aligned rectangle in the frame (u, v) about o, returned counter-
/// clockwise IN THE WORLD whichever handedness (u, v) has. The dock frame
/// (right = starboardOf(-out), out) is left-handed, so a naive corner order
/// would hand the layer clockwise polygons.
inline std::vector<Vec2> rect(
  Vec2 o, Vec2 u, Vec2 v, double u0, double u1, double v0, double v1)
{
  std::vector<Vec2> q{
    o + u * u0 + v * v0, o + u * u1 + v * v0, o + u * u1 + v * v1, o + u * u0 + v * v1};
  if (nav::cross(u, v) < 0.0) {std::reverse(q.begin(), q.end());}
  return q;
}

inline Hazard circleHazard(HazardSource src, int id, Vec2 c, double r)
{
  Hazard h;
  h.kind = HazardKind::Circle;
  h.source = src;
  h.id = id;
  h.c = c;
  h.r = r;
  return h;
}

inline Hazard polygonHazard(HazardSource src, int id, std::vector<Vec2> poly)
{
  Hazard h;
  h.kind = HazardKind::Polygon;
  h.source = src;
  h.id = id;
  h.poly = std::move(poly);
  return h;
}

/// The point at arc length s along the path (cum = cumulative arc per vertex), and
/// the index of the first vertex at or beyond it.
inline Vec2 pointAtArc(
  const std::vector<Vec2> & path, const std::vector<double> & cum, double s, std::size_t * idx)
{
  const std::size_t i = static_cast<std::size_t>(
    std::lower_bound(cum.begin(), cum.end(), s) - cum.begin());
  if (i >= path.size()) {
    *idx = path.size() - 1;
    return path.back();
  }
  *idx = i;
  if (i == 0) {return path[0];}
  const double seg = cum[i] - cum[i - 1];     // > 0: cum[i-1] < s <= cum[i]
  return path[i - 1] + (path[i] - path[i - 1]) * ((s - cum[i - 1]) / seg);
}

}  // namespace detail

// ---------------------------------------------------------------- clearance

/// Signed distance from p to the hazard's LETHAL boundary (surface + keepout); < 0 inside.
inline double clearance(const Hazard & h, Vec2 p)
{
  const double d = h.kind == HazardKind::Circle ?
    nav::norm(p - h.c) - h.r : detail::polygonSignedDistance(h.poly, p);
  return d - h.keepout;
}

/// +inf when empty: no hazards means nothing is near.
inline double minClearance(const std::vector<Hazard> & hz, Vec2 p)
{
  double best = detail::kInf;
  for (const Hazard & h : hz) {
    const double c = clearance(h, p);
    if (c < best) {best = c;}
  }
  return best;
}

// ------------------------------------------------------------------ the dock

/// A bay track whose outward direction was actually measured. outward() falls
/// back to unit(view_sum), and unit() of nothing is NORTH (nav_math.hpp:75-79):
/// a dock drawn facing north because nobody knew is a guess wearing a
/// measurement's clothes, so such a track draws nothing.
inline bool outwardUsable(const dock::BayTrack & t)
{
  return (t.n_normal > 0 && nav::norm(t.normal_sum) > 1e-6) || nav::norm(t.view_sum) > 1e-6;
}

/// Per DockBook track with n >= min_obs and a usable outward(): two finger rectangles
/// (lateral +/-(slip/2 + finger_w/2) along right = starboardOf(-out), 0..finger_len out from
/// the face) and one deck rectangle (-deck_depth..0 along out, lateral +/-(slip/2 + finger_w)).
///
/// `right` is dock::layout()'s own (dock_math.hpp:615), so the fingers sit where the
/// numbered bays do. Adjacent bays share a finger and both draw it: the layer
/// rasterises a duplicate for free, and deduplicating would need an association
/// rule that can only be wrong.
inline std::vector<Hazard> dockHazards(const dock::DockBook & b, int min_obs, const NavParams & p)
{
  std::vector<Hazard> out;
  const double lat = 0.5 * p.dock_slip_w_m + 0.5 * p.dock_finger_w_m;   // finger centre
  const double half = 0.5 * p.dock_finger_w_m;
  const double deck_half = 0.5 * p.dock_slip_w_m + p.dock_finger_w_m;
  for (const dock::BayTrack & t : b.tracks) {
    if (t.n < min_obs || !detail::finite(t.p) || !outwardUsable(t)) {continue;}
    const Vec2 out_v = t.outward();
    const Vec2 right = nav::starboardOf(out_v * -1.0);
    // lateral u0..u1 along `right`, v0..v1 along `out`, both from the face centre
    const auto add = [&](double u0, double u1, double v0, double v1) {
        out.push_back(detail::polygonHazard(
            HazardSource::Dock, t.id, detail::rect(t.p, right, out_v, u0, u1, v0, v1)));
      };
    for (const double s : {-1.0, 1.0}) {add(s * lat - half, s * lat + half, 0.0, p.dock_finger_len_m);}
    add(-deck_half, deck_half, -p.dock_deck_depth_m, 0.0);
  }
  return out;
}

/// Plan buoys (radius buoy_radius_m), unmatched confirmed tracks (track_radius_m), and the
/// dock (dockHazards). Consumed buoys are NOT dropped - a buoy dealt with is still there.
///
/// A hazard with no finite position is left out, not drawn at (0,0): "a hazard
/// nobody can place is left out, not drawn somewhere plausible" (Hazard.msg).
/// This is the ONE function the hazard publisher and the leaves both call
/// (via context.hpp knownHazards), so the costmap and the BT's local checks can
/// never disagree about the known field.
inline std::vector<Hazard> buildHazards(
  const std::vector<nav::Buoy> & buoys, const std::vector<nav::Buoy> & tracks,
  const dock::DockBook & dock_book, int dock_min_obs, const NavParams & p)
{
  std::vector<Hazard> out;
  for (const nav::Buoy & b : buoys) {
    if (detail::finite(b.p)) {
      out.push_back(detail::circleHazard(HazardSource::PlanBuoy, b.id, b.p, p.buoy_radius_m));
    }
  }
  for (const nav::Buoy & t : tracks) {
    if (detail::finite(t.p)) {
      out.push_back(detail::circleHazard(HazardSource::Track, t.id, t.p, p.track_radius_m));
    }
  }
  const std::vector<Hazard> d = dockHazards(dock_book, dock_min_obs, p);
  out.insert(out.end(), d.begin(), d.end());
  return out;
}

/// Removes PlanBuoy hazards whose id is listed, Track hazards within exempt_radius of a
/// listed buoy, and every Dock hazard when exempt_dock.
///
/// WHY TRACKS TOO. The LiDAR sees the exempt buoy itself and, unless the plan's
/// position and the track agree to within the association radius, the track is
/// "unmatched" and becomes a second hazard on top of the one we just exempted -
/// which would block the gate crossing it was exempted for. A track is tied to the
/// buoy by distance to the buoy's PLAN position, taken from the hazard list
/// itself, so a buoy that is not in the list exempts nothing around it.
inline std::vector<Hazard> exempt(
  const std::vector<Hazard> & hz, const std::vector<int> & buoy_ids, bool exempt_dock,
  double exempt_radius_m)
{
  const auto listed = [&buoy_ids](int id) {
      return std::find(buoy_ids.begin(), buoy_ids.end(), id) != buoy_ids.end();
    };
  std::vector<Vec2> centres;
  for (const Hazard & h : hz) {
    if (h.source == HazardSource::PlanBuoy && listed(h.id)) {centres.push_back(h.c);}
  }
  std::vector<Hazard> out;
  for (const Hazard & h : hz) {
    if (h.source == HazardSource::PlanBuoy && listed(h.id)) {continue;}
    if (h.source == HazardSource::Dock && exempt_dock) {continue;}
    if (h.source == HazardSource::Track && h.kind == HazardKind::Circle) {
      const bool near_exempt = std::any_of(
        centres.begin(), centres.end(),
        [&](Vec2 c) {return nav::norm(h.c - c) <= exempt_radius_m;});
      if (near_exempt) {continue;}
    }
    out.push_back(h);
  }
  return out;
}

// ------------------------------------------------------------ segments, paths

/// True when every point of a->b (sampled each step_m, both ends included) is at
/// least clearance_m from every hazard. A non-finite end is NOT clear: NaN
/// compares false against everything, so without the guard it would pass.
inline bool segmentClear(
  Vec2 a, Vec2 b, const std::vector<Hazard> & hz, double clearance_m, double step_m = 0.1)
{
  if (!detail::finite(a) || !detail::finite(b)) {return false;}
  return detail::walkSegment(
    a, b, step_m, [&](Vec2 q) {return !(minClearance(hz, q) < clearance_m);});
}

/// First index >= from_i whose point is within clearance_m of any hazard; -1 = none.
inline int firstConflict(
  const std::vector<Vec2> & path, std::size_t from_i, const std::vector<Hazard> & hz,
  double clearance_m)
{
  for (std::size_t i = from_i; i < path.size(); ++i) {
    if (minClearance(hz, path[i]) < clearance_m) {return static_cast<int>(i);}
  }
  return -1;
}

inline double pathLength(const std::vector<Vec2> & path, std::size_t from_i = 0)
{
  double len = 0.0;
  for (std::size_t i = from_i + 1; i < path.size(); ++i) {len += nav::norm(path[i] - path[i - 1]);}
  return len;
}

/// Closest vertex, searching forward from `hint` (never backwards by more than 2 m of arc).
///
/// Forward-only on purpose: a path that doubles back near itself must not make the
/// boat "jump" to the later pass and skip the stretch in between. The 2 m of slack
/// lets a boat that was pushed back by the sea re-find itself without that jump.
inline std::size_t closestIndex(const std::vector<Vec2> & path, Vec2 p, std::size_t hint)
{
  if (path.empty()) {return 0;}
  std::size_t lo = std::min(hint, path.size() - 1);
  double back = 0.0;
  while (lo > 0) {
    const double seg = nav::norm(path[lo] - path[lo - 1]);
    if (back + seg > 2.0) {break;}
    back += seg;
    --lo;
  }
  std::size_t best = lo;
  double bd = nav::norm(path[lo] - p);
  for (std::size_t i = lo + 1; i < path.size(); ++i) {
    const double d = nav::norm(path[i] - p);
    if (d < bd) {bd = d; best = i;}
  }
  return best;
}

namespace detail
{

/// Where the boat sits on the path: the foot of its projection on segment
/// [seg, seg+1], at fraction t. Taken from the two segments around the closest
/// VERTEX, which is enough for a path whose vertices are dense (Smac, 0.1 m) and is
/// the only thing that works for a sparse one (the straight stub is two vertices,
/// and "the closest vertex" of a boat half-way along it is a coin toss).
struct Foot
{
  std::size_t seg = 0;
  double t = 0.0;
  Vec2 p;
};

inline Foot projectOnPath(const std::vector<Vec2> & path, Vec2 boat, std::size_t hint)
{
  Foot f;
  if (path.empty()) {f.p = boat; return f;}
  f.p = path[0];
  if (path.size() == 1) {return f;}
  const std::size_t ci = closestIndex(path, boat, hint);
  double best = detail::kInf;
  for (const std::size_t seg : {ci > 0 ? ci - 1 : ci, ci}) {
    if (seg + 1 >= path.size()) {continue;}
    const Vec2 a = path[seg], ab = path[seg + 1] - a;
    const double l2 = nav::dot(ab, ab);
    const double t = l2 < 1e-18 ? 0.0 : std::clamp(nav::dot(boat - a, ab) / l2, 0.0, 1.0);
    const Vec2 q = a + ab * t;
    const double d = nav::norm(boat - q);
    if (d < best) {best = d; f.seg = seg; f.t = t; f.p = q;}
  }
  return f;
}

/// The path still ahead of the boat: its foot, then every later vertex.
inline std::vector<Vec2> remainingPath(const std::vector<Vec2> & path, Vec2 boat, std::size_t hint)
{
  if (path.size() < 2) {return path;}
  const Foot f = projectOnPath(path, boat, hint);
  std::vector<Vec2> out;
  if (f.t < 1.0 - 1e-9) {out.push_back(f.p);}
  out.insert(out.end(), path.begin() + static_cast<std::ptrdiff_t>(f.seg) + 1, path.end());
  return out;
}

/// firstConflict() for a path whose vertices may be far apart: also samples every
/// segment longer than 0.25 m. Smac's vertices are 0.1 m apart so for them this IS
/// firstConflict; the straight stub's two vertices are not, and a vertex-only check
/// would wave a line straight through the dock.
inline bool pathBlocked(
  const std::vector<Vec2> & path, const std::vector<Hazard> & hz, double clearance_m)
{
  if (firstConflict(path, 0, hz, clearance_m) >= 0) {return true;}
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (nav::norm(path[i] - path[i - 1]) > 0.25 &&
      !segmentClear(path[i - 1], path[i], hz, clearance_m))
    {
      return true;
    }
  }
  return false;
}

}  // namespace detail

// ----------------------------------------------------------------- the goal

struct Moved
{
  Vec2 p;
  bool moved = false;
  bool ok = true;
};

/// If goal is within hard + goal_margin of any hazard, walk it toward `from` in 0.1 m steps
/// until clear; ok=false if the whole segment is blocked (goal returned unchanged).
///
/// WHY. SmacPlanner2D refuses a goal in lethal space and a goal a waypoint sits on
/// top of is exactly what CircleBuoy and a buoy-side waypoint produce. The leg
/// still judges ARRIVAL on the true goal (planned_leg.hpp), so pushing the
/// planner's goal out costs nothing but a metre of standoff.
inline Moved pushGoalOut(
  Vec2 goal, Vec2 from, const std::vector<Hazard> & hz, double hard, double margin)
{
  Moved m;
  m.p = goal;
  const double need = hard + margin;
  if (!(minClearance(hz, goal) < need)) {return m;}
  const double len = nav::norm(from - goal);
  const Vec2 dir = nav::unit(from - goal);
  const int steps = static_cast<int>(std::floor(len / 0.1));
  for (int k = 1; k <= steps + 1; ++k) {
    const Vec2 q = k <= steps ? goal + dir * (0.1 * k) : from;     // the last try is `from` itself
    if (!(minClearance(hz, q) < need)) {
      m.p = q;
      m.moved = true;
      return m;
    }
  }
  m.ok = false;
  return m;
}

/// Smac does not bounds-check its goal, which is undefined behaviour off the
/// costmap (spec 3.4), so no request may name a point outside the rolling window.
/// clip_r <= 0 disables the clip rather than collapsing the goal onto the boat.
inline Vec2 clipToWindow(Vec2 goal, Vec2 from, double clip_r)
{
  const double d = nav::norm(goal - from);
  if (!(clip_r > 0.0) || d <= clip_r) {return goal;}
  return from + nav::unit(goal - from) * clip_r;
}

struct Escape
{
  bool needed = false;
  bool ok = false;
  Vec2 p;       ///< meaningful only when ok
};

/// needed when minClearance(boat) < hard. Candidates: radii wp_radius + {0.5, 1.0, 2.0},
/// 16 bearings. Valid: clearance >= hard + escape_margin AND along boat->cand (0.1 m steps)
/// no hazard gets closer than min(its clearance at boat, hard) - 0.05. Pick min
/// |cand - goal| + 0.5 |cand - boat|. ok=false if none.
///
/// WHY. Smac THROWS when its start is inside the inscribed zone (spec 3.4), so a
/// boat that has drifted into the hard band cannot plan at all. This finds a point
/// to drive to FIRST. The radii start at wp_radius + 0.5 so ArduRover (WP_RADIUS
/// 2.0) does not count the escape point as reached the moment it is sent. The
/// per-hazard floor stops the straight run out from brushing a SECOND hazard (a boat
/// in a gate escapes along the gate axis, not through a buoy), while a hazard the
/// boat is already inside the band of may only get no closer than it is now.
inline Escape escapeStart(
  Vec2 boat, Vec2 goal, const std::vector<Hazard> & hz, const NavParams & p)
{
  Escape e;
  if (!(minClearance(hz, boat) < p.hard_m)) {return e;}      // NaN-safe: nothing to escape
  e.needed = true;
  std::vector<double> floor_at;
  for (const Hazard & h : hz) {floor_at.push_back(std::min(clearance(h, boat), p.hard_m) - 0.05);}
  double best = detail::kInf;
  for (const double extra : {0.5, 1.0, 2.0}) {
    const double r = p.wp_radius_m + extra;
    for (int k = 0; k < 16; ++k) {
      const double a = 2.0 * nav::kPi * k / 16.0;
      const Vec2 cand = boat + Vec2{std::cos(a), std::sin(a)} * r;
      if (!(minClearance(hz, cand) >= p.hard_m + p.escape_margin_m)) {continue;}
      const bool non_approaching = detail::walkSegment(
        boat, cand, 0.1, [&](Vec2 q) {
          for (std::size_t i = 0; i < hz.size(); ++i) {
            if (clearance(hz[i], q) < floor_at[i]) {return false;}
          }
          return true;
        });
      if (!non_approaching) {continue;}
      const double cost = nav::norm(cand - goal) + 0.5 * nav::norm(cand - boat);
      if (cost < best) {best = cost; e.ok = true; e.p = cand;}
    }
  }
  return e;
}

// -------------------------------------------------------------------- carrot

struct Carrot
{
  Vec2 p;
  std::size_t idx = 0;      ///< first path vertex at or beyond p
  bool is_end = false;      ///< p is the end of the path
};

/// Farthest point within [lookahead_min, lookahead] of arc from the boat's projection
/// such that every path vertex between is within max_chord_dev of the chord AND the chord
/// boat->p is segmentClear at hard - local_check_tol. Falls back to lookahead_min. The end
/// of the path when less than lookahead_min remains (is_end = true).
///
/// WHY A CARROT. ArduRover flies a straight line to each position setpoint, so the
/// whole path cannot be sent: the boat would cut every corner the planner took
/// around a buoy. A point a few metres ahead on the path is far enough to be
/// smooth and near enough that its chord stays on the path. The chord test is what
/// makes "a few metres" shorter around a bend.
///
/// is_end is also true when the chosen point IS the path's last vertex (remaining
/// arc between lookahead_min and lookahead): the leg must not keep re-sending the
/// end of the path as the boat closes on it.
///
/// An empty path returns a carrot ON THE BOAT (a hold), never the origin.
inline Carrot carrot(
  const std::vector<Vec2> & path, Vec2 boat, std::size_t hint, const std::vector<Hazard> & hz,
  const NavParams & p)
{
  Carrot out;
  const std::size_t n = path.size();
  if (n == 0) {out.p = boat; out.is_end = true; return out;}
  if (n == 1) {out.p = path[0]; out.is_end = true; return out;}

  std::vector<double> cum(n, 0.0);
  for (std::size_t i = 1; i < n; ++i) {cum[i] = cum[i - 1] + nav::norm(path[i] - path[i - 1]);}
  const double total = cum[n - 1];

  // The boat's projection onto the path, as an arc position.
  const detail::Foot foot = detail::projectOnPath(path, boat, hint);
  const double s0 = cum[foot.seg] + foot.t * (cum[foot.seg + 1] - cum[foot.seg]);

  const auto end_carrot = [&]() {
      out.p = path.back();
      out.idx = n - 1;
      out.is_end = true;
      return out;
    };
  if (total - s0 < p.lookahead_min_m) {return end_carrot();}

  const double s_lo = s0 + p.lookahead_min_m;
  const double s_hi = std::max(std::min(s0 + p.lookahead_m, total), s_lo);
  const double tol = p.hard_m - p.local_check_tol_m;
  const double step = 0.1;
  const int tries = static_cast<int>(std::ceil((s_hi - s_lo) / step));
  for (int k = 0; k <= tries; ++k) {
    const double s = std::max(s_hi - k * step, s_lo);
    std::size_t idx = 0;
    const Vec2 cand = detail::pointAtArc(path, cum, s, &idx);
    bool follows = true;
    for (std::size_t i = 0; i < n && follows; ++i) {
      if (cum[i] > s0 + 1e-9 && cum[i] < s - 1e-9) {
        follows = detail::distToSegment(path[i], boat, cand) <= p.max_chord_dev_m;
      }
    }
    if (follows && segmentClear(boat, cand, hz, tol)) {
      if (s >= total - 1e-9) {return end_carrot();}
      out.p = cand;
      out.idx = idx;
      return out;
    }
  }
  // Nothing satisfied the chord tests: the path itself is valid, so take it at the minimum.
  if (s_lo >= total - 1e-9) {return end_carrot();}
  out.p = detail::pointAtArc(path, cum, s_lo, &out.idx);
  return out;
}

/// Switch to the new path only if the current one is invalid, or the new one is shorter by
/// BOTH >= frac AND >= abs_m. Lengths are path length + |path end - true goal|.
///
/// Hysteresis, because two near-equal routes round a buoy differ by a hair every
/// 0.5 s replan and a boat that follows whichever is momentarily shorter zig-zags.
/// "Both" so a long path needs a worthwhile saving in metres and a short one a
/// worthwhile saving in proportion.
inline bool preferNew(bool cur_valid, double cur_len, double new_len, double frac, double abs_m)
{
  if (!cur_valid) {return true;}
  const double saved = cur_len - new_len;
  return saved >= frac * cur_len && saved >= abs_m;
}

// --------------------------------------------------------------------- orbit

/// ring[0] = on the anchor->from bearing at `radius` (the explicit hop target),
/// ring[k] = a0 + k*step, step = 360/n, ring[n] == ring[0] (the 360 point).
/// cw = decreasing ENU angle (as nav::orbit).
///
/// nav::orbit starts at k = 1 from the boat's own bearing, so its first waypoint is
/// already a step round the circle and a boat that starts far out sweeps only
/// ~313 degrees (task1-panel-build memory). Here the ring starts ON the bearing
/// at the orbit radius: hop 0 is the approach, and ring[0..n] is a full turn from
/// wherever the boat began. ring[n] is a copy of ring[0], not a recomputed
/// cos(a0 + 2 pi), so that hop is EXACTLY the first.
///
/// `overshoot_deg` runs the ring PAST that full turn, the same way round: n + 1
/// points up to a sweep of exactly 360 + overshoot_deg, the last step shorter when the
/// overshoot is not a whole number of steps. Why: a hop counts as arrived within the
/// leg tolerance (2 m at a 6 m ring is ~19 degrees), so a ring that ends at exactly 360
/// closes the circle ~19 degrees short, and an approach that enters off ring[0] loses
/// more; the referee's 330 degrees was missed at 328 (task1_blocked_exit, 2026-10-01).
/// 0 gives the n + 1 point ring above, bit for bit. Clamped to [0, 360]: a negative or
/// NaN value is no overshoot, and the ring never goes more than one extra lap.
inline std::vector<Vec2> orbitRing(
  Vec2 anchor, Vec2 from, double radius, int n, bool cw, double overshoot_deg = 0.0)
{
  std::vector<Vec2> ring;
  if (n < 1) {return ring;}
  const double over_deg = std::min(std::max(0.0, overshoot_deg), 360.0);   // NaN -> 0
  const double step_deg = 360.0 / static_cast<double>(n);
  const double total_deg = 360.0 + over_deg;
  const int whole = static_cast<int>(std::floor(total_deg / step_deg + 1e-9));
  const Vec2 d = from - anchor;
  const double a0 = nav::norm(d) < 1e-6 ? 0.0 : std::atan2(d.y, d.x);
  const double step = (cw ? -1.0 : 1.0) * 2.0 * nav::kPi / static_cast<double>(n);
  const auto at = [&](double a) {return anchor + Vec2{std::cos(a), std::sin(a)} * radius;};
  ring.reserve(static_cast<std::size_t>(whole) + 2);
  for (int k = 0; k < whole + 1; ++k) {
    ring.push_back(k == n ? ring.front() : at(a0 + step * k));
  }
  if (total_deg - whole * step_deg > 1e-6) {
    ring.push_back(at(a0 + (cw ? -1.0 : 1.0) * total_deg * nav::kDeg));      // the odd last step
  }
  return ring;
}

/// Each ring point with minClearance < orbit_clear_m: push radially out in 0.25 m steps
/// up to orbit_max_push_m; still short -> drop it (logged by the caller).
///
/// ring[0] is never touched: it is the explicit hop the leg promised. Neither is a copy
/// of it (the 360 point, which closes the circle on it): the orbit is meant to end
/// where it began. EVERY OTHER point is fair game, including the last one when the ring
/// overshoots (orbitRing's overshoot_deg): that one is just another point round the
/// circle. `dropped` counts the points given up on so the caller can say so rather than
/// silently orbiting a polygon with a side missing.
inline std::vector<Vec2> adjustRing(
  const std::vector<Vec2> & ring, Vec2 anchor, const std::vector<Hazard> & hz,
  const NavParams & p, int * dropped = nullptr)
{
  if (dropped != nullptr) {*dropped = 0;}
  std::vector<Vec2> out;
  const int pushes = static_cast<int>(std::floor(p.orbit_max_push_m / 0.25 + 1e-9));
  for (std::size_t i = 0; i < ring.size(); ++i) {
    const bool is_start = i == 0 || (ring[i].x == ring[0].x && ring[i].y == ring[0].y);
    if (is_start || !(minClearance(hz, ring[i]) < p.orbit_clear_m)) {
      out.push_back(ring[i]);
      continue;
    }
    const Vec2 dir = nav::unit(ring[i] - anchor);
    bool fixed = false;
    for (int s = 1; s <= pushes && !fixed; ++s) {
      const Vec2 q = ring[i] + dir * (0.25 * s);
      if (!(minClearance(hz, q) < p.orbit_clear_m)) {out.push_back(q); fixed = true;}
    }
    if (!fixed && dropped != nullptr) {++*dropped;}
  }
  return out;
}

}  // namespace path
}  // namespace crusader_bt

#endif  // CRUSADER_BT__PATH_MATH_HPP_
