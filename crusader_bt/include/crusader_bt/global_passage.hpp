// global_passage.hpp — Task 1 planned end to end, over the whole known field, in one search.
//
// Header-only, stdlib + nav_math.hpp + path_math.hpp. No ROS, no Nav2, no BehaviorTree.CPP.
// Same contract as path_math.hpp, and for the same reason: a sign error here MIRRORS THE
// COURSE, and this is the part that compiles and runs on a laptop in a second:
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include test/test_global_passage.cpp
//
// WHY THIS EXISTS. Above Core tier the aircraft hands the boat EVERY buoy, its colour and the
// ENTRY and EXIT before the boat moves. The per-gate tree (task1_disruptive.xml) still drives
// the field one gate at a time: an approach point, a straight crossing, side fences for the
// planner, an orbit ring with fixed radius. In a field with 3-5 m between buoys those pieces
// land on top of each other - the approach point sits beside a black buoy, a fence closes the
// next gap, the 6 m orbit ring runs through the first gate - and no leg finds a path. Here the
// whole passage is ONE problem: from the boat, round the ENTRY clockwise, through the field
// with every red to starboard and every green to port, round the EXIT counter-clockwise.
//
// THE ONE IDEA: A SIDE IS TOPOLOGY, NOT GEOMETRY. Every red buoy casts a RAY to starboard of the
// entry -> exit axis, every green a ray to port. Close the transit with a ray running back
// from where it starts and one running on from where it ends, both along the axis: the closed
// curve splits the plane in two, and a red is on the boat's starboard hand exactly when the
// curve crosses its ray an EVEN number of times (zero for the plain case of driving past it).
// So the search is A* over (grid cell, parity of crossings per ray), and a goal is reached
// only with every parity even. That is:
//
//   * COMPLETE. If any path keeps every side and the clearance, it is found: a passage that
//     winds back across a fence is just a path that crosses that ray twice.
//   * OPTIMAL on the grid for length plus a clearance cost, which is what centres the boat in
//     a 3 m gap instead of shaving the nearer buoy.
//   * UNIFORM. Paired gates, single reds and greens, black buoys and LiDAR tracks are all the
//     same thing to it: obstacles, some of which carry a parity bit. Nothing pairs buoys,
//     nothing places gate points, nothing draws fences.
//
// Rays are perpendicular to the axis, and a red or green buoy is held to its side only when it
// lies (along the axis) between the far edge of the ENTRY orbit and the near edge of the EXIT
// orbit: such a ray can never meet the two closing rays, so the transit's start and goal
// parities are all zero. A red or green beside or behind an orbit is passed DURING the orbit,
// has no sensible side, and is kept clear of as a plain obstacle (the plan says so in `notes`).
//
// THE ORBITS are circles round the buoy's own fused position, as large as nav_orbit_radius_m
// allows and shrunk until every other hazard keeps orbit_clear_m of water; a red, green or
// blue buoy is never inside one. The point where the ENTRY orbit is left and the EXIT orbit is
// joined are chosen BY THE SEARCH (the transit starts from every point of the entry ring, each
// charged the extra orbit it costs, and ends on any point of the exit ring), so the orbit and
// the transit are one optimisation, not two guesses.
//
// NOTHING HERE DRIVES. plan() returns four legs (approach, entry orbit, transit, exit orbit) as
// dense polylines, plus the checkpoints the Disruptive tree stops at. path_follower.hpp drives
// a leg; src/global_leaves.cpp is the glue.
#ifndef CRUSADER_BT__GLOBAL_PASSAGE_HPP_
#define CRUSADER_BT__GLOBAL_PASSAGE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "crusader_bt/nav_math.hpp"
#include "crusader_bt/path_math.hpp"

namespace crusader_bt
{
namespace gp
{

using nav::Vec2;
using path::Hazard;

/// Where the mission is. The plan for a phase always starts at the boat.
enum class Phase {Approach = 0, EntryOrbit = 1, Transit = 2, ExitOrbit = 3, Done = 4};

inline const char * phaseName(Phase p)
{
  switch (p) {
    case Phase::Approach: return "approach";
    case Phase::EntryOrbit: return "entry_orbit";
    case Phase::Transit: return "transit";
    case Phase::ExitOrbit: return "exit_orbit";
    case Phase::Done: return "done";
  }
  return "done";
}

/// "approach" | "entry_orbit" | "transit" | "exit_orbit". False for anything else.
inline bool parsePhase(const std::string & s, Phase & out)
{
  for (const Phase p : {Phase::Approach, Phase::EntryOrbit, Phase::Transit, Phase::ExitOrbit}) {
    if (s == phaseName(p)) {out = p; return true;}
  }
  return false;
}

struct Params
{
  // Clearance is the boat CENTRE's distance to a hazard's edge (path::clearance).
  double hard_m = 0.8;          ///< never planned closer (nav_hard_m)
  double soft_m = 2.0;          ///< beyond this, more water buys nothing (nav_soft_m)
  double w_clear = 4.0;         ///< a metre driven AT hard_m costs 1 + w_clear metres
  /// Tried in turn, only when nothing fits at hard_m. A plan made with one says relaxed = true.
  double relax1_m = 0.7, relax2_m = 0.6;
  double res_m = 0.1;           ///< grid; coarsened automatically above max_cells
  double margin_m = 10.0;       ///< grid beyond the field, the boat and the trajectory
  int max_cells = 250000;
  long max_expansions = 3000000;
  int max_layers = 24;          ///< parity layers the search may open (memory bound)
  // Orbits.
  double orbit_r_pref_m = 6.0;  ///< the radius wanted (nav_orbit_radius_m)
  double orbit_r_min_m = 1.5;   ///< the smallest normally accepted
  double orbit_clear_m = 1.4;   ///< every OTHER hazard keeps this much water from the ring
  double orbit_step_m = 0.1;    ///< ring vertex spacing
  double overshoot_deg = 45.0;  ///< past the full turn, the same way round (at least)
  /// The follower ends a pass-through leg this far early (its lookahead + arrival slack):
  /// the overshoot is grown so the sweep still clears 360 + 20 degrees.
  double follow_early_m = 2.5;
  double along_margin_m = 0.5;  ///< an orbit stays this far short (along the axis) of the passage
  double anchor_snap_m = 3.0;   ///< a blue buoy this close to the aircraft's ENTRY/EXIT anchors the orbit
  double buoy_radius_m = 0.3;   ///< when the anchor has no hazard of its own
  // Checkpoints (Disruptive): past each gate the path crosses.
  double cp_after_m = 1.5;      ///< at least this far past the gate line
  double cp_window_m = 3.0;     ///< ... and within this much more, at the most open water
  double path_step_m = 0.1;     ///< output vertex spacing
  /// A re-reported buoy (or ENTRY/EXIT) that moved less than this is the aircraft's own
  /// scatter, not a re-task (fieldChange). Its fix error is < 1 m, so two reports differ by
  /// up to ~1.6 m.
  double retask_move_m = 2.0;
};

// --------------------------------------------------------------------- rays

/// The side rule for one red or green buoy: a ray from it, perpendicular to the axis,
/// on the side the boat must NOT pass (red: starboard of the axis, green: port).
struct Ray
{
  int id = -1;
  Vec2 o;
  Vec2 d;
  bool red = false;
};

/// The closing rays are this long: far beyond any field, short enough to stay exact.
constexpr double kFar = 1.0e4;
constexpr int kMaxRays = 31;

/// Does segment p->q cross the ray? Half-open by side (a point exactly on the ray's line
/// counts as left of it), so a polyline crossing the line is counted exactly once and a
/// polyline that touches it and turns back is counted zero or two times, never one.
inline bool segmentCrossesRay(const Ray & r, Vec2 p, Vec2 q)
{
  const double sp = nav::cross(r.d, p - r.o);
  const double sq = nav::cross(r.d, q - r.o);
  if ((sp >= 0.0) == (sq >= 0.0)) {return false;}
  const Vec2 x = p + (q - p) * (sp / (sp - sq));
  return nav::dot(x - r.o, r.d) >= 0.0;
}

inline std::uint32_t segmentMask(const std::vector<Ray> & rays, Vec2 p, Vec2 q)
{
  std::uint32_t m = 0;
  for (std::size_t k = 0; k < rays.size(); ++k) {
    if (segmentCrossesRay(rays[k], p, q)) {m ^= (1u << k);}
  }
  return m;
}

inline std::uint32_t polylineMask(const std::vector<Ray> & rays, const std::vector<Vec2> & pl)
{
  std::uint32_t m = 0;
  for (std::size_t i = 1; i < pl.size(); ++i) {m ^= segmentMask(rays, pl[i - 1], pl[i]);}
  return m;
}

/// The ray that closes the transit at its START: from far back along the axis up to p.
inline std::uint32_t startClosure(const std::vector<Ray> & rays, Vec2 p, Vec2 axis)
{
  return segmentMask(rays, p - axis * kFar, p);
}

/// ... and at its END: from p on along the axis.
inline std::uint32_t endClosure(const std::vector<Ray> & rays, Vec2 p, Vec2 axis)
{
  return segmentMask(rays, p, p + axis * kFar);
}

/// The side ray of a red or green buoy (any other colour: id -1).
inline Ray rayOf(const nav::Buoy & b, Vec2 axis)
{
  Ray r;
  const bool red = b.state == nav::Beacon::FlashingRed;
  if ((!red && b.state != nav::Beacon::FlashingGreen) || !std::isfinite(b.p.x) ||
    !std::isfinite(b.p.y))
  {
    return r;
  }
  r.id = b.id;
  r.o = b.p;
  r.red = red;
  r.d = red ? nav::starboardOf(axis) : nav::portOf(axis);
  return r;
}

/// A ray per red and green buoy that `why_not` (returning "" to keep) does not exclude; the
/// rest go to `excluded`, with the reason in `notes`. At most kMaxRays.
template<typename F>
std::vector<Ray> buildRays(
  const std::vector<nav::Buoy> & buoys, Vec2 axis, F why_not, std::vector<int> * excluded,
  std::vector<std::string> * notes)
{
  std::vector<Ray> rays;
  for (const nav::Buoy & b : buoys) {
    const Ray r = rayOf(b, axis);
    if (r.id < 0 && !nav::isSideConstrained(b.state)) {continue;}
    std::string why = r.id < 0 ? std::string("no position") : why_not(r);
    if (why.empty() && static_cast<int>(rays.size()) >= kMaxRays) {why = "too many buoys";}
    if (!why.empty()) {
      if (excluded != nullptr) {excluded->push_back(b.id);}
      if (notes != nullptr) {
        notes->push_back(std::string(r.red ? "RED " : "GREEN ") + std::to_string(b.id) + " " + why +
          ": kept clear of, side not held");
      }
      continue;
    }
    rays.push_back(r);
  }
  return rays;
}

// --------------------------------------------------------------------- grid

struct Grid
{
  double x0 = 0.0, y0 = 0.0, res = 0.1;
  int nx = 0, ny = 0;
  std::vector<float> clear;     ///< per cell: minClearance of its centre, capped at 1e6

  int size() const {return nx * ny;}
  Vec2 centre(int c) const
  {
    return {x0 + ((c % nx) + 0.5) * res, y0 + ((c / nx) + 0.5) * res};
  }
  int cellOf(Vec2 p) const
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return -1;}
    const double fi = std::floor((p.x - x0) / res), fj = std::floor((p.y - y0) / res);
    if (fi < 0.0 || fj < 0.0 || fi >= nx || fj >= ny) {return -1;}
    return static_cast<int>(fj) * nx + static_cast<int>(fi);
  }
};

/// A grid over every point in `pts` plus margin, with the clearance of every cell centre.
inline Grid makeGrid(const std::vector<Vec2> & pts, const std::vector<Hazard> & hz, const Params & p)
{
  Grid g;
  double xmin = std::numeric_limits<double>::infinity(), ymin = xmin;
  double xmax = -xmin, ymax = -xmin;
  for (const Vec2 & q : pts) {
    if (!std::isfinite(q.x) || !std::isfinite(q.y)) {continue;}
    xmin = std::min(xmin, q.x);
    xmax = std::max(xmax, q.x);
    ymin = std::min(ymin, q.y);
    ymax = std::max(ymax, q.y);
  }
  if (!(xmax >= xmin)) {return g;}
  xmin -= p.margin_m;
  ymin -= p.margin_m;
  xmax += p.margin_m;
  ymax += p.margin_m;
  double res = p.res_m > 1e-3 ? p.res_m : 0.1;
  const double w = xmax - xmin, h = ymax - ymin;
  while ((w / res) * (h / res) > std::max(1000, p.max_cells)) {res *= 1.25;}
  g.res = res;
  g.x0 = xmin;
  g.y0 = ymin;
  g.nx = static_cast<int>(std::ceil(w / res));
  g.ny = static_cast<int>(std::ceil(h / res));
  g.clear.assign(static_cast<std::size_t>(g.size()), 1e6f);
  for (int c = 0; c < g.size(); ++c) {
    const double d = path::minClearance(hz, g.centre(c));
    g.clear[static_cast<std::size_t>(c)] = static_cast<float>(std::min(d, 1e6));
  }
  return g;
}

// ------------------------------------------------------------------- search

struct Source
{
  Vec2 p;
  double g0 = 0.0;              ///< what reaching p already cost (the orbit it implies)
  std::uint32_t mask = 0;       ///< parity at p, closure included
};

struct GoalPt
{
  Vec2 p;
  std::uint32_t mask = 0;       ///< the closure from p on: the path's parity must equal it
};

struct Found
{
  bool ok = false;
  std::vector<Vec2> pts;        ///< source point, cell centres, goal point
  int src = -1, goal = -1;      ///< indices into the source and goal lists
  double cost = 0.0;
  long expanded = 0;
  int layers = 0;
  std::string why;
};

namespace detail
{

inline std::string fmt(double v, int digits = 1)
{
  char b[32];
  std::snprintf(b, sizeof(b), "%.*f", digits, v);
  return b;
}

inline double penalty(double clear, const Params & p)
{
  if (!(clear < p.soft_m)) {return 0.0;}
  const double span = std::max(0.2, p.soft_m - p.hard_m);
  const double x = (p.soft_m - std::max(0.0, clear)) / span;
  return x * x;
}

}  // namespace detail

/// A* over (cell, parity mask). `hard` is the clearance below which a cell is lethal (a
/// source cell may be lethal: the boat then only moves to cells with at least as much water,
/// which is how it climbs out). Diagonal moves may not cut a lethal corner. The heuristic is
/// the straight distance to a circle (h_c, h_r) on which every goal lies: admissible, because
/// every move costs at least its length.
inline Found search(
  const Grid & g, const std::vector<Ray> & rays, const std::vector<Source> & srcs,
  const std::vector<GoalPt> & goals, Vec2 h_c, double h_r, double hard, const Params & p)
{
  Found out;
  const int n = g.size();
  if (n <= 0) {out.why = "empty grid"; return out;}

  // goal cell -> (required mask AT THE CELL CENTRE, goal index)
  std::unordered_map<int, std::vector<std::pair<std::uint32_t, int>>> goal_at;
  for (std::size_t k = 0; k < goals.size(); ++k) {
    const int c = g.cellOf(goals[k].p);
    if (c < 0) {continue;}
    const std::uint32_t need = goals[k].mask ^ segmentMask(rays, g.centre(c), goals[k].p);
    goal_at[c].push_back({need, static_cast<int>(k)});
  }
  if (goal_at.empty()) {out.why = "no goal on the grid"; return out;}

  std::unordered_map<std::uint32_t, int> layer_of;
  std::vector<std::uint32_t> layer_mask;
  std::vector<std::vector<float>> G;
  std::vector<std::vector<std::int32_t>> P;
  std::vector<std::vector<std::uint8_t>> closed;
  const auto layer = [&](std::uint32_t m) -> int {
      const auto it = layer_of.find(m);
      if (it != layer_of.end()) {return it->second;}
      if (static_cast<int>(G.size()) >= std::max(1, p.max_layers)) {return -1;}
      const int id = static_cast<int>(G.size());
      layer_of[m] = id;
      layer_mask.push_back(m);
      G.emplace_back(static_cast<std::size_t>(n), std::numeric_limits<float>::infinity());
      P.emplace_back(static_cast<std::size_t>(n), -1);
      closed.emplace_back(static_cast<std::size_t>(n), 0);
      return id;
    };
  const auto lethal = [&](int c) {return g.clear[static_cast<std::size_t>(c)] < hard;};
  const auto h = [&](int c) {
      return std::max(0.0, nav::norm(g.centre(c) - h_c) - h_r - g.res);
    };

  using Item = std::pair<double, std::int64_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  std::unordered_map<std::int64_t, int> src_of;     // node -> source index (parentless nodes)

  for (std::size_t k = 0; k < srcs.size(); ++k) {
    const int c = g.cellOf(srcs[k].p);
    if (c < 0) {continue;}
    const std::uint32_t m = srcs[k].mask ^ segmentMask(rays, srcs[k].p, g.centre(c));
    const int L = layer(m);
    if (L < 0) {continue;}
    const double g0 = srcs[k].g0 + nav::norm(srcs[k].p - g.centre(c));
    float & gc = G[static_cast<std::size_t>(L)][static_cast<std::size_t>(c)];
    if (g0 < gc) {
      gc = static_cast<float>(g0);
      const std::int64_t node = static_cast<std::int64_t>(L) * n + c;
      src_of[node] = static_cast<int>(k);
      open.push({g0 + h(c), node});
    }
  }
  if (open.empty()) {out.why = "no source on the grid"; return out;}

  static const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  while (!open.empty()) {
    const Item it = open.top();
    open.pop();
    const int L = static_cast<int>(it.second / n);
    const int c = static_cast<int>(it.second % n);
    const std::size_t sl = static_cast<std::size_t>(L), sc = static_cast<std::size_t>(c);
    if (closed[sl][sc]) {continue;}
    closed[sl][sc] = 1;
    if (++out.expanded > p.max_expansions) {
      out.why = "search limit reached (" + std::to_string(p.max_expansions) + " expansions)";
      out.layers = static_cast<int>(G.size());
      return out;
    }
    const std::uint32_t m = layer_mask[sl];
    const auto ga = goal_at.find(c);
    if (ga != goal_at.end()) {
      for (const auto & need : ga->second) {
        if (need.first != m) {continue;}
        // walk back
        std::vector<Vec2> rev;
        rev.push_back(goals[static_cast<std::size_t>(need.second)].p);
        std::int64_t node = it.second;
        for (;;) {
          const int nc = static_cast<int>(node % n);
          rev.push_back(g.centre(nc));
          const std::int32_t par =
            P[static_cast<std::size_t>(node / n)][static_cast<std::size_t>(nc)];
          if (par < 0) {break;}
          node = par;
        }
        const int si = src_of.count(node) ? src_of[node] : -1;
        if (si >= 0) {rev.push_back(srcs[static_cast<std::size_t>(si)].p);}
        out.pts.assign(rev.rbegin(), rev.rend());
        out.ok = si >= 0;
        out.src = si;
        out.goal = need.second;
        out.cost = G[sl][sc];
        out.layers = static_cast<int>(G.size());
        if (!out.ok) {out.why = "internal: path without a source";}
        return out;
      }
    }
    const int ci = c % g.nx, cj = c / g.nx;
    const bool here_lethal = lethal(c);
    const Vec2 pc = g.centre(c);
    const double pen_c = detail::penalty(g.clear[sc], p);
    for (int k = 0; k < 8; ++k) {
      const int ni = ci + di[k], nj = cj + dj[k];
      if (ni < 0 || nj < 0 || ni >= g.nx || nj >= g.ny) {continue;}
      const int nc = nj * g.nx + ni;
      if (lethal(nc) && !(here_lethal && g.clear[static_cast<std::size_t>(nc)] >= g.clear[sc])) {
        continue;
      }
      if (k >= 4 && !here_lethal && (lethal(cj * g.nx + ni) || lethal(nj * g.nx + ci))) {continue;}
      const Vec2 pn = g.centre(nc);
      const std::uint32_t nm = m ^ segmentMask(rays, pc, pn);
      const int NL = layer(nm);
      if (NL < 0) {continue;}
      const double len = k >= 4 ? g.res * std::sqrt(2.0) : g.res;
      const double step = len *
        (1.0 + p.w_clear * 0.5 * (pen_c + detail::penalty(g.clear[static_cast<std::size_t>(nc)], p)));
      const double ng = G[sl][sc] + step;
      float & gn = G[static_cast<std::size_t>(NL)][static_cast<std::size_t>(nc)];
      if (ng < gn) {
        gn = static_cast<float>(ng);
        P[static_cast<std::size_t>(NL)][static_cast<std::size_t>(nc)] =
          static_cast<std::int32_t>(it.second);
        open.push({ng + h(nc), static_cast<std::int64_t>(NL) * n + nc});
      }
    }
  }
  out.layers = static_cast<int>(G.size());
  out.why = "no path";
  return out;
}

// ------------------------------------------------------------ polylines

inline std::vector<double> arcLengths(const std::vector<Vec2> & pl)
{
  std::vector<double> cum(pl.size(), 0.0);
  for (std::size_t i = 1; i < pl.size(); ++i) {cum[i] = cum[i - 1] + nav::norm(pl[i] - pl[i - 1]);}
  return cum;
}

/// The point at arc s (clamped to the ends).
inline Vec2 pointAt(const std::vector<Vec2> & pl, const std::vector<double> & cum, double s)
{
  if (pl.empty()) {return {};}
  if (s <= 0.0) {return pl.front();}
  if (s >= cum.back()) {return pl.back();}
  const std::size_t i = static_cast<std::size_t>(
    std::upper_bound(cum.begin(), cum.end(), s) - cum.begin());       // cum[i-1] <= s < cum[i]
  const double seg = cum[i] - cum[i - 1];
  return seg < 1e-12 ? pl[i] : pl[i - 1] + (pl[i] - pl[i - 1]) * ((s - cum[i - 1]) / seg);
}

/// Vertices every <= step metres, every original vertex kept.
inline std::vector<Vec2> densify(const std::vector<Vec2> & pl, double step)
{
  std::vector<Vec2> out;
  if (pl.empty()) {return out;}
  out.push_back(pl.front());
  const double st = step > 1e-3 ? step : 0.1;
  for (std::size_t i = 1; i < pl.size(); ++i) {
    const double len = nav::norm(pl[i] - pl[i - 1]);
    if (len < 1e-9) {continue;}
    const int k = static_cast<int>(std::ceil(len / st));
    for (int j = 1; j <= k; ++j) {out.push_back(pl[i - 1] + (pl[i] - pl[i - 1]) * (double(j) / k));}
  }
  return out;
}

/// The part of `pl` from arc s0 to arc s1 (clamped), with exact end points.
inline std::vector<Vec2> slice(const std::vector<Vec2> & pl, double s0, double s1)
{
  std::vector<Vec2> out;
  if (pl.size() < 2) {return pl;}
  const std::vector<double> cum = arcLengths(pl);
  s0 = std::clamp(s0, 0.0, cum.back());
  s1 = std::clamp(s1, s0, cum.back());
  out.push_back(pointAt(pl, cum, s0));
  for (std::size_t i = 0; i < pl.size(); ++i) {
    if (cum[i] > s0 + 1e-9 && cum[i] < s1 - 1e-9) {out.push_back(pl[i]);}
  }
  const Vec2 e = pointAt(pl, cum, s1);
  if (nav::norm(e - out.back()) > 1e-9 || out.size() == 1) {out.push_back(e);}
  return out;
}

/// Lowest clearance along a polyline, sampled every `step` metres.
inline double polylineClearance(
  const std::vector<Vec2> & pl, const std::vector<Hazard> & hz, double step = 0.05)
{
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < pl.size(); ++i) {
    if (i == 0) {best = std::min(best, path::minClearance(hz, pl[0]));}
    if (i + 1 < pl.size()) {
      path::detail::walkSegment(pl[i], pl[i + 1], step, [&](Vec2 q) {
          best = std::min(best, path::minClearance(hz, q));
          return true;
        });
    }
  }
  return best;
}

/// String pulling that changes nothing that matters: a shortcut i -> j replaces the vertices
/// between only if it crosses every ray the same number of times mod 2 as they did (so no
/// red or green changes side; a black buoy may) and keeps, ALL ALONG, the clearance the
/// replaced stretch had at the same place (within 0.1 m, capped at soft, never below
/// `floor_m`). Locally, not just at the stretch's narrowest point: a shortcut allowed down to
/// a gate's 1.2 m everywhere would shave every buoy it passed on the way to that gate.
inline std::vector<Vec2> smooth(
  const std::vector<Vec2> & pts, const std::vector<Ray> & rays, const std::vector<Hazard> & hz,
  double soft_m, double floor_m)
{
  const std::size_t n = pts.size();
  if (n < 3) {return pts;}
  std::vector<double> cl(n);
  std::vector<std::uint32_t> pre(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    cl[i] = path::minClearance(hz, pts[i]);
    if (i > 0) {pre[i] = pre[i - 1] ^ segmentMask(rays, pts[i - 1], pts[i]);}
  }
  std::vector<Vec2> out{pts[0]};
  std::size_t i = 0;
  while (i + 1 < n) {
    std::size_t best = i + 1;
    double run = std::min(soft_m, std::min(cl[i], cl[i + 1]));
    for (std::size_t j = i + 2; j < n; ++j) {
      run = std::min(run, cl[j]);
      if (segmentMask(rays, pts[i], pts[j]) != (pre[j] ^ pre[i])) {break;}
      if (!path::segmentClear(pts[i], pts[j], hz, std::max(floor_m, std::min(soft_m, run) - 0.05), 0.05)) {
        break;
      }
      // local: each sample against the replaced vertices round the same fraction of the way
      bool keeps = true;
      const int ns = std::max(1, static_cast<int>(std::ceil(nav::norm(pts[j] - pts[i]) / 0.05)));
      for (int k = 0; k <= ns && keeps; ++k) {
        const double t = static_cast<double>(k) / ns;
        const std::size_t at = i + static_cast<std::size_t>(std::lround(t * static_cast<double>(j - i)));
        double ref = cl[at];
        for (std::size_t m = (at >= i + 2 ? at - 2 : i); m <= std::min(j, at + 2); ++m) {
          ref = std::min(ref, cl[m]);
        }
        const double thr = std::max(floor_m, std::min(soft_m, ref) - 0.1);
        keeps = !(path::minClearance(hz, pts[i] + (pts[j] - pts[i]) * t) < thr);
      }
      if (!keeps) {break;}
      best = j;
    }
    out.push_back(pts[best]);
    i = best;
  }
  return out;
}

// -------------------------------------------------------------------- orbits

struct Ring
{
  bool ok = false;
  Vec2 c;
  double r = 0.0;
  bool cw = true;
  std::vector<Vec2> pts;       ///< n vertices in the orbit direction, pts[0] due east of c
  double clear = 0.0;          ///< the ring's lowest clearance from every OTHER hazard
  bool encloses = false;       ///< a black buoy or a track is inside it
  std::string why;
};

inline std::vector<Vec2> ringPoints(Vec2 c, double r, bool cw, double step_m)
{
  const int n = std::max(16, static_cast<int>(std::ceil(2.0 * nav::kPi * r / std::max(0.02, step_m))));
  std::vector<Vec2> pts;
  pts.reserve(static_cast<std::size_t>(n));
  for (int k = 0; k < n; ++k) {
    const double a = (cw ? -1.0 : 1.0) * 2.0 * nav::kPi * k / n;
    pts.push_back(c + Vec2{std::cos(a), std::sin(a)} * r);
  }
  return pts;
}

/// The orbit round the anchor `c`: the largest radius up to min(orbit_r_pref_m, r_cap) whose
/// ring keeps max(hard, orbit_clear_m) from every other hazard and encloses nothing (then:
/// the same, a black buoy or a track allowed inside); failing both, the radius with the most
/// water on both sides that is still >= hard from everything. A red, green or blue buoy is
/// never inside an orbit. `others` must already leave out the anchor's own hazards; `buoys`
/// says which hazard is which colour.
inline Ring chooseRing(
  Vec2 c, double anchor_r, const std::vector<Hazard> & others, const std::vector<nav::Buoy> & buoys,
  double r_cap, bool cw, double hard, const Params & p)
{
  Ring best;
  best.c = c;
  best.cw = cw;
  const auto colour_of = [&](int id) {
      const nav::Buoy * b = nav::findById(buoys, id);
      return b == nullptr ? nav::Beacon::Unknown : b->state;
    };
  // true = something that must never be inside is inside; *black = something else is
  const auto inside = [&](double r, bool * black) {
      for (const Hazard & h : others) {
        const bool circle = h.kind == path::HazardKind::Circle;
        const Vec2 hc = circle ? h.c : (h.poly.empty() ? Vec2{} : h.poly.front());
        if (!circle && h.poly.empty()) {continue;}
        if (nav::norm(hc - c) >= r) {continue;}
        const nav::Beacon s = h.source == path::HazardSource::PlanBuoy ? colour_of(h.id) :
          nav::Beacon::Off;
        if (s == nav::Beacon::FlashingRed || s == nav::Beacon::FlashingGreen ||
          s == nav::Beacon::FlashingBlue || s == nav::Beacon::SteadyBlue)
        {
          return true;
        }
        *black = true;
      }
      return false;
    };
  const auto ringClear = [&](double r) {
      double m = std::numeric_limits<double>::infinity();
      for (const Vec2 & q : ringPoints(c, r, cw, 0.2)) {m = std::min(m, path::minClearance(others, q));}
      return m;
    };
  const double need = std::max(hard, p.orbit_clear_m);
  double hi = p.orbit_r_pref_m;
  if (r_cap >= p.orbit_r_min_m) {hi = std::min(hi, r_cap);}
  const double lo_tight = anchor_r + hard;
  for (int pass = 0; pass < 2; ++pass) {
    for (double r = hi; r >= p.orbit_r_min_m - 1e-9; r -= 0.05) {
      bool black = false;
      if (inside(r, &black) || (pass == 0 && black)) {continue;}
      if (r - anchor_r < hard) {continue;}
      const double rc = ringClear(r);
      if (rc >= need) {
        best.ok = true;
        best.r = r;
        best.clear = rc;
        best.encloses = black;
        best.pts = ringPoints(c, r, cw, p.orbit_step_m);
        return best;
      }
    }
  }
  // tight: the most water on both sides, at least hard
  double best_m = -1.0, best_r = 0.0, best_c = 0.0;
  bool best_black = false;
  for (double r = std::max(hi, lo_tight); r >= lo_tight - 1e-9; r -= 0.05) {
    bool black = false;
    if (inside(r, &black)) {continue;}
    const double rc = ringClear(r);
    const double m = std::min(rc, r - anchor_r);
    if (m > best_m) {best_m = m; best_r = r; best_c = rc; best_black = black;}
  }
  if (best_m >= hard) {
    best.ok = true;
    best.r = best_r;
    best.clear = best_c;
    best.encloses = best_black;
    best.pts = ringPoints(c, best_r, cw, p.orbit_step_m);
    return best;
  }
  best.why = best_m < 0.0 ? "every radius encloses a red, green or blue buoy" :
    "no radius keeps " + detail::fmt(hard, 2) + " m from the buoy and from its neighbours "
    "(best " + detail::fmt(best_m, 2) + " m)";
  return best;
}

/// How many ring steps past one full turn: at least overshoot_deg, and enough that a leg that
/// ends follow_early_m short still sweeps 360 + 20 degrees.
inline int overshootSteps(const Ring & ring, const Params & p)
{
  const int n = static_cast<int>(ring.pts.size());
  if (n == 0 || !(ring.r > 0.0)) {return 0;}
  const double early_deg = p.follow_early_m / ring.r / nav::kDeg + 20.0;
  const double deg = std::min(330.0, std::max(p.overshoot_deg, early_deg));
  return static_cast<int>(std::ceil(deg / (360.0 / n)));
}

/// One full turn from vertex `start`, then `extra` more steps the same way round.
inline std::vector<Vec2> orbitPath(const Ring & ring, int start, int extra)
{
  std::vector<Vec2> out;
  const int n = static_cast<int>(ring.pts.size());
  if (n == 0) {return out;}
  for (int i = 0; i <= n + extra; ++i) {
    out.push_back(ring.pts[static_cast<std::size_t>(((start + i) % n + n) % n)]);
  }
  return out;
}

/// Degrees swept round `c` by a polyline, signed (+ = counter-clockwise).
inline double sweptDeg(Vec2 c, const std::vector<Vec2> & pl)
{
  double sum = 0.0;
  for (std::size_t i = 1; i < pl.size(); ++i) {
    const Vec2 a = pl[i - 1] - c, b = pl[i] - c;
    sum += std::atan2(nav::cross(a, b), nav::dot(a, b));
  }
  return sum / nav::kDeg;
}

// ------------------------------------------------------------ re-tasking

/// What a plan was made against, as far as re-tasking goes: the aircraft's own report (each
/// buoy's id, colour and position, its ENTRY and EXIT). NOT the fused field: the tracker moves
/// fused positions every frame, and that is the clearance monitor's business, not a re-task.
struct FieldSig
{
  std::vector<nav::PlanBuoy> buoys;      ///< sorted by id
  Vec2 entry, exitp;
};

inline FieldSig fieldSig(std::vector<nav::PlanBuoy> buoys, Vec2 entry, Vec2 exitp)
{
  std::sort(buoys.begin(), buoys.end(),
    [](const nav::PlanBuoy & a, const nav::PlanBuoy & b) {return a.id < b.id;});
  return {std::move(buoys), entry, exitp};
}

inline const char * beaconWord(nav::Beacon b)
{
  switch (b) {
    case nav::Beacon::Off: return "OFF";
    case nav::Beacon::FlashingRed: return "RED";
    case nav::Beacon::FlashingGreen: return "GREEN";
    case nav::Beacon::FlashingBlue: return "ENTRY";
    case nav::Beacon::SteadyBlue: return "EXIT";
    case nav::Beacon::Unknown: return "UNKNOWN";
  }
  return "UNKNOWN";
}

/// "" when `now` is the same field as `was` for planning purposes; otherwise the first thing
/// that changed, for the log. A change is a buoy's colour, a buoy appearing or vanishing, or a
/// buoy, the ENTRY or the EXIT moving further than move_m. Scatter below that is not one: the
/// aircraft's re-reports of a field nobody touched differ by its own fix error every time, and
/// re-planning on each of them re-shaped the orbits every 5 s (SITL, 2026-10-02).
inline std::string fieldChange(const FieldSig & was, const FieldSig & now, double move_m)
{
  const auto moved = [move_m](Vec2 a, Vec2 b) {
      const bool fa = std::isfinite(a.x) && std::isfinite(a.y);
      const bool fb = std::isfinite(b.x) && std::isfinite(b.y);
      return fa != fb || (fa && nav::norm(a - b) > move_m);
    };
  if (moved(was.entry, now.entry)) {return "the ENTRY moved";}
  if (moved(was.exitp, now.exitp)) {return "the EXIT moved";}
  if (was.buoys.size() != now.buoys.size()) {
    return "the field has " + std::to_string(now.buoys.size()) + " buoys, was " +
      std::to_string(was.buoys.size());
  }
  for (std::size_t i = 0; i < now.buoys.size(); ++i) {
    const nav::PlanBuoy & a = was.buoys[i];
    const nav::PlanBuoy & b = now.buoys[i];
    if (a.id != b.id) {return "buoy " + std::to_string(b.id) + " is new";}
    if (a.state != b.state) {
      return "buoy " + std::to_string(b.id) + " " + beaconWord(a.state) + " -> " +
        beaconWord(b.state);
    }
    if (moved(a.p, b.p)) {return "buoy " + std::to_string(b.id) + " moved";}
  }
  return "";
}

// --------------------------------------------------------------- the plan

struct Checkpoint
{
  double s = 0.0;               ///< arc along the transit leg
  Vec2 p;
  int red_id = -1, green_id = -1;
};

struct Request
{
  Phase from = Phase::Approach;
  Vec2 boat;
  std::vector<nav::Buoy> buoys;          ///< the fused field, colours from the aircraft
  std::vector<Hazard> hazards;           ///< everything to keep clear of (no side fences)
  Vec2 entry, exitp;                     ///< the aircraft's ENTRY and EXIT
  /// From Transit on: the entry ring the boat orbited (its far edge bounds the passage) and
  /// the transit as driven, first point = where it began (its parity is the boat's state).
  std::vector<Vec2> entry_ring_done;
  std::vector<Vec2> traj;
  /// From Transit on: the (id, red) constraints the previous plan held. A buoy recoloured
  /// after the boat passed it on what is now the wrong side is not chased back round.
  std::vector<std::pair<int, bool>> prev_rays;
  std::vector<std::pair<int, int>> cleared;   ///< gates already confirmed (no checkpoint again)
};

struct Plan
{
  bool ok = false;
  std::string why;
  Phase from = Phase::Approach;
  std::vector<Vec2> legs[4];             ///< indexed by Phase; empty before `from`
  std::vector<Checkpoint> checkpoints;   ///< on legs[Transit], in order
  Vec2 axis;
  Vec2 entry_c, exit_c;
  int entry_id = -1, exit_id = -1;
  double entry_r = 0.0, exit_r = 0.0;
  std::vector<Vec2> entry_ring;          ///< the entry ring this plan orbits (empty from Transit on)
  std::vector<Ray> rays;
  std::vector<int> excluded;             ///< red/green kept clear of but not held to a side
  double hard_used = 0.0;
  bool relaxed = false;
  double min_clear = std::numeric_limits<double>::infinity();
  double transit_clear = std::numeric_limits<double>::infinity();
  double entry_sweep_deg = 0.0, exit_sweep_deg = 0.0;
  long expanded = 0;
  int layers = 0;
  std::vector<std::string> notes;
};

namespace detail
{

/// The fused blue buoy nearest the aircraft's point (within snap_m) anchors the orbit: the
/// tracker's fix is better than a fix from 60 m up. None: the aircraft's point, id -1.
inline std::pair<Vec2, int> anchorOf(
  const std::vector<nav::Buoy> & buoys, Vec2 point, nav::Beacon want, double snap_m)
{
  const nav::Buoy * best = nullptr;
  double bd = snap_m;
  for (const nav::Buoy & b : buoys) {
    if (b.state != want || !std::isfinite(b.p.x) || !std::isfinite(b.p.y)) {continue;}
    const double d = nav::norm(b.p - point);
    if (d <= bd) {bd = d; best = &b;}
  }
  return best ? std::make_pair(best->p, best->id) : std::make_pair(point, -1);
}

inline double anchorRadius(const std::vector<Hazard> & hz, int id, double fallback)
{
  for (const Hazard & h : hz) {
    if (h.source == path::HazardSource::PlanBuoy && h.id == id &&
      h.kind == path::HazardKind::Circle)
    {
      return h.r + h.keepout;
    }
  }
  return fallback;
}

/// First crossing of the polyline with segment a-b, as an arc length; -1 = none.
inline double firstCrossing(
  const std::vector<Vec2> & pl, const std::vector<double> & cum, Vec2 a, Vec2 b)
{
  for (std::size_t i = 1; i < pl.size(); ++i) {
    const Vec2 p = pl[i - 1], r = pl[i] - pl[i - 1], s = b - a;
    const double den = nav::cross(r, s);
    if (std::fabs(den) < 1e-12) {continue;}
    const double t = nav::cross(a - p, s) / den, u = nav::cross(a - p, r) / den;
    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {return cum[i - 1] + t * nav::norm(r);}
  }
  return -1.0;
}

}  // namespace detail

/// Where the Disruptive tree stops to ask: just past every GATE the transit drives through, at
/// the most open water of a short window. A gate is a held red and a held green, min_w..max_w
/// apart, whose connecting segment the transit actually crosses; the narrowest such pairs are
/// taken first and no buoy is in two gates. Pairing by the PATH rather than by the axis is what
/// keeps a winding passage's gates (a hairpin's, crossed at 90 degrees to the axis) as gates.
/// A gate already in `cleared` still claims its two buoys but gets no checkpoint.
inline std::vector<Checkpoint> placeCheckpoints(
  const std::vector<Vec2> & transit, const std::vector<Ray> & rays,
  const std::vector<Hazard> & hz, const std::vector<std::pair<int, int>> & cleared,
  const Params & p, double min_w = 2.0, double max_w = 20.0)
{
  std::vector<Checkpoint> out;
  if (transit.size() < 2) {return out;}
  const std::vector<double> cum = arcLengths(transit);
  const double total = cum.back();
  struct Cand {double w, sx; int r, g;};
  std::vector<Cand> cands;
  for (const Ray & rr : rays) {
    if (!rr.red) {continue;}
    for (const Ray & gg : rays) {
      if (gg.red) {continue;}
      const double w = nav::norm(rr.o - gg.o);
      if (w < min_w || w > max_w) {continue;}
      const double sx = detail::firstCrossing(transit, cum, rr.o, gg.o);
      if (sx >= 0.0) {cands.push_back({w, sx, rr.id, gg.id});}
    }
  }
  std::sort(cands.begin(), cands.end(), [](const Cand & a, const Cand & b) {return a.w < b.w;});
  std::vector<int> used;
  std::vector<Cand> gates;
  const auto taken = [&](int id) {return std::find(used.begin(), used.end(), id) != used.end();};
  for (const Cand & c : cands) {
    if (taken(c.r) || taken(c.g)) {continue;}
    used.push_back(c.r);
    used.push_back(c.g);
    if (!nav::gateCleared(cleared, c.r, c.g)) {gates.push_back(c);}
  }
  std::sort(gates.begin(), gates.end(), [](const Cand & a, const Cand & b) {return a.sx < b.sx;});
  for (std::size_t k = 0; k < gates.size(); ++k) {
    const double sx = gates[k].sx;
    const double lo = std::min(sx + p.cp_after_m, total);
    double hi = std::min(sx + p.cp_after_m + p.cp_window_m, total - 0.5);
    if (k + 1 < gates.size()) {hi = std::min(hi, gates[k + 1].sx - 1.0);}
    if (hi < lo) {hi = lo;}
    double best_s = lo, best_c = -1.0;
    for (double s = lo; s <= hi + 1e-9; s += 0.1) {
      const double c = path::minClearance(hz, pointAt(transit, cum, s));
      if (c > best_c + 1e-6) {best_c = c; best_s = s;}
    }
    Checkpoint cp;
    cp.s = best_s;
    cp.p = pointAt(transit, cum, best_s);
    cp.red_id = gates[k].r;
    cp.green_id = gates[k].g;
    out.push_back(cp);
  }
  return out;
}

/// Which side of the path each held buoy ended up on, at its closest approach, the way a
/// referee would judge it. Returns the ids on the WRONG side (should always be empty: the
/// parity guarantees it for any path that does not loop round a buoy).
inline std::vector<int> wrongSide(const std::vector<Vec2> & pl, const std::vector<Ray> & rays)
{
  std::vector<int> bad;
  if (pl.size() < 2) {return bad;}
  for (const Ray & r : rays) {
    double bd = std::numeric_limits<double>::infinity();
    double side = 0.0;
    for (std::size_t i = 1; i < pl.size(); ++i) {
      const Vec2 a = pl[i - 1], ab = pl[i] - pl[i - 1];
      const double l2 = nav::dot(ab, ab);
      if (l2 < 1e-12) {continue;}
      const double t = std::clamp(nav::dot(r.o - a, ab) / l2, 0.0, 1.0);
      const double d = nav::norm(r.o - (a + ab * t));
      if (d < bd) {bd = d; side = nav::cross(ab, r.o - a);}
    }
    // red must be to starboard (cross < 0), green to port (cross > 0)
    if ((r.red && side >= 0.0) || (!r.red && side <= 0.0)) {bad.push_back(r.id);}
  }
  return bad;
}

namespace detail
{

inline std::vector<GoalPt> ringGoals(const Ring & ring, const std::vector<Ray> & rays, Vec2 axis)
{
  std::vector<GoalPt> goals;
  for (const Vec2 & q : ring.pts) {goals.push_back({q, endClosure(rays, q, axis)});}
  return goals;
}

/// Plan at one clearance. `grid` is shared by every try (its clearance does not depend on it).
inline Plan planAt(const Request & rq, const Params & p, const Grid & grid, double hard)
{
  Plan out;
  out.from = rq.from;
  out.hard_used = hard;
  const Vec2 axis = nav::unit(rq.exitp - rq.entry);
  out.axis = axis;
  const double L = nav::norm(rq.exitp - rq.entry);

  // Anchors and their rings.
  const auto ea = anchorOf(rq.buoys, rq.entry, nav::Beacon::FlashingBlue, p.anchor_snap_m);
  const auto xa = anchorOf(rq.buoys, rq.exitp, nav::Beacon::SteadyBlue, p.anchor_snap_m);
  out.entry_c = ea.first;
  out.entry_id = ea.second;
  out.exit_c = xa.first;
  out.exit_id = xa.second;
  // The rings stay short of the passage (along the axis): the first and last red/green.
  double first_along = std::numeric_limits<double>::infinity(), last_along = -first_along;
  for (const nav::Buoy & b : rq.buoys) {
    if (!nav::isSideConstrained(b.state) || !std::isfinite(b.p.x) || !std::isfinite(b.p.y)) {
      continue;
    }
    const double a = nav::dot(b.p - rq.entry, axis);
    if (a > 0.0) {first_along = std::min(first_along, a);}
    if (a < L) {last_along = std::max(last_along, a);}
  }
  const auto others_of = [&](int id) {
      return id >= 0 ? path::exempt(rq.hazards, {id}, false, 1.0) : rq.hazards;
    };
  Ring xr;
  {
    const double cap = std::isfinite(last_along) ?
      (L - last_along) - p.along_margin_m : std::numeric_limits<double>::infinity();
    xr = chooseRing(out.exit_c, anchorRadius(rq.hazards, out.exit_id, p.buoy_radius_m),
        others_of(out.exit_id), rq.buoys, cap, false, hard, p);
    if (!xr.ok) {out.why = "no orbit fits round the EXIT: " + xr.why; return out;}
  }
  out.exit_r = xr.r;
  Ring er;
  const bool with_entry = rq.from == Phase::Approach || rq.from == Phase::EntryOrbit;
  if (with_entry) {
    const double cap = std::isfinite(first_along) ?
      first_along - p.along_margin_m : std::numeric_limits<double>::infinity();
    er = chooseRing(out.entry_c, anchorRadius(rq.hazards, out.entry_id, p.buoy_radius_m),
        others_of(out.entry_id), rq.buoys, cap, true, hard, p);
    if (!er.ok) {out.why = "no orbit fits round the ENTRY: " + er.why; return out;}
    out.entry_r = er.r;
    out.entry_ring = er.pts;
  }

  // The passage: which red/green are held to a side. Not one whose ray runs into the line
  // closing the transit behind the ENTRY or beyond the EXIT (it is behind the start or past
  // the finish, and holding it would send the boat round it), nor one the orbit itself
  // brushes past (its side is passed both ways while circling).
  const std::vector<Vec2> & ering = with_entry ? er.pts : rq.entry_ring_done;
  const auto brushes = [](const std::vector<Vec2> & ring, Vec2 q) {
      return std::any_of(ring.begin(), ring.end(), [q](Vec2 v) {return nav::norm(v - q) < 1.3;});
    };
  out.rays = buildRays(rq.buoys, axis, [&](const Ray & r) -> std::string {
        if (segmentCrossesRay(r, out.entry_c - axis * kFar, out.entry_c)) {return "is behind the ENTRY";}
        if (segmentCrossesRay(r, out.exit_c, out.exit_c + axis * kFar)) {return "is beyond the EXIT";}
        if (brushes(ering, r.o)) {return "is beside the ENTRY orbit";}
        if (brushes(xr.pts, r.o)) {return "is beside the EXIT orbit";}
        return "";
      }, &out.excluded, &out.notes);

  // Transit sources.
  std::vector<Source> tsrc;
  int approach_goal = -1;
  std::vector<Vec2> approach;
  if (with_entry) {
    // boat -> the entry ring: no sides yet
    std::vector<GoalPt> eg;
    for (const Vec2 & q : er.pts) {eg.push_back({q, 0u});}
    const Found a = search(grid, {}, {{rq.boat, 0.0, 0u}}, eg, er.c, er.r, hard, p);
    out.expanded += a.expanded;
    if (!a.ok) {out.why = "no water from the boat to the ENTRY orbit (" + a.why + ")"; return out;}
    approach_goal = a.goal;
    approach = densify(smooth(a.pts, {}, rq.hazards, p.soft_m, hard - 0.05), p.path_step_m);
    // every ring vertex can start the transit, charged the orbit it adds past one full turn
    const int n = static_cast<int>(er.pts.size());
    const int ovs = overshootSteps(er, p);
    const double step_len = 2.0 * nav::kPi * er.r / n;
    for (int k = 0; k < n; ++k) {
      const int rel = ((k - approach_goal) % n + n) % n;
      const int extra = rel >= ovs ? rel : rel + n;
      tsrc.push_back({er.pts[static_cast<std::size_t>(k)], extra * step_len,
          startClosure(out.rays, er.pts[static_cast<std::size_t>(k)], axis)});
    }
  } else if (rq.from == Phase::Transit) {
    std::uint32_t m;
    if (rq.traj.empty()) {
      m = startClosure(out.rays, rq.boat, axis);
    } else {
      m = startClosure(out.rays, rq.traj.front(), axis) ^ polylineMask(out.rays, rq.traj) ^
        segmentMask(out.rays, rq.traj.back(), rq.boat);
    }
    // A buoy recoloured (or newly held) after the boat went past it on what is now the wrong
    // side: chasing back round it is not what the change asks for. Leave it as an obstacle.
    std::vector<Ray> kept;
    std::uint32_t km = 0;
    for (std::size_t k = 0; k < out.rays.size(); ++k) {
      const Ray & r = out.rays[k];
      const bool odd = (m >> k) & 1u;
      const bool before = std::any_of(rq.prev_rays.begin(), rq.prev_rays.end(),
          [&](const std::pair<int, bool> & pr) {return pr.first == r.id && pr.second == r.red;});
      const bool behind = nav::dot(r.o - rq.boat, axis) < -1.0;
      if (odd && !before && behind && !rq.prev_rays.empty()) {
        out.excluded.push_back(r.id);
        out.notes.push_back("buoy " + std::to_string(r.id) + " became " +
          (r.red ? "RED" : "GREEN") + " after the boat passed it: side not chased");
        continue;
      }
      if (odd) {km |= (1u << kept.size());}
      kept.push_back(r);
    }
    out.rays = kept;
    m = km;
    tsrc.push_back({rq.boat, 0.0, m});
  }

  // The transit, or (from ExitOrbit) the hop back onto the exit ring.
  const int xovs = overshootSteps(xr, p);
  if (rq.from == Phase::ExitOrbit) {
    std::vector<GoalPt> xg;
    for (const Vec2 & q : xr.pts) {xg.push_back({q, 0u});}
    const Found c = search(grid, {}, {{rq.boat, 0.0, 0u}}, xg, xr.c, xr.r, hard, p);
    out.expanded += c.expanded;
    if (!c.ok) {out.why = "no water from the boat to the EXIT orbit (" + c.why + ")"; return out;}
    std::vector<Vec2> leg = densify(smooth(c.pts, {}, rq.hazards, p.soft_m, hard - 0.05), p.path_step_m);
    const std::vector<Vec2> orbit = orbitPath(xr, c.goal, xovs);
    leg.insert(leg.end(), orbit.begin() + 1, orbit.end());
    out.legs[static_cast<int>(Phase::ExitOrbit)] = leg;
    out.exit_sweep_deg = sweptDeg(xr.c, orbit);
  } else {
    const Found t = search(grid, out.rays, tsrc, ringGoals(xr, out.rays, axis), xr.c, xr.r, hard, p);
    out.expanded += t.expanded;
    out.layers = t.layers;
    if (!t.ok) {
      out.why = "no passage keeps every side (" + t.why + ", " + std::to_string(out.rays.size()) +
        " buoys held)";
      return out;
    }
    std::vector<Vec2> transit =
      densify(smooth(t.pts, out.rays, rq.hazards, p.soft_m, hard - 0.05), p.path_step_m);
    // The parity is the contract: check it on what will actually be driven.
    const std::uint32_t total_mask = tsrc[static_cast<std::size_t>(t.src)].mask ^
      polylineMask(out.rays, transit) ^ endClosure(out.rays, transit.back(), axis);
    if (total_mask != 0u) {
      out.why = "internal: the transit does not keep every side (mask " +
        std::to_string(total_mask) + ")";
      return out;
    }
    if (with_entry) {
      const int n = static_cast<int>(er.pts.size());
      const int ovs = overshootSteps(er, p);
      const int rel = ((t.src - approach_goal) % n + n) % n;
      const std::vector<Vec2> orbit = orbitPath(er, approach_goal, rel >= ovs ? rel : rel + n);
      out.entry_sweep_deg = sweptDeg(er.c, orbit);
      if (rq.from == Phase::Approach) {
        out.legs[static_cast<int>(Phase::Approach)] = approach;
        out.legs[static_cast<int>(Phase::EntryOrbit)] = orbit;
      } else {
        std::vector<Vec2> leg = approach;
        leg.insert(leg.end(), orbit.begin() + 1, orbit.end());
        out.legs[static_cast<int>(Phase::EntryOrbit)] = leg;
      }
    }
    out.legs[static_cast<int>(Phase::Transit)] = transit;
    const std::vector<Vec2> xorbit = orbitPath(xr, t.goal, xovs);
    out.legs[static_cast<int>(Phase::ExitOrbit)] = xorbit;
    out.exit_sweep_deg = sweptDeg(xr.c, xorbit);
    out.transit_clear = polylineClearance(transit, rq.hazards);
    out.checkpoints = placeCheckpoints(transit, out.rays, rq.hazards, rq.cleared, p);
    // judged like a referee would: on the WHOLE transit, the part already driven included
    std::vector<Vec2> judged = transit;
    if (rq.from == Phase::Transit && !rq.traj.empty()) {
      judged = rq.traj;
      judged.push_back(rq.boat);
      judged.insert(judged.end(), transit.begin(), transit.end());
    }
    for (const int id : wrongSide(judged, out.rays)) {
      out.notes.push_back("WARNING buoy " + std::to_string(id) +
        " is on the wrong side at its closest approach (the path loops round it?)");
    }
  }
  for (int k = static_cast<int>(rq.from); k < 4; ++k) {
    if (!out.legs[k].empty()) {out.min_clear = std::min(out.min_clear, polylineClearance(out.legs[k], rq.hazards));}
  }
  out.ok = true;
  return out;
}

}  // namespace detail

/// The rest of the mission from `rq.from`, starting at the boat. Tries hard_m, then the two
/// relaxed clearances; the first that fits wins (relaxed = true if not the first).
inline Plan plan(const Request & rq, const Params & p)
{
  Plan out;
  out.from = rq.from;
  if (!path::detail::finite(rq.entry) || !path::detail::finite(rq.exitp) ||
    nav::norm(rq.exitp - rq.entry) < 1.0)
  {
    out.why = "no ENTRY -> EXIT axis";
    return out;
  }
  if (!path::detail::finite(rq.boat)) {out.why = "no boat position"; return out;}
  if (rq.from == Phase::Done) {out.why = "nothing left to plan"; return out;}
  std::vector<Vec2> pts{rq.boat, rq.entry, rq.exitp};
  for (const nav::Buoy & b : rq.buoys) {pts.push_back(b.p);}
  pts.insert(pts.end(), rq.traj.begin(), rq.traj.end());
  // room for the largest orbit
  for (const Vec2 c : {rq.entry, rq.exitp}) {
    pts.push_back(c + Vec2{p.orbit_r_pref_m + 1.0, p.orbit_r_pref_m + 1.0});
    pts.push_back(c - Vec2{p.orbit_r_pref_m + 1.0, p.orbit_r_pref_m + 1.0});
  }
  const Grid grid = makeGrid(pts, rq.hazards, p);
  std::vector<double> tries{p.hard_m};
  for (const double r : {p.relax1_m, p.relax2_m}) {
    if (r > 0.0 && r < tries.back() - 1e-9) {tries.push_back(r);}
  }
  std::string whys;
  for (const double hard : tries) {
    Plan pl = detail::planAt(rq, p, grid, hard);
    if (pl.ok) {
      pl.relaxed = hard < p.hard_m - 1e-9;
      if (pl.relaxed) {
        pl.notes.push_back("RELAXED: nothing fits at " + detail::fmt(p.hard_m, 2) +
          " m of clearance; planned at " + detail::fmt(hard, 2) + " m (" + whys + ")");
      }
      return pl;
    }
    whys += (whys.empty() ? "" : "; ") + detail::fmt(hard, 2) + " m: " + pl.why;
    out = pl;
  }
  out.why = whys;
  return out;
}

/// One line for the log.
inline std::string summary(const Plan & pl)
{
  if (!pl.ok) {return "no plan: " + pl.why;}
  std::string s = "from " + std::string(phaseName(pl.from)) + ":";
  const char * names[4] = {"approach", "entry orbit", "transit", "exit orbit"};
  for (int k = 0; k < 4; ++k) {
    if (pl.legs[k].empty()) {continue;}
    s += std::string(" ") + names[k] + " " +
      detail::fmt(arcLengths(pl.legs[k]).back(), 1) + " m,";
  }
  if (pl.entry_r > 0.0) {s += " entry r " + detail::fmt(pl.entry_r, 2) + " (" + detail::fmt(pl.entry_sweep_deg, 0) + " deg),";}
  s += " exit r " + detail::fmt(pl.exit_r, 2) + " (" + detail::fmt(pl.exit_sweep_deg, 0) + " deg),";
  s += " " + std::to_string(pl.rays.size()) + " buoys held to a side";
  if (!pl.excluded.empty()) {s += ", " + std::to_string(pl.excluded.size()) + " not held (see notes)";}
  s += ", " + std::to_string(pl.checkpoints.size()) + " checkpoint(s), min clearance " +
    detail::fmt(pl.min_clear, 2) + " m";
  if (pl.relaxed) {s += " RELAXED to " + detail::fmt(pl.hard_used, 2) + " m";}
  return s;
}

}  // namespace gp
}  // namespace crusader_bt

#endif  // CRUSADER_BT__GLOBAL_PASSAGE_HPP_
