// test_side_fences — prove the Task 1 side fences with no ROS, no Nav2, no boat.
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include -o /tmp/t test/test_side_fences.cpp && /tmp/t
//
// handbook 3.3.2: every RED is passed on the boat's starboard, every GREEN on its port. The
// planner knows hazards, not sides, so path::sideFences turns each coloured buoy into a wall
// of small hazard circles running OUTWARD from it. This file checks
//
//   1  the geometry of the walls (paired: along the gate line away from the partner; single:
//      red right / green left of the entry -> exit axis), the count, the start, the source id;
//   2  what stays out of the way: the orbit ring at ENTRY and EXIT (adjustRing), and a gate's
//      own straight crossing line;
//   3  that the walls WORK: a reference planner (a grid A*, test code only -- Nav2 is not here)
//      drives the legs the disruptive tree plans over the task1_unpaired field, and every red
//      and green ends up on its handbook side with the fences and, for the singles, does not
//      without them.
//
// The reference planner is a stand-in for SmacPlanner2D on the inflated costmap: shortest path
// over cells at least 1.0 m from every hazard surface (hard 0.8 + 0.2), with a penalty inside
// the soft zone. It is not a replica -- it shows that the walls leave a correct route the
// shortest one, which is the property the sim then confirms with the real planner.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <queue>
#include <string>
#include <vector>

#include "crusader_bt/path_math.hpp"

using namespace crusader_bt;         // NOLINT(build/namespaces) -- a test
using namespace crusader_bt::path;   // NOLINT(build/namespaces)
using nav::Beacon;
using nav::Buoy;
using nav::Vec2;

static int g_fails = 0;
static int g_checks = 0;

static void chk(const std::string & name, bool pass)
{
  ++g_checks;
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s\n", pass ? "ok" : "FAIL", name.c_str());
}

static Buoy mk(int id, double x, double y, Beacon s) {return Buoy{id, {x, y}, s, false};}

static std::vector<Hazard> buoyHazards(const std::vector<Buoy> & field, const NavParams & P)
{
  return buildHazards(field, {}, dock::DockBook{}, 0, P);
}

/// The hazard set bt_runner publishes in nav_mode on: buoys plus fences.
static std::vector<Hazard> withFences(
  const std::vector<Buoy> & field, const nav::Passage & pa, Vec2 entry, Vec2 exitp,
  const NavParams & P)
{
  std::vector<Hazard> hz = buoyHazards(field, P);
  const std::vector<Hazard> f = sideFences(field, pa, entry, exitp, P);
  hz.insert(hz.end(), f.begin(), f.end());
  return hz;
}

static std::size_t countFor(const std::vector<Hazard> & hz, int id)
{
  return static_cast<std::size_t>(std::count_if(
      hz.begin(), hz.end(),
      [&](const Hazard & h) {return h.source == HazardSource::Fence && h.id == id;}));
}

/// The fence circles of one buoy, nearest first.
static std::vector<Hazard> fenceOf(const std::vector<Hazard> & hz, int id)
{
  std::vector<Hazard> out;
  for (const Hazard & h : hz) {
    if (h.source == HazardSource::Fence && h.id == id) {out.push_back(h);}
  }
  return out;
}

// ------------------------------------------------------- reference planner

/// Shortest path over a 0.25 m grid, 8-connected, cells >= `min_clear` from every hazard
/// surface, with a penalty inside `soft`. Empty when there is none.
static std::vector<Vec2> gridPath(
  const std::vector<Hazard> & hz, Vec2 a, Vec2 b, double min_clear = 1.0, double soft = 2.0)
{
  const double res = 0.25;
  double x0 = std::min(a.x, b.x), x1 = std::max(a.x, b.x);
  double y0 = std::min(a.y, b.y), y1 = std::max(a.y, b.y);
  for (const Hazard & h : hz) {
    x0 = std::min(x0, h.c.x); x1 = std::max(x1, h.c.x);
    y0 = std::min(y0, h.c.y); y1 = std::max(y1, h.c.y);
  }
  x0 -= 6.0; y0 -= 6.0; x1 += 6.0; y1 += 6.0;
  const int nx = static_cast<int>((x1 - x0) / res) + 1, ny = static_cast<int>((y1 - y0) / res) + 1;
  const auto at = [&](int i, int j) {return Vec2{x0 + i * res, y0 + j * res};};
  const auto cell = [&](Vec2 p) {
      return std::pair<int, int>{static_cast<int>(std::lround((p.x - x0) / res)),
        static_cast<int>(std::lround((p.y - y0) / res))};
    };

  std::vector<double> mult(static_cast<std::size_t>(nx) * ny, 1.0);
  const auto ci = [&](int i, int j) {return static_cast<std::size_t>(j) * nx + i;};
  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      const Vec2 p = at(i, j);
      double c = minClearance(hz, p);
      // the start and goal sit where the tree leaves the boat: never "inside" a hazard there
      if (nav::norm(p - a) < 1.5 || nav::norm(p - b) < 1.5) {c = std::max(c, soft);}
      mult[ci(i, j)] = c < min_clear ? -1.0 : (c < soft ? 1.0 + 3.0 * (soft - c) / (soft - min_clear) : 1.0);
    }
  }
  const auto s = cell(a), g = cell(b);
  if (s.first < 0 || s.second < 0 || g.first >= nx || g.second >= ny) {return {};}

  using Node = std::pair<double, std::size_t>;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
  std::vector<double> best(mult.size(), 1e18);
  std::vector<long> from(mult.size(), -1);
  best[ci(s.first, s.second)] = 0.0;
  open.push({0.0, ci(s.first, s.second)});
  const std::size_t goal = ci(g.first, g.second);
  while (!open.empty()) {
    const auto [f, u] = open.top();
    open.pop();
    if (u == goal) {break;}
    const int ui = static_cast<int>(u % nx), uj = static_cast<int>(u / nx);
    if (f > best[u] + nav::norm(at(ui, uj) - b) + 1e-9) {continue;}
    for (int dj = -1; dj <= 1; ++dj) {
      for (int di = -1; di <= 1; ++di) {
        if (di == 0 && dj == 0) {continue;}
        const int vi = ui + di, vj = uj + dj;
        if (vi < 0 || vj < 0 || vi >= nx || vj >= ny) {continue;}
        const double m = mult[ci(vi, vj)];
        if (m < 0.0) {continue;}
        const double step = res * (di != 0 && dj != 0 ? 1.41421356 : 1.0) * m;
        const std::size_t v = ci(vi, vj);
        if (best[u] + step < best[v]) {
          best[v] = best[u] + step;
          from[v] = static_cast<long>(u);
          open.push({best[v] + nav::norm(at(vi, vj) - b), v});
        }
      }
    }
  }
  if (from[goal] < 0 && goal != ci(s.first, s.second)) {return {};}
  std::vector<Vec2> path{b};
  for (long u = static_cast<long>(goal); from[static_cast<std::size_t>(u)] >= 0;
    u = from[static_cast<std::size_t>(u)])
  {
    path.push_back(at(static_cast<int>(from[static_cast<std::size_t>(u)] % nx),
      static_cast<int>(from[static_cast<std::size_t>(u)] / nx)));
  }
  path.push_back(a);
  std::reverse(path.begin(), path.end());
  return path;
}

/// The path resampled to at most `step` between points, so a pass is never missed between two
/// far-apart vertices (the straight crossing is only two).
static std::vector<Vec2> resample(const std::vector<Vec2> & path, double step = 0.25)
{
  std::vector<Vec2> out;
  for (std::size_t k = 0; k + 1 < path.size(); ++k) {
    const double len = nav::norm(path[k + 1] - path[k]);
    const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
    for (int i = 0; i < n; ++i) {out.push_back(path[k] + (path[k + 1] - path[k]) * (static_cast<double>(i) / n));}
  }
  if (!path.empty()) {out.push_back(path.back());}
  return out;
}

/// +1 = the buoy was passed with it to PORT, -1 = to STARBOARD, 0 = never passed. A pass is the
/// moment the buoy goes from ahead of the boat to behind it (the track abeam of it), with the
/// direction of travel taken over +-1 m of path; of several, the nearest -- as the referee does.
static int passSide(const std::vector<Vec2> & raw, Vec2 buoy, double * range = nullptr)
{
  const std::vector<Vec2> path = resample(raw);
  const int w = 4;                                           // 4 points = 1 m
  const int n = static_cast<int>(path.size());
  const auto ahead = [&](int k) {
      const Vec2 v = path[std::min(n - 1, k + w)] - path[std::max(0, k - w)];
      return nav::dot(v, buoy - path[k]);
    };
  int side = 0;
  double best = 1e18;
  for (int k = 0; k + 1 < n; ++k) {
    if (!(ahead(k) > 0.0 && ahead(k + 1) <= 0.0)) {continue;}
    const double r = nav::norm(buoy - path[k + 1]);
    if (r < best) {
      best = r;
      const Vec2 v = path[std::min(n - 1, k + 1 + w)] - path[std::max(0, k + 1 - w)];
      side = nav::cross(v, buoy - path[k + 1]) > 0.0 ? +1 : -1;
    }
  }
  if (range != nullptr) {*range = best;}
  return side;
}

static bool onItsSide(const std::vector<Vec2> & path, const Buoy & b)
{
  const int s = passSide(path, b.p);
  return b.state == Beacon::FlashingRed ? s == -1 : s == +1;     // red starboard, green port
}

static std::vector<Vec2> join(std::vector<Vec2> a, const std::vector<Vec2> & b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

// ------------------------------------------------------------ the field

/// task1_unpaired.yaml's passage (crusader_sim/courses), as the UAV would send it. The boat
/// starts at the origin heading east, so starboard is south: reds south, greens north.
///   ids   0 entry   1 exit   2,3 the gate (red, green)   4 a lone green on the course line's
///   right-hand side, 4 m south of it   5,6 the wrong-orientation pair (red NORTH of its green,
///   10 m ahead of it: only an S-curve passes both)   7-9 black
struct Field
{
  std::vector<Buoy> buoys;
  Vec2 entry{10.0, 2.0}, exitp{88.0, 0.0}, boat{0.0, 0.0};
};

static Field unpairedField()
{
  Field f;
  f.buoys = {
    mk(0, 10.0, 2.0, Beacon::FlashingBlue), mk(1, 88.0, 0.0, Beacon::SteadyBlue),
    mk(2, 30.0, -4.5, Beacon::FlashingRed), mk(3, 30.0, 4.5, Beacon::FlashingGreen),
    mk(4, 76.0, -4.0, Beacon::FlashingGreen),
    mk(5, 56.0, 1.2, Beacon::FlashingRed), mk(6, 66.0, -1.2, Beacon::FlashingGreen),
    mk(7, 19.0, 1.0, Beacon::Off), mk(8, 45.0, 0.2, Beacon::Off), mk(9, 79.0, 3.0, Beacon::Off)};
  return f;
}

/// The legs task1_disruptive.xml plans over the field's FIRST gate: ring end -> approach
/// (planned), approach -> through (straight), through -> the exit ring's first point (planned).
/// Empty when a planned leg finds no route.
static std::vector<Vec2> driveLegs(const Field & f, const std::vector<Hazard> & hz)
{
  const nav::Passage pa = nav::planPassage(f.buoys, f.entry, f.exitp);
  if (pa.gates.empty()) {return {};}
  const nav::Buoy * r = nav::findById(f.buoys, pa.gates[0].red_id);
  const nav::Buoy * g = nav::findById(f.buoys, pa.gates[0].green_id);
  const nav::Gate gate = nav::gateWaypoints(r->p, g->p, 6.0, 8.0);
  const std::vector<Vec2> ring = orbitRing(f.entry, f.boat, 6.0, 8, true, 45.0);
  const std::vector<Vec2> l1 = gridPath(hz, ring.back(), gate.approach);
  const std::vector<Vec2> xr = orbitRing(f.exitp, gate.through, 6.0, 8, false, 45.0);
  const std::vector<Vec2> l3 = gridPath(hz, gate.through, xr.front());
  if (l1.empty() || l3.empty()) {return {};}
  return join(join(l1, {gate.approach, gate.through}), l3);
}

/// Drive the field's legs with and without the fences and check every red and green. Prints the
/// side each was passed on.
static void scenario(const Field & f, const NavParams & P)
{
  const nav::Passage pa = nav::planPassage(f.buoys, f.entry, f.exitp);
  chk("the field has exactly one gate (ids 2 and 3)", pa.gates.size() == 1 &&
    ((pa.gates[0].red_id == 2 && pa.gates[0].green_id == 3) ||
    (pa.gates[0].red_id == 3 && pa.gates[0].green_id == 2)));
  chk("... and 3 singles: the lone buoy and the wrong-way pair", pa.unpaired.size() == 3);

  const std::vector<Hazard> fenced = withFences(f.buoys, pa, f.entry, f.exitp, P);
  const std::vector<Vec2> with = driveLegs(f, fenced);
  std::printf("    route: %zu points, %.1f m\n", with.size(), pathLength(with));
  chk("a route exists with the fences", with.size() > 10);
  int coloured = 0;
  bool all = true;
  for (const Buoy & b : f.buoys) {
    if (b.state != Beacon::FlashingRed && b.state != Beacon::FlashingGreen) {continue;}
    ++coloured;
    double r = 0.0;
    const int s = passSide(with, b.p, &r);
    const bool ok = onItsSide(with, b);
    all = all && ok;
    std::printf("    buoy %d %s: passed with it to %s at %.1f m\n", b.id,
      b.state == Beacon::FlashingRed ? "RED  " : "GREEN",
      s < 0 ? "starboard" : s > 0 ? "port" : "(not passed)", r);
    chk("buoy " + std::to_string(b.id) + " on its handbook side", ok);
  }
  chk("all " + std::to_string(coloured) + " coloured buoys are right", all && coloured == 5);

  double nearest = 1e18;
  for (const Vec2 & q : with) {nearest = std::min(nearest, minClearance(fenced, q));}
  chk("the route keeps >= 0.9 m from every hazard surface", nearest >= 0.9);

  // The control: the same legs over the buoys alone, the way nav_mode on used to plan them. The
  // lone buoy and the wrong-way pair are only obstacles then, and the shortest way past them is
  // the wrong one for at least one -- that is what the fences are for.
  const std::vector<Vec2> bare = driveLegs(f, buoyHazards(f.buoys, P));
  int wrong = 0;
  for (const Buoy & b : f.buoys) {
    if ((b.state == Beacon::FlashingRed || b.state == Beacon::FlashingGreen) && !onItsSide(bare, b)) {
      ++wrong;
    }
  }
  std::printf("    without fences: %d coloured buoys on the wrong side\n", wrong);
  chk("without the fences the same legs get at least one buoy wrong", wrong >= 1);
}

int main()
{
  const NavParams P;

  // ----------------------------------------------------------- 1. geometry
  std::printf("1. sideFences geometry\n");
  {
    // northbound axis, so starboard is EAST: a gate has its red east of its green
    const Vec2 entry{0, 0}, exitp{0, 100};
    const std::vector<Buoy> field{
      mk(0, 0, 0, Beacon::FlashingBlue), mk(1, 0, 100, Beacon::SteadyBlue),
      mk(2, 5, 30, Beacon::FlashingRed), mk(3, -5, 30, Beacon::FlashingGreen),     // a gate
      mk(4, 12, 60, Beacon::FlashingRed),                                          // single red
      mk(5, -12, 70, Beacon::FlashingGreen),                                       // single green
      mk(6, 0, 50, Beacon::Off)};
    const nav::Passage pa = nav::planPassage(field, entry, exitp);
    chk("the field plans one gate and two singles", pa.gates.size() == 1 && pa.unpaired.size() == 2);
    const std::vector<Hazard> fz = sideFences(field, pa, entry, exitp, P);

    // 10 m at 0.5 m is 21 circles from the buoy's hazard edge outward
    chk("21 circles per coloured buoy", countFor(fz, 2) == 21 && countFor(fz, 3) == 21 &&
      countFor(fz, 4) == 21 && countFor(fz, 5) == 21);
    chk("none for the entry, exit or a black buoy",
      countFor(fz, 0) == 0 && countFor(fz, 1) == 0 && countFor(fz, 6) == 0);
    chk("every circle is a Fence-source circle of fence_radius_m", std::all_of(
        fz.begin(), fz.end(), [&](const Hazard & h) {
          return h.source == HazardSource::Fence && h.kind == HazardKind::Circle &&
          std::fabs(h.r - P.fence_radius_m) < 1e-12;
        }));

    const std::vector<Hazard> r2 = fenceOf(fz, 2), g3 = fenceOf(fz, 3);
    // the red of the gate: east, away from its green at x = -5
    chk("gate red: the wall runs EAST, away from the green", std::all_of(
        r2.begin(), r2.end(), [](const Hazard & h) {return std::fabs(h.c.y - 30.0) < 1e-9 && h.c.x > 5.0;}));
    chk("gate green: the wall runs WEST, away from the red", std::all_of(
        g3.begin(), g3.end(), [](const Hazard & h) {return std::fabs(h.c.y - 30.0) < 1e-9 && h.c.x < -5.0;}));
    chk("the wall starts touching the buoy's own hazard circle",
      std::fabs(r2.front().c.x - (5.0 + P.buoy_radius_m + P.fence_radius_m)) < 1e-9);
    chk("... and spans fence_len_m", std::fabs(r2.back().c.x - r2.front().c.x - P.fence_len_m) < 1e-9);
    chk("... each circle overlapping the next (a wall, not dots)",
      P.fence_spacing_m < 2.0 * P.fence_radius_m);

    // the singles ignore any partner and go perpendicular to the axis (north): red EAST
    const std::vector<Hazard> s4 = fenceOf(fz, 4), s5 = fenceOf(fz, 5);
    chk("single red (northbound): the wall runs east, to the RIGHT of the axis", std::all_of(
        s4.begin(), s4.end(),
        [](const Hazard & h) {return std::fabs(h.c.y - 60.0) < 1e-9 && h.c.x > 12.0;}));
    chk("single green (northbound): the wall runs west, to the LEFT", std::all_of(
        s5.begin(), s5.end(),
        [](const Hazard & h) {return std::fabs(h.c.y - 70.0) < 1e-9 && h.c.x < -12.0;}));

    // a tilted axis: the singles still go perpendicular to IT, right for red / left for green
    const Vec2 e2{0, 0}, x2{80, 60};                       // axis (0.8, 0.6)
    const Vec2 axis = nav::unit(x2 - e2);
    const std::vector<Buoy> tilt{mk(0, 0, 0, Beacon::FlashingBlue), mk(1, 80, 60, Beacon::SteadyBlue),
      mk(2, 30, 20, Beacon::FlashingRed), mk(3, 40, 40, Beacon::FlashingGreen)};
    const std::vector<Hazard> tz = sideFences(tilt, nav::planPassage(tilt, e2, x2), e2, x2, P);
    const std::vector<Hazard> tr = fenceOf(tz, 2), tg = fenceOf(tz, 3);
    chk("tilted axis: red wall is right of the axis (cross < 0)", !tr.empty() && std::all_of(
        tr.begin(), tr.end(), [&](const Hazard & h) {return nav::cross(axis, h.c - Vec2{30, 20}) < 0.0;}));
    chk("tilted axis: green wall is left of the axis (cross > 0)", !tg.empty() && std::all_of(
        tg.begin(), tg.end(), [&](const Hazard & h) {return nav::cross(axis, h.c - Vec2{40, 40}) > 0.0;}));
    chk("tilted axis: and exactly perpendicular to it", std::all_of(
        tr.begin(), tr.end(),
        [&](const Hazard & h) {return std::fabs(nav::dot(axis, h.c - Vec2{30, 20})) < 1e-9;}));
  }

  std::printf("1b. switches and degenerate input\n");
  {
    const Field f = unpairedField();
    const nav::Passage pa = nav::planPassage(f.buoys, f.entry, f.exitp);
    NavParams off = P;
    off.fence_len_m = 0.0;
    chk("fence_len_m 0 = no fences", sideFences(f.buoys, pa, f.entry, f.exitp, off).empty());
    NavParams bad = P;
    bad.fence_spacing_m = 0.0;
    chk("a zero spacing is no fences, not an endless loop",
      sideFences(f.buoys, pa, f.entry, f.exitp, bad).empty());
    chk("an unplanned passage has no fences",
      sideFences(f.buoys, nav::Passage{}, f.entry, f.exitp, P).empty());
    chk("entry == exit has no fences",
      sideFences(f.buoys, pa, f.entry, f.entry, P).empty());
    std::vector<Buoy> nan = f.buoys;
    nan[4].p.x = std::nan("");
    chk("a buoy nobody can place draws no fence",
      countFor(sideFences(nan, pa, f.entry, f.exitp, P), 4) == 0);
  }

  // ------------------------------------------- 2a. the entry / exit orbits
  std::printf("2a. the orbit rings stay untouched\n");
  {
    // A red or green single placed every which way round the ENTRY, from 7.8 m (the nearest a
    // buoy can stand without ITSELF disturbing the ring: 6 + 1.4 + 0.3 + a little) to 25 m. Its wall (perpendicular to the axis) sweeps through the ring's neighbourhood in
    // the worst way a placement can. adjustRing must leave all eight ring points + the
    // overshoot one where orbitRing put them, with fence_clear_m at its default 8.0.
    const Vec2 entry{0, 0}, exitp{100, 0};
    const auto worst = [&](double clear_m, double * min_gap) {
        NavParams Q = P;
        Q.fence_clear_m = clear_m;
        int moved = 0;
        double gap = 1e18;
        for (const Beacon colour : {Beacon::FlashingRed, Beacon::FlashingGreen}) {
          for (double r = 7.8; r <= 25.0; r += 0.5) {
            for (double a = 0.0; a < 360.0; a += 7.5) {
              const Vec2 p = entry + Vec2{std::cos(a * nav::kDeg), std::sin(a * nav::kDeg)} * r;
              const std::vector<Buoy> field{mk(0, 0, 0, Beacon::FlashingBlue),
                mk(1, 100, 0, Beacon::SteadyBlue), mk(2, p.x, p.y, colour)};
              const std::vector<Hazard> hz = withFences(field, nav::planPassage(field, entry, exitp),
                  entry, exitp, Q);
              for (const double from : {0.0, 90.0, 200.0}) {
                const std::vector<Vec2> raw = orbitRing(
                  entry, entry + Vec2{std::cos(from * nav::kDeg), std::sin(from * nav::kDeg)} * 12.0,
                  6.0, 8, true, 45.0);
                int dropped = 0;
                const std::vector<Vec2> adj = adjustRing(raw, entry, hz, Q, &dropped);
                bool same = dropped == 0 && adj.size() == raw.size();
                for (std::size_t i = 0; same && i < raw.size(); ++i) {
                  same = nav::norm(adj[i] - raw[i]) < 1e-9;
                }
                if (!same) {++moved;}
                for (const Vec2 & q : raw) {gap = std::min(gap, minClearance(hz, q));}
              }
            }
          }
        }
        if (min_gap != nullptr) {*min_gap = gap;}
        return moved;
      };
    double gap8 = 0.0, gap75 = 0.0;
    const int moved8 = worst(8.0, &gap8), moved75 = worst(7.5, &gap75);
    chk("fence_clear_m 8.0: no ring point moves or drops, wherever a single sits", moved8 == 0);
    chk("... the nearest approach of any ring point to any hazard stays >= orbit_clear_m",
      gap8 >= P.orbit_clear_m - 1e-9);
    std::printf("    (ring clearance %.3f m at 8.0; at 7.5 it is %.3f m and %d placements move a point)\n",
      gap8, gap75, moved75);
    chk("7.5 would NOT do: 6 + orbit_clear_m 1.4 + radius 0.3 = 7.7 needs a wider berth", moved75 > 0);
    chk("the default is the one that works", P.fence_clear_m == 8.0 &&
      P.fence_clear_m >= 6.0 + P.orbit_clear_m + P.fence_radius_m);
  }

  std::printf("2b. no circle within fence_clear_m of the entry or exit\n");
  {
    const Field f = unpairedField();
    // a single green 6 m short of the exit and 8 m off the line: its wall runs north, across
    // the exit's neighbourhood, and must lose the circles inside 8 m of the exit buoy
    std::vector<Buoy> b = f.buoys;
    b[2] = mk(2, 88.0 - 6.0, -8.0, Beacon::FlashingGreen);
    const nav::Passage pa = nav::planPassage(b, f.entry, f.exitp);
    const std::vector<Hazard> fz = sideFences(b, pa, f.entry, f.exitp, P);
    double nearest = 1e18;
    for (const Hazard & h : fz) {
      nearest = std::min({nearest, nav::norm(h.c - f.entry), nav::norm(h.c - f.exitp)});
    }
    chk("every circle is >= fence_clear_m from both the entry and the exit", nearest >= P.fence_clear_m);
    chk("... and the ones that would have been inside are gone (fewer than 21)", countFor(fz, 2) < 21);
  }

  // -------------------------------------------- 2c. a gate's crossing line
  std::printf("2c. a gate's own fences stay off its straight crossing line\n");
  {
    // The crossing is approach (8 m short of the middle) -> through (6 m past it), driven
    // straight with the gate's two buoys exempt and EVERYTHING ELSE holding the boat. The
    // boat arrives within 3 m (NavigateTo tolerance) of the approach point, off the line.
    double worst = 1e18;
    int gates = 0;
    for (const double width : {4.0, 6.0, 9.0, 14.0}) {
      for (double rot = -50.0; rot <= 50.0; rot += 12.5) {
        const double h = rot * nav::kDeg;                                   // course, ccw from east
        const Vec2 u{std::cos(h), std::sin(h)}, right{u.y, -u.x};            // starboard of the course
        const Vec2 mid{40.0, 3.0};
        const std::vector<Buoy> field{mk(0, 0, 0, Beacon::FlashingBlue), mk(1, 120, 0, Beacon::SteadyBlue),
          mk(2, (mid + right * (width / 2)).x, (mid + right * (width / 2)).y, Beacon::FlashingRed),
          mk(3, (mid - right * (width / 2)).x, (mid - right * (width / 2)).y, Beacon::FlashingGreen)};
        const Vec2 entry{0, 0}, exitp{120, 0};
        const nav::Passage pa = nav::planPassage(field, entry, exitp);
        if (pa.gates.size() != 1) {continue;}
        ++gates;
        const std::vector<Hazard> fz = sideFences(field, pa, entry, exitp, P);
        // the leg exempts the gate's two BUOYS (path::exempt); the walls are not buoys, so they
        // stay in the set the guarded leg checks -- and the crossing has to clear them as is
        if (gates == 1) {
          std::vector<Hazard> all = buoyHazards(field, P);
          all.insert(all.end(), fz.begin(), fz.end());
          const std::vector<Hazard> left = exempt(all, {2, 3}, false, P.exempt_radius_m);
          chk("exempt=gate drops the gate's two buoys and keeps every fence circle",
            left.size() == all.size() - 2 && countFor(left, 2) == 21 && countFor(left, 3) == 21);
        }
        const nav::Gate g = nav::gateWaypoints(field[2].p, field[3].p, 6.0, 8.0);
        for (const double lateral : {-3.0, -1.5, 0.0, 1.5, 3.0}) {
          const Vec2 start = g.approach + right * lateral;
          detail::walkSegment(start, g.through, 0.1, [&](Vec2 q) {
              worst = std::min(worst, minClearance(fz, q));
              return true;
            });
        }
      }
    }
    chk("gates of 4..14 m, rotated +-50 deg, were tested", gates >= 30);
    std::printf("    (nearest any own-fence surface gets to the crossing: %.2f m; hard clearance %.2f m)\n",
      worst, P.hard_m);
    chk("the crossing keeps >= hard_m - local_check_tol_m from the gate's own fences, even "
      "3 m off the approach point", worst >= P.hard_m - P.local_check_tol_m);
  }

  // --------------------------------------------- 3. the planned legs work
  std::printf("3. the planned legs pass every red and green on its handbook side\n");
  scenario(unpairedField(), P);

  // The same field mirrored top to bottom with red and green swapped: starboard is now the
  // other way round for every buoy, so every wall must flip with it. A left/right slip in
  // sideFences passes one field and fails the other.
  std::printf("3b. the mirrored field (y -> -y, red <-> green)\n");
  {
    Field m = unpairedField();
    for (Buoy & b : m.buoys) {
      b.p.y = -b.p.y;
      if (b.state == Beacon::FlashingRed) {b.state = Beacon::FlashingGreen;}
      else if (b.state == Beacon::FlashingGreen) {b.state = Beacon::FlashingRed;}
    }
    m.entry.y = -m.entry.y;
    m.exitp.y = -m.exitp.y;
    scenario(m, P);
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  std::printf("%s\n", g_fails == 0 ? "PASS" : "FAIL");
  return g_fails == 0 ? 0 : 1;
}
