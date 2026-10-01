// test_path_math — prove the avoidance geometry with no ROS, no Nav2, no boat.
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include -o /tmp/t test/test_path_math.cpp && /tmp/t
//
// Same job as test_nav_math: every assertion is a mistake that would otherwise be
// found on the water, once. Here that mistake is a boat that thinks a clear line is
// blocked (and holds forever), or a blocked one is clear (and drives into a buoy).
// The numbers follow docs/nav2_avoidance_spec.md section 5.9, "test_path_math.cpp";
// the section numbers below are that list's.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "crusader_bt/path_math.hpp"

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
  if (!pass) {std::printf("   (got %.9f, want %.9f)\n", got, want);}
}

static Hazard circle(int id, double x, double y, double r, double keepout = 0.0,
  HazardSource src = HazardSource::PlanBuoy)
{
  Hazard h;
  h.kind = HazardKind::Circle;
  h.source = src;
  h.id = id;
  h.c = {x, y};
  h.r = r;
  h.keepout = keepout;
  return h;
}

static Hazard square(double half, double keepout = 0.0, bool ccw = true)
{
  Hazard h;
  h.kind = HazardKind::Polygon;
  h.poly = {{-half, -half}, {half, -half}, {half, half}, {-half, half}};
  if (!ccw) {std::reverse(h.poly.begin(), h.poly.end());}
  h.keepout = keepout;
  return h;
}

/// A dense straight path along +x with 0.1 m vertices, like Smac's.
static std::vector<Vec2> line(double x0, double x1)
{
  std::vector<Vec2> p;
  for (double x = x0; x < x1 + 1e-9; x += 0.1) {p.push_back({x, 0.0});}
  return p;
}

static bool near2c(Vec2 a, Vec2 b) {return nav::norm(a - b) < 1e-9;}

static double signedArea(const std::vector<Vec2> & q)
{
  double a = 0.0;
  for (std::size_t i = 0; i < q.size(); ++i) {a += nav::cross(q[i], q[(i + 1) % q.size()]);}
  return 0.5 * a;
}

int main()
{
  const NavParams P;      // the defaults ARE the yaml's, so the tests pin the defaults

  // ----------------------------------------------- 1. clearance: signs, keepout
  std::printf("1. clearance\n");
  {
    const Hazard c = circle(1, 0, 0, 0.3);
    chk_near("circle: outside is distance to the SURFACE", clearance(c, {2, 0}), 1.7, 1e-12);
    chk("circle: inside is negative", clearance(c, {0.2, 0}) < 0.0);
    chk_near("circle: the surface itself is zero", clearance(c, {0.3, 0}), 0.0, 1e-12);
    const Hazard k = circle(1, 0, 0, 0.3, 0.5);
    chk_near("circle keepout pads the lethal boundary", clearance(k, {2, 0}), 1.2, 1e-12);
    chk("... and the keepout band itself is inside (negative)", clearance(k, {0.7, 0}) < 0.0);

    const Hazard sq = square(1.0);
    chk_near("polygon: outside, nearest an edge", clearance(sq, {3, 0}), 2.0, 1e-12);
    chk_near("polygon: outside, nearest a corner", clearance(sq, {2, 2}), std::sqrt(2.0), 1e-12);
    chk_near("polygon: inside is minus the distance to the nearest edge",
      clearance(sq, {0.25, 0}), -0.75, 1e-12);
    chk_near("polygon: centre of a 2 m square", clearance(sq, {0, 0}), -1.0, 1e-12);
    chk_near("polygon keepout pads it too", clearance(square(1.0, 0.25), {3, 0}), 1.75, 1e-12);
    chk_near("a CLOCKWISE polygon gives the same answer (winding is not trusted)",
      clearance(square(1.0, 0.0, false), {0.25, 0}), -0.75, 1e-12);

    chk("minClearance of nothing is +inf", std::isinf(minClearance({}, {1, 1})));
    chk_near("minClearance picks the nearest of several",
      minClearance({circle(1, 10, 0, 0.3), circle(2, 3, 0, 0.3)}, {0, 0}), 2.7, 1e-12);
  }

  // --------------------------------------------------------- 2. buildHazards
  std::printf("2. buildHazards\n");
  {
    std::vector<nav::Buoy> buoys{
      {3, {10, 0}, nav::Beacon::FlashingRed, false},
      {4, {10, 6}, nav::Beacon::FlashingGreen, true},          // consumed: STILL there
      {5, {std::nan(""), 0}, nav::Beacon::Off, false}};        // nobody can place it
    std::vector<nav::Buoy> tracks{{9, {-5, 5}, nav::Beacon::Unknown, false}};

    dock::DockBook book;
    dock::BayTrack good;      // confirmed, outward measured: faces NORTH
    good.id = 7; good.p = {20, 0}; good.n = 6; good.normal_sum = {0, 1}; good.n_normal = 1;
    dock::BayTrack thin = good;       // too few sightings
    thin.id = 8; thin.p = {30, 0}; thin.n = 2;
    dock::BayTrack blind = good;      // no normal and no view: facing is a guess
    blind.id = 9; blind.p = {40, 0}; blind.normal_sum = {}; blind.n_normal = 0;
    book.tracks = {good, thin, blind};

    const std::vector<Hazard> hz = buildHazards(buoys, tracks, book, 5, P);
    int nb = 0, nt = 0, nd = 0;
    for (const Hazard & h : hz) {
      nb += h.source == HazardSource::PlanBuoy;
      nt += h.source == HazardSource::Track;
      nd += h.source == HazardSource::Dock;
    }
    chk("counts: 2 placed plan buoys, 1 track, 3 dock rectangles (one usable bay)",
      nb == 2 && nt == 1 && nd == 3 && hz.size() == 6);
    chk("a consumed buoy is NOT dropped", hz.size() > 1 && hz[1].id == 4);
    chk("plan buoy: position, id and radius", hz[0].id == 3 && hz[0].c.x == 10 && hz[0].c.y == 0 &&
      hz[0].r == P.buoy_radius_m && hz[0].kind == HazardKind::Circle);
    chk("track: its own source, id and radius", hz[2].source == HazardSource::Track &&
      hz[2].id == 9 && hz[2].c.x == -5 && hz[2].r == P.track_radius_m);
    chk("no hazard at (0,0) for the unplaceable buoy", [&] {
        for (const Hazard & h : hz) {if (h.id == 5) {return false;}}
        return true;
      }());

    // The bay faces north, so right = starboardOf(south) = WEST: the fingers sit
    // at x = 20 -/+ 1.0 (== +/-(slip/2 + finger_w/2)), 0..2 m north of the face.
    const std::vector<Hazard> d = dockHazards(book, 5, P);
    chk("dockHazards: one bay -> 2 fingers + 1 deck", d.size() == 3);
    bool all_ccw = true;
    for (const Hazard & h : d) {all_ccw = all_ccw && signedArea(h.poly) > 0.0;}
    chk("every dock polygon is counter-clockwise (Hazard.msg)", all_ccw);
    chk("the dock hazards carry the bay's track id", d[0].id == 7 && d[2].id == 7);
    chk_near("finger 1: centre of its 0.5 m x 2 m rectangle is 1.0 m to one side",
      clearance(d[0], {21.0, 1.0}), -0.25, 1e-9);
    chk_near("finger 2: the mirror, 1.0 m to the other", clearance(d[1], {19.0, 1.0}), -0.25, 1e-9);
    chk_near("fingers are 2.0 m long (0..2 out from the face)", clearance(d[1], {19.0, 2.5}), 0.5, 1e-9);
    chk_near("... and start AT the face", clearance(d[1], {19.0, -0.5}), 0.5, 1e-9);
    chk("the slip between the fingers is clear", clearance(d[0], {20.0, 1.0}) > 0.0 &&
      clearance(d[1], {20.0, 1.0}) > 0.0);
    chk_near("deck: 1.0 m deep behind the face, +/-1.25 m wide", clearance(d[2], {20.0, -0.5}), -0.5, 1e-9);
    chk_near("... and ends at the face", clearance(d[2], {20.0, 0.5}), 0.5, 1e-9);
    chk_near("... and at 1.25 m to the side", clearance(d[2], {21.75, -0.5}), 0.5, 1e-9);
  }

  // ---------------------------------------------------------------- 3. exempt
  std::printf("3. exempt\n");
  {
    std::vector<Hazard> hz{
      circle(1, 0, 0, 0.3),                                           // gate red
      circle(2, 2, 0, 0.3),                                           // gate green
      circle(3, 1, 10, 0.3),                                          // somebody else's
      circle(50, 0.6, 0, 0.3, 0.0, HazardSource::Track),              // 0.6 m from buoy 1
      circle(51, 0, 1.5, 0.3, 0.0, HazardSource::Track),              // 1.5 m from buoy 1
      circle(52, 30, 30, 0.3, 0.0, HazardSource::Track)};
    Hazard dk = square(1.0);
    dk.source = HazardSource::Dock;
    dk.id = 7;
    hz.push_back(dk);

    const auto ids = [](const std::vector<Hazard> & v) {
        std::vector<int> o;
        for (const Hazard & h : v) {o.push_back(h.id);}
        return o;
      };
    const std::vector<int> gate = ids(exempt(hz, {1, 2}, false, 1.0));
    chk("the listed plan buoys are removed, the others stay",
      gate == std::vector<int>({3, 51, 52, 7}));
    chk("a track 0.6 m from an exempt buoy is dropped (50 gone)", std::find(gate.begin(), gate.end(), 50) == gate.end());
    chk("a track 1.5 m away is kept (51 stays)", std::find(gate.begin(), gate.end(), 51) != gate.end());
    chk("the dock stays unless exempt_dock", std::find(gate.begin(), gate.end(), 7) != gate.end());
    chk("exempt_dock removes every dock hazard",
      ids(exempt(hz, {}, true, 1.0)) == std::vector<int>({1, 2, 3, 50, 51, 52}));
    chk("nothing listed, nothing removed", exempt(hz, {}, false, 1.0).size() == hz.size());
    chk("an id that is not in the list exempts nothing around it",
      ids(exempt(hz, {99}, false, 1.0)).size() == hz.size());
  }

  // -------------------------------------------- 4. segmentClear, firstConflict
  std::printf("4. segmentClear and firstConflict\n");
  {
    const std::vector<Hazard> hz{circle(1, 5, 0, 0.3)};
    chk("a segment through a buoy is blocked", !segmentClear({0, 0}, {10, 0}, hz, 0.7));
    chk("a segment 0.6 m off the surface is blocked at 0.7", !segmentClear({0, 0.9}, {10, 0.9}, hz, 0.7));
    chk("... and clear 0.8 m off the surface", segmentClear({0, 1.1}, {10, 1.1}, hz, 0.7));
    chk("a far segment is clear", segmentClear({0, 3}, {10, 3}, hz, 0.7));
    chk("the END of a segment is sampled (goal inside the zone)", !segmentClear({0, 3}, {5, 0.2}, hz, 0.7));
    chk("a zero-length segment in clear water is clear", segmentClear({0, 3}, {0, 3}, hz, 0.7));
    chk("a zero-length segment in a hazard is not", !segmentClear({5, 0}, {5, 0}, hz, 0.7));
    chk("a NaN end is NOT clear", !segmentClear({0, 3}, {std::nan(""), 3}, hz, 0.7));
    chk("no hazards: always clear", segmentClear({0, 0}, {100, 100}, {}, 5.0));

    const std::vector<Vec2> path{{0, 0}, {2, 0}, {4, 0}, {6, 0}};
    chk("firstConflict: a hazard on the last vertex", firstConflict(path, 0, {circle(1, 6, 0, 0.3)}, 0.7) == 3);
    chk("... and from_i past it finds nothing", firstConflict(path, 4, {circle(1, 6, 0, 0.3)}, 0.7) == -1);
    chk("... and from_i ON it finds it", firstConflict(path, 3, {circle(1, 6, 0, 0.3)}, 0.7) == 3);
    chk("firstConflict: within clearance_m of a vertex counts",
      firstConflict(path, 0, {circle(1, 4.5, 0, 0.3)}, 0.7) == 2);
    chk("firstConflict: a clear path is -1", firstConflict(path, 0, {circle(1, 3, 5, 0.3)}, 0.7) == -1);
    chk_near("pathLength", pathLength(path), 6.0, 1e-12);
    chk_near("pathLength from an index", pathLength(path, 2), 2.0, 1e-12);
    chk_near("pathLength of one point", pathLength({{1, 1}}), 0.0, 1e-12);
    const std::vector<Vec2> l = line(0, 20);
    chk("closestIndex finds the nearest vertex", closestIndex(l, {5.04, 1.0}, 0) == 50);
    chk("closestIndex does not go back more than 2 m of arc from the hint",
      closestIndex(l, {1.0, 0.0}, 100) == 80);
    chk("... but does search forward past the hint", closestIndex(l, {15.0, 0.0}, 100) == 150);
    chk("closestIndex of an empty path is 0", closestIndex({}, {1, 1}, 5) == 0);
  }

  // ------------------------------------------------------------ 5. pushGoalOut
  std::printf("5. pushGoalOut\n");
  {
    const std::vector<Hazard> hz{circle(1, 50, 0, 0.3)};
    const double need = P.hard_m + P.goal_margin_m;
    const Moved m = pushGoalOut({50, 0}, {0, 0}, hz, P.hard_m, P.goal_margin_m);
    chk("a goal on a buoy centre is moved, ok", m.moved && m.ok);
    chk("... to at least hard + margin from the SURFACE", clearance(hz[0], m.p) >= need - 1e-12);
    chk("... but not far past it (0.1 m steps)", clearance(hz[0], m.p) < need + 0.11);
    chk_near("... on the line toward `from` (y stays 0)", m.p.y, 0.0, 1e-12);
    chk("... on the `from` side of the buoy", m.p.x < 50.0);
    const Moved free = pushGoalOut({50, 20}, {0, 0}, hz, P.hard_m, P.goal_margin_m);
    chk("a free goal is unchanged", !free.moved && free.ok && free.p.x == 50 && free.p.y == 20);
    const Moved blocked = pushGoalOut({50, 0}, {50.5, 0}, hz, P.hard_m, P.goal_margin_m);
    chk("a fully blocked line gives ok=false", !blocked.ok && !blocked.moved);
    chk("... and returns the goal unchanged", blocked.p.x == 50 && blocked.p.y == 0);
    const Moved edge = pushGoalOut({50, 0}, {51.3, 0}, hz, P.hard_m, P.goal_margin_m);
    chk("a `from` 1.3 m away is itself inside the zone (1.4 m needed): ok = false", !edge.ok);
    const Moved far_from = pushGoalOut({50, 0}, {51.45, 0}, hz, P.hard_m, P.goal_margin_m);
    chk("... while a clear `from` 1.45 m away ends the walk exactly there", far_from.ok && far_from.moved &&
      near2c(far_from.p, {51.45, 0}));
  }

  // ------------------------------------------------------------ 6. clipToWindow
  std::printf("6. clipToWindow\n");
  {
    const Vec2 a = clipToWindow({100, 0}, {0, 0}, 35.0);
    chk_near("a 100 m goal is clipped to 35 m out", a.x, 35.0, 1e-12);
    chk_near("... on the same line", a.y, 0.0, 1e-12);
    const Vec2 b = clipToWindow({20, 0}, {0, 0}, 35.0);
    chk("a goal inside the window is untouched", b.x == 20 && b.y == 0);
    const Vec2 c = clipToWindow({10, 110}, {10, 10}, 35.0);
    chk_near("the clip is measured from the BOAT, not the origin", c.y, 45.0, 1e-12);
    chk("a non-positive radius disables the clip",
      clipToWindow({100, 0}, {0, 0}, 0.0).x == 100);
    const Vec2 d = clipToWindow({35.0, 0}, {0, 0}, 35.0);
    chk("exactly on the radius is inside", d.x == 35.0);
  }

  // ------------------------------------------------------------- 7. escapeStart
  std::printf("7. escapeStart\n");
  {
    // A boat 0.5 m off a buoy's SURFACE (0.8 m from its centre: inside hard = 0.8).
    const std::vector<Hazard> one{circle(1, 0, 0, 0.3)};
    const Vec2 boat{0.8, 0};
    const Escape e = escapeStart(boat, {20, 0}, one, P);
    chk("0.5 m off a buoy: an escape is needed and found", e.needed && e.ok);
    chk("... at least 2.5 m from the boat", nav::norm(e.p - boat) >= 2.5 - 1e-9);
    chk("... and clear by hard + escape_margin", minClearance(one, e.p) >= P.hard_m + P.escape_margin_m - 1e-9);
    bool approaches = false;
    for (int k = 0; k <= 100; ++k) {
      const Vec2 q = boat + (e.p - boat) * (k / 100.0);
      approaches = approaches || clearance(one[0], q) < 0.5 - 0.05 - 1e-9;
    }
    chk("... and never gets closer to the buoy on the way out", !approaches);
    chk("... and heads for the goal, not back through the buoy", e.p.x > boat.x);

    const Escape fine = escapeStart({5, 0}, {20, 0}, one, P);
    chk("a boat in the clear needs no escape", !fine.needed && !fine.ok);
    chk("no hazards at all: no escape", !escapeStart({0, 0}, {20, 0}, {}, P).needed);
    chk("a NaN boat needs no escape (and invents none)",
      !escapeStart({std::nan(""), 0}, {20, 0}, one, P).needed);

    // The middle of a 2 m gate: both buoys 1.0 m away = 0.7 m of clearance < 0.8.
    const std::vector<Hazard> gate{circle(1, -1, 0, 0.3), circle(2, 1, 0, 0.3)};
    const Escape g = escapeStart({0, 0}, {0, 30}, gate, P);
    chk("mid-gate: an escape is needed and found", g.needed && g.ok);
    chk_near("... along the gate AXIS (x stays 0), not through a buoy", g.p.x, 0.0, 1e-9);
    chk("... toward the goal", g.p.y > 2.5 - 1e-9);
    const Escape g2 = escapeStart({0, 0}, {0, -30}, gate, P);
    chk("... and the other way when the goal is that way", g2.ok && g2.p.y < -2.5 + 1e-9);

    // Four buoys 1.0 m from the boat on the axes: every way out brushes one.
    const std::vector<Hazard> box{circle(1, -1, 0, 0.3), circle(2, 1, 0, 0.3),
      circle(3, 0, 1, 0.3), circle(4, 0, -1, 0.3)};
    const Escape bx = escapeStart({0, 0}, {20, 0}, box, P);
    chk("boxed in by four buoys: needed, and ok = false", bx.needed && !bx.ok);
  }

  // ---------------------------------------------------------------- 8. carrot
  std::printf("8. carrot\n");
  {
    const std::vector<Vec2> l = line(0, 30);
    const Carrot c = carrot(l, {0, 0}, 0, {}, P);
    chk_near("a straight path: the carrot is 5 m ahead", c.p.x, 5.0, 1e-6);
    chk("... not the end", !c.is_end);
    chk("... with idx at the first vertex beyond it", l[c.idx].x >= c.p.x - 1e-9 && l[c.idx - 1].x < c.p.x);
    const Carrot s = carrot({{0, 0}, {30, 0}}, {0, 0}, 0, {}, P);
    chk_near("a SPARSE path (two vertices) still gets a carrot 5 m out", s.p.x, 5.0, 1e-6);
    const Carrot adv = carrot(l, {12.34, 0.1}, 100, {}, P);
    chk_near("the boat's PROJECTION is the arc origin (12.34 + 5)", adv.p.x, 17.34, 1e-6);
    const Carrot mid = carrot({{0, 0}, {30, 0}}, {20, 0}, 1, {}, P);
    chk_near("a sparse path with the boat nearer its far vertex still works", mid.p.x, 25.0, 1e-6);

    // Half a circle round a buoy at (10,0): 1.3 m out (clearance 1.0 m), dense.
    std::vector<Vec2> arc = line(0, 8.7);
    const double step = 0.1 / 1.3;
    for (double a = nav::kPi - step; a > 0.0; a -= step) {
      arc.push_back({10 + 1.3 * std::cos(a), 1.3 * std::sin(a)});
    }
    for (const Vec2 & q : line(11.3, 30)) {arc.push_back(q);}
    const std::vector<Hazard> buoy{circle(1, 10, 0, 0.3)};
    const Carrot ac = carrot(arc, {6, 0}, 60, buoy, P);
    chk("round a buoy: the carrot is SHORTER than the 5 m lookahead",
      ac.idx < 60 + 50 - 1 && !ac.is_end);
    chk("... but at least the 3 m minimum", ac.idx >= 60 + 30 - 1);
    chk("... and its chord is clear at hard - tol", segmentClear({6, 0}, ac.p, buoy, P.hard_m - P.local_check_tol_m));
    bool hugs = true;
    for (std::size_t i = 61; i + 1 < ac.idx; ++i) {
      hugs = hugs && detail::distToSegment(arc[i], {6, 0}, ac.p) <= P.max_chord_dev_m + 1e-9;
    }
    chk("... and every vertex it skips is within max_chord_dev of the chord", hugs);

    const Carrot e1 = carrot(l, {28, 0}, 280, {}, P);
    chk("less than 3 m left: the end of the path, is_end", e1.is_end && e1.p.x == l.back().x && e1.idx == l.size() - 1);
    const Carrot e2 = carrot(l, {26.5, 0}, 265, {}, P);
    chk("3.5 m left: the carrot reaches the end of the path, so is_end too", e2.is_end && e2.p.x == l.back().x);
    const Carrot e3 = carrot(l, {20, 0}, 200, {}, P);
    chk("10 m left: not the end", !e3.is_end);
    chk("an empty path is a carrot ON THE BOAT, not the origin",
      carrot({}, {3, 4}, 0, {}, P).p.x == 3 && carrot({}, {3, 4}, 0, {}, P).p.y == 4);
    chk("one vertex: that vertex, is_end", carrot({{7, 7}}, {0, 0}, 0, {}, P).is_end);
  }

  // ---------------------------------------------------------------- 9. preferNew
  std::printf("9. preferNew\n");
  {
    chk("10 % shorter is kept (not switched)", !preferNew(true, 100, 90, 0.2, 3.0));
    chk("30 % and 5 m shorter is adopted", preferNew(true, 100, 70, 0.2, 3.0));
    chk("25 % shorter but only 2.5 m: BOTH are needed, so kept", !preferNew(true, 10, 7.5, 0.2, 3.0));
    chk("5 m shorter but only 5 %: kept", !preferNew(true, 100, 95, 0.2, 3.0));
    chk("an invalid current path is always replaced", preferNew(false, 100, 100, 0.2, 3.0));
    chk("... even by a LONGER path", preferNew(false, 100, 500, 0.2, 3.0));
    chk("a longer path never replaces a valid one", !preferNew(true, 100, 120, 0.2, 3.0));
  }

  // --------------------------------------------------------------- 10. orbitRing
  std::printf("10. orbitRing\n");
  {
    const Vec2 anchor{0, 0};
    const std::vector<Vec2> cw = orbitRing(anchor, {0, 20}, 4.0, 8, true);
    chk("n = 8: nine points", cw.size() == 9);
    chk("ring[0] == ring[8], exactly", cw[0].x == cw[8].x && cw[0].y == cw[8].y);
    chk_near("ring[0] is on the anchor->from bearing at the radius (x)", cw[0].x, 0.0, 1e-9);
    chk_near("ring[0] ... (y)", cw[0].y, 4.0, 1e-9);
    bool on_circle = true;
    for (const Vec2 & q : cw) {on_circle = on_circle && std::fabs(nav::norm(q - anchor) - 4.0) < 1e-9;}
    chk("every point is on the circle", on_circle);
    const std::vector<Vec2> tail_cw(cw.begin() + 1, cw.end());
    chk_near("cw: the sweep from ring[0] over ring[1..8] is -360", nav::sweepDeg(anchor, tail_cw, cw[0]), -360.0, 1e-6);
    chk("cw from the north goes EAST first (clockwise on a north-up chart)", cw[1].x > 0.0);
    const std::vector<Vec2> ccw = orbitRing(anchor, {0, 20}, 4.0, 8, false);
    const std::vector<Vec2> tail_ccw(ccw.begin() + 1, ccw.end());
    chk_near("ccw: the sweep is +360", nav::sweepDeg(anchor, tail_ccw, ccw[0]), 360.0, 1e-6);
    chk("ccw from the north goes WEST first", ccw[1].x < 0.0);

    // THE FAR-START CASE: the boat is 60 m out. The ring still starts ON the
    // bearing at the orbit radius, so the sweep from it is a full turn.
    const std::vector<Vec2> far = orbitRing(anchor, {60, 0}, 4.0, 8, true);
    const std::vector<Vec2> tail_far(far.begin() + 1, far.end());
    chk_near("far start (60,0): ring[0] is at the orbit radius, not at the boat", nav::norm(far[0]), 4.0, 1e-9);
    chk_near("far start: the sweep is still -360", nav::sweepDeg(anchor, tail_far, far[0]), -360.0, 1e-6);
    const std::vector<Vec2> far_ccw = orbitRing(anchor, {60, 0}, 4.0, 8, false);
    const std::vector<Vec2> tail_far_ccw(far_ccw.begin() + 1, far_ccw.end());
    chk_near("far start, ccw: +360", nav::sweepDeg(anchor, tail_far_ccw, far_ccw[0]), 360.0, 1e-6);
    chk("n < 1 gives an empty ring", orbitRing(anchor, {0, 20}, 4.0, 0, true).empty());
    chk("a boat ON the anchor starts at bearing 0 (east), not NaN",
      std::isfinite(orbitRing(anchor, anchor, 4.0, 8, true)[0].x) &&
      orbitRing(anchor, anchor, 4.0, 8, true)[0].x > 3.9);
  }

  // -------------------------------------------------------------- 11. adjustRing
  std::printf("11. adjustRing\n");
  {
    const Vec2 anchor{0, 0};
    const std::vector<Vec2> ring = orbitRing(anchor, {0, 20}, 4.0, 8, true);
    int dropped = -1;
    const std::vector<Hazard> clear_field{circle(1, 40, 40, 0.3)};
    const std::vector<Vec2> same = adjustRing(ring, anchor, clear_field, P, &dropped);
    chk("a clear ring is returned as it was", same.size() == ring.size() && dropped == 0);

    // A buoy sitting ON ring[3]: that point is pushed radially out until it clears 1.4 m.
    const std::vector<Hazard> on3{circle(1, ring[3].x, ring[3].y, 0.3)};
    const std::vector<Vec2> adj = adjustRing(ring, anchor, on3, P, &dropped);
    chk("a buoy on the ring pushes THAT point out, none dropped", adj.size() == ring.size() && dropped == 0);
    chk("... to a clearance of at least 1.4 m", minClearance(on3, adj[3]) >= P.orbit_clear_m - 1e-9);
    chk("... radially (still on the anchor->point ray)", std::fabs(nav::cross(adj[3], ring[3])) < 1e-9 &&
      nav::norm(adj[3]) > nav::norm(ring[3]));
    chk("... by whole 0.25 m steps", std::fabs((nav::norm(adj[3]) - 4.0) / 0.25 -
      std::round((nav::norm(adj[3]) - 4.0) / 0.25)) < 1e-9);
    bool others_same = true;
    for (std::size_t i = 0; i < ring.size(); ++i) {
      if (i != 3 && (adj[i].x != ring[i].x || adj[i].y != ring[i].y)) {others_same = false;}
    }
    chk("... and no other point moves", others_same);

    // An unfixable field: a hazard 8 m across that no 3 m push can leave.
    const std::vector<Hazard> wall{circle(1, ring[3].x, ring[3].y, 8.0)};
    const std::vector<Vec2> gone = adjustRing(ring, anchor, wall, P, &dropped);
    chk("an unfixable point is dropped, and counted", dropped >= 1 &&
      gone.size() + static_cast<std::size_t>(dropped) == ring.size());
    chk("the endpoints are kept, whatever their clearance",
      gone.front().x == ring.front().x && gone.back().y == ring.back().y &&
      minClearance(wall, ring.front()) < 1.4);
    chk("dropped may be omitted", adjustRing(ring, anchor, wall, P).size() == gone.size());
  }

  // ------------------------------------------------------ 12. projection parity
  std::printf("12. projection parity (spec 3.2; WP3's Python test pins the same literal)\n");
  {
    const nav::LatLon datum{1.2806, 103.8557};
    const Vec2 v = nav::toLocal({datum.lat + 0.0009, datum.lon + 0.0009}, datum);
    chk_near("x = 100.050439", v.x, 100.050439, 1e-6);
    chk_near("y = 100.075434", v.y, 100.075434, 1e-6);
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  std::printf("%s\n", g_fails == 0 ? "PASS" : "FAIL");
  return g_fails == 0 ? 0 : 1;
}
