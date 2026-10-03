// test_gate_waypoints — path::clearGateWaypoints, with no ROS, no Nav2, no boat.
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include -o /tmp/t test/test_gate_waypoints.cpp && /tmp/t
//
// The gate crossing is driven STRAIGHT with only the gate's own pair exempt, so a gate point
// put on top of another buoy, or a crossing line through one, holds the boat for good. The
// field below is the one that did it in the sim on 2026-10-02 (a compact custom layout, ENTRY
// south, EXIT north): gate (b1, b2)'s through point, 6 m past the middle, landed 1.4 m from
// green b4 with b4 ON the crossing line, and gate (b8, b4)'s approach point landed 1 m from red b1.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "crusader_bt/path_math.hpp"

using namespace crusader_bt;         // NOLINT(build/namespaces) -- a test
using namespace crusader_bt::path;   // NOLINT(build/namespaces)
using nav::Beacon;
using nav::Buoy;
using nav::Vec2;
using nav::dot;

static int g_fails = 0;
static int g_checks = 0;

static void chk(const std::string & name, bool pass)
{
  ++g_checks;
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s\n", pass ? "ok" : "FAIL", name.c_str());
}

static Buoy mk(int id, double x, double y, Beacon s) {return Buoy{id, {x, y}, s, false};}


/// The hazard set the crossing leg checks in nav_mode on: buoys + fences, the pair exempt.
static std::vector<Hazard> crossingHazards(
  const std::vector<Buoy> & field, Vec2 entry, Vec2 exitp, int red_id, int green_id,
  const NavParams & P)
{
  std::vector<Hazard> hz = buildHazards(field, {}, dock::DockBook{}, 0, P);
  const std::vector<Hazard> f = sideFences(field, nav::planPassage(field, entry, exitp), entry, exitp, P);
  hz.insert(hz.end(), f.begin(), f.end());
  return exempt(hz, {red_id, green_id}, false, P.exempt_radius_m);
}

/// What the crossing leg's guard needs: the straight line approach -> through clear by
/// hard_m - local_check_tol_m (planned_leg.hpp lineBlocked), and both points off every hazard.
static bool crossingDrivable(const nav::Gate & g, const std::vector<Hazard> & hz, const NavParams & P)
{
  return segmentClear(g.approach, g.through, hz, P.hard_m - P.local_check_tol_m) &&
         minClearance(hz, g.approach) >= P.hard_m && minClearance(hz, g.through) >= P.hard_m;
}

static void userField(const NavParams & P)
{
  std::printf("the 2026-10-02 custom field\n");
  const std::vector<Buoy> f = {
    mk(0, 11.0, 5.8, Beacon::FlashingBlue), mk(1, 27.9, 17.9, Beacon::FlashingRed),
    mk(2, 11.0, 13.6, Beacon::FlashingGreen), mk(3, 18.5, 32.4, Beacon::FlashingRed),
    mk(4, 18.3, 20.0, Beacon::FlashingGreen), mk(5, 8.4, 30.9, Beacon::SteadyBlue),
    mk(6, 18.3, 10.4, Beacon::Off), mk(7, 31.9, 24.8, Beacon::Off),
    mk(8, 23.7, 27.3, Beacon::FlashingRed), mk(9, 13.3, 27.3, Beacon::Off)};
  const Vec2 entry = f[0].p, exitp = f[5].p;
  const nav::Passage pa = nav::planPassage(f, entry, exitp);
  chk("two gates, (1,2) then (8,4)", pa.gates.size() == 2 &&
    pa.gates[0].red_id == 1 && pa.gates[0].green_id == 2 &&
    pa.gates[1].red_id == 8 && pa.gates[1].green_id == 4);
  if (pa.gates.size() != 2) {return;}

  for (const nav::PlannedGate & pg : pa.gates) {
    const Vec2 red = nav::findById(f, pg.red_id)->p, green = nav::findById(f, pg.green_id)->p;
    const std::vector<Hazard> hz = crossingHazards(f, entry, exitp, pg.red_id, pg.green_id, P);
    const nav::Gate plain = nav::gateWaypoints(red, green, 6.0, 8.0);
    double shift = 0.0;
    const nav::Gate g = clearGateWaypoints(red, green, 6.0, 8.0, hz, GateClear{}, &shift);
    const std::string tag = "gate (" + std::to_string(pg.red_id) + "," + std::to_string(pg.green_id) + ")";
    const Vec2 mid = (red + green) * 0.5, u = nav::headingVec(g.heading_deg), v = nav::unit(red - green);
    std::printf("    %s plain approach (%.1f, %.1f) through (%.1f, %.1f) -> approach (%.1f, %.1f) "
      "through (%.1f, %.1f), shift %+.1f m\n", tag.c_str(), plain.approach.x, plain.approach.y,
      plain.through.x, plain.through.y, g.approach.x, g.approach.y, g.through.x, g.through.y, shift);
    chk(tag + ": the plain crossing would hold the boat", !crossingDrivable(plain, hz, P));
    chk(tag + ": the moved crossing is drivable", crossingDrivable(g, hz, P));
    chk(tag + ": and keeps 1.5 m from everything", segmentClear(g.approach, g.through, hz, 1.5));
    chk(tag + ": through is >= 3.5 m past the gate line", dot(g.through - mid, u) >= 3.5 - 1e-9);
    chk(tag + ": approach is >= 4 m short of it", dot(mid - g.approach, u) >= 4.0 - 1e-9);
    const double along = dot(g.approach - mid, v);    // where the line meets the gate
    chk(tag + ": the crossing is inside the gap, 1.8 m off both buoys",
      std::fabs(along) <= 0.5 * nav::norm(red - green) - 1.8 + 1e-9);
    chk(tag + ": it stays perpendicular to the gate", std::fabs(dot(g.through - mid, v) - along) < 1e-6);
  }
}

static void clearField(const NavParams & P)
{
  std::printf("a clear gate is left exactly where it was\n");
  const Vec2 red{30.0, -4.5}, green{30.0, 4.5};
  const std::vector<Hazard> hz = {detail::circleHazard(HazardSource::PlanBuoy, 9, {60.0, 20.0}, P.buoy_radius_m)};
  const nav::Gate plain = nav::gateWaypoints(red, green, 6.0, 8.0);
  const nav::Gate g = clearGateWaypoints(red, green, 6.0, 8.0, hz);
  chk("same approach", nav::norm(g.approach - plain.approach) < 1e-9);
  chk("same through", nav::norm(g.through - plain.through) < 1e-9);
}

static void shorterBeforeSideways(const NavParams & P)
{
  std::printf("a buoy just short of the through point: back off, do not shift\n");
  const Vec2 red{30.0, -4.5}, green{30.0, 4.5};      // crossing east, through at (36, 0)
  const std::vector<Hazard> hz = {detail::circleHazard(HazardSource::PlanBuoy, 9, {37.0, 0.0}, P.buoy_radius_m)};
  double shift = 9.0;
  const nav::Gate g = clearGateWaypoints(red, green, 6.0, 8.0, hz, GateClear{}, &shift);
  chk("no shift", std::fabs(shift) < 1e-9);
  chk("through backed off to (35, 0): the 0.5 m step that keeps 1.5 m off the buoy",
    std::fabs(g.through.x - 35.0) < 1e-6 && std::fabs(g.through.y) < 1e-6);
}

static void nothingFits(const NavParams & P)
{
  std::printf("a narrow gate with a buoy in the middle of it: plain points, the guard holds\n");
  const Vec2 red{30.0, -1.5}, green{30.0, 1.5};      // 3 m wide: no room to shift
  const std::vector<Hazard> hz = {detail::circleHazard(HazardSource::PlanBuoy, 9, {33.0, 0.0}, P.buoy_radius_m)};
  const nav::Gate plain = nav::gateWaypoints(red, green, 6.0, 8.0);
  const nav::Gate g = clearGateWaypoints(red, green, 6.0, 8.0, hz);
  chk("plain through", nav::norm(g.through - plain.through) < 1e-9);
  chk("plain approach", nav::norm(g.approach - plain.approach) < 1e-9);
}

static void exitOrbitStart(const NavParams & P)
{
  std::printf("the goal rule: the EXIT orbit's first hop beside black b9 (same field, 2026-10-02)\n");
  // the boat's own positions from that run: EXIT, b9, and a second (LiDAR) track on b9
  const Vec2 exitp{8.60, 30.78}, boat{16.14, 26.92};
  const std::vector<Hazard> hz = {
    detail::circleHazard(HazardSource::PlanBuoy, 9, {13.73, 26.97}, P.buoy_radius_m),
    detail::circleHazard(HazardSource::Track, 7, {13.46, 27.14}, P.track_radius_m)};
  const std::vector<Vec2> ring = adjustRing(orbitRing(exitp, boat, 6.0, 8, false, 45.0), exitp, hz, P);
  std::printf("    ring[0] (%.2f, %.2f) clearance %.2f m\n", ring[0].x, ring[0].y, minClearance(hz, ring[0]));
  chk("ring[0] lands inside the orbit clearance (adjustRing never moves it)",
    minClearance(hz, ring[0]) < P.orbit_clear_m);
  const Vec2 q = clearPoint(ring[0], hz, P.orbit_clear_m);
  std::printf("    moved to (%.2f, %.2f), %.2f m, clearance %.2f m\n", q.x, q.y, nav::norm(q - ring[0]),
    minClearance(hz, q));
  chk("clearPoint moves it out to the orbit clearance", minClearance(hz, q) >= P.orbit_clear_m);
  chk("... by no more than it has to (<= 3 m)", nav::norm(q - ring[0]) <= 3.0 + 1e-9);
  chk("... and that is >= 1.7 m from both buoy centres",
    nav::norm(q - Vec2{13.73, 26.97}) >= 1.7 && nav::norm(q - Vec2{13.46, 27.14}) >= 1.7);
  chk("a clear point is not moved", nav::norm(clearPoint({0.0, 0.0}, hz, 1.4) - Vec2{0.0, 0.0}) < 1e-12);
}

int main()
{
  const NavParams P;
  exitOrbitStart(P);
  userField(P);
  clearField(P);
  shorterBeforeSideways(P);
  nothingFits(P);
  std::printf("%d/%d checks passed\n", g_checks - g_fails, g_checks);
  return g_fails == 0 ? 0 : 1;
}
