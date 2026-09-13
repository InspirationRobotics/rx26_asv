// test_nav_math — prove the Task 1 geometry with no ROS, no boat, no colcon.
//
//     g++ -std=c++17 -O2 -I include -o /tmp/t test/test_nav_math.cpp && /tmp/t
//
// Runs in under a second on a laptop with nothing installed, which is the point:
// every assertion here is a mistake that would otherwise be discovered on the
// water, once, by watching the boat pass a buoy on the wrong side.
//
// The side rule is the reason this file exists. handbook 3.3.2 says a FLASHING
// RED light is passed on the vehicle's STARBOARD side and GREEN on its PORT —
// which is inverted from what a sailor expects and means the WAYPOINT for a red
// buoy is offset to PORT. On a symmetric layout a mirrored implementation looks
// perfectly plausible on a plot, so it is asserted against hand-worked cases
// with the compass directions spelled out.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "crusader_bt/nav_math.hpp"

using namespace crusader_bt::nav;   // NOLINT(build/namespaces) — a test

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
  std::printf(
    "  [%s] %s%s", pass ? "ok" : "FAIL", name.c_str(),
    pass ? "\n" : "");
  if (!pass) {std::printf("   (got %.6f, want %.6f)\n", got, want);}
}

int main()
{
  // ------------------------------------------------------------- projection
  const LatLon origin{1.28060, 103.85570};        // Marina Bay, near enough
  {
    const LatLon p{1.28150, 103.85700};
    const Vec2 v = toLocal(p, origin);
    chk("east is +x", v.x > 0.0);
    chk("north is +y", v.y > 0.0);
    const LatLon back = toLatLon(v, origin);
    chk_near("round trip lat", back.lat, p.lat, 1e-9);
    chk_near("round trip lon", back.lon, p.lon, 1e-9);
    // 0.0009 deg of latitude is ~100 m.
    chk_near("100 m north reads as 100 m", toLocal({origin.lat + 0.0009, origin.lon}, origin).y,
      100.07, 0.5);
  }

  // ---------------------------------------------------------------- bearing
  chk_near("north is 0", bearingDeg({0, 0}, {0, 10}), 0.0, 1e-9);
  chk_near("east is 90", bearingDeg({0, 0}, {10, 0}), 90.0, 1e-9);
  chk_near("south is 180", bearingDeg({0, 0}, {0, -10}), 180.0, 1e-9);
  chk_near("west is 270", bearingDeg({0, 0}, {-10, 0}), 270.0, 1e-9);
  chk_near("headingVec(90) points east", headingVec(90.0).x, 1.0, 1e-9);
  chk_near("headingVec(180) points south", headingVec(180.0).y, -1.0, 1e-9);

  // ---------------------------------------------------- port and starboard
  {
    const Vec2 north{0, 1};
    chk("heading north, PORT is west", portOf(north).x < -0.99);
    chk("heading north, STARBOARD is east", starboardOf(north).x > 0.99);
    const Vec2 east{1, 0};
    chk("heading east, PORT is north", portOf(east).y > 0.99);
    chk("heading east, STARBOARD is south", starboardOf(east).y < -0.99);
  }

  // ------------------------------------------------------------- cross track
  {
    // Sailing north from the origin. A point to the east is to STARBOARD.
    chk("east of a northward track is starboard (negative)",
      crossTrack({0, 0}, {0, 10}, {5, 5}) < 0.0);
    chk("west of a northward track is port (positive)",
      crossTrack({0, 0}, {0, 10}, {-5, 5}) > 0.0);
  }

  // ------------------------------------------------ THE SIDE RULE, by hand
  {
    // Boat at the origin, sailing NORTH. A red buoy 20 m ahead.
    // The red light must end up on our STARBOARD (east) side, so we must pass
    // to its WEST — the waypoint is offset WEST, i.e. to port of travel.
    const Vec2 travel{0, 1};
    const Vec2 red{0, 20};
    const Vec2 wp_red = sideWaypoint(red, travel, Beacon::FlashingRed, 4.0);
    chk_near("RED waypoint is 4 m WEST of the buoy", wp_red.x, -4.0, 1e-9);
    chk_near("RED waypoint keeps the buoy's northing", wp_red.y, 20.0, 1e-9);

    // ...and sailing that waypoint really does leave the buoy to starboard.
    chk("sailing the RED waypoint leaves it to starboard",
      crossTrack({0, 0}, wp_red, red) < 0.0);

    // GREEN is the mirror: it must end on our PORT (west), so we pass EAST.
    const Vec2 green{0, 20};
    const Vec2 wp_grn = sideWaypoint(green, travel, Beacon::FlashingGreen, 4.0);
    chk_near("GREEN waypoint is 4 m EAST of the buoy", wp_grn.x, 4.0, 1e-9);
    chk("sailing the GREEN waypoint leaves it to port",
      crossTrack({0, 0}, wp_grn, green) > 0.0);

    // A buoy with no side constraint is steered at directly.
    const Vec2 off = sideWaypoint({7, 7}, travel, Beacon::Off, 4.0);
    chk("an OFF buoy carries no offset", off.x == 7.0 && off.y == 7.0);

    // The rule must hold on a non-axis-aligned heading too, or the test has
    // only proved that north works.
    const Vec2 ne = unit({1, 1});
    const Vec2 r2 = sideWaypoint({20, 20}, ne, Beacon::FlashingRed, 4.0);
    chk("RED holds on a NE heading", crossTrack({0, 0}, r2, {20, 20}) < 0.0);
    const Vec2 g2 = sideWaypoint({20, 20}, ne, Beacon::FlashingGreen, 4.0);
    chk("GREEN holds on a NE heading", crossTrack({0, 0}, g2, {20, 20}) > 0.0);
  }

  // ----------------------------------------------------- passedCorrectly
  {
    Buoy red{1, {5, 5}, Beacon::FlashingRed, false};
    chk("red to starboard scores", passedCorrectly({0, 0}, {0, 10}, red));
    Buoy red_w{2, {-5, 5}, Beacon::FlashingRed, false};
    chk("red to port does NOT score", !passedCorrectly({0, 0}, {0, 10}, red_w));
    Buoy grn{3, {-5, 5}, Beacon::FlashingGreen, false};
    chk("green to port scores", passedCorrectly({0, 0}, {0, 10}, grn));
    Buoy off{4, {5, 5}, Beacon::Off, false};
    chk("an OFF buoy always scores", passedCorrectly({0, 0}, {0, 10}, off));
  }

  // ------------------------------------------------------------------ orbit
  {
    const Vec2 anchor{0, 0};
    const Vec2 from{0, 6};                       // boat due north of the buoy
    const std::vector<Vec2> cw = orbit(anchor, from, 6.0, 5, true);
    chk("orbit produces the requested count", cw.size() == 5u);
    for (const Vec2 & w : cw) {
      chk_near("every waypoint is on the circle", norm(w - anchor), 6.0, 1e-9);
    }
    chk_near("CW sweep is a full -360", sweepDeg(anchor, cw, from), -360.0, 1e-6);
    chk_near("CW closes back on the start bearing",
      norm(cw.back() - from), 0.0, 1e-6);

    const std::vector<Vec2> ccw = orbit(anchor, from, 6.0, 5, false);
    chk_near("CCW sweep is a full +360", sweepDeg(anchor, ccw, from), 360.0, 1e-6);

    // Direction really differs: the first CW waypoint is east of north, the
    // first CCW one west. (Core Tier: ENTRY clockwise, EXIT counterclockwise.)
    chk("first CW waypoint goes east", cw.front().x > 0.0);
    chk("first CCW waypoint goes west", ccw.front().x < 0.0);

    // Degenerate: boat sitting exactly on the anchor must not produce NaN.
    const std::vector<Vec2> deg = orbit(anchor, anchor, 6.0, 5, true);
    chk("orbit from the anchor itself is finite",
      std::isfinite(deg.front().x) && std::isfinite(deg.front().y));
  }

  // ----------------------------------------------------------- nextWaypoint
  {
    const Vec2 boat{0, 0};
    const Vec2 exit_p{0, 100};
    std::vector<Buoy> field{
      {1, {0, 20}, Beacon::FlashingRed, false},
      {2, {0, 40}, Beacon::FlashingGreen, false},
      {3, {0, -30}, Beacon::FlashingRed, false},     // ASTERN
      {4, {10, 30}, Beacon::Off, false},             // no constraint
    };

    const double north = 0.0;    // boat pointing north
    NextWaypoint n = nextWaypoint(field, boat, north, true, exit_p, 4.0);
    chk("picks the nearest buoy AHEAD", n.ok && n.buoy_id == 1);
    chk("and offsets it to port for RED", n.wp.x < 0.0);

    field[0].consumed = true;
    n = nextWaypoint(field, boat, north, true, exit_p, 4.0);
    chk("consuming one advances to the next", n.ok && n.buoy_id == 2);
    chk("and offsets to starboard for GREEN", n.wp.x > 0.0);

    field[1].consumed = true;
    n = nextWaypoint(field, boat, north, true, exit_p, 4.0);
    chk("with nothing ahead it steers at the exit", n.ok && n.buoy_id == -1);
    chk_near("...at the exit itself", norm(n.wp - exit_p), 0.0, 1e-9);

    // The buoy astern is never selected, even though it is unconsumed and the
    // only side-constrained one left. Steering back to it is how a boat ends up
    // circling in the middle of the field.
    chk("the astern buoy is never chosen", n.buoy_id != 3);

    // No exit and nothing ahead -> the loop's secondary way out.
    // Heading north with the only buoy 30 m ASTERN. Before the heading was
    // passed in, travel was derived from the nearest buoy — which made that
    // buoy "ahead" by construction and turned the boat round.
    std::vector<Buoy> astern_only{{3, {0, -30}, Beacon::FlashingRed, false}};
    chk("no exit and the only buoy astern returns not-ok",
      !nextWaypoint(astern_only, boat, north, false, {}, 4.0).ok);
    chk("...but facing SOUTH it is ahead, and is taken",
      nextWaypoint(astern_only, boat, 180.0, false, {}, 4.0).ok);
    chk("a NaN heading with no exit steers nowhere",
      !nextWaypoint(astern_only, boat, std::nan(""), false, {}, 4.0).ok);

    // No exit but a buoy ahead: travel falls back to that buoy's bearing, and
    // it must still be offset to the correct side rather than steered at.
    std::vector<Buoy> ahead_only{{5, {0, 25}, Beacon::FlashingRed, false}};
    n = nextWaypoint(ahead_only, boat, north, false, {}, 4.0);
    chk("no exit still yields a side-offset waypoint", n.ok && n.buoy_id == 5);
    chk("...offset, not steered at", norm(n.wp - ahead_only[0].p) > 3.9);

    // An empty field is not a crash.
    chk("an empty field returns not-ok",
      !nextWaypoint({}, boat, north, false, {}, 4.0).ok);
  }

  // ------------------------------------------------------------------ gates
  //
  // The whole claim is: heading = bearingDeg(green -> red) - 90. Each case
  // below names the compass direction the boat should end up on, because a
  // mirrored implementation passes any symmetric case perfectly.
  {
    // red EAST of green -> the boat runs NORTH, red on its starboard hand.
    const Gate g = gateWaypoints({3, 0}, {-3, 0}, 6.0, 8.0);
    chk("gate red-E green-W is valid", g.valid);
    chk_near("... course is north", g.heading_deg, 0.0, 1e-9);
    chk_near("... approach is SHORT of the middle", g.approach.y, -8.0, 1e-9);
    chk_near("... through is PAST the middle", g.through.y, 6.0, 1e-9);
    chk_near("... both sit on the gate centreline", g.through.x, 0.0, 1e-9);
  }
  {
    // Mirrored. If the signs were wrong this is the case that catches it.
    const Gate g = gateWaypoints({-3, 0}, {3, 0}, 6.0, 8.0);
    chk_near("gate red-W green-E runs south", g.heading_deg, 180.0, 1e-9);
    chk_near("... approach is north of the middle", g.approach.y, 8.0, 1e-9);
  }
  chk_near("gate red-N green-S runs west",
    gateWaypoints({0, 3}, {0, -3}, 6.0, 8.0).heading_deg, 270.0, 1e-9);
  chk_near("gate red-S green-N runs east",
    gateWaypoints({0, -3}, {0, 3}, 6.0, 8.0).heading_deg, 90.0, 1e-9);
  chk_near("gate on the NE/SW diagonal runs NW",
    gateWaypoints({2, 2}, {-2, -2}, 6.0, 8.0).heading_deg, 315.0, 1e-9);
  chk_near("a skewed pair still resolves",
    gateWaypoints({5, 1}, {-1, 4}, 6.0, 8.0).heading_deg, 26.565, 1e-3);

  {
    // The property, stated independently of any hand-worked number: standing at
    // the gate centre on the computed course, RED is to starboard and GREEN to
    // port. This is what the boat actually has to get right.
    const Vec2 red{5, 1}, green{-1, 4};
    const Gate g = gateWaypoints(red, green, 6.0, 8.0);
    const Vec2 u = headingVec(g.heading_deg);
    const Vec2 mid = (red + green) * 0.5;
    chk("on course, RED is to starboard", dot(red - mid, starboardOf(u)) > 0.0);
    chk("on course, GREEN is to port", dot(green - mid, portOf(u)) > 0.0);
    chk("approach -> through points along the course",
      dot(g.through - g.approach, u) > 0.0);
  }

  // Degenerate pairs: every one of these must REFUSE, not steer somewhere
  // plausible. A gate is the one place a wrong bearing drives the boat between
  // the buoys backwards.
  {
    chk("a gate narrower than the minimum is refused",
      !gateWaypoints({0.5, 0}, {-0.5, 0}, 6.0, 8.0, 2.0).valid);
    chk("two buoys in the same place are refused",
      !gateWaypoints({4, 4}, {4, 4}, 6.0, 8.0).valid);
    const Gate g = gateWaypoints({0.5, 0}, {-0.5, 0}, 6.0, 8.0, 2.0);
    chk("... and says why", std::string(g.why).find("narrow") != std::string::npos);
  }

  // ------------------------------------------------------- gates, by UAV id
  {
    std::vector<Buoy> f{
      {0, {0, 20}, Beacon::FlashingBlue, false},
      {1, {6, 38}, Beacon::FlashingRed, false},
      {2, {-6, 38}, Beacon::FlashingGreen, false},
      {9, {0, 92}, Beacon::SteadyBlue, false}};

    const Gate g = gateFromIds(f, 1, 2, 6.0, 8.0);
    chk("gate from ids resolves", g.valid);
    chk_near("... and runs north up the field", g.heading_deg, 0.0, 1e-9);

    chk("findById finds a real id", findById(f, 9) != nullptr);
    chk("findById refuses NO_BUOY", findById(f, kNoBuoy) == nullptr);
    chk("findById refuses a negative id", findById(f, -1) == nullptr);

    chk("an unfilled red id (255) is refused",
      !gateFromIds(f, kNoBuoy, 2, 6.0, 8.0).valid);
    chk("an unfilled green id (255) is refused",
      !gateFromIds(f, 1, kNoBuoy, 6.0, 8.0).valid);
    chk("an id we have never seen is refused",
      !gateFromIds(f, 1, 7, 6.0, 8.0).valid);
    chk("the same id twice is refused",
      !gateFromIds(f, 1, 1, 6.0, 8.0).valid);

    // The pair the UAV sends when the passage is finished. It must read as
    // "no gate", never as a gate at the origin.
    chk("the 255/255 end-of-passage pair is not a gate",
      !gateFromIds(f, kNoBuoy, kNoBuoy, 6.0, 8.0).valid);
  }

  // ------------------------------------------------------------- findBeacon
  {
    std::vector<Buoy> f{
      {1, {0, 10}, Beacon::FlashingBlue, false},
      {2, {0, 90}, Beacon::SteadyBlue, false}};
    const Buoy * e = findBeacon(f, Beacon::FlashingBlue);
    const Buoy * x = findBeacon(f, Beacon::SteadyBlue);
    chk("entry is the FLASHING blue", e != nullptr && e->id == 1);
    chk("exit is the STEADY blue", x != nullptr && x->id == 2);
    chk("an absent state returns null", findBeacon(f, Beacon::Off) == nullptr);
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  std::printf("%s\n", g_fails == 0 ? "PASS" : "FAIL");
  return g_fails == 0 ? 0 : 1;
}
