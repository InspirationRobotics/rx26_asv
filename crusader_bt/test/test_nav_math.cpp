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

  // ----------------------------------------------------------------- fusion
  {
    // A plan the aircraft sent, and what the boat's own tracker made of the
    // same water. The tracker is a metre or so off, has NOT seen the far buoy
    // at all, and has invented a contact the plan knows nothing about.
    std::vector<PlanBuoy> plan{
      {0, {0, 20}, Beacon::FlashingBlue},
      {1, {6, 38}, Beacon::FlashingRed},
      {2, {-6, 38}, Beacon::FlashingGreen},
      {9, {0, 92}, Beacon::SteadyBlue}};
    std::vector<Buoy> tracked{
      {100, {0.4, 20.3}, Beacon::FlashingBlue, false},
      {101, {6.2, 37.6}, Beacon::FlashingGreen, false},   // WRONG colour
      {102, {-5.7, 38.4}, Beacon::FlashingGreen, false},
      {103, {30, 12}, Beacon::Off, false}};               // not in the plan

    const Fused f = fusePassage(plan, tracked, 3.0);

    chk("every plan buoy survives fusion", f.passage.size() == 4);
    chk("an unmatched contact becomes an obstacle", f.obstacles.size() == 1);
    chk("... and it is the one the plan never mentioned",
      f.obstacles.size() == 1 && f.obstacles[0].id == 103);
    chk("an obstacle carries NO beacon state",
      f.obstacles.size() == 1 && f.obstacles[0].state == Beacon::Unknown);

    const Buoy * red = findById(f.passage, 1);
    chk("ids stay the UAV's, not the tracker's", red != nullptr);
    // The whole point of "the UAV wins": the tracker called this one GREEN.
    chk("THE UAV WINS on a colour disagreement",
      red != nullptr && red->state == Beacon::FlashingRed);
    chk("... and the tracker's better position is kept",
      red != nullptr && std::fabs(red->p.x - 6.2) < 1e-9 &&
      std::fabs(red->p.y - 37.6) < 1e-9);

    const Buoy * far = findById(f.passage, 9);
    chk("a buoy the tracker never saw keeps the plan's position",
      far != nullptr && std::fabs(far->p.y - 92.0) < 1e-9);

    // Association must be mutually exclusive. Two plan buoys 1 m apart with a
    // single contact between them: per-buoy nearest would give BOTH the same
    // point, the gate would have zero width, and gateWaypoints would refuse a
    // gate that is really there.
    std::vector<PlanBuoy> tight{
      {1, {0.5, 0}, Beacon::FlashingRed},
      {2, {-0.5, 0}, Beacon::FlashingGreen}};
    std::vector<Buoy> one{{200, {0, 0}, Beacon::Unknown, false}};
    const Fused g = fusePassage(tight, one, 5.0);
    chk("one contact is claimed by exactly one plan buoy",
      g.passage.size() == 2 &&
      !(g.passage[0].p.x == g.passage[1].p.x &&
        g.passage[0].p.y == g.passage[1].p.y));

    // Degenerate inputs.
    chk("a plan with no tracker at all still yields the passage",
      fusePassage(plan, {}, 3.0).passage.size() == 4);
    chk("... using the plan's own positions",
      std::fabs(fusePassage(plan, {}, 3.0).passage[1].p.x - 6.0) < 1e-9);
    chk("no plan means no passage, whatever the tracker saw",
      fusePassage({}, tracked, 3.0).passage.empty());
    chk("... and every contact is then an obstacle",
      fusePassage({}, tracked, 3.0).obstacles.size() == 4);
    chk("a zero radius associates nothing",
      std::fabs(fusePassage(plan, tracked, 0.0).passage[0].p.y - 20.0) < 1e-9);
  }

  // --------------------------------------------------------- avoidObstacles
  //
  // Collision avoidance lives in the tree now, not in the autopilot: OA_TYPE is
  // a black box that cannot be watched on the tree view or tested in a pool.
  {
    // Due north, 40 m. Port is WEST (-x), starboard is EAST (+x).
    const Vec2 from{0, 0}, to{0, 40};
    const double clear = 5.0, margin = 2.0;

    chk("clear water returns the goal untouched",
      !avoidObstacles(from, to, {}, clear, margin).detoured);

    std::vector<Buoy> abeam{{7, {12, 20}, Beacon::Off, false}};
    chk("something well off the track is not in the way",
      !avoidObstacles(from, to, abeam, clear, margin).detoured);

    std::vector<Buoy> behind{{7, {0, -10}, Beacon::Off, false}};
    chk("something astern is never in the way",
      !avoidObstacles(from, to, behind, clear, margin).detoured);

    std::vector<Buoy> beyond{{7, {0, 60}, Beacon::Off, false}};
    chk("something past the goal is not in the way either",
      !avoidObstacles(from, to, beyond, clear, margin).detoured);

    // Sitting to PORT of the track: pass it to starboard, so the detour goes
    // east of it.
    std::vector<Buoy> to_port{{7, {-2, 20}, Beacon::Off, false}};
    Detour d = avoidObstacles(from, to, to_port, clear, margin);
    chk("a blocker to port forces a detour", d.detoured && d.around_id == 7);
    chk("...and the boat passes it to starboard", d.wp.x > -2.0);
    chk_near("...by clearance + margin", d.wp.x - (-2.0), clear + margin, 1e-6);
    chk_near("...abeam of the obstacle", d.wp.y, 20.0, 1e-6);
    chk_near("...and reports how close the line came", d.miss_m, 2.0, 1e-6);

    std::vector<Buoy> to_stbd{{7, {2, 20}, Beacon::Off, false}};
    Detour e = avoidObstacles(from, to, to_stbd, clear, margin);
    chk("a blocker to starboard is passed to port", e.detoured && e.wp.x < 2.0);

    // Dead ahead has no favoured side. Break the tie to starboard: the
    // give-way side, and what a human driver expects.
    std::vector<Buoy> ahead{{7, {0, 20}, Beacon::Off, false}};
    Detour f = avoidObstacles(from, to, ahead, clear, margin);
    chk("dead ahead still forces a detour", f.detoured);
    chk("...and the tie breaks to starboard", f.wp.x > 0.0);

    // The FIRST blocker along the path, not the nearest to the boat. The one
    // at 30 m is closer to the track, but the boat meets the 10 m one first.
    std::vector<Buoy> two{
      {8, {3.0, 30}, Beacon::Off, false},
      {9, {-4.0, 10}, Beacon::Off, false}};
    Detour g = avoidObstacles(from, to, two, clear, margin);
    chk("the first blocker along the path wins", g.detoured && g.around_id == 9);

    chk("a zero-length leg cannot be blocked",
      !avoidObstacles(from, from, ahead, clear, margin).detoured);
    chk("zero clearance disables avoidance",
      !avoidObstacles(from, to, ahead, 0.0, margin).detoured);
  }

  // ------------------------------------------------------------ planPassage
  //
  // The boat does the path planning now: the aircraft sends ten buoys with
  // colours and nothing about which pair is a gate or what order to drive them.
  {
    // A three-gate channel running due north from the entry, with the reds to
    // the EAST so a northbound boat has them to starboard. Deliberately not
    // symmetric: gate 2 is offset east, so ordering by distance-from-entry and
    // ordering by projection-onto-the-axis disagree, and only the second is
    // right.
    const std::vector<Buoy> field{
      {0, {0, 0}, Beacon::FlashingBlue, false},      // entry
      {1, {5, 20}, Beacon::FlashingRed, false},      // gate A
      {2, {-5, 20}, Beacon::FlashingGreen, false},
      {3, {14, 40}, Beacon::FlashingRed, false},     // gate B, shoved east
      {4, {4, 40}, Beacon::FlashingGreen, false},
      {5, {5, 60}, Beacon::FlashingRed, false},      // gate C
      {6, {-5, 60}, Beacon::FlashingGreen, false},
      {7, {30, 10}, Beacon::Off, false},             // black, must be ignored
      {8, {0, 80}, Beacon::SteadyBlue, false}};      // exit
    const Vec2 entry{0, 0}, exitp{0, 80};

    Passage pa = planPassage(field, entry, exitp);
    chk("a field with an axis plans", pa.valid);
    chk("three pairs make three gates", pa.gates.size() == 3);
    chk("nothing is left unpaired", pa.unpaired.empty());
    chk("gates come out entry-first", pa.gates.size() == 3 &&
      pa.gates[0].red_id == 1 && pa.gates[1].red_id == 3 && pa.gates[2].red_id == 5);
    chk("each gate pairs red with the green beside it", pa.gates.size() == 3 &&
      pa.gates[0].green_id == 2 && pa.gates[1].green_id == 4 && pa.gates[2].green_id == 6);
    chk("a black buoy is never a gate buoy", [&] {
        for (const auto & g : pa.gates) {
          if (g.red_id == 7 || g.green_id == 7) {return false;}
        }
        return true;
      }());
    chk_near("along_m is the projection, not the range", pa.gates[1].along_m, 40.0, 1e-6);

    // Ordering must survive the field being handed over in any order at all --
    // the aircraft's id order is not the course order and never was.
    std::vector<Buoy> shuffled{field[8], field[5], field[2], field[7],
      field[3], field[0], field[6], field[1], field[4]};
    Passage sh = planPassage(shuffled, entry, exitp);
    chk("input order does not change the plan", sh.gates.size() == 3 &&
      sh.gates[0].red_id == 1 && sh.gates[1].red_id == 3 && sh.gates[2].red_id == 5);

    // Driving the passage backwards is a different course, and the gate order
    // must reverse with it.
    Passage rev = planPassage(field, exitp, entry);
    chk("reversing entry and exit reverses the order", rev.gates.size() == 3 &&
      rev.gates[0].red_id == 5 && rev.gates[2].red_id == 1);
  }

  // Pairing must be GLOBAL, not nearest-from-each-red. Both reds here are
  // closest to the same green; taking each red's nearest independently pairs
  // one of them across the channel.
  {
    const std::vector<Buoy> greedy{
      {1, {0, 0}, Beacon::FlashingRed, false},
      {2, {0, 30}, Beacon::FlashingRed, false},
      {3, {8, 4}, Beacon::FlashingGreen, false},     // nearest to BOTH reds
      {4, {8, 30}, Beacon::FlashingGreen, false}};
    Passage pa = planPassage(greedy, {0, -10}, {0, 50});
    chk("the closest pair claims each other first", pa.gates.size() == 2 &&
      pa.gates[0].red_id == 1 && pa.gates[0].green_id == 3);
    chk("and the loser takes the green that is left", pa.gates.size() == 2 &&
      pa.gates[1].red_id == 2 && pa.gates[1].green_id == 4);
  }

  // Width is a filter. A lone red on one side of the field and a lone green on
  // the other are not a gate, however much they are each other's nearest.
  {
    const std::vector<Buoy> wide{
      {1, {0, 20}, Beacon::FlashingRed, false},
      {2, {60, 20}, Beacon::FlashingGreen, false}};
    Passage pa = planPassage(wide, {0, 0}, {0, 40}, 20.0);
    chk("a pair wider than max_width is not a gate", pa.valid && pa.gates.empty());
    chk("... and both buoys are reported unpaired", pa.unpaired.size() == 2);
    chk("an empty passage is still a valid answer", pa.valid);

    const std::vector<Buoy> narrow{
      {1, {0, 20}, Beacon::FlashingRed, false},
      {2, {0.5, 20}, Beacon::FlashingGreen, false}};
    chk("a pair narrower than min_width is not a gate either",
      planPassage(narrow, {0, 0}, {0, 40}).gates.empty());
  }

  // Degenerate and lopsided fields.
  {
    chk("no axis means no plan",
      !planPassage({}, {5, 5}, {5, 5}).valid);
    chk("an empty field plans to zero gates",
      planPassage({}, {0, 0}, {0, 50}).valid &&
      planPassage({}, {0, 0}, {0, 50}).gates.empty());

    const std::vector<Buoy> lop{
      {1, {5, 20}, Beacon::FlashingRed, false},
      {2, {-5, 20}, Beacon::FlashingGreen, false},
      {3, {5, 40}, Beacon::FlashingRed, false}};      // no partner
    Passage pa = planPassage(lop, {0, 0}, {0, 60});
    chk("an odd red still yields the gate that does pair", pa.gates.size() == 1);
    chk("... and the orphan is named", pa.unpaired.size() == 1 && pa.unpaired[0] == 3);
  }

  // ------------------------------------------------ gateCleared / nextGate
  //
  // Cleared gates are remembered BY BUOY IDS. A confirmation can bring new
  // colours, the plan re-runs, and the order can change underneath the boat;
  // counting "I have done two, start at index two" skips a gate never driven.
  {
    const std::vector<Buoy> field{
      {1, {5, 20}, Beacon::FlashingRed, false},
      {2, {-5, 20}, Beacon::FlashingGreen, false},
      {3, {5, 40}, Beacon::FlashingRed, false},
      {4, {-5, 40}, Beacon::FlashingGreen, false}};
    Passage pa = planPassage(field, {0, 0}, {0, 60});
    std::vector<std::pair<int, int>> cleared;

    const PlannedGate * g = nextGate(pa, cleared);
    chk("the first gate is the nearest to the entry", g != nullptr && g->red_id == 1);
    cleared.push_back({g->red_id, g->green_id});

    g = nextGate(pa, cleared);
    chk("clearing one advances to the next", g != nullptr && g->red_id == 3);
    cleared.push_back({g->red_id, g->green_id});
    chk("clearing them all ends the passage", nextGate(pa, cleared) == nullptr);

    chk("a cleared pair is recognised either way round",
      gateCleared(cleared, 2, 1) && gateCleared(cleared, 1, 2));
    chk("an undriven pair is not cleared", !gateCleared(cleared, 1, 4));

    // The recolour case, which is the whole reason ids are the key: buoys 1
    // and 2 swap colours, so the near gate is now (2, 1) rather than (1, 2).
    // It is the same two buoys and must stay cleared.
    std::vector<Buoy> recoloured = field;
    recoloured[0].state = Beacon::FlashingGreen;
    recoloured[1].state = Beacon::FlashingRed;
    Passage after = planPassage(recoloured, {0, 0}, {0, 60});
    chk("a recolour keeps the same two buoys as a gate", after.gates.size() == 2 &&
      after.gates[0].red_id == 2 && after.gates[0].green_id == 1);
    // Only the NEAR gate cleared, so there is still one to find. Clearing both
    // would have proved nothing: nextGate returns null either way.
    const std::vector<std::pair<int, int>> one{{1, 2}};
    chk("the recoloured gate is still cleared, by its ids",
      gateCleared(one, after.gates[0].red_id, after.gates[0].green_id));
    chk("so the boat moves on to the gate it has not driven",
      nextGate(after, one) != nullptr && nextGate(after, one)->red_id == 3);
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
