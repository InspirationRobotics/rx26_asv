// nav_math.hpp — every piece of geometry Task 1 needs, and nothing else.
//
// Header-only, <cmath> and <vector> only. No ROS, no BehaviorTree.CPP, no
// rclcpp. That is the whole point: this is the one file in the behaviour-tree
// package where a sign error MIRRORS THE WORLD, and it is the one file that can
// be compiled and tested on a laptop in a second:
//
//     g++ -std=c++17 -O2 -o t test/test_nav_math.cpp && ./t
//
// Same split as crusader_perception's proximity_core.py: the leaves that use
// this are thin wrappers that read ports, call in here, publish and poll. All
// the arithmetic that can be wrong in an invisible way lives on this side.
//
// THE CONVENTIONS, in the order they bite:
//
//   Vec2 is a LOCAL TANGENT PLANE in metres: x EAST, y NORTH. That is ENU
//   without the up, which makes a positive rotation counter-clockwise and lets
//   the port/starboard maths read like plane geometry rather than navigation.
//
//   Bearings, where they appear, are degrees CLOCKWISE FROM NORTH, because that
//   is what the autopilot and every chart use. bearingDeg() is the only place
//   the two conventions meet.
//
//   THE SIDE RULE IS INVERTED FROM WHAT A SAILOR EXPECTS, and the handbook is
//   the authority (3.3.2):
//       "A FLASHING RED light must be passed on the surface system's starboard
//        side.  A FLASHING GREEN light must be passed on the port side."
//   So for a RED buoy WE go to ITS port side, and for GREEN we go to its
//   starboard side. Inverting this inverts the task, which is why sideWaypoint()
//   carries the sentence and the test asserts against a hand-worked case.
#ifndef CRUSADER_BT__NAV_MATH_HPP_
#define CRUSADER_BT__NAV_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

namespace crusader_bt
{
namespace nav
{

constexpr double kEarthR = 6371000.0;      // mean radius, m

// Spelled out rather than taken from M_PI. M_PI is a POSIX extension, not
// standard C++: it is absent under -std=c++17 on some toolchains (mingw among
// them) and the failure is a compile error in a header everything includes.
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

struct LatLon
{
  double lat = 0.0;
  double lon = 0.0;
};

/// Local tangent plane, metres. x EAST, y NORTH.
struct Vec2
{
  double x = 0.0;
  double y = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) {return {a.x + b.x, a.y + b.y};}
inline Vec2 operator-(Vec2 a, Vec2 b) {return {a.x - b.x, a.y - b.y};}
inline Vec2 operator*(Vec2 a, double s) {return {a.x * s, a.y * s};}
inline double dot(Vec2 a, Vec2 b) {return a.x * b.x + a.y * b.y;}
inline double norm(Vec2 a) {return std::hypot(a.x, a.y);}

/// 2D cross product. POSITIVE when b lies to the LEFT of a.
inline double cross(Vec2 a, Vec2 b) {return a.x * b.y - a.y * b.x;}

inline Vec2 unit(Vec2 a)
{
  const double n = norm(a);
  return n < 1e-9 ? Vec2{0.0, 1.0} : Vec2{a.x / n, a.y / n};   // degenerate -> north
}

// ---------------------------------------------------------------- projection
//
// Equirectangular about an origin. Good to well under a centimetre over the
// couple of hundred metres a task course spans, and it has the property that
// matters here: it is EXACTLY invertible, so a waypoint computed in metres and
// converted back to lat/lon lands where the maths put it.
//
// The origin must be held fixed for the whole mission. Re-deriving it from the
// boat's current position each tick would make every stored position drift as
// the boat moves — the classic moving-frame bug.

inline Vec2 toLocal(LatLon p, LatLon origin)
{
  return {
    (p.lon - origin.lon) * kDeg * kEarthR * std::cos(origin.lat * kDeg),
    (p.lat - origin.lat) * kDeg * kEarthR};
}

inline LatLon toLatLon(Vec2 v, LatLon origin)
{
  return {
    origin.lat + (v.y / kEarthR) / kDeg,
    origin.lon + (v.x / (kEarthR * std::cos(origin.lat * kDeg))) / kDeg};
}

/// Unit vector for a compass heading in degrees clockwise from north.
///
/// Returns north for a non-finite heading rather than NaN, but callers must
/// check isfinite() FIRST and refuse to act — on this boat heading comes from
/// GPS yaw with the compass disabled, so NaN is a real and current state, not a
/// theoretical one, and steering north because the heading was unknown is
/// exactly the guess this codebase forbids.
inline Vec2 headingVec(double heading_deg)
{
  if (!std::isfinite(heading_deg)) {return {0.0, 1.0};}
  return {std::sin(heading_deg * kDeg), std::cos(heading_deg * kDeg)};
}

/// Degrees clockwise from north, in [0, 360). The one place ENU meets compass.
inline double bearingDeg(Vec2 from, Vec2 to)
{
  const Vec2 d = to - from;
  double b = std::atan2(d.x, d.y) / kDeg;      // note: (east, north), not (y, x)
  return b < 0.0 ? b + 360.0 : b;
}

// ------------------------------------------------------------ port/starboard

/// Unit vector 90 deg to PORT of `travel` (i.e. rotated +90, counter-clockwise).
inline Vec2 portOf(Vec2 travel)
{
  const Vec2 u = unit(travel);
  return {-u.y, u.x};
}

inline Vec2 starboardOf(Vec2 travel)
{
  const Vec2 u = unit(travel);
  return {u.y, -u.x};
}

/// >0 when `p` lies to PORT of the track a->b, <0 to STARBOARD, 0 on the line.
///
/// This is the POST-HOC scoring check: run it over the logged track at the end
/// of a mission to find out which side each buoy was ACTUALLY passed on. The
/// plan says what we meant to do; this says what we did.
inline double crossTrack(Vec2 a, Vec2 b, Vec2 p)
{
  return cross(b - a, p - a);
}

// ------------------------------------------------------------- beacon states

/// Mirrors BeaconState in RoboCommand's rx_common.proto.
enum class Beacon
{
  Unknown = 0,
  Off = 1,
  FlashingRed = 2,
  FlashingGreen = 3,
  FlashingBlue = 4,   // ENTRY
  SteadyBlue = 5      // EXIT
};

inline bool isSideConstrained(Beacon b)
{
  return b == Beacon::FlashingRed || b == Beacon::FlashingGreen;
}

struct Buoy
{
  int id = -1;
  Vec2 p;
  Beacon state = Beacon::Unknown;
  bool consumed = false;
};

/// Where to steer so `buoy` ends up on the side the handbook demands.
///
/// handbook 3.3.2: "A FLASHING RED light must be passed on the surface system's
/// starboard side. A FLASHING GREEN light must be passed on the port side."
///
/// Read that carefully before touching the signs. For the buoy to end up on OUR
/// starboard, WE must pass down ITS port side — so a RED buoy's waypoint is
/// offset to PORT of the direction of travel, not to starboard. The two are
/// opposite and swapping them inverts the task while still looking plausible on
/// a plot.
inline Vec2 sideWaypoint(Vec2 buoy, Vec2 travel, Beacon state, double offset_m)
{
  if (state == Beacon::FlashingRed) {
    return buoy + portOf(travel) * offset_m;        // we go to ITS port
  }
  if (state == Beacon::FlashingGreen) {
    return buoy + starboardOf(travel) * offset_m;   // we go to ITS starboard
  }
  return buoy;    // OFF / unknown carry no ordering constraint
}

/// True when `buoy` was passed on the side the handbook demands, given the
/// track segment a->b the boat actually sailed.
inline bool passedCorrectly(Vec2 a, Vec2 b, const Buoy & buoy)
{
  if (!isSideConstrained(buoy.state)) {return true;}
  const double c = crossTrack(a, b, buoy.p);
  // RED must be to STARBOARD (c < 0); GREEN to PORT (c > 0).
  return buoy.state == Beacon::FlashingRed ? c < 0.0 : c > 0.0;
}

// -------------------------------------------------------------------- orbit

/// `n` waypoints once around `anchor` at `radius`, starting from the bearing
/// the boat already sits on so the first leg is short.
///
/// Core Tier wants a FULL circle, so the last waypoint returns to the starting
/// bearing: k runs 1..n, and k == n is a complete revolution. Direction is a
/// world-frame sense — clockwise means clockwise on a north-up chart, which in
/// this ENU frame is DECREASING angle.
inline std::vector<Vec2> orbit(
  Vec2 anchor, Vec2 from, double radius, int n, bool clockwise)
{
  std::vector<Vec2> out;
  if (n < 1) {return out;}
  const Vec2 d = from - anchor;
  const double a0 = (norm(d) < 1e-6) ? 0.0 : std::atan2(d.y, d.x);
  const double step = (clockwise ? -1.0 : 1.0) * 2.0 * kPi / static_cast<double>(n);
  out.reserve(static_cast<std::size_t>(n));
  for (int k = 1; k <= n; ++k) {
    const double a = a0 + step * k;
    out.push_back({anchor.x + radius * std::cos(a), anchor.y + radius * std::sin(a)});
  }
  return out;
}

/// Signed total turn of a closed waypoint ring, in degrees.
///
/// -360 for one clockwise revolution, +360 counter-clockwise. Used to assert a
/// generated orbit really goes all the way round the right way — a skipped or
/// mis-signed waypoint shows up here and nowhere else.
inline double sweepDeg(Vec2 anchor, const std::vector<Vec2> & ring, Vec2 from)
{
  if (ring.empty()) {return 0.0;}
  double total = 0.0;
  Vec2 prev = from;
  for (const Vec2 & w : ring) {
    const Vec2 a = prev - anchor;
    const Vec2 b = w - anchor;
    total += std::atan2(cross(a, b), dot(a, b)) / kDeg;
    prev = w;
  }
  return total;
}

// ------------------------------------------------------------- next waypoint

struct NextWaypoint
{
  bool ok = false;        ///< false = nothing sensible left to steer to
  Vec2 wp;
  int buoy_id = -1;       ///< -1 when the waypoint IS the exit
};

/// The next place to steer during the transit.
///
/// Rules, in order:
///   1. Direction of travel is toward the EXIT when it is known, else the
///      boat's own HEADING. Never the bearing of a candidate buoy — that would
///      make every candidate "ahead" and defeat rule 2 entirely.
///   2. Only buoys AHEAD of us count — dot(buoy - boat, travel) > 0. A buoy
///      already abeam or astern has been dealt with, and steering back to it
///      is how a boat ends up circling in the middle of the field.
///   3. Among those, take the NEAREST, and offset it to the required side.
///   4. Nothing ahead but the exit is known -> steer at the exit. NearExit in
///      the tree is what ends the transit; this function never decides that.
///   5. Nothing ahead and no exit -> ok = false, which is the loop's secondary
///      way out. Same result when the heading is NaN and there is no exit:
///      with no notion of "ahead" the honest answer is to steer nowhere.
///
/// Note what is NOT here: pairing buoys into gates. The handbook constrains
/// each RED/GREEN buoy individually and says nothing about pairs, so pairing is
/// an inference we do not need and that fails badly when one of a pair is
/// missed. That reasoning still holds for anything the BOAT infers on its own.
///
/// Above Core tier it is not an inference: the UAV can see the whole passage
/// from above and NAMES the pair, so the boat is told (red_id, green_id) rather
/// than guessing it. gateWaypoints() below is for that case only.
inline NextWaypoint nextWaypoint(
  const std::vector<Buoy> & buoys, Vec2 boat, double heading_deg, bool has_exit,
  Vec2 exit_p, double offset_m)
{
  // Where "ahead" points. It MUST NOT be derived from the candidate buoys:
  // taking the bearing of the nearest one makes that buoy ahead by
  // construction, the astern filter below stops filtering anything, and the
  // boat turns round to re-approach a buoy it has already passed. Caught by
  // test_nav_math on 2026-09-06, when this function did exactly that.
  Vec2 travel;
  if (has_exit) {
    travel = unit(exit_p - boat);
  } else if (std::isfinite(heading_deg)) {
    travel = headingVec(heading_deg);
  } else {
    return {};    // no exit and no heading: there is no "ahead" to speak of
  }

  const Buoy * pick = nullptr;
  double best_r = 0.0;
  for (const Buoy & b : buoys) {
    if (b.consumed || !isSideConstrained(b.state)) {continue;}
    const Vec2 rel = b.p - boat;
    if (dot(rel, travel) <= 0.0) {continue;}          // abeam or astern
    const double r = norm(rel);
    if (pick == nullptr || r < best_r) {pick = &b; best_r = r;}
  }

  if (pick != nullptr) {
    return {true, sideWaypoint(pick->p, travel, pick->state, offset_m), pick->id};
  }
  if (has_exit) {return {true, exit_p, -1};}
  return {};
}

// -------------------------------------------------------------------- gates

/// The UAV's "id not known" sentinel. Matches NO_BUOY in Mesh/rxl.py, which is
/// what RXL_NEXT_BUOY_SET puts in a buoy_id field it cannot fill.
constexpr int kNoBuoy = 255;

/// A gate the UAV has named, resolved into two places to steer.
struct Gate
{
  bool valid = false;
  Vec2 approach;                     ///< line up here, short of the gate
  Vec2 through;                      ///< then cross to here, past the middle
  double heading_deg = 0.0;          ///< the course through, deg cw from north
  const char * why = "no gate";      ///< why it is invalid, for the log
};

/// Where to steer to run the gate made by one RED and one GREEN buoy.
///
/// THE THROUGH-COURSE IS FIXED BY THE PAIR ALONE:
///
///     heading = bearingDeg(green -> red) - 90
///
/// RED must end up to starboard and GREEN to port, so the vector from green to
/// red points along the boat's starboard beam, and starboard is 90 deg clockwise
/// of the course. Subtracting 90 recovers the course.
///
/// Note what this does NOT read: the boat's own heading. That independence is
/// the point. `nextWaypoint` has to fall back on the live heading when the exit
/// is unknown, and in SITL on 2026-09-09 that lost the whole transit — a 360 deg
/// entry orbit ended the boat pointing at 207 deg, both gate buoys 25 m north
/// scored dot(rel, travel) < 0, and every one of them was discarded as astern.
/// A gate cannot be mis-ordered that way: the pair says which way through.
///
/// `approach` sits short of the middle and `through` past it, so the boat lines
/// up and then CROSSES rather than stopping between the buoys and turning.
///
/// Invalid when the two are closer than `min_width_m`: below that the bearing
/// between them is dominated by position noise, and a wrong bearing here puts
/// the boat through the gate sideways or backwards. Refusing is the honest
/// answer; the caller can still fall back on sideWaypoint() for a lone buoy.
inline Gate gateWaypoints(
  Vec2 red, Vec2 green, double standoff_m, double approach_m,
  double min_width_m = 2.0)
{
  const double width = norm(red - green);
  if (!(width >= min_width_m)) {          // NaN-safe: a NaN width is invalid
    return {false, {}, {}, 0.0, "gate too narrow to take a bearing"};
  }

  double h = bearingDeg(green, red) - 90.0;
  if (h < 0.0) {h += 360.0;}

  const Vec2 u = headingVec(h);
  const Vec2 mid = (red + green) * 0.5;
  return {true, mid - u * approach_m, mid + u * standoff_m, h, ""};
}

/// The buoy with this id, or nullptr. Ids are the UAV's, not the tracker's.
inline const Buoy * findById(const std::vector<Buoy> & buoys, int id)
{
  if (id < 0 || id == kNoBuoy) {return nullptr;}
  for (const Buoy & b : buoys) {
    if (b.id == id) {return &b;}
  }
  return nullptr;
}

/// gateWaypoints() for a pair the UAV named by id.
///
/// Every way this can fail says so in `why` rather than steering somewhere
/// plausible: an unfilled id (255), an id for a buoy we have never seen, and
/// the same id twice — which would otherwise give a zero-width gate and a
/// meaningless bearing.
inline Gate gateFromIds(
  const std::vector<Buoy> & buoys, int red_id, int green_id,
  double standoff_m, double approach_m, double min_width_m = 2.0)
{
  if (red_id == green_id) {
    return {false, {}, {}, 0.0, "red and green are the same buoy"};
  }
  const Buoy * r = findById(buoys, red_id);
  const Buoy * g = findById(buoys, green_id);
  if (r == nullptr && g == nullptr) {return {false, {}, {}, 0.0, "neither buoy known"};}
  if (r == nullptr) {return {false, {}, {}, 0.0, "red buoy not known"};}
  if (g == nullptr) {return {false, {}, {}, 0.0, "green buoy not known"};}
  return gateWaypoints(r->p, g->p, standoff_m, approach_m, min_width_m);
}

/// The first buoy in `buoys` with the given beacon state, or nullptr.
inline const Buoy * findBeacon(const std::vector<Buoy> & buoys, Beacon want)
{
  for (const Buoy & b : buoys) {
    if (b.state == want) {return &b;}
  }
  return nullptr;
}

}  // namespace nav
}  // namespace crusader_bt

#endif  // CRUSADER_BT__NAV_MATH_HPP_
