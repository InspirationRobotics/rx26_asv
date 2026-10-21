// nav_math.hpp — every piece of geometry Task 1 needs, and nothing else.
//
// Header-only, <cmath> and <vector> only: no ROS, no BehaviorTree.CPP, no
// rclcpp. A sign error here MIRRORS THE WORLD, and this compiles and runs on a
// laptop in a second:
//
//     g++ -std=c++17 -O2 -o t test/test_nav_math.cpp && ./t
//
// THE CONVENTIONS, in the order they bite:
//
//   Vec2 is a LOCAL TANGENT PLANE in metres: x EAST, y NORTH (ENU without the
//   up), so a positive rotation is counter-clockwise.
//
//   Bearings are degrees CLOCKWISE FROM NORTH (autopilot and chart convention).
//   bearingDeg() is the only place the two conventions meet.
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
#include <cctype>
#include <cmath>
#include <string>
#include <utility>
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
// Equirectangular about an origin: good to well under a centimetre over a task
// course and EXACTLY invertible. Hold the origin fixed for the whole mission;
// re-deriving it from the boat's position each tick makes every stored position drift.

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
/// check isfinite() FIRST and refuse to act: heading comes from GPS yaw with the
/// compass disabled, so NaN is a real state, and steering north because the
/// heading was unknown is a guess.
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
/// Post-hoc scoring check on the logged track.
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

/// A tracked target's label as a beacon state; anything unrecognised is Unknown (no side
/// constraint), never guessed into a colour. ENTRY is FLASHING blue, EXIT STEADY blue:
/// camera_node's buoy model names a lit shape with no prefix (blue_diamond) and a flashing
/// one flash_*, while a detector that says only blue_buoy cannot tell the two apart.
inline Beacon beaconFromLabel(const std::string & raw)
{
  std::string s;
  for (char c : raw) {s += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));}
  const auto has = [&s](const char * w) {return s.find(w) != std::string::npos;};
  if (has("red")) {return Beacon::FlashingRed;}
  if (has("green")) {return Beacon::FlashingGreen;}
  if (has("blue")) {
    if (has("flash")) {return Beacon::FlashingBlue;}
    if (has("steady") || has("solid")) {return Beacon::SteadyBlue;}
    if (s.rfind("blue_", 0) == 0 && s != "blue_buoy") {return Beacon::SteadyBlue;}
    return Beacon::Unknown;
  }
  if (has("off") || has("black")) {return Beacon::Off;}
  return Beacon::Unknown;
}

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
/// For the buoy to end up on OUR starboard, WE must pass down ITS port side, so a
/// RED buoy's waypoint is offset to PORT of the direction of travel, not to
/// starboard. Swapping them inverts the task while still looking plausible on a
/// plot.
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
/// Core Tier wants a FULL circle: k runs 1..n and k == n is a complete revolution.
/// Direction is a world-frame sense: clockwise on a north-up chart is DECREASING
/// angle in this ENU frame.
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
/// -360 for one clockwise revolution, +360 counter-clockwise. Asserts a generated
/// orbit really goes all the way round the right way.
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
///      boat's own HEADING. Never the bearing of a candidate buoy: that makes
///      every candidate "ahead" and defeats rule 2.
///   2. Only buoys AHEAD of us count: dot(buoy - boat, travel) > 0. A buoy
///      already abeam or astern has been dealt with.
///   3. Among those, take the NEAREST, and offset it to the required side.
///   4. Nothing ahead but the exit is known -> steer at the exit. NearExit in
///      the tree is what ends the transit; this function never decides that.
///   5. Nothing ahead and no exit -> ok = false, the loop's secondary way out.
///      Same when the heading is NaN and there is no exit.
///
/// No pairing of buoys into gates here (the handbook constrains each RED/GREEN buoy
/// individually); above Core tier the UAV names the pair and gateWaypoints() below
/// is for that case.
inline NextWaypoint nextWaypoint(
  const std::vector<Buoy> & buoys, Vec2 boat, double heading_deg, bool has_exit,
  Vec2 exit_p, double offset_m)
{
  // Where "ahead" points. It MUST NOT be derived from the candidate buoys: that
  // makes the nearest one ahead by construction, the astern filter below stops
  // filtering, and the boat turns round to re-approach a buoy it has passed.
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

/// "No buoy id": a gate side with no buoy.
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

/// The direction a boat must travel to cross the pair (red, green) with the red to
/// STARBOARD: the red-minus-green vector turned 90 deg counter-clockwise, as a unit
/// vector. The red then lies to the right of it. gateWaypoints()'s heading is this
/// direction as a compass bearing.
inline Vec2 crossingDir(Vec2 red_minus_green) {return portOf(red_minus_green);}

/// Where to steer to run the gate made by one RED and one GREEN buoy.
///
/// THE THROUGH-COURSE IS FIXED BY THE PAIR ALONE:
///
///     heading = bearingDeg(green -> red) - 90
///
/// RED must end up to starboard and GREEN to port, so the vector from green to red
/// points along the boat's starboard beam, which is 90 deg clockwise of the course.
///
/// It deliberately does not read the boat's own heading: nextWaypoint() falls back on
/// it when the exit is unknown, and a heading 180 deg off there discards both gate
/// buoys as astern.
///
/// `approach` sits short of the middle and `through` past it, so the boat lines up
/// and then CROSSES. Invalid when the two are closer than `min_width_m` (the bearing
/// between them is then noise): refusing is the honest answer, and the caller can
/// fall back on sideWaypoint() for a lone buoy.
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

/// gateWaypoints() for a pair the UAV named by id. Every way this can fail says so
/// in `why` rather than steering somewhere plausible: an unfilled id (255), an id for
/// a buoy we have never seen, or the same id twice (a zero-width gate).
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

// ------------------------------------------------------------------- fusion

/// One buoy exactly as the UAV reported it, before anything local touches it.
struct PlanBuoy
{
  int id = -1;                     ///< the UAV's index, 0..9. NOT a tracker id.
  Vec2 p;
  Beacon state = Beacon::Unknown;
};

struct Fused
{
  /// The passage, keyed by UAV id. Beacon state always came from the plan.
  std::vector<Buoy> passage;
  /// Tracked contacts no plan buoy claimed. Beacon is always Unknown, and
  /// these can never become a gate candidate.
  std::vector<Buoy> obstacles;
};

/// Merge what the aircraft says with what the boat can see.
/// THE UAV WINS, narrowly:
///
///   * BEACON STATE always comes from the plan (above Core tier the passage is only
///     detectable from the air; the boat's own colour guess is noise).
///   * POSITION comes from the TRACKER when a contact was associated, else the plan:
///     LiDAR is best close in, where metre accuracy at a gate matters.
///   * THE ID stays the UAV's (the tracker's ids are another number space).
///
/// A tracked contact matching nothing in the plan is NOT a passage buoy: it goes to
/// `obstacles` with Beacon::Unknown (never half a gate) but still becomes a hazard
/// (path::buildHazards). Association is GLOBAL greedy nearest-pair, since per-buoy
/// lets one contact be claimed twice. `consumed` is NOT carried here: this function
/// is pure and called fresh every tick, so the caller (bt_runner_node) re-applies it
/// after every rebuild.
inline Fused fusePassage(
  const std::vector<PlanBuoy> & plan, const std::vector<Buoy> & tracked,
  double assoc_radius_m)
{
  Fused out;
  std::vector<bool> claimed(tracked.size(), false);
  std::vector<int> match(plan.size(), -1);

  // Greedy: take the closest pair still available, repeat. At most
  // min(|plan|, |tracked|) rounds over a ten-buoy field.
  for (;;) {
    double best = assoc_radius_m;
    int bi = -1, bj = -1;
    for (std::size_t i = 0; i < plan.size(); ++i) {
      if (match[i] >= 0) {continue;}
      for (std::size_t j = 0; j < tracked.size(); ++j) {
        if (claimed[j]) {continue;}
        const double r = norm(tracked[j].p - plan[i].p);
        if (r <= best) {
          best = r;
          bi = static_cast<int>(i);
          bj = static_cast<int>(j);
        }
      }
    }
    if (bi < 0) {break;}
    match[bi] = bj;
    claimed[bj] = true;
  }

  for (std::size_t i = 0; i < plan.size(); ++i) {
    Buoy b;
    b.id = plan[i].id;
    b.state = plan[i].state;                       // the UAV wins, always
    b.p = (match[i] >= 0) ? tracked[match[i]].p : plan[i].p;
    out.passage.push_back(b);
  }
  for (std::size_t j = 0; j < tracked.size(); ++j) {
    if (claimed[j]) {continue;}
    Buoy o = tracked[j];
    o.state = Beacon::Unknown;    // whatever the boat thought it saw, it is not
    out.obstacles.push_back(o);   // part of the passage
  }
  return out;
}

/// One gate of the passage, as the BOAT worked it out.
struct PlannedGate
{
  int red_id = kNoBuoy;
  int green_id = kNoBuoy;
  Vec2 mid;                        ///< midpoint of the pair
  double along_m = 0.0;            ///< distance along entry -> exit, for ordering
  double width_m = 0.0;            ///< how far apart the pair is
};

/// The whole passage, ordered entry to exit.
struct Passage
{
  bool valid = false;              ///< false = cannot plan; `why` says what is missing
  std::vector<PlannedGate> gates;  ///< in the order the boat should drive them
  std::vector<int> unpaired;       ///< red or green buoys that found no partner
  std::string why;
};

/// Pair the red and green buoys into gates and order them entry -> exit. THE BOAT
/// DOES THIS, NOT THE AIRCRAFT (which sends ten buoys with beacon states).
///
/// PAIRING IS GLOBAL GREEDY NEAREST, as in fusePassage: repeatedly take the closest
/// red-green pair still unclaimed (per-red nearest lets two reds share a green and
/// leaves the other paired across the channel).
///
/// ORDERING IS BY PROJECTION ONTO entry -> exit, not distance from the entry, which
/// orders a curved channel wrongly.
///
/// WIDTH IS A FILTER: a pair further apart than `max_width_m` is a red from one gate
/// and a green from another; those buoys go to `unpaired`.
///
/// ORIENTATION IS A FILTER TOO. A pair is a gate only if the boat can drive it
/// FORWARDS with the red to starboard: the crossing direction
///
///     d = rot90ccw(unit(red - green))        (red lies to the right of d)
///
/// must be within `max_cross_deg` of the entry -> exit axis. A red on the LEFT of its
/// green, facing down the course, has d pointing back at the entry. Such a pair
/// claims neither buoy (each can still pair with a better partner) and, failing that,
/// both stay unpaired singles, which the side fences (path::sideFences) hold to their
/// correct sides.
///
/// An empty `gates` with valid=true is a real answer: entry and exit with no
/// red-green pairs means drive straight to the exit.
inline Passage planPassage(
  const std::vector<Buoy> & buoys, Vec2 entry, Vec2 exitp,
  double max_width_m = 20.0, double min_width_m = 2.0, double max_cross_deg = 60.0)
{
  Passage out;

  if (norm(exitp - entry) < 1e-6) {
    out.why = "entry and exit are the same point; no axis to order gates along";
    return out;
  }
  const Vec2 axis = unit(exitp - entry);
  const double min_cos = std::cos(max_cross_deg * kDeg);

  std::vector<int> reds, greens;
  for (std::size_t i = 0; i < buoys.size(); ++i) {
    if (buoys[i].state == Beacon::FlashingRed) {reds.push_back(static_cast<int>(i));}
    if (buoys[i].state == Beacon::FlashingGreen) {greens.push_back(static_cast<int>(i));}
  }

  std::vector<bool> red_used(reds.size(), false), green_used(greens.size(), false);
  for (;;) {
    double best = max_width_m;
    int bi = -1, bj = -1;
    for (std::size_t i = 0; i < reds.size(); ++i) {
      if (red_used[i]) {continue;}
      for (std::size_t j = 0; j < greens.size(); ++j) {
        if (green_used[j]) {continue;}
        const Vec2 rg = buoys[reds[i]].p - buoys[greens[j]].p;
        const double w = norm(rg);
        if (w <= best && w >= min_width_m && dot(crossingDir(rg), axis) >= min_cos - 1e-9) {
          best = w;
          bi = static_cast<int>(i);
          bj = static_cast<int>(j);
        }
      }
    }
    if (bi < 0) {break;}
    red_used[bi] = true;
    green_used[bj] = true;

    const Buoy & r = buoys[reds[bi]];
    const Buoy & g = buoys[greens[bj]];
    PlannedGate gate;
    gate.red_id = r.id;
    gate.green_id = g.id;
    gate.mid = (r.p + g.p) * 0.5;
    gate.along_m = dot(gate.mid - entry, axis);
    gate.width_m = best;
    out.gates.push_back(gate);
  }

  for (std::size_t i = 0; i < reds.size(); ++i) {
    if (!red_used[i]) {out.unpaired.push_back(buoys[reds[i]].id);}
  }
  for (std::size_t j = 0; j < greens.size(); ++j) {
    if (!green_used[j]) {out.unpaired.push_back(buoys[greens[j]].id);}
  }

  std::sort(
    out.gates.begin(), out.gates.end(),
    [](const PlannedGate & a, const PlannedGate & b) {return a.along_m < b.along_m;});

  out.valid = true;
  if (out.gates.empty()) {
    out.why = "no red-green pair within the gate width; driving straight to the exit";
  }
  return out;
}

/// Is this pair already in `cleared`? Order-insensitive on purpose.
///
/// A gate is remembered by the IDS OF ITS TWO BUOYS, not its index: a
/// confirmation can bring new colours and planPassage re-runs with a changed
/// order, so "I have done three" would skip a gate that was never driven.
inline bool gateCleared(
  const std::vector<std::pair<int, int>> & cleared, int red_id, int green_id)
{
  for (const auto & c : cleared) {
    if ((c.first == red_id && c.second == green_id) ||
      (c.first == green_id && c.second == red_id))
    {
      return true;
    }
  }
  return false;
}

/// The first gate of `passage` the boat has not cleared, or nullptr when done.
inline const PlannedGate * nextGate(
  const Passage & passage, const std::vector<std::pair<int, int>> & cleared)
{
  for (const PlannedGate & g : passage.gates) {
    if (!gateCleared(cleared, g.red_id, g.green_id)) {return &g;}
  }
  return nullptr;
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
