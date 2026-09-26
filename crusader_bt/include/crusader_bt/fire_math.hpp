// fire_math.hpp — the fixed-nozzle shot: hold a range and a heading, wait until
// the boat is still, fire. Every number in it, and nothing else.
//
// Same contract as nav_math.hpp and dock_math.hpp: header-only, stdlib only, no
// ROS, no BehaviorTree.CPP, so it runs on a laptop in a second:
//
//     g++ -std=c++17 -O2 -I include -o t test/test_fire_math.cpp && ./t
//
// WHY A NOZZLE NEEDS ITS OWN MATHS. The Task 3 nozzle is fixed (~45 deg, no pan,
// no tilt), so the boat IS the aim. Its distance from the dock sets how high the
// water lands and its heading sets how far left or right. tools/squirt_cal found
// the range that hits a window by firing at it; this file turns that number into
// "drive this far, point this way, and fire when you are still".
//
//   the wall      /crsd/wall_range, filtered: median over a short window, and
//                 BLANKED while water is in the air (the spray returns LiDAR
//                 points short of the wall).
//   the aim       heading so the stream's line passes through the window, from
//                 the boat's offset in the slip (LiDAR fingers), the window's
//                 bearing (camera), or an assumed centreline.
//   the keep      signed speed toward the firing range, square to the wall
//                 until close, then on the aim heading. A BANDED law, not a
//                 smooth one: ArduRover's speed loop is untuned below ~0.2 m/s,
//                 so a small command either does nothing or overshoots. Inside
//                 the band: zero. Outside: at least v_min.
//   steady        the hull's roll and pitch, and no thrust recently: a degree of
//                 pitch moves the hit ~2-3 cm.
//   the gate      all of the above held for hold_s, then one burst.
//
// CONVENTIONS (the same as dock_math):
//
//   Heading is compass degrees, clockwise from north. Body frame is REP-103:
//   x FORWARD, y LEFT, so "turn left" makes the compass heading SMALLER.
//   WallRange.angle_deg is the body bearing of the wall's nearest point, + LEFT:
//   turning left by angle_deg squares the bow to the wall, so the wall's inward
//   normal is at compass (heading - angle_deg).
//   WallRange.lat_m is the boat's offset from the slip centre, + LEFT. A window's
//   lateral position is from the face centre, + LEFT (the upper-left window is
//   +0.22 m), the same sign as squirt_cal's target_lat_m.
#ifndef CRUSADER_BT__FIRE_MATH_HPP_
#define CRUSADER_BT__FIRE_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace crusader_bt
{
namespace fire
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

inline double wrap180(double a)
{
  a = std::fmod(a + 180.0, 360.0);
  if (a < 0) {a += 360.0;}
  return a - 180.0;
}

inline double wrap360(double a)
{
  a = std::fmod(a, 360.0);
  return a < 0 ? a + 360.0 : a;
}

inline double median(std::vector<double> v)
{
  if (v.empty()) {return kNaN;}
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

// ------------------------------------------------------------------ the wall

struct WallSample
{
  double t = 0.0;           // seconds, the runner's monotonic clock
  bool valid = false;
  double range_m = kNaN;    // body origin -> wall, perpendicular
  double angle_deg = kNaN;  // + = the wall's nearest point is to the LEFT
  double lat_m = kNaN;      // offset from the slip centre, + LEFT; NaN = no fingers
  double heading_deg = kNaN;  // the boat's compass heading when it was taken
};

// The wall as the shot sees it: medians over the last window_s of VALID
// samples that arrived outside the blanking interval. A burst blanks the
// filter from its start until the water has landed (BurstBook sets it), because
// the stream returns LiDAR points short of the wall and a median of those would
// walk the boat forward into its own spray.
class WallFilter
{
public:
  double window_s = 0.5;
  double keep_s = 5.0;

  void add(const WallSample & s)
  {
    buf_.push_back(s);
    while (!buf_.empty() && buf_.front().t < s.t - keep_s) {buf_.pop_front();}
    last_t_ = s.t;
    if (s.valid) {last_valid_t_ = s.t;}
  }

  void blank_until(double t) {blank_until_ = std::max(blank_until_, t);}
  double blanked_until() const {return blank_until_;}
  bool blanked(double now) const {return now < blank_until_;}

  // seconds since the last valid sample; +inf with none
  double valid_age(double now) const
  {
    return last_valid_t_ < 0 ? std::numeric_limits<double>::infinity() : now - last_valid_t_;
  }

  double range(double now) const {return pick(now, [](const WallSample & s) {return s.range_m;});}
  double lat(double now) const {return pick(now, [](const WallSample & s) {return s.lat_m;});}

  // Compass bearing of the wall's inward normal, from each sample's own heading
  // (heading - angle), so a turn in progress does not smear it. Median of the
  // wrapped offsets from the newest, then re-anchored: a circular median good
  // for the few degrees a wall normal wanders.
  double inward_deg(double now) const
  {
    std::vector<double> v;
    double ref = kNaN;
    for (auto it = buf_.rbegin(); it != buf_.rend(); ++it) {
      if (!use(*it, now) || !std::isfinite(it->angle_deg) || !std::isfinite(it->heading_deg)) {
        continue;
      }
      const double b = wrap360(it->heading_deg - it->angle_deg);
      if (!std::isfinite(ref)) {ref = b;}
      v.push_back(wrap180(b - ref));
    }
    if (v.empty()) {return kNaN;}
    return wrap360(ref + median(v));
  }

  void reset() {buf_.clear(); blank_until_ = -1e18; last_t_ = last_valid_t_ = -1.0;}

private:
  bool use(const WallSample & s, double now) const
  {
    return s.valid && s.t >= now - window_s && s.t > blank_until_ && s.t <= now + 1e-9;
  }

  template<class F>
  double pick(double now, F f) const
  {
    std::vector<double> v;
    for (const auto & s : buf_) {
      const double x = f(s);
      if (use(s, now) && std::isfinite(x)) {v.push_back(x);}
    }
    return median(v);
  }

  std::deque<WallSample> buf_;
  double blank_until_ = -1e18;
  double last_t_ = -1.0;
  double last_valid_t_ = -1.0;
};

// ------------------------------------------------------------------ the aim

enum class Lateral {Fingers, Camera, Centreline};

inline Lateral lateralFromName(const std::string & s)
{
  if (s == "fingers") {return Lateral::Fingers;}
  if (s == "cv" || s == "camera") {return Lateral::Camera;}
  return Lateral::Centreline;
}

struct AimParams
{
  double fire_range_m = 3.22;   // the calibrated wall range for this window (squirt_cal)
  double window_lat_m = 0.22;   // the window from the face centre, + LEFT (UL = +0.22)
  double yaw_bias_deg = 0.0;    // calibrated: + aims further LEFT (the stream's own skew)
  double cam_x_m = 0.37;        // camera ahead of the body origin (for the CV bearing)
  double max_aim_deg = 20.0;    // refuse an aim wider than this: something is wrong
};

struct Aim
{
  bool ok = false;
  double range_target_m = kNaN;   // corrected for the yaw
  double heading_deg = kNaN;      // compass heading that puts the stream on the window
  double aim_left_deg = kNaN;     // how far left of the wall normal that is
  std::string source;
  std::string why;
};

// Where to point and how far to stand off.
//
// The nozzle is on the boat's centreline, so the stream follows the centreline.
// Turned LEFT of the wall normal by psi, that line meets the wall r*tan(psi)
// left of the body origin's foot. To put it on a window `ahead_left` metres
// left of the foot: psi = atan(ahead_left / r).
//
// And the range: the calibration was shot roughly square on, so the stream's
// horizontal run was ~r_cal. Turned by psi the run is r/cos(psi); keeping it
// the same means standing at r_cal*cos(psi). Millimetres at 4 deg, but free.
//
//   fingers      ahead_left = window_lat - lat_now   (the LiDAR's offset in
//                the slip: the same number the calibration logged)
//   camera       the window's bearing b in the camera frame (+ left). It sits
//                (r - cam_x)*tan(b) left of the CURRENT centreline, so the
//                turn is relative to the current heading, not the normal
//   centreline   assume the boat is on the slip centreline: ahead_left = window_lat
inline Aim solveAim(
  const AimParams & p, Lateral mode, double range_m, double inward_deg,
  double heading_now_deg, double lat_now_m, double window_bearing_deg)
{
  Aim a;
  if (!std::isfinite(range_m) || range_m <= 0.3) {a.why = "no wall range"; return a;}
  if (!std::isfinite(inward_deg)) {a.why = "no wall bearing"; return a;}
  double psi = kNaN;                      // left of the wall normal, degrees
  if (mode == Lateral::Camera) {
    if (!std::isfinite(window_bearing_deg)) {a.why = "window not in view"; return a;}
    if (!std::isfinite(heading_now_deg)) {a.why = "no heading"; return a;}
    const double y = (range_m - p.cam_x_m) * std::tan(window_bearing_deg * kDeg);
    const double turn_left = std::atan2(y, range_m) / kDeg + p.yaw_bias_deg;
    const double target = wrap360(heading_now_deg - turn_left);
    psi = wrap180(inward_deg - target);
    a.source = "camera";
  } else {
    double ahead_left = p.window_lat_m;
    if (mode == Lateral::Fingers) {
      if (!std::isfinite(lat_now_m)) {a.why = "no slip fingers in view"; return a;}
      ahead_left -= lat_now_m;
      a.source = "fingers";
    } else {
      a.source = "centreline";
    }
    psi = std::atan2(ahead_left, range_m) / kDeg + p.yaw_bias_deg;
  }
  if (std::fabs(psi) > p.max_aim_deg) {
    a.why = "aim of " + std::to_string(static_cast<int>(std::lround(psi))) +
      " deg is too wide";
    return a;
  }
  a.aim_left_deg = psi;
  a.heading_deg = wrap360(inward_deg - psi);
  a.range_target_m = p.fire_range_m * std::cos(psi * kDeg);
  a.ok = true;
  return a;
}

// ------------------------------------------------------------------ the cross-check

// The perpendicular distance from a point (the boat) to a face plane, given a
// point on the face and its OUTWARD unit normal: + = in front of the face.
inline double planeDistance(double be, double bn, double pe, double pn, double oe, double on)
{
  return (be - pe) * oe + (bn - pn) * on;
}

// Do the LiDAR and the camera agree on how far the dock is? The wall fit's
// likeliest mistake at a real dock is to lock onto the slip FINGER TIPS - four
// collinear 0.5 m faces, nearer and denser than the deck edge behind them -
// and read ~finger_len (2 m) short; the keep would then stand 2 m too far
// out, or, locked on something behind the dock, drive into it. The camera's
// dock book places the faces independently. No camera range (the pool, a
// dead detector) = no check: NaN agrees. tol <= 0 turns it off.
inline bool rangesAgree(double lidar_m, double camera_m, double tol_m)
{
  if (!(tol_m > 0.0) || !std::isfinite(camera_m)) {return true;}
  return std::isfinite(lidar_m) && std::fabs(lidar_m - camera_m) <= tol_m;
}

// ------------------------------------------------------------------ the keep

struct KeepParams
{
  double deadband_m = 0.05;     // inside this of the target: no thrust at all
  double kp = 0.6;              // m/s per m of error, above v_min
  double v_min = 0.12;          // the least speed ArduRover acts on reliably
  double v_max_fwd = 0.25;
  double v_max_rev = 0.25;
  double accel = 0.25;          // m/s^2: gentle, because thrust rocks the hull
  double min_range_m = 1.5;     // never drive FORWARD closer than this
  double turn_first_deg = 10.0; // heading error above this: turn, don't drive
};

struct KeepCmd
{
  double heading_deg = kNaN;
  double speed_mps = 0.0;       // + forward (closer), - astern
  std::string why;
};

// Which heading the keep holds. SQUARE to the wall (its inward normal) while
// more than square_m from the firing range, the aim heading inside that.
// Driving the whole approach on the aim heading walks the boat sideways - 4 m
// at 2-4 degrees is 15-25 cm, more than half a window - which breaks the
// centreline assumption and, from far out, the fingers are not in view to
// aim with at all. Square first keeps the boat on the line it started on.
// NaN = nothing to hold (no wall bearing, or in close with no aim).
inline double keepHeading(
  const Aim & a, double inward_deg, double range_m, double fire_range_m, double square_m)
{
  if (std::isfinite(inward_deg) && std::isfinite(range_m) &&
    std::fabs(range_m - fire_range_m) > square_m)
  {
    return inward_deg;
  }
  return a.ok ? a.heading_deg : kNaN;
}

// Signed speed toward the range target. `prev_speed` and `dt` bound the change
// (the slew), so no tick ever asks for a jump in thrust.
inline KeepCmd stationKeep(
  const KeepParams & p, double range_m, double target_m, double heading_now_deg,
  double heading_target_deg, double prev_speed, double dt)
{
  KeepCmd c;
  c.heading_deg = heading_target_deg;
  double want = 0.0;
  const double err = range_m - target_m;              // + = too far out
  const double herr = std::isfinite(heading_now_deg) && std::isfinite(heading_target_deg) ?
    std::fabs(wrap180(heading_target_deg - heading_now_deg)) : 999.0;
  if (!std::isfinite(err)) {
    c.why = "no range";
  } else if (herr > p.turn_first_deg) {
    c.why = "turning first";
  } else if (std::fabs(err) <= p.deadband_m) {
    c.why = "in the band";
  } else {
    const double mag = std::max(p.v_min, p.kp * std::fabs(err));
    want = err > 0 ? std::min(mag, p.v_max_fwd) : -std::min(mag, p.v_max_rev);
    c.why = err > 0 ? "closing in" : "backing off";
    if (want > 0 && range_m <= p.min_range_m) {
      want = 0.0;
      c.why = "at the minimum range";
    }
  }
  const double step = std::max(0.0, p.accel * dt);
  c.speed_mps = std::clamp(want, prev_speed - step, prev_speed + step);
  if (std::fabs(c.speed_mps) < 1e-6) {c.speed_mps = 0.0;}
  return c;
}

// ------------------------------------------------------------------ steady

struct SteadyParams
{
  double rate_max_dps = 4.0;    // roll and pitch rate
  double band_deg = 1.5;        // either side of the running mean
  double hold_s = 0.5;          // all of the above over this window
  double mean_s = 5.0;
  double quiet_s = 1.5;         // no thrust commanded for this long
  double min_hz = 8.0;          // attitude samples per second needed to judge
  double att_timeout_s = 0.5;
};

struct SteadyStatus
{
  bool steady = false;
  std::string why;              // the first reason it is not, "" when steady
  double rate_max_dps = kNaN;
  double pitch_dev_deg = kNaN;
  double roll_dev_deg = kNaN;
  double quiet_s = kNaN;
  double att_hz = 0.0;
};

// The hull, still enough to shoot. A port of tools/squirt_cal/steady_core.py,
// with one change: "the pilot's sticks are quiet" becomes "WE have not asked
// for thrust", because the tree is the pilot now and thrust is what sets the
// hull rocking.
class SteadyMonitor
{
public:
  SteadyParams p;

  void feed_att(double t, double roll_deg, double pitch_deg, double rr_dps, double pr_dps)
  {
    att_.push_back({t, roll_deg, pitch_deg, rr_dps, pr_dps});
    const double keep = std::max(p.mean_s, 10.0);
    while (!att_.empty() && att_.front().t < t - keep) {att_.pop_front();}
  }

  // Every command the keep sends: any non-zero speed restarts the quiet clock.
  void feed_cmd(double t, double speed_mps)
  {
    if (std::fabs(speed_mps) > 1e-6 || last_thrust_t_ < 0) {last_thrust_t_ = t;}
  }

  double att_hz(double now, double window = 2.0) const
  {
    int n = 0;
    for (const auto & a : att_) {if (a.t >= now - window) {++n;}}
    return n / window;
  }

  SteadyStatus status(double now) const
  {
    SteadyStatus s;
    s.att_hz = att_hz(now);
    s.quiet_s = last_thrust_t_ < 0 ? kNaN : now - last_thrust_t_;
    if (att_.empty() || now - att_.back().t > p.att_timeout_s) {s.why = "no attitude"; return s;}
    if (s.att_hz < p.min_hz) {
      s.why = "attitude only " + std::to_string(static_cast<int>(s.att_hz)) + " Hz";
      return s;
    }
    double sr = 0, sp = 0; int n = 0;
    for (const auto & a : att_) {if (a.t >= now - p.mean_s) {sr += a.roll; sp += a.pitch; ++n;}}
    const double mr = sr / n, mp = sp / n;
    double rate = 0, rdev = 0, pdev = 0;
    for (const auto & a : att_) {
      if (a.t < now - p.hold_s) {continue;}
      rate = std::max({rate, std::fabs(a.rr), std::fabs(a.pr)});
      rdev = std::max(rdev, std::fabs(a.roll - mr));
      pdev = std::max(pdev, std::fabs(a.pitch - mp));
    }
    s.rate_max_dps = rate; s.roll_dev_deg = rdev; s.pitch_dev_deg = pdev;
    if (rate > p.rate_max_dps) {s.why = "rocking"; return s;}
    if (pdev > p.band_deg) {s.why = "pitch swinging"; return s;}
    if (rdev > p.band_deg) {s.why = "roll swinging"; return s;}
    if (!std::isfinite(s.quiet_s) || s.quiet_s < p.quiet_s) {s.why = "thrust too recent"; return s;}
    s.steady = true;
    return s;
  }

  void reset() {att_.clear(); last_thrust_t_ = -1.0;}

private:
  struct A {double t, roll, pitch, rr, pr;};
  std::deque<A> att_;
  double last_thrust_t_ = -1.0;
};

// ------------------------------------------------------------------ the gate

// "Ready" must HOLD, not flicker: a burst fires only after every condition has
// been true for hold_s without a break.
struct SolutionGate
{
  double hold_s = 1.0;
  double since = -1.0;

  bool update(double t, bool ok)
  {
    if (!ok) {since = -1.0; return false;}
    if (since < 0) {since = t;}
    return t - since >= hold_s;
  }
  double held(double t) const {return since < 0 ? 0.0 : t - since;}
  void reset() {since = -1.0;}
};

inline bool inBand(double range_m, double target_m, double tol_m)
{
  return std::isfinite(range_m) && std::isfinite(target_m) && std::fabs(range_m - target_m) <= tol_m;
}

inline bool aimed(double heading_deg, double target_deg, double tol_deg)
{
  return std::isfinite(heading_deg) && std::isfinite(target_deg) &&
         std::fabs(wrap180(target_deg - heading_deg)) <= tol_deg;
}

// Shots fired, and when the next may go. `blank_until` is when the wall filter
// may trust the LiDAR again: the burst plus the water's flight.
struct BurstBook
{
  int fired = 0;
  double last_end_t = -1e18;

  bool ready(double t, double gap_s) const {return t - last_end_t >= gap_s;}

  // returns blank_until
  double record(double t_start, double duration_s, double flight_s)
  {
    ++fired;
    last_end_t = t_start + duration_s;
    return t_start + duration_s + flight_s;
  }
  void reset() {fired = 0; last_end_t = -1e18;}
};

}  // namespace fire
}  // namespace crusader_bt

#endif  // CRUSADER_BT__FIRE_MATH_HPP_
