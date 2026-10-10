// fire_math.hpp — the fixed-nozzle shot: hold a range and a heading, wait until
// the boat is still, fire. Every number in it, and nothing else.
//
// Same contract as nav_math.hpp and dock_math.hpp: header-only, stdlib only, no
// ROS, no BehaviorTree.CPP, so it runs on a laptop in a second:
//
//     g++ -std=c++17 -O2 -I include -o t test/test_fire_math.cpp && ./t
//
// WHY A NOZZLE NEEDS ITS OWN MATHS. The Task 3 nozzle is fixed (30 deg, no pan,
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
// TWO WAYS TO HOLD THE SPOT. The first (solveAim / stationKeep) is for GUIDED
// heading + speed, which cannot strafe: left/right aim is by TURNING. The
// second (the strafe keep, below) is for MANUAL on this OmniX hull, which can:
// the bow stays square to the face, the boat SLIDES until the window is on the
// nozzle's line, and range is the LiDAR's. Its left/right and its square-up
// both come from the CAMERA (DockObservation window x,y,z and the face plane),
// not from the LiDAR's wall angle: the camera sees the window itself, the
// LiDAR only a wall that may not be flat.
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
#include <array>
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <string>
#include <utility>
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

  // d(range)/dt [m/s] over the last `window` s of usable samples (least
  // squares): the surge axis's damping in MANUAL, where no autopilot speed loop
  // does it. NaN with too few.
  double rate(double now, double window = 0.8) const
  {
    double st = 0, sv = 0, stt = 0, stv = 0; int n = 0;
    for (const auto & s : buf_) {
      if (!s.valid || s.t < now - window || s.t <= blank_until_ || s.t > now + 1e-9 ||
        !std::isfinite(s.range_m)) {continue;}
      const double t = s.t - now;
      st += t; sv += s.range_m; stt += t * t; stv += t * s.range_m; ++n;
    }
    if (n < 4) {return kNaN;}
    const double den = n * stt - st * st;
    return std::fabs(den) < 1e-12 ? kNaN : (n * stv - st * sv) / den;
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
  double fire_range_m = 1.4;    // the calibrated wall range for this window (30 deg nozzle)
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
  // Slew-limit only a GROWING speed; less is allowed at once (the autopilot
  // brakes at its own ATC_ACCEL_MAX), so the floor is never ramped through.
  const double step = std::max(0.0, p.accel * dt);
  if (want * prev_speed < 0.0) {c.speed_mps = std::clamp(want, -step, step);}
  else if (std::fabs(want) <= std::fabs(prev_speed)) {c.speed_mps = want;}
  else {c.speed_mps = std::clamp(want, prev_speed - step, prev_speed + step);}
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

// ------------------------------------------------------------------ the camera, for the strafe keep

// A number over time: medians, a least-squares slope, an age. Compass
// headings go in `circular` series so 359 and 1 are 2 degrees apart.
class Series
{
public:
  explicit Series(bool circular = false) : circular_(circular) {}

  void add(double t, double v)
  {
    if (!std::isfinite(v)) {return;}
    buf_.push_back({t, v});
    while (!buf_.empty() && buf_.front().first < t - 10.0) {buf_.pop_front();}
  }

  double age(double now) const
  {
    return buf_.empty() ? std::numeric_limits<double>::infinity() : now - buf_.back().first;
  }

  double median(double now, double window) const
  {
    std::vector<double> v;
    const double ref = buf_.empty() ? 0.0 : buf_.back().second;
    for (const auto & s : buf_) {
      if (s.first >= now - window && s.first <= now + 1e-9) {
        v.push_back(circular_ ? wrap180(s.second - ref) : s.second);
      }
    }
    if (v.empty()) {return kNaN;}
    const double m = fire::median(v);
    return circular_ ? wrap360(ref + m) : m;
  }

  double slope(double now, double window) const   // per second; not for circular
  {
    double st = 0, sv = 0, stt = 0, stv = 0; int n = 0;
    for (const auto & s : buf_) {
      if (s.first < now - window || s.first > now + 1e-9) {continue;}
      const double t = s.first - now;
      st += t; sv += s.second; stt += t * t; stv += t * s.second; ++n;
    }
    if (n < 4) {return kNaN;}
    const double den = n * stt - st * st;
    return std::fabs(den) < 1e-12 ? kNaN : (n * stv - st * sv) / den;
  }

  void reset() {buf_.clear();}

  /// The samples newer than t, oldest first: for a consumer that must see
  /// each one exactly once (the lateral estimator).
  std::vector<std::pair<double, double>> since(double t) const
  {
    std::vector<std::pair<double, double>> out;
    for (const auto & s : buf_) {if (s.first > t) {out.push_back(s);}}
    return out;
  }

private:
  bool circular_;
  std::deque<std::pair<double, double>> buf_;
};

// ------------------------------------------------------ the lateral estimator
//
// WHY. The strafe keep steered on the camera's window position in the BOW
// frame, a 0.6 s median, and a 0.8 s slope for the rate. On the water
// (2026-10-02) that held +-0.3-0.5 m: every degree of yaw wobble moves a window
// 1.6 m away ~3 cm sideways in the bow frame, so D braked against motion the
// hull never made; and the frames are 50-100 ms old on arrival.
//
// WHAT. Track the window's sideways offset AS IF THE BOW WERE SQUARE to the
// face (yaw taken out with the compass heading AT EACH FRAME's capture time),
// and its rate, with a constant-velocity Kalman filter; project it to now; then
// put the CURRENT yaw back for the aim (the nozzle is fixed, so the shot needs
// the window in the bow frame now). The rate handed to D is the square-frame
// rate: the hull's sideways motion, not the yaw's.

// Headings by time (compass degrees; the autopilot EKF's yaw - gyro plus the
// dual-antenna GPS yaw, the compass itself is off), so a frame is rotated with
// the heading the boat had when the frame was taken. Ordered; wrap-aware
// interpolation.
class HeadingHistory
{
public:
  void add(double t, double deg)
  {
    if (!std::isfinite(deg) || !std::isfinite(t)) {return;}
    if (!buf_.empty() && t <= buf_.back().first) {return;}
    buf_.push_back({t, wrap360(deg)});
    while (!buf_.empty() && buf_.front().first < t - 5.0) {buf_.pop_front();}
  }

  /// The heading at t: interpolated inside the span, the nearest end within
  /// max_gap_s outside it, NaN when nothing is that close.
  double at(double t, double max_gap_s = 0.5) const
  {
    if (buf_.empty() || !std::isfinite(t)) {return kNaN;}
    if (t <= buf_.front().first) {
      return buf_.front().first - t <= max_gap_s ? buf_.front().second : kNaN;
    }
    if (t >= buf_.back().first) {
      return t - buf_.back().first <= max_gap_s ? buf_.back().second : kNaN;
    }
    for (std::size_t i = 1; i < buf_.size(); ++i) {
      if (buf_[i].first >= t) {
        const auto & a = buf_[i - 1];
        const auto & b = buf_[i];
        const double f = (t - a.first) / std::max(b.first - a.first, 1e-9);
        return wrap360(a.second + f * wrap180(b.second - a.second));
      }
    }
    return kNaN;
  }

  void reset() {buf_.clear();}

private:
  std::deque<std::pair<double, double>> buf_;
};

// A window at (x_body, y_body) [m, REP-103: x ahead, y LEFT] seen with the bow
// yaw_err_deg LEFT of square (+ = turn right to square, StrafeInputs'
// convention): its sideways offset in the square frame. And back.
inline double squareOffset(double x_body, double y_body, double yaw_err_deg)
{
  const double d = yaw_err_deg * kDeg;
  return x_body * std::sin(d) + y_body * std::cos(d);
}

inline double bodyOffset(double x_body, double y_square, double yaw_err_deg)
{
  const double d = yaw_err_deg * kDeg;
  const double c = std::cos(d);
  return std::fabs(c) < 1e-3 ? kNaN : (y_square - x_body * std::sin(d)) / c;
}

struct LateralEstParams
{
  double q = 0.01;        // process noise (white acceleration), m^2/s^3: 0.03 m/s of
                          // rate noise at 2 cm camera noise, sees a 0.1 m/s start in 0.4 s
  double r = 0.03;        // camera noise per frame, m
  double gate = 4.0;      // innovation beyond this many sigma: rejected
  double v0_sigma = 0.3;  // rate uncertainty at (re)start, m/s
  int reinit_after = 3;   // that many rejections in a row: restart on the new value
};

// Constant-velocity Kalman filter on one coordinate. update() takes the
// measurements in time order (older ones are ignored); at() projects to any
// later time without changing the filter.
class LateralEstimator
{
public:
  void reset() {inited_ = false; n_rej_ = 0;}
  bool inited() const {return inited_;}
  double lastMeasT() const {return tm_;}
  int rejectedInARow() const {return n_rej_;}
  double sigmaP() const {return std::sqrt(std::max(P_[0][0], 0.0));}

  /// true = accepted (or restarted on it); false = rejected or out of order
  bool update(double t, double z, const LateralEstParams & k)
  {
    if (!std::isfinite(z) || !std::isfinite(t)) {return false;}
    if (!inited_) {init(t, z, k); return true;}
    if (t < t_ - 1e-9) {return false;}
    predict(t, k);
    const double S = P_[0][0] + k.r * k.r;
    const double y = z - x_[0];
    if (y * y > k.gate * k.gate * S) {
      if (++n_rej_ >= k.reinit_after) {init(t, z, k); return true;}
      return false;
    }
    n_rej_ = 0;
    const double K0 = P_[0][0] / S, K1 = P_[1][0] / S;
    x_[0] += K0 * y;
    x_[1] += K1 * y;
    const double p00 = P_[0][0], p01 = P_[0][1], p10 = P_[1][0], p11 = P_[1][1];
    P_[0][0] = (1.0 - K0) * p00;
    P_[0][1] = (1.0 - K0) * p01;
    P_[1][0] = p10 - K1 * p00;
    P_[1][1] = p11 - K1 * p01;
    tm_ = t;
    return true;
  }

  /// (position, rate) projected to `now`
  std::pair<double, double> at(double now) const
  {
    const double dt = std::max(0.0, now - t_);
    return {x_[0] + x_[1] * dt, x_[1]};
  }

private:
  void init(double t, double z, const LateralEstParams & k)
  {
    x_ = {z, 0.0};
    P_ = {{{k.r * k.r, 0.0}, {0.0, k.v0_sigma * k.v0_sigma}}};
    t_ = tm_ = t;
    inited_ = true;
    n_rej_ = 0;
  }

  void predict(double t, const LateralEstParams & k)
  {
    const double dt = t - t_;
    if (dt <= 0.0) {return;}
    x_[0] += x_[1] * dt;
    const double p00 = P_[0][0], p01 = P_[0][1], p10 = P_[1][0], p11 = P_[1][1];
    double n00 = p00 + dt * (p10 + p01) + dt * dt * p11;
    double n01 = p01 + dt * p11, n10 = p10 + dt * p11, n11 = p11;
    n00 += k.q * dt * dt * dt / 3.0;
    n01 += k.q * dt * dt / 2.0;
    n10 += k.q * dt * dt / 2.0;
    n11 += k.q * dt;
    P_ = {{{n00, n01}, {n10, n11}}};
    t_ = t;
  }

  bool inited_ = false;
  double t_ = 0.0, tm_ = -1e18;
  std::array<double, 2> x_{{0.0, 0.0}};
  std::array<std::array<double, 2>, 2> P_{};
  int n_rej_ = 0;
};

struct P3 {double x = kNaN, y = kNaN, z = kNaN;};

// camera_link -> base_link. The camera frame is REP-103 (x along the optical
// axis, y left, z up), pitched by `pitch_deg` (+ = aimed DOWN, as dock::Mount)
// and yawed by `yaw_deg` (+ = aimed LEFT), with its origin at (mx, my) in the
// body. `point` false rotates a DIRECTION (a plane normal) without moving it.
inline P3 camToBody(
  const P3 & c, double mx, double my, double yaw_deg, double pitch_deg, bool point = true)
{
  const double p = pitch_deg * kDeg, y = yaw_deg * kDeg;
  const double xl = c.x * std::cos(p) + c.z * std::sin(p);     // undo the pitch
  const double zl = -c.x * std::sin(p) + c.z * std::cos(p);
  P3 b;
  b.x = xl * std::cos(y) - c.y * std::sin(y);                  // undo the yaw
  b.y = xl * std::sin(y) + c.y * std::cos(y);
  b.z = zl;
  if (point) {b.x += mx; b.y += my;}
  return b;
}

// How far to turn LEFT (deg) to square the bow to the face, from the two
// windows of ONE bay in ONE frame, in the body frame: square on, the upper-
// left window is straight LEFT of the lower-right one. NaN if their spacing
// is not the face's (expect_m +- tol_m): then one of them is not what the
// detector says it is.
inline double squareFromWindows(
  double ulx, double uly, double lrx, double lry, double expect_m, double tol_m)
{
  const double dx = ulx - lrx, dy = uly - lry;
  const double sep = std::hypot(dx, dy);
  if (!std::isfinite(sep) || std::fabs(sep - expect_m) > tol_m || dy <= 0.0) {return kNaN;}
  return std::atan2(-dx, dy) / kDeg;
}

// The same, from the face's plane normal in the body frame (pointing AT the
// camera, as DockBay has it): square on it is (-1, 0).
inline double squareFromNormal(double nx, double ny)
{
  if (!std::isfinite(nx) || !std::isfinite(ny) || -nx < 0.5) {return kNaN;}
  return std::atan2(-ny, -nx) / kDeg;
}

// ------------------------------------------------------------------ the strafe keep (MANUAL)

// Stick deflections in microseconds about each channel's neutral, in the
// sense dp_hold drove this hull: fwd + = ahead, lat + = to STARBOARD,
// yaw + = turn right. The runner maps them onto the RC channels.
struct Sticks
{
  double fwd_us = 0.0, lat_us = 0.0, yaw_us = 0.0;
  double max_abs() const {return std::max({std::fabs(fwd_us), std::fabs(lat_us), std::fabs(yaw_us)});}
};

struct StrafeParams
{
  double fire_range_m = 1.4;     // LiDAR range to fire from (30 deg nozzle, upper-left)
  double deadband_range_m = 0.05;
  double deadband_lat_m = 0.04;
  // Square matters little once the aim is by strafing: the window's y in the
  // BODY frame is where the stream lands whatever the heading, and 3 deg off
  // square changes the stream's run by ~4 mm. A tight yaw band only makes the
  // camera's noise into thrust, and thrust into rocking.
  double deadband_yaw_deg = 3.0;
  // dp_hold's tune on this hull: 90 us/m forward and lateral, 4 us/deg yaw with
  // 3 us per deg/s of damping, 120 us cap. The D terms on range and lateral are
  // new: MANUAL has no autopilot speed loop to stop an overshoot.
  double kp_fwd = 90.0, kd_fwd = 60.0;     // us per m, us per m/s
  double kp_lat = 90.0, kd_lat = 30.0;
  double kp_yaw = 4.0, kd_yaw = 3.0;       // us per deg, us per deg/s
  // I on range and lateral: a steady current or wind needs a steady push, and
  // P alone only gives one at an error - 0.65 m of it for 12 cm/s (the sim).
  // Integrated only when STALLED - within i_zone_m and nearly still: held off,
  // not arriving - so the approach does not wind it up into an overshoot.
  // Capped.
  double ki_fwd = 20.0, ki_lat = 30.0;     // us per m.s
  double i_max_us = 80.0;
  double i_zone_m = 1.0;
  double i_rate_mps = 0.03;
  double min_us = 30.0;          // ADDED to every correction: the ESC deadband (+-25) and a bit
  double max_us = 120.0;
  double slew_us_s = 200.0;      // per axis: gentle, thrust rocks the hull
  double min_range_m = 1.5;      // never push AHEAD inside this
  double square_first_deg = 10.0;  // further off square than this: turn only
  // Lateral only. coast_lat_s: axisLaw's coasting window (0 = never coast - on
  // water, drag stops the hull short of a target it was "coasting" to, then a
  // fresh kick; 2026-10-02). lat_min_us: the deadband offset for the lateral
  // axis alone (-1 = min_us): this hull barely slides below ~50 us.
  double coast_lat_s = 1.5;
  double lat_min_us = -1.0;
  // Surge and sway: inside the deadband the law commands NOTHING, so a hull
  // with little drag coasts straight through it and gets a fresh kick from the
  // other side - a limit cycle (the sim's Task 3 berth, 2026-10-05: +-11 cm about
  // 1.6 m, 9 s period, forever). brake_mps: in the band and still moving faster
  // than this, the D term alone brakes (deadband-compensated). NaN = never, the
  // fixed-nozzle trees' tune; SlotKeep sets it.
  double brake_mps = kNaN;
};

// The brake for one axis inside its band: -kd * rate, deadband-compensated and
// capped, when |rate| > brake; 0 otherwise. `rate` is d(err)/dt in axisLaw's
// sense (+ = the error growing).
inline double bandBrake(double rate, double kd, double brake, double min_us, double max_us)
{
  if (!std::isfinite(brake) || !std::isfinite(rate) || std::fabs(rate) <= brake) {return 0.0;}
  const double u = kd * rate;
  if (u == 0.0) {return 0.0;}
  return std::copysign(std::min(min_us + std::fabs(u), max_us), u);
}

// Live overrides of the keep's gains, from bt_runner_node's strafe.* parameters
// (the ground station's Tuning tab, or ros2 param set), so a tune on the water
// does not mean stopping the node and the run. NEGATIVE = the tree's own value:
// the tree file stays the default, and an override reads as a deviation from it
// in every strafe log line. Not reset between goals; it lives as long as the
// node, like any parameter. The two windows are the camera's smoothing: the
// window position is a median over window_median_s, its rate a fit over
// rate_window_s - at 12 Hz the 0.6 s median is ~0.3 s of lag in the loop.
struct StrafeTune
{
  double fire_range_m = -1.0;      // the LiDAR range held (StrafeKeep fire_range_m, SlotKeep standoff_m)
  double kp_fwd = -1.0, kd_fwd = -1.0, ki_fwd = -1.0;
  double kp_lat = -1.0, kd_lat = -1.0, ki_lat = -1.0;
  double kp_yaw = -1.0, kd_yaw = -1.0;
  double i_max_us = -1.0;
  double window_median_s = -1.0;   // tree default 0.6 s
  double rate_window_s = -1.0;     // tree default 0.8 s
  double coast_s = -1.0;           // lateral coasting window, tree default 1.5 s
  double lat_min_us = -1.0;        // lateral deadband offset, tree default = min_us
  // The lateral estimator (LateralEstimator): est_enable >= 0.5 switches the
  // window's sideways position and rate from the camera median/slope to it.
  double est_enable = -1.0;        // off unless set
  double est_q = -1.0;             // process noise, m^2/s^3 (default 0.01)
  double est_r = -1.0;             // camera noise per frame, m (default 0.03)
  double track_s = -1.0;           // ride through dropouts this long (default 1.0 s)

  bool estOn() const {return est_enable >= 0.5;}

  /// Overwrite sp's gains where an override is set; "" when none is, else the
  /// overridden ones as "kd_lat 80 kp_lat 60" for the log.
  std::string apply(StrafeParams & sp) const
  {
    const std::pair<const char *, std::pair<double, double *>> rows[] = {
      {"range", {fire_range_m, &sp.fire_range_m}},
      {"kp_fwd", {kp_fwd, &sp.kp_fwd}}, {"kd_fwd", {kd_fwd, &sp.kd_fwd}},
      {"ki_fwd", {ki_fwd, &sp.ki_fwd}}, {"kp_lat", {kp_lat, &sp.kp_lat}},
      {"kd_lat", {kd_lat, &sp.kd_lat}}, {"ki_lat", {ki_lat, &sp.ki_lat}},
      {"kp_yaw", {kp_yaw, &sp.kp_yaw}}, {"kd_yaw", {kd_yaw, &sp.kd_yaw}},
      {"i_max_us", {i_max_us, &sp.i_max_us}}, {"coast_s", {coast_s, &sp.coast_lat_s}},
      {"lat_min_us", {lat_min_us, &sp.lat_min_us}},
    };
    std::string out;
    char buf[40];
    for (const auto & r : rows) {
      if (r.second.first < 0.0) {continue;}
      *r.second.second = r.second.first;
      std::snprintf(buf, sizeof(buf), "%s%s %g", out.empty() ? "" : " ", r.first, r.second.first);
      out += buf;
    }
    for (const auto & w : {std::make_pair("median_s", window_median_s),
                           std::make_pair("rate_s", rate_window_s),
                           std::make_pair("est", est_enable), std::make_pair("est_q", est_q),
                           std::make_pair("est_r", est_r), std::make_pair("track_s", track_s)}) {
      if (w.second < 0.0) {continue;}
      std::snprintf(buf, sizeof(buf), "%s%s %g", out.empty() ? "" : " ", w.first, w.second);
      out += buf;
    }
    return out;
  }
};

struct StrafeInputs
{
  double range_m = kNaN;         // LiDAR, filtered
  double range_rate = kNaN;      // m/s, + = opening
  double lat_err_m = kNaN;       // the window, LEFT of the nozzle's line (+) [m]
  double lat_rate = kNaN;        // d(lat_err)/dt
  double yaw_err_deg = kNaN;     // turn RIGHT this much to be square (+)
  double yaw_rate_dps = kNaN;    // + = turning right (ATTITUDE.yawspeed)
};

struct StrafeCmd
{
  Sticks sticks;
  bool range_ok = false, lat_ok = false, yaw_ok = false;
  // any axis outside its band (P/D acting): the boat is being MOVED, which is
  // what rocks it. A steady holding push (I alone) is not correcting.
  bool correcting = false;
  std::string why;
};

// What the keep carries from tick to tick: the last sticks (for the slew) and
// the two integrators.
struct StrafeState
{
  Sticks prev;
  double i_fwd = 0.0, i_lat = 0.0;    // us; i_lat in the lateral stick's sense (+ starboard)
};

// One axis: nothing inside the band; outside it P + D, with the thruster
// deadband COMPENSATED - min_us is ADDED to every non-zero command, not used as
// a floor. Blue Robotics ESCs do nothing within +-25 us of neutral, so as a
// floor a 0.5 m error still asked for next to no thrust, and 5 cm/s of current
// held the boat there (the sim's fire_current). Capped at max_us. COASTING:
// already moving toward the target fast enough to arrive within coast_s, it
// pushes no more - a kick there is what makes it hunt - unless D says brake.
inline double axisLaw(double err, double derr, double kp, double kd, double db,
  double min_us, double max_us, double coast_s = 1.5)
{
  if (!std::isfinite(err) || std::fabs(err) <= db) {return 0.0;}
  const double d = std::isfinite(derr) ? derr : 0.0;
  const double u = kp * err + kd * d;
  if (err * d < 0.0 && std::fabs(err) < std::fabs(d) * coast_s && u * err > 0.0) {return 0.0;}
  if (u == 0.0) {return 0.0;}
  return std::copysign(std::min(min_us + std::fabs(u), max_us), u);
}

// The three sticks toward: range = fire_range (LiDAR), the window on the
// nozzle's line (camera, by strafing), the bow square to the face (camera).
// `st` carries the slew and the integrators; `dt` is since the last call.
inline StrafeCmd strafeKeep(
  const StrafeParams & p, const StrafeInputs & in, StrafeState & st, double dt)
{
  const Sticks prev = st.prev;
  StrafeCmd c;
  Sticks want;
  const double rerr = in.range_m - p.fire_range_m;             // + = too far out
  c.range_ok = std::isfinite(rerr) && std::fabs(rerr) <= p.deadband_range_m;
  c.lat_ok = std::isfinite(in.lat_err_m) && std::fabs(in.lat_err_m) <= p.deadband_lat_m;
  c.yaw_ok = std::isfinite(in.yaw_err_deg) && std::fabs(in.yaw_err_deg) <= p.deadband_yaw_deg;

  want.yaw_us = axisLaw(in.yaw_err_deg, std::isfinite(in.yaw_rate_dps) ? -in.yaw_rate_dps : kNaN,
      p.kp_yaw, p.kd_yaw, p.deadband_yaw_deg, p.min_us, p.max_us);
  c.correcting = want.yaw_us != 0.0;
  if (std::isfinite(in.yaw_err_deg) && std::fabs(in.yaw_err_deg) > p.square_first_deg) {
    c.why = "squaring up first";
  } else {
    const bool yawing = c.correcting;
    const double pd_fwd = axisLaw(rerr, in.range_rate, p.kp_fwd, p.kd_fwd, p.deadband_range_m,
        p.min_us, p.max_us);
    // window LEFT of the line -> slide left -> lateral stick NEGATIVE (+ = starboard)
    double pd_lat = -axisLaw(in.lat_err_m, in.lat_rate, p.kp_lat, p.kd_lat,
        p.deadband_lat_m, p.lat_min_us >= 0.0 ? p.lat_min_us : p.min_us, p.max_us, p.coast_lat_s);
    double pd_fwd_b = pd_fwd;
    if (pd_fwd_b == 0.0 && c.range_ok) {
      pd_fwd_b = bandBrake(in.range_rate, p.kd_fwd, p.brake_mps, p.min_us, p.max_us);
    }
    if (pd_lat == 0.0 && c.lat_ok) {
      pd_lat = -bandBrake(in.lat_rate, p.kd_lat, p.brake_mps,
          p.lat_min_us >= 0.0 ? p.lat_min_us : p.min_us, p.max_us);
    }
    c.correcting = yawing || pd_fwd_b != 0.0 || pd_lat != 0.0;
    const double h = std::max(0.0, dt);
    auto still = [&p](double r) {return !std::isfinite(r) || std::fabs(r) < p.i_rate_mps;};
    if (std::isfinite(rerr) && std::fabs(rerr) < p.i_zone_m && still(in.range_rate)) {
      st.i_fwd = std::clamp(st.i_fwd + p.ki_fwd * rerr * h, -p.i_max_us, p.i_max_us);
    }
    if (std::isfinite(in.lat_err_m) && std::fabs(in.lat_err_m) < p.i_zone_m && still(in.lat_rate)) {
      st.i_lat = std::clamp(st.i_lat - p.ki_lat * in.lat_err_m * h, -p.i_max_us, p.i_max_us);
    }
    want.fwd_us = std::isfinite(rerr) ? std::clamp(pd_fwd_b + st.i_fwd, -p.max_us, p.max_us) : 0.0;
    want.lat_us = std::isfinite(in.lat_err_m) ?
      std::clamp(pd_lat + st.i_lat, -p.max_us, p.max_us) : 0.0;
    if (want.fwd_us > 0.0 && std::isfinite(in.range_m) && in.range_m <= p.min_range_m) {
      want.fwd_us = 0.0;
      st.i_fwd = std::min(st.i_fwd, 0.0);
      c.why = "at the minimum range";
    }
    if (c.why.empty()) {
      if (!std::isfinite(rerr)) {c.why = "no range";}
      else if (!std::isfinite(in.lat_err_m)) {c.why = "window not seen: holding sideways";}
      else if (!std::isfinite(in.yaw_err_deg)) {c.why = "no face angle yet";}
      else if (c.range_ok && c.lat_ok && c.yaw_ok) {c.why = "on the spot";}
      else {c.why = "moving in";}
    }
  }
  // Slew-limit only GROWING thrust. Less thrust is always allowed at once -
  // above all at the minimum range, where a ramp down is thrust toward the
  // dock after the floor said none.
  const double step = std::max(0.0, p.slew_us_s * dt);
  auto slew = [step](double w, double pr) {
      double v;
      if (w * pr < 0.0) {v = std::clamp(w, -step, step);}                 // through zero
      else if (std::fabs(w) <= std::fabs(pr)) {v = w;}                    // easing off
      else {v = std::clamp(w, pr - step, pr + step);}                     // building up
      return std::fabs(v) < 1e-6 ? 0.0 : v;
    };
  c.sticks.fwd_us = slew(want.fwd_us, prev.fwd_us);
  c.sticks.lat_us = slew(want.lat_us, prev.lat_us);
  c.sticks.yaw_us = slew(want.yaw_us, prev.yaw_us);
  st.prev = c.sticks;
  return c;
}

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
