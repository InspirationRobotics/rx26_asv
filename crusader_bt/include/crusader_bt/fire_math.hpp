// fire_math.hpp — Task 3 on the sticks (MANUAL): the docked hold and its gate. Every
// number in it, and nothing else.
//
// Header-only, stdlib only, no ROS, no BehaviorTree.CPP (same contract as
// nav_math.hpp), so it runs on a laptop in a second:
//
//     g++ -std=c++17 -O2 -I include -o t test/test_fire_math.cpp && ./t
//
//   the wall      /crsd/wall_range, median-filtered.
//   steady        the hull's roll and pitch, and no thrust recently.
//   the hold      the sticks toward: the bay's face at the firing range (surge), the
//                 bay on the bow (sway), the bow square to the face (yaw). This OmniX
//                 hull strafes in MANUAL, so the bow stays square and the boat slides.
//   the gate      all of the above held for hold_s.
//
// The pan/tilt cannon aims itself (telemetry_bridge, cannon_core): the boat only has to
// face and centre the bay.
//
// CONVENTIONS (the same as dock_math):
//
//   Heading is compass degrees, clockwise from north. Body frame is REP-103:
//   x FORWARD, y LEFT, so "turn left" makes the compass heading SMALLER.
//   WallRange.angle_deg is the body bearing of the wall's nearest point, + LEFT.
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

// The wall as the shot sees it: medians over the last window_s of VALID samples
// outside the blanking interval. A burst blanks the filter from its start until the
// water has landed (BurstBook sets it): the stream returns LiDAR points short of the
// wall and would walk the boat forward into its own spray.
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

  // Compass bearing of the wall's inward normal from each sample's own heading
  // (heading - angle), so a turn in progress does not smear it: a circular median
  // (offsets from the newest, re-anchored).
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

// ------------------------------------------------------------------ the cross-check

// The perpendicular distance from a point (the boat) to a face plane, given a
// point on the face and its OUTWARD unit normal: + = in front of the face.
inline double planeDistance(double be, double bn, double pe, double pn, double oe, double on)
{
  return (be - pe) * oe + (bn - pn) * on;
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
  double fire_range_m = 3.22;    // LiDAR range to fire from (squirt_cal)
  double deadband_range_m = 0.05;
  double deadband_lat_m = 0.04;
  // Square matters little once the aim is by strafing: the window's y in the
  // BODY frame is where the stream lands whatever the heading (3 deg off square
  // changes the run by ~4 mm). A tight yaw band only turns camera noise into
  // thrust, and thrust into rocking.
  double deadband_yaw_deg = 3.0;
  // dp_hold's tune on this hull: 90 us/m forward and lateral, 4 us/deg yaw with
  // 3 us per deg/s of damping, 120 us cap. The D terms on range and lateral are
  // new: MANUAL has no autopilot speed loop to stop an overshoot.
  double kp_fwd = 90.0, kd_fwd = 60.0;     // us per m, us per m/s
  double kp_lat = 90.0, kd_lat = 30.0;
  double kp_yaw = 4.0, kd_yaw = 3.0;       // us per deg, us per deg/s
  // I on range and lateral: a steady current needs a steady push, and P alone
  // only gives one at an error. Integrated only when STALLED (within i_zone_m
  // and nearly still) so the approach does not wind it up into an overshoot.
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

// One axis: nothing inside the band; outside it P + D, with the thruster deadband
// COMPENSATED: min_us is ADDED to every non-zero command, not used as a floor (Blue
// Robotics ESCs do nothing within +-25 us of neutral, so a floor left a 0.5 m error
// with next to no thrust). Capped at max_us. COASTING: already moving toward the
// target fast enough to arrive within coast_s, it pushes no more (a kick there makes
// it hunt) unless D says brake.
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
    const double pd_lat = -axisLaw(in.lat_err_m, in.lat_rate, p.kp_lat, p.kd_lat,
        p.deadband_lat_m, p.min_us, p.max_us);
    c.correcting = yawing || pd_fwd != 0.0 || pd_lat != 0.0;
    const double h = std::max(0.0, dt);
    auto still = [&p](double r) {return !std::isfinite(r) || std::fabs(r) < p.i_rate_mps;};
    if (std::isfinite(rerr) && std::fabs(rerr) < p.i_zone_m && still(in.range_rate)) {
      st.i_fwd = std::clamp(st.i_fwd + p.ki_fwd * rerr * h, -p.i_max_us, p.i_max_us);
    }
    if (std::isfinite(in.lat_err_m) && std::fabs(in.lat_err_m) < p.i_zone_m && still(in.lat_rate)) {
      st.i_lat = std::clamp(st.i_lat - p.ki_lat * in.lat_err_m * h, -p.i_max_us, p.i_max_us);
    }
    want.fwd_us = std::isfinite(rerr) ? std::clamp(pd_fwd + st.i_fwd, -p.max_us, p.max_us) : 0.0;
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
  // Slew-limit only GROWING thrust. Less is allowed at once, above all at the
  // minimum range, where a ramp down is thrust toward the dock after the floor
  // said none.
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

}  // namespace fire
}  // namespace crusader_bt

#endif  // CRUSADER_BT__FIRE_MATH_HPP_
