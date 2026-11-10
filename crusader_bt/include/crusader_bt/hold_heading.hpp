// hold_heading.hpp — the decision behind the HoldHeading leaf. Header only, stdlib + nav_math.
//
// HoldHeading keeps the boat near the point P where it started, pointed at a set compass heading.
// GUIDED's position setpoint ignores yaw, so the leaf uses the heading+speed command instead
// (/crsd/guided_heading_speed): speed 0 turns the boat to the heading and stops it, but does
// not hold position. Wind and current push it off P; when it has drifted past radius_m the
// command becomes "head back to P at a gentle speed", and once back inside radius_m/2 it
// turns to the set heading again. The hysteresis keeps the boat from chattering at the edge.
#ifndef CRUSADER_BT__HOLD_HEADING_HPP_
#define CRUSADER_BT__HOLD_HEADING_HPP_

#include <algorithm>
#include <cmath>

#include "crusader_bt/nav_math.hpp"

namespace crusader_bt
{
namespace holdheading
{

/// The bridge clamps heading+speed at hs_max_speed_mps (0.4); never ask for more.
constexpr double kMaxReturnSpeedMps = 0.4;
/// Return speed = min(return_speed_mps, kGainPerS * distance): slows into P, 0.3 m/s at 1 m.
constexpr double kGainPerS = 0.3;

struct Params
{
  double heading_deg = 0.0;          ///< the heading to hold, deg true
  double radius_m = 2.0;             ///< start returning beyond this
  double return_speed_mps = 0.3;     ///< the fastest the return goes
};

struct Command
{
  double heading_deg;   ///< compass deg [0, 360)
  double speed_mps;     ///< 0 = hold the heading, stopped
  bool returning;       ///< the hysteresis state to carry to the next tick
};

/// Compass degrees folded into [0, 360).
inline double wrap360(double deg)
{
  double d = std::fmod(deg, 360.0);
  return d < 0.0 ? d + 360.0 : d;
}

/// The command for one tick. `returning` = the previous tick's Command::returning (false at
/// start). Beyond radius_m it returns; once returning it keeps returning until inside
/// radius_m / 2; otherwise it holds heading_deg with speed 0.
inline Command decide(const nav::Vec2 & boat, const nav::Vec2 & anchor, bool returning, const Params & p)
{
  const double dist = nav::norm(anchor - boat);
  const bool back = returning ? dist >= 0.5 * p.radius_m : dist > p.radius_m;
  if (!back) {return {wrap360(p.heading_deg), 0.0, false};}
  const double v = std::min({p.return_speed_mps, kMaxReturnSpeedMps, kGainPerS * dist});
  return {nav::bearingDeg(boat, anchor), std::max(0.0, v), true};
}

}  // namespace holdheading
}  // namespace crusader_bt

#endif  // CRUSADER_BT__HOLD_HEADING_HPP_
