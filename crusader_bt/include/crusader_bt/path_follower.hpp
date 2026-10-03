// path_follower.hpp — drive a dense polyline with GUIDED position setpoints, and know how far
// along it the boat is.
//
// Header-only, pure (no ROS, no clock). Pinned by test/test_global_passage.cpp, which closes
// the loop with a kinematic boat and drives whole plans.
//
// WHY NOT path::carrot(). Two things it was never asked to do. An ORBIT ends where it began,
// and closestIndex() searches the whole remaining path, so a boat at the start of a lap is
// "closest" to the end of it and skips the lap; here progress is searched only in a short
// window round where the boat already is, and never runs backwards. And its carrot sits at
// least lookahead_min (3 m) ahead, which on a 2-3 m orbit is a chord through the buoy.
//
// HOW ARDUROVER TAKES A SETPOINT (4.6, GUID_OPTIONS = 0, on the boat and in SITL alike). A
// position target in GUIDED goes to AR_WPNav::set_desired_location_expect_fast_update(): the
// position controller's input shaping runs a target straight at the new point at up to
// WP_SPEED and decelerates to stop ON it, and "reached" needs that target within 1 cm of the
// point. So a point 1.5 m ahead is driven to, not declared reached at WP_RADIUS, and a new
// point sent before then simply re-aims the boat. That is what makes a short lookahead
// possible: pure pursuit 1.5 m ahead, re-sent each resend_m the target moves (~3 Hz at 1 m/s).
//
// PURE PURSUIT CUTS INSIDE A CURVE. Aiming at a point L ahead on a circle of radius R settles
// on radius sqrt(R^2 - L^2): at R 3, L 1.5 that is 0.4 m inside, toward the buoy being
// circled. The target is therefore pushed OUTWARD by kappa L^2 / 2 (where the tangent line
// would be at arc L), capped at max_comp_m, and only where the pushed point is itself clear.
#ifndef CRUSADER_BT__PATH_FOLLOWER_HPP_
#define CRUSADER_BT__PATH_FOLLOWER_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "crusader_bt/global_passage.hpp"
#include "crusader_bt/nav_math.hpp"
#include "crusader_bt/path_math.hpp"

namespace crusader_bt
{
namespace gp
{

struct FollowParams
{
  double lookahead_m = 1.5;     ///< pure pursuit, along the path
  double resend_m = 0.3;        ///< re-send once the target has moved this far
  double end_tol_m = 0.8;       ///< arrival at the last point (a stop)
  double pass_early_m = 0.0;    ///< > 0: done this far before the end, without stopping
  double check_clear_m = 0.7;   ///< the path ahead must keep this much; less = blocked
  double check_ahead_m = 10.0;
  double max_comp_m = 0.6;      ///< curvature lead cap
  double window_back_m = 1.0;   ///< progress search window round the last progress
  double window_fwd_m = 4.0;
  double max_offpath_m = 4.0;   ///< further than this from the path, progress does not move
};

struct FollowOut
{
  bool send = false;
  Vec2 setpoint;
  bool arrived = false;
  bool blocked = false;
  double s = 0.0, total = 0.0;
  double offpath = 0.0;
  double clear_ahead = std::numeric_limits<double>::infinity();
  Vec2 target;                  ///< what would be sent (sent or not)
};

class Follower
{
public:
  explicit Follower(FollowParams p = FollowParams{}) : p_(p) {}

  void setParams(const FollowParams & p) {p_ = p;}
  const FollowParams & params() const {return p_;}

  void reset(std::vector<Vec2> path)
  {
    path_ = std::move(path);
    cum_ = arcLengths(path_);
    s_ = 0.0;
    have_sent_ = false;
  }

  /// After a hold: the next target goes out whatever the deadband says.
  void forgetSent() {have_sent_ = false;}

  bool empty() const {return path_.empty();}
  double s() const {return s_;}
  double total() const {return cum_.empty() ? 0.0 : cum_.back();}
  const std::vector<Vec2> & path() const {return path_;}
  Vec2 at(double s) const {return pointAt(path_, cum_, s);}

  /// The path from the boat's progress on.
  std::vector<Vec2> remaining() const {return slice(path_, s_, total());}

  FollowOut step(Vec2 boat, const std::vector<Hazard> & hz)
  {
    FollowOut o;
    o.total = total();
    if (path_.empty()) {o.arrived = true; return o;}
    if (path_.size() == 1) {
      o.target = path_[0];
      o.arrived = nav::norm(boat - path_[0]) <= p_.end_tol_m;
      sendRule(o, true);
      return o;
    }

    // 1. progress: the nearest point in a window round the last one, forward only
    double best_d = std::numeric_limits<double>::infinity(), best_s = s_;
    for (std::size_t i = 1; i < path_.size(); ++i) {
      if (cum_[i] < s_ - p_.window_back_m) {continue;}
      if (cum_[i - 1] > s_ + p_.window_fwd_m) {break;}
      const Vec2 a = path_[i - 1], ab = path_[i] - a;
      const double l2 = nav::dot(ab, ab);
      const double t = l2 < 1e-12 ? 0.0 : std::clamp(nav::dot(boat - a, ab) / l2, 0.0, 1.0);
      const double d = nav::norm(boat - (a + ab * t));
      if (d < best_d) {best_d = d; best_s = cum_[i - 1] + t * std::sqrt(l2);}
    }
    o.offpath = best_d;
    if (best_d <= p_.max_offpath_m && best_s > s_) {s_ = best_s;}
    o.s = s_;

    // 2. arrival
    const bool pass = p_.pass_early_m > 0.0;
    const double early = pass ? std::max(p_.end_tol_m, p_.pass_early_m) : p_.end_tol_m;
    if (o.total - s_ <= early + 1e-9 && nav::norm(boat - path_.back()) <= early + (pass ? 0.5 : 0.0)) {
      o.arrived = true;
      o.target = path_.back();
      return o;
    }

    // 3. the path ahead
    for (std::size_t i = 0; i < path_.size(); ++i) {
      if (cum_[i] < s_ - 1e-9) {continue;}
      if (cum_[i] > s_ + p_.check_ahead_m) {break;}
      o.clear_ahead = std::min(o.clear_ahead, path::minClearance(hz, path_[i]));
    }
    o.blocked = o.clear_ahead < p_.check_clear_m;

    // 4. the target
    const double remaining = o.total - s_;
    bool is_end = false;
    if (remaining <= p_.lookahead_m) {
      o.target = path_.back();
      is_end = true;
    } else {
      o.target = aim(boat, hz);
    }
    sendRule(o, is_end);
    return o;
  }

private:
  /// Pure pursuit lookahead_m ahead with the curvature lead; shorter when the chord from the
  /// boat would pass closer than check_clear_m to anything.
  Vec2 aim(Vec2 boat, const std::vector<Hazard> & hz) const
  {
    const double total = this->total();
    for (const double f : {1.0, 0.75, 0.5}) {
      const double st = std::min(s_ + f * p_.lookahead_m, total);
      const Vec2 base = at(st);
      const Vec2 lead = base + outward(st) * (f * f);
      for (const Vec2 cand : {lead, base}) {
        if (!(path::minClearance(hz, cand) < p_.check_clear_m) &&
          path::segmentClear(boat, cand, hz, p_.check_clear_m, 0.1))
        {
          return cand;
        }
      }
    }
    // Nothing clear from where the boat is (it is off the path beside something): the path
    // itself is clear, so head for it just ahead.
    return at(std::min(s_ + 0.5, total));
  }

  /// kappa L^2 / 2, pointing away from the centre of curvature at arc s, capped.
  Vec2 outward(double s) const
  {
    const double h = 0.5;
    const double total = this->total();
    if (total < 2.0 * h) {return {};}
    const double sm = std::clamp(s, h, total - h);
    const Vec2 a = at(sm - h), b = at(sm), c = at(sm + h);
    const double ab = nav::norm(b - a), bc = nav::norm(c - b), ac = nav::norm(c - a);
    if (ab < 1e-6 || bc < 1e-6 || ac < 1e-6) {return {};}
    const double kappa = 2.0 * nav::cross(b - a, c - b) / (ab * bc * ac);   // + = turning left
    const Vec2 left = nav::portOf(c - a);
    double off = kappa * p_.lookahead_m * p_.lookahead_m * 0.5;
    off = std::clamp(off, -p_.max_comp_m, p_.max_comp_m);
    return left * (-off);
  }

  void sendRule(FollowOut & o, bool is_end)
  {
    const double moved = have_sent_ ? nav::norm(o.target - last_sent_) :
      std::numeric_limits<double>::infinity();
    if (!have_sent_ || moved >= p_.resend_m || (is_end && moved > 0.05)) {
      o.send = true;
      o.setpoint = o.target;
      last_sent_ = o.target;
      have_sent_ = true;
    }
  }

  FollowParams p_;
  std::vector<Vec2> path_;
  std::vector<double> cum_;
  double s_ = 0.0;
  bool have_sent_ = false;
  Vec2 last_sent_;
};

}  // namespace gp
}  // namespace crusader_bt

#endif  // CRUSADER_BT__PATH_FOLLOWER_HPP_
