// fire_leaves.cpp — Task 3 on the sticks (MANUAL, RC override) with the pan/tilt cannon:
// slide along the dock to the GREEN bay, close in on it, hold off it while the cannon
// (telemetry_bridge, from SprayUntilHit's aim point) puts the fire out.
//
// Same rules as leaves.cpp and task3_leaves.cpp:
//
//   * Every calculation is in fire_math.hpp, tested off-ROS
//     (test/test_fire_math.cpp). A formula growing here belongs there.
//   * The leaves never subscribe: the runner folds /crsd/wall_range,
//     /crsd/attitude and the dock observations into the Context.
//   * Outputs (the sticks, the mode request) are called OUTSIDE ctx_->mu: the
//     runner's publish callbacks may take it.
//
// THE BAY, NOT A WINDOW. The boat faces and centres the committed bay (its place in
// the dock book: the camera's sightings in GPS coordinates, averaged); the cannon aims
// at the window itself. A missing input is a zero stick on that axis and a reason in
// the log, never a FAILURE: the run keeps going (no timeouts) until the pilot takes it.
//
// publish_setpoints off (Gate G1): everything is computed and logged, nothing moves.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

#include "crusader_bt/context.hpp"
#include "crusader_bt/fire_math.hpp"

namespace crusader_bt
{
namespace
{

/// The committed bay, as the boat sees it now: ahead and left of the body origin (m),
/// the turn to face it square (deg, + = right), and the distance to its face plane.
struct BayView
{
  bool ok = false;
  double ahead_m = fire::kNaN, left_m = fire::kNaN;
  double bearing_deg = fire::kNaN;      ///< + = LEFT of the bow
  double face_m = fire::kNaN;           ///< to the face plane, along its normal
  double square_deg = fire::kNaN;       ///< the heading that faces it square
  double off_centre_m = fire::kNaN;     ///< its centreline, + LEFT, square to its face
};

/// The camera saw the committed bay this recently (s): it is in the picture.
constexpr double kInViewS = 1.0;

/// CALL UNDER mu. Not ok with no committed bay or no fresh pose.
BayView bayView(const Context & c)
{
  BayView v;
  const dock::BayTrack * t = c.dock.find(c.chosen_track);
  if (t == nullptr || !c.pose_fresh || !std::isfinite(c.heading_deg)) {return v;}
  const nav::Vec2 fwd = nav::headingVec(c.heading_deg);
  const nav::Vec2 rel = t->p - c.boat;
  v.ahead_m = nav::dot(rel, fwd);
  v.left_m = nav::dot(rel, nav::portOf(fwd));
  v.bearing_deg = std::atan2(v.left_m, v.ahead_m) / fire::kDeg;
  const nav::Vec2 o = t->outward();
  v.face_m = fire::planeDistance(c.boat.x, c.boat.y, t->p.x, t->p.y, o.x, o.y);
  v.square_deg = nav::bearingDeg({0.0, 0.0}, o * -1.0);
  v.off_centre_m = nav::dot(rel, nav::portOf(nav::headingVec(v.square_deg)));
  v.ok = true;
  return v;
}

// ---------------------------------------------------------------- guards

/// The autopilot is in exactly this mode.
class ModeIs : public CrusaderCondition
{
public:
  ModeIs(const std::string & n, const BT::NodeConfig & c) : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("mode", "GUIDED", "the mode this subtree needs")};
  }
  BT::NodeStatus tick() override
  {
    std::string want = getInput<std::string>("mode").value_or("GUIDED");
    for (auto & ch : want) {ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));}
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->mode == want ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

/// The autonomy-drop latch (SD, ch9) is clear. The bridge refuses our sticks while it
/// is tripped; knowing it here ends the run instead of commanding into refusals.
class NotDropped : public CrusaderCondition
{
public:
  NotDropped(const std::string & n, const BT::NodeConfig & c) : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->drop_tripped ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

/// The LiDAR has seen a wall recently (use_lidar false: always true).
class WallRangeAlive : public CrusaderCondition
{
public:
  WallRangeAlive(const std::string & n, const BT::NodeConfig & c) : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("max_age_s", 1.0, "seconds without a VALID wall range"),
      BT::InputPort<bool>("use_lidar", true, "false: not needed, always true")};
  }
  BT::NodeStatus tick() override
  {
    if (!getInput<bool>("use_lidar").value_or(true)) {return BT::NodeStatus::SUCCESS;}
    const double max_age = getInput<double>("max_age_s").value_or(1.0);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->wall.valid_age(ctx_->now_s) <= max_age ? BT::NodeStatus::SUCCESS :
           BT::NodeStatus::FAILURE;
  }
};

/// Attitude is arriving: without it "steady" is a guess.
class AttitudeAlive : public CrusaderCondition
{
public:
  AttitudeAlive(const std::string & n, const BT::NodeConfig & c) : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("max_age_s", 0.5, "seconds without /crsd/attitude")};
  }
  BT::NodeStatus tick() override
  {
    const double max_age = getInput<double>("max_age_s").value_or(0.5);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->att_age_s <= max_age ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// ---------------------------------------------------------------- the docked hold

/// Hold off the committed bay on the STICKS: its face plane at fire_range_m (surge; the
/// LiDAR wall with use_lidar), its centre on the bow (sway) and the bow square to its
/// face (yaw; heading_deg, the dock's, until the bay's face is known). ALWAYS SUCCESS:
/// "not there yet" is a stick command and a reason, never a FAILURE.
class StrafeKeep : public CrusaderSyncAction
{
public:
  StrafeKeep(const std::string & n, const BT::NodeConfig & c) : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("fire_range_m", 1.2, "hold the bay's face this far off"),
      BT::InputPort<double>("heading_deg", fire::kNaN, "square to this until the face is known"),
      BT::InputPort<bool>("use_lidar", true, "the range from the LiDAR wall when it has one"),
      BT::InputPort<double>("deadband_range_m", 0.05, ""),
      BT::InputPort<double>("deadband_lat_m", 0.04, ""),
      BT::InputPort<double>("deadband_yaw_deg", 3.0, ""),
      BT::InputPort<double>("kp_fwd", 90.0, "us per m"),
      BT::InputPort<double>("kd_fwd", 60.0, "us per m/s"),
      BT::InputPort<double>("kp_lat", 90.0, "us per m"),
      BT::InputPort<double>("kd_lat", 30.0, "us per m/s"),
      BT::InputPort<double>("kp_yaw", 4.0, "us per deg"),
      BT::InputPort<double>("kd_yaw", 3.0, "us per deg/s"),
      BT::InputPort<double>("min_us", 30.0, "added to every correction: the ESC deadband"),
      BT::InputPort<double>("max_us", 120.0, "deflection cap (the bridge caps it too)"),
      BT::InputPort<double>("slew_us_s", 200.0, "per axis"),
      BT::InputPort<double>("min_range_m", 0.6, "never push ahead inside this"),
      BT::InputPort<double>("square_first_deg", 10.0, "further off square: turn only"),
      BT::InputPort<double>("ki_fwd", 20.0, "us per m.s (holds against a current)"),
      BT::InputPort<double>("ki_lat", 30.0, "us per m.s"),
      BT::InputPort<double>("i_max_us", 80.0, "integral cap"),
    };
  }

  BT::NodeStatus tick() override
  {
    fire::StrafeParams sp;
    auto in = [this](const char * k, double d) {return getInput<double>(k).value_or(d);};
    sp.fire_range_m = in("fire_range_m", 1.2);
    sp.deadband_range_m = in("deadband_range_m", sp.deadband_range_m);
    sp.deadband_lat_m = in("deadband_lat_m", sp.deadband_lat_m);
    sp.deadband_yaw_deg = in("deadband_yaw_deg", sp.deadband_yaw_deg);
    sp.kp_fwd = in("kp_fwd", sp.kp_fwd); sp.kd_fwd = in("kd_fwd", sp.kd_fwd);
    sp.kp_lat = in("kp_lat", sp.kp_lat); sp.kd_lat = in("kd_lat", sp.kd_lat);
    sp.kp_yaw = in("kp_yaw", sp.kp_yaw); sp.kd_yaw = in("kd_yaw", sp.kd_yaw);
    sp.ki_fwd = in("ki_fwd", sp.ki_fwd); sp.ki_lat = in("ki_lat", sp.ki_lat);
    sp.i_max_us = in("i_max_us", sp.i_max_us);
    sp.min_us = in("min_us", sp.min_us); sp.max_us = in("max_us", sp.max_us);
    sp.slew_us_s = in("slew_us_s", sp.slew_us_s);
    sp.min_range_m = in("min_range_m", 0.6);
    sp.square_first_deg = in("square_first_deg", sp.square_first_deg);
    const double set_h = in("heading_deg", fire::kNaN);
    const bool lidar = getInput<bool>("use_lidar").value_or(true);

    fire::StrafeCmd cmd;
    fire::StrafeInputs si;
    bool publish = false;
    std::string src = "no range";
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const double now = ctx_->now_s;
      const BayView b = bayView(*ctx_);
      const double dt = ctx_->last_strafe_t < 0 ? 0.1 : std::max(0.0, now - ctx_->last_strafe_t);
      ctx_->last_strafe_t = now;
      if (lidar && std::isfinite(ctx_->wall.range(now))) {
        si.range_m = ctx_->wall.range(now);
        si.range_rate = ctx_->wall.rate(now);
        src = "LiDAR";
      } else if (b.ok && b.face_m > 0.0) {
        si.range_m = b.face_m;
        src = "bay";
      }
      if (b.ok) {si.lat_err_m = b.left_m;}
      // rates from the last tick (the bay's place moves only as the boat does)
      if (dt > 0.0 && dt < 1.0) {
        auto rate = [dt](double now_v, double prev_v, double prev_r) {
            return std::isfinite(now_v) && std::isfinite(prev_v) ?
                   0.7 * (std::isfinite(prev_r) ? prev_r : 0.0) + 0.3 * (now_v - prev_v) / dt :
                   fire::kNaN;
          };
        if (src == "bay") {si.range_rate = rate(si.range_m, prev_.range_m, prev_.range_rate);}
        si.lat_rate = rate(si.lat_err_m, prev_.lat_err_m, prev_.lat_rate);
      }
      const double square = b.ok ? b.square_deg : set_h;
      if (std::isfinite(square) && std::isfinite(ctx_->heading_deg)) {
        si.yaw_err_deg = fire::wrap180(square - ctx_->heading_deg);
      }
      si.yaw_rate_dps = ctx_->yaw_rate_dps;
      cmd = fire::strafeKeep(sp, si, ctx_->strafe_state, dt);
      ctx_->strafe_state.prev = cmd.sticks;
      ctx_->strafe = cmd;
      ctx_->strafe_in = si;
      // "quiet" = not being moved: a steady holding push against a current
      // (the integrators alone) does not rock the hull, a correction does
      ctx_->steady.feed_cmd(now, cmd.correcting ? 1.0 : 0.0);
      publish = ctx_->publish_setpoints;
      if (publish) {ctx_->sticks_commanded = true;}
      if (ctx_->task3_phase.rfind("LINE UP", 0) != 0) {ctx_->task3_phase = "HOLD: " + cmd.why;}
    }
    prev_ = si;
    if (publish && ctx_->sticks) {
      ctx_->sticks(cmd.sticks.fwd_us, cmd.sticks.lat_us, cmd.sticks.yaw_us);
    }
    if (ctx_->node) {
      RCLCPP_INFO_THROTTLE(log(), *ctx_->node->get_clock(), 1000,
        "hold: range %.2f m (%s), bay %+.2f m left, square %+.1f deg | sticks fwd %+.0f "
        "lat %+.0f yaw %+.0f us (%s)%s",
        si.range_m, src.c_str(), si.lat_err_m, si.yaw_err_deg, cmd.sticks.fwd_us,
        cmd.sticks.lat_us, cmd.sticks.yaw_us, cmd.why.c_str(),
        publish ? "" : " [shadow: publish_setpoints is off]");
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  fire::StrafeInputs prev_;
};

/// Wait until the hold has held the spot for hold_s: range, the bay centred, square, the
/// hull still. RUNNING until then; FAILURE only after timeout_s (0 = never).
class AwaitStrafeSolution : public CrusaderAction
{
public:
  AwaitStrafeSolution(const std::string & n, const BT::NodeConfig & c) : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("range_tol_m", 0.06, "+- about fire_range_m"),
      BT::InputPort<double>("lat_tol_m", 0.05, "+- the bay off the bow"),
      BT::InputPort<double>("yaw_tol_deg", 5.0, "+- off square"),
      BT::InputPort<double>("fire_range_m", 1.2, "= StrafeKeep's"),
      BT::InputPort<double>("hold_s", 1.0, "everything true for this long"),
      BT::InputPort<double>("timeout_s", 0.0, "give up after this; 0 = never"),
    };
  }

  BT::NodeStatus onStart() override
  {
    gate_.hold_s = getInput<double>("hold_s").value_or(1.0);
    gate_.reset();
    std::lock_guard<std::mutex> lk(ctx_->mu);
    t0_ = ctx_->now_s;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const double rtol = getInput<double>("range_tol_m").value_or(0.06);
    const double ltol = getInput<double>("lat_tol_m").value_or(0.05);
    const double ytol = getInput<double>("yaw_tol_deg").value_or(5.0);
    const double target = getInput<double>("fire_range_m").value_or(1.2);
    const double timeout = getInput<double>("timeout_s").value_or(0.0);
    std::string why;
    bool held = false;
    double now = 0.0;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      now = ctx_->now_s;
      const fire::StrafeInputs & si = ctx_->strafe_in;
      const fire::SteadyStatus st = ctx_->steady.status(now);
      char buf[96];
      if (!std::isfinite(si.lat_err_m)) {
        why = "no bay committed or no pose";
      } else if (!std::isfinite(si.yaw_err_deg)) {
        why = "no face angle yet";
      } else if (!fire::inBand(si.range_m, target, rtol)) {
        std::snprintf(buf, sizeof(buf), "range %.2f, want %.2f +-%.2f", si.range_m, target, rtol);
        why = buf;
      } else if (std::fabs(si.lat_err_m) > ltol) {
        std::snprintf(buf, sizeof(buf), "bay %+.2f m off the bow", si.lat_err_m);
        why = buf;
      } else if (std::fabs(si.yaw_err_deg) > ytol) {
        std::snprintf(buf, sizeof(buf), "%+.1f deg off square", si.yaw_err_deg);
        why = buf;
      } else if (!st.steady) {
        why = "not steady: " + st.why;
      }
      held = gate_.update(now, why.empty());
      ctx_->task3_phase = why.empty() ? "LINE UP: holding" : "LINE UP: " + why;
    }
    if (held) {
      RCLCPP_INFO(log(), "on station, held %.1f s", gate_.held(now));
      return BT::NodeStatus::SUCCESS;
    }
    if (timeout > 0.0 && now - t0_ > timeout) {
      RCLCPP_WARN(log(), "not on station in %.0f s: %s", timeout, why.c_str());
      return BT::NodeStatus::FAILURE;
    }
    if (ctx_->node && !why.empty()) {
      RCLCPP_INFO_THROTTLE(log(), *ctx_->node->get_clock(), 2000, "lining up: %s", why.c_str());
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {gate_.reset();}

private:
  fire::SolutionGate gate_;
  double t0_ = 0.0;
};

/// Close in on the committed bay square to its face: strafe onto its centreline first
/// (no forward push while more than centre_tol_m off it), then a constant, smoothly
/// ramped push straight in while the sway keeps trimming. The bow holds square to the
/// bay's face (heading_deg, the dock's, until that is known). The push needs the camera
/// to see the bay; out of view it keeps strafing on the bay's remembered place. SUCCESS
/// at handover_m from the face (the LiDAR wall with use_lidar); never FAILURE.
class ApproachBay : public CrusaderAction
{
public:
  ApproachBay(const std::string & n, const BT::NodeConfig & c) : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("fwd_us", 60.0, "the forward stick once centred; also the sway cap"),
      BT::InputPort<double>("slew_us_s", 40.0, "forward ramp, both ways"),
      BT::InputPort<double>("centre_tol_m", 0.25, "this far off the centreline: strafe only"),
      BT::InputPort<double>("handover_m", 1.8, "SUCCESS this far from the face"),
      BT::InputPort<double>("heading_deg", fire::kNaN, "the dock's heading, until the face is known"),
      BT::InputPort<bool>("use_lidar", true, "the range from the LiDAR wall when it has one"),
      BT::InputPort<double>("kp_lat", 100.0, "us per m off the centreline"),
      BT::InputPort<double>("kp_yaw", 4.0, "us per deg"),
      BT::InputPort<double>("kd_yaw", 3.0, "us per deg/s"),
      BT::InputPort<double>("min_us", 30.0, "smallest stick that moves the hull"),
      BT::InputPort<double>("max_yaw_us", 80.0, "yaw stick cap"),
    };
  }

  BT::NodeStatus onStart() override
  {
    fwd_ = 0.0;
    std::lock_guard<std::mutex> lk(ctx_->mu);
    t_prev_ = ctx_->now_s;
    ctx_->task3_phase = "APPROACH";
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto in = [this](const char * k, double d) {return getInput<double>(k).value_or(d);};
    const double fwd_us = in("fwd_us", 60.0), min_us = in("min_us", 30.0);
    double yaw_us = 0.0, want_fwd = 0.0, lat_us = 0.0, range = fire::kNaN, off_c = fire::kNaN;
    bool publish = false, done = false;
    std::string why;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const double now = ctx_->now_s, dt = std::clamp(now - t_prev_, 0.0, 0.5);
      t_prev_ = now;
      const BayView b = bayView(*ctx_);
      range = b.face_m;
      if (getInput<bool>("use_lidar").value_or(true) && std::isfinite(ctx_->wall.range(now))) {
        range = ctx_->wall.range(now);
      }
      if (!b.ok) {
        why = "no bay or no pose: waiting";
      } else {
        const double square = std::isfinite(b.square_deg) ? b.square_deg : in("heading_deg", 0.0);
        const double off = fire::wrap180(square - ctx_->heading_deg);   // + = turn right
        const double rate = ctx_->yaw_rate_dps;
        yaw_us = fire::axisLaw(off, std::isfinite(rate) ? -rate : fire::kNaN, in("kp_yaw", 4.0),
          in("kd_yaw", 3.0), 3.0, min_us, in("max_yaw_us", 80.0));
        off_c = b.off_centre_m;                                // + = the centreline is LEFT
        lat_us = -fire::axisLaw(off_c, fire::kNaN, in("kp_lat", 100.0), 0.0, 0.05, min_us, fwd_us);
        const dock::BayTrack * t = ctx_->dock.find(ctx_->chosen_track);
        const bool in_view = ctx_->dock_t - t->last_seen <= kInViewS;
        if (std::isfinite(range) && range <= in("handover_m", 1.8)) {
          done = true;
        } else if (std::fabs(off_c) > in("centre_tol_m", 0.25)) {
          why = "strafing to the centreline";
        } else if (!in_view) {
          why = "centred, bay out of view: waiting for it";
        } else {
          want_fwd = fwd_us;
          why = "closing in";
        }
      }
      const double step = in("slew_us_s", 40.0) * dt;
      fwd_ = done ? 0.0 : std::clamp(want_fwd, fwd_ - step, fwd_ + step);
      publish = ctx_->publish_setpoints;
      if (publish) {ctx_->sticks_commanded = true;}
    }
    if (publish && ctx_->sticks) {
      ctx_->sticks(done ? 0.0 : fwd_, done ? 0.0 : lat_us, done ? 0.0 : yaw_us);
    }
    if (done) {
      RCLCPP_INFO(log(), "approach: %.2f m from the bay: handing over to the hold", range);
      return BT::NodeStatus::SUCCESS;
    }
    if (ctx_->node) {
      RCLCPP_INFO_THROTTLE(log(), *ctx_->node->get_clock(), 1000,
        "approach: bay %.2f m out, %+.2f m left of centre | fwd %+.0f lat %+.0f yaw %+.0f us "
        "(%s)%s", range, off_c, fwd_, lat_us, yaw_us, why.c_str(),
        publish ? "" : " [shadow: publish_setpoints is off]");
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    bool publish = false;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      publish = ctx_->publish_setpoints;
    }
    if (publish && ctx_->sticks) {ctx_->sticks(0.0, 0.0, 0.0);}
  }

private:
  double fwd_ = 0.0, t_prev_ = 0.0;
};

// ---------------------------------------------------------------- the waypoint run

/// The run's OWN switch from `from` (GUIDED) to `mode` (MANUAL), for the waypoint run:
/// GUIDED cannot strafe on this hull, so everything after the waypoint is MANUAL on the
/// sticks. Only FROM `from`: it never takes a mode the pilot chose. Asks on
/// /crsd/set_mode twice a second until the autopilot reports `mode`; FAILURE after
/// timeout_s or if the mode turns into anything else. Moving SC still takes the boat
/// back (the next ModeIs ends the run). Shadow (publish_setpoints off): asks nothing and
/// waits for the pilot to switch.
class RequestMode : public CrusaderAction
{
public:
  RequestMode(const std::string & n, const BT::NodeConfig & c) : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("mode", "MANUAL", "the mode to switch to"),
      BT::InputPort<std::string>("from", "GUIDED", "only from this mode"),
      BT::InputPort<double>("timeout_s", 10.0, "give up after this")};
  }

  BT::NodeStatus onStart() override
  {
    want_ = upper(getInput<std::string>("mode").value_or("MANUAL"));
    from_ = upper(getInput<std::string>("from").value_or("GUIDED"));
    std::string mode;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      t0_ = ctx_->now_s;
      mode = ctx_->mode;
      shadow_ = !ctx_->publish_setpoints;
      ctx_->task3_phase = "SWITCHING TO " + want_;
    }
    if (mode != want_ && mode != from_) {
      RCLCPP_WARN(log(), "RequestMode: the mode is %s, not %s: not switching a mode the "
        "pilot chose", mode.c_str(), from_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(log(), "switching %s -> %s%s", from_.c_str(), want_.c_str(),
      shadow_ ? " [shadow: not asking; waiting for the pilot]" : "");
    last_ask_ = -1e18;
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    double now = 0.0;
    std::string mode;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      now = ctx_->now_s;
      mode = ctx_->mode;
      if (mode == want_) {ctx_->handed_mode = want_;}   // planned, not the pilot taking over
    }
    if (mode == want_) {
      RCLCPP_INFO(log(), "mode is %s after %.1f s", want_.c_str(), now - t0_);
      return BT::NodeStatus::SUCCESS;
    }
    if (mode != from_ || now - t0_ > getInput<double>("timeout_s").value_or(10.0)) {
      RCLCPP_WARN(log(), "RequestMode: %s after %.0f s, not %s", mode.c_str(),
        now - t0_, want_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    if (!shadow_ && now - last_ask_ >= 0.5 && ctx_->request_mode) {
      last_ask_ = now;
      ctx_->request_mode(want_);
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  static std::string upper(std::string s)
  {
    for (auto & ch : s) {ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));}
    return s;
  }

  std::string want_, from_;
  double t0_ = 0.0, last_ask_ = 0.0;
  bool shadow_ = false;
};

/// Slide along the dock, bow square to its faces and at least min_out_m out, looking
/// for the GREEN bay: leg_m one way of the start, then leg_m the other way, then
/// FAILURE. Ended from outside (a ReactiveFallback on SafeBayKnown, or on
/// CommittedBayInView with first="committed": toward the committed bay). With no dock
/// layout in the book yet it slides across the bow and holds the start line.
class StrafeSearch : public CrusaderAction
{
public:
  StrafeSearch(const std::string & n, const BT::NodeConfig & c) : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("first", "left",
        "left | right: which way to slide first; committed: toward the committed bay"),
      BT::InputPort<double>("leg_m", 4.0, "slide this far each way of the start"),
      BT::InputPort<double>("lat_us", 70.0, "the lateral stick while sliding"),
      BT::InputPort<double>("kp_yaw", 5.0, "us per deg (heading hold)"),
      BT::InputPort<double>("kd_yaw", 0.5, "us per deg/s"),
      BT::InputPort<double>("min_us", 30.0, "smallest yaw / surge stick that moves the hull"),
      BT::InputPort<double>("max_us", 100.0, "yaw stick cap"),
      BT::InputPort<double>("min_out_m", 3.5, "hold at least this far out from the faces"),
      BT::InputPort<double>("kp_out", 80.0, "us per m (the distance hold)"),
      BT::InputPort<double>("kd_out", 200.0, "us per m/s"),
      BT::InputPort<double>("out_max_us", 80.0, "surge stick cap"),
      BT::InputPort<double>("timeout_s", 120.0, "FAILURE after this (not with forever)"),
      BT::InputPort<bool>("forever", false, "keep sweeping both ways; never FAILURE"),
      BT::InputPort<double>("heading_deg", fire::kNaN,
        "the bow's heading facing the dock (deg true, the EKF yaw): turn to it, slide across "
        "it; unset = the dock's layout, else the heading at the start"),
    };
  }

  BT::NodeStatus onStart() override
  {
    const double set_h = getInput<double>("heading_deg").value_or(fire::kNaN);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->pose_fresh || !ctx_->origin_set || !std::isfinite(ctx_->heading_deg)) {
      RCLCPP_WARN(log(), "StrafeSearch: no pose or heading to slide by");
      return BT::NodeStatus::FAILURE;
    }
    p0_ = ctx_->boat;
    const dock::DockLayout L = dock::layout(ctx_->dock, ctx_->dock_min_obs);
    if (L.ok) {
      right_ = L.right;
      out_ = L.out;
      face_ = L.centre;
      h0_ = nav::bearingDeg({0.0, 0.0}, L.out * -1.0);      // bow on the faces
      out_target_ = std::max(nav::dot(p0_ - face_, out_),
        getInput<double>("min_out_m").value_or(3.5));
    } else {
      h0_ = ctx_->heading_deg;
      right_ = nav::starboardOf(nav::headingVec(h0_));
      out_ = nav::headingVec(h0_) * -1.0;                   // the dock taken to be ahead
      face_ = p0_;
      out_target_ = 0.0;                                    // the start line
    }
    if (std::isfinite(set_h)) {                             // a heading given: it wins
      h0_ = set_h;
      right_ = nav::starboardOf(nav::headingVec(h0_));
      out_ = nav::headingVec(h0_) * -1.0;
      out_target_ = L.ok ? std::max(nav::dot(p0_ - face_, out_),
        getInput<double>("min_out_m").value_or(3.5)) : 0.0;
    }
    const std::string first = getInput<std::string>("first").value_or("left");
    toward_ = first == "committed";
    if (toward_) {
      const dock::BayTrack * t = ctx_->dock.find(ctx_->chosen_track);
      if (t == nullptr) {
        RCLCPP_WARN(log(), "StrafeSearch: first=\"committed\", but no bay is committed");
        return BT::NodeStatus::FAILURE;
      }
      dir_ = nav::dot(t->p - p0_, right_) >= 0.0 ? 1.0 : -1.0;
    } else {
      dir_ = first == "right" ? 1.0 : -1.0;
    }
    RCLCPP_INFO(log(), "StrafeSearch: %s, bow %.0f deg%s, sliding %s first",
      L.ok ? "along the dock" : "no dock layout yet: across the bow", h0_,
      std::isfinite(set_h) ? " (set)" : "", dir_ > 0.0 ? "right" : "left");
    leg_ = 0;
    t0_ = ctx_->now_s;
    t_prev_ = -1.0;
    out_rate_ = 0.0;
    anchored_ = L.ok;            // no layout: the start line is set once the turn is done
    ctx_->task3_phase = "SEARCH";
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const double leg_m = getInput<double>("leg_m").value_or(4.0);
    const double min_us = getInput<double>("min_us").value_or(30.0);
    double yaw_us = 0.0, fwd_us = 0.0, lat_us = 0.0, d = 0.0, out_err = 0.0, yaw_err = 0.0;
    bool publish = false, fail = false;
    std::string why;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const double now = ctx_->now_s;
      if (!ctx_->pose_fresh || !std::isfinite(ctx_->heading_deg)) {
        fail = true;
        why = "lost the pose";
      } else {
        d = nav::dot(ctx_->boat - p0_, right_);           // how far right of the start (+)
        const bool forever = getInput<bool>("forever").value_or(false);
        if (dir_ * d >= leg_m) {
          if (leg_ == 0 || forever) {
            dir_ = -dir_;
            leg_ = 1;
          } else {
            fail = true;
            why = toward_ ? "the committed bay never came into view" : "no GREEN bay along the dock";
          }
        }
        if (!forever && now - t0_ > getInput<double>("timeout_s").value_or(120.0)) {
          fail = true;
          why = "timed out";
        }
        yaw_err = fire::wrap180(h0_ - ctx_->heading_deg);   // + = turn right
        if (!anchored_ && std::fabs(yaw_err) <= 15.0) {     // the turn drifts the boat
          anchored_ = true;
          p0_ = ctx_->boat;
          face_ = ctx_->boat;
          d = 0.0;
        }
        const double rate = ctx_->yaw_rate_dps;
        yaw_us = fire::axisLaw(yaw_err, std::isfinite(rate) ? -rate : fire::kNaN,
          getInput<double>("kp_yaw").value_or(5.0), getInput<double>("kd_yaw").value_or(0.5),
          3.0, min_us, getInput<double>("max_us").value_or(100.0));
        out_err = nav::dot(ctx_->boat - face_, out_) - out_target_;   // + = too far out
        if (t_prev_ >= 0.0 && now > t_prev_) {
          out_rate_ = 0.7 * out_rate_ + 0.3 * (out_err - out_prev_) / (now - t_prev_);
        }
        t_prev_ = now;
        out_prev_ = out_err;
        if (std::fabs(yaw_err) <= 30.0) {
          fwd_us = fire::axisLaw(out_err, out_rate_, getInput<double>("kp_out").value_or(80.0),
            getInput<double>("kd_out").value_or(200.0), 0.25, min_us,
            getInput<double>("out_max_us").value_or(80.0));
        }
        // turn first, then slide
        lat_us = std::fabs(yaw_err) > 15.0 ? 0.0 :
          dir_ * getInput<double>("lat_us").value_or(70.0);
        publish = ctx_->publish_setpoints;
        if (publish) {ctx_->sticks_commanded = true;}
      }
    }
    if (publish && ctx_->sticks) {
      if (fail) {ctx_->sticks(0.0, 0.0, 0.0);} else {ctx_->sticks(fwd_us, lat_us, yaw_us);}
    }
    if (fail) {
      RCLCPP_WARN(log(), "StrafeSearch: %s (%.1f m %s of the start)", why.c_str(),
        std::fabs(d), d >= 0.0 ? "right" : "left");
      return BT::NodeStatus::FAILURE;
    }
    if (ctx_->node) {
      RCLCPP_INFO_THROTTLE(log(), *ctx_->node->get_clock(), 1000,
        "%s: %.1f m %s of the start, sliding %s | out %+.2f m, bow %+.0f deg off | "
        "sticks fwd %+.0f lat %+.0f yaw %+.0f us%s",
        toward_ ? "bringing the GREEN bay into view" : "searching for the GREEN bay",
        std::fabs(d), d >= 0.0 ? "right" : "left", dir_ > 0.0 ? "right" : "left",
        out_err, yaw_err, fwd_us, lat_us, yaw_us,
        publish ? "" : " [shadow: publish_setpoints is off]");
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    bool publish = false;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      publish = ctx_->publish_setpoints;
    }
    if (publish && ctx_->sticks) {ctx_->sticks(0.0, 0.0, 0.0);}
  }

private:
  double h0_ = 0.0, dir_ = -1.0, t0_ = 0.0;
  nav::Vec2 p0_, right_{1.0, 0.0}, out_{0.0, -1.0}, face_;
  double out_target_ = 0.0, out_prev_ = 0.0, out_rate_ = 0.0, t_prev_ = -1.0;
  int leg_ = 0;
  bool toward_ = false;      ///< first="committed": toward the committed bay
  bool anchored_ = false;    ///< the start line is set (after the turn, with no layout)
};

}  // namespace

void registerFireNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<ModeIs>("ModeIs");
  factory.registerNodeType<NotDropped>("NotDropped");
  factory.registerNodeType<WallRangeAlive>("WallRangeAlive");
  factory.registerNodeType<AttitudeAlive>("AttitudeAlive");
  factory.registerNodeType<StrafeKeep>("StrafeKeep");
  factory.registerNodeType<AwaitStrafeSolution>("AwaitStrafeSolution");
  factory.registerNodeType<ApproachBay>("ApproachBay");
  factory.registerNodeType<RequestMode>("RequestMode");
  factory.registerNodeType<StrafeSearch>("StrafeSearch");
}

}  // namespace crusader_bt
