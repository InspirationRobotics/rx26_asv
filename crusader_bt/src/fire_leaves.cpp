// fire_leaves.cpp — the fixed-nozzle shot: stand off, point, wait, fire.
//
// Same rules as leaves.cpp and task3_leaves.cpp:
//
//   * Every calculation is in fire_math.hpp, tested off-ROS
//     (test/test_fire_math.cpp). A formula growing here belongs there.
//   * The leaves never subscribe. The runner folds /crsd/wall_range,
//     /crsd/attitude and /crsd/pump_state into the Context.
//   * Outputs (heading+speed, the pump, avoidance) are called OUTSIDE
//     ctx_->mu: the runner's publish callbacks may take it.
//
// THE SHAPE (behavior_trees/task3_fire_test.xml), and why:
//
//   ReactiveSequence   guard band, re-ticked every tick
//     IsAutonomous, ModeIs GUIDED, NotDropped, WallRangeAlive, AttitudeAlive
//     StationKeep                 ALWAYS SUCCESS: commands heading+speed
//     Sequence
//       SetAvoidance false        the autopilot's avoidance stops the boat 2 m out
//       ...AwaitFiringSolution -> FireBurst -> WindowOut, retried...
//       StopBoat, SetAvoidance true
//
// "Not in the band yet" lives INSIDE the stateful AwaitFiringSolution, never
// as a sibling condition: a ReactiveSequence fails the whole mission on the
// first FAILURE, and the boat is out of the band most of the time.
//
// TWO POSTURE SWITCHES, both off by default: publish_setpoints (may move the
// boat: Gate G1) and fire_pump (may squirt: Gate G7). With both off this tree
// is a shadow - it computes and logs every solution and does nothing.
//
// TWO TREES. task3_fire_test.xml holds the spot in GUIDED with heading+speed
// (StationKeep, AwaitFiringSolution): it cannot strafe, so it aims by turning.
// task3_fire_manual.xml holds it in MANUAL on the sticks (StrafeKeep,
// AwaitStrafeSolution): square to the face, SLIDE onto the window. The
// shot itself (FireBurst) and the guards are shared.
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

// PumpState.RESULT_* (crusader_msgs/msg/PumpState.msg)
constexpr int kPumpSent = 1, kPumpAccepted = 2, kPumpRejected = 3, kPumpRefused = 4;

/// The newest sighting of window `index` in any bay track, as a camera-frame
/// bearing (+ left), or NaN. CALL UNDER mu.
double windowBearing(const Context & c, int index, double max_age_s)
{
  const dock::BayTrack * best = nullptr;
  for (const auto & t : c.dock.tracks) {
    if (best == nullptr || t.windows_t > best->windows_t) {best = &t;}
  }
  if (best == nullptr || c.dock_t - best->windows_t > max_age_s) {return fire::kNaN;}
  for (const auto & w : best->windows) {
    if (w.index == index && w.has_position && w.x > 0.1) {
      return std::atan2(w.y, w.x) / fire::kDeg;
    }
  }
  return fire::kNaN;
}

/// The camera's distance from the boat to the dock faces: the dock book's
/// best-seen fresh bay, perpendicular to its face. NaN with no fresh bay or no
/// fresh pose (then there is nothing to cross-check against). CALL UNDER mu.
double cameraFaceRange(const Context & c, double max_age_s)
{
  if (!c.pose_fresh) {return fire::kNaN;}
  const dock::BayTrack * best = nullptr;
  for (const auto & t : c.dock.tracks) {
    if (c.dock_t - t.last_seen > max_age_s || t.n < 3) {continue;}
    if (best == nullptr || t.n > best->n) {best = &t;}
  }
  if (best == nullptr) {return fire::kNaN;}
  const nav::Vec2 o = best->outward();
  const double d = fire::planeDistance(c.boat.x, c.boat.y, best->p.x, best->p.y, o.x, o.y);
  return d > 0.0 ? d : fire::kNaN;
}

// ---------------------------------------------------------------- guards

/// The autopilot is in exactly this mode. IsAutonomous accepts AUTO, LOITER
/// and RTL too, where ArduRover silently ignores heading+speed targets.
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

/// The autonomy-drop latch is clear (ch9; SD is the proposed switch, none is
/// wired yet, so today it never trips). The bridge refuses our
/// commands while it is tripped; knowing it here ends the mission instead of
/// commanding into a wall of refusals.
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

/// The LiDAR has seen a wall recently. Blind, the boat must not keep driving
/// at a dock: the keep's only range is this one.
class WallRangeAlive : public CrusaderCondition
{
public:
  WallRangeAlive(const std::string & n, const BT::NodeConfig & c) : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("max_age_s", 1.0, "seconds without a VALID wall range")};
  }
  BT::NodeStatus tick() override
  {
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

/// The shot put the window out (the timing layer's "hit": GREEN held).
class WindowOut : public CrusaderCondition
{
public:
  WindowOut(const std::string & n, const BT::NodeConfig & c) : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->dock_hits > 0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

/// At least `min` bursts went out (accepted, or run dry).
class ShotsFired : public CrusaderCondition
{
public:
  ShotsFired(const std::string & n, const BT::NodeConfig & c) : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("min", 1, "bursts needed for success")};
  }
  BT::NodeStatus tick() override
  {
    const int want = getInput<int>("min").value_or(1);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->bursts.fired >= want ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// ---------------------------------------------------------------- actions

/// The autopilot's simple avoidance on or off. It stops the boat at
/// AVOID_MARGIN (2 m) from anything, and the firing range is inside that.
class SetAvoidance : public CrusaderSyncAction
{
public:
  SetAvoidance(const std::string & n, const BT::NodeConfig & c) : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("enable", true, "true = avoidance on")};
  }
  BT::NodeStatus tick() override
  {
    const bool on = getInput<bool>("enable").value_or(true);
    if (ctx_->set_avoidance) {ctx_->set_avoidance(on);}
    RCLCPP_INFO(log(), "avoidance %s", on ? "ON" : "OFF (the tree's range floor keeps us off the dock)");
    return BT::NodeStatus::SUCCESS;
  }
};

/// Hold the firing range and point at the window. ALWAYS SUCCESS: it sits in
/// the guard band and runs every tick, like UpdateDockBook; "can't aim yet" is
/// a zero-speed command and a reason in the log, never a FAILURE.
class StationKeep : public CrusaderSyncAction
{
public:
  StationKeep(const std::string & n, const BT::NodeConfig & c) : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("fire_range_m", 3.22, "calibrated wall range for this window (squirt_cal)"),
      BT::InputPort<double>("window_lat_m", 0.22, "window from the face centre, + LEFT (UL 0.22)"),
      BT::InputPort<double>("yaw_bias_deg", 0.0, "calibrated: + aims further left"),
      BT::InputPort<std::string>("lateral", "centreline", "fingers | camera | centreline"),
      BT::InputPort<int>("window_index", 0, "DockWindow.index for the camera bearing (0 = UL)"),
      BT::InputPort<double>("deadband_m", 0.05, "inside this of the target: no thrust"),
      BT::InputPort<double>("kp", 0.6, "m/s per m of range error, above v_min"),
      BT::InputPort<double>("v_min", 0.12, "least speed the autopilot acts on"),
      BT::InputPort<double>("v_max", 0.25, "speed cap, either way"),
      BT::InputPort<double>("accel", 0.25, "m/s^2: gentle, thrust rocks the hull"),
      BT::InputPort<double>("min_range_m", 1.5, "never drive forward closer than this"),
      BT::InputPort<double>("turn_first_deg", 10.0, "heading error above this: turn only"),
      BT::InputPort<double>("square_until_m", 0.5,
        "further than this from the firing range: approach square to the wall"),
      BT::InputPort<double>("range_check_m", 0.5,
        "LiDAR vs camera range disagreement that stops the keep; 0 = off"),
      BT::InputPort<double>("face_setback_m", 0.0,
        "the camera's faces stand this far behind the edge the LiDAR ranges"),
    };
  }

  BT::NodeStatus tick() override
  {
    fire::AimParams ap;
    ap.fire_range_m = getInput<double>("fire_range_m").value_or(3.22);
    ap.window_lat_m = getInput<double>("window_lat_m").value_or(0.22);
    ap.yaw_bias_deg = getInput<double>("yaw_bias_deg").value_or(0.0);
    const fire::Lateral mode =
      fire::lateralFromName(getInput<std::string>("lateral").value_or("centreline"));
    const int widx = getInput<int>("window_index").value_or(0);
    fire::KeepParams kp;
    kp.deadband_m = getInput<double>("deadband_m").value_or(kp.deadband_m);
    kp.kp = getInput<double>("kp").value_or(kp.kp);
    kp.v_min = getInput<double>("v_min").value_or(kp.v_min);
    kp.v_max_fwd = kp.v_max_rev = getInput<double>("v_max").value_or(kp.v_max_fwd);
    kp.accel = getInput<double>("accel").value_or(kp.accel);
    kp.min_range_m = getInput<double>("min_range_m").value_or(kp.min_range_m);
    kp.turn_first_deg = getInput<double>("turn_first_deg").value_or(kp.turn_first_deg);
    const double square_m = getInput<double>("square_until_m").value_or(0.5);
    const double range_check = getInput<double>("range_check_m").value_or(0.5);
    const double setback = getInput<double>("face_setback_m").value_or(0.0);

    fire::KeepCmd cmd;
    bool publish = false;
    double range = fire::kNaN;
    double heading_now = fire::kNaN;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const double now = ctx_->now_s;
      range = ctx_->wall.range(now);
      heading_now = ctx_->heading_deg;
      const double inward = ctx_->wall.inward_deg(now);
      const double lat = ctx_->wall.lat(now);
      const double bearing = mode == fire::Lateral::Camera ? windowBearing(*ctx_, widx, 1.0) :
        fire::kNaN;
      ap.cam_x_m = ctx_->cam_mount.x;
      ctx_->aim = fire::solveAim(ap, mode, range, inward, ctx_->heading_deg, lat, bearing);
      // A LiDAR fit on the wrong thing (the finger tips, 2 m short) must not
      // be driven to: no aim, no thrust, no shot, and the log says why.
      const double cam = cameraFaceRange(*ctx_, 2.0) - setback;
      const bool range_ok = fire::rangesAgree(range, cam, range_check);
      if (!range_ok) {
        char buf[112];
        std::snprintf(buf, sizeof(buf),
          "LiDAR says %.2f m to the dock, the camera %.2f m (wall fit on the fingers?)", range, cam);
        ctx_->aim.ok = false;
        ctx_->aim.why = buf;
      }
      const double dt = ctx_->last_keep_t < 0 ? 0.1 : std::max(0.0, now - ctx_->last_keep_t);
      ctx_->last_keep_t = now;
      const double want_heading = range_ok ?
        fire::keepHeading(ctx_->aim, inward, range, ap.fire_range_m, square_m) : fire::kNaN;
      if (!std::isfinite(want_heading)) {
        cmd.heading_deg = ctx_->heading_deg;       // hold what we have
        cmd.speed_mps = 0.0;
        cmd.why = ctx_->aim.why;
      } else {
        const double target = ctx_->aim.ok ? ctx_->aim.range_target_m : ap.fire_range_m;
        cmd = fire::stationKeep(kp, range, target, ctx_->heading_deg, want_heading,
            ctx_->cmd_speed, dt);
        if (!ctx_->aim.ok) {cmd.why += " (square; " + ctx_->aim.why + ")";}
      }
      if (ctx_->wall.blanked(now)) {             // water in the air: hold still
        cmd.speed_mps = 0.0;
        cmd.why = "shot in the air";
      }
      ctx_->cmd_speed = cmd.speed_mps;
      ctx_->steady.feed_cmd(now, cmd.speed_mps);
      publish = ctx_->publish_setpoints && std::isfinite(cmd.heading_deg);
      if (publish) {ctx_->hs_commanded = true;}
      if (ctx_->task3_phase.rfind("LINE UP", 0) != 0 && ctx_->task3_phase != "FIRE") {
        ctx_->task3_phase = "KEEP: " + cmd.why;
      }
    }
    if (publish && ctx_->heading_speed) {ctx_->heading_speed(cmd.heading_deg, cmd.speed_mps);}
    if (ctx_->node) {
      RCLCPP_INFO_THROTTLE(log(), *ctx_->node->get_clock(), 1000,
        "keep: range %.2f m, heading %.1f -> %.1f, speed %+.2f m/s (%s)%s",
        range, heading_now, cmd.heading_deg, cmd.speed_mps, cmd.why.c_str(),
        publish ? "" : " [shadow: publish_setpoints is off]");
    }
    return BT::NodeStatus::SUCCESS;
  }
};

/// Wait until everything that decides the hit has held for hold_s: in the
/// range band, pointed at the window, the hull still, no thrust recently, and
/// the last shot's water down. RUNNING until then; FAILURE on timeout, saying
/// which condition never held.
class AwaitFiringSolution : public CrusaderAction
{
public:
  AwaitFiringSolution(const std::string & n, const BT::NodeConfig & c) : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("range_tol_m", 0.06, "+- about the range target"),
      BT::InputPort<double>("heading_tol_deg", 2.0, "+- about the aim heading"),
      BT::InputPort<double>("hold_s", 1.0, "everything true for this long"),
      BT::InputPort<double>("gap_s", 2.0, "from the last burst's end"),
      BT::InputPort<double>("timeout_s", 60.0, "give up after this"),
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
    const double htol = getInput<double>("heading_tol_deg").value_or(2.0);
    const double gap = getInput<double>("gap_s").value_or(2.0);
    const double timeout = getInput<double>("timeout_s").value_or(60.0);
    std::string why;
    bool held = false;
    double now = 0.0;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      now = ctx_->now_s;
      const fire::Aim & a = ctx_->aim;
      const double range = ctx_->wall.range(now);
      const fire::SteadyStatus st = ctx_->steady.status(now);
      char buf[96];
      if (!a.ok) {
        why = "no aim: " + a.why;
      } else if (ctx_->wall.blanked(now)) {
        why = "water still in the air";
      } else if (!ctx_->bursts.ready(now, gap)) {
        why = "gap after the last burst";
      } else if (!fire::inBand(range, a.range_target_m, rtol)) {
        std::snprintf(buf, sizeof(buf), "range %.2f, want %.2f +-%.2f", range, a.range_target_m, rtol);
        why = buf;
      } else if (!fire::aimed(ctx_->heading_deg, a.heading_deg, htol)) {
        std::snprintf(buf, sizeof(buf), "heading %.1f, want %.1f", ctx_->heading_deg, a.heading_deg);
        why = buf;
      } else if (!st.steady) {
        why = "not steady: " + st.why;
      }
      held = gate_.update(now, why.empty());
      ctx_->task3_phase = why.empty() ? "LINE UP: holding" : "LINE UP: " + why;
    }
    if (held) {
      RCLCPP_INFO(log(), "firing solution held %.1f s", gate_.held(now));
      return BT::NodeStatus::SUCCESS;
    }
    if (now - t0_ > timeout) {
      RCLCPP_WARN(log(), "no firing solution in %.0f s: %s", timeout, why.c_str());
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

/// One burst. With fire_pump off it runs DRY: logs and counts, sends nothing.
/// With it on, it asks the bridge (/crsd/pump_cmd), which may refuse; SUCCESS
/// when the bridge reports our sequence ACCEPTED (or SENT with no ACK by
/// ack_timeout_s: MAVProxy may not forward every ACK) and the water has had
/// time to land. The autopilot times the burst; on halt we still send OFF.
class FireBurst : public CrusaderAction
{
public:
  FireBurst(const std::string & n, const BT::NodeConfig & c) : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("burst_s", 0.5, "pump on for this long (the bridge caps it)"),
      BT::InputPort<double>("flight_s", 0.8, "the water's flight after the pump stops"),
      BT::InputPort<double>("ack_timeout_s", 1.5, "no answer from the bridge by then: fail"),
    };
  }

  BT::NodeStatus onStart() override
  {
    burst_ = getInput<double>("burst_s").value_or(0.5);
    flight_ = getInput<double>("flight_s").value_or(0.8);
    ack_timeout_ = getInput<double>("ack_timeout_s").value_or(1.5);
    sent_ = accepted_ = false;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      dry_ = !ctx_->fire_pump;
      t0_ = ctx_->now_s;
      if (!dry_ && (ctx_->pump_age_s > 1.0 || !ctx_->pump_enabled)) {
        RCLCPP_WARN(log(), "FireBurst: the bridge's pump path is not ready (%s)",
          ctx_->pump_age_s > 1.0 ? "no /crsd/pump_state" : ctx_->pump_reason.c_str());
        return BT::NodeStatus::FAILURE;
      }
      seq_ = ++ctx_->pump_seq;
      ctx_->wall.blank_until(t0_ + burst_ + flight_);
      ctx_->task3_phase = "FIRE";
    }
    if (dry_) {
      RCLCPP_INFO(log(), "FIRE #%u (DRY: fire_pump is off) %.2f s", seq_, burst_);
    } else if (ctx_->pump) {
      ctx_->pump(burst_, seq_);
      sent_ = true;
      RCLCPP_INFO(log(), "FIRE #%u: %.2f s burst asked of the bridge", seq_, burst_);
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const double now = ctx_->now_s;
    if (!dry_ && !accepted_) {
      const bool ours = ctx_->pump_last_seq == seq_;
      if (ours && (ctx_->pump_result == kPumpRefused || ctx_->pump_result == kPumpRejected)) {
        RCLCPP_WARN(log(), "FIRE #%u refused: %s", seq_, ctx_->pump_reason.c_str());
        sent_ = false;
        return BT::NodeStatus::FAILURE;
      }
      if (ours && ctx_->pump_result == kPumpAccepted) {
        accepted_ = true;
      } else if (now - t0_ > ack_timeout_) {
        if (ours && ctx_->pump_result == kPumpSent) {
          RCLCPP_WARN(log(), "FIRE #%u: sent, no ACK in %.1f s - assuming it fired", seq_, ack_timeout_);
          accepted_ = true;
        } else {
          RCLCPP_WARN(log(), "FIRE #%u: no answer from the bridge", seq_);
          return BT::NodeStatus::FAILURE;
        }
      }
    }
    if ((dry_ || accepted_) && now - t0_ >= burst_ + flight_) {
      ctx_->bursts.record(t0_, burst_, flight_);
      sent_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (sent_ && !dry_ && ctx_->pump) {ctx_->pump(0.0, seq_);}   // OFF, never refused
    sent_ = false;
  }

private:
  double burst_ = 0.5, flight_ = 0.8, ack_timeout_ = 1.5, t0_ = 0.0;
  std::uint32_t seq_ = 0;
  bool dry_ = true, sent_ = false, accepted_ = false;
};

/// Hold the firing spot on the STICKS (MANUAL, RC override): the LiDAR range
/// to the firing range (surge), the window onto the nozzle's line (sway,
/// from the camera's window x,y,z) and the bow square to the face (yaw, from
/// the camera's two windows or the face plane, closed on the compass so a
/// dropped frame does not drop the heading). ALWAYS SUCCESS, like StationKeep:
/// "not there yet" is a stick command and a reason, never a FAILURE.
class StrafeKeep : public CrusaderSyncAction
{
public:
  StrafeKeep(const std::string & n, const BT::NodeConfig & c) : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("fire_range_m", 3.22, "LiDAR wall range to fire from (squirt_cal)"),
      BT::InputPort<int>("window_index", 0, "DockWindow.index to hit (0 = upper-left)"),
      BT::InputPort<double>("lateral_bias_m", 0.0,
        "calibrated: + puts the stream this much further LEFT at the window"),
      BT::InputPort<double>("nozzle_y_m", 0.0, "nozzle LEFT of the centreline"),
      BT::InputPort<double>("cam_timeout_s", 0.5, "window older than this: no sideways thrust"),
      BT::InputPort<double>("face_timeout_s", 3.0, "square-up older than this: hold the last"),
      BT::InputPort<double>("range_check_m", 0.5,
        "LiDAR vs camera disagreement that stops the keep; 0 = off"),
      BT::InputPort<double>("face_setback_m", 0.0,
        "the face stands this far behind the edge the LiDAR ranges"),
      BT::InputPort<double>("deadband_range_m", 0.05, ""),
      BT::InputPort<double>("deadband_lat_m", 0.04, ""),
      BT::InputPort<double>("deadband_yaw_deg", 3.0, "loose: strafing aims, not the heading"),
      BT::InputPort<double>("kp_fwd", 90.0, "us per m"),
      BT::InputPort<double>("kd_fwd", 60.0, "us per m/s"),
      BT::InputPort<double>("kp_lat", 90.0, "us per m"),
      BT::InputPort<double>("kd_lat", 30.0, "us per m/s"),
      BT::InputPort<double>("kp_yaw", 4.0, "us per deg"),
      BT::InputPort<double>("kd_yaw", 3.0, "us per deg/s"),
      BT::InputPort<double>("min_us", 30.0, "added to every correction: the ESC deadband"),
      BT::InputPort<double>("max_us", 120.0, "deflection cap (the bridge caps it too)"),
      BT::InputPort<double>("slew_us_s", 200.0, "per axis"),
      BT::InputPort<double>("min_range_m", 1.5, "never push ahead inside this"),
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
    sp.fire_range_m = in("fire_range_m", sp.fire_range_m);
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
    sp.min_range_m = in("min_range_m", sp.min_range_m);
    sp.square_first_deg = in("square_first_deg", sp.square_first_deg);
    const int widx = std::clamp(getInput<int>("window_index").value_or(0), 0, 1);
    const double bias = in("lateral_bias_m", 0.0), noz_y = in("nozzle_y_m", 0.0);
    const double cam_to = in("cam_timeout_s", 0.5), face_to = in("face_timeout_s", 3.0);
    const double check = in("range_check_m", 0.5), setback = in("face_setback_m", 0.0);

    fire::StrafeCmd cmd;
    fire::StrafeInputs si;
    bool publish = false;
    double heading_now = fire::kNaN;
    std::string block, tuned;
    bool est_on = false;
    double est_p = fire::kNaN, est_v = fire::kNaN, est_age = fire::kNaN, cam_y = fire::kNaN;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const double now = ctx_->now_s;
      // live gain overrides (bt_runner_node strafe.*) on top of the tree's
      tuned = ctx_->strafe_tune.apply(sp);
      const double med_s = ctx_->strafe_tune.window_median_s >= 0.0 ?
        ctx_->strafe_tune.window_median_s : 0.6;
      const double rate_s = ctx_->strafe_tune.rate_window_s >= 0.0 ?
        ctx_->strafe_tune.rate_window_s : 0.8;
      heading_now = ctx_->heading_deg;
      si.range_m = ctx_->wall.range(now);
      si.range_rate = ctx_->wall.rate(now);
      const double age = ctx_->fire_win_y[widx].age(now);
      const bool fresh = age <= cam_to;
      // the median always spans the newest sample: a window shorter than its
      // age would find nothing and read as "not seen" while it is fresh
      const double win = std::max(med_s, age + 1e-6);
      const double wx = fresh ? ctx_->fire_win_x[widx].median(now, win) : fire::kNaN;
      const double wy = fresh ? ctx_->fire_win_y[widx].median(now, win) : fire::kNaN;
      if (fresh) {
        si.lat_err_m = wy - noz_y - bias;
        si.lat_rate = ctx_->fire_win_y[widx].slope(now, rate_s);
        cam_y = si.lat_err_m;               // the raw camera value, for the log
      }
      if (ctx_->face_heading.age(now) <= face_to) {
        ctx_->face_target = ctx_->face_heading.median(now, 2.0);
      }
      if (std::isfinite(ctx_->face_target) && std::isfinite(heading_now)) {
        si.yaw_err_deg = fire::wrap180(ctx_->face_target - heading_now);
      }
      si.yaw_rate_dps = ctx_->yaw_rate_dps;
      // THE LATERAL ESTIMATOR (strafe.est_enable). Every new camera sample of
      // this window, yaw taken out with the heading at ITS capture time, into
      // the filter; the result projected to now and the current yaw put back.
      // Replaces the median/slope above; rides through dropouts up to track_s.
      const fire::StrafeTune & tn = ctx_->strafe_tune;
      if (tn.estOn()) {
        fire::LateralEstParams kp;
        if (tn.est_q >= 0.0) {kp.q = tn.est_q;}
        if (tn.est_r >= 0.0) {kp.r = std::max(tn.est_r, 1e-3);}
        const double track = tn.track_s >= 0.0 ? tn.track_s : 1.0;
        auto & E = ctx_->lat_est;
        if (ctx_->lat_est_widx != widx) {
          E.reset();
          ctx_->lat_est_widx = widx;
          ctx_->lat_est_seen_t = -1e18;
        }
        // the window's distance ahead: slow, so a median is right for it
        const double xw = ctx_->fire_win_x[widx].median(now, 1.0);
        const double x_use = std::isfinite(xw) ? xw : 0.0;
        for (const auto & s : ctx_->fire_win_y[widx].since(ctx_->lat_est_seen_t)) {
          ctx_->lat_est_seen_t = s.first;
          const double h = ctx_->heading_hist.at(s.first);
          // no square reference or heading yet: no yaw to take out
          const double ye = (std::isfinite(ctx_->face_target) && std::isfinite(h)) ?
            fire::wrap180(ctx_->face_target - h) : 0.0;
          E.update(s.first, fire::squareOffset(x_use, s.second, ye), kp);
        }
        est_age = E.inited() ? now - E.lastMeasT() : fire::kNaN;
        if (E.inited() && est_age <= track) {
          const auto pv = E.at(now);
          est_p = pv.first;
          est_v = pv.second;
          const double ye_now = std::isfinite(si.yaw_err_deg) ? si.yaw_err_deg : 0.0;
          si.lat_err_m = fire::bodyOffset(x_use, est_p, ye_now) - noz_y - bias;
          si.lat_rate = est_v;
        } else {
          si.lat_err_m = fire::kNaN;              // lost for longer than track_s
          si.lat_rate = fire::kNaN;
        }
        est_on = true;
      }
      // the camera's distance to the face against the LiDAR's to the edge
      // (no LiDAR range - water in the air - is not a disagreement)
      if (std::isfinite(wx) && std::isfinite(si.range_m) &&
        !fire::rangesAgree(si.range_m + setback, wx, check))
      {
        char buf[112];
        std::snprintf(buf, sizeof(buf),
          "LiDAR says %.2f m to the dock, the camera %.2f m (wall fit on the fingers?)",
          si.range_m, wx - setback);
        block = buf;
      }
      const double dt = ctx_->last_strafe_t < 0 ? 0.1 :
        std::max(0.0, now - ctx_->last_strafe_t);
      ctx_->last_strafe_t = now;
      if (!block.empty()) {
        cmd.why = block;                        // hold still: every stick at zero
      } else {
        cmd = fire::strafeKeep(sp, si, ctx_->strafe_state, dt);
        if (ctx_->wall.blanked(now)) {          // water in the air: hold still
          cmd.sticks = fire::Sticks{};
          cmd.why = "shot in the air";
        }
      }
      ctx_->strafe_state.prev = cmd.sticks;
      ctx_->strafe = cmd;
      ctx_->strafe_in = si;
      ctx_->strafe_block = block;
      // "quiet" = not being moved: a steady holding push against a current
      // (the integrators alone) does not rock the hull, a correction does
      ctx_->steady.feed_cmd(now, cmd.correcting ? 1.0 : 0.0);
      publish = ctx_->publish_setpoints;
      if (publish) {ctx_->sticks_commanded = true;}
      if (ctx_->task3_phase.rfind("LINE UP", 0) != 0 && ctx_->task3_phase != "FIRE") {
        ctx_->task3_phase = "STRAFE: " + cmd.why;
      }
    }
    if (publish && ctx_->sticks) {
      ctx_->sticks(cmd.sticks.fwd_us, cmd.sticks.lat_us, cmd.sticks.yaw_us);
    }
    if (ctx_->node) {
      const std::string live = tuned.empty() ? "" : " [live: " + tuned + "]";
      // with the estimator on: what it holds (square-frame offset, rate, age of
      // the last frame) beside the raw camera value it replaced
      char est[96] = "";
      if (est_on) {
        std::snprintf(est, sizeof(est), " {est p %+.2f v %+.2f age %.2f | cam %+.2f}",
          est_p, est_v, est_age, cam_y);
      }
      RCLCPP_INFO_THROTTLE(log(), *ctx_->node->get_clock(), 1000,
        "strafe: range %.2f m, window %+.2f m left, square %+.1f deg | sticks fwd %+.0f "
        "lat %+.0f yaw %+.0f us (%s)%s%s%s",
        si.range_m, si.lat_err_m, si.yaw_err_deg, cmd.sticks.fwd_us, cmd.sticks.lat_us,
        cmd.sticks.yaw_us, cmd.why.c_str(), est,
        publish ? "" : " [shadow: publish_setpoints is off]", live.c_str());
    }
    return BT::NodeStatus::SUCCESS;
  }
};

/// Wait until the strafe keep has held the spot for hold_s: range, window on
/// the line, square, the camera fresh, the hull still, the sticks quiet, the
/// last shot's water down. RUNNING until then; FAILURE on timeout, saying
/// which never held.
class AwaitStrafeSolution : public CrusaderAction
{
public:
  AwaitStrafeSolution(const std::string & n, const BT::NodeConfig & c) : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("range_tol_m", 0.06, "+- about fire_range_m"),
      BT::InputPort<double>("lat_tol_m", 0.05, "+- the window off the nozzle's line"),
      BT::InputPort<double>("yaw_tol_deg", 5.0, "+- off square (loose: see StrafeKeep)"),
      BT::InputPort<double>("fire_range_m", 3.22, "= StrafeKeep's"),
      BT::InputPort<double>("hold_s", 1.0, "everything true for this long"),
      BT::InputPort<double>("gap_s", 2.0, "from the last burst's end"),
      BT::InputPort<double>("timeout_s", 60.0, "give up after this"),
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
    const double target = getInput<double>("fire_range_m").value_or(3.22);
    const double gap = getInput<double>("gap_s").value_or(2.0);
    const double timeout = getInput<double>("timeout_s").value_or(60.0);
    std::string why;
    bool held = false;
    double now = 0.0;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      now = ctx_->now_s;
      const fire::StrafeInputs & si = ctx_->strafe_in;
      const fire::SteadyStatus st = ctx_->steady.status(now);
      char buf[96];
      if (!ctx_->strafe_block.empty()) {
        why = ctx_->strafe_block;
      } else if (ctx_->wall.blanked(now)) {
        why = "water still in the air";
      } else if (!ctx_->bursts.ready(now, gap)) {
        why = "gap after the last burst";
      } else if (!std::isfinite(si.lat_err_m)) {
        why = "window not in view";
      } else if (!std::isfinite(si.yaw_err_deg)) {
        why = "no face angle yet";
      } else if (!fire::inBand(si.range_m, target, rtol)) {
        std::snprintf(buf, sizeof(buf), "range %.2f, want %.2f +-%.2f", si.range_m, target, rtol);
        why = buf;
      } else if (std::fabs(si.lat_err_m) > ltol) {
        std::snprintf(buf, sizeof(buf), "window %+.2f m off the line", si.lat_err_m);
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
      RCLCPP_INFO(log(), "firing solution held %.1f s", gate_.held(now));
      return BT::NodeStatus::SUCCESS;
    }
    if (now - t0_ > timeout) {
      RCLCPP_WARN(log(), "no firing solution in %.0f s: %s", timeout, why.c_str());
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

/// Speed zero, heading held. The runner also stops the boat on every exit;
/// this is the tree saying so when it means to.
class StopBoat : public CrusaderSyncAction
{
public:
  StopBoat(const std::string & n, const BT::NodeConfig & c) : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    double heading = fire::kNaN;
    bool publish = false;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      heading = ctx_->heading_deg;
      ctx_->cmd_speed = 0.0;
      publish = ctx_->publish_setpoints && std::isfinite(heading);
      if (publish) {ctx_->hs_commanded = true;}
    }
    if (publish && ctx_->heading_speed) {ctx_->heading_speed(heading, 0.0);}
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace

void registerFireNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<ModeIs>("ModeIs");
  factory.registerNodeType<NotDropped>("NotDropped");
  factory.registerNodeType<WallRangeAlive>("WallRangeAlive");
  factory.registerNodeType<AttitudeAlive>("AttitudeAlive");
  factory.registerNodeType<WindowOut>("WindowOut");
  factory.registerNodeType<ShotsFired>("ShotsFired");
  factory.registerNodeType<SetAvoidance>("SetAvoidance");
  factory.registerNodeType<StationKeep>("StationKeep");
  factory.registerNodeType<AwaitFiringSolution>("AwaitFiringSolution");
  factory.registerNodeType<FireBurst>("FireBurst");
  factory.registerNodeType<StopBoat>("StopBoat");
  factory.registerNodeType<StrafeKeep>("StrafeKeep");
  factory.registerNodeType<AwaitStrafeSolution>("AwaitStrafeSolution");
}

}  // namespace crusader_bt
