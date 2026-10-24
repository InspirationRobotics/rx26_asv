// fire_leaves.cpp — the fixed-nozzle shot: stand off, point, wait, fire.
//
// Same rules as leaves.cpp and task3_leaves.cpp:
//
//   * Every calculation is in fire_math.hpp, tested off-ROS
//     (test/test_fire_math.cpp). A formula growing here belongs there.
//   * The leaves never subscribe: the runner folds /crsd/wall_range,
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
/// at a dock: the keep's only range is this one. With or_camera_s, a dock window
/// the camera saw that recently also counts (StrafeKeep range_fallback="camera").
class WallRangeAlive : public CrusaderCondition
{
public:
  WallRangeAlive(const std::string & n, const BT::NodeConfig & c) : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("max_age_s", 1.0, "seconds without a VALID wall range"),
      BT::InputPort<double>("or_camera_s", 0.0, "or a dock window seen this recently; 0 = off"),
      BT::InputPort<bool>("use_lidar", true, "false: the LiDAR wall does not count")};
  }
  BT::NodeStatus tick() override
  {
    const double max_age = getInput<double>("max_age_s").value_or(1.0);
    const double cam = getInput<double>("or_camera_s").value_or(0.0);
    const bool lidar = getInput<bool>("use_lidar").value_or(true);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const double now = ctx_->now_s;
    const bool seen = cam > 0.0 &&
      std::min(ctx_->fire_win_x[0].age(now), ctx_->fire_win_x[1].age(now)) <= cam;
    return (lidar && ctx_->wall.valid_age(now) <= max_age) || seen ? BT::NodeStatus::SUCCESS :
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
      BT::InputPort<std::string>("range_fallback", "",
        "camera = with no LiDAR wall, the range is the camera's to the window"),
      BT::InputPort<bool>("use_lidar", true, "false: ignore the LiDAR wall (camera only)"),
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
    std::string block;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const double now = ctx_->now_s;
      heading_now = ctx_->heading_deg;
      if (getInput<bool>("use_lidar").value_or(true)) {
        si.range_m = ctx_->wall.range(now);
        si.range_rate = ctx_->wall.rate(now);
      }
      const bool fresh = ctx_->fire_win_y[widx].age(now) <= cam_to;
      const double wx = fresh ? ctx_->fire_win_x[widx].median(now, 0.6) : fire::kNaN;
      const double wy = fresh ? ctx_->fire_win_y[widx].median(now, 0.6) : fire::kNaN;
      if (fresh) {
        si.lat_err_m = wy - noz_y - bias;
        si.lat_rate = ctx_->fire_win_y[widx].slope(now, 0.8);
      }
      // out of the LiDAR's wall (too far, not square): close in on the camera's range
      const bool cam_range = !std::isfinite(si.range_m) && std::isfinite(wx) &&
        getInput<std::string>("range_fallback").value_or("") == "camera";
      if (cam_range) {
        si.range_m = wx - setback;
        si.range_rate = ctx_->fire_win_x[widx].slope(now, 0.8);
      }
      if (ctx_->face_heading.age(now) <= face_to) {
        ctx_->face_target = ctx_->face_heading.median(now, 2.0);
      }
      if (std::isfinite(ctx_->face_target) && std::isfinite(heading_now)) {
        si.yaw_err_deg = fire::wrap180(ctx_->face_target - heading_now);
      }
      si.yaw_rate_dps = ctx_->yaw_rate_dps;
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
        if (cam_range) {cmd.why += " (camera range)";}
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
      RCLCPP_INFO_THROTTLE(log(), *ctx_->node->get_clock(), 1000,
        "strafe: range %.2f m, window %+.2f m left, square %+.1f deg | sticks fwd %+.0f "
        "lat %+.0f yaw %+.0f us (%s)%s",
        si.range_m, si.lat_err_m, si.yaw_err_deg, cmd.sticks.fwd_us, cmd.sticks.lat_us,
        cmd.sticks.yaw_us, cmd.why.c_str(),
        publish ? "" : " [shadow: publish_setpoints is off]");
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
      BT::InputPort<double>("timeout_s", 120.0, "FAILURE after this"),
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
        if (dir_ * d >= leg_m) {
          if (leg_ == 0) {
            dir_ = -dir_;
            leg_ = 1;
          } else {
            fail = true;
            why = toward_ ? "the committed bay never came into view" : "no GREEN bay along the dock";
          }
        }
        if (now - t0_ > getInput<double>("timeout_s").value_or(120.0)) {
          fail = true;
          why = "timed out";
        }
        yaw_err = fire::wrap180(h0_ - ctx_->heading_deg);   // + = turn right
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
  factory.registerNodeType<RequestMode>("RequestMode");
  factory.registerNodeType<StrafeSearch>("StrafeSearch");
}

}  // namespace crusader_bt
