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

/// The pilot has not dropped autonomy on SE (ch9). The bridge refuses our
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
}

}  // namespace crusader_bt
