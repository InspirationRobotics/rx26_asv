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
    return {BT::InputPort<double>("max_age_s", 1.0, "seconds without a VALID wall range"),
      BT::InputPort<double>("face_max_age_s", 0.0,
        "> 0: ALSO alive while the camera's bay face is this fresh (a StrafeKeep with "
        "range_fallback=\"face\" ranges on it until the LiDAR has the wall)")};
  }
  BT::NodeStatus tick() override
  {
    const double max_age = getInput<double>("max_age_s").value_or(1.0);
    const double face_age = getInput<double>("face_max_age_s").value_or(0.0);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const double now = ctx_->now_s;
    const bool ok = ctx_->wall.valid_age(now) <= max_age ||
      (face_age > 0.0 && ctx_->fire_face_x.age(now) <= face_age);
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

/// The LiDAR has found the slip recently (/crsd/dock_slot: side walls and the
/// back wall). SlotKeep's every axis comes from it.
class DockSlotAlive : public CrusaderCondition
{
public:
  DockSlotAlive(const std::string & n, const BT::NodeConfig & c) : CrusaderCondition(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("max_age_s", 2.0, "seconds without a VALID slip")};
  }
  BT::NodeStatus tick() override
  {
    const double max_age = getInput<double>("max_age_s").value_or(2.0);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    return ctx_->slot.valid_age(ctx_->now_s) <= max_age ? BT::NodeStatus::SUCCESS :
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
      BT::InputPort<double>("fire_range_m", 1.4, "calibrated wall range for this window (30 deg nozzle)"),
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
    ap.fire_range_m = getInput<double>("fire_range_m").value_or(1.4);
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
      BT::InputPort<int>("window_index", -1,
        "the window this burst is aimed at, kept for ReportFirefighting (-1: not kept)"),
    };
  }

  BT::NodeStatus onStart() override
  {
    burst_ = getInput<double>("burst_s").value_or(0.5);
    flight_ = getInput<double>("flight_s").value_or(0.8);
    ack_timeout_ = getInput<double>("ack_timeout_s").value_or(1.5);
    widx_ = getInput<int>("window_index").value_or(-1);
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
      if (widx_ >= 0) {ctx_->fired_window = widx_;}   // ReportFirefighting's window
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
  int widx_ = -1;
  std::uint32_t seq_ = 0;
  bool dry_ = true, sent_ = false, accepted_ = false;
};

/// Look left, then right, on the STICKS (MANUAL; the yaw stick only), so the
/// camera sweeps the whole dock: Task 3 part 2 numbering the bays from its
/// line-up point. Turns to the starting heading - swing_deg, holds hold_s,
/// turns to + swing_deg, holds, turns back. SUCCESS when back; FAILURE with
/// no heading or a turn that takes longer than turn_timeout_s.
///
/// The trig (OAK-D LR, 82 deg across): the three faces span 5 m (centres 2 m
/// apart, 1 m wide). Square on to the MIDDLE bay they all fit from 2.9 m
/// (camera to face); square on to an OUTER bay never in practice (5.2 m), but
/// with the bow turned ~25 deg toward the others they fit from 2.6 m. So a
/// 25 deg swing each way, from 3 m or more out, shows every face whole once.
///
/// Shadow (publish_setpoints off): no sticks, so it cannot turn; it waits the
/// holds out and succeeds, logging what it would have done.
class LookAround : public CrusaderAction
{
public:
  LookAround(const std::string & n, const BT::NodeConfig & c) : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("swing_deg", 25.0, "each way from the starting heading"),
      BT::InputPort<double>("hold_s", 3.0, "look this long at each side"),
      BT::InputPort<double>("tol_deg", 4.0, "close enough to the look heading"),
      BT::InputPort<double>("kp_yaw", 4.0, "us per deg"),
      BT::InputPort<double>("kd_yaw", 1.0, "us per deg/s"),
      BT::InputPort<double>("min_us", 30.0, "smallest yaw stick that turns the hull"),
      BT::InputPort<double>("max_us", 100.0, "yaw stick cap"),
      BT::InputPort<double>("turn_timeout_s", 20.0, "one turn longer than this: FAILURE"),
    };
  }

  BT::NodeStatus onStart() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->pose_fresh || !std::isfinite(ctx_->heading_deg)) {
      RCLCPP_WARN(log(), "LookAround: no heading to turn by");
      return BT::NodeStatus::FAILURE;
    }
    h0_ = ctx_->heading_deg;
    step_ = 0;
    t_step_ = ctx_->now_s;
    held_since_ = -1.0;
    shadow_ = !ctx_->publish_setpoints;
    ctx_->task3_phase = "LOOK";
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const double swing = getInput<double>("swing_deg").value_or(25.0);
    const double hold = getInput<double>("hold_s").value_or(3.0);
    const double tol = getInput<double>("tol_deg").value_or(4.0);
    const double turn_to = getInput<double>("turn_timeout_s").value_or(20.0);
    const double offsets[3] = {-swing, swing, 0.0};
    double yaw_us = 0.0, err = fire::kNaN, now = 0.0;
    bool publish = false, done = false, fail = false;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      now = ctx_->now_s;
      const double target = fire::wrap360(h0_ + offsets[step_]);
      // the last step (back to square) needs no hold: StrafeKeep takes over
      const double need = step_ < 2 ? hold : 0.0;
      if (!std::isfinite(ctx_->heading_deg)) {
        fail = true;
      } else if (shadow_) {
        // cannot turn without the sticks: pretend each step arrives at once
        if (now - t_step_ >= need) {next(now);}
        done = step_ > 2;
      } else {
        err = fire::wrap180(target - ctx_->heading_deg);          // + = turn right
        const double rate = ctx_->yaw_rate_dps;
        const bool there = std::fabs(err) <= tol &&
          (!std::isfinite(rate) || std::fabs(rate) < 5.0);
        if (there) {
          if (held_since_ < 0.0) {held_since_ = now;}
          if (now - held_since_ >= need) {next(now);}
        } else {
          held_since_ = -1.0;
          if (now - t_step_ > turn_to) {fail = true;}
        }
        done = step_ > 2;
        if (!done && !fail) {
          yaw_us = fire::axisLaw(err, std::isfinite(rate) ? -rate : fire::kNaN,
            getInput<double>("kp_yaw").value_or(4.0), getInput<double>("kd_yaw").value_or(1.0),
            there ? 180.0 : 0.0,                  // at the look heading: hands off
            getInput<double>("min_us").value_or(30.0), getInput<double>("max_us").value_or(100.0));
        }
        publish = ctx_->publish_setpoints;
        if (publish) {ctx_->sticks_commanded = true;}
      }
    }
    if (publish && ctx_->sticks) {ctx_->sticks(0.0, 0.0, done || fail ? 0.0 : yaw_us);}
    if (fail) {
      RCLCPP_WARN(log(), "LookAround: %s", std::isfinite(err) ?
        "a turn took too long" : "lost the heading");
      return BT::NodeStatus::FAILURE;
    }
    if (done) {
      RCLCPP_INFO(log(), "looked %.0f deg each way%s", swing, shadow_ ? " [shadow]" : "");
      return BT::NodeStatus::SUCCESS;
    }
    if (ctx_->node) {
      RCLCPP_INFO_THROTTLE(log(), *ctx_->node->get_clock(), 1000,
        "look %d/3: %+.1f deg to go | yaw stick %+.0f us%s", step_ + 1, err, yaw_us,
        shadow_ ? " [shadow: publish_setpoints is off]" : "");
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
  void next(double now)
  {
    ++step_;
    t_step_ = now;
    held_since_ = -1.0;
  }

  double h0_ = 0.0, t_step_ = 0.0, held_since_ = -1.0;
  int step_ = 0;
  bool shadow_ = false;
};

/// Slide SIDEWAYS along the dock on the lateral stick (MANUAL) until something
/// else ends it: in the trees, a ReactiveFallback whose first child is
/// SafeBayKnown (the camera has the GREEN indicator) or CommittedBayInView. The
/// search for the GREEN bay from a waypoint handed over in front of the dock
/// (task3_cannon_slip.xml): leg_m toward `first`, then back the other way to
/// leg_m past the start; then FAILURE (nothing along 2 x leg_m of dock).
///
/// With the dock laid out in the book (dock::layout: its faces' line and which
/// way they face), it slides ALONG THE DOCK, the bow held square to the faces,
/// and holds its distance out from them: the start's, or min_out_m if that is
/// closer - the fingers stand 2 m out from the faces. Without a layout, across
/// the bow it had at the start, holding that start line. In the sim, sliding
/// across a bow left 25 deg off square by the GUIDED leg, from 2.3 m out (the
/// leg coasted 0.7 m past a 3 m waypoint), put the hull against a finger and it
/// slid nowhere for 120 s (2026-10-17). It turns first: no slide while the bow
/// is more than 15 deg off. The distance is the GPS pose's. slide_mps > 0 holds
/// a sliding SPEED instead of a fixed stick: against a 6 m/s crosswind the fixed
/// 70 us barely moved the hull (the sim, 2026-10-17).
///
/// first="committed": toward the committed bay's face in the book, under a
/// ReactiveFallback whose first child is CommittedBayInView - the bay was
/// committed from far off and is out of the picture in close.
///
/// Shadow (publish_setpoints off): no sticks; it waits out timeout_s.
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
      BT::InputPort<double>("slide_mps", 0.0,
        "> 0: hold this sliding speed (lat_us + kv_lat x the shortfall, up to lat_max_us), "
        "so a crosswind or a current does not stop the slide; 0 = lat_us alone"),
      BT::InputPort<double>("kv_lat", 250.0, "us per m/s of sliding-speed shortfall"),
      BT::InputPort<double>("lat_max_us", 160.0, "lateral stick cap with slide_mps"),
      BT::InputPort<double>("kp_yaw", 5.0, "us per deg (heading hold)"),
      BT::InputPort<double>("kd_yaw", 0.5, "us per deg/s"),
      BT::InputPort<double>("min_us", 30.0, "smallest yaw / surge stick that moves the hull"),
      BT::InputPort<double>("max_us", 100.0, "yaw stick cap"),
      BT::InputPort<double>("min_out_m", 3.5,
        "hold at least this far out from the faces (body origin), m: fingers 2 m + the hull + 1 m"),
      BT::InputPort<double>("kp_out", 80.0, "us per m (the distance hold)"),
      BT::InputPort<double>("kd_out", 200.0, "us per m/s"),
      BT::InputPort<double>("out_max_us", 80.0, "surge stick cap"),
      BT::InputPort<double>("timeout_s", 120.0, "FAILURE after this"),
    };
  }

  BT::NodeStatus onStart() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->pose_fresh || !ctx_->origin_set || !std::isfinite(ctx_->heading_deg)) {
      RCLCPP_WARN(log(), "StrafeSearch: no pose or heading to slide by");
      return BT::NodeStatus::FAILURE;
    }
    p0_ = ctx_->boat;
    const dock::DockLayout L = dock::layout(ctx_->dock, ctx_->dock_min_obs, ctx_->dock_bays);
    along_dock_ = L.ok;
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
    const std::string first = getInput<std::string>("first").value_or("left");
    toward_ = first == "committed";
    if (toward_) {
      // toward the committed bay's face, from its place in the book
      const dock::BayTrack * t = ctx_->dock.find(ctx_->chosen_track);
      if (t == nullptr) {
        RCLCPP_WARN(log(), "StrafeSearch: first=\"committed\", but no bay is committed");
        return BT::NodeStatus::FAILURE;
      }
      const double d = nav::dot(t->p - p0_, right_);
      dir_ = d >= 0.0 ? 1.0 : -1.0;
      RCLCPP_INFO(log(), "StrafeSearch: the committed bay is %.1f m to the %s: sliding that way",
        std::fabs(d), d >= 0.0 ? "right" : "left");
    } else {
      dir_ = first == "right" ? 1.0 : -1.0;
    }
    if (along_dock_) {
      RCLCPP_INFO(log(), "StrafeSearch: along the dock, bow on the faces (%.0f deg), %.1f m out",
        h0_, out_target_);
    } else {
      RCLCPP_INFO(log(), "StrafeSearch: no dock layout yet: across the bow (%.0f deg), "
        "holding the start line", h0_);
    }
    leg_ = 0;
    t0_ = ctx_->now_s;
    t_prev_ = -1.0;
    out_rate_ = 0.0;
    d_prev_ = 0.0;
    d_rate_ = 0.0;
    shadow_ = !ctx_->publish_setpoints;
    ctx_->task3_phase = "SEARCH";
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const double leg_m = getInput<double>("leg_m").value_or(4.0);
    const double lat_us = getInput<double>("lat_us").value_or(70.0);
    const double slide_mps = getInput<double>("slide_mps").value_or(0.0);
    const double min_us = getInput<double>("min_us").value_or(30.0);
    double yaw_us = 0.0, fwd_us = 0.0, lat_cmd = 0.0, d = 0.0, out_err = 0.0, yaw_err = 0.0;
    double now = 0.0;
    bool publish = false, fail = false;
    std::string why;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      now = ctx_->now_s;
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
        // the bow: square to the faces (or the start's heading)
        yaw_err = fire::wrap180(h0_ - ctx_->heading_deg);         // + = turn right
        const double rate = ctx_->yaw_rate_dps;
        yaw_us = fire::axisLaw(yaw_err, std::isfinite(rate) ? -rate : fire::kNaN,
          getInput<double>("kp_yaw").value_or(5.0), getInput<double>("kd_yaw").value_or(0.5),
          3.0, min_us, getInput<double>("max_us").value_or(100.0));
        // the distance out: + = too far out, so ahead (the bow is on the faces)
        out_err = nav::dot(ctx_->boat - face_, out_) - out_target_;
        if (t_prev_ >= 0.0 && now > t_prev_) {
          const double dt = now - t_prev_;
          out_rate_ = 0.7 * out_rate_ + 0.3 * (out_err - out_prev_) / dt;
          d_rate_ = 0.7 * d_rate_ + 0.3 * (d - d_prev_) / dt;
        }
        t_prev_ = now;
        out_prev_ = out_err;
        d_prev_ = d;
        if (std::fabs(yaw_err) <= 30.0) {
          fwd_us = fire::axisLaw(out_err, out_rate_, getInput<double>("kp_out").value_or(80.0),
            getInput<double>("kd_out").value_or(200.0), 0.25, min_us,
            getInput<double>("out_max_us").value_or(80.0));
        }
        // turn first, then slide; with slide_mps at that speed, whatever pushes the hull
        if (std::fabs(yaw_err) > 15.0) {
          lat_cmd = 0.0;
        } else if (slide_mps > 0.0) {
          const double lat_max = getInput<double>("lat_max_us").value_or(160.0);
          lat_cmd = std::clamp(dir_ * lat_us + getInput<double>("kv_lat").value_or(250.0) *
            (dir_ * slide_mps - d_rate_), -lat_max, lat_max);
        } else {
          lat_cmd = dir_ * lat_us;
        }
        publish = ctx_->publish_setpoints;
        if (publish) {ctx_->sticks_commanded = true;}
      }
    }
    if (publish && ctx_->sticks) {
      if (fail) {
        ctx_->sticks(0.0, 0.0, 0.0);
      } else {
        ctx_->sticks(fwd_us, lat_cmd, yaw_us);           // lateral + = starboard
      }
    }
    if (fail) {
      RCLCPP_WARN(log(), "StrafeSearch: %s (%.1f m %s of the start)", why.c_str(),
        std::fabs(d), d >= 0.0 ? "right" : "left");
      return BT::NodeStatus::FAILURE;
    }
    if (ctx_->node) {
      RCLCPP_INFO_THROTTLE(log(), *ctx_->node->get_clock(), 1000,
        "%s: %.1f m %s of the start, sliding %s at %.2f m/s%s | out %+.2f m, bow %+.0f deg off | "
        "sticks fwd %+.0f lat %+.0f yaw %+.0f us%s",
        toward_ ? "bringing the committed bay into view" : "searching for the GREEN bay",
        std::fabs(d), d >= 0.0 ? "right" : "left", dir_ > 0.0 ? "right" : "left",
        std::fabs(d_rate_), lat_cmd == 0.0 ? " (squaring up first)" : "", out_err, yaw_err, fwd_us, lat_cmd, yaw_us,
        shadow_ ? " [shadow: publish_setpoints is off]" : "");
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
  double d_prev_ = 0.0, d_rate_ = 0.0;   ///< the slide, m right of the start, and its speed
  int leg_ = 0;
  bool shadow_ = false;
  bool along_dock_ = false;  ///< sliding along the dock's faces (a layout), not across the bow
  bool toward_ = false;      ///< first="committed": toward the committed bay
};

/// Hold the firing spot on the STICKS (MANUAL, RC override): the LiDAR range
/// to the firing range (surge), the window onto the nozzle's line (sway,
/// from the camera's window x,y,z) and the bow square to the face (yaw, from
/// the camera's two windows or the face plane, closed on the compass so a
/// dropped frame does not drop the heading). ALWAYS SUCCESS, like StationKeep:
/// "not there yet" is a stick command and a reason, never a FAILURE.
///
/// lateral_ref="face" holds the bay's FACE CENTRE on the line instead of a
/// window (ingestFireFace: the committed bay, else the GREEN one, else the
/// nearest). The face is the steadier detection up close (the windows come and
/// go: seen on the water, 2026-10-09), and with the pan/tilt cannon aiming
/// itself the hold only has to keep the boat in front of the bay.
class StrafeKeep : public CrusaderSyncAction
{
public:
  StrafeKeep(const std::string & n, const BT::NodeConfig & c) : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("fire_range_m", 1.4, "LiDAR wall range to fire from (30 deg nozzle)"),
      BT::InputPort<int>("window_index", 0, "DockWindow.index to hit (0 = upper-left)"),
      BT::InputPort<std::string>("lateral_ref", "window",
        "window | face: hold window_index's window on the line, or the bay's face centre"),
      BT::InputPort<double>("approach_m", 0.0,
        "> 0: farther than the held range + this, point the BOW at the bay (heading, keeps "
        "the whole face in view) and do not strafe; nearer, square up and strafe"),
      BT::InputPort<bool>("watch", false,
        "true: hold the WATCH range (live strafe.watch_range_m, else watch_range_m) "
        "instead of fire_range_m - backed off after docking, to see the lights"),
      BT::InputPort<double>("watch_range_m", 1.7, "the watch range, when watch is true"),
      BT::InputPort<std::string>("range_fallback", "none",
        "none | face: with no valid LiDAR wall, range on the camera's face distance "
        "(brings the boat in until the LiDAR has the wall)"),
      // The hold puts the window `bias` LEFT of the nozzle's line (lat_err =
      // wy - nozzle_y - bias -> 0), so the line, and the shot, moves RIGHT.
      BT::InputPort<double>("lateral_bias_m", 0.0,
        "+ moves the shot this much further RIGHT at the window (the window is held this far LEFT of the line)"),
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
      BT::InputPort<double>("max_close_mps", -1.0,
        "approach speed limit, m/s (<= 0 = off, the default); strafe.max_close_mps live"),
      BT::InputPort<double>("close_gain", 0.5, "the limit shrinks to this x the remaining m, per s"),
      BT::InputPort<double>("kv_close", 300.0, "us per m/s over the limit"),
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
    sp.max_close_mps = in("max_close_mps", -1.0);
    sp.close_gain = in("close_gain", 0.5);
    sp.kv_close = in("kv_close", 300.0);
    sp.square_first_deg = in("square_first_deg", sp.square_first_deg);
    const int widx = std::clamp(getInput<int>("window_index").value_or(0), 0, 1);
    const std::string ref = getInput<std::string>("lateral_ref").value_or("window");
    if (ref != "window" && ref != "face") {
      throw BT::RuntimeError("StrafeKeep: lateral_ref is window or face, not '" + ref + "'");
    }
    const bool on_face = ref == "face";
    const std::string fb = getInput<std::string>("range_fallback").value_or("none");
    if (fb != "none" && fb != "face") {
      throw BT::RuntimeError("StrafeKeep: range_fallback is none or face, not '" + fb + "'");
    }
    const bool range_from_face = fb == "face";
    const double approach_m = in("approach_m", 0.0);
    const bool watching = getInput<bool>("watch").value_or(false);
    const double watch_port = in("watch_range_m", 1.7);
    bool approach = false;
    double brg = fire::kNaN;
    bool range_cam = false;
    const double bias = in("lateral_bias_m", 0.0), noz_y = in("nozzle_y_m", 0.0);
    const double cam_to = in("cam_timeout_s", 0.5), face_to = in("face_timeout_s", 3.0);
    const double check = in("range_check_m", 0.5), setback = in("face_setback_m", 0.0);

    fire::StrafeCmd cmd;
    fire::StrafeInputs si;
    bool publish = false;
    double heading_now = fire::kNaN;
    std::string block, tuned, ref_name = ref;
    bool est_on = false;
    double est_p = fire::kNaN, est_v = fire::kNaN, est_age = fire::kNaN, cam_y = fire::kNaN;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const double now = ctx_->now_s;
      // what goes on the line: window_index's window, or the bay's face centre
      fire::Series & ref_x = on_face ? ctx_->fire_face_x : ctx_->fire_win_x[widx];
      fire::Series & ref_y = on_face ? ctx_->fire_face_y : ctx_->fire_win_y[widx];
      ctx_->strafe_ref = ref;
      if (on_face && !ctx_->face_pick.empty()) {ref_name = "face (" + ctx_->face_pick + ")";}
      // live gain overrides (bt_runner_node strafe.*) on top of the tree's
      tuned = ctx_->strafe_tune.apply(sp);
      if (watching) {
        sp.fire_range_m = ctx_->strafe_tune.watch_range_m >= 0.0 ?
          ctx_->strafe_tune.watch_range_m : watch_port;
      }
      ctx_->strafe_target_range = sp.fire_range_m;
      ctx_->strafe_target_t = now;
      const double med_s = ctx_->strafe_tune.window_median_s >= 0.0 ?
        ctx_->strafe_tune.window_median_s : 0.6;
      const double rate_s = ctx_->strafe_tune.rate_window_s >= 0.0 ?
        ctx_->strafe_tune.rate_window_s : 0.8;
      heading_now = ctx_->heading_deg;
      si.range_m = ctx_->wall.range(now);
      si.range_rate = ctx_->wall.rate(now);
      // No wall from the LiDAR (too far out, or at an angle it cannot fit): the
      // camera's distance to the bay face instead, until the LiDAR has it. Both
      // are from the body origin; the face stands `setback` behind the LiDAR's.
      if (!std::isfinite(si.range_m) && range_from_face &&
        ctx_->fire_face_x.age(now) <= cam_to)
      {
        si.range_m = ctx_->fire_face_x.median(now, std::max(med_s, 0.3)) - setback;
        si.range_rate = ctx_->fire_face_x.slope(now, rate_s);
        range_cam = true;
      }
      const double age = ref_y.age(now);
      const bool fresh = age <= cam_to;
      // the median always spans the newest sample: a window shorter than its
      // age would find nothing and read as "not seen" while it is fresh
      const double win = std::max(med_s, age + 1e-6);
      const double wx = fresh ? ref_x.median(now, win) : fire::kNaN;
      const double wy = fresh ? ref_y.median(now, win) : fire::kNaN;
      if (fresh) {
        si.lat_err_m = wy - noz_y - bias;
        si.lat_rate = ref_y.slope(now, rate_s);
        cam_y = si.lat_err_m;               // the raw camera value, for the log
      }
      if (ctx_->face_heading.age(now) <= face_to) {
        ctx_->face_target = ctx_->face_heading.median(now, 2.0);
      }
      if (std::isfinite(ctx_->face_target) && std::isfinite(heading_now)) {
        si.yaw_err_deg = fire::wrap180(ctx_->face_target - heading_now);
      }
      si.yaw_rate_dps = ctx_->yaw_rate_dps;
      // THE APPROACH (approach_m): far out, the bow points AT the bay - the
      // heading keeps both sides of the face in view - and nothing strafes.
      // On the water (2026-10-11) squaring to the face from 4.8 m and strafing
      // across drove in 6-12 deg off with the bay sliding out of the image.
      // Hysteresis 0.5 m, so it does not flick between the two at the line.
      if (approach_m > 0.0 && std::isfinite(si.range_m)) {
        auto & st = ctx_->strafe_state;
        if (st.approach && si.range_m < sp.fire_range_m + approach_m) {st.approach = false;}
        if (!st.approach && si.range_m > sp.fire_range_m + approach_m + 0.5) {st.approach = true;}
        if (st.approach && ctx_->fire_face_y.age(now) <= cam_to) {
          const double w = std::max(med_s, 0.3);
          const double fx = ctx_->fire_face_x.median(now, w) - ctx_->cam_mount.x;
          const double fy = ctx_->fire_face_y.median(now, w) - ctx_->cam_mount.y;
          brg = std::atan2(fy, fx) / fire::kDeg;     // + = the bay is to port
          si.yaw_err_deg = -brg;                     // + = turn right
          si.lat_err_m = 0.0;
          si.lat_rate = 0.0;
          approach = true;
        }
      }
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
        const int est_key = on_face ? 2 : widx;   // 0, 1 the windows; 2 the face
        if (ctx_->lat_est_widx != est_key) {
          E.reset();
          ctx_->lat_est_widx = est_key;
          ctx_->lat_est_seen_t = -1e18;
        }
        // the window's distance ahead: slow, so a median is right for it
        const double xw = ref_x.median(now, 1.0);
        const double x_use = std::isfinite(xw) ? xw : 0.0;
        for (const auto & s : ref_y.since(ctx_->lat_est_seen_t)) {
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
      if (!range_cam && std::isfinite(wx) && std::isfinite(si.range_m) &&
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
        if (approach) {
          // in toward the bay only as fast as the bow is on it: full under 5 deg
          // off, a fifth by 17 deg (strafeKeep turns in place past square_first_deg)
          if (cmd.sticks.fwd_us > 0.0) {
            cmd.sticks.fwd_us *= std::clamp(1.0 - (std::fabs(brg) - 5.0) / 15.0, 0.2, 1.0);
          }
          char why[64];
          std::snprintf(why, sizeof(why), "approach: bow on the bay, %+.0f deg", brg);
          cmd.why = why;
        }
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
        "strafe: range %.2f m%s, %s %+.2f m left, square %+.1f deg | sticks fwd %+.0f "
        "lat %+.0f yaw %+.0f us (%s)%s%s%s",
        si.range_m, range_cam ? " (camera)" : "", ref_name.c_str(), si.lat_err_m, si.yaw_err_deg,
        cmd.sticks.fwd_us,
        cmd.sticks.lat_us,
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
      BT::InputPort<double>("fire_range_m", 1.4, "= StrafeKeep's"),
      BT::InputPort<double>("hold_s", 1.0, "everything true for this long"),
      BT::InputPort<double>("gap_s", 2.0, "from the last burst's end"),
      BT::InputPort<double>("timeout_s", 60.0, "give up after this"),
      BT::InputPort<bool>("steady", true,
        "also the hull still and the sticks quiet (a FIXED nozzle aims with the hull); "
        "false for docking with the pan/tilt cannon, which aims itself"),
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
    double target = getInput<double>("fire_range_m").value_or(1.4);
    const double gap = getInput<double>("gap_s").value_or(2.0);
    const double timeout = getInput<double>("timeout_s").value_or(60.0);
    const bool need_steady = getInput<bool>("steady").value_or(true);
    std::string why;
    bool held = false;
    double now = 0.0;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      now = ctx_->now_s;
      const fire::StrafeInputs & si = ctx_->strafe_in;
      const fire::SteadyStatus st = ctx_->steady.status(now);
      // the live distance (strafe.fire_range_m) moves this check with the keep
      // the range the keep is holding now (a live override, the watch range)
      if (std::isfinite(ctx_->strafe_target_range) && now - ctx_->strafe_target_t <= 1.0) {
        target = ctx_->strafe_target_range;
      } else if (ctx_->strafe_tune.fire_range_m >= 0.0) {
        target = ctx_->strafe_tune.fire_range_m;
      }
      char buf[96];
      if (!ctx_->strafe_block.empty()) {
        why = ctx_->strafe_block;
      } else if (ctx_->wall.blanked(now)) {
        why = "water still in the air";
      } else if (!ctx_->bursts.ready(now, gap)) {
        why = "gap after the last burst";
      } else if (!std::isfinite(si.lat_err_m)) {
        why = ctx_->strafe_ref + " not in view";
      } else if (!std::isfinite(si.yaw_err_deg)) {
        why = "no face angle yet";
      } else if (!fire::inBand(si.range_m, target, rtol)) {
        std::snprintf(buf, sizeof(buf), "range %.2f, want %.2f +-%.2f", si.range_m, target, rtol);
        why = buf;
      } else if (std::fabs(si.lat_err_m) > ltol) {
        std::snprintf(buf, sizeof(buf), "%s %+.2f m off the line", ctx_->strafe_ref.c_str(),
          si.lat_err_m);
        why = buf;
      } else if (std::fabs(si.yaw_err_deg) > ytol) {
        std::snprintf(buf, sizeof(buf), "%+.1f deg off square", si.yaw_err_deg);
        why = buf;
      } else if (need_steady && !st.steady) {
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

/// Task 3 with the pan/tilt cannon: hold the berth on the STICKS (MANUAL, RC
/// override) from the LiDAR's slip alone (/crsd/dock_slot): the LiDAR
/// `standoff_m` from the back wall (surge), the hull on the slip's centreline
/// (sway), the bow square to the slip's side walls (yaw, closed on the compass
/// as StrafeKeep's is, so one dropped sweep does not drop the heading). The
/// camera only aims the cannon now; nothing here waits for a window.
///
/// The controller is StrafeKeep's (fire::strafeKeep): the same deadband
/// compensation, integrators, caps, slew and "square first" rule, the same
/// gains, and the same live strafe.* overrides. It writes the same strafe_in /
/// strafe_block, so AwaitStrafeSolution judges "docked" for it unchanged (with
/// fire_range_m = standoff_m). ALWAYS SUCCESS: "not there yet" is a stick
/// command and a reason, never a FAILURE.
class SlotKeep : public CrusaderSyncAction
{
public:
  SlotKeep(const std::string & n, const BT::NodeConfig & c) : CrusaderSyncAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("standoff_m", 1.1, "LiDAR -> back wall to hold (along the slip)"),
      BT::InputPort<bool>("watch", false,
        "true: hold the WATCH standoff (live strafe.watch_range_m, else watch_range_m) "
        "instead, e.g. backed off in the slip to see the lights"),
      BT::InputPort<double>("watch_range_m", 1.6, "the watch standoff, when watch is true"),
      BT::InputPort<double>("lateral_bias_m", 0.0,
        "+ holds the hull this far LEFT of the slip's centreline"),
      BT::InputPort<double>("slot_timeout_s", 0.6, "slip older than this: every stick at zero"),
      BT::InputPort<double>("deadband_range_m", 0.04, ""),
      BT::InputPort<double>("deadband_lat_m", 0.04, ""),
      BT::InputPort<double>("deadband_yaw_deg", 2.0, ""),
      BT::InputPort<double>("kp_fwd", 90.0, "us per m"),
      BT::InputPort<double>("kd_fwd", 60.0, "us per m/s"),
      BT::InputPort<double>("kp_lat", 90.0, "us per m"),
      BT::InputPort<double>("kd_lat", 60.0, "us per m/s"),
      BT::InputPort<double>("kp_yaw", 4.0, "us per deg"),
      BT::InputPort<double>("kd_yaw", 3.0, "us per deg/s"),
      BT::InputPort<double>("min_us", 30.0, "added to every correction: the ESC deadband"),
      BT::InputPort<double>("max_us", 120.0, "deflection cap (the bridge caps it too)"),
      BT::InputPort<double>("slew_us_s", 200.0, "per axis"),
      BT::InputPort<double>("min_range_m", 0.9, "never push ahead with the LiDAR inside this"),
      BT::InputPort<double>("square_first_deg", 10.0, "further off square: turn only"),
      BT::InputPort<double>("ki_fwd", 20.0, "us per m.s (holds against a current)"),
      BT::InputPort<double>("ki_lat", 20.0, "us per m.s"),
      BT::InputPort<double>("i_max_us", 80.0, "integral cap"),
      BT::InputPort<double>("brake_mps", 0.03,
        "in the deadband but moving faster than this: brake (0 = never)"),
      BT::InputPort<double>("max_close_mps", 0.25,
        "approach speed limit along the slip, m/s (<= 0 = off); strafe.max_close_mps live"),
      BT::InputPort<double>("close_gain", 0.5, "the limit shrinks to this x the remaining m, per s"),
      BT::InputPort<double>("kv_close", 300.0, "us per m/s over the limit"),
      BT::InputPort<double>("centre_first_m", -1.0,
        "> 0: further than this off the slip's centreline, do not move in (hold the range) "
        "until the sideways stick has centred it; <= 0 = off"),
    };
  }

  BT::NodeStatus tick() override
  {
    fire::StrafeParams sp;
    auto in = [this](const char * k, double d) {return getInput<double>(k).value_or(d);};
    sp.fire_range_m = in("standoff_m", 1.1);
    sp.brake_mps = in("brake_mps", 0.03) > 0.0 ? in("brake_mps", 0.03) : fire::kNaN;
    sp.max_close_mps = in("max_close_mps", 0.25);
    sp.close_gain = in("close_gain", 0.5);
    sp.kv_close = in("kv_close", 300.0);
    sp.centre_first_m = in("centre_first_m", -1.0);
    sp.deadband_range_m = in("deadband_range_m", 0.04);
    sp.deadband_lat_m = in("deadband_lat_m", 0.04);
    sp.deadband_yaw_deg = in("deadband_yaw_deg", 2.0);
    sp.kp_fwd = in("kp_fwd", sp.kp_fwd); sp.kd_fwd = in("kd_fwd", sp.kd_fwd);
    sp.kp_lat = in("kp_lat", sp.kp_lat); sp.kd_lat = in("kd_lat", 60.0);
    sp.kp_yaw = in("kp_yaw", sp.kp_yaw); sp.kd_yaw = in("kd_yaw", sp.kd_yaw);
    sp.ki_fwd = in("ki_fwd", sp.ki_fwd); sp.ki_lat = in("ki_lat", 20.0);
    sp.i_max_us = in("i_max_us", sp.i_max_us);
    sp.min_us = in("min_us", sp.min_us); sp.max_us = in("max_us", sp.max_us);
    sp.slew_us_s = in("slew_us_s", sp.slew_us_s);
    sp.min_range_m = in("min_range_m", 0.9);
    sp.square_first_deg = in("square_first_deg", sp.square_first_deg);
    const double bias = in("lateral_bias_m", 0.0), slot_to = in("slot_timeout_s", 0.6);
    const bool watching = getInput<bool>("watch").value_or(false);
    const double watch_port = in("watch_range_m", 1.6);
    fire::StrafeCmd cmd;
    fire::StrafeInputs si;
    bool publish = false;
    std::string tuned, block;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      const double now = ctx_->now_s;
      tuned = ctx_->strafe_tune.apply(sp);
      if (watching) {
        sp.fire_range_m = ctx_->strafe_tune.watch_range_m >= 0.0 ?
          ctx_->strafe_tune.watch_range_m : watch_port;
      }
      // the standoff held now, for AwaitStrafeSolution and %RANGE%: after a
      // camera approach (task3_cannon_slip.xml) the last one is StrafeKeep's
      ctx_->strafe_target_range = sp.fire_range_m;
      ctx_->strafe_target_t = now;
      const double rate_s = ctx_->strafe_tune.rate_window_s >= 0.0 ?
        ctx_->strafe_tune.rate_window_s : 0.8;
      const bool fresh = ctx_->slot.valid_age(now) <= slot_to;
      if (fresh) {
        si.range_m = ctx_->slot.range(now);
        si.range_rate = ctx_->slot.rate(now, rate_s);
        si.lat_err_m = ctx_->slot.lat(now) - bias;
        si.lat_rate = ctx_->slot_lat.slope(now, rate_s);
        // the heading that squares the bow to the slip, from each sweep's own
        // heading: held through a turn, as StrafeKeep's face heading is
        const double square = ctx_->slot.inward_deg(now);
        if (std::isfinite(square) && std::isfinite(ctx_->heading_deg)) {
          si.yaw_err_deg = fire::wrap180(square - ctx_->heading_deg);
        }
      } else {
        block = "no slip from the LiDAR" + (ctx_->slot_why.empty() ? std::string() :
          " (" + ctx_->slot_why + ")");
      }
      si.yaw_rate_dps = ctx_->yaw_rate_dps;
      const double dt = ctx_->last_strafe_t < 0 ? 0.1 :
        std::max(0.0, now - ctx_->last_strafe_t);
      ctx_->last_strafe_t = now;
      if (!block.empty()) {
        cmd.why = block;                        // hold still: every stick at zero
      } else {
        cmd = fire::strafeKeep(sp, si, ctx_->strafe_state, dt);
      }
      ctx_->strafe_state.prev = cmd.sticks;
      ctx_->strafe = cmd;
      ctx_->strafe_in = si;
      ctx_->strafe_ref = "slip";
      ctx_->strafe_block = block;
      ctx_->steady.feed_cmd(now, cmd.correcting ? 1.0 : 0.0);
      publish = ctx_->publish_setpoints;
      if (publish) {ctx_->sticks_commanded = true;}
      if (ctx_->task3_phase.rfind("LINE UP", 0) != 0) {
        ctx_->task3_phase = "SLOT: " + cmd.why;
      }
    }
    if (publish && ctx_->sticks) {
      ctx_->sticks(cmd.sticks.fwd_us, cmd.sticks.lat_us, cmd.sticks.yaw_us);
    }
    if (ctx_->node) {
      const std::string live = tuned.empty() ? "" : " [live: " + tuned + "]";
      RCLCPP_INFO_THROTTLE(log(), *ctx_->node->get_clock(), 1000,
        "slot: LiDAR %.2f m from the back wall (holding %.2f%s), centreline %+.2f m left, "
        "square %+.1f deg | sticks fwd %+.0f lat %+.0f yaw %+.0f us (%s)%s%s",
        si.range_m, sp.fire_range_m, watching ? ", watch" : "", si.lat_err_m, si.yaw_err_deg,
        cmd.sticks.fwd_us, cmd.sticks.lat_us,
        cmd.sticks.yaw_us, cmd.why.c_str(),
        publish ? "" : " [shadow: publish_setpoints is off]", live.c_str());
    }
    return BT::NodeStatus::SUCCESS;
  }
};

/// Wait, RUNNING, until the LiDAR has had the WHOLE slip (DockSlot valid: both
/// side walls, or one and slip_width_m, and the back wall) for hold_s with no gap
/// over gap_s. The hand-over from the camera's approach to SlotKeep
/// (task3_cannon_slip.xml): one stray sweep does not hand the boat to a slip
/// that is not there. Says why not, every 2 s. FAILURE after timeout_s.
///
/// AND IT MUST BE THE BAY THE CAMERA IS STEERING FOR (match_face_m > 0): the
/// slip's centreline, carried out along its own axis to the camera's face (the
/// committed bay's, fire_face_x/y in the body frame), within match_face_m of
/// the face's centre. A slip is all the LiDAR knows; from 1-2 m off a slip's
/// line it fits the NEIGHBOURING one just as happily, and SlotKeep then docked
/// the boat in a RED bay (the sim, 2026-10-17, task3_slip_angle: handed over
/// 4.6 m out to the slip straight ahead, 1.9 m right of the GREEN one; the
/// water, 2026-10-16: the wrong wall ~1 m off the line). Slips are 2 m apart.
/// No face from the camera in the last face_max_age_s: not handed over.
class AwaitDockSlot : public CrusaderAction
{
public:
  AwaitDockSlot(const std::string & n, const BT::NodeConfig & c) : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("hold_s", 1.0, "the slip seen this long"),
      BT::InputPort<double>("gap_s", 0.5, "...with no sweep gap longer than this"),
      BT::InputPort<double>("timeout_s", 1e6, "give up after this"),
      BT::InputPort<double>("match_face_m", 0.6,
        "the slip must line up with the camera's committed face within this (<= 0 = any slip)"),
      BT::InputPort<double>("face_max_age_s", 1.0, "the camera's face older than this: no match")};
  }
  BT::NodeStatus onStart() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    t0_ = ctx_->now_s;
    since_ = -1.0;
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    const double hold = getInput<double>("hold_s").value_or(1.0);
    const double gap = getInput<double>("gap_s").value_or(0.5);
    const double match = getInput<double>("match_face_m").value_or(0.6);
    const double face_age = getInput<double>("face_max_age_s").value_or(1.0);
    double now = 0.0, range = fire::kNaN, lat = fire::kNaN;
    std::string why;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      now = ctx_->now_s;
      range = ctx_->slot.range(now);
      lat = ctx_->slot.lat(now);
      why = ctx_->slot_why;
      bool ok = ctx_->slot.valid_age(now) <= gap;
      if (ok && match > 0.0) {
        // the slip's centreline at the face's distance vs the face's centre
        const double fx = ctx_->fire_face_x.median(now, 0.5);
        const double fy = ctx_->fire_face_y.median(now, 0.5);
        const double square = ctx_->slot.inward_deg(now);
        const double ang = std::isfinite(square) && std::isfinite(ctx_->heading_deg) ?
          fire::wrap180(ctx_->heading_deg - square) : fire::kNaN;   // + = the slip points LEFT
        if (ctx_->fire_face_y.age(now) > face_age || !std::isfinite(fx) || !std::isfinite(fy) ||
          !std::isfinite(lat) || !std::isfinite(ang))
        {
          ok = false;
          why = "a slip, but no camera face to check it is the committed bay's";
        } else {
          const double y_at_face = lat + fx * std::tan(ang * fire::kPi / 180.0);
          const double off = y_at_face - fy;
          if (std::fabs(off) > match) {
            ok = false;
            char b[120];
            std::snprintf(b, sizeof(b), "a slip, but %.1f m %s of the committed bay's face: "
              "not that bay's", std::fabs(off), off > 0.0 ? "left" : "right");
            why = b;
          }
        }
      }
      if (ok) {
        if (since_ < 0.0) {since_ = now;}
      } else {
        since_ = -1.0;
      }
    }
    if (since_ >= 0.0 && now - since_ >= hold) {
      {
        // the hold changes reference (the camera's face -> the LiDAR's slip):
        // what the integrals built up against the old one starts over
        std::lock_guard<std::mutex> lk(ctx_->mu);
        ctx_->strafe_state.i_fwd = 0.0;
        ctx_->strafe_state.i_lat = 0.0;
      }
      RCLCPP_INFO(log(), "the LiDAR has the slip: back wall %.2f m, centreline %+.2f m left",
        range, lat);
      return BT::NodeStatus::SUCCESS;
    }
    if (ctx_->node) {
      const std::string note = since_ >= 0.0 ? " (seen, holding)" :
        (why.empty() ? std::string() : " (" + why + ")");
      RCLCPP_INFO_THROTTLE(log(), *ctx_->node->get_clock(), 2000,
        "waiting for the LiDAR to see the slip%s", note.c_str());
    }
    if (now - t0_ > getInput<double>("timeout_s").value_or(1e6)) {
      RCLCPP_WARN(log(), "AwaitDockSlot: no slip after %.0f s", now - t0_);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}

private:
  double t0_ = 0.0;
  double since_ = -1.0;
};

/// Wait for the autopilot to be in `mode` - for a mission whose next part runs
/// in another mode than the last (Task 3: the approach in GUIDED, the dock and
/// the shot in MANUAL). The TREE NEVER CHANGES THE MODE: the pilot does, on SC
/// (or the sim's operator script, on /crsd/set_mode). RUNNING until then, with
/// the phase saying what it waits for; FAILURE after timeout_s.
class AwaitMode : public CrusaderAction
{
public:
  AwaitMode(const std::string & n, const BT::NodeConfig & c) : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("mode", "MANUAL", "the mode to wait for"),
      BT::InputPort<double>("timeout_s", 120.0, "give up after this")};
  }

  BT::NodeStatus onStart() override
  {
    want_ = getInput<std::string>("mode").value_or("MANUAL");
    for (auto & ch : want_) {ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));}
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      t0_ = ctx_->now_s;
      ctx_->task3_phase = "WAITING FOR " + want_ + " (SC)";
    }
    RCLCPP_INFO(log(), "waiting for the pilot to select %s (SC)", want_.c_str());
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
      ctx_->task3_phase = "WAITING FOR " + want_ + " (SC)";
    }
    if (mode == want_) {
      {
        std::lock_guard<std::mutex> lk(ctx_->mu);
        ctx_->handed_mode = want_;       // a planned switch, not the pilot taking over
      }
      RCLCPP_INFO(log(), "mode is %s after %.1f s", want_.c_str(), now - t0_);
      return BT::NodeStatus::SUCCESS;
    }
    if (now - t0_ > getInput<double>("timeout_s").value_or(120.0)) {
      RCLCPP_WARN(log(), "AwaitMode: still %s after %.0f s, not %s", mode.c_str(),
        now - t0_, want_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  std::string want_;
  double t0_ = 0.0;
};

/// The run's OWN switch from `from` (GUIDED) to `mode` (MANUAL): the waypoint
/// run's hand-over, the transmitter left as set for GUIDED (task3_cannon_slip.xml,
/// task3_cannon_sim.xml; the user's call, 2026-10-18). Only FROM `from`: it
/// never takes a mode the pilot chose. Asks on /crsd/set_mode twice a second
/// until the autopilot reports `mode`, then SUCCESS; FAILURE after timeout_s or
/// when the mode turns into anything else. The pilot still outranks it: moving
/// SC re-applies the switch's mode (telemetry_bridge) and the run ends on the
/// next ModeIs; SD drops the overrides.
///
/// Shadow (publish_setpoints off): asks nothing, and waits like AwaitMode.
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
      if (mode == want_) {
        ctx_->handed_mode = want_;       // a planned switch, not the pilot taking over
      }
    }
    if (mode == want_) {
      RCLCPP_INFO(log(), "mode is %s after %.1f s", want_.c_str(), now - t0_);
      return BT::NodeStatus::SUCCESS;
    }
    if (mode != from_) {
      RCLCPP_WARN(log(), "RequestMode: the mode went to %s while switching to %s",
        mode.c_str(), want_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    if (now - t0_ > getInput<double>("timeout_s").value_or(10.0)) {
      RCLCPP_WARN(log(), "RequestMode: still %s after %.0f s, not %s", mode.c_str(),
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

/// Turn the bow straight into the committed bay's slip, on the yaw STICK only
/// (MANUAL): the heading along -berth.out (face -> water, reversed), from the
/// berth the approach computed. GUIDED setpoints carry no heading, so the
/// approach can stop on the line-up point pointing anywhere (bay 1 in the sim,
/// 2026-10-05: 74 deg off), and the LiDAR then has no slip ahead to see. SUCCESS
/// once within tol_deg and nearly still for hold_s; FAILURE with no berth, no
/// heading, or after turn_timeout_s. Shadow: succeeds at once (it cannot turn).
class FaceBay : public CrusaderAction
{
public:
  FaceBay(const std::string & n, const BT::NodeConfig & c) : CrusaderAction(n, c) {}
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("tol_deg", 6.0, "close enough to straight in"),
      BT::InputPort<double>("hold_s", 0.5, "...and nearly still this long"),
      BT::InputPort<double>("kp_yaw", 4.0, "us per deg"),
      BT::InputPort<double>("kd_yaw", 1.0, "us per deg/s"),
      BT::InputPort<double>("min_us", 30.0, "smallest yaw stick that turns the hull"),
      BT::InputPort<double>("max_us", 100.0, "yaw stick cap"),
      BT::InputPort<double>("turn_timeout_s", 30.0, "longer than this: FAILURE"),
    };
  }

  BT::NodeStatus onStart() override
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    nav::Vec2 out{0.0, 0.0};
    if (ctx_->berth.ok) {
      out = ctx_->berth.out;
    } else if (const dock::BayTrack * t = ctx_->dock.find(ctx_->chosen_track)) {
      out = t->outward();
    }
    if (nav::norm(out) < 1e-6 || !std::isfinite(ctx_->heading_deg)) {
      RCLCPP_WARN(log(), "FaceBay: no committed bay (or no heading) to face");
      return BT::NodeStatus::FAILURE;
    }
    target_ = nav::bearingDeg(nav::Vec2{0.0, 0.0}, nav::Vec2{-out.x, -out.y});
    t0_ = ctx_->now_s;
    held_since_ = -1.0;
    shadow_ = !ctx_->publish_setpoints;
    ctx_->task3_phase = "FACE THE BAY";
    RCLCPP_INFO(log(), "facing bay %d: heading %.0f, now %.0f", ctx_->chosen_bay, target_,
      ctx_->heading_deg);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    const double tol = getInput<double>("tol_deg").value_or(6.0);
    const double hold = getInput<double>("hold_s").value_or(0.5);
    double yaw_us = 0.0, err = fire::kNaN, now = 0.0;
    bool publish = false, done = false, fail = false;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      now = ctx_->now_s;
      if (shadow_) {
        done = true;
      } else if (!std::isfinite(ctx_->heading_deg)) {
        fail = true;
      } else {
        err = fire::wrap180(target_ - ctx_->heading_deg);           // + = turn right
        const double rate = ctx_->yaw_rate_dps;
        const bool there = std::fabs(err) <= tol && (!std::isfinite(rate) || std::fabs(rate) < 5.0);
        if (there) {
          if (held_since_ < 0.0) {held_since_ = now;}
          done = now - held_since_ >= hold;
        } else {
          held_since_ = -1.0;
          fail = now - t0_ > getInput<double>("turn_timeout_s").value_or(30.0);
        }
        if (!done && !fail) {
          yaw_us = fire::axisLaw(err, std::isfinite(rate) ? -rate : fire::kNaN,
            getInput<double>("kp_yaw").value_or(4.0), getInput<double>("kd_yaw").value_or(1.0),
            there ? 180.0 : 0.0,
            getInput<double>("min_us").value_or(30.0), getInput<double>("max_us").value_or(100.0));
        }
        publish = ctx_->publish_setpoints;
        if (publish) {ctx_->sticks_commanded = true;}
      }
    }
    if (publish && ctx_->sticks) {ctx_->sticks(0.0, 0.0, done || fail ? 0.0 : yaw_us);}
    if (fail) {
      RCLCPP_WARN(log(), "FaceBay: %s", std::isfinite(err) ? "the turn took too long" : "lost the heading");
      return BT::NodeStatus::FAILURE;
    }
    if (done) {
      RCLCPP_INFO(log(), "facing the bay%s", shadow_ ? " [shadow: publish_setpoints is off]" : "");
      return BT::NodeStatus::SUCCESS;
    }
    if (ctx_->node) {
      RCLCPP_INFO_THROTTLE(log(), *ctx_->node->get_clock(), 1000,
        "facing the bay: %+.1f deg to go | yaw stick %+.0f us", err, yaw_us);
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
  double target_ = 0.0, t0_ = 0.0, held_since_ = -1.0;
  bool shadow_ = false;
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
  factory.registerNodeType<LookAround>("LookAround");
  factory.registerNodeType<StrafeKeep>("StrafeKeep");
  factory.registerNodeType<AwaitStrafeSolution>("AwaitStrafeSolution");
  factory.registerNodeType<DockSlotAlive>("DockSlotAlive");
  factory.registerNodeType<SlotKeep>("SlotKeep");
  factory.registerNodeType<AwaitDockSlot>("AwaitDockSlot");
  factory.registerNodeType<AwaitMode>("AwaitMode");
  factory.registerNodeType<RequestMode>("RequestMode");
  factory.registerNodeType<StrafeSearch>("StrafeSearch");
  factory.registerNodeType<FaceBay>("FaceBay");
}

}  // namespace crusader_bt
