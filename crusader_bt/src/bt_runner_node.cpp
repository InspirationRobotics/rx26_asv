// bt_runner_node — runs one of crusader_bt's built-in trees per RunMission goal.
//
//     ros2 action send_goal /crsd/run_mission crusader_msgs/action/RunMission "{name: competition, tier: 1}"
//
// The goal NAMES a tree in this package's behavior_trees/; it never carries XML. One mission at
// a time. The action owns what the XML cannot be trusted with: the mission timeout, cancel
// (halts the tree, so every RUNNING leaf's onHalted() fires) and the terminal state.
//
//   in:  /crsd/fcu_status, /crsd/pose, /crsd/attitude, /crsd/world_targets, /crsd/passage_plan,
//        dock/observations, /crsd/ocs_command, /crsd/wall_range, /crsd/pump_state,
//        /crsd/autonomy_drop, /crsd/datum
//   out: /crsd/bt_status (JSON, 1 Hz idle / 2 Hz running), /crsd/robocommand/report,
//        /crsd/current_task, /crsd/autonomy_active, /crsd/avoidance_enable,
//        /crsd/nav/hazards, /crsd/nav/leg_status, /crsd/water_cannon, and, only with
//        publish_setpoints, /crsd/guided_setpoint, /crsd/guided_heading_speed, /crsd/rc_override
//        (fire_pump: /crsd/pump_cmd)
//
// THREADING. The tick loop sleeps, so it runs on its own thread (worker_), joined never
// detached, and the action server sits in a reentrant group under a MultiThreadedExecutor so a
// cancel is serviced while a goal executes. Cancel is polled at the top of every tick.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "crusader_bt/tree_view.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "crusader_msgs/action/run_mission.hpp"
#include "crusader_msgs/msg/attitude.hpp"
#include "crusader_msgs/msg/dock_observation.hpp"
#include "crusader_msgs/msg/fcu_status.hpp"
#include "crusader_msgs/msg/guided_heading_speed.hpp"
#include "crusader_msgs/msg/guided_setpoint.hpp"
#include "crusader_msgs/msg/hazard.hpp"
#include "crusader_msgs/msg/hazard_array.hpp"
#include "crusader_msgs/msg/lat_lon_head.hpp"
#include "crusader_msgs/msg/passage_plan.hpp"
#include "crusader_msgs/msg/pump_command.hpp"
#include "crusader_msgs/msg/pump_state.hpp"
#include "crusader_msgs/msg/rc_channels.hpp"
#include "crusader_msgs/msg/robo_command_report.hpp"
#include "crusader_msgs/msg/tracked_target_array.hpp"
#include "crusader_msgs/msg/wall_range.hpp"

#include "crusader_bt/context.hpp"
#include "crusader_bt/dock_math.hpp"
#include "crusader_bt/nav_math.hpp"
#include "crusader_bt/nav_params.hpp"
#include "crusader_bt/ros_planner_port.hpp"

namespace crusader_bt
{

using RunMission = crusader_msgs::action::RunMission;
using GoalHandle = rclcpp_action::ServerGoalHandle<RunMission>;
using namespace std::chrono_literals;

/// The idle task token. TASK_UNKNOWN is never sent: RoboCommand reads it as "unparsed".
static constexpr const char * kTaskNone = "TASK_NONE";

/// The goal cannot run: the message is the whole result detail.
struct GoalError : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

static const char * outcomeName(uint8_t o)
{
  switch (o) {
    case RunMission::Result::OUTCOME_SUCCESS: return "SUCCESS";
    case RunMission::Result::OUTCOME_FAILURE: return "FAILURE";
    case RunMission::Result::OUTCOME_TIMEOUT: return "TIMEOUT";
    case RunMission::Result::OUTCOME_CANCELLED: return "CANCELLED";
    case RunMission::Result::OUTCOME_NOT_AUTONOMOUS: return "NOT_AUTONOMOUS";
    default: return "FAULT";
  }
}

class BtRunner : public rclcpp::Node
{
public:
  BtRunner()
  // The node name MUST equal its section in crusader_params.yaml, or ROS hands
  // it a different node's parameters and it comes up misconfigured while
  // looking perfectly healthy.
  : rclcpp::Node("bt_runner_node")
  {
    action_name_ = declare_parameter<std::string>("action_name", "/crsd/run_mission");
    trees_dir_ = ament_index_cpp::get_package_share_directory("crusader_bt") + "/behavior_trees";
    tick_hz_ = declare_parameter<double>("tick_hz", 10.0);
    default_timeout_s_ = declare_parameter<double>("default_timeout_s", 600.0);
    mode_grace_s_ = declare_parameter<double>("mode_grace_s", 3.0);
    stream_timeout_s_ = declare_parameter<double>("stream_timeout_s", 1.0);
    assoc_radius_m_ = declare_parameter<double>("assoc_radius_m", 5.0);
    verbose_tree_ = declare_parameter<bool>("verbose_tree", true);
    auto modes = declare_parameter<std::vector<std::string>>(
      "autonomous_modes", std::vector<std::string>{"GUIDED", "AUTO", "LOITER", "RTL"});
    for (auto & m : modes) {
      for (auto & c : m) {c = static_cast<char>(std::toupper(c));}
      auto_modes_.insert(m);
    }
    ctx_ = std::make_shared<Context>();
    ctx_->node = this;
    ctx_->publish_setpoints = declare_parameter<bool>("publish_setpoints", false);
    // The fixed-nozzle shot may squirt: Gate G7, separate from moving (G1).
    ctx_->fire_pump = declare_parameter<bool>("fire_pump", false);
    // The sticks for the MANUAL fire tree (StrafeKeep): which RC channel is
    // [ahead, lateral, yaw] and its neutral, as the team's dp_hold drove it.
    stick_ch_ = declare_parameter<std::vector<int64_t>>("stick_channels", {3, 4, 1});
    stick_neutral_ = declare_parameter<std::vector<int64_t>>("stick_neutral_us", {1495, 1495, 1489});
    stick_reverse_ = declare_parameter<std::vector<bool>>("stick_reverse", {false, false, false});
    if (stick_ch_.size() != 3 || stick_neutral_.size() != 3 || stick_reverse_.size() != 3) {
      throw std::runtime_error("stick_channels / stick_neutral_us / stick_reverse need 3 entries "
        "each: [ahead, lateral, yaw]");
    }
    t0_ = std::chrono::steady_clock::now();

    // Task 3. THE CAMERA EXTRINSIC MUST EQUAL target_tracker's cam_x / cam_y /
    // cam_yaw_deg: two nodes placing one camera in two places put the bays and
    // the buoys in two different worlds, and neither looks wrong on its own.
    ctx_->cam_mount.x = declare_parameter<double>("cam_x", 0.37);
    ctx_->cam_mount.y = declare_parameter<double>("cam_y", 0.0);
    ctx_->cam_mount.yaw_deg = declare_parameter<double>("cam_yaw_deg", 0.0);
    ctx_->cam_mount.pitch_deg = declare_parameter<double>("cam_pitch_deg", 0.0);
    ctx_->dock_face_dz = declare_parameter<double>("dock_face_dz_m", 0.39);
    dock_topic_ = declare_parameter<std::string>("dock_topic", "dock/observations");
    declareNavParams();
    nav_set_cb_ = add_on_set_parameters_callback(&BtRunner::checkNavSet);

    // Latched: a subscriber that starts mid-mission must learn the current
    // value, not sit on a default. A late proximity_bridge defaulting
    // avoidance_enable to off would silently disarm avoidance.
    auto latched = rclcpp::QoS(1).reliable().transient_local();
    setpoint_pub_ = create_publisher<crusader_msgs::msg::GuidedSetpoint>(
      "/crsd/guided_setpoint", 10);
    task_pub_ = create_publisher<std_msgs::msg::String>("/crsd/current_task", latched);
    avoid_pub_ = create_publisher<std_msgs::msg::Bool>("/crsd/avoidance_enable", latched);
    autonomy_pub_ = create_publisher<std_msgs::msg::Bool>("/crsd/autonomy_active", 10);
    // Every report is kept: depth 20 and reliable, so a burst survives a slow ocs_client.
    report_pub_ = create_publisher<crusader_msgs::msg::RoboCommandReport>(
      "/crsd/robocommand/report", rclcpp::QoS(20).reliable());
    bt_status_pub_ = create_publisher<std_msgs::msg::String>("/crsd/bt_status", 10);
    cannon_pub_ = create_publisher<std_msgs::msg::String>("/crsd/water_cannon", 10);
    // The fixed-nozzle shot: GUIDED heading+speed and pump bursts, both through
    // telemetry_bridge, which gates each (mode + drop latch; pump_core).
    hs_pub_ = create_publisher<crusader_msgs::msg::GuidedHeadingSpeed>(
      "/crsd/guided_heading_speed", 10);
    pump_pub_ = create_publisher<crusader_msgs::msg::PumpCommand>("/crsd/pump_cmd", 10);
    // The sticks (MANUAL): telemetry_bridge forwards them only in MANUAL, only
    // on its override_channels, clamped, and releases them on silence.
    rc_pub_ = create_publisher<crusader_msgs::msg::RcChannels>("/crsd/rc_override", 10);
    // Obstacle avoidance. The hazards are the WHOLE known field, each message
    // replacing the last, so a late-starting costmap must get the current one:
    // latched, like the datum it is drawn in.
    hazards_pub_ = create_publisher<crusader_msgs::msg::HazardArray>(nav_hazards_topic_, latched);
    leg_pub_ = create_publisher<std_msgs::msg::String>(nav_leg_topic_, 10);

    status_sub_ = create_subscription<crusader_msgs::msg::FcuStatus>(
      "/crsd/fcu_status", 10,
      [this](crusader_msgs::msg::FcuStatus::SharedPtr m) {onStatus(m);});
    pose_sub_ = create_subscription<crusader_msgs::msg::LatLonHead>(
      "/crsd/pose", 10,
      [this](crusader_msgs::msg::LatLonHead::SharedPtr m) {onPose(m);});
    plan_sub_ = create_subscription<crusader_msgs::msg::PassagePlan>(
      "/crsd/passage_plan", 10, [this](crusader_msgs::msg::PassagePlan::SharedPtr m) {onPlan(m);});
    targets_sub_ = create_subscription<crusader_msgs::msg::TrackedTargetArray>(
      "/crsd/world_targets", 10,
      [this](crusader_msgs::msg::TrackedTargetArray::SharedPtr m) {onTargets(m);});
    dock_sub_ = create_subscription<crusader_msgs::msg::DockObservation>(
      dock_topic_, 10,
      [this](crusader_msgs::msg::DockObservation::SharedPtr m) {onDock(m);});
    ocs_sub_ = create_subscription<std_msgs::msg::String>(
      "/crsd/ocs_command", 10,
      [this](std_msgs::msg::String::SharedPtr m) {onOcsCommand(m);});
    wall_sub_ = create_subscription<crusader_msgs::msg::WallRange>(
      "/crsd/wall_range", 10,
      [this](crusader_msgs::msg::WallRange::SharedPtr m) {onWallRange(m);});
    att_sub_ = create_subscription<crusader_msgs::msg::Attitude>(
      "/crsd/attitude", 10,
      [this](crusader_msgs::msg::Attitude::SharedPtr m) {onAttitude(m);});
    pump_sub_ = create_subscription<crusader_msgs::msg::PumpState>(
      "/crsd/pump_state", 10,
      [this](crusader_msgs::msg::PumpState::SharedPtr m) {onPumpState(m);});
    // Latched on the bridge side (TRANSIENT_LOCAL): must match to get the
    // current value rather than wait for the next change.
    drop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/crsd/autonomy_drop", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::SharedPtr m) {
        std::lock_guard<std::mutex> lk(ctx_->mu);
        ctx_->drop_tripped = m->data;
      });
    // The frame, latched on the publisher's side so a restarted runner learns it. Without one,
    // nav_mode off pins the first fix (applyDatum, ingestPose).
    datum_sub_ = create_subscription<crusader_msgs::msg::LatLonHead>(
      nav_datum_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](crusader_msgs::msg::LatLonHead::SharedPtr m) {onDatum(m);});
    // The hazards go out whether or not a goal is active, so the costmap is
    // current before the first leg needs it.
    hazard_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / nav_hazard_rate_hz_), [this] {publishHazards();});

    wireContext();
    publishTask(kTaskNone);
    publishAvoidance(true);

    group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    server_ = rclcpp_action::create_server<RunMission>(
      this, action_name_,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const RunMission::Goal>) {
        if (busy_.exchange(true)) {
          RCLCPP_WARN(
            get_logger(),
            "REJECTING a second goal — one mission at a time. Cancel the "
            "running one first.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        RCLCPP_INFO(get_logger(), "cancel requested; halting at the next tick");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> gh) {
        // Own thread, joined never detached: see the file header. The previous
        // goal is always finished by the time a new one is accepted (busy_
        // gates that), so this join is instant.
        if (worker_.joinable()) {worker_.join();}
        worker_ = std::thread([this, gh] {execute(gh);});
      },
      rcl_action_server_get_default_options(), group_);
    idle_timer_ = create_wall_timer(1s, [this] {if (!busy_) {publishIdle();}});

    RCLCPP_INFO(
      get_logger(), "bt_runner ready on %s: trees %s, %.0f Hz, setpoints %s",
      action_name_.c_str(), builtinTrees().c_str(), tick_hz_,
      ctx_->publish_setpoints ? "ENABLED" : "DISABLED (read-only)");
    if (!ctx_->publish_setpoints) {
      RCLCPP_WARN(
        get_logger(),
        "publish_setpoints is FALSE — this node CANNOT move the boat. Legs "
        "still poll for arrival, so a human can drive the mission and the tree "
        "will follow. That is the read-only posture, not a fault.");
    }
    RCLCPP_INFO(
      get_logger(), "obstacle avoidance: nav_mode %s%s", navModeName(ctx_->nav_mode),
      ctx_->nav_mode == path::Mode::Off ?
      " (straight legs, no planner: a known hazard on the line holds the leg, then fails it)" :
      ctx_->nav_mode == path::Mode::Shadow ? " (plans and displays, drives the legacy legs)" :
      " (planned legs drive the boat)");
  }

  ~BtRunner() override {shutdown();}

private:
  // ------------------------------------------------------------ subscriptions

  void onStatus(const crusader_msgs::msg::FcuStatus::SharedPtr m)
  {
    std::string mode = m->mode;
    for (auto & c : mode) {c = static_cast<char>(std::toupper(c));}
    std::lock_guard<std::mutex> lk(ctx_->mu);
    ctx_->autonomous = auto_modes_.count(mode) > 0;
    ctx_->mode = mode;
    status_t_ = now();
    have_status_ = true;
  }

  // --- the fixed-nozzle shot: stamped on the runner's steady clock at
  // receipt, the same clock ctx_->now_s reads (see refreshFreshness) ---

  double nowS() const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
  }

  void onWallRange(const crusader_msgs::msg::WallRange::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    ingestWallRange(*ctx_, nowS(), m->valid, m->range_m, m->angle_deg, m->lat_m);
  }

  void onAttitude(const crusader_msgs::msg::Attitude::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    ingestAttitude(*ctx_, nowS(), m->roll, m->pitch, m->rollspeed, m->pitchspeed, m->yawspeed);
    att_t_ = nowS();
    have_att_ = true;
  }

  void onPumpState(const crusader_msgs::msg::PumpState::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    ctx_->pump_enabled = m->enabled;
    ctx_->pump_on = m->on;
    ctx_->pump_last_seq = m->last_seq;
    ctx_->pump_result = m->last_result;
    ctx_->pump_reason = m->last_reason;
    pump_t_ = nowS();
    have_pump_ = true;
  }

  /// The frame rule (spec 2) is ingestPose's, shared with the off-ROS runner.
  void onPose(const crusader_msgs::msg::LatLonHead::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    switch (ingestPose(*ctx_, {m->latitude, m->longitude}, m->heading)) {
      case PoseEvent::Bad:
        return;
      case PoseEvent::Pinned:
        // Pinned ONCE, at the first fix. Re-deriving it from the current position
        // each tick would make every stored buoy drift as the boat moves.
        RCLCPP_INFO(
          get_logger(), "local frame origin pinned at %.7f, %.7f (no /crsd/datum: pinned at "
          "the first fix; nav_mode off, frame not shared)", m->latitude, m->longitude);
        // A plan can arrive BEFORE the first fix (rxl_link_node is up long before
        // telemetry_bridge has a position), and refuseLocked() bails out with no
        // origin. Without this the plan sits unused until the next one, up to 5 s
        // at 0.2 Hz, and a goal sent then fails "entry buoy not available" while
        // the plan reads as fresh.
        refuseLocked();
        break;
      case PoseEvent::Waiting:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "waiting for /crsd/datum - is crusader_nav nav.launch.py running? "
          "(nav_mode %s does not pin the frame at the first fix)", navModeName(ctx_->nav_mode));
        break;
      case PoseEvent::Tracked:
        break;
    }
    pose_t_ = now();
  }

  /// /crsd/datum: the one frame origin (spec 2). The rule is applyDatum's.
  void onDatum(const crusader_msgs::msg::LatLonHead::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    switch (applyDatum(*ctx_, {m->latitude, m->longitude})) {
      case DatumEvent::Adopted:
        RCLCPP_INFO(
          get_logger(), "local frame origin adopted from /crsd/datum: %.7f, %.7f",
          m->latitude, m->longitude);
        refuseLocked();          // a plan that arrived before the origin can be placed now
        break;
      case DatumEvent::Mismatch:
        RCLCPP_ERROR(
          get_logger(), "/crsd/datum %.7f, %.7f differs from the origin in use (%.7f, %.7f): "
          "datum changed, restart bt_runner. Planned legs hold and hazards stop until then "
          "(straight legs are unaffected). Never re-pinned mid-run: the DockBook, home and "
          "the gate bookkeeping hold local coordinates.",
          m->latitude, m->longitude, ctx_->origin.lat, ctx_->origin.lon);
        break;
      case DatumEvent::Unchanged:
      case DatumEvent::Ignored:
        break;
    }
  }

  void onTargets(const crusader_msgs::msg::TrackedTargetArray::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->origin_set) {return;}     // nothing to express them relative to

    std::vector<nav::Buoy> next;
    next.reserve(m->targets.size());
    for (const auto & t : m->targets) {
      // TENTATIVE tracks are skipped: TrackedTarget.msg says one "may be one
      // frame of detector noise, so a mission should not commit to it", and
      // committing here means steering to a side of it or circling it as ENTRY.
      // The tracker publishes them for the AVOIDANCE consumer, not this one.
      if (!t.confirmed) {continue;}
      nav::Buoy b;
      b.id = static_cast<int>(t.id);
      b.p = nav::toLocal({t.latitude, t.longitude}, ctx_->origin);
      b.state = nav::beaconFromLabel(t.label);
      b.consumed = consumed_.count(b.id) > 0;
      next.push_back(b);
    }
    tracked_.swap(next);          // RAW. ctx_->buoys is the FUSION; see below.
    targets_t_ = now();
    refuseLocked();
  }

  /// The UAV's passage. Stored raw and folded in by refuseLocked(): a plan can
  /// arrive before there is an origin to express it in, and dropping it would
  /// lose the plan the whole mission is gated on.
  void onPlan(const crusader_msgs::msg::PassagePlan::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    plan_raw_ = m;
    refuseLocked();
  }

  /// Re-mark buoys the mission has already dealt with. Both regimes need it.
  void applyConsumed(std::vector<nav::Buoy> & buoys) const
  {
    for (auto & b : buoys) {b.consumed = consumed_.count(b.id) > 0;}
  }

  /// Rebuild ctx_->buoys from the plan and the tracker. CALL WITH ctx_->mu HELD.
  ///
  /// One place, called from both inputs, because fusing in each callback means
  /// two copies of the rule and only one of them gets the next fix.
  void refuseLocked()
  {
    if (!ctx_->origin_set) {return;}

    std::vector<nav::PlanBuoy> plan;
    if (plan_raw_) {
      plan.reserve(plan_raw_->buoys.size());
      for (const auto & pb : plan_raw_->buoys) {
        nav::PlanBuoy b;
        b.id = static_cast<int>(pb.id);
        b.p = nav::toLocal({pb.latitude, pb.longitude}, ctx_->origin);
        b.state = static_cast<nav::Beacon>(pb.beacon);
        plan.push_back(b);
      }
    }
    ctx_->plan = plan;

    // NO PLAN MEANS CORE TIER, and the tracker is the only source. fusePassage()
    // rightly calls every unmatched contact an obstacle ("the UAV wins"), but
    // against an EMPTY plan that puts the whole field in `obstacles` and the
    // Core tree sees no buoys at all.
    if (plan.empty()) {
      ctx_->buoys = tracked_;
      ctx_->obstacles.clear();
    } else {
      nav::Fused f = nav::fusePassage(plan, tracked_, assoc_radius_m_);
      ctx_->buoys.swap(f.passage);
      ctx_->obstacles.swap(f.obstacles);
    }
    // Consumption is the RUNNER's, not the fusion's: fusePassage is pure and
    // runs fresh on every input, so a buoy already dealt with would come back
    // un-consumed and the boat would steer to it again.
    applyConsumed(ctx_->buoys);

    if (plan_raw_) {
      // ENTRY and EXIT come from the PLAN above Core tier. Not from findBeacon
      // over what the boat can see: the EXIT sits ~92 m out in a full field,
      // far past the camera, and a transit that waits to SEE it never starts.
      ctx_->entry = nav::toLocal(
        {plan_raw_->entry_latitude, plan_raw_->entry_longitude}, ctx_->origin);
      ctx_->exitp = nav::toLocal(
        {plan_raw_->exit_latitude, plan_raw_->exit_longitude}, ctx_->origin);
      ctx_->have_entry = true;
      ctx_->have_exit = true;
      return;
    }
    // Core tier: no aircraft, so the boat's own eyes are all there is.
    const nav::Buoy * e = nav::findBeacon(ctx_->buoys, nav::Beacon::FlashingBlue);
    const nav::Buoy * x = nav::findBeacon(ctx_->buoys, nav::Beacon::SteadyBlue);
    ctx_->have_entry = e != nullptr;
    if (e) {ctx_->entry = e->p;}
    ctx_->have_exit = x != nullptr;
    if (x) {ctx_->exitp = x->p;}
  }

  // ---------------------------------------------------------------- outputs

  void wireContext()
  {
    ctx_->send_setpoint = [this](nav::LatLon ll) {
        crusader_msgs::msg::GuidedSetpoint sp;
        sp.header.stamp = now();
        sp.latitude = ll.lat;
        sp.longitude = ll.lon;
        sp.yaw = std::nan("");          // position only; the mask ignores yaw
        setpoint_pub_->publish(sp);
      };
    ctx_->set_task = [this](const std::string & t) {publishTask(t);};
    ctx_->consume_buoy = [this](int id) {consumed_.insert(id);};
    ctx_->report = [this](const std::string & kind, const std::string & body) {
        crusader_msgs::msg::RoboCommandReport m;
        m.header.stamp = now();
        m.kind = kind;
        m.body_json = body;
        report_pub_->publish(m);
      };
    ctx_->cannon = [this](bool fire, double x, double y, double z) {
        publishJson(cannon_pub_, dock::cannonJson(fire, x, y, z));
      };
    ctx_->heading_speed = [this](double heading, double speed) {
        crusader_msgs::msg::GuidedHeadingSpeed m;
        m.header.stamp = now();
        m.heading_deg = heading;
        m.speed_mps = speed;
        last_hs_heading_ = heading;
        hs_pub_->publish(m);
      };
    ctx_->pump = [this](double seconds, std::uint32_t seq) {
        crusader_msgs::msg::PumpCommand m;
        m.header.stamp = now();
        m.duration_s = static_cast<float>(seconds);
        m.seq = seq;
        m.source = "bt_runner";
        pump_pub_->publish(m);
      };
    ctx_->set_avoidance = [this](bool on) {publishAvoidance(on);};
    ctx_->sticks = [this](double fwd, double lat, double yaw) {
        const double v[3] = {fwd, lat, yaw};
        crusader_msgs::msg::RcChannels m;
        m.header.stamp = now();
        for (auto & c : m.channels) {c = 0;}              // everything else: the pilot's
        for (int i = 0; i < 3; ++i) {
          const int64_t ch = stick_ch_[i];
          if (ch < 1 || ch > 18) {continue;}
          const double d = stick_reverse_[i] ? -v[i] : v[i];
          m.channels[ch - 1] = static_cast<uint16_t>(std::lround(stick_neutral_[i] + d));
        }
        rc_pub_->publish(m);
      };
    ctx_->release_sticks = [this]() {
        crusader_msgs::msg::RcChannels m;
        m.header.stamp = now();
        for (auto & c : m.channels) {c = 0;}
        rc_pub_->publish(m);
      };
  }

  /// Motion was commanded last tick and not this one: say STOP, once. A leaf
  /// has no hook for "I stopped being ticked", and ArduRover carries on for 3 s
  /// - 0.9 m next to a dock. After every tick, and on every exit. The bridge's
  /// own dead-man (hs_deadman_s) is the backstop to this.
  void stopIfSilent(bool exiting)
  {
    bool commanded;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      commanded = ctx_->hs_commanded && !exiting;
      ctx_->hs_commanded = false;
    }
    if (hs_active_ && !commanded && std::isfinite(last_hs_heading_)) {
      crusader_msgs::msg::GuidedHeadingSpeed m;
      m.header.stamp = now();
      m.heading_deg = last_hs_heading_;
      m.speed_mps = 0.0;
      hs_pub_->publish(m);
    }
    hs_active_ = commanded;
    // ...and the sticks: released (handed back to the pilot), not held at neutral
    bool sticks;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      sticks = ctx_->sticks_commanded && !exiting;
      ctx_->sticks_commanded = false;
    }
    if (sticks_active_ && !sticks && ctx_->release_sticks) {ctx_->release_sticks();}
    sticks_active_ = sticks;
  }

  static void publishJson(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & pub, const std::string & s)
  {
    std_msgs::msg::String m;
    m.data = s;
    pub->publish(m);
  }

  // ------------------------------------------------------------------ Task 3

  /// DockObservation -> dock::Frame -> ingestDockObservation(). Field copies only (which range
  /// to believe is dock::sightingRange's call).
  void onDock(const crusader_msgs::msg::DockObservation::SharedPtr m)
  {
    dock::Frame f;
    // The CAMERA instant, not now(): the timing layer's 1 s flashes and the
    // tree's hold times are measured on these stamps.
    f.t = rclcpp::Time(m->header.stamp).seconds();
    for (const auto & b : m->bays) {
      dock::BaySighting s;
      s.bearing_deg = b.bearing_deg;
      s.range_m = dock::sightingRange(
        b.has_plane, b.plane_normal[0], b.plane_normal[1], b.plane_offset, b.bearing_deg,
        b.range_from_size_m);
      s.has_normal = b.has_plane && std::isfinite(b.plane_normal[0]) &&
        std::isfinite(b.plane_normal[1]) && std::isfinite(b.plane_normal[2]);
      s.nx = b.plane_normal[0];
      s.ny = b.plane_normal[1];
      s.nz = b.plane_normal[2];
      s.d = b.plane_offset;
      s.truncated = b.truncated;
      s.indicator_present = b.indicator_present;
      s.indicator = dock::colourFromCv(b.indicator_colour);
      s.indicator_conf = b.indicator_confidence;
      s.lit_window_index = b.lit_window_index;
      for (const auto & w : b.windows) {
        dock::WindowSighting ws;
        ws.index = w.index;
        ws.state = dock::colourFromCv(w.state);
        ws.conf = w.state_confidence;
        ws.has_position = w.has_position;
        ws.x = w.x;
        ws.y = w.y;
        ws.z = w.z;
        s.windows.push_back(ws);
      }
      f.bays.push_back(s);
    }
    f.target_pattern = m->target_pattern;
    for (const auto & c : m->target_colours) {f.target_colours.push_back(dock::colourFromName(c));}
    f.target_window_index = m->target_window_index;
    f.last_event = m->last_event;
    f.observed_fps = m->observed_fps;

    const rclcpp::Time t = now();
    std::lock_guard<std::mutex> lk(ctx_->mu);
    // Freshness at THIS instant: ingest places bays only on a fresh pose.
    ctx_->pose_fresh = ctx_->origin_set && (t - pose_t_).seconds() < stream_timeout_s_;
    ingestDockObservation(*ctx_, f);
    ingestFireWindows(*ctx_, nowS(), f);
    dock_t_ = t;
    have_dock_ = true;
  }

  /// ocs_client republishes RoboCommand's commands here as JSON and acts on none of them. Two
  /// matter: the RunStart WaitForRunStart waits for, and Task 3's ReadinessConfirm that answers
  /// the docking report. A key test is enough for each and keeps a JSON library out of this node.
  void onOcsCommand(const std_msgs::msg::String::SharedPtr m)
  {
    int run_id = -1;
    bool run_start;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      run_start = ingestRunStart(*ctx_, nowS(), m->data, &run_id);
    }
    if (run_start) {RCLCPP_INFO(get_logger(), "RoboCommand RunStart received (run_id %d)", run_id);}
    if (m->data.find("\"readiness_confirm\"") != std::string::npos) {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->readiness_confirmed = true;
      RCLCPP_INFO(get_logger(), "RoboCommand confirmed the docking report");
    }
  }

  void publishTask(const std::string & token)
  {
    {
      std::lock_guard<std::mutex> lk(status_mu_);
      current_task_ = token;
    }
    std_msgs::msg::String m;
    m.data = token;
    task_pub_->publish(m);
  }

  void publishAvoidance(bool on)
  {
    std_msgs::msg::Bool m;
    m.data = on;
    avoid_pub_->publish(m);
  }

  // ------------------------------------------------------- obstacle avoidance

  /// bt_runner_node's nav_* parameters (spec 5.8). Every default is path::NavParams{}'s, so the
  /// C++ and crusader_params.yaml cannot drift apart unnoticed (check_config cross-checks the
  /// ones the costmap also depends on).
  ///
  /// TWO KINDS, AND THE DESCRIPTOR SAYS WHICH. The planner knobs (path::kNavKnobs)
  /// are dynamic: reloadNavParams() copies them into ctx_->nav when a goal is accepted, so a
  /// `ros2 param set` or the ground station's Tuning tab applies at the next START. Everything
  /// else (rates, frames, topic and service names, the planner id, nav_mode) is wired into a
  /// timer, a topic or the planner port ONCE, so it is read_only: a live set would succeed and
  /// change nothing.
  void declareNavParams()
  {
    path::NavParams & n = ctx_->nav;
    for (const path::NavKnob & k : path::kNavKnobs) {
      const auto d = navKnobDescriptor(path::navRangeText(k));
      if (k.isWhole()) {
        const int64_t v = declare_parameter<int64_t>(k.name, static_cast<int64_t>(k.get(n)), d);
        k.set(n, static_cast<double>(v));
      } else {
        k.set(n, declare_parameter<double>(k.name, k.get(n), d));
      }
    }
    // A value the planner cannot work with stops the node here, like nav_mode below, rather than
    // waiting for a mission to find it. (The set-callback guards every later change.)
    const std::string bad_knob = path::checkNavParams(n);
    if (!bad_knob.empty()) {
      throw std::runtime_error(bad_knob + " (a nav_* value from crusader_params.yaml or -p)");
    }

    const rcl_interfaces::msg::ParameterDescriptor fixed = navFixedDescriptor();
    nav_hazard_rate_hz_ = declare_parameter<double>("nav_hazard_rate_hz", 2.0, fixed);
    nav_status_hz_ = declare_parameter<double>("nav_status_hz", 2.0, fixed);
    if (!(nav_hazard_rate_hz_ > 0.0) || !(nav_status_hz_ > 0.0)) {
      throw std::runtime_error("nav_hazard_rate_hz and nav_status_hz must be > 0");
    }
    nav_map_frame_ = declare_parameter<std::string>("nav_map_frame", "map", fixed);
    nav_datum_topic_ = declare_parameter<std::string>("nav_datum_topic", "/crsd/datum", fixed);
    nav_hazards_topic_ =
      declare_parameter<std::string>("nav_hazards_topic", "/crsd/nav/hazards", fixed);
    nav_leg_topic_ = declare_parameter<std::string>("nav_leg_topic", "/crsd/nav/leg_status", fixed);
    const std::string planner_action =
      declare_parameter<std::string>("nav_planner_action", "/compute_path_to_pose", fixed);
    const std::string valid_service =
      declare_parameter<std::string>("nav_valid_service", "/is_path_valid", fixed);
    const std::string clear_service = declare_parameter<std::string>(
      "nav_clear_service", "/global_costmap/clear_entirely_global_costmap", fixed);
    const std::string planner_id =
      declare_parameter<std::string>("nav_planner_id", "GridBased", fixed);

    // A typo here must stop the node, never fall back to a mode nobody asked for.
    //
    // DYNAMICALLY TYPED ON PURPOSE. `-p nav_mode:=on` is parsed as YAML 1.1, where a
    // bare on / off is a BOOLEAN, so a string-typed parameter dies at startup with
    // InvalidParameterTypeException on the spelling the docs use. (The YAML params
    // file quotes "off".) Both spellings mean what they say.
    rcl_interfaces::msg::ParameterDescriptor mode_desc = fixed;
    mode_desc.dynamic_typing = true;
    mode_desc.description = "off | shadow | on (a bare on or off on the command line is a "
      "YAML boolean and is accepted as that mode). Fixed at startup: the planner port is built "
      "from it once.";
    const rclcpp::ParameterValue & mode_value = declare_parameter(
      "nav_mode", rclcpp::ParameterValue(std::string("off")), mode_desc);
    std::string mode;
    if (mode_value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      mode = mode_value.get<std::string>();
    } else if (mode_value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      mode = mode_value.get<bool>() ? "on" : "off";
    }
    if (!parseNavMode(mode, ctx_->nav_mode)) {
      // Show what was given, with its type: an int or double has no mode string,
      // and "got ''" would not say which the operator wrote.
      throw std::runtime_error(
        "nav_mode must be off, shadow or on (a bare on or off boolean is that mode); got '" +
        rclcpp::to_string(mode_value) + "' (" + rclcpp::to_string(mode_value.get_type()) + ")");
    }
    if (ctx_->nav_mode == path::Mode::Off) {return;}
    ctx_->planner = makeRosPlannerPort(
      this, nav_map_frame_, planner_id, planner_action, valid_service, clear_service);
    if (!ctx_->planner) {
      throw std::runtime_error(
        "bt_runner was built without nav2_msgs (image without Nav2); nav_mode must be off");
    }
  }

  /// A planner knob's descriptor: when it applies, and what it may hold. The Tuning tab shows
  /// the description beside the row.
  static rcl_interfaces::msg::ParameterDescriptor navKnobDescriptor(const std::string & range)
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.description = "planner knob: applies at the next START (read when a goal is accepted, "
      "never mid-mission). Range: " + range;
    return d;
  }

  /// A nav_* key that is wired once at startup: refused live, in rclcpp's own words.
  static rcl_interfaces::msg::ParameterDescriptor navFixedDescriptor()
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;
    d.description = "fixed at startup (a timer, topic or the planner port is built from it "
      "once): change it in crusader_params.yaml and restart bt_runner_node";
    return d;
  }

  /// A numeric parameter as a double (a double or an integer); false for any other type.
  static bool numericValue(const rclcpp::Parameter & p, double * v)
  {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      *v = p.as_double();
    } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      *v = static_cast<double>(p.as_int());
    } else {
      return false;
    }
    return true;
  }

  /// The set-callback: a planner-knob value the planner cannot work with is refused when it is
  /// SET, with the reason, not left for the next mission to trip on. Other parameters pass.
  static rcl_interfaces::msg::SetParametersResult checkNavSet(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult r;
    r.successful = true;
    for (const rclcpp::Parameter & p : params) {
      double v = 0.0;
      if (!numericValue(p, &v)) {continue;}
      const std::string why = path::checkNavValue(p.get_name(), v);
      if (!why.empty()) {
        r.successful = false;
        r.reason = why;
        return r;
      }
    }
    return r;
  }

  /// The built-in tree `name` names, or GoalError. A bare stem only: never a path.
  std::string treePath(const std::string & name) const
  {
    const bool bare = !name.empty() && std::all_of(name.begin(), name.end(), [](char c) {
        return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
      });
    const std::string path = trees_dir_ + "/" + name + ".xml";
    std::error_code ec;
    if (!bare || !std::filesystem::is_regular_file(path, ec)) {
      throw GoalError("no built-in tree '" + name + "' (built-in: " + builtinTrees() + ")");
    }
    return path;
  }

  /// The built-in tree names, comma separated.
  std::string builtinTrees() const
  {
    std::vector<std::string> names;
    std::error_code ec;
    for (const auto & e : std::filesystem::directory_iterator(trees_dir_, ec)) {
      if (e.path().extension() == ".xml") {names.push_back(e.path().stem().string());}
    }
    std::sort(names.begin(), names.end());
    std::string out;
    for (const auto & n : names) {out += (out.empty() ? "" : ", ") + n;}
    return out.empty() ? "none" : out;
  }

  /// Re-reads every planner knob into ctx_->nav and logs the ones that changed since the
  /// previous goal. Called when a goal is ACCEPTED (execute(), before the tree is built) and
  /// never while a mission runs: a `ros2 param set` or a Tuning-tab set changes the PARAMETER at
  /// once, and the planner picks it up at the next START.
  void reloadNavParams()
  {
    path::NavParams fresh;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      fresh = ctx_->nav;
    }
    for (const path::NavKnob & k : path::kNavKnobs) {
      double v = 0.0;
      if (numericValue(get_parameter(k.name), &v)) {k.set(fresh, v);}
    }
    // checkNavSet refuses these when they are set, so this fires only for a value that got in
    // another way. Keep the previous, known-good set rather than plan with it.
    const std::string bad = path::checkNavParams(fresh);
    if (!bad.empty()) {
      RCLCPP_ERROR(
        get_logger(), "nav_*: NOT reloading, keeping the previous goal's values - %s", bad.c_str());
      return;
    }
    std::vector<path::NavChange> changes;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      changes = path::diffNavParams(ctx_->nav, fresh);
      ctx_->nav = fresh;
    }
    if (changes.empty()) {
      RCLCPP_INFO(get_logger(), "nav_*: unchanged since the previous goal");
    } else {
      RCLCPP_INFO(
        get_logger(), "nav_*: %zu changed since the previous goal: %s", changes.size(),
        path::describeNavChanges(changes).c_str());
    }
  }

  /// The known field, to the costmap (spec 5.4 item 4). A hazard in the wrong frame
  /// is a lethal blob in the wrong place, so nothing goes out until the origin IS
  /// the datum, and nothing after it stops being.
  void publishHazards()
  {
    crusader_msgs::msg::HazardArray msg;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!hazardsPublishable(*ctx_)) {return;}
      for (const path::Hazard & h : knownHazards(*ctx_)) {
        crusader_msgs::msg::Hazard m;
        m.kind = h.kind == path::HazardKind::Circle ? crusader_msgs::msg::Hazard::CIRCLE :
          crusader_msgs::msg::Hazard::POLYGON;
        m.source = static_cast<uint8_t>(h.source);
        m.id = h.id;
        m.x = h.c.x;
        m.y = h.c.y;
        m.radius_m = h.r;
        for (const nav::Vec2 & v : h.poly) {
          m.polygon_x.push_back(v.x);
          m.polygon_y.push_back(v.y);
        }
        m.keepout_m = h.keepout;
        msg.hazards.push_back(m);
      }
    }
    msg.header.frame_id = nav_map_frame_;
    msg.header.stamp = now();
    msg.seq = ++hazard_seq_;
    hazards_pub_->publish(msg);
  }

  /// The running leg, to the ground station (spec 4.3, 5.4 item 5), paced by
  /// leg_pacer_: a state change always goes out, otherwise nav_status_hz.
  /// Called from the tick thread only.
  void publishLegStatus(double elapsed_s)
  {
    std::string json;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!leg_pacer_.due(*ctx_, ctx_->now_s, 1.0 / nav_status_hz_)) {return;}
      json = legStatusJson(*ctx_, elapsed_s);
    }
    std_msgs::msg::String m;
    m.data = json;
    leg_pub_->publish(m);
  }

  /// A new mission, or the end of one: no leg is running, and the map should say so.
  void resetLeg(double elapsed_s)
  {
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      clearLeg(*ctx_);
    }
    publishLegStatus(elapsed_s);
  }

  /// Age out the streams. A mission acting on a pose it has not heard for a
  /// second is the frozen-pose failure the whole stack guards against.
  void refreshFreshness()
  {
    const rclcpp::Time t = now();
    std::lock_guard<std::mutex> lk(ctx_->mu);
    ctx_->pose_fresh = ctx_->origin_set &&
      (t - pose_t_).seconds() < stream_timeout_s_;

    // And so do the buoys: a dead tracker must not leave ctx_->buoys frozen and
    // perfectly convincing. Blanks over guesses.
    if (!tracked_.empty() && (t - targets_t_).seconds() >= stream_timeout_s_) {
      tracked_.clear();
      refuseLocked();
    }
    if (have_status_ && (t - status_t_).seconds() >= stream_timeout_s_) {
      ctx_->autonomous = false;        // a dead HEARTBEAT is not permission
    }
    // The dock detector publishes every frame, bays or not, so its silence is
    // a dead node. DockCameraAlive reads the age; the tree picks the limit.
    ctx_->dock_obs_age_s = have_dock_ ? (t - dock_t_).seconds() : 1e9;
    // The fixed-nozzle shot's clock and ages (steady clock, like its samples).
    ctx_->now_s = nowS();
    ctx_->att_age_s = have_att_ ? ctx_->now_s - att_t_ : 1e9;
    ctx_->pump_age_s = have_pump_ ? ctx_->now_s - pump_t_ : 1e9;
  }

  // ---------------------------------------------------------------- execute

  void execute(const std::shared_ptr<GoalHandle> gh)
  {
    const auto goal = gh->get_goal();
    const double timeout_s = goal->timeout_s > 0.0f ? goal->timeout_s : default_timeout_s_;
    const rclcpp::Time started = now();
    auto result = std::make_shared<RunMission::Result>();
    {
      std::lock_guard<std::mutex> lk(status_mu_);
      running_name_ = goal->name;
    }

    consumed_.clear();
    // The planner knobs as they are NOW: a change made since the last goal applies to this one.
    reloadNavParams();
    // Before pose_fresh is read below, or have_home holds what the last mission left.
    refreshFreshness();
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->approach_lat = goal->approach_latitude;
      ctx_->approach_lon = goal->approach_longitude;
      ctx_->has_approach = goal->approach_latitude != 0.0 || goal->approach_longitude != 0.0;
      ctx_->have_waypoint = false;
      // Per-mission state starts empty: a second attempt must not inherit the first's.
      ctx_->cleared_gates.clear();
      ctx_->global_passage = Context::GlobalPassage{};
      ctx_->tier = goal->tier;
      resetTask3(*ctx_);
      resetFire(*ctx_);
      goal_mode_ = ctx_->mode;
      // "Home" is where THIS attempt started, not the autopilot's HOME.
      ctx_->home = ctx_->boat;
      ctx_->have_home = ctx_->pose_fresh;
    }
    leg_pacer_ = LegStatusPacer{};
    resetLeg(0.0);

    uint8_t outcome = RunMission::Result::OUTCOME_FAULT;
    std::string detail;
    try {
      const std::string path = treePath(goal->name);
      BT::BehaviorTreeFactory factory;
      registerCrusaderNodes(factory);
      auto bb = BT::Blackboard::create();
      bb->set("ctx", ctx_);
      BT::Tree tree = factory.createTreeFromFile(path, bb);
      TreeView view(tree);

      RCLCPP_INFO(
        get_logger(), "GOAL %s tier=%u timeout=%.0fs approach=(%.7f, %.7f)", goal->name.c_str(),
        goal->tier, timeout_s, goal->approach_latitude, goal->approach_longitude);

      rclcpp::Rate rate(tick_hz_);
      auto fb = std::make_shared<RunMission::Feedback>();
      double next_status = 0.0;

      while (rclcpp::ok() && !stop_) {
        // Cancel first: a cancel that waits for anything does not work.
        if (gh->is_canceling()) {
          tree.haltTree();
          outcome = RunMission::Result::OUTCOME_CANCELLED;
          detail = "cancelled by the operator";
          break;
        }
        const double elapsed = (now() - started).seconds();
        if (elapsed >= timeout_s) {
          tree.haltTree();
          outcome = RunMission::Result::OUTCOME_TIMEOUT;
          detail = "mission timeout " + std::to_string(static_cast<int>(timeout_s)) + "s expired";
          break;
        }
        // An unknown mode is not a manual one: tolerated briefly at goal start, never for long.
        if (!have_status_ && elapsed > mode_grace_s_) {
          tree.haltTree();
          outcome = RunMission::Result::OUTCOME_FAULT;
          detail = "no HEARTBEAT: cannot tell whether we are allowed to drive";
          break;
        }

        refreshFreshness();
        const BT::NodeStatus st = tree.tickOnce();
        stopIfSilent(false);
        publishLegStatus(elapsed);
        if (verbose_tree_) {
          const std::string frame = view.renderIfChanged();
          if (!frame.empty()) {RCLCPP_INFO(get_logger(), "tree @ %5.1fs\n%s", elapsed, frame.c_str());}
        }
        // The mast light goes GREEN because a mission is ticking, not because a node is alive.
        std_msgs::msg::Bool a;
        a.data = true;
        autonomy_pub_->publish(a);
        if (elapsed >= next_status) {
          next_status = elapsed + kRunningStatusPeriodS;
          publishRunning(gh, fb, view, goal->name, elapsed);
        }

        if (st != BT::NodeStatus::RUNNING) {
          if (st == BT::NodeStatus::SUCCESS) {
            outcome = RunMission::Result::OUTCOME_SUCCESS;
            detail = "tree " + goal->name + " completed";
          } else {
            std::lock_guard<std::mutex> lk(ctx_->mu);
            // The pilot took it back = the mode CHANGED. A MANUAL tree is never "autonomous".
            if (ctx_->mode != goal_mode_ || (!ctx_->autonomous && goal_mode_ != "MANUAL")) {
              outcome = RunMission::Result::OUTCOME_NOT_AUTONOMOUS;
              detail = ctx_->mode.empty() ? std::string("no flight mode yet (no HEARTBEAT)") :
                "flight mode " + (ctx_->mode != goal_mode_ ?
                goal_mode_ + " -> " + ctx_->mode : ctx_->mode + " is not autonomous") +
                ": the pilot took control; not fighting for it";
            } else {
              outcome = RunMission::Result::OUTCOME_FAILURE;
              detail = "tree " + goal->name + " returned FAILURE (the tree log says which leaf)";
            }
          }
          break;
        }
        rate.sleep();
      }
    } catch (const GoalError & e) {
      detail = e.what();
      RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
    } catch (const std::exception & e) {
      // Never let this reach rclcpp_action: an unset Result's outcome 0 is SUCCESS.
      detail = std::string("tree raised: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
    }

    result->outcome = outcome;
    result->detail = detail;
    result->elapsed_s = (now() - started).seconds();
    if (outcome == RunMission::Result::OUTCOME_SUCCESS) {
      RCLCPP_INFO(get_logger(), "RESULT SUCCESS after %.1fs: %s", result->elapsed_s, detail.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "RESULT %s after %.1fs: %s", outcomeName(outcome),
        result->elapsed_s, detail.c_str());
    }
    {
      std::lock_guard<std::mutex> lk(status_mu_);
      last_ = "{\"name\":" + jsonQuote(goal->name) + ",\"outcome\":\"" + outcomeName(outcome) +
        "\",\"detail\":" + jsonQuote(detail) + ",\"elapsed_s\":" + jsonNum(result->elapsed_s, 1) + "}";
      running_name_.clear();
    }

    // Never succeed() a cancel: the caller would carry on as though the mission worked.
    if (outcome == RunMission::Result::OUTCOME_CANCELLED) {
      gh->canceled(result);
    } else if (outcome == RunMission::Result::OUTCOME_SUCCESS) {
      gh->succeed(result);
    } else {
      gh->abort(result);
    }

    // Every exit path, the exception one included: current_task back to NONE, the pump OFF
    // (a tree that throws never halts its leaves), the boat stopped if we were driving it.
    if (ctx_->cannon) {ctx_->cannon(false, 0.0, 0.0, 0.0);}
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->global_passage.plan = gp::Plan{};
    }
    stopIfSilent(true);
    resetLeg(result->elapsed_s);
    publishTask(kTaskNone);
    publishAvoidance(true);
    std_msgs::msg::Bool off;
    off.data = false;
    autonomy_pub_->publish(off);
    busy_ = false;
    publishIdle();
  }

  /// Feedback and /crsd/bt_status while a goal runs (kRunningStatusPeriodS).
  void publishRunning(
    const std::shared_ptr<GoalHandle> & gh, const std::shared_ptr<RunMission::Feedback> & fb,
    TreeView & view, const std::string & name, double elapsed)
  {
    std::string warning;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      warning = ctx_->pose_fresh ? "" : "pose is stale";
      if (ctx_->leg.state == "BLOCKED") {
        warning += std::string(warning.empty() ? "" : "; ") + "nav BLOCKED " + ctx_->leg.why;
      }
    }
    fb->elapsed_s = elapsed;
    fb->running_node = view.runningNode();
    fb->current_task = currentTask();
    fb->warning = warning;
    gh->publish_feedback(fb);

    std_msgs::msg::String m;
    m.data = "{\"state\":\"RUNNING\",\"name\":" + jsonQuote(name) +
      ",\"elapsed_s\":" + jsonNum(elapsed, 1) + ",\"running_node\":" + jsonQuote(fb->running_node) +
      ",\"current_task\":" + jsonQuote(fb->current_task) + ",\"warning\":" + jsonQuote(warning) +
      ",\"nodes\":" + view.nodesJson() + "}";
    bt_status_pub_->publish(m);
  }

  /// /crsd/bt_status between goals: IDLE and how the last one ended.
  void publishIdle()
  {
    std_msgs::msg::String m;
    {
      std::lock_guard<std::mutex> lk(status_mu_);
      m.data = "{\"state\":\"IDLE\",\"last\":" + (last_.empty() ? std::string("null") : last_) + "}";
    }
    bt_status_pub_->publish(m);
  }

  std::string currentTask()
  {
    std::lock_guard<std::mutex> lk(status_mu_);
    return current_task_;
  }

  /// Stop the mission thread and JOIN it before any member it touches is
  /// destroyed (see the file header).
  void shutdown()
  {
    stop_ = true;
    if (worker_.joinable()) {worker_.join();}
  }

  // ------------------------------------------------------------------ state
  static constexpr double kRunningStatusPeriodS = 0.5;
  ContextPtr ctx_;
  std::thread worker_;
  std::atomic<bool> stop_{false};
  std::string action_name_, trees_dir_;
  double tick_hz_ = 10.0, default_timeout_s_ = 600.0, mode_grace_s_ = 3.0;
  double stream_timeout_s_ = 1.0;
  double assoc_radius_m_ = 5.0;
  bool verbose_tree_ = true;
  std::set<std::string> auto_modes_;
  std::set<int> consumed_;
  std::atomic<bool> busy_{false};
  bool have_status_ = false;
  rclcpp::Time pose_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time status_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time targets_t_{0, 0, RCL_ROS_TIME};
  /// RAW inputs. ctx_->buoys is the FUSION of these two; see refuseLocked().
  std::vector<nav::Buoy> tracked_;
  crusader_msgs::msg::PassagePlan::SharedPtr plan_raw_;
  /// For /crsd/bt_status and feedback: the running goal's name, the current task token, and
  /// how the last goal ended (JSON). Guarded by status_mu_.
  std::mutex status_mu_;
  std::string running_name_, current_task_ = kTaskNone, last_;

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp_action::Server<RunMission>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr idle_timer_;
  rclcpp::Publisher<crusader_msgs::msg::GuidedSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_, bt_status_pub_;
  rclcpp::Publisher<crusader_msgs::msg::RoboCommandReport>::SharedPtr report_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr avoid_pub_, autonomy_pub_;
  rclcpp::Subscription<crusader_msgs::msg::FcuStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<crusader_msgs::msg::LatLonHead>::SharedPtr pose_sub_;
  rclcpp::Subscription<crusader_msgs::msg::TrackedTargetArray>::SharedPtr targets_sub_;
  rclcpp::Subscription<crusader_msgs::msg::PassagePlan>::SharedPtr plan_sub_;

  // Task 3
  std::string dock_topic_;
  rclcpp::Time dock_t_{0, 0, RCL_ROS_TIME};
  bool have_dock_ = false;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cannon_pub_;
  rclcpp::Subscription<crusader_msgs::msg::DockObservation>::SharedPtr dock_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ocs_sub_;

  // The fixed-nozzle shot
  std::chrono::steady_clock::time_point t0_{};
  double att_t_ = 0.0, pump_t_ = 0.0;   // nowS() at receipt; guarded by ctx_->mu
  bool have_att_ = false, have_pump_ = false;
  bool hs_active_ = false;              // motion commanded last tick (tick thread only)
  bool sticks_active_ = false;          // the sticks commanded last tick (tick thread only)
  std::string goal_mode_;               // the mode when this goal started (under ctx_->mu)
  std::vector<int64_t> stick_ch_, stick_neutral_;
  std::vector<bool> stick_reverse_;
  rclcpp::Publisher<crusader_msgs::msg::RcChannels>::SharedPtr rc_pub_;
  double last_hs_heading_ = std::nan("");
  rclcpp::Publisher<crusader_msgs::msg::GuidedHeadingSpeed>::SharedPtr hs_pub_;
  rclcpp::Publisher<crusader_msgs::msg::PumpCommand>::SharedPtr pump_pub_;
  rclcpp::Subscription<crusader_msgs::msg::WallRange>::SharedPtr wall_sub_;
  rclcpp::Subscription<crusader_msgs::msg::Attitude>::SharedPtr att_sub_;
  rclcpp::Subscription<crusader_msgs::msg::PumpState>::SharedPtr pump_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drop_sub_;

  // Obstacle avoidance (spec 5.4)
  double nav_hazard_rate_hz_ = 2.0, nav_status_hz_ = 2.0;
  std::string nav_map_frame_, nav_datum_topic_, nav_hazards_topic_, nav_leg_topic_;
  std::uint32_t hazard_seq_ = 0;        // hazard timer thread only
  LegStatusPacer leg_pacer_;            // tick thread only
  rclcpp::Publisher<crusader_msgs::msg::HazardArray>::SharedPtr hazards_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr leg_pub_;
  rclcpp::Subscription<crusader_msgs::msg::LatLonHead>::SharedPtr datum_sub_;
  rclcpp::TimerBase::SharedPtr hazard_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr nav_set_cb_;
};

}  // namespace crusader_bt

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // MultiThreadedExecutor is not optional — see the file header.
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<crusader_bt::BtRunner>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
