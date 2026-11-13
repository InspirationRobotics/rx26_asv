// bt_runner_node — runs one of crusader_bt's trees per RunMission goal.
//
//     ros2 action send_goal /crsd/run_mission crusader_msgs/action/RunMission "{name: task1_advanced}"
//
// The goal NAMES a tree: a built-in in this package's behavior_trees/, else a mission file in
// user_trees_dir that the ground station uploaded (mission_loader.hpp); it never carries XML. A
// name that is neither is REJECTED when the goal arrives, and the file is read again for every
// goal. One mission at a time. The action owns what the XML cannot be trusted with: the mission
// timeout, cancel (halts the tree, so every RUNNING leaf's onHalted() fires) and the terminal
// state.
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
#include "std_srvs/srv/trigger.hpp"

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
#include "crusader_bt/mission_loader.hpp"
#include "crusader_bt/nav_params.hpp"
#include "crusader_bt/ros_planner_port.hpp"

namespace crusader_bt
{

using RunMission = crusader_msgs::action::RunMission;
using GoalHandle = rclcpp_action::ServerGoalHandle<RunMission>;
using namespace std::chrono_literals;

/// Where the Core tier's ENTRY or EXIT in use comes from this run. Never from an earlier run:
/// the competition re-deals the light beacons, so nothing about them is kept between runs.
enum class AnchorSource {None, Plan, Seen};

static const char * sourceName(AnchorSource s)
{
  switch (s) {
    case AnchorSource::Plan: return "plan";
    case AnchorSource::Seen: return "seen";
    case AnchorSource::None: break;
  }
  return "none";
}

/// A plan position that means something: finite and not the (0, 0) a cleared or partial
/// PassagePlan carries for "no point".
static bool validLatLon(double lat, double lon)
{
  return std::isfinite(lat) && std::isfinite(lon) && std::fabs(lat) <= 90.0 &&
         std::fabs(lon) <= 180.0 && !(lat == 0.0 && lon == 0.0);
}

/// Does the plan hold any side rule? None = Core tier (the plan gives only the anchors).
static bool planHasSides(const std::vector<nav::PlanBuoy> & plan)
{
  for (const nav::PlanBuoy & b : plan) {
    if (nav::isSideConstrained(b.state)) {return true;}
  }
  return false;
}

struct Anchor
{
  bool have = false;
  nav::Vec2 p{};
  AnchorSource src = AnchorSource::None;
};

/// The point used for one beacon: the plan's, else the one the boat sees now.
static Anchor pickAnchor(bool plan_ok, nav::Vec2 plan_p, const nav::Buoy * seen)
{
  if (plan_ok) {return {true, plan_p, AnchorSource::Plan};}
  if (seen != nullptr) {return {true, seen->p, AnchorSource::Seen};}
  return {};
}

/// The idle task token. TASK_UNKNOWN is never sent: RoboCommand reads it as "unparsed".
static constexpr const char * kTaskNone = "TASK_NONE";

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
    tick_hz_ = declare_parameter<double>("tick_hz", 10.0);
    default_timeout_s_ = declare_parameter<double>("default_timeout_s", 600.0);
    declare_parameter<double>("dock_heading_deg", -1.0);   // live: the GCS sets it, read per goal
    declare_parameter<std::string>("search_first", "");    // live, per goal: left | right | "" = the tree's
    // The operator's mission inputs (the GCS Run panel sets them; gcs_bridge keeps them across a
    // restart of this node). Read at every Start, like the two above.
    declare_parameter<double>("t1_field_lat", 0.0);        // Task 1 field's rough centre; 0, 0 = unset
    declare_parameter<double>("t1_field_lon", 0.0);
    declare_parameter<double>("t1_scan_dwell_s", 5.0);     // each look of a scan, each mid-field pause
    declare_parameter<double>("t1_pause_every_m", 5.0);    // a look every this far along the transit; 0 = none
    declare_parameter<double>("t1_approach_lat", 0.0);    // Task 1: drive here, then scan; 0, 0 = scan from where the boat is
    declare_parameter<double>("t1_approach_lon", 0.0);
    declare_parameter<double>("t1_unknown_range_m", 12.0); // stop to look at a buoy with no colour this near ahead; 0 = never
    declare_parameter<double>("t3_backoff_m", 4.0);        // the docked hold once the fire is out
    declare_parameter<double>("t3_via_lat", 0.0);          // Task 1 -> Task 3 through here; 0, 0 = straight
    declare_parameter<double>("t3_via_lon", 0.0);
    // Task 4 Disruptive (task4_math.hpp): re-read every second, so a change applies at once.
    declare_parameter<bool>("t4_enabled", true);           // false: alerts shown, never acted on
    declare_parameter<double>("t4_margin_m", 3.0);         // kept beyond the object's keep-out
    declare_parameter<double>("t4_radius_m", 10.0);        // keep-out when an alert has none (3.3.5)
    declare_parameter<double>("t4_horizon_s", 40.0);       // its projected travel this far ahead
    mode_grace_s_ = declare_parameter<double>("mode_grace_s", 3.0);
    // Backstop for the RunStart gate: a RunStart older than this never opens it, whatever the
    // OCS did or did not clear. An hour covers a long wait for the previous team's run.
    run_start_max_age_s_ = declare_parameter<double>("run_start_max_age_s", 3600.0);
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
    // Copied into the context once, so read_only: a live set would succeed and change nothing.
    rcl_interfaces::msg::ParameterDescriptor posture;
    posture.read_only = true;
    posture.description = "fixed at startup: change it in crusader_params.yaml and restart bt_runner_node";
    ctx_->publish_setpoints = declare_parameter<bool>("publish_setpoints", false, posture);
    // Where uploaded missions live (mission_loader.hpp): inside the workspace bind mount, like
    // gcs_bridge's record_dir. gcs_bridge's user_trees_dir MUST equal it (check_config).
    const std::string user_trees_dir =
      declare_parameter<std::string>("user_trees_dir", "/root/robotx_ws/missions", posture);
    dirs_ = missionDirs(user_trees_dir);
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
    // SetCameraModel sets this node's live `model` parameter, as the GCS Sensors tab does.
    camera_node_ = declare_parameter<std::string>("camera_node", "/camera_node");
    camera_params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, camera_node_);
    declareNavParams();
    nav_set_cb_ = add_on_set_parameters_callback(&BtRunner::checkNavSet);

    // Latched: a subscriber that starts mid-mission must learn the current
    // value, not sit on a default. A late lidar_node defaulting
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
    set_mode_pub_ = create_publisher<std_msgs::msg::String>("/crsd/set_mode", 10);
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
    // Latched by ocs_client_node: a runner restarted mid-run still has the course.
    course_sub_ = create_subscription<std_msgs::msg::String>(
      "/crsd/ocs_course", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::SharedPtr m) {onCourse(m);});
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
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const RunMission::Goal> goal) {
        // A name that is no file is refused here, not accepted and then FAULTed in 0 s.
        try {
          const std::string warning = resolveMission(dirs_, goal->name).warning;
          if (!warning.empty()) {RCLCPP_WARN(get_logger(), "%s", warning.c_str());}
        } catch (const MissionError & e) {
          RCLCPP_WARN(get_logger(), "REJECTING goal: %s", e.what());
          return rclcpp_action::GoalResponse::REJECT;
        }
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
    // The bench/lake release: lets the ONE goal held at the RunStart gate start without
    // RoboCommand. Answers only while a goal is held, so a stray call cannot pre-release the next.
    release_srv_ = create_service<std_srvs::srv::Trigger>(
      "/crsd/bt_runner/release_run_start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (gate_waiting_) {
          bench_release_ = true;
          res->success = true;
          res->message = "released the waiting mission without RoboCommand's RunStart";
        } else {
          res->success = false;
          res->message = "no mission is waiting for RunStart";
        }
      });
    // Task 1 between runs: every mission start makes target_tracker forget the buoys' COLOURS
    // (the competition re-deals the light beacons, red and green, ENTRY and EXIT, every run).
    // Nothing about ENTRY/EXIT is kept: they come from what the boat sees this run.
    forget_colours_pub_ =
      create_publisher<std_msgs::msg::String>("/crsd/world_model/forget_colours", 10);
    idle_timer_ = create_wall_timer(1s, [this] {if (!busy_) {publishIdle();}});

    RCLCPP_INFO(
      get_logger(),
      "bt_runner ready on %s: built-in trees %s; your missions in %s: %s; %.0f Hz, setpoints %s",
      action_name_.c_str(), joinNames(missionNames(dirs_.builtin)).c_str(), dirs_.user.c_str(),
      joinNames(missionNames(dirs_.user)).c_str(), tick_hz_,
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
        // A plan can arrive BEFORE the first fix (rfd_link_node is up long before
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
      b.hits = static_cast<int>(t.hits);
      b.conf = t.confidence;
      next.push_back(b);
    }
    tracked_.swap(next);          // RAW. ctx_->buoys is the FUSION; see below.
    targets_t_ = now();
    refuseLocked();
  }

  /// The UAV's passage. Stored raw and folded in by refuseLocked(): a plan can
  /// arrive before there is an origin to express it in, and dropping it would
  /// lose the plan the whole mission is gated on. The receipt time is kept too:
  /// the stored plan is latched, so its age is what tells a live link from a dead one.
  void onPlan(const crusader_msgs::msg::PassagePlan::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    plan_raw_ = m;
    plan_t_ = now();
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
    // A plan with NO red or green (the ground station's MANUAL field holding only ENTRY/EXIT,
    // or a cleared one) is CORE TIER too: it gives at most the anchors, and the reds and greens
    // are the boat's own, with the sensor replans (global_leaves coreMode: plan empty).
    if (!planHasSides(plan)) {plan.clear();}
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

    // The plan's ENTRY/EXIT, where it has them: (0, 0) is a cleared plan's "no point".
    const bool plan_e = plan_raw_ &&
      validLatLon(plan_raw_->entry_latitude, plan_raw_->entry_longitude);
    const bool plan_x = plan_raw_ &&
      validLatLon(plan_raw_->exit_latitude, plan_raw_->exit_longitude);
    const nav::Vec2 pe = plan_e ?
      nav::toLocal({plan_raw_->entry_latitude, plan_raw_->entry_longitude}, ctx_->origin) :
      nav::Vec2{};
    const nav::Vec2 px = plan_x ?
      nav::toLocal({plan_raw_->exit_latitude, plan_raw_->exit_longitude}, ctx_->origin) :
      nav::Vec2{};
    if (!plan.empty()) {
      // ENTRY and EXIT come from the PLAN above Core tier. Not from findBeacon
      // over what the boat can see: the EXIT sits ~92 m out in a full field,
      // far past the camera, and a transit that waits to SEE it never starts.
      useAnchor("ENTRY", pickAnchor(plan_e, pe, nullptr), ctx_->entry, ctx_->have_entry,
        entry_src_);
      useAnchor("EXIT", pickAnchor(plan_x, px, nullptr), ctx_->exitp, ctx_->have_exit, exit_src_);
      ctx_->entry_seen = ctx_->have_entry;
      ctx_->exit_seen = ctx_->have_exit;
      ctx_->exit_guessed = false;
      return;
    }
    // Core tier: the MANUAL field's points first, then the boat's own eyes (pickAnchor says the
    // order).
    // By EVIDENCE and sticky (nav::bestBeacon), not the first in the list: labels flicker, and
    // on the real course unlit buoys were often called blue.
    const nav::Buoy * e = chooseBeacon("ENTRY", nav::Beacon::FlashingBlue, entry_track_id_,
        exit_track_id_);
    const nav::Buoy * x = chooseBeacon("EXIT", nav::Beacon::SteadyBlue, exit_track_id_,
        entry_track_id_);
    useAnchor("ENTRY", pickAnchor(plan_e, pe, e), ctx_->entry, ctx_->have_entry, entry_src_);
    useAnchor("EXIT", pickAnchor(plan_x, px, x), ctx_->exitp, ctx_->have_exit, exit_src_);
    // Plan or seen are the only sources, so a point in use is a point seen or given.
    ctx_->entry_seen = ctx_->have_entry;
    ctx_->exit_seen = ctx_->have_exit;
    guessExit();
  }

  /// Core tier, no EXIT known (not seen or given) but the ENTRY and the operator's
  /// field centre are: GUESS it at the far end of the field, the centre mirrored through the
  /// ENTRY (on the 2026-11-10 field that lands 1.1 m from the real EXIT). Only to plan the approach
  /// and the transit with: the boat's own sighting replaces it, and an EXIT that moves before its
  /// orbit re-plans. Only for a mission whose root says t1_guess_exit="true" (ctx_->guess_exit);
  /// any other tree plans round what it has seen or been given. CALL WITH ctx_->mu HELD.
  void guessExit()
  {
    const bool was = ctx_->exit_guessed;
    ctx_->exit_guessed = false;
    nav::Vec2 c;
    if (ctx_->guess_exit && !ctx_->have_exit && ctx_->have_entry && fieldCentre(*ctx_, &c)) {
      const nav::Vec2 g = c + (c - ctx_->entry);
      if (nav::norm(g - ctx_->entry) >= 4.0) {      // an ENTRY at the centre gives no axis
        ctx_->exitp = g;
        ctx_->have_exit = true;
        ctx_->exit_guessed = true;
      }
    }
    if (ctx_->exit_guessed != was) {
      if (ctx_->exit_guessed) {
        const nav::LatLon ll = nav::toLatLon(ctx_->exitp, ctx_->origin);
        RCLCPP_INFO(get_logger(), "EXIT GUESSED at %.7f, %.7f (the field centre mirrored through "
          "the ENTRY) until the boat sees it", ll.lat, ll.lon);
      } else {
        RCLCPP_INFO(get_logger(), "EXIT no longer guessed (%s)", ctx_->have_exit ?
          sourceName(exit_src_) : "no ENTRY or field centre");
      }
    }
  }

  /// Set one of the context's ENTRY/EXIT and log where it now comes from, once per change.
  /// CALL WITH ctx_->mu HELD.
  void useAnchor(const char * name, const Anchor & a, nav::Vec2 & p, bool & have,
    AnchorSource & src)
  {
    have = a.have;
    if (a.have) {p = a.p;}
    if (a.src != src) {
      RCLCPP_INFO(get_logger(), "%s from %s (was %s)%s", name, sourceName(a.src),
        sourceName(src), a.have ? "" : ": none known yet");
      src = a.src;
    }
  }

  /// A new run: the competition re-deals red and green, so target_tracker forgets every buoy's
  /// colour (it keeps where they are), and this run's ENTRY/EXIT choice starts afresh.
  void forgetColours(const std::string & mission)
  {
    std_msgs::msg::String m;
    m.data = "mission start: " + mission;
    forget_colours_pub_->publish(m);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    entry_track_id_ = exit_track_id_ = -1;
    RCLCPP_INFO(get_logger(), "mission %s: asked target_tracker to forget the buoy colours",
      mission.c_str());
  }

  /// The Core tier's ENTRY/EXIT for /crsd/bt_status: where each comes from now (seen, guessed
  /// or none; the ground station's Map tab shows them). Takes ctx_->mu.
  std::string task1Json()
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    nav::Vec2 fc;
    const bool have_fc = fieldCentre(*ctx_, &fc);
    return std::string("{\"entry_src\":\"") + sourceName(entry_src_) +
      "\",\"exit_src\":\"" + (ctx_->exit_guessed ? "guessed" : sourceName(exit_src_)) +
      "\",\"exit_guess\":" + jsonLatLon(*ctx_, ctx_->exit_guessed, ctx_->exitp) +
      ",\"field_centre\":" + jsonLatLon(*ctx_, have_fc, fc) + "}";
  }

  /// Task 3 for /crsd/bt_status, which the run recording keeps whole: the phase, the bay
  /// committed to, every track across/along the dock (m, + across = right facing the bays)
  /// with its colour reads (r, g, off; other = every non-RED/GREEN read, off included), its
  /// GREEN confidence and verdict under the rule CommitFirstGreen used last ("rule"), seconds
  /// since it was seen, and what the hold steered on. The 11-12 dock runs had only the
  /// throttled log to say where the tree was.
  std::string task3Json()
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const Context & c = *ctx_;
    const nav::Vec2 f = nav::headingVec(c.dock_heading >= 0.0 ? c.dock_heading : c.heading_deg);
    const dock::ColourRule & rule = c.colour_rule;
    std::string tracks;
    for (const auto & t : c.dock.tracks) {
      tracks += std::string(tracks.empty() ? "" : ",") + "{\"id\":" + std::to_string(t.id) +
        ",\"n\":" + std::to_string(t.n) + ",\"across\":" + jsonNum(nav::dot(t.p, nav::starboardOf(f)), 2) +
        ",\"along\":" + jsonNum(nav::dot(t.p, f), 2) + ",\"r\":" + std::to_string(t.red) +
        ",\"g\":" + std::to_string(t.green) + ",\"off\":" + std::to_string(t.off) +
        ",\"other\":" + std::to_string(t.other) +
        ",\"conf\":" + jsonNum(dock::greenConfidence(t, rule.off_is_green), 2) +
        ",\"v\":\"" + dock::verdictChar(dock::confidenceVerdict(t, rule)) + "\"" +
        ",\"age_s\":" + jsonNum(c.now_s - t.last_seen, 1) + "}";
    }
    const std::string rule_json = "{\"off_is_green\":" +
      std::string(rule.off_is_green ? "true" : "false") + ",\"min_reads\":" +
      std::to_string(rule.min_reads) + ",\"min_conf\":" + jsonNum(rule.min_conf, 2) + "}";
    const fire::StrafeInputs & si = c.strafe_in;
    const fire::SteadyStatus st = c.steady.status(c.now_s);
    return "{\"phase\":" + jsonQuote(c.task3_phase) + ",\"bay\":" + std::to_string(c.chosen_bay) +
      ",\"track\":" + std::to_string(c.chosen_track) + ",\"frames\":" +
      std::to_string(c.dock.frames) + ",\"obs_age_s\":" + jsonNum(c.dock_obs_age_s, 1) +
      ",\"course\":" + jsonQuote(c.course_id) + ",\"fenced\":" + std::to_string(c.dock.fenced) +
      ",\"rule\":" + rule_json + ",\"tracks\":[" + tracks + "],\"range_m\":" + jsonNum(si.range_m, 2) + ",\"lat_err_m\":" +
      jsonNum(si.lat_err_m, 2) + ",\"yaw_err_deg\":" + jsonNum(si.yaw_err_deg, 1) +
      ",\"steady\":" + jsonQuote(st.steady ? "yes" : st.why) + "}";
  }

  /// The tracked buoy that is the Core tier's ENTRY or EXIT: bestBeacon over ctx_->buoys, never
  /// the track the other one holds. `remembered` is updated, and a change of choice logged once
  /// (old -> new, position, hits). CALL WITH ctx_->mu HELD.
  const nav::Buoy * chooseBeacon(const char * name, nav::Beacon want, int & remembered, int other)
  {
    const nav::Buoy * b = nav::bestBeacon(ctx_->buoys, want, remembered, other);
    if (b != nullptr && b->id != remembered) {
      RCLCPP_INFO(get_logger(), "%s track %d -> %d at (%.1f, %.1f) m, %d hits, conf %.2f", name,
        remembered, b->id, b->p.x, b->p.y, b->hits, b->conf);
      remembered = b->id;
    }
    return b;
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
    ctx_->request_mode = [this](const std::string & mode) {
        std_msgs::msg::String m;
        m.data = mode;
        set_mode_pub_->publish(m);
      };
    ctx_->set_camera_model = [this](const std::string & model, const std::string & indicator,
        std::string & why) {
        return askCameraModel(model, indicator, why);
      };
    ctx_->release_sticks = [this]() {
        crusader_msgs::msg::RcChannels m;
        m.header.stamp = now();
        for (auto & c : m.channels) {c = 0;}
        rc_pub_->publish(m);
      };
  }

  /// SetCameraModel's ask: camera_node's `model`, and its `dock_indicator` when `indicator`
  /// is not "". Both are read first and only what differs is set: a `model` set reloads the
  /// model even to the value in force, seconds with no detections (`dock_indicator` never
  /// reloads). The answer lands in ctx_->camera_model_ack, and only the newest ask's does.
  bool askCameraModel(const std::string & model, const std::string & indicator, std::string & why)
  {
    if (!camera_params_->service_is_ready()) {
      why = camera_node_ + " has no parameter service: is it running?";
      return false;
    }
    const int ask = ++camera_ask_;
    askCamera(model, indicator, [this, ask](int ack, const std::string & reason) {
        std::lock_guard<std::mutex> lk(ctx_->mu);
        if (ask != camera_ask_) {return;}
        ctx_->camera_model_ack = ack;
        ctx_->camera_model_why = reason;
      });
    return true;
  }

  /// askCameraModel's read, then set. rclpy answers a read that names an undeclared
  /// parameter with no values: a camera_node from before dock_indicator, which reads the
  /// indicator the yolo way and no other, so for yolo it is asked again for the model alone.
  void askCamera(
    const std::string & model, const std::string & indicator,
    std::function<void(int, const std::string &)> answer)
  {
    using Params = std::shared_future<std::vector<rclcpp::Parameter>>;
    using Results = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
    std::vector<std::string> names{"model"};
    if (!indicator.empty()) {names.push_back("dock_indicator");}
    camera_params_->get_parameters(names, [this, model, indicator, names, answer](Params f) {
        try {
          const auto now = f.get();
          auto holds = [&now](std::size_t i, const std::string & want) {
              return i < now.size() &&
                     now[i].get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
                     now[i].as_string() == want;
            };
          if (!indicator.empty() && (now.size() < names.size() ||
            now[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING))
          {
            if (indicator == "yolo") {
              askCamera(model, "", answer);
              return;
            }
            answer(-1, "camera_node has no dock_indicator parameter (an older build): it "
              "cannot read the indicator by " + indicator);
            return;
          }
          std::vector<rclcpp::Parameter> set;
          if (!holds(0, model)) {set.emplace_back("model", model);}
          if (!indicator.empty() && !holds(1, indicator)) {
            set.emplace_back("dock_indicator", indicator);
          }
          if (set.empty()) {
            answer(1, "");
            return;
          }
          camera_params_->set_parameters(set, [answer](Results r) {
              try {
                const auto res = r.get();
                if (res.empty()) {answer(-1, "an empty reply"); return;}
                for (const auto & one : res) {
                  if (!one.successful) {answer(-1, one.reason); return;}
                }
                answer(1, "");
              } catch (const std::exception & e) {answer(-1, e.what());}
            });
        } catch (const std::exception & e) {answer(-1, e.what());}
      });
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
      if (b.indicator_present) {s.ind_u_px = 0.5 * (b.indicator_bbox[0] + b.indicator_bbox[2]);}
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
        ws.u_px = 0.5 * (w.bbox[0] + w.bbox[2]);
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
    dock_t_ = t;
    have_dock_ = true;
  }

  /// ocs_client republishes RoboCommand's commands here as JSON and acts on none of them. Two
  /// matter: the RunStart WaitForRunStart waits for, and Task 3's ReadinessConfirm that answers
  /// the docking report. A key test is enough for each and keeps a JSON library out of this node.
  /// RoboCommand's course from the OCS: the dock book drops faces placed outside it.
  void onCourse(const std_msgs::msg::String::SharedPtr m)
  {
    std::string id;
    std::vector<nav::LatLon> corners;
    const bool ok = parseCourse(m->data, id, corners);
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->course_id = id;
      ctx_->course_corners = corners;
      ++ctx_->course_seq;
    }
    if (ok) {
      RCLCPP_INFO(get_logger(), "course %s from the OCS: %zu corners; dock faces outside it "
        "are dropped", id.c_str(), corners.size());
    } else {
      RCLCPP_WARN(get_logger(), "no course from the OCS (or under 3 corners): dock faces are "
        "not checked against a course");
    }
  }

  void onOcsCommand(const std_msgs::msg::String::SharedPtr m)
  {
    int run_id = -1;
    bool run_start;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      run_start = ingestRunStart(*ctx_, nowS(), m->data, &run_id);
    }
    if (run_start) {RCLCPP_INFO(get_logger(), "RoboCommand RunStart received (run_id %d)", run_id);}
    runstart::Directive dir;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      dir = ingestRunDirective(*ctx_, m->data);
    }
    if (dir != runstart::Directive::None) {
      RCLCPP_INFO(
        get_logger(), "RunStart latch cleared by the OCS (%s)",
        dir == runstart::Directive::StandbyAuto ? "standby_auto: a new declaration" :
        "run_end: the run is over");
    }
    if (m->data.find("\"readiness_confirm\"") != std::string::npos) {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->readiness_confirmed = true;
      RCLCPP_INFO(get_logger(), "RoboCommand confirmed the docking report");
    }
    // Task 4: a new run, a run over or an All Clear forgets every moving object
    if (run_start || dir != runstart::Directive::None || t4::isAllClear(m->data)) {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!ctx_->task4.objects.empty()) {
        RCLCPP_INFO(get_logger(), "Task 4: %zu moving object(s) forgotten (%s)",
          ctx_->task4.objects.size(), run_start ? "RunStart" :
          dir != runstart::Directive::None ? "the run ended" : "All Clear");
        ctx_->task4.objects.clear();
      }
      ctx_->task4.last_seq = 0.0;
    }
    t4::Alert alert;
    if (t4::parseAlert(m->data, &alert)) {onMovingObject(alert);}
  }

  /// One MovingObjectAlert (handbook 3.3.5 Disruptive), folded in whether or not a mission runs.
  void onMovingObject(const t4::Alert & a)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    ++ctx_->task4.alerts;
    if (!t4::acceptSeq(a.seq, ctx_->task4.last_seq)) {
      RCLCPP_WARN(get_logger(), "Task 4: MovingObjectAlert seq %.0f IGNORED: a re-delivered or "
        "late alert (seq %.0f was already applied)", a.seq, ctx_->task4.last_seq);
      return;
    }
    if (!ctx_->origin_set) {
      RCLCPP_WARN(get_logger(), "Task 4: MovingObjectAlert at %.7f, %.7f IGNORED: no position "
        "fix yet to place it against", a.lat, a.lon);
      return;
    }
    const double t = nowS();
    ctx_->now_s = std::max(ctx_->now_s, t);
    const nav::Vec2 p = nav::toLocal({a.lat, a.lon}, ctx_->origin);
    const std::size_t before = ctx_->task4.objects.size();
    const int id = t4::ingest(ctx_->task4.objects, ctx_->task4.next_id, a, p, t,
        ctx_->task4.settings);
    ctx_->task4.last_alert_s = t;
    std::string domains;
    for (const std::string & d : a.domains) {domains += (domains.empty() ? "" : " ") + d;}
    const double dist = ctx_->pose_fresh ? nav::norm(p - ctx_->boat) : std::nan("");
    RCLCPP_WARN(get_logger(), "Task 4: MovingObjectAlert -> object %d (%s): %.7f, %.7f heading "
      "%.0f deg at %.2f m/s, keep-out %s m, domains [%s]%s; %.0f m from the boat%s", id,
      ctx_->task4.objects.size() > before ? "new" : "update", a.lat, a.lon, a.heading_deg,
      a.speed_mps, a.radius_m > 0.0 ? std::to_string(a.radius_m).c_str() : "none given",
      domains.empty() ? "none named: all" : domains.c_str(),
      t4::affectsUsv(a) ? "" : " (not the USV)",
      dist, ctx_->task4.settings.enabled ? "" : " [Task 4 avoidance is OFF: t4_enabled]");
    if (t4::shouldAck(ctx_->task4.objects.size() > before, a)) {
      sendReport(*ctx_, "incident_ack", t4::incidentAckJson(a.seq));
      RCLCPP_WARN(get_logger(), "Task 4: IncidentAck sent for object %d (command_seq %.0f)",
        id, a.seq);
    }
  }

  /// The operator's mission inputs, as the parameters are now. CALL WITH ctx_->mu HELD.
  void readInputs()
  {
    Context::Inputs & in = ctx_->inputs;
    const double lat = get_parameter("t1_field_lat").as_double();
    const double lon = get_parameter("t1_field_lon").as_double();
    in.field_set = latLonGiven(lat, lon);
    in.field_ll = {lat, lon};
    in.scan_dwell_s = std::clamp(get_parameter("t1_scan_dwell_s").as_double(), 1.0, 30.0);
    in.pause_every_m = std::clamp(get_parameter("t1_pause_every_m").as_double(), 0.0, 50.0);
    const double alat = get_parameter("t1_approach_lat").as_double();
    const double alon = get_parameter("t1_approach_lon").as_double();
    in.approach_set = latLonGiven(alat, alon);
    in.approach_ll = in.approach_set ? nav::LatLon{alat, alon} : nav::LatLon{0.0, 0.0};
    in.unknown_range_m = std::clamp(get_parameter("t1_unknown_range_m").as_double(), 0.0, 30.0);
    in.backoff_m = std::clamp(get_parameter("t3_backoff_m").as_double(), 1.0, 8.0);
    const double vlat = get_parameter("t3_via_lat").as_double();
    const double vlon = get_parameter("t3_via_lon").as_double();
    in.via_set = latLonGiven(vlat, vlon);
    in.via_ll = in.via_set ? nav::LatLon{vlat, vlon} : nav::LatLon{0.0, 0.0};
  }

  /// Every Run-panel setting the mission reads once per run, as the parameters are now: the dock
  /// heading, which way to slide first (left | right, else the tree's own) and readInputs().
  /// Read at goal accept AND again when the RunStart hold opens, so a Set made while the run
  /// waited is the one used. CALL WITH ctx_->mu HELD.
  void readRunPanel()
  {
    ctx_->dock_heading = get_parameter("dock_heading_deg").as_double();
    ctx_->search_first = get_parameter("search_first").as_string();
    readInputs();
    if (!ctx_->search_first.empty() && ctx_->search_first != "left" &&
      ctx_->search_first != "right")
    {
      RCLCPP_WARN(get_logger(), "search_first '%s' is not left or right: the tree's own",
        ctx_->search_first.c_str());
      ctx_->search_first.clear();
    }
  }

  /// The Run-panel settings in the GOAL log line's words. CALL WITH ctx_->mu HELD.
  std::string runPanelText() const
  {
    return std::string("dock_heading=") + (ctx_->dock_heading >= 0.0 ?
      std::to_string(static_cast<int>(std::lround(ctx_->dock_heading))) : std::string("unset")) +
      " search_first=" + (ctx_->search_first.empty() ? std::string("tree") : ctx_->search_first) +
      " " + inputsText();
  }

  /// The operator's numbers the XML uses directly ({t3_backoff_m} ...), from ctx_->inputs.
  /// Takes ctx_->mu.
  void putInputsOnBlackboard(const BT::Blackboard::Ptr & bb)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const Context::Inputs & in = ctx_->inputs;
    bb->set("t3_backoff_m", in.backoff_m);
    bb->set("t1_scan_dwell_s", in.scan_dwell_s);
    bb->set("t1_pause_every_m", in.pause_every_m);
    bb->set("t1_unknown_range_m", in.unknown_range_m);
    bb->set("t1_approach_lat", in.approach_ll.lat);    // 0, 0 when unset: FixGiven says no
    bb->set("t1_approach_lon", in.approach_ll.lon);
    bb->set("t3_via_lat", in.via_ll.lat);              // 0, 0 when unset: FixGiven says no
    bb->set("t3_via_lon", in.via_ll.lon);
  }

  /// For the GOAL log line. CALL WITH ctx_->mu HELD.
  std::string inputsText() const
  {
    const Context::Inputs & in = ctx_->inputs;
    char b[360];
    char f[40] = "unset", v[40] = "none", ap[40] = "none";
    if (in.field_set) {std::snprintf(f, sizeof(f), "%.7f,%.7f", in.field_ll.lat, in.field_ll.lon);}
    if (in.via_set) {std::snprintf(v, sizeof(v), "%.7f,%.7f", in.via_ll.lat, in.via_ll.lon);}
    if (in.approach_set) {
      std::snprintf(ap, sizeof(ap), "%.7f,%.7f", in.approach_ll.lat, in.approach_ll.lon);
    }
    std::snprintf(b, sizeof(b),
      "t1_field=%s t1_approach=%s scan_dwell=%.0fs pause_every=%.1fm unknown_range=%.1fm "
      "t3_backoff=%.1fm t3_via=%s",
      f, ap, in.scan_dwell_s, in.pause_every_m, in.unknown_range_m, in.backoff_m, v);
    return b;
  }

  /// Task 4's settings, re-read every second (a change applies at once, mid-run too).
  void readTask4Settings()
  {
    t4::Settings s;
    s.enabled = get_parameter("t4_enabled").as_bool();
    s.margin_m = std::clamp(get_parameter("t4_margin_m").as_double(), 0.0, 20.0);
    s.fallback_radius_m = std::clamp(get_parameter("t4_radius_m").as_double(), 1.0, 50.0);
    s.horizon_s = std::clamp(get_parameter("t4_horizon_s").as_double(), 5.0, 180.0);
    s.sweep_s = std::min(s.sweep_s, s.horizon_s);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (s.enabled != ctx_->task4.settings.enabled) {
      RCLCPP_WARN(get_logger(), "Task 4 avoidance %s", s.enabled ? "ON" : "OFF (t4_enabled false)");
    }
    // a radius that came with no alert follows the fallback
    for (t4::Invader & v : ctx_->task4.objects) {
      if (!v.radius_given) {v.radius_m = s.fallback_radius_m;}
    }
    ctx_->task4.settings = s;
  }

  /// Task 4 for /crsd/bt_status: the settings, every object (where it is predicted NOW, its
  /// projected track over the horizon, how close it comes to the boat), and what the guard is
  /// doing. Ages, not times: a viewer never needs our clock. Takes ctx_->mu.
  std::string task4Json()
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const Context & c = *ctx_;
    const Context::Task4 & k = c.task4;
    const double t = nowS();
    const t4::Settings & s = k.settings;
    std::string objs;
    for (const t4::Invader & v : k.objects) {
      const nav::Vec2 p = t4::at(v, t);
      const bool boat = c.pose_fresh;
      const t4::Cpa cpa = boat ? t4::cpaFixed(v, c.boat, t, s.horizon_s) : t4::Cpa{};
      const double ko = t4::keepout(v, s);
      std::string track;
      for (int i = 0; i <= 4; ++i) {
        track += std::string(i ? "," : "") + jsonLatLon(c, true, t4::at(v, t + s.horizon_s * i / 4.0));
      }
      objs += std::string(objs.empty() ? "" : ",") + "{\"id\":" + std::to_string(v.id) +
        ",\"now\":" + jsonLatLon(c, true, p) +
        ",\"reported\":[" + jsonNum(v.ll0.lat, 7) + "," + jsonNum(v.ll0.lon, 7) + "]" +
        ",\"age_s\":" + jsonNum(t - v.t0, 1) +
        ",\"heading_deg\":" + jsonNum(v.heading_deg, 0) +
        ",\"speed_mps\":" + jsonNum(v.speed_mps, 2) +
        ",\"radius_m\":" + jsonNum(v.radius_m, 1) +
        ",\"radius_given\":" + (v.radius_given ? "true" : "false") +
        ",\"keepout_m\":" + jsonNum(ko, 1) +
        ",\"affects_usv\":" + (v.affects_usv ? "true" : "false") +
        ",\"domains\":" + jsonQuote(v.domains) +
        ",\"reports\":" + std::to_string(v.reports) +
        ",\"dist_m\":" + (boat ? jsonNum(nav::norm(p - c.boat), 1) : "null") +
        ",\"cpa_m\":" + (boat ? jsonNum(cpa.d, 1) : "null") +
        ",\"cpa_in_s\":" + (boat ? jsonNum(cpa.dt, 0) : "null") +
        ",\"inside\":" + (boat && t4::counts(v, s) && cpa.d < ko ? "true" : "false") +
        ",\"track\":[" + track + "]}";
    }
    const bool live = k.state_s >= 0.0 && c.now_s - k.state_s < 2.0;
    return std::string("{\"enabled\":") + (s.enabled ? "true" : "false") +
      ",\"margin_m\":" + jsonNum(s.margin_m, 1) + ",\"radius_m\":" + jsonNum(s.fallback_radius_m, 1) +
      ",\"horizon_s\":" + jsonNum(s.horizon_s, 0) + ",\"alerts\":" + std::to_string(k.alerts) +
      ",\"last_alert_age_s\":" + (k.last_alert_s >= 0.0 ? jsonNum(t - k.last_alert_s, 0) : "null") +
      ",\"guard\":" + jsonQuote(live ? k.state : "") + ",\"why\":" + jsonQuote(live ? k.why : "") +
      ",\"target\":" + jsonLatLon(c, live && k.have_target, k.target) +
      ",\"evasions\":" + std::to_string(k.evasions) + ",\"objects\":[" + objs + "]}";
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

  /// /crsd/autonomy_active: true every tick while a mission runs OR is held at the RunStart gate.
  void publishAutonomy(bool on)
  {
    std_msgs::msg::Bool m;
    m.data = on;
    autonomy_pub_->publish(m);
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
    // Task 4 runs at all times: its settings follow the parameters, objects age out, and between
    // missions (no tick to move the clock) the swept keep-outs still advance.
    readTask4Settings();
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!busy_) {ctx_->now_s = std::max(ctx_->now_s, nowS());}
      const int gone = t4::expire(ctx_->task4.objects, nowS(), ctx_->task4.settings);
      if (gone > 0) {
        RCLCPP_INFO(get_logger(), "Task 4: %d object(s) not re-reported for %.0f s, dropped", gone,
          ctx_->task4.settings.expire_s);
      }
    }
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

    // THE PLAN AGES LIKE THE POSE. plan_raw_ is latched, so a dead radio leaves it standing:
    // the age is what PassagePlanFresh reads. An empty plan (the ground station's Clear) is
    // no plan, never a fresh one.
    ctx_->plan_age_s = (plan_raw_ && !plan_raw_->buoys.empty()) ?
      (t - plan_t_).seconds() : Context::kNoPlanAgeS;

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
    rclcpp::Time started = now();    // restarted when the RunStart gate opens
    auto result = std::make_shared<RunMission::Result>();
    {
      std::lock_guard<std::mutex> lk(status_mu_);
      running_name_ = goal->name;
    }

    consumed_.clear();
    forgetColours(goal->name);
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
      // The gate is per goal: a release (RunStart or bench) is never carried to the next.
      ctx_->run_released = false;
      bench_release_ = false;
      // Per-mission state starts empty: a second attempt must not inherit the first's.
      ctx_->cleared_gates.clear();
      ctx_->global_passage = Context::GlobalPassage{};
      resetTask3(*ctx_);
      resetFire(*ctx_);
      goal_mode_ = ctx_->mode;
      ctx_->handed_mode.clear();
      ctx_->looked_unnamed.clear();
      ctx_->guess_exit = false;        // set from the mission's root once it is read, below
      readRunPanel();
      ctx_->task4.state.clear();
      ctx_->task4.why.clear();
      ctx_->task4.state_s = -1.0;
      ctx_->task4.have_target = false;
      ctx_->task4.evasions = 0;
      goal_inputs_ = "mode=" + (ctx_->mode.empty() ? std::string("?") : ctx_->mode) +
        (ctx_->drop_tripped ? " DROP-LATCHED" : "") + " " + runPanelText();
      // "Home" is where THIS attempt started, not the autopilot's HOME.
      ctx_->home = ctx_->boat;
      ctx_->have_home = ctx_->pose_fresh;
    }
    leg_pacer_ = LegStatusPacer{};
    resetLeg(0.0);

    uint8_t outcome = RunMission::Result::OUTCOME_FAULT;
    std::string detail;
    try {
      // Read again for every goal: an upload since the last one is this one's mission.
      const MissionFile file = resolveMission(dirs_, goal->name);
      BT::BehaviorTreeFactory factory;
      registerCrusaderNodes(factory);
      auto bb = BT::Blackboard::create();
      bb->set("ctx", ctx_);
      // The operator's numbers the XML uses directly ({t3_backoff_m} ...), as they were at Start.
      putInputsOnBlackboard(bb);
      {
        std::lock_guard<std::mutex> lk(ctx_->mu);
        ctx_->guess_exit = rootFlag(file.path, "t1_guess_exit");
      }
      BT::Tree tree = loadMission(factory, dirs_, file, bb);
      TreeView view(tree);
      FailureTrace failed_at(tree);

      const std::string limit = timeout_s > 0.0 ?
        std::to_string(static_cast<int>(timeout_s)) + "s" : std::string("none");
      RCLCPP_INFO(
        get_logger(), "GOAL %s timeout=%s approach=(%.7f, %.7f) %s", goal->name.c_str(),
        limit.c_str(), goal->approach_latitude, goal->approach_longitude, goal_inputs_.c_str());

      rclcpp::Rate rate(tick_hz_);
      auto fb = std::make_shared<RunMission::Feedback>();
      double next_status = 0.0;

      if (!awaitRunStart(gh, fb, goal->name, started, &outcome, &detail)) {
        // Cancelled, no HEARTBEAT, or the node is stopping: nothing was ticked, nothing to halt.
        throw GateClosed{};
      }
      // A Set the operator made while the run waited for RunStart is the one to use.
      {
        std::string before, after;
        {
          std::lock_guard<std::mutex> lk(ctx_->mu);
          before = runPanelText();
          readRunPanel();
          after = runPanelText();
        }
        putInputsOnBlackboard(bb);
        if (after != before) {RCLCPP_INFO(get_logger(), "inputs at RunStart: %s", after.c_str());}
      }

      while (rclcpp::ok() && !stop_) {
        // Cancel first: a cancel that waits for anything does not work.
        if (gh->is_canceling()) {
          tree.haltTree();
          outcome = RunMission::Result::OUTCOME_CANCELLED;
          detail = "cancelled by the operator";
          break;
        }
        const double elapsed = (now() - started).seconds();
        if (timeout_s > 0.0 && elapsed >= timeout_s) {   // 0 = never give up
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
        failed_at.beginTick();
        const BT::NodeStatus st = tree.tickOnce();
        stopIfSilent(false);
        publishLegStatus(elapsed);
        if (verbose_tree_) {
          const std::string frame = view.renderIfChanged();
          if (!frame.empty()) {RCLCPP_INFO(get_logger(), "tree @ %5.1fs\n%s", elapsed, frame.c_str());}
        }
        // The mast light goes GREEN because a mission is ticking, not because a node is alive.
        publishAutonomy(true);
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
            // A switch the tree made itself (RequestMode) is the mode it expects now.
            const std::string want = ctx_->handed_mode.empty() ? goal_mode_ : ctx_->handed_mode;
            if (ctx_->mode != want || (!ctx_->autonomous && want != "MANUAL")) {
              outcome = RunMission::Result::OUTCOME_NOT_AUTONOMOUS;
              detail = ctx_->mode.empty() ? std::string("no flight mode yet (no HEARTBEAT)") :
                "flight mode " + (ctx_->mode != want ?
                want + " -> " + ctx_->mode : ctx_->mode + " is not autonomous") +
                ": the pilot took control; not fighting for it";
            } else {
              outcome = RunMission::Result::OUTCOME_FAILURE;
              detail = "tree " + goal->name + " returned FAILURE" + failureWhy(failed_at);
            }
          }
          break;
        }
        rate.sleep();
      }
    } catch (const GateClosed &) {
      // outcome and detail were set by awaitRunStart().
    } catch (const MissionError & e) {
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
    publishAutonomy(false);
    busy_ = false;
    publishIdle();
  }

  /// Thrown out of execute()'s try block when the RunStart gate ended the goal.
  struct GateClosed {};

  /// The RunStart gate: holds station and does NOT tick the tree until RoboCommand's RunStart
  /// (relayed by the OCS) has reached the boat, or the bench release was pressed. True when it
  /// opened; false with *outcome and *detail set when the goal ended while held. Every goal
  /// goes through it, so no tree can skip it. The mission timeout does not run here.
  ///
  /// It publishes /crsd/autonomy_active true every tick: that is what makes ocs_client report
  /// STATE_AUTO, and RoboCommand sends RunStart only to a vehicle in STATE_AUTO.
  bool awaitRunStart(
    const std::shared_ptr<GoalHandle> & gh, const std::shared_ptr<RunMission::Feedback> & fb,
    const std::string & name, rclcpp::Time & started, uint8_t * outcome, std::string * detail)
  {
    gate_waiting_ = true;
    const rclcpp::Time began = now();
    rclcpp::Rate rate(tick_hz_);
    double next_status = 0.0;
    bool held = false;
    bool opened = false;
    RCLCPP_INFO(
      get_logger(), "holding for RoboCommand's RunStart (max age %.0f s)", run_start_max_age_s_);
    while (rclcpp::ok() && !stop_) {
      if (gh->is_canceling()) {
        *outcome = RunMission::Result::OUTCOME_CANCELLED;
        *detail = "cancelled by the operator while waiting for RunStart";
        break;
      }
      const double waiting = (now() - began).seconds();
      if (!have_status_ && waiting > mode_grace_s_) {
        *outcome = RunMission::Result::OUTCOME_FAULT;
        *detail = "no HEARTBEAT: cannot tell whether we are allowed to drive";
        break;
      }
      refreshFreshness();
      // Once, retried only until the first send: the wait can begin before the first pose.
      if (!held) {held = sendHold(*ctx_, get_logger(), "RunStart gate hold at");}
      publishAutonomy(true);

      runstart::Latch latch;
      double now_s;
      {
        std::lock_guard<std::mutex> lk(ctx_->mu);
        latch = ctx_->run_start;
        now_s = ctx_->now_s;
      }
      const bool bench = bench_release_;
      if (bench || latch.counts(now_s, run_start_max_age_s_)) {
        if (bench) {
          RCLCPP_WARN(get_logger(), "released WITHOUT RoboCommand's RunStart (bench)");
        } else {
          RCLCPP_INFO(
            get_logger(), "RunStart accepted (run_id %d, received %.1f s ago): starting %s",
            latch.id, std::max(0.0, now_s - latch.t_s), name.c_str());
        }
        opened = true;
        break;
      }
      if (waiting >= next_status) {
        next_status = waiting + kRunningStatusPeriodS;
        publishWaiting(gh, fb, name, waiting);
      }
      rate.sleep();
    }
    gate_waiting_ = false;
    if (!opened) {return false;}
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->run_released = true;
      // "Home" is where the attempt STARTS, which is now, not where the goal was accepted.
      ctx_->home = ctx_->boat;
      ctx_->have_home = ctx_->pose_fresh;
      // So is the mode NOT_AUTONOMOUS compares against: the pilot may switch while held.
      goal_mode_ = ctx_->mode;
    }
    started = now();
    return true;
  }

  /// Feedback and /crsd/bt_status while the RunStart gate holds the goal.
  void publishWaiting(
    const std::shared_ptr<GoalHandle> & gh, const std::shared_ptr<RunMission::Feedback> & fb,
    const std::string & name, double waiting)
  {
    const std::string warning = "waiting for RoboCommand's RunStart via the OCS";
    fb->elapsed_s = waiting;
    fb->running_node = "RunStart gate";
    fb->current_task = kTaskNone;
    fb->warning = warning;
    gh->publish_feedback(fb);

    std_msgs::msg::String m;
    m.data = "{\"state\":\"WAITING_RUN_START\",\"name\":" + jsonQuote(name) +
      ",\"waiting_s\":" + jsonNum(waiting, 1) + ",\"running_node\":\"\",\"current_task\":" +
      jsonQuote(kTaskNone) + ",\"warning\":" + jsonQuote(warning) + ",\"task1\":" +
      task1Json() + ",\"task4\":" + task4Json() + "}";
    bt_status_pub_->publish(m);
  }

  /// " at NotDropped in scan_fire > manual: <why, in the pilot's words>" for a FAILURE
  /// result, which the GCS shows. Under ctx_->mu. The guards fail silently (a condition
  /// has nothing to log), so they are the ones that need saying here.
  std::string failureWhy(const FailureTrace & trace) const
  {
    const std::string at = trace.text();
    if (at.empty()) {return " (no leaf reported a failure)";}
    const std::string & type = trace.failed().type;
    const std::string mode = ctx_->mode.empty() ? std::string("unknown") : ctx_->mode;
    std::string why;
    if (type == "NotDropped") {
      why = "the autonomy-drop latch is set (SC down in MANUAL, or RC lost): "
        "SC up, then Start";
    } else if (type == "IsAutonomous") {
      why = "flight mode " + mode + " is not autonomous";
      if (mode == "MANUAL" && ctx_->drop_tripped) {
        why = "the autonomy-drop latch is set (SC down in MANUAL, or RC lost): SC up, then Start";
      } else if (mode == "MANUAL") {
        why += "; this mission's guard is a plain IsAutonomous, which ends it in MANUAL: open it "
          "in the Mission tab and save it again (a mission that drives in MANUAL is saved with "
          "IsAutonomous manual=\"true\")";
      }
    } else if (type == "ModeIs") {
      why = "the autopilot is in " + mode;
    } else if (type == "SetCameraModel") {
      why = ctx_->camera_model_why + "; the Sensors tab shows camera_node's model state";
    } else {
      why = "its own warning, if it gave one, is in the bt_runner_node log";
    }
    return " at " + at + ": " + why;
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
      ",\"nodes\":" + view.nodesJson() + ",\"task1\":" + task1Json() + ",\"task3\":" +
      task3Json() + ",\"task4\":" + task4Json() + "}";
    bt_status_pub_->publish(m);
  }

  /// /crsd/bt_status between goals: IDLE and how the last one ended.
  void publishIdle()
  {
    std_msgs::msg::String m;
    const std::string task1 = task1Json();     // takes ctx_->mu: never inside status_mu_
    const std::string task4 = task4Json();
    {
      std::lock_guard<std::mutex> lk(status_mu_);
      m.data = "{\"state\":\"IDLE\",\"last\":" + (last_.empty() ? std::string("null") : last_) +
        ",\"task1\":" + task1 + ",\"task4\":" + task4 + "}";
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
  std::string action_name_;
  MissionDirs dirs_;
  double tick_hz_ = 10.0, default_timeout_s_ = 600.0, mode_grace_s_ = 3.0;
  double run_start_max_age_s_ = 3600.0;
  /// The RunStart gate: held (worker_ writes, the service reads) and the bench release (the
  /// service writes, worker_ reads and clears at every goal start).
  std::atomic<bool> gate_waiting_{false}, bench_release_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_srv_;
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
  rclcpp::Time plan_t_{0, 0, RCL_ROS_TIME};      ///< when plan_raw_ was received
  /// RAW inputs. ctx_->buoys is the FUSION of these two; see refuseLocked().
  std::vector<nav::Buoy> tracked_;
  /// Core tier: the tracks chosen as ENTRY and EXIT (-1 = none yet), kept while still candidates.
  int entry_track_id_ = -1, exit_track_id_ = -1;
  /// Where the context's ENTRY/EXIT come from now, under ctx_->mu.
  AnchorSource entry_src_ = AnchorSource::None, exit_src_ = AnchorSource::None;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr forget_colours_pub_;
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ocs_sub_, course_sub_;
  std::string camera_node_;
  rclcpp::AsyncParametersClient::SharedPtr camera_params_;
  std::atomic<int> camera_ask_{0};      // SetCameraModel asks; a reply to an older one is dropped

  // The fixed-nozzle shot
  std::chrono::steady_clock::time_point t0_{};
  double att_t_ = 0.0, pump_t_ = 0.0;   // nowS() at receipt; guarded by ctx_->mu
  bool have_att_ = false, have_pump_ = false;
  bool hs_active_ = false;              // motion commanded last tick (tick thread only)
  bool sticks_active_ = false;          // the sticks commanded last tick (tick thread only)
  std::string goal_mode_;               // the mode when this goal started (under ctx_->mu)
  std::string goal_inputs_;             // mode, drop latch and GCS settings at the start, for GOAL
  std::vector<int64_t> stick_ch_, stick_neutral_;
  std::vector<bool> stick_reverse_;
  rclcpp::Publisher<crusader_msgs::msg::RcChannels>::SharedPtr rc_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_mode_pub_;
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
