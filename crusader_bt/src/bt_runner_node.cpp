// bt_runner_node — Crusader's bt_navigator: one coarse action outside, a tree
// of primitives inside.
//
//     ros2 action send_goal /crsd/safe_passage crusader_msgs/action/SafePassage "{tier: 0, timeout_s: 600}"
//
//   in:  /crsd/fcu_status    (FcuStatus)          mode + armed — THE autonomy latch
//        /crsd/pose          (LatLonHead)         position and GPS-yaw heading
//        /crsd/world_targets (TrackedTargetArray) the buoy field
//        dock/observations   (DockObservation)    Task 3: the bays, every frame
//        /crsd/ocs_command   (String, JSON)       Task 3: RoboCommand's readiness
//   out: /crsd/guided_setpoint    (GuidedSetpoint) ONLY when publish_setpoints
//        /crsd/current_task       (String, latched)
//        /crsd/autonomy_active    (Bool)   turns the mast light GREEN
//        /crsd/avoidance_enable   (Bool, latched)
//        /crsd/safe_passage_report (String, JSON)
//        /crsd/docking_report, /crsd/firefighting_report,
//        /crsd/resource_delivery_request (String, JSON)  Task 3, for the OCS
//        /crsd/uav_resource_request (String, JSON)       Task 3, for the radio
//        /crsd/water_cannon       (String, JSON)          Task 3, the pump
//
// NOTE: the Task 3 plumbing here (onDock, onOcsCommand, the five publishers)
// has NOT been built against real ROS. It was written on a laptop with no ROS
// and checked there only with g++ -fsyntax-only against a MOCK of the rclcpp
// API this file uses and structs generated rosidl-style from crusader_msgs
// (the unmodified task1-disruptive version of this file passed the same check,
// which is what the mock was calibrated against). Everything it feeds is
// compiled and exercised off-ROS by crusader_bt/offros/, whose frameFromJson()
// is this file's onDock() field for field. First colcon build: read the errors
// here before anywhere else.
//
// WHY THE COARSE ACTION SURVIVES the move to a behaviour tree. It is exactly
// Nav2's shape: bt_navigator exposes ONE NavigateToPose action and runs a tree
// of ComputePathToPose / FollowPath / Spin inside it. The action is what lets
// the ground station, a bench test or a higher-level tree say "run Task 1"
// without knowing that trees exist, and it is where the three cross-cutting
// guarantees live:
//
//   * the MISSION TIMEOUT — kept here, not in the XML, so a tree that somehow
//     never terminates still cannot strand the boat;
//   * CANCEL — halts the tree, which fires onHalted() on every RUNNING leaf;
//   * the terminal state — a cancelled goal that is succeed()-ed reports success
//     to the caller, and the caller carries on as though the mission worked.
//
// THREADING — and where the "does the poll loop block the callbacks?" worry
// actually lands, because it lands in a different place than you would expect.
//
// IT IS NOT IN THE LEAVES. A leaf never loops and never blocks. CircleBuoy's
// onStart() publishes one waypoint and returns RUNNING; its onRunning() does one
// distance comparison and returns. The polling loop that a per-primitive action
// server would each need is HOISTED OUT of the leaves into the single tick loop
// below — one loop for the whole mission instead of one per primitive, which is
// most of the reason the leaves are 40 lines each.
//
// IT IS HERE. The tick loop does sleep, so it runs on its OWN thread (worker_)
// rather than on an executor thread, and cannot starve the subscriptions that
// feed it or the cancel that has to interrupt it. The action server also sits in
// a ReentrantCallbackGroup under a MultiThreadedExecutor so the cancel callback
// can run while a goal is executing. Cancel is polled at the top of every tick,
// so its worst-case latency is one tick — 100 ms at 10 Hz — not one leaf.
//
// worker_ is OWNED AND JOINED, never detached: a detached thread outlives the
// node and keeps publishing through destroyed members, and the moment that
// happens is Ctrl+C or `systemctl stop` mid-mission.
//
// Subscriptions write Context under its mutex; the tick loop reads under the
// same one.
#include <chrono>
#include <memory>
#include <mutex>
#include <atomic>
#include <set>
#include <sstream>
#include <thread>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "crusader_bt/tree_view.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "crusader_msgs/action/safe_passage.hpp"
#include "crusader_msgs/msg/dock_observation.hpp"
#include "crusader_msgs/msg/fcu_status.hpp"
#include "crusader_msgs/msg/gate_pair.hpp"
#include "crusader_msgs/msg/guided_setpoint.hpp"
#include "crusader_msgs/msg/lat_lon_head.hpp"
#include "crusader_msgs/msg/passage_plan.hpp"
#include "crusader_msgs/msg/tracked_target_array.hpp"

#include "crusader_bt/context.hpp"
#include "crusader_bt/dock_math.hpp"
#include "crusader_bt/nav_math.hpp"

namespace crusader_bt
{

using SafePassage = crusader_msgs::action::SafePassage;
using GoalHandle = rclcpp_action::ServerGoalHandle<SafePassage>;
using namespace std::chrono_literals;

/// The idle task token. TASK_UNKNOWN is never sent — RoboCommand's schema says
/// it means "unset or unparsed, NOT a stand-down signal".
static constexpr const char * kTaskNone = "TASK_NONE";

/// Maps a tracked target's label to a beacon state.
///
/// Deliberately conservative: anything it does not recognise becomes Unknown,
/// which carries no side constraint, rather than being guessed into a colour.
/// A mis-called colour steers the boat to the wrong side of a buoy; an Unknown
/// one is simply left to the autopilot's avoidance.
static nav::Beacon beaconFromLabel(const std::string & raw)
{
  std::string s;
  for (char c : raw) {s += static_cast<char>(std::tolower(c));}
  const bool flashing = s.find("flash") != std::string::npos;
  if (s.find("red") != std::string::npos) {return nav::Beacon::FlashingRed;}
  if (s.find("green") != std::string::npos) {return nav::Beacon::FlashingGreen;}
  if (s.find("blue") != std::string::npos) {
    // ENTRY is FLASHING blue, EXIT is STEADY blue, and they are the only two
    // buoys where the distinction matters. Absent a flash label we cannot tell
    // them apart, so we say so rather than pick one.
    if (flashing) {return nav::Beacon::FlashingBlue;}
    if (s.find("steady") != std::string::npos || s.find("solid") != std::string::npos) {
      return nav::Beacon::SteadyBlue;
    }
    return nav::Beacon::Unknown;
  }
  if (s.find("off") != std::string::npos || s.find("black") != std::string::npos) {
    return nav::Beacon::Off;
  }
  return nav::Beacon::Unknown;
}

class BtRunner : public rclcpp::Node
{
public:
  BtRunner()
  // The node name MUST equal its section in crusader_params.yaml, or ROS hands
  // it a different node's parameters and it comes up misconfigured while
  // looking perfectly healthy. Named "safe_passage_server" once, which is the
  // PYTHON server's section, so it advertised that node's
  // action_name (/crsd/safe_passage_dwell) and nobody could find it.
  : rclcpp::Node("bt_runner_node")
  {
    tree_file_ = declare_parameter<std::string>("tree_file", "");
    action_name_ = declare_parameter<std::string>("action_name", "/crsd/safe_passage");
    tick_hz_ = declare_parameter<double>("tick_hz", 10.0);
    default_timeout_s_ = declare_parameter<double>("default_timeout_s", 600.0);
    mode_grace_s_ = declare_parameter<double>("mode_grace_s", 3.0);
    stream_timeout_s_ = declare_parameter<double>("stream_timeout_s", 1.0);
    // The UAV plan arrives at ~0.2 Hz, so its timeout is a different order of
    // magnitude from a 20 Hz pose stream and cannot share stream_timeout_s.
    plan_timeout_s_ = declare_parameter<double>("plan_timeout_s", 15.0);
    assoc_radius_m_ = declare_parameter<double>("assoc_radius_m", 5.0);
    verbose_tree_ = declare_parameter<bool>("verbose_tree", true);
    task_token_ = declare_parameter<std::string>("task_token", "TASK_SAFE_PASSAGE");
    auto modes = declare_parameter<std::vector<std::string>>(
      "autonomous_modes", std::vector<std::string>{"GUIDED", "AUTO", "LOITER", "RTL"});
    for (auto & m : modes) {
      for (auto & c : m) {c = static_cast<char>(std::toupper(c));}
      auto_modes_.insert(m);
    }
    if (tree_file_.empty()) {
      throw std::runtime_error(
        "tree_file is empty. Point it at behavior_trees/task1_safe_passage.xml "
        "in this package's share dir; there is no default, because a runner "
        "that silently ticks the wrong tree is worse than one that will not "
        "start.");
    }

    ctx_ = std::make_shared<Context>();
    ctx_->node = this;
    ctx_->publish_setpoints = declare_parameter<bool>("publish_setpoints", false);

    // Task 3. THE CAMERA EXTRINSIC MUST EQUAL target_tracker's cam_x / cam_y /
    // cam_yaw_deg: two nodes placing one camera in two places put the bays and
    // the buoys in two different worlds, and neither looks wrong on its own.
    ctx_->cam_mount.x = declare_parameter<double>("cam_x", 0.37);
    ctx_->cam_mount.y = declare_parameter<double>("cam_y", 0.0);
    ctx_->cam_mount.yaw_deg = declare_parameter<double>("cam_yaw_deg", 0.0);
    dock_topic_ = declare_parameter<std::string>("dock_topic", "dock/observations");

    // Latched: a subscriber that starts mid-mission must learn the current
    // value rather than sit on a default. avoidance_enable especially —
    // proximity_bridge coming up late and defaulting to off would silently
    // disarm avoidance.
    auto latched = rclcpp::QoS(1).reliable().transient_local();
    gate_reached_pub_ = create_publisher<std_msgs::msg::UInt8>("/crsd/gate_reached", 10);
    setpoint_pub_ = create_publisher<crusader_msgs::msg::GuidedSetpoint>(
      "/crsd/guided_setpoint", 10);
    task_pub_ = create_publisher<std_msgs::msg::String>("/crsd/current_task", latched);
    avoid_pub_ = create_publisher<std_msgs::msg::Bool>("/crsd/avoidance_enable", latched);
    autonomy_pub_ = create_publisher<std_msgs::msg::Bool>("/crsd/autonomy_active", 10);
    report_pub_ = create_publisher<std_msgs::msg::String>("/crsd/safe_passage_report", 10);
    // The live tree picture, for anything that wants to draw it — a terminal,
    // the ground station, a recording. Plain text, no escapes.
    bt_status_pub_ = create_publisher<std_msgs::msg::String>("/crsd/bt_status", 10);
    // Task 3. The three reports are JSON for the OCS to relay, like
    // safe_passage_report; the field names are rx_reports.proto's.
    docking_pub_ = create_publisher<std_msgs::msg::String>("/crsd/docking_report", 10);
    firefighting_pub_ = create_publisher<std_msgs::msg::String>("/crsd/firefighting_report", 10);
    request_pub_ = create_publisher<std_msgs::msg::String>(
      "/crsd/resource_delivery_request", 10);
    uav_request_pub_ = create_publisher<std_msgs::msg::String>("/crsd/uav_resource_request", 10);
    cannon_pub_ = create_publisher<std_msgs::msg::String>("/crsd/water_cannon", 10);

    status_sub_ = create_subscription<crusader_msgs::msg::FcuStatus>(
      "/crsd/fcu_status", 10,
      [this](crusader_msgs::msg::FcuStatus::SharedPtr m) {onStatus(m);});
    pose_sub_ = create_subscription<crusader_msgs::msg::LatLonHead>(
      "/crsd/pose", 10,
      [this](crusader_msgs::msg::LatLonHead::SharedPtr m) {onPose(m);});
    plan_sub_ = create_subscription<crusader_msgs::msg::PassagePlan>(
      "/crsd/passage_plan", 10, [this](crusader_msgs::msg::PassagePlan::SharedPtr m) {onPlan(m);});
    gate_sub_ = create_subscription<crusader_msgs::msg::GatePair>(
      "/crsd/next_gate", 10, [this](crusader_msgs::msg::GatePair::SharedPtr m) {onGate(m);});
    targets_sub_ = create_subscription<crusader_msgs::msg::TrackedTargetArray>(
      "/crsd/world_targets", 10,
      [this](crusader_msgs::msg::TrackedTargetArray::SharedPtr m) {onTargets(m);});
    dock_sub_ = create_subscription<crusader_msgs::msg::DockObservation>(
      dock_topic_, 10,
      [this](crusader_msgs::msg::DockObservation::SharedPtr m) {onDock(m);});
    ocs_sub_ = create_subscription<std_msgs::msg::String>(
      "/crsd/ocs_command", 10,
      [this](std_msgs::msg::String::SharedPtr m) {onOcsCommand(m);});

    wireContext();
    publishTask(kTaskNone);
    publishAvoidance(true);

    group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    server_ = rclcpp_action::create_server<SafePassage>(
      this, action_name_,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const SafePassage::Goal>) {
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
        // The mission runs on its OWN thread, not an executor thread, so its
        // tick loop cannot starve the subscriptions that feed it or the cancel
        // callback that has to interrupt it.
        //
        // Held and JOINED, never detached. A detached thread keeps touching
        // `this`, the publishers and the context after the node is destroyed —
        // and the moment that happens is Ctrl+C or `systemctl stop` DURING a
        // mission, which is exactly when someone is already having a bad day.
        // The previous goal is always finished by the time a new one is
        // accepted (busy_ gates that), so this join is instant.
        if (worker_.joinable()) {worker_.join();}
        worker_ = std::thread([this, gh] {execute(gh);});
      },
      rcl_action_server_get_default_options(), group_);

    RCLCPP_INFO(
      get_logger(), "bt_runner ready on %s: tree %s, %.0f Hz, setpoints %s",
      action_name_.c_str(), tree_file_.c_str(), tick_hz_,
      ctx_->publish_setpoints ? "ENABLED" : "DISABLED (read-only)");
    if (!ctx_->publish_setpoints) {
      RCLCPP_WARN(
        get_logger(),
        "publish_setpoints is FALSE — this node CANNOT move the boat. Legs "
        "still poll for arrival, so a human can drive the mission and the tree "
        "will follow. That is the read-only posture, not a fault.");
    }
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
    status_t_ = now();
    have_status_ = true;
  }

  void onPose(const crusader_msgs::msg::LatLonHead::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->origin_set) {
      // Pinned ONCE, at the first fix. Re-deriving it from the current position
      // each tick would make every stored buoy drift as the boat moves.
      ctx_->origin = {m->latitude, m->longitude};
      ctx_->origin_set = true;
      RCLCPP_INFO(
        get_logger(), "local frame origin pinned at %.7f, %.7f",
        m->latitude, m->longitude);
      // A plan can arrive BEFORE the first fix -- rxl_link_node is up long
      // before telemetry_bridge has a position -- and refuseLocked() bails out
      // when there is no origin to express it in. Without this the plan sits
      // unused until the NEXT one arrives, which at 0.2 Hz is up to 5 s, and a
      // goal sent in that window fails with "entry buoy not available" while
      // the guard band happily reports the plan as fresh.
      refuseLocked();
    }
    ctx_->boat = nav::toLocal({m->latitude, m->longitude}, ctx_->origin);
    ctx_->heading_deg = m->heading;      // NaN when GPS yaw is unresolved
    pose_t_ = now();
  }

  void onTargets(const crusader_msgs::msg::TrackedTargetArray::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (!ctx_->origin_set) {return;}     // nothing to express them relative to

    std::vector<nav::Buoy> next;
    next.reserve(m->targets.size());
    for (const auto & t : m->targets) {
      // TENTATIVE tracks are skipped. TrackedTarget.msg is explicit that one
      // "may be one frame of detector noise, so a mission should not commit to
      // it" — and committing here means steering to a particular side of it, or
      // declaring it the ENTRY buoy and circling it. The tracker publishes them
      // anyway because a suppressed obstacle cannot be reconsidered; that is
      // the AVOIDANCE consumer's business, not this one's.
      if (!t.confirmed) {continue;}
      nav::Buoy b;
      b.id = static_cast<int>(t.id);
      b.p = nav::toLocal({t.latitude, t.longitude}, ctx_->origin);
      b.state = beaconFromLabel(t.label);
      b.consumed = consumed_.count(b.id) > 0;
      next.push_back(b);
    }
    tracked_.swap(next);          // RAW. ctx_->buoys is the FUSION; see below.
    targets_t_ = now();
    refuseLocked();
  }

  /// The UAV's passage. Stored raw and folded in by refuseLocked(), because a
  /// plan can arrive before the first GPS fix and there would be no origin to
  /// express it in yet -- dropping it then would lose the plan the whole
  /// mission is gated on.
  void onPlan(const crusader_msgs::msg::PassagePlan::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    plan_raw_ = m;
    plan_t_ = now();
    ctx_->plan_version = m->plan_version;
    refuseLocked();
  }

  /// The next gate pair. Accepted ONLY when it answers the sequence we asked
  /// about: on a lossy radio a retransmission of the previous answer is
  /// otherwise indistinguishable from the answer to this request, and the boat
  /// would drive the wrong gate with nothing in any log looking wrong.
  /// The aircraft's answer to a confirmation request.
  ///
  /// THIS IS AN ACKNOWLEDGEMENT, NOT AN ASSIGNMENT. The message used to carry
  /// the next pair for the boat to drive; the boat now works its own gate order
  /// out of the ten buoys, so all that matters here is the sequence number --
  /// "yes, the field I just retransmitted is current as of your gate N".
  ///
  /// The red_id/green_id fields are vestigial and are deliberately IGNORED
  /// rather than removed: the message is on the air between two vehicles whose
  /// software is updated separately, and a boat that quietly drove whatever
  /// pair an older aircraft happened to put in them would be worse than one
  /// that ignores them. They are logged when set, so a mismatched aircraft is
  /// visible rather than silent.
  void onGate(const crusader_msgs::msg::GatePair::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    if (m->gate_seq != ctx_->gate_seq) {
      RCLCPP_WARN(
        get_logger(), "ignoring a confirmation for gate %u; we asked about %u",
        static_cast<unsigned>(m->gate_seq),
        static_cast<unsigned>(ctx_->gate_seq));
      return;
    }
    const bool carried_a_pair =
      m->red_id != crusader_msgs::msg::GatePair::NO_BUOY ||
      m->green_id != crusader_msgs::msg::GatePair::NO_BUOY;
    if (carried_a_pair) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "the aircraft is still sending gate assignments (red %u, green %u). "
        "Ignored - the boat plans its own gate order now.",
        static_cast<unsigned>(m->red_id), static_cast<unsigned>(m->green_id));
    }
    ctx_->have_confirmation = true;
    gate_t_ = now();
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

    // NO PLAN MEANS CORE TIER, and there the tracker is the only source there
    // is. fusePassage() is right to call every unmatched contact an obstacle --
    // that is what "the UAV wins" means -- but running it against an EMPTY plan
    // would put the whole field in `obstacles` and leave ctx_->buoys empty, so
    // the Core tree would see no buoys at all. Caught in SITL on 2026-09-13:
    // the tracker was publishing and the tree reported buoys_known 0.
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
    ctx_->publish_report = [this] {publishReport();};
    ctx_->report_gate_reached = [this](std::uint8_t seq) {
        std_msgs::msg::UInt8 m;
        m.data = seq;
        gate_reached_pub_->publish(m);
      };
    // Task 3. The JSON is built by dock_math, so the off-ROS sim runner sends
    // byte-identical text and test_dock_math pins the field names.
    ctx_->report_docking = [this](int bay) {
        publishJson(docking_pub_, dock::dockingReportJson(bay));
      };
    ctx_->report_firefighting = [this](int w) {
        publishJson(firefighting_pub_, dock::firefightingReportJson(w));
      };
    ctx_->report_request = [this](const dock::Request & r) {
        publishJson(request_pub_, dock::resourceRequestJson(r));
      };
    ctx_->relay_request = [this](const dock::Request & r) {
        publishJson(uav_request_pub_, dock::uavRequestJson(r, ++uav_seq_));
      };
    ctx_->cannon = [this](bool fire, double x, double y, double z) {
        publishJson(cannon_pub_, dock::cannonJson(fire, x, y, z));
      };
  }

  static void publishJson(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & pub, const std::string & s)
  {
    std_msgs::msg::String m;
    m.data = s;
    pub->publish(m);
  }

  // ------------------------------------------------------------------ Task 3

  /// DockObservation -> dock::Frame -> ingestDockObservation().
  ///
  /// Field copies only. The twin of offros_runner.cpp's frameFromJson(), which
  /// the sim exercises: change one, change both. Which range to believe is
  /// dock::sightingRange's decision, not this function's.
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
        std::isfinite(b.plane_normal[1]);
      s.nx = b.plane_normal[0];
      s.ny = b.plane_normal[1];
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
    dock_t_ = t;
    have_dock_ = true;
  }

  /// ocs_client republishes RoboCommand's commands here as JSON and acts on
  /// none of them. The one Task 3 cares about is the ReadinessConfirm that
  /// answers the docking report; a key test is enough for one key and keeps a
  /// JSON library out of this node.
  void onOcsCommand(const std_msgs::msg::String::SharedPtr m)
  {
    if (m->data.find("\"readiness_confirm\"") == std::string::npos) {return;}
    std::lock_guard<std::mutex> lk(ctx_->mu);
    ctx_->readiness_confirmed = true;
    RCLCPP_INFO(get_logger(), "RoboCommand confirmed the docking report");
  }

  void publishTask(const std::string & token)
  {
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

  /// SafePassageReport as JSON, for ocs_client to relay.
  ///
  /// JSON rather than protobuf for the reason in ocs_link.py: the OCS
  /// re-serialises everything before it reaches RoboCommand, so the boat
  /// carries no generated code. Blank fields are OMITTED rather than sent as
  /// zero — (0, 0) is a real place in the Gulf of Guinea and a report carrying
  /// it reads as a measurement.
  void publishReport()
  {
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(7);
    os << "{\"buoys\":[";
    bool first = true;
    nav::LatLon origin;
    std::vector<nav::Buoy> snap;
    bool he = false, hx = false;
    nav::Vec2 e, x;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!ctx_->origin_set) {return;}
      origin = ctx_->origin;
      snap = ctx_->buoys;
      he = ctx_->have_entry; e = ctx_->entry;
      hx = ctx_->have_exit;  x = ctx_->exitp;
    }
    for (const auto & b : snap) {
      const nav::LatLon ll = nav::toLatLon(b.p, origin);
      if (!first) {os << ",";}
      first = false;
      os << "{\"position\":{\"latitude\":" << ll.lat << ",\"longitude\":" << ll.lon
         << "},\"state\":\"" << beaconName(b.state) << "\"}";
    }
    os << "]";
    if (he) {
      const nav::LatLon ll = nav::toLatLon(e, origin);
      os << ",\"entry_position\":{\"latitude\":" << ll.lat
         << ",\"longitude\":" << ll.lon << "}";
    }
    if (hx) {
      const nav::LatLon ll = nav::toLatLon(x, origin);
      os << ",\"exit_position\":{\"latitude\":" << ll.lat
         << ",\"longitude\":" << ll.lon << "}";
    }
    os << "}";
    std_msgs::msg::String m;
    m.data = os.str();
    report_pub_->publish(m);
  }

  static const char * beaconName(nav::Beacon b)
  {
    switch (b) {
      case nav::Beacon::Off: return "BEACON_STATE_OFF";
      case nav::Beacon::FlashingRed: return "BEACON_STATE_FLASHING_RED";
      case nav::Beacon::FlashingGreen: return "BEACON_STATE_FLASHING_GREEN";
      case nav::Beacon::FlashingBlue: return "BEACON_STATE_FLASHING_BLUE";
      case nav::Beacon::SteadyBlue: return "BEACON_STATE_STEADY_BLUE";
      default: return "BEACON_STATE_UNKNOWN";
    }
  }

  /// Age out the streams. A mission acting on a pose it has not heard for a
  /// second is the frozen-pose failure the whole stack guards against.
  void refreshFreshness()
  {
    const rclcpp::Time t = now();
    std::lock_guard<std::mutex> lk(ctx_->mu);
    ctx_->pose_fresh = ctx_->origin_set &&
      (t - pose_t_).seconds() < stream_timeout_s_;

    // THE PLAN AGES LIKE THE POSE. A dead radio must stop the boat, not leave
    // it driving a passage nobody can still confirm -- rxl_link_node publishes
    // NOTHING when the link is quiet, exactly so this can notice.
    ctx_->plan_age_s = plan_raw_ ? (t - plan_t_).seconds() : 0.0;
    ctx_->plan_fresh = plan_raw_ != nullptr && ctx_->plan_age_s < plan_timeout_s_;

    // And so do the buoys. targets_t_ was recorded here for weeks and never
    // read, so a tracker that died left ctx_->buoys frozen and perfectly
    // convincing. Blanks over guesses: if nothing has arrived, say so.
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
  }

  // ---------------------------------------------------------------- execute

  void execute(const std::shared_ptr<GoalHandle> gh)
  {
    const auto goal = gh->get_goal();
    const double timeout_s = goal->timeout_s > 0.0f ? goal->timeout_s : default_timeout_s_;
    const rclcpp::Time started = now();
    auto result = std::make_shared<SafePassage::Result>();
    result->entry_latitude = std::nan("");
    result->entry_longitude = std::nan("");
    result->exit_latitude = std::nan("");
    result->exit_longitude = std::nan("");

    consumed_.clear();
    // BEFORE reading pose_fresh below. It is computed by refreshFreshness(),
    // which otherwise only runs inside the tick loop — so at goal start it
    // still holds whatever the last mission left, which on a freshly started
    // runner is false. That made have_home false for the whole mission and
    // NavigateTo target="home" fail at the very end of a tour that had
    // otherwise worked. Found in SITL 2026-09-06.
    refreshFreshness();
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->approach_lat = goal->approach_latitude;
      ctx_->approach_lon = goal->approach_longitude;
      ctx_->has_approach = goal->approach_latitude != 0.0 || goal->approach_longitude != 0.0;
      ctx_->have_waypoint = false;
      // THE GATE HANDSHAKE IS PER-MISSION STATE AND MUST START EMPTY.
      //
      // passage_complete latches when the aircraft answers 255/255, and it used
      // to survive the end of the mission. The next goal then found it already
      // true on the FIRST tick: PassageComplete succeeded, the Inverter failed,
      // KeepRunningUntilFailure ended, and the boat orbited the entry, drove to
      // the exit, orbited that, and reported RESULT SUCCESS having never asked
      // for a single gate. 62.6 s, no warning, no failed leaf. SITL 2026-09-13.
      //
      // gate_seq carried over the same way, so a second run also re-asked under
      // whatever sequence number the last one stopped at. The aircraft answers a
      // REPEATED seq idempotently -- that is what makes a lossy radio safe --
      // which means a stale seq is answered with the previous run's pair rather
      // than being noticed as wrong.
      //
      // The aircraft rewinds its own half in set_gates; this is the boat's.
      ctx_->passage_complete = false;
      ctx_->gate_seq = 0;
      ctx_->gate_cleared = false;
      ctx_->have_confirmation = false;
      ctx_->gate_red_id = -1;
      ctx_->gate_green_id = -1;
      ctx_->cleared_gates.clear();
      ctx_->passage = nav::Passage{};
      // Task 3's per-mission state, for the same reason: a second attempt must
      // not inherit the first one's bays, votes or committed bay.
      ctx_->tier = goal->tier;
      resetTask3(*ctx_);
      // "Home" is where THIS attempt started, captured once. Not the autopilot's
      // HOME, which is wherever it was armed and is usually somewhere else after
      // the boat has been driven out manually.
      ctx_->home = ctx_->boat;
      ctx_->have_home = ctx_->pose_fresh;
    }

    uint8_t outcome = SafePassage::Result::OUTCOME_FAULT;
    std::string detail;
    try {
      BT::BehaviorTreeFactory factory;
      registerCrusaderNodes(factory);
      auto bb = BT::Blackboard::create();
      bb->set("ctx", ctx_);
      BT::Tree tree = factory.createTreeFromFile(tree_file_, bb);

      // The whole tree with every node's status, reprinted only when something
      // changes. See tree_view.hpp for why this rather than the transition log
      // BT.CPP ships: a scrolling list of transitions cannot answer "where is
      // it now and what has it already done" without replaying it in your head.
      TreeView view(tree);

      RCLCPP_INFO(
        get_logger(), "GOAL tier=%u timeout=%.0fs approach=(%.7f, %.7f)",
        goal->tier, timeout_s, goal->approach_latitude, goal->approach_longitude);

      rclcpp::Rate rate(tick_hz_);
      auto fb = std::make_shared<SafePassage::Feedback>();
      BT::NodeStatus st = BT::NodeStatus::RUNNING;

      while (rclcpp::ok() && !stop_) {
        // Cancel first. A cancel that waits for anything is a cancel that does
        // not work.
        if (gh->is_canceling()) {
          tree.haltTree();
          outcome = SafePassage::Result::OUTCOME_CANCELLED;
          detail = "cancelled by the operator";
          break;
        }
        const double elapsed = (now() - started).seconds();
        if (elapsed >= timeout_s) {
          tree.haltTree();
          outcome = SafePassage::Result::OUTCOME_TIMEOUT;
          detail = "mission timeout " + std::to_string(static_cast<int>(timeout_s)) +
            "s expired";
          break;
        }
        // An unknown mode is not a manual one. Tolerated briefly at goal start
        // (HEARTBEAT may not have landed), never for long.
        if (!have_status_ && elapsed > mode_grace_s_) {
          tree.haltTree();
          outcome = SafePassage::Result::OUTCOME_FAULT;
          detail = "no HEARTBEAT — cannot tell whether we are allowed to drive";
          break;
        }

        refreshFreshness();
        st = tree.tickOnce();

        if (verbose_tree_) {
          const std::string frame = view.renderIfChanged();
          if (!frame.empty()) {
            RCLCPP_INFO(get_logger(), "tree @ %5.1fs\n%s", elapsed, frame.c_str());
          }
        }
        // JSON every tick, whether or not it changed: a GUI wants the current
        // picture when it asks, not the last time something happened to move.
        // Structured rather than the rendered text, because a viewer that has
        // to parse box-drawing characters back into a tree is a viewer that
        // breaks the first time a node is renamed.
        {
          std_msgs::msg::String bs;
          bs.data = view.json(elapsed);
          bt_status_pub_->publish(bs);
        }

        // The LED heartbeat, inside the loop on purpose: the mast light goes
        // GREEN because a mission is actually ticking, not because a node is
        // merely alive.
        std_msgs::msg::Bool a;
        a.data = true;
        autonomy_pub_->publish(a);

        publishFeedback(gh, fb, elapsed, timeout_s);

        if (st != BT::NodeStatus::RUNNING) {
          if (st == BT::NodeStatus::SUCCESS) {
            outcome = SafePassage::Result::OUTCOME_SUCCESS;
            // Says what happened, not what a Task 1 run WOULD have done. The
            // runner ticks whatever tree_file names — the demo tour reported
            // "entry circled, field transited, exit circled" for a mission that
            // did none of those, which is the kind of confident-and-wrong
            // report that costs an hour later.
            detail = "tree completed: " + treeName();
          } else {
            std::lock_guard<std::mutex> lk(ctx_->mu);
            if (!ctx_->autonomous) {
              outcome = SafePassage::Result::OUTCOME_NOT_AUTONOMOUS;
              detail = "flight mode left the autonomous set — the pilot took "
                "control; not fighting for it";
            } else {
              outcome = SafePassage::Result::OUTCOME_NO_ENTRY;
              detail = "tree " + treeName() + " returned FAILURE — see the "
                "node transitions in the log for which leaf did it";
            }
          }
          break;
        }
        rate.sleep();
      }
      fillResult(result);
    } catch (const std::exception & e) {
      // NEVER let this reach rclcpp_action: an unset Result has outcome 0,
      // which is OUTCOME_SUCCESS. A crash that reports success is the worst
      // failure this stack can have.
      outcome = SafePassage::Result::OUTCOME_FAULT;
      detail = std::string("tree raised: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
    }

    result->outcome = outcome;
    result->detail = detail;
    result->elapsed_s = (now() - started).seconds();

    if (outcome == SafePassage::Result::OUTCOME_SUCCESS) {
      RCLCPP_INFO(get_logger(), "RESULT SUCCESS after %.1fs: %s",
        result->elapsed_s, detail.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "RESULT %u after %.1fs: %s",
        outcome, result->elapsed_s, detail.c_str());
    }

    // The RIGHT terminal state, not just any: a cancelled goal that is
    // succeed()-ed reports success, and the caller carries on as though the
    // mission had worked.
    if (outcome == SafePassage::Result::OUTCOME_CANCELLED) {
      gh->canceled(result);
    } else if (outcome == SafePassage::Result::OUTCOME_SUCCESS) {
      gh->succeed(result);
    } else {
      gh->abort(result);
    }

    // Every exit path, including the exception one. Leaving current_task set
    // tells the OCS the attempt is still running on behalf of a mission that
    // has stopped. And the pump OFF: SprayUntilHit switches it off when it is
    // halted, but a tree that throws never halts its leaves.
    if (ctx_->cannon) {ctx_->cannon(false, 0.0, 0.0, 0.0);}
    publishTask(kTaskNone);
    publishAvoidance(true);
    std_msgs::msg::Bool off;
    off.data = false;
    autonomy_pub_->publish(off);
    busy_ = false;
  }

  void publishFeedback(
    const std::shared_ptr<GoalHandle> & gh,
    const std::shared_ptr<SafePassage::Feedback> & fb,
    double elapsed, double timeout_s)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    // A Task 3 tree names its own phase; a Task 1 tree never sets it.
    fb->phase = !ctx_->task3_phase.empty() ? ctx_->task3_phase :
      (ctx_->have_exit ? "TRANSIT" : (ctx_->have_entry ? "ENTRY" : "APPROACH"));
    fb->progress = static_cast<float>(std::min(1.0, elapsed / timeout_s));
    fb->buoys_known = static_cast<uint32_t>(ctx_->buoys.size());
    uint32_t resolved = 0;
    for (const auto & b : ctx_->buoys) {
      if (b.state != nav::Beacon::Unknown) {++resolved;}
    }
    fb->buoys_resolved = resolved;
    // Was consumed_.size(), which was a placeholder from before a plan
    // version existed: the field is documented as the UAV's plan version and
    // an operator watching it would have read buoy progress as re-tasking.
    fb->plan_version = ctx_->plan_version;
    fb->warning = ctx_->pose_fresh ? "" : "pose is stale";
    gh->publish_feedback(fb);
  }

  void fillResult(const std::shared_ptr<SafePassage::Result> & r)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    r->buoys_classified = 0;
    for (const auto & b : ctx_->buoys) {
      if (b.state != nav::Beacon::Unknown) {++r->buoys_classified;}
    }
    r->buoys_passed_correctly = 0;      // needs the logged track; see README
    if (!ctx_->origin_set) {return;}
    if (ctx_->have_entry) {
      const nav::LatLon ll = nav::toLatLon(ctx_->entry, ctx_->origin);
      r->entry_latitude = ll.lat;
      r->entry_longitude = ll.lon;
    }
    if (ctx_->have_exit) {
      const nav::LatLon ll = nav::toLatLon(ctx_->exitp, ctx_->origin);
      r->exit_latitude = ll.lat;
      r->exit_longitude = ll.lon;
    }
  }

  /// The tree file's basename, for result messages that name what actually ran.
  std::string treeName() const
  {
    const auto slash = tree_file_.find_last_of('/');
    std::string n = slash == std::string::npos ? tree_file_
      : tree_file_.substr(slash + 1);
    const auto dot = n.find_last_of('.');
    return dot == std::string::npos ? n : n.substr(0, dot);
  }

  /// Stop the mission thread and JOIN it before any member it touches is
  /// destroyed. Without this, a Ctrl+C or `systemctl stop` mid-mission tears
  /// the node down under a thread that is still publishing through it.
  void shutdown()
  {
    stop_ = true;
    if (worker_.joinable()) {worker_.join();}
  }

  // ------------------------------------------------------------------ state
  ContextPtr ctx_;
  std::thread worker_;
  std::atomic<bool> stop_{false};
  std::string tree_file_, action_name_, task_token_;
  double tick_hz_ = 10.0, default_timeout_s_ = 600.0, mode_grace_s_ = 3.0;
  double stream_timeout_s_ = 1.0;
  double plan_timeout_s_ = 15.0;
  double assoc_radius_m_ = 5.0;
  bool verbose_tree_ = true;
  std::set<std::string> auto_modes_;
  std::set<int> consumed_;
  std::atomic<bool> busy_{false};
  bool have_status_ = false;
  rclcpp::Time pose_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time status_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time targets_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time plan_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time gate_t_{0, 0, RCL_ROS_TIME};
  /// RAW inputs. ctx_->buoys is the FUSION of these two; see refuseLocked().
  std::vector<nav::Buoy> tracked_;
  crusader_msgs::msg::PassagePlan::SharedPtr plan_raw_;

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp_action::Server<SafePassage>::SharedPtr server_;
  rclcpp::Publisher<crusader_msgs::msg::GuidedSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gate_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_, report_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bt_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr avoid_pub_, autonomy_pub_;
  rclcpp::Subscription<crusader_msgs::msg::FcuStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<crusader_msgs::msg::LatLonHead>::SharedPtr pose_sub_;
  rclcpp::Subscription<crusader_msgs::msg::TrackedTargetArray>::SharedPtr targets_sub_;
  rclcpp::Subscription<crusader_msgs::msg::PassagePlan>::SharedPtr plan_sub_;
  rclcpp::Subscription<crusader_msgs::msg::GatePair>::SharedPtr gate_sub_;

  // Task 3
  std::string dock_topic_;
  rclcpp::Time dock_t_{0, 0, RCL_ROS_TIME};
  bool have_dock_ = false;
  int uav_seq_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr docking_pub_, firefighting_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr request_pub_, uav_request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cannon_pub_;
  rclcpp::Subscription<crusader_msgs::msg::DockObservation>::SharedPtr dock_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ocs_sub_;
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
