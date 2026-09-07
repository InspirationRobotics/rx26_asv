// bt_runner_node — Crusader's bt_navigator: one coarse action outside, a tree
// of primitives inside.
//
//     ros2 action send_goal /crsd/safe_passage crusader_msgs/action/SafePassage "{tier: 0, timeout_s: 600}"
//
//   in:  /crsd/fcu_status    (FcuStatus)          mode + armed — THE autonomy latch
//        /crsd/pose          (LatLonHead)         position and GPS-yaw heading
//        /crsd/world_targets (TrackedTargetArray) the buoy field
//   out: /crsd/guided_setpoint    (GuidedSetpoint) ONLY when publish_setpoints
//        /crsd/current_task       (String, latched)
//        /crsd/autonomy_active    (Bool)   turns the mast light GREEN
//        /crsd/avoidance_enable   (Bool, latched)
//        /crsd/safe_passage_report (String, JSON)
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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "crusader_msgs/action/safe_passage.hpp"
#include "crusader_msgs/msg/fcu_status.hpp"
#include "crusader_msgs/msg/guided_setpoint.hpp"
#include "crusader_msgs/msg/lat_lon_head.hpp"
#include "crusader_msgs/msg/tracked_target_array.hpp"

#include "crusader_bt/context.hpp"
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
  : rclcpp::Node("safe_passage_server")
  {
    tree_file_ = declare_parameter<std::string>("tree_file", "");
    action_name_ = declare_parameter<std::string>("action_name", "/crsd/safe_passage");
    tick_hz_ = declare_parameter<double>("tick_hz", 10.0);
    default_timeout_s_ = declare_parameter<double>("default_timeout_s", 600.0);
    mode_grace_s_ = declare_parameter<double>("mode_grace_s", 3.0);
    stream_timeout_s_ = declare_parameter<double>("stream_timeout_s", 1.0);
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

    // Latched: a subscriber that starts mid-mission must learn the current
    // value rather than sit on a default. avoidance_enable especially —
    // proximity_bridge coming up late and defaulting to off would silently
    // disarm avoidance.
    auto latched = rclcpp::QoS(1).reliable().transient_local();
    setpoint_pub_ = create_publisher<crusader_msgs::msg::GuidedSetpoint>(
      "/crsd/guided_setpoint", 10);
    task_pub_ = create_publisher<std_msgs::msg::String>("/crsd/current_task", latched);
    avoid_pub_ = create_publisher<std_msgs::msg::Bool>("/crsd/avoidance_enable", latched);
    autonomy_pub_ = create_publisher<std_msgs::msg::Bool>("/crsd/autonomy_active", 10);
    report_pub_ = create_publisher<std_msgs::msg::String>("/crsd/safe_passage_report", 10);

    status_sub_ = create_subscription<crusader_msgs::msg::FcuStatus>(
      "/crsd/fcu_status", 10,
      [this](crusader_msgs::msg::FcuStatus::SharedPtr m) {onStatus(m);});
    pose_sub_ = create_subscription<crusader_msgs::msg::LatLonHead>(
      "/crsd/pose", 10,
      [this](crusader_msgs::msg::LatLonHead::SharedPtr m) {onPose(m);});
    targets_sub_ = create_subscription<crusader_msgs::msg::TrackedTargetArray>(
      "/crsd/world_targets", 10,
      [this](crusader_msgs::msg::TrackedTargetArray::SharedPtr m) {onTargets(m);});

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
    ctx_->buoys.swap(next);

    const nav::Buoy * e = nav::findBeacon(ctx_->buoys, nav::Beacon::FlashingBlue);
    const nav::Buoy * x = nav::findBeacon(ctx_->buoys, nav::Beacon::SteadyBlue);
    ctx_->have_entry = e != nullptr;
    if (e) {ctx_->entry = e->p;}
    ctx_->have_exit = x != nullptr;
    if (x) {ctx_->exitp = x->p;}
    targets_t_ = now();
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
    if (have_status_ && (t - status_t_).seconds() >= stream_timeout_s_) {
      ctx_->autonomous = false;        // a dead HEARTBEAT is not permission
    }
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
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->approach_lat = goal->approach_latitude;
      ctx_->approach_lon = goal->approach_longitude;
      ctx_->has_approach = goal->approach_latitude != 0.0 || goal->approach_longitude != 0.0;
      ctx_->have_waypoint = false;
    }

    uint8_t outcome = SafePassage::Result::OUTCOME_FAULT;
    std::string detail;
    try {
      BT::BehaviorTreeFactory factory;
      registerCrusaderNodes(factory);
      auto bb = BT::Blackboard::create();
      bb->set("ctx", ctx_);
      BT::Tree tree = factory.createTreeFromFile(tree_file_, bb);

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
            detail = "entry circled, field transited, exit circled";
          } else {
            std::lock_guard<std::mutex> lk(ctx_->mu);
            if (!ctx_->autonomous) {
              outcome = SafePassage::Result::OUTCOME_NOT_AUTONOMOUS;
              detail = "flight mode left the autonomous set — the pilot took "
                "control; not fighting for it";
            } else {
              outcome = SafePassage::Result::OUTCOME_NO_ENTRY;
              detail = "the tree failed; most likely the entry or exit buoy "
                "never resolved";
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
    // has stopped.
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
    fb->phase = ctx_->have_exit ? "TRANSIT" : (ctx_->have_entry ? "ENTRY" : "APPROACH");
    fb->progress = static_cast<float>(std::min(1.0, elapsed / timeout_s));
    fb->buoys_known = static_cast<uint32_t>(ctx_->buoys.size());
    uint32_t resolved = 0;
    for (const auto & b : ctx_->buoys) {
      if (b.state != nav::Beacon::Unknown) {++resolved;}
    }
    fb->buoys_resolved = resolved;
    fb->plan_version = static_cast<uint32_t>(consumed_.size());
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
  std::set<std::string> auto_modes_;
  std::set<int> consumed_;
  std::atomic<bool> busy_{false};
  bool have_status_ = false;
  rclcpp::Time pose_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time status_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time targets_t_{0, 0, RCL_ROS_TIME};

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp_action::Server<SafePassage>::SharedPtr server_;
  rclcpp::Publisher<crusader_msgs::msg::GuidedSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_, report_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr avoid_pub_, autonomy_pub_;
  rclcpp::Subscription<crusader_msgs::msg::FcuStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<crusader_msgs::msg::LatLonHead>::SharedPtr pose_sub_;
  rclcpp::Subscription<crusader_msgs::msg::TrackedTargetArray>::SharedPtr targets_sub_;
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
