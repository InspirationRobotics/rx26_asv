// offros_runner — bt_runner_node without ROS, for the Task 3 simulator.
//
//     tools/task3_sim/.build/offros_runner --tree crusader_bt/behavior_trees/task3_disruptive.xml
//
// THE SAME TREE, THE SAME LEAVES, THE SAME CONTEXT. Built by
// tools/task3_sim/build.py from leaves.cpp, task3_leaves.cpp and this file,
// against BehaviorTree.CPP compiled from source and a four-symbol rclcpp shim
// (offros/shim). Only bt_runner_node's plumbing is replaced: its subscriptions
// become JSON lines on STDIN and its publishers become JSON lines on STDOUT.
// Logs go to STDERR.
//
// WHY. The Task 1 rig needs WSL2, Docker and ArduRover SITL, and a laptop
// without hardware virtualisation can run none of them. Everything the tree
// decides can still be exercised: it reads a Context and calls functions.
//
// WHAT MUST STAY IN STEP WITH bt_runner_node.cpp, because this is a copy of
// its shape rather than a shared implementation (that one cannot be compiled
// without ROS):
//
//   execute()           cancel first; the mission timeout; no HEARTBEAT past
//                       mode_grace_s is a fault; refresh, tick, report; the
//                       FAILURE-means-not-autonomous-or-tree-failed split
//   refreshFreshness()  pose and dock observations age out; a dead HEARTBEAT
//                       is not permission
//   onPose()            the origin is pinned ONCE, at the first fix - unless
//                       nav_mode is shadow or on, when it comes from the datum
//
// What a DockObservation MEANS is NOT copied: both runners hand a dock::Frame
// to ingestDockObservation() in context.hpp. The frame rule is NOT copied either:
// onPose and onDatum are one call each into ingestPose() and applyDatum() there,
// and the leg status JSON is legStatusJson()'s, so the two runners cannot differ.
//
// OBSTACLE AVOIDANCE (docs/nav2_avoidance_spec.md 5.7). `--nav-mode off|shadow|on`,
// default off: tools/task3_sim/test_e2e.py runs against off and is unchanged. With
// shadow or on the planner is path::StraightPlannerPort - an instant straight
// [start, goal] path, not a planner - so the REAL PlannedLeg state machine and the
// real leaves run with no Nav2 around; the leg's own "plan crosses a known hazard"
// rejection is what makes it refuse a line through the dock. With shadow or on the
// origin is NOT pinned at the first fix: it waits for a "datum" line, as
// bt_runner_node waits for /crsd/datum.
//
// STDIN, one JSON object per line ("type" says which):
//   status        {mode, armed}                           /crsd/fcu_status
//   pose          {lat, lon, heading|null}                /crsd/pose
//   datum         {lat, lon}                              /crsd/datum
//   dock_obs      DockObservation's fields, stamp in s    dock/observations
//   ocs_command   {data: {...}}                           /crsd/ocs_command
//   mount         {x, y, yaw_deg, pitch_deg, face_dz}     bt_runner_node params
//   wall_range    {valid, range_m, angle_deg, lat_m}      /crsd/wall_range
//   attitude      {roll, pitch, rollspeed, pitchspeed}    /crsd/attitude (rad)
//   pump_state    {enabled, on, last_seq, last_result, last_reason}  /crsd/pump_state
//   autonomy_drop {data: bool}                            /crsd/autonomy_drop
//   goal          {tier, timeout_s, approach_latitude, approach_longitude}
//   cancel, quit
// STDOUT: ready, setpoint, task, autonomy, docking_report, firefighting_report,
//   resource_request, uav_request, cannon, bt, book, feedback, result,
//   heading_speed {heading_deg, speed_mps}, pump {duration_s, seq, source},
//   avoidance {enable},
//   leg {t, leaf, name, mode, avoid, state, why, blocked_s, goal, target, path,
//        plan_ms, [hop, hops]}   /crsd/nav/leg_status (spec 4.3), on change,
//   hazards {n}                  how many hazards /crsd/nav/hazards would carry, 2 Hz,
//                                only once the origin IS the datum.
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/contrib/json.hpp"

#include "crusader_bt/context.hpp"
#include "crusader_bt/dock_math.hpp"
#include "crusader_bt/nav_math.hpp"
#include "crusader_bt/tree_view.hpp"

namespace crusader_bt
{
namespace offros
{

using json = nlohmann::json;
using Clock = std::chrono::steady_clock;

// SafePassage.action's outcomes: the action stays the one mission trigger.
constexpr int OUTCOME_SUCCESS = 0;
constexpr int OUTCOME_TIMEOUT = 1;
constexpr int OUTCOME_NO_ENTRY = 2;
constexpr int OUTCOME_CANCELLED = 3;
constexpr int OUTCOME_NOT_AUTONOMOUS = 4;
constexpr int OUTCOME_FAULT = 5;

static constexpr const char * kTaskNone = "TASK_NONE";

/// A number from a JSON object, or `dflt` when the key is missing OR null.
/// The sim writes NaN as null, because NaN is not JSON.
double num(const json & j, const char * key, double dflt)
{
  const auto it = j.find(key);
  if (it == j.end() || !it->is_number()) {return dflt;}
  return it->get<double>();
}

/// A DockObservation, as JSON with the message's own field names -> dock::Frame.
///
/// The twin of bt_runner_node's conversion from crusader_msgs/DockObservation.
/// Field copies only; the range choice is dock::sightingRange, so neither
/// runner decides it.
dock::Frame frameFromJson(const json & j)
{
  dock::Frame f;
  f.t = num(j, "stamp", 0.0);
  for (const auto & b : j.value("bays", json::array())) {
    dock::BaySighting s;
    s.bearing_deg = num(b, "bearing_deg", dock::kNaN);
    const bool has_plane = b.value("has_plane", false);
    double n[3] = {0.0, 0.0, 0.0};
    const json pn = b.value("plane_normal", json::array());
    for (std::size_t i = 0; i < 3 && i < pn.size(); ++i) {
      n[i] = pn[i].is_number() ? pn[i].get<double>() : dock::kNaN;
    }
    s.range_m = dock::sightingRange(
      has_plane, n[0], n[1], num(b, "plane_offset", dock::kNaN), s.bearing_deg,
      num(b, "range_from_size_m", dock::kNaN));
    s.has_normal = has_plane && std::isfinite(n[0]) && std::isfinite(n[1]) &&
      std::isfinite(n[2]);
    s.nx = n[0];
    s.ny = n[1];
    s.nz = n[2];
    s.d = num(b, "plane_offset", dock::kNaN);
    s.truncated = b.value("truncated", false);
    s.indicator_present = b.value("indicator_present", false);
    s.indicator = dock::colourFromCv(b.value("indicator_colour", 0));
    s.indicator_conf = num(b, "indicator_confidence", 0.0);
    s.lit_window_index = b.value("lit_window_index", -1);
    for (const auto & w : b.value("windows", json::array())) {
      dock::WindowSighting ws;
      ws.index = w.value("index", -1);
      ws.state = dock::colourFromCv(w.value("state", 0));
      ws.conf = num(w, "state_confidence", 0.0);
      ws.has_position = w.value("has_position", false);
      ws.x = num(w, "x", 0.0);
      ws.y = num(w, "y", 0.0);
      ws.z = num(w, "z", 0.0);
      s.windows.push_back(ws);
    }
    f.bays.push_back(s);
  }
  f.target_pattern = j.value("target_pattern", std::string());
  for (const auto & c : j.value("target_colours", json::array())) {
    f.target_colours.push_back(dock::colourFromName(c.get<std::string>()));
  }
  f.target_window_index = j.value("target_window_index", -1);
  f.last_event = j.value("last_event", std::string());
  f.observed_fps = num(j, "observed_fps", 0.0);
  return f;
}

class Runner
{
public:
  struct Params
  {
    std::string tree_file;
    double tick_hz = 10.0;
    double default_timeout_s = 600.0;
    double mode_grace_s = 3.0;
    double stream_timeout_s = 1.0;
    bool publish_setpoints = false;
    bool fire_pump = false;        ///< bt_runner_node's fire_pump: FireBurst may squirt
    int task3_bay = 0;             ///< bt_runner_node's task3_bay: part 2's bay, 0 = not given
    bool verbose_tree = false;
    path::Mode nav_mode = path::Mode::Off;   ///< bt_runner_node's nav_mode
  };

  explicit Runner(const Params & p)
  : p_(p), node_("bt_runner_node")
  {
    ctx_ = std::make_shared<Context>();
    ctx_->node = &node_;
    ctx_->publish_setpoints = p_.publish_setpoints;
    ctx_->fire_pump = p_.fire_pump;
    ctx_->nav_mode = p_.nav_mode;
    if (ctx_->nav_mode != path::Mode::Off) {
      ctx_->planner = std::make_shared<path::StraightPlannerPort>();
    }
    t0_ = Clock::now();
    wireContext();
  }

  /// Read stdin until quit/EOF, dispatching as bt_runner_node's callbacks do.
  void readLoop()
  {
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line.empty()) {continue;}
      json j;
      try {
        j = json::parse(line);
      } catch (const std::exception & e) {
        RCLCPP_WARN(node_.get_logger(), "dropped a malformed line: %s", e.what());
        continue;
      }
      const std::string type = j.value("type", std::string());
      if (type == "status") {
        onStatus(j);
      } else if (type == "pose") {
        onPose(j);
      } else if (type == "datum") {
        onDatum(j);
      } else if (type == "dock_obs") {
        onDock(j);
      } else if (type == "ocs_command") {
        onOcsCommand(j);
      } else if (type == "mount") {
        onMount(j);
      } else if (type == "wall_range") {
        onWallRange(j);
      } else if (type == "attitude") {
        onAttitude(j);
      } else if (type == "pump_state") {
        onPumpState(j);
      } else if (type == "autonomy_drop") {
        std::lock_guard<std::mutex> lk(ctx_->mu);
        ctx_->drop_tripped = j.value("data", true);
      } else if (type == "goal") {
        std::lock_guard<std::mutex> lk(goal_mu_);
        if (busy_) {
          RCLCPP_WARN(node_.get_logger(),
            "REJECTING a second goal — one mission at a time. Cancel the running one first.");
          emit({{"type", "result"}, {"outcome", -1}, {"detail", "goal REJECTED: one already running"}});
        } else {
          pending_ = j;
          have_pending_ = true;
          busy_ = true;
          goal_cv_.notify_one();
        }
      } else if (type == "cancel") {
        cancel_ = true;
      } else if (type == "quit") {
        break;
      }
    }
    stop_ = true;
    cancel_ = true;
    goal_cv_.notify_one();
  }

  /// Wait for goals and run them, one at a time, on THIS thread.
  void missionLoop()
  {
    emit({{"type", "ready"}, {"tree", p_.tree_file},
        {"publish_setpoints", p_.publish_setpoints}, {"fire_pump", p_.fire_pump},
        {"nav_mode", navModeName(ctx_->nav_mode)}});
    publishTask(kTaskNone);
    while (!stop_) {
      json goal;
      {
        std::unique_lock<std::mutex> lk(goal_mu_);
        goal_cv_.wait(lk, [this] {return have_pending_ || stop_;});
        if (stop_) {break;}
        goal = pending_;
        have_pending_ = false;
      }
      cancel_ = false;
      execute(goal);
      std::lock_guard<std::mutex> lk(goal_mu_);
      busy_ = false;
    }
  }

private:
  // ------------------------------------------------------------- inputs

  void onStatus(const json & j)
  {
    std::string mode = j.value("mode", std::string());
    for (auto & c : mode) {c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));}
    std::lock_guard<std::mutex> lk(ctx_->mu);
    // shared.autonomous_modes, as bt_runner_node declares it by default.
    ctx_->autonomous = mode == "GUIDED" || mode == "AUTO" || mode == "LOITER" || mode == "RTL";
    ctx_->mode = mode;
    status_t_ = Clock::now();
    have_status_ = true;
  }

  /// The frame rule (spec 2) is ingestPose's, shared with bt_runner_node.
  void onPose(const json & j)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const double lat = num(j, "lat", dock::kNaN), lon = num(j, "lon", dock::kNaN);
    // null heading = GPS yaw unresolved
    switch (ingestPose(*ctx_, {lat, lon}, num(j, "heading", dock::kNaN))) {
      case PoseEvent::Bad:
        return;
      case PoseEvent::Pinned:
        RCLCPP_INFO(node_.get_logger(), "local frame origin pinned at %.7f, %.7f (no /crsd/datum: "
          "pinned at the first fix; nav_mode off, frame not shared)", lat, lon);
        break;
      case PoseEvent::Waiting:
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
          "waiting for a datum line (nav_mode %s does not pin the frame at the first fix)",
          navModeName(ctx_->nav_mode));
        break;
      case PoseEvent::Tracked:
        break;
    }
    ingestHeading(*ctx_, nowS());
    pose_t_ = Clock::now();
    have_pose_ = true;
  }

  /// {"type":"datum","lat":..,"lon":..}: the frame origin, as /crsd/datum is for
  /// bt_runner_node. The rule is applyDatum's.
  void onDatum(const json & j)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    const double lat = num(j, "lat", dock::kNaN), lon = num(j, "lon", dock::kNaN);
    switch (applyDatum(*ctx_, {lat, lon})) {
      case DatumEvent::Adopted:
        RCLCPP_INFO(node_.get_logger(), "local frame origin adopted from the datum: %.7f, %.7f",
          lat, lon);
        break;
      case DatumEvent::Mismatch:
        RCLCPP_ERROR(node_.get_logger(), "datum %.7f, %.7f differs from the origin in use "
          "(%.7f, %.7f): datum changed, restart the runner. Planned legs hold until then.",
          lat, lon, ctx_->origin.lat, ctx_->origin.lon);
        break;
      case DatumEvent::Unchanged:
      case DatumEvent::Ignored:
        break;
    }
  }

  void onDock(const json & j)
  {
    const dock::Frame f = frameFromJson(j);
    std::lock_guard<std::mutex> lk(ctx_->mu);
    // Freshness first: ingest places bays only on a fresh pose.
    ctx_->pose_fresh = have_pose_ && ctx_->origin_set &&
      secondsSince(pose_t_) < p_.stream_timeout_s;
    ingestDockObservation(*ctx_, f);
    ingestFireWindows(*ctx_, nowS(), f);
    dock_t_ = Clock::now();
    have_dock_ = true;
  }

  /// The camera extrinsic and the face height: bt_runner_node's cam_x,
  /// cam_y, cam_yaw_deg, cam_pitch_deg and dock_face_dz_m parameters. The sim
  /// sends them, because its scenario can change the camera's pitch.
  void onMount(const json & j)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    ctx_->cam_mount.x = num(j, "x", ctx_->cam_mount.x);
    ctx_->cam_mount.y = num(j, "y", ctx_->cam_mount.y);
    ctx_->cam_mount.yaw_deg = num(j, "yaw_deg", ctx_->cam_mount.yaw_deg);
    ctx_->cam_mount.pitch_deg = num(j, "pitch_deg", ctx_->cam_mount.pitch_deg);
    ctx_->dock_face_dz = num(j, "face_dz", ctx_->dock_face_dz);
    RCLCPP_INFO(node_.get_logger(), "camera mount: x %.2f y %.2f yaw %.1f pitch %.1f, face %+.2f m",
      ctx_->cam_mount.x, ctx_->cam_mount.y, ctx_->cam_mount.yaw_deg, ctx_->cam_mount.pitch_deg,
      ctx_->dock_face_dz);
  }

  /// /crsd/wall_range: crusader_msgs/WallRange as JSON (valid, range_m,
  /// angle_deg, lat_m; null = NaN). Stamped on THIS runner's clock at receipt.
  void onWallRange(const json & j)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    ingestWallRange(*ctx_, nowS(), j.value("valid", false), num(j, "range_m", dock::kNaN),
      num(j, "angle_deg", dock::kNaN), num(j, "lat_m", dock::kNaN));
  }

  /// /crsd/attitude: roll, pitch and their rates in radians (the autopilot's axes).
  void onAttitude(const json & j)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    ingestAttitude(*ctx_, nowS(), num(j, "roll", 0.0), num(j, "pitch", 0.0),
      num(j, "rollspeed", 0.0), num(j, "pitchspeed", 0.0), num(j, "yawspeed", dock::kNaN));
    att_t_ = Clock::now();
    have_att_ = true;
  }

  /// /crsd/pump_state: what the bridge did with our bursts.
  void onPumpState(const json & j)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    ctx_->pump_enabled = j.value("enabled", false);
    ctx_->pump_on = j.value("on", false);
    ctx_->pump_last_seq = j.value("last_seq", 0u);
    ctx_->pump_result = j.value("last_result", 0);
    ctx_->pump_reason = j.value("last_reason", std::string());
    pump_t_ = Clock::now();
    have_pump_ = true;
  }

  double nowS() const {return std::chrono::duration<double>(Clock::now() - t0_).count();}

  /// /crsd/ocs_command: RoboCommand's RxCommand as JSON. Only the readiness
  /// confirmation matters to Task 3, and the tree does not wait on it - it
  /// waits on the light. It only stops the docking report being re-sent.
  void onOcsCommand(const json & j)
  {
    const json d = j.value("data", json::object());
    if (d.contains("readiness_confirm")) {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->readiness_confirmed = true;
      RCLCPP_INFO(node_.get_logger(), "RoboCommand confirmed the docking report");
    }
  }

  // ------------------------------------------------------------ outputs

  void emit(const json & j)
  {
    const std::string s = j.dump();
    std::lock_guard<std::mutex> lk(out_mu_);
    std::fwrite(s.data(), 1, s.size(), stdout);
    std::fputc('\n', stdout);
    std::fflush(stdout);
  }

  void publishTask(const std::string & t) {emit({{"type", "task"}, {"token", t}});}

  void wireContext()
  {
    ctx_->send_setpoint = [this](nav::LatLon ll) {
        emit({{"type", "setpoint"}, {"lat", ll.lat}, {"lon", ll.lon}});
      };
    ctx_->set_task = [this](const std::string & t) {publishTask(t);};
    ctx_->consume_buoy = [](int) {};
    ctx_->publish_report = [] {};
    ctx_->report_gate_reached = [](std::uint8_t) {};
    ctx_->report_docking = [this](int bay) {
        emit({{"type", "docking_report"}, {"json", dock::dockingReportJson(bay)}});
      };
    ctx_->report_firefighting = [this](int w) {
        emit({{"type", "firefighting_report"}, {"json", dock::firefightingReportJson(w)}});
      };
    ctx_->report_request = [this](const dock::Request & r) {
        emit({{"type", "resource_request"}, {"json", dock::resourceRequestJson(r)}});
      };
    ctx_->relay_request = [this](const dock::Request & r) {
        emit({{"type", "uav_request"}, {"json", dock::uavRequestJson(r, ++uav_seq_)}});
      };
    ctx_->cannon = [this](bool fire, double x, double y, double z) {
        emit({{"type", "cannon"}, {"json", dock::cannonJson(fire, x, y, z)}});
      };
    // The fixed-nozzle shot. bt_runner_node publishes the same three.
    ctx_->heading_speed = [this](double heading, double speed) {
        last_hs_heading_ = heading;
        emit({{"type", "heading_speed"}, {"heading_deg", heading}, {"speed_mps", speed}});
      };
    ctx_->pump = [this](double seconds, std::uint32_t seq) {
        emit({{"type", "pump"}, {"duration_s", seconds}, {"seq", seq}, {"source", "bt_runner"}});
      };
    ctx_->set_avoidance = [this](bool on) {emit({{"type", "avoidance"}, {"enable", on}});};
    // The sticks (MANUAL). bt_runner_node maps them onto RC channels; the sim
    // does the same mapping, with the same defaults, before the bridge's gate.
    ctx_->sticks = [this](double fwd, double lat, double yaw) {
        emit({{"type", "sticks"}, {"fwd_us", fwd}, {"lat_us", lat}, {"yaw_us", yaw}});
      };
    ctx_->release_sticks = [this]() {emit({{"type", "sticks"}, {"release", true}});};
  }

  /// Motion was commanded last tick and not this one: say STOP, once. A leaf
  /// has no hook for "I stopped being ticked", and 3 s of the autopilot
  /// carrying on is 0.9 m next to a dock. Called after every tick and on exit.
  void stopIfSilent(bool exiting)
  {
    bool commanded;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      commanded = ctx_->hs_commanded && !exiting;
      ctx_->hs_commanded = false;
    }
    if (hs_active_ && !commanded && std::isfinite(last_hs_heading_)) {
      emit({{"type", "heading_speed"}, {"heading_deg", last_hs_heading_}, {"speed_mps", 0.0}});
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

  /// What the boat believes about the dock, for the sim page to draw next to
  /// the truth. Lat/lon, so the page can put it on the same map.
  void emitBook()
  {
    json out = {{"type", "book"}};
    // Built under ctx_->mu, written after it is released: out_mu_ and
    // ctx_->mu are never held together anywhere, and this is not the place to
    // start.
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!ctx_->origin_set) {return;}
      fillBook(out);
    }
    emit(out);
  }

  /// /crsd/nav/leg_status: the pacing and the JSON are context.hpp's, so this
  /// is byte for byte what bt_runner_node publishes, plus the "type" tag.
  void emitLeg(double elapsed_s)
  {
    std::string text;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!leg_pacer_.due(*ctx_, ctx_->now_s, 1.0 / kLegStatusHz)) {return;}
      text = legStatusJson(*ctx_, elapsed_s);
    }
    json j = json::parse(text);
    j["type"] = "leg";
    emit(j);
  }

  /// A new mission, or the end of one: no leg is running.
  void resetLeg(double elapsed_s)
  {
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      clearLeg(*ctx_);
    }
    emitLeg(elapsed_s);
  }

  /// How many hazards the costmap would be told about (bt_runner_node's hazard
  /// publisher, minus the message): only once the origin IS the datum.
  void emitHazards()
  {
    std::size_t n;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (!hazardsPublishable(*ctx_)) {return;}
      n = knownHazards(*ctx_).size();
    }
    emit({{"type", "hazards"}, {"n", n}});
  }

  /// CALL UNDER ctx_->mu.
  void fillBook(json & out)
  {
    const dock::DockLayout L = dock::layout(ctx_->dock, ctx_->dock_min_obs);
    const dock::Choice strict = dock::chooseSafeBay(ctx_->dock, L, ctx_->dock_votes, true);
    json tracks = json::array();
    for (const auto & t : ctx_->dock.tracks) {
      const nav::LatLon ll = nav::toLatLon(t.p, ctx_->origin);
      const nav::Vec2 o = t.outward();
      tracks.push_back({{"id", t.id}, {"lat", ll.lat}, {"lon", ll.lon}, {"n", t.n},
          {"red", t.red}, {"green", t.green}, {"ema", t.green_ema},
          {"verdict", std::string(1, dock::verdictChar(dock::verdict(t, ctx_->dock_votes)))},
          {"number", L.numberOf(t.id)}, {"out_e", o.x}, {"out_n", o.y}});
    }
    out["tracks"] = tracks;
    out["layout"] = {{"ok", L.ok}, {"why", L.why}};
    out["choice"] = {{"ok", strict.ok}, {"bay", strict.bay_number}, {"why", strict.why}};
    // Resolved: a merge may have folded the committed track into another, and
    // the page matches this against the ids in "tracks".
    out["chosen_track"] = ctx_->chosen_track < 0 ? -1 : ctx_->dock.resolve(ctx_->chosen_track);
    out["chosen_bay"] = ctx_->chosen_bay;
    if (ctx_->berth.ok) {
      const nav::LatLon p = nav::toLatLon(ctx_->berth.predock, ctx_->origin);
      const nav::LatLon b = nav::toLatLon(ctx_->berth.berth, ctx_->origin);
      out["berth"] = {{"predock_lat", p.lat}, {"predock_lon", p.lon},
        {"berth_lat", b.lat}, {"berth_lon", b.lon}};
    }
    out["phase"] = ctx_->task3_phase;
    out["pattern"] = ctx_->dock_pattern;
    json cs = json::array();
    for (auto c : ctx_->dock_colours) {cs.push_back(dock::colourName(c));}
    out["colours"] = cs;
    out["target_window"] = ctx_->dock_target_window;
    out["hits"] = ctx_->dock_hits;
    out["readiness_confirmed"] = ctx_->readiness_confirmed;
    if (ctx_->have_request) {out["request"] = ctx_->request.why;}
    out["survey_attempt"] = ctx_->survey_attempt;
  }

  // ------------------------------------------------------------ the loop

  static double secondsSince(Clock::time_point t)
  {
    return std::chrono::duration<double>(Clock::now() - t).count();
  }

  /// bt_runner_node::refreshFreshness, for the streams this runner has.
  void refreshFreshness()
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    ctx_->pose_fresh = have_pose_ && ctx_->origin_set &&
      secondsSince(pose_t_) < p_.stream_timeout_s;
    ctx_->dock_obs_age_s = have_dock_ ? secondsSince(dock_t_) : 1e9;
    ctx_->now_s = nowS();
    ctx_->att_age_s = have_att_ ? secondsSince(att_t_) : 1e9;
    ctx_->pump_age_s = have_pump_ ? secondsSince(pump_t_) : 1e9;
    if (have_status_ && secondsSince(status_t_) >= p_.stream_timeout_s) {
      ctx_->autonomous = false;        // a dead HEARTBEAT is not permission
    }
  }

  std::string treeName() const
  {
    const auto slash = p_.tree_file.find_last_of("/\\");
    std::string n = slash == std::string::npos ? p_.tree_file : p_.tree_file.substr(slash + 1);
    const auto dot = n.find_last_of('.');
    return dot == std::string::npos ? n : n.substr(0, dot);
  }

  void execute(const json & goal)
  {
    const double gt = num(goal, "timeout_s", 0.0);
    const double timeout_s = gt > 0.0 ? gt : p_.default_timeout_s;
    const auto started = Clock::now();

    refreshFreshness();
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      ctx_->tier = goal.value("tier", 0);
      ctx_->approach_lat = num(goal, "approach_latitude", 0.0);
      ctx_->approach_lon = num(goal, "approach_longitude", 0.0);
      ctx_->has_approach = ctx_->approach_lat != 0.0 || ctx_->approach_lon != 0.0;
      ctx_->have_waypoint = false;
      resetTask3(*ctx_);
      resetFire(*ctx_);
      if (p_.task3_bay > 0) {        // as bt_runner_node: the number only
        ctx_->chosen_bay = p_.task3_bay;
        RCLCPP_INFO(node_.get_logger(), "task3_bay: the docking report will name bay %d", ctx_->chosen_bay);
      }
      goal_mode_ = ctx_->mode;
      ctx_->home = ctx_->boat;
      ctx_->have_home = ctx_->pose_fresh;
    }
    leg_pacer_ = LegStatusPacer{};
    resetLeg(0.0);

    int outcome = OUTCOME_FAULT;
    std::string detail;
    try {
      BT::BehaviorTreeFactory factory;
      registerCrusaderNodes(factory);
      auto bb = BT::Blackboard::create();
      bb->set("ctx", ctx_);
      BT::Tree tree = factory.createTreeFromFile(p_.tree_file, bb);
      TreeView view(tree);

      RCLCPP_INFO(node_.get_logger(), "GOAL tier=%d timeout=%.0fs approach=(%.7f, %.7f)",
        goal.value("tier", 0), timeout_s, num(goal, "approach_latitude", 0.0),
        num(goal, "approach_longitude", 0.0));

      const auto period = std::chrono::duration<double>(1.0 / p_.tick_hz);
      auto next = Clock::now();
      int tick = 0;
      BT::NodeStatus st = BT::NodeStatus::RUNNING;
      while (!stop_) {
        if (cancel_) {
          tree.haltTree();
          outcome = OUTCOME_CANCELLED;
          detail = "cancelled by the operator";
          break;
        }
        const double elapsed = secondsSince(started);
        if (elapsed >= timeout_s) {
          tree.haltTree();
          outcome = OUTCOME_TIMEOUT;
          detail = "mission timeout " + std::to_string(static_cast<int>(timeout_s)) +
            "s expired";
          break;
        }
        if (!have_status_ && elapsed > p_.mode_grace_s) {
          tree.haltTree();
          outcome = OUTCOME_FAULT;
          detail = "no HEARTBEAT — cannot tell whether we are allowed to drive";
          break;
        }

        refreshFreshness();
        st = tree.tickOnce();
        stopIfSilent(false);
        emitLeg(elapsed);

        if (p_.verbose_tree) {
          const std::string frame = view.renderIfChanged(false);
          if (!frame.empty()) {
            RCLCPP_INFO(node_.get_logger(), "tree @ %5.1fs\n%s", elapsed, frame.c_str());
          }
        }
        emit({{"type", "bt"}, {"status", json::parse(view.json(elapsed))}});
        emit({{"type", "autonomy"}, {"active", true}});
        {
          std::string phase;
          {
            std::lock_guard<std::mutex> lk(ctx_->mu);
            phase = ctx_->task3_phase;
          }
          emit({{"type", "feedback"}, {"phase", phase},
              {"progress", std::min(1.0, elapsed / timeout_s)}, {"elapsed_s", elapsed}});
        }
        if (tick++ % 5 == 0) {
          emitBook();
          emitHazards();                   // 2 Hz at 10 Hz ticks, as the node's timer
        }

        if (st != BT::NodeStatus::RUNNING) {
          if (st == BT::NodeStatus::SUCCESS) {
            outcome = OUTCOME_SUCCESS;
            detail = "tree completed: " + treeName();
          } else {
            std::lock_guard<std::mutex> lk(ctx_->mu);
            // The pilot took it back = the MODE CHANGED during the run. A tree
            // that drives in MANUAL (task3_fire_manual.xml) is never
            // "autonomous", so "not autonomous" alone is not the pilot.
            if (ctx_->mode != goal_mode_ || (!ctx_->autonomous && goal_mode_ != "MANUAL")) {
              outcome = OUTCOME_NOT_AUTONOMOUS;
              detail = "flight mode " + (ctx_->mode != goal_mode_ ?
                goal_mode_ + " -> " + ctx_->mode : ctx_->mode + " is not autonomous") +
                " — the pilot took control; not fighting for it";
            } else {
              outcome = OUTCOME_NO_ENTRY;
              detail = "tree " + treeName() + " returned FAILURE — see the "
                "node transitions in the log for which leaf did it";
            }
          }
          break;
        }
        next += std::chrono::duration_cast<Clock::duration>(period);
        std::this_thread::sleep_until(next);
      }
      emitBook();
    } catch (const std::exception & e) {
      // As in bt_runner_node: a crash must never read as success.
      outcome = OUTCOME_FAULT;
      detail = std::string("tree raised: ") + e.what();
      RCLCPP_ERROR(node_.get_logger(), "%s", detail.c_str());
    }

    const double el = secondsSince(started);
    if (outcome == OUTCOME_SUCCESS) {
      RCLCPP_INFO(node_.get_logger(), "RESULT SUCCESS after %.1fs: %s", el, detail.c_str());
    } else {
      RCLCPP_WARN(node_.get_logger(), "RESULT %d after %.1fs: %s", outcome, el, detail.c_str());
    }
    emit({{"type", "result"}, {"outcome", outcome}, {"detail", detail}, {"elapsed_s", el}});
    // Every exit path: the cannon off, the boat stopped, avoidance back on,
    // the task stood down, the light off, no leg running.
    if (ctx_->cannon) {ctx_->cannon(false, 0.0, 0.0, 0.0);}
    stopIfSilent(true);
    resetLeg(el);
    emit({{"type", "avoidance"}, {"enable", true}});
    publishTask(kTaskNone);
    emit({{"type", "autonomy"}, {"active", false}});
  }

  Params p_;
  rclcpp::Node node_;
  ContextPtr ctx_;
  std::mutex out_mu_;

  std::mutex goal_mu_;
  std::condition_variable goal_cv_;
  json pending_;
  bool have_pending_ = false;
  bool busy_ = false;
  std::atomic<bool> cancel_{false};
  std::atomic<bool> stop_{false};

  // Arrival times, guarded by ctx_->mu like the fields they age.
  Clock::time_point pose_t_{}, status_t_{}, dock_t_{}, att_t_{}, pump_t_{};
  bool have_pose_ = false, have_status_ = false, have_dock_ = false;
  bool have_att_ = false, have_pump_ = false;
  Clock::time_point t0_{};                   // the clock ctx.now_s counts from
  LegStatusPacer leg_pacer_;                 // the mission thread only
  static constexpr double kLegStatusHz = 2.0;   // bt_runner_node's nav_status_hz default
  bool hs_active_ = false;                   // motion was commanded last tick
  bool sticks_active_ = false;               // the sticks were commanded last tick
  std::string goal_mode_;                    // the mode when this goal started
  double last_hs_heading_ = dock::kNaN;
  int uav_seq_ = 0;
};

}  // namespace offros
}  // namespace crusader_bt

int main(int argc, char ** argv)
{
  crusader_bt::offros::Runner::Params p;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char * what) -> std::string {
        if (i + 1 >= argc) {
          std::fprintf(stderr, "%s needs a value\n", what);
          std::exit(2);
        }
        return argv[++i];
      };
    if (a == "--tree") {
      p.tree_file = next("--tree");
    } else if (a == "--tick-hz") {
      p.tick_hz = std::stod(next("--tick-hz"));
    } else if (a == "--default-timeout") {
      p.default_timeout_s = std::stod(next("--default-timeout"));
    } else if (a == "--stream-timeout") {
      p.stream_timeout_s = std::stod(next("--stream-timeout"));
    } else if (a == "--publish-setpoints") {
      p.publish_setpoints = true;
    } else if (a == "--fire-pump") {
      p.fire_pump = true;
    } else if (a == "--task3-bay") {
      p.task3_bay = std::stoi(next("--task3-bay"));
      if (p.task3_bay < 0 || p.task3_bay > 3) {
        std::fprintf(stderr, "--task3-bay must be 0..3\n");
        return 2;
      }
    } else if (a == "--verbose-tree") {
      p.verbose_tree = true;
    } else if (a == "--nav-mode") {
      if (!crusader_bt::parseNavMode(next("--nav-mode"), p.nav_mode)) {
        std::fprintf(stderr, "--nav-mode must be off, shadow or on\n");
        return 2;
      }
    } else {
      std::fprintf(stderr, "unknown argument %s\n", a.c_str());
      return 2;
    }
  }
  if (p.tree_file.empty()) {
    // As bt_runner_node: no default, because a runner that silently ticks the
    // wrong tree is worse than one that will not start.
    std::fprintf(stderr, "--tree is required (e.g. crusader_bt/behavior_trees/task3_disruptive.xml)\n");
    return 2;
  }
  crusader_bt::offros::Runner r(p);
  std::thread reader([&r] {r.readLoop();});
  r.missionLoop();
  reader.join();
  return 0;
}
