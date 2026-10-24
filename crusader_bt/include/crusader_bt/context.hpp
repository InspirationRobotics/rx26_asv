// context.hpp — the one shared object every leaf reads, and the leaf base class.
//
// WHY A CONTEXT AND NOT BLACKBOARD PORTS FOR EVERYTHING. A non-trivial port type
// needs a convertFromString specialisation before it can appear as a literal in
// the XML, and a missing one is a RUNTIME error at tree-load — on the boat, at
// the dock. So world state (pose, buoys, entry/exit, the local origin) travels in
// this struct: one object, written by the runner, read by every leaf.
//
// THE LEAVES NEVER SUBSCRIBE. The runner owns every subscription and refreshes
// this struct once per tick. A leaf with its own subscription could not be
// reasoned about off-boat, would double the DDS traffic, and would see a
// different instant of the world than its siblings.
//
// LOCKING. The runner writes under `mu` from ROS callbacks; leaves read under it
// from the tick thread, which is not the callbacks' thread. Every access locks.
#ifndef CRUSADER_BT__CONTEXT_HPP_
#define CRUSADER_BT__CONTEXT_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

#include "crusader_bt/dock_math.hpp"
#include "crusader_bt/fire_math.hpp"
#include "crusader_bt/global_passage.hpp"
#include "crusader_bt/nav_math.hpp"
#include "crusader_bt/path_math.hpp"
#include "crusader_bt/planned_leg.hpp"
#include "crusader_bt/run_start.hpp"

namespace crusader_bt
{

/// World state, refreshed by the runner once per tick. Leaves write only their
/// own bookkeeping (waypoint, gates, Task 3 commitments, fire and leg state).
struct Context
{
  std::mutex mu;
  rclcpp::Node * node = nullptr;              ///< borrowed; the runner owns it

  /// The node's logger, or a generic one before the runner has set `node`.
  rclcpp::Logger logger() const
  {
    return node ? node->get_logger() : rclcpp::get_logger("crusader_bt");
  }

  // ---- fixed for the whole mission ----
  nav::LatLon origin;                         ///< local frame origin, set ONCE
  bool origin_set = false;
  double approach_lat = 0.0;
  double approach_lon = 0.0;
  bool has_approach = false;

  // ---- refreshed every tick ----
  bool autonomous = false;                    ///< mode in shared.autonomous_modes
  bool pose_fresh = false;
  nav::Vec2 boat;
  double heading_deg = std::nan("");          ///< NaN when GPS yaw is unresolved
  std::vector<nav::Buoy> buoys;               ///< FUSED: plan + tracker
  bool have_entry = false;
  bool have_exit = false;
  nav::Vec2 entry;
  nav::Vec2 exitp;                            ///< `exit` is a libc function

  // ---- the UAV's passage plan, above Core tier ----
  //
  // `buoys` above is the FUSION of this and the tracker's targets
  // (nav::fusePassage), with its beacon states from here. entry/exitp come from
  // the plan too; bt_runner_node's refuseLocked says why.
  std::vector<nav::PlanBuoy> plan;
  std::vector<nav::Buoy> obstacles;           ///< tracked, but not in the plan
  /// Seconds since the last non-empty plan arrived (PassagePlanFresh). kNoPlanAgeS = none yet,
  /// or the newest one was empty: a blank, never a back-filled age.
  static constexpr double kNoPlanAgeS = 1e9;
  double plan_age_s = kNoPlanAgeS;

  /// Gates already driven, as (red_id, green_id): by buoy ids, so a replan that reorders the
  /// field cannot skip one.
  std::vector<std::pair<int, int>> cleared_gates;

  // ---- Task 3: the docking bays ----
  //
  // `dock` is written by the runner through ingestDockObservation() below;
  // everything after it by the Task 3 leaves.
  /// RoboCommand's newest RunStart, on `now_s`'s clock. NOT reset per goal: a RunStart that
  /// arrives before the operator presses START must still count (WaitForRunStart).
  runstart::Latch run_start;
  dock::DockBook dock;                        ///< the bays, world-anchored
  dock::Mount cam_mount;                      ///< camera_link in base_link
  /// Face centre above the camera, m: places sightings from a PITCHED camera
  /// (dock::faceInBody). Copied into the book's params at every ingest.
  double dock_face_dz = 0.39;
  double dock_obs_age_s = 1e9;                ///< since the last DockObservation
  std::uint32_t dock_seq = 0;                 ///< bumps on every DockObservation
  double dock_t = 0.0;                        ///< that frame's stamp, seconds
  // The timing layer's verdict on the latest frame, for the bay it tracks.
  std::string dock_pattern;                   ///< "", steady, flash, code, ...
  std::vector<dock::Colour> dock_colours;
  int dock_target_window = -1;                ///< DockWindow.index, -1 = none
  double dock_fps = 0.0;                      ///< the timing layer's observed rate
  /// "hit" events seen so far. COUNTED, because DockObservation.last_event is
  /// set on one frame only and a leaf ticking at 10 Hz can miss that frame.
  int dock_hits = 0;

  /// How much indicator evidence makes a verdict, and how many sightings make
  /// a bay. HERE, not on leaf ports, because SafeBayKnown and CommitSafeBay
  /// must apply the SAME rule: a condition looser than its commit stalls the tree.
  dock::VoteParams dock_votes;
  int dock_min_obs = 5;

  std::string task3_phase;                    ///< for feedback and the viewers
  int survey_attempt = 0;                     ///< PickVantage calls, all of them
  int survey_looks = 0;                       ///< ... of which placed from seen bays
  int chosen_track = -1;                      ///< committed by CommitSafeBay
  int chosen_bay = 0;                         ///< its 1-based number
  dock::Berth berth;                          ///< for the chosen bay
  /// RoboCommand's ReadinessConfirm for the docking report (/crsd/ocs_command): stops the
  /// report being re-sent. The light is what the leaves wait on.
  bool readiness_confirmed = false;

  /// Where the boat was when the goal was accepted. "Go home" in a mission
  /// means "back to where this attempt started", not the autopilot's HOME —
  /// those differ whenever the boat was driven out manually first.
  nav::Vec2 home;
  bool have_home = false;

  // ---- the fixed-nozzle shot (fire_math; src/fire_leaves.cpp) ----
  //
  // Stand off a wall at the calibrated range, pointed at a window, and fire
  // when still. The runner feeds the streams through the ingest*() functions
  // below; now_s is its monotonic clock for this tick, the SAME clock the
  // samples are stamped with.
  double now_s = 0.0;
  std::string mode;                           ///< the autopilot's mode, upper case
  /// The mode this run switched to itself (RequestMode), "" = none: the goal's mode
  /// changing to it is planned, not the pilot taking over. Cleared at goal start.
  std::string handed_mode;
  /// /crsd/autonomy_drop (latched): the drop latch (ch9) has cut autonomy.
  /// Unknown counts as tripped: no message is not permission.
  bool drop_tripped = true;
  fire::WallFilter wall;                      ///< /crsd/wall_range, filtered
  fire::SteadyMonitor steady;                 ///< /crsd/attitude + our own thrust
  double att_age_s = 1e9;
  // /crsd/pump_state, as the bridge reports it
  bool pump_enabled = false;
  bool pump_on = false;
  std::uint32_t pump_last_seq = 0;
  int pump_result = 0;                        ///< PumpState.RESULT_*
  std::string pump_reason;
  double pump_age_s = 1e9;
  /// Posture, like publish_setpoints: false = FireBurst runs DRY (logs, sends
  /// nothing). Firing is G7's gate; moving is G1's. Two switches, not one.
  bool fire_pump = false;
  std::uint32_t pump_seq = 100000;            ///< the tree's burst numbering
  // written by the fire leaves
  fire::Aim aim;                              ///< StationKeep's latest solution
  fire::BurstBook bursts;
  double cmd_speed = 0.0;                     ///< the last speed StationKeep asked for
  double last_keep_t = -1.0;
  /// Set by StationKeep on every tick it commands; the runner sends STOP when it
  /// stops being set (stopIfSilent).
  bool hs_commanded = false;

  // ---- the strafe keep (MANUAL; fire_math's second half) ----
  // The camera's windows of the bay ahead, in the BODY frame, per
  // DockWindow.index (0 = upper-left, 1 = lower-right), stamped on now_s's
  // clock at receipt (ingestFireWindows). And the compass heading that squares
  // the bow to that face, from the two windows (or the face plane).
  fire::Series fire_win_x[2];
  fire::Series fire_win_y[2];
  fire::Series face_heading{true};
  std::string face_src;                       ///< "windows" / "plane": the last source
  double face_target = fire::kNaN;            ///< the last good square heading, held
  double yaw_rate_dps = fire::kNaN;           ///< ATTITUDE.yawspeed, + = turning right
  fire::StrafeState strafe_state;             ///< the keep's slew and integrators
  fire::StrafeInputs strafe_in;               ///< what the keep saw last tick, for the gate
  fire::StrafeCmd strafe;                     ///< ... and what it made of it
  std::string strafe_block;                   ///< non-empty: the keep refuses (why)
  double last_strafe_t = -1.0;
  /// Set by StrafeKeep on every tick it commands the sticks; the runner RELEASES
  /// them when it stops being set.
  bool sticks_commanded = false;

  // ---- the whole-field Task 1 plan (global_passage.hpp; src/global_leaves.cpp) ----
  //
  // The Task 1 trees (lib/common/task1_plan_drive.xml) plan the WHOLE passage at once (approach,
  // entry orbit, transit, exit orbit) and drive it phase by phase. Per mission: reset with the
  // goal, like the gates.
  struct GlobalPassage
  {
    gp::Phase phase = gp::Phase::Approach;   ///< the phase being driven, or the next one
    gp::Plan plan;                           ///< ok = false until one was made
    gp::Params params;                       ///< PlanGlobalPassage's, reused by every replan
    gp::FieldSig field;                      ///< the aircraft's report the plan was made against
    double transit_s = 0.0;                  ///< progress along plan.legs[Transit] at the last stop
    std::vector<nav::Vec2> traj;             ///< the transit as driven, first point = its start
    std::vector<nav::Vec2> entry_ring;       ///< the ring the boat orbited (or is about to)
    int plans = 0;
  };
  GlobalPassage global_passage;

  // ---- written by NextWaypoint, read by NavigateTo ----
  nav::Vec2 waypoint;
  bool have_waypoint = false;
  int waypoint_buoy_id = -1;

  // ---- posture ----
  bool publish_setpoints = false;             ///< false = this tree cannot move the boat

  // ---- obstacle avoidance (path_math.hpp, planned_leg.hpp) ----
  // Spec: docs/nav2_avoidance_spec.md sections 2 and 5.4. Written by the runner
  // (the frame through ingestPose / applyDatum below), read by NavigateTo and
  // CircleBuoy.
  path::Mode nav_mode = path::Mode::Off;
  path::NavParams nav;                          ///< bt_runner_node's nav_* params
  std::shared_ptr<path::PlannerPort> planner;   ///< null when nav_mode is off (or no Nav2 build)
  /// The latest raw fix, kept so a datum that arrives after the first pose can
  /// still place the boat in it.
  nav::LatLon fix;
  bool have_fix = false;
  /// `origin` came from /crsd/datum, i.e. it IS the TF frame `map`. A first-fix
  /// origin is not: hazards are published only when this is set (hazardsPublishable).
  bool datum_from_topic = false;
  /// A different datum arrived after one was adopted (applyDatum): every planned
  /// leg holds and the hazard publisher stops.
  bool datum_mismatch = false;

  /// What the running leg reports; the runner turns it into /crsd/nav/leg_status.
  struct LegStatus
  {
    std::string leaf, name, state = "IDLE", why;
    bool avoid = false;
    double blocked_s = 0.0;
    bool have_goal = false;
    nav::Vec2 goal;
    bool have_target = false;
    nav::Vec2 target;
    std::vector<nav::Vec2> path;
    double plan_ms = -1.0;                      ///< -1 = unknown (a blank, not a zero)
    int hop = -1, hops = 0;                     ///< CircleBuoy only; hop < 0 = not an orbit
  };
  LegStatus leg;
  std::uint32_t leg_seq = 0;                    ///< bumped by a leaf whenever it writes `leg`

  // ---- outputs the runner wires up ----
  std::function<void(nav::LatLon)> send_setpoint;
  std::function<void(const std::string &)> set_task;
  /// When current_task last became TASK_NONE (announceTask), on now_s's clock; far in the past
  /// until it has, so the first attempt of a run never waits.
  double none_since_s = -1e9;
  std::function<void(int)> consume_buoy;      ///< mark a buoy dealt with
  /// One RoboCommand report: (RxReport oneof name, that message as protobuf JSON), to
  /// /crsd/robocommand/report for ocs_client.
  std::function<void(const std::string &, const std::string &)> report;

  // ---- Task 3 outputs ----
  /// The water cannon: fire or not, aimed at a point in camera_link.
  std::function<void(bool, double, double, double)> cannon;

  // ---- fixed-nozzle shot outputs ----
  /// GUIDED heading (compass deg) + signed speed (m/s): /crsd/guided_heading_speed.
  std::function<void(double, double)> heading_speed;
  /// A pump burst of `seconds` (0 = OFF) with sequence `seq`: /crsd/pump_cmd.
  std::function<void(double, std::uint32_t)> pump;
  /// The autopilot's simple avoidance on/off: /crsd/avoidance_enable.
  std::function<void(bool)> set_avoidance;
  /// The sticks (MANUAL, RC override): deflections in us about neutral, as
  /// fire::Sticks - fwd + ahead, lat + starboard, yaw + right. /crsd/rc_override.
  std::function<void(double, double, double)> sticks;
  /// Hand every channel back to the pilot (an all-zero override).
  std::function<void()> release_sticks;
  /// Ask the autopilot for a flight mode: /crsd/set_mode (RequestMode).
  std::function<void(const std::string &)> request_mode;
};

using ContextPtr = std::shared_ptr<Context>;

// ------------------------------------------------------------ the shared frame
//
// Spec section 2. The TF frame `map`, ctx.origin and every HazardArray
// coordinate are ONE local ENU plane, centred on one datum that nav_frames_node
// owns and publishes on /crsd/datum. BOTH runners learn it only through these
// two functions: the rule lives here, not in two copies that drift.

/// What one /crsd/pose did to the frame.
enum class PoseEvent
{
  Tracked,   ///< the origin was already set; the boat moved
  Pinned,    ///< nav_mode off: this first fix became the origin (the legacy behaviour)
  Waiting,   ///< nav_mode shadow/on and no datum yet: the boat is NOT placed
  Bad        ///< a non-finite lat/lon: ignored entirely
};

/// One /crsd/pose. CALL UNDER ctx.mu.
///
/// Without a datum, nav_mode off pins the origin at the first fix. Shadow and on wait for
/// /crsd/datum: a first-fix origin is not the costmap's frame, a silent offset.
inline PoseEvent ingestPose(Context & c, nav::LatLon fix, double heading_deg)
{
  if (!std::isfinite(fix.lat) || !std::isfinite(fix.lon)) {return PoseEvent::Bad;}
  c.fix = fix;
  c.have_fix = true;
  c.heading_deg = heading_deg;
  PoseEvent ev = PoseEvent::Tracked;
  if (!c.origin_set) {
    if (c.nav_mode != path::Mode::Off) {return PoseEvent::Waiting;}
    c.origin = fix;
    c.origin_set = true;
    ev = PoseEvent::Pinned;
  }
  c.boat = nav::toLocal(fix, c.origin);
  return ev;
}

/// What a /crsd/datum did to the frame.
enum class DatumEvent
{
  Adopted,    ///< it is now the origin (it came before any pose pinned one)
  Unchanged,  ///< the same datum as the origin, to 1 cm
  Mismatch,   ///< a DIFFERENT datum, the first time: datum_mismatch is now set
  Ignored     ///< non-finite, a mismatch already reported, or off with a first-fix pin
};

/// A /crsd/datum arrived. CALL UNDER ctx.mu.
///
/// Adopted whenever no origin is set yet, in every nav_mode. Never re-pins mid-run: a datum
/// more than 0.01 m from the origin in use sets datum_mismatch (the dock book and `home` hold
/// local coordinates). In off a first-fix origin shares no costmap frame, so a later datum is
/// ignored there rather than reported as a mismatch.
inline DatumEvent applyDatum(Context & c, nav::LatLon datum)
{
  if (!std::isfinite(datum.lat) || !std::isfinite(datum.lon)) {return DatumEvent::Ignored;}
  if (!c.origin_set) {
    c.origin = datum;
    c.origin_set = true;
    c.datum_from_topic = true;
    // A pose that arrived first was held back (ingestPose, Waiting): place it now.
    if (c.have_fix) {c.boat = nav::toLocal(c.fix, c.origin);}
    return DatumEvent::Adopted;
  }
  if (nav::norm(nav::toLocal(datum, c.origin)) <= 0.01) {
    c.datum_from_topic = true;          // a first-fix pin that equals the datum IS the frame
    return DatumEvent::Unchanged;
  }
  if (c.datum_mismatch || (c.nav_mode == path::Mode::Off && !c.datum_from_topic)) {
    return DatumEvent::Ignored;
  }
  c.datum_mismatch = true;
  return DatumEvent::Mismatch;
}

/// The costmap's frame is the BT's frame only when the origin came from the datum.
/// CALL UNDER ctx.mu.
inline bool hazardsPublishable(const Context & c)
{
  return c.origin_set && c.datum_from_topic && !c.datum_mismatch;
}

/// Everything the BT knows to be in the water: plan buoys, unmatched confirmed
/// tracks and the dock. CALL UNDER ctx.mu.
///
/// THE ONE FUNCTION both the hazard publisher and the leaves call, so the costmap
/// and the BT's own local checks can never disagree about the known field.
///
inline std::vector<path::Hazard> knownHazards(const Context & c)
{
  return path::buildHazards(c.buoys, c.obstacles, c.dock, c.dock_min_obs, c.nav);
}

// ------------------------------------------------------------------ nav_mode

/// "off" | "shadow" | "on". False for anything else, so a typo in the YAML is an
/// error and never a silent fall-back to a mode the operator did not ask for.
inline bool parseNavMode(const std::string & s, path::Mode & out)
{
  if (s == "off") {
    out = path::Mode::Off;
  } else if (s == "shadow") {
    out = path::Mode::Shadow;
  } else if (s == "on") {
    out = path::Mode::On;
  } else {
    return false;
  }
  return true;
}

inline const char * navModeName(path::Mode m)
{
  switch (m) {
    case path::Mode::Off: return "off";
    case path::Mode::Shadow: return "shadow";
    case path::Mode::On: return "on";
  }
  return "off";
}

// ----------------------------------------------------------------- leg status
//
// /crsd/nav/leg_status (spec 4.3) is JSON, built here once so the ROS runner and
// the off-ROS one send byte-identical text. A blank is `null`, never a number
// that looks like a measurement.

inline std::string jsonQuote(const std::string & s)
{
  std::string o = "\"";
  for (const unsigned char ch : s) {
    switch (ch) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default:
        if (ch < 0x20) {
          char b[8];
          std::snprintf(b, sizeof(b), "\\u%04x", static_cast<unsigned>(ch));
          o += b;
        } else {
          o += static_cast<char>(ch);
        }
    }
  }
  return o + "\"";
}

/// A JSON number with `digits` decimals; `null` for NaN/inf (NaN is not JSON).
inline std::string jsonNum(double v, int digits)
{
  if (!std::isfinite(v)) {return "null";}
  char b[48];
  std::snprintf(b, sizeof(b), "%.*f", digits, v);
  return b;
}

/// `[lat, lon]` of a local point, or `null` when there is none (or no origin).
inline std::string jsonLatLon(const Context & c, bool have, nav::Vec2 p)
{
  if (!have || !c.origin_set) {return "null";}
  const nav::LatLon ll = nav::toLatLon(p, c.origin);
  return "[" + jsonNum(ll.lat, 7) + "," + jsonNum(ll.lon, 7) + "]";
}

/// RxReport's beacon names for a buoy state.
inline const char * beaconStateName(nav::Beacon b)
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

/// `{"latitude":..,"longitude":..}` of a local point. CALL UNDER ctx.mu, with origin_set.
inline std::string jsonPosition(const Context & c, nav::Vec2 p)
{
  const nav::LatLon ll = nav::toLatLon(p, c.origin);
  return "{\"latitude\":" + jsonNum(ll.lat, 7) + ",\"longitude\":" + jsonNum(ll.lon, 7) + "}";
}

/// SafePassageReport as protobuf JSON: every fused buoy, and ENTRY / EXIT when known. A
/// position we do not have is omitted, never sent as (0, 0). CALL UNDER ctx.mu, with origin_set.
inline std::string safePassageReportJson(const Context & c)
{
  std::string o = "{\"buoys\":[";
  for (std::size_t i = 0; i < c.buoys.size(); ++i) {
    o += (i ? "," : "") + std::string("{\"position\":") + jsonPosition(c, c.buoys[i].p) +
      ",\"state\":\"" + beaconStateName(c.buoys[i].state) + "\"}";
  }
  o += "]";
  if (c.have_entry) {o += ",\"entry_position\":" + jsonPosition(c, c.entry);}
  if (c.have_exit) {o += ",\"exit_position\":" + jsonPosition(c, c.exitp);}
  return o + "}";
}

/// Send one RoboCommand report through the runner. TAKES NO LOCK.
inline void sendReport(Context & c, const std::string & kind, const std::string & body_json)
{
  if (c.report) {c.report(kind, body_json);}
}

/// The leg status as one JSON object, `path` decimated to at most `max_path`
/// points (first and last kept). CALL UNDER ctx.mu. `t_s` is the mission clock.
inline std::string legStatusJson(const Context & c, double t_s, std::size_t max_path = 60)
{
  const Context::LegStatus & s = c.leg;
  std::string o = "{\"t\":" + jsonNum(t_s, 1);
  if (s.state == "IDLE") {return o + ",\"state\":\"IDLE\"}";}   // no leg running
  o += ",\"leaf\":" + jsonQuote(s.leaf) + ",\"name\":" + jsonQuote(s.name) +
    ",\"mode\":\"" + navModeName(c.nav_mode) + "\",\"avoid\":" + (s.avoid ? "true" : "false") +
    ",\"state\":" + jsonQuote(s.state) + ",\"why\":" + jsonQuote(s.why) +
    ",\"blocked_s\":" + jsonNum(s.blocked_s, 1) +
    ",\"goal\":" + jsonLatLon(c, s.have_goal, s.goal) +
    ",\"target\":" + jsonLatLon(c, s.have_target, s.target) + ",\"path\":[";
  const std::size_t n = s.path.size();
  const std::size_t m = std::min(n, std::max<std::size_t>(max_path, 2));
  for (std::size_t k = 0; k < m; ++k) {
    const std::size_t i = m == 1 ? 0 : k * (n - 1) / (m - 1);
    o += (k == 0 ? "" : ",") + jsonLatLon(c, true, s.path[i]);
  }
  o += "],\"plan_ms\":" + (s.plan_ms >= 0.0 ? jsonNum(s.plan_ms, 1) : std::string("null"));
  if (s.hop >= 0) {
    o += ",\"hop\":" + std::to_string(s.hop) + ",\"hops\":" + std::to_string(s.hops);
  }
  return o + "}";
}

/// Decides WHEN the leg status goes out: when the leg's state changes (always), and
/// otherwise at most every `period_s` after a leaf wrote it. A leg that ended
/// (ARRIVED or FAILED) is shown once; if no leaf has started another by the next
/// tick the status goes back to IDLE, so the map never keeps drawing a finished
/// leg's path (blanks over guesses). One per runner, on the tick thread.
struct LegStatusPacer
{
  std::uint32_t seq = 0;
  std::string state;                   ///< "" until the first publish, so the first call is due
  double sent_s = -1e18;
  bool terminal = false;

  /// CALL UNDER ctx.mu. True = publish legStatusJson(c, ...) now.
  bool due(Context & c, double now_s, double period_s)
  {
    if (terminal && c.leg_seq == seq) {
      c.leg = Context::LegStatus{};
      ++c.leg_seq;
    }
    const bool state_changed = c.leg.state != state;
    const bool written = c.leg_seq != seq;
    if (!state_changed && !(written && now_s - sent_s >= period_s)) {return false;}
    seq = c.leg_seq;
    state = c.leg.state;
    sent_s = now_s;
    terminal = state == "ARRIVED" || state == "FAILED";
    return true;
  }
};

/// Send `p` as the GUIDED setpoint when publish_setpoints allows it, and log
/// "<what> lat, lon", marked [NOT SENT] when it did not go out. `what` empty = no log line.
/// The local point is converted under the lock; the publish is not. TAKES ctx.mu ITSELF.
inline void sendSetpoint(Context & c, const rclcpp::Logger & lg, nav::Vec2 p, const std::string & what)
{
  nav::LatLon ll;
  {
    std::lock_guard<std::mutex> lk(c.mu);
    ll = nav::toLatLon(p, c.origin);
  }
  const bool sent = c.publish_setpoints && c.send_setpoint;
  if (sent) {c.send_setpoint(ll);}
  if (!what.empty()) {
    RCLCPP_INFO(
      lg, "%s %.7f, %.7f%s", what.c_str(), ll.lat, ll.lon,
      sent ? "" : "  [NOT SENT: publish_setpoints is false]");
  }
}

/// Back to "no leg running". CALL UNDER ctx.mu.
inline void clearLeg(Context & c)
{
  c.leg = Context::LegStatus{};
  ++c.leg_seq;
}

/// Hold station: send the boat's own position as the setpoint. False, and nothing sent, without
/// a fresh pose. TAKES ctx.mu ITSELF. HoldStation, WaitForRunStart and the whole-field holds.
inline bool sendHold(Context & c, const rclcpp::Logger & lg, const std::string & what)
{
  nav::Vec2 here;
  {
    std::lock_guard<std::mutex> lk(c.mu);
    if (!c.pose_fresh) {return false;}
    here = c.boat;
  }
  sendSetpoint(c, lg, here, what);
  return true;
}

/// One /crsd/ocs_command message. True when it was RoboCommand's RunStart: latched with `now_s`
/// (Context::now_s's clock), `*run_id` its id. Both runners call it, so they cannot disagree
/// about what a RunStart is. CALL UNDER ctx.mu.
inline bool ingestRunStart(Context & c, double now_s, const std::string & json, int * run_id)
{
  if (!runstart::parse(json, run_id)) {return false;}
  c.run_start.set(now_s, *run_id);
  return true;
}

/// Announce current_task (handbook 3.4.11) and log it. TASK_NONE is stamped, so TaskAttempt can
/// hold it long enough for the 2 Hz heartbeat to carry it. TAKES ctx.mu ITSELF.
inline void announceTask(Context & c, const std::string & token)
{
  if (c.set_task) {c.set_task(token);}
  if (token == "TASK_NONE") {
    std::lock_guard<std::mutex> lk(c.mu);
    c.none_since_s = c.now_s;
  }
  RCLCPP_INFO(c.logger(), "current_task -> %s", token.c_str());
}

/// Fold one DockObservation into the context. CALL UNDER ctx.mu.
///
/// Both runners convert their input (the ROS message, or the sim's JSON line)
/// into a dock::Frame and hand it here, so neither decides what a frame MEANS.
///
/// The bays are placed only with a fresh pose and a resolved heading - a
/// sighting placed from a stale pose is a bay in the wrong place, forever. The
/// timing layer's verdict does not depend on the pose, so it is always taken.
inline void ingestDockObservation(Context & c, const dock::Frame & f)
{
  if (c.origin_set && c.pose_fresh && std::isfinite(c.heading_deg)) {
    c.dock.prm.face_dz = c.dock_face_dz;
    c.dock.ingest(f, c.boat, c.heading_deg, c.cam_mount);
  }
  ++c.dock_seq;
  c.dock_t = f.t;
  c.dock_pattern = f.target_pattern;
  c.dock_colours = f.target_colours;
  c.dock_target_window = f.target_window_index;
  c.dock_fps = f.observed_fps;
  if (f.last_event == "hit") {++c.dock_hits;}
}

/// One /crsd/wall_range message. CALL UNDER ctx.mu. Stamped with the heading
/// at receipt, so the wall's compass bearing survives the boat turning.
inline void ingestWallRange(
  Context & c, double t, bool valid, double range_m, double angle_deg, double lat_m)
{
  fire::WallSample s;
  s.t = t;
  s.valid = valid && std::isfinite(range_m);
  s.range_m = range_m;
  s.angle_deg = angle_deg;
  s.lat_m = lat_m;
  s.heading_deg = c.heading_deg;
  c.wall.add(s);
}

/// One /crsd/attitude message (radians, the autopilot's axes). CALL UNDER ctx.mu.
/// Only magnitudes and swings matter to "steady", so the axes' signs do not.
inline void ingestAttitude(
  Context & c, double t, double roll, double pitch, double roll_rate, double pitch_rate,
  double yaw_rate = fire::kNaN)
{
  constexpr double d = 180.0 / fire::kPi;
  c.steady.feed_att(t, roll * d, pitch * d, roll_rate * d, pitch_rate * d);
  c.yaw_rate_dps = yaw_rate * d;     // NED yaw rate: + = clockwise = turning right
}

/// The strafe keep's view of one DockObservation. CALL UNDER ctx.mu, after
/// ingestDockObservation, with `t` on now_s's clock (the frame's own stamp is
/// another clock). Takes the bay nearest the bow with a positioned window, puts
/// its windows into the body frame, and derives the heading that squares the bow
/// to it: from both windows at the face's spacing, else the face plane's normal.
inline void ingestFireWindows(Context & c, double t, const dock::Frame & f)
{
  constexpr double kFaceSepM = 0.45, kFaceSepTolM = 0.15;   // UL-LR, build guide
  const dock::BaySighting * best = nullptr;
  for (const auto & b : f.bays) {
    bool any = false;
    for (const auto & w : b.windows) {any = any || (w.has_position && w.index >= 0 && w.index <= 1);}
    if (!any) {continue;}
    if (best == nullptr || std::fabs(b.bearing_deg) < std::fabs(best->bearing_deg)) {best = &b;}
  }
  if (best == nullptr) {return;}
  const auto & m = c.cam_mount;
  fire::P3 body[2];
  bool have[2] = {false, false};
  for (const auto & w : best->windows) {
    if (!w.has_position || w.index < 0 || w.index > 1) {continue;}
    body[w.index] = fire::camToBody(fire::P3{w.x, w.y, w.z}, m.x, m.y, m.yaw_deg, m.pitch_deg);
    have[w.index] = true;
    c.fire_win_x[w.index].add(t, body[w.index].x);
    c.fire_win_y[w.index].add(t, body[w.index].y);
  }
  if (!std::isfinite(c.heading_deg)) {return;}
  double left = fire::kNaN;
  if (have[0] && have[1]) {
    left = fire::squareFromWindows(body[0].x, body[0].y, body[1].x, body[1].y,
        kFaceSepM, kFaceSepTolM);
    if (std::isfinite(left)) {c.face_src = "windows";}
  }
  if (!std::isfinite(left) && best->has_normal) {
    const fire::P3 n = fire::camToBody(fire::P3{best->nx, best->ny, best->nz}, 0, 0,
        m.yaw_deg, m.pitch_deg, false);
    left = fire::squareFromNormal(n.x, n.y);
    if (std::isfinite(left)) {c.face_src = "plane";}
  }
  if (std::isfinite(left)) {c.face_heading.add(t, fire::wrap360(c.heading_deg - left));}
}

/// Forget the last attempt's shots. Called at goal start with resetTask3.
inline void resetFire(Context & c)
{
  c.aim = fire::Aim{};
  c.bursts.reset();
  c.cmd_speed = 0.0;
  c.last_keep_t = -1.0;
  c.hs_commanded = false;
  c.strafe_state = fire::StrafeState{};
  c.strafe = fire::StrafeCmd{};
  c.strafe_in = fire::StrafeInputs{};
  c.strafe_block.clear();
  c.last_strafe_t = -1.0;
  c.face_target = fire::kNaN;
  c.sticks_commanded = false;
}

/// Forget everything Task 3 learned. Called at goal start: a second attempt
/// must not inherit the first one's bays, votes or commitments.
inline void resetTask3(Context & c)
{
  c.dock = dock::DockBook{};
  c.dock_hits = 0;
  c.task3_phase.clear();
  c.survey_attempt = 0;
  c.survey_looks = 0;
  c.chosen_track = -1;
  c.chosen_bay = 0;
  c.berth = dock::Berth{};
  c.readiness_confirmed = false;
}

/// A place to go, passed BETWEEN LEAVES on the blackboard: a compute leaf writes
/// one to an output port, an action leaf reads it from an input port, and the
/// XML shows the wire:
///
///     <NextWaypoint buoys="{buoys}" out="{goal}"/>
///     <NavigateTo   goal="{goal}"/>
///
/// NO convertFromString SPECIALISATION IS NEEDED: one is only required to parse
/// a LITERAL out of the XML (goal="1.28;103.85"). A value that only travels
/// between leaves as {ref} is moved as the real type and never touches a string.
///
/// `why` lets a log line, feedback or a viewer say WHICH buoy this is and why we
/// steer there; a bare lat/lon says nothing about what the boat was thinking.
struct Waypoint
{
  double lat = 0.0;
  double lon = 0.0;
  std::string why;
};

/// Fetch the context from the blackboard, or throw with a message that names
/// the cause. A leaf constructed without it is a wiring bug in the runner, and
/// failing loudly at tree-load beats a null dereference on the water.
inline ContextPtr contextFrom(const BT::NodeConfig & cfg, const std::string & who)
{
  ContextPtr ctx;
  if (!cfg.blackboard || !cfg.blackboard->get("ctx", ctx) || !ctx) {
    throw BT::RuntimeError(
      who + ": no 'ctx' on the blackboard. "
      "If this node sits inside a <SubTree>, that is the cause: a subtree gets "
      "its OWN blackboard and does not inherit the parent's entries. Add "
      "_autoremap=\"true\" to the SubTree tag, e.g. "
      "<SubTree ID=\"Whatever\" _autoremap=\"true\"/>. "
      "Verified against BT.CPP 4.9 on 2026-09-07: without it every leaf in the "
      "subtree throws this at tree-load; with it the subtree runs. "
      "Otherwise the runner failed to set it before creating the tree.");
  }
  return ctx;
}

/// Base for leaves that take more than one tick (publish, then poll).
class CrusaderAction : public BT::StatefulActionNode
{
public:
  CrusaderAction(const std::string & name, const BT::NodeConfig & cfg)
  : BT::StatefulActionNode(name, cfg), ctx_(contextFrom(cfg, name)) {}

  /// Default: nothing to undo. Leaves that latch state override this.
  void onHalted() override {}

protected:
  ContextPtr ctx_;

  rclcpp::Logger log() const {return ctx_->logger();}
};

/// Base for a leaf that DOES something but finishes within one tick.
///
/// Not decoration: BT.CPP's SyncActionNode::executeTick THROWS if the leaf
/// returns RUNNING, the right guard for an instantaneous leaf, and the NodeType
/// differs from CrusaderCondition's, so Groot draws a thing that publishes
/// differently from a thing that answers a question.
///
/// If it publishes, writes a port, or changes anything: this one.
/// If it only reads state and answers yes/no: CrusaderCondition.
class CrusaderSyncAction : public BT::SyncActionNode
{
public:
  CrusaderSyncAction(const std::string & name, const BT::NodeConfig & cfg)
  : BT::SyncActionNode(name, cfg), ctx_(contextFrom(cfg, name)) {}

protected:
  ContextPtr ctx_;

  rclcpp::Logger log() const {return ctx_->logger();}
};

/// Base for leaves that answer in one tick and have NO side effects.
class CrusaderCondition : public BT::ConditionNode
{
public:
  CrusaderCondition(const std::string & name, const BT::NodeConfig & cfg)
  : BT::ConditionNode(name, cfg), ctx_(contextFrom(cfg, name)) {}

protected:
  ContextPtr ctx_;

  rclcpp::Logger log() const {return ctx_->logger();}
};

/// Register every Crusader leaf with `factory`.
///
/// Registered DIRECTLY, not as plugins via registerFromPlugin(). Plugins buy
/// hot-swapping without relinking, which we will never do mid-competition, and
/// cost a shared-library search path that is wrong in the container and fails at
/// tree-load with a message about a missing .so rather than about the tree.
void registerCrusaderNodes(BT::BehaviorTreeFactory & factory);

/// The Task 3 leaves (src/task3_leaves.cpp). This and the two below are called
/// by registerCrusaderNodes, so a runner registers everything with one call.
void registerTask3Nodes(BT::BehaviorTreeFactory & factory);
/// The fixed-nozzle shot's leaves (src/fire_leaves.cpp).
void registerFireNodes(BT::BehaviorTreeFactory & factory);
/// The whole-field Task 1 leaves (src/global_leaves.cpp, lib/common/task1_plan_drive.xml).
void registerGlobalPassageNodes(BT::BehaviorTreeFactory & factory);

}  // namespace crusader_bt

#endif  // CRUSADER_BT__CONTEXT_HPP_
