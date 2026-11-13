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
#include <cstdlib>
#include <cstdio>
#include <functional>
#include <set>
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
#include "crusader_bt/t3_tuning.hpp"
#include "crusader_bt/task4_math.hpp"

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
  /// The runner's RunStart gate opened for this goal (set under mu by execute(), reset at every
  /// goal start). WaitForRunStart then succeeds at once: an old saved mission that still
  /// carries the leaf must not wait a second time for a RunStart the gate already took.
  bool run_released = false;
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
  /// The colour rule CommitFirstGreen judged the bays by on its last tick (its ports), so
  /// /crsd/bt_status records each bay's GREEN confidence and verdict as the tree saw them.
  dock::ColourRule colour_rule;
  int dock_min_obs = 5;
  /// The bow's heading facing the dock, deg true: bt_runner_node dock_heading_deg (the
  /// GCS sets it), read at goal start. < 0 = not set.
  double dock_heading = -1.0;
  /// RoboCommand's course (its RxCourse corners, lat/lon), as the OCS relays it on
  /// /crsd/ocs_course; empty = no course. The dock book drops any face placed outside it
  /// (placeCourse). course_seq counts the courses heard, so a new one is placed afresh.
  std::vector<nav::LatLon> course_corners;
  std::string course_id;
  int course_seq = 0;
  /// Which way StrafeSearch slides first: bt_runner_node search_first (the GCS sets it),
  /// read at goal start. "left" | "right", "" = not set.
  std::string search_first;
  /// Which way StrafeSearch is sliding now (facing the bays): -1 left, +1 right, 0 no search
  /// this run. The bays it has slid past are the ones it has seen, which is what numbers a
  /// bay committed before all three are known (dock::guessBayNumber).
  int search_dir = 0;
  /// A bay CommitFirstGreen has worked out is ours (the others RED, or the dock's end) while
  /// the camera does not see it: StrafeSearch slides toward it. -1 = none.
  int search_target_track = -1;
  /// StrafeSearch legs run to their end (it turned back): the slide has passed the dock.
  int search_leg_ends = 0;

  std::string task3_phase;                    ///< for feedback and the viewers
  int survey_attempt = 0;                     ///< PickVantage calls, all of them
  int survey_looks = 0;                       ///< ... of which placed from seen bays
  int chosen_track = -1;                      ///< committed by CommitSafeBay
  int chosen_bay = 0;                         ///< its 1-based number
  dock::Berth berth;                          ///< for the chosen bay
  /// RoboCommand's ReadinessConfirm for the docking report (/crsd/ocs_command): stops the
  /// report being re-sent. The light is what the leaves wait on.
  bool readiness_confirmed = false;
  /// now_s when the docking report last went out; < 0 = not yet this run. AwaitFireTarget
  /// re-sends from it, so a fire branch running beside the docking never reports early.
  double docking_report_s = -1.0;
  /// Fires put out this run (SprayUntilHit SUCCESS): after one, a RED must hold longer
  /// than the resource code's 1 s flashes before it is another fire.
  int fires_out = 0;
  /// The GREEN bay, straight off the newest frame that showed one (pickGreen): Task 3 has
  /// one GREEN indicator, so no track and no identity - the boat aligns and the cannon aims
  /// on it wherever the frame puts it.
  struct GreenBay
  {
    double t = -1.0;                          ///< that frame's stamp (dock_t's clock); < 0 never
    dock::FrameBay bay;
    std::vector<dock::WindowSighting> windows;  ///< from its newest frame that had windows
    double windows_t = -1.0;
  } green;
  /// CommitFirstGreen colour="any" (task3_first_dock): `green` is then the bay committed
  /// to, whatever its indicator reads, not the GREEN one (pickGreen).
  bool target_any = false;
  /// The centring spot (our bay's face; never its indicator), kept on the map by a Kalman
  /// filter from every frame that showed it: one memory for ApproachBay and StrafeKeep, so
  /// the hand-over does not start the hold blind, and a few lost frames are ridden through.
  fire::CentreTarget centre;

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
  /// /crsd/autonomy_drop (latched): the drop latch (SC down, ch8) has cut autonomy.
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
  double cmd_speed = 0.0;                     ///< the last speed StationKeep asked for
  double last_keep_t = -1.0;
  /// Set by StationKeep on every tick it commands; the runner sends STOP when it
  /// stops being set (stopIfSilent).
  bool hs_commanded = false;

  // ---- the docked hold (MANUAL, StrafeKeep): the committed bay, on the sticks ----
  double yaw_rate_dps = fire::kNaN;           ///< ATTITUDE.yawspeed, + = turning right
  fire::StrafeState strafe_state;             ///< the keep's slew and integrators
  fire::StrafeInputs strafe_in;               ///< what the keep saw last tick, for the gate
  fire::StrafeCmd strafe;                     ///< ... and what it made of it
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

  // ---- Task 1 Core: where the ENTRY/EXIT in use came from (bt_runner_node refuseLocked) ----
  /// SEEN by the boat this run, or GIVEN (the GCS MANUAL field): never kept from an earlier run
  /// (the competition re-deals the light beacons every run), not guessed. The Core scan (ScanForBuoys, AnchorSeen) looks until it has these.
  bool entry_seen = false;
  bool exit_seen = false;
  /// The EXIT in use is a GUESS: the operator's field centre mirrored through the ENTRY. Good
  /// enough to plan the approach and the transit; replaced the moment the boat sees the EXIT.
  bool exit_guessed = false;
  /// This mission lets the runner GUESS the EXIT (the root's t1_guess_exit="true", read at Start;
  /// only the Core scan trees set it). Off: guessExit does nothing and exit_guessed stays false.
  bool guess_exit = false;
  /// Ids of the unnamed buoys (no colour yet) DrivePlanPhase has already stopped to look at
  /// (pause_unknown_m): one look per buoy per run, across phase restarts, Retry re-plans and a
  /// Task 4 resume. Cleared at goal start. Under mu.
  std::set<int> looked_unnamed;

  // ---- the operator's mission inputs: the GCS Run panel -> bt_runner_node, read at Start ----
  struct Inputs
  {
    bool field_set = false;          ///< t1_field_lat/lon are set (0, 0 = not)
    nav::LatLon field_ll;            ///< the Task 1 buoy field's approximate centre
    double scan_dwell_s = 5.0;       ///< t1_scan_dwell_s: each look of a scan, each mid-field pause
    double pause_every_m = 5.0;      ///< t1_pause_every_m: a look every this far along the transit
    double backoff_m = 4.0;          ///< t3_backoff_m: the docked hold once the fire is out
    bool via_set = false;            ///< t3_via_lat/lon are set (0, 0 = not)
    nav::LatLon via_ll;              ///< a point the boat drives through from Task 1 to Task 3
    bool approach_set = false;       ///< t1_approach_lat/lon are set (0, 0 = not)
    nav::LatLon approach_ll;         ///< where Task 1 drives to before it scans for the field
    double unknown_range_m = 12.0;   ///< t1_unknown_range_m: stop to look at a colourless buoy this near (0 = never)
    t3tune::Values t3 = t3tune::defaults();   ///< Task 3 control, t3_tuning.hpp's knobs, clamped
  };
  Inputs inputs;

  // ---- Task 4 Disruptive: RoboCommand's moving objects (task4_math.hpp, task4_leaves.cpp) ----
  //
  // The runner folds every MovingObjectAlert in (any time, a mission running or not) and
  // re-reads the settings every second; knownHazards() carries the objects' swept keep-outs;
  // the AvoidInvaders decorator steps the boat aside and writes what it is doing below.
  struct Task4
  {
    t4::Settings settings;           ///< bt_runner_node t4_*
    std::vector<t4::Invader> objects;
    int next_id = 1;
    int alerts = 0;                  ///< MovingObjectAlerts received since the node started
    double last_alert_s = -1.0;      ///< now_s of the newest; < 0 = none yet
    double last_seq = 0.0;           ///< RxCommand.seq of the newest alert applied; 0 = none (a run start, run end or All Clear resets it)
    std::string state;               ///< AvoidInvaders: CLEAR EVADING WAITING RETURNING RESTORING
    std::string why;
    double state_s = -1.0;           ///< now_s when the guard last wrote; < 0 = never this run
    bool have_target = false;        ///< where the guard is driving (escape or return point)
    nav::Vec2 target;
    t4::Assessment last;             ///< the guard's last verdict
    int evasions = 0;                ///< this run
  };
  Task4 task4;

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
  /// Ask camera_node for a model (its live `model` parameter, SetCameraModel) and, when
  /// `indicator` is not "", how the dock model reads the indicator (its live
  /// `dock_indicator`: yolo | cv). False, with the reason in `why`, when it cannot be asked;
  /// the answer lands in camera_model_ack.
  std::function<bool(const std::string & model, const std::string & indicator,
    std::string & why)> set_camera_model;
  int camera_model_ack = 0;                   ///< the last ask: 0 waiting, 1 applied, -1 refused
  std::string camera_model_why;               ///< camera_node's refusal, then the leaf's FAILURE
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
  std::vector<path::Hazard> hz =
    path::buildHazards(c.buoys, c.obstacles, c.dock, c.dock_min_obs, c.nav);
  // Task 4: every moving object's keep-out, swept along its projected travel (SRC_KEEPOUT)
  const std::vector<path::Hazard> ko = t4::hazards(c.task4.objects, c.task4.settings, c.now_s);
  hz.insert(hz.end(), ko.begin(), ko.end());
  return hz;
}

/// The operator's Task 1 field centre in the local frame. False without one (or no origin).
/// CALL UNDER ctx.mu.
inline bool fieldCentre(const Context & c, nav::Vec2 * p)
{
  if (!c.inputs.field_set || !c.origin_set) {return false;}
  *p = nav::toLocal(c.inputs.field_ll, c.origin);
  return true;
}

/// Hazards from Task 4's moving objects only. CALL UNDER ctx.mu.
inline std::vector<path::Hazard> invaderHazards(const Context & c)
{
  return t4::hazards(c.task4.objects, c.task4.settings, c.now_s);
}

/// A Task 4 keep-out is within near_m of `path` (a point or polyline). A leg blocked by one is
/// WAITING for a moving object to pass, which must not time the leg out. CALL UNDER ctx.mu.
inline bool invaderNear(const Context & c, const std::vector<nav::Vec2> & path, double near_m)
{
  const std::vector<path::Hazard> ko = invaderHazards(c);
  if (ko.empty() || path.empty()) {return false;}
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (path::minClearance(ko, path[i]) < near_m) {return true;}
    if (i + 1 < path.size()) {
      const nav::Vec2 a = path[i], b = path[i + 1];
      const double len = nav::norm(b - a);
      const int n = static_cast<int>(len / 0.5);
      for (int k = 1; k < n; ++k) {
        if (path::minClearance(ko, a + (b - a) * (static_cast<double>(k) / n)) < near_m) {
          return true;
        }
      }
    }
  }
  return false;
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
/// A position the operator typed: finite, on the globe, and not the 0, 0 a blank GCS field leaves.
inline bool latLonGiven(double lat, double lon)
{
  return std::isfinite(lat) && std::isfinite(lon) && std::fabs(lat) <= 90.0 &&
         std::fabs(lon) <= 180.0 && !(lat == 0.0 && lon == 0.0);
}

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
  // a GUESSED EXIT (the field centre mirrored through the ENTRY) steers the plan, never the report
  if (c.have_exit && !c.exit_guessed) {o += ",\"exit_position\":" + jsonPosition(c, c.exitp);}
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

/// The same message as an `ocs_directive`: `standby_auto` (a new declaration) or `run_end`
/// clears the latch, because a RunStart older than either belonged to a run that is over.
/// Returns what it did (Directive::None = left the latch alone). CALL UNDER ctx.mu.
inline runstart::Directive ingestRunDirective(Context & c, const std::string & json)
{
  const runstart::Directive d = runstart::parseDirective(json);
  if (d != runstart::Directive::None) {c.run_start.clear();}
  return d;
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

/// Across the dock heading, how far a bay whose indicator this frame did not read GREEN may
/// be from where the GREEN one last was and still be it (pickGreen). Under half a bay's width.
constexpr double kSameBayM = 0.6;

/// This frame's GREEN bay. Committed: the face on our track. Before: the bay it read GREEN
/// (should two, the one nearer where it last was), else - a frame that read no GREEN - the
/// bay not read RED within kSameBayM of where the GREEN one last was. Measured ACROSS the
/// dock heading: the camera's range errors run along it. nullptr when neither.
/// CALL UNDER ctx.mu.
inline const dock::FrameBay * pickGreen(const Context & c)
{
  // Committed (chosen_track): our bay is the face the book put on its track this frame,
  // whatever it reads, or none. Following the last sighting instead lost the bay for good
  // whenever a cut face dragged that sighting sideways (the 11-12 recording, replayed).
  if (c.chosen_track >= 0) {return c.dock.onTrack(c.chosen_track);}
  const double square = c.dock_heading >= 0.0 ? c.dock_heading : c.heading_deg;
  const bool had = c.green.t >= 0.0;
  auto across = [&](const dock::FrameBay & b) {
      return had ? std::fabs(fire::toDockFrame(b.p - c.green.bay.p, square).left) : 0.0;
    };
  const dock::FrameBay * best = nullptr;
  for (const auto & b : c.dock.frame_bays) {
    if (!c.target_any && b.indicator == dock::Colour::Green &&
      (best == nullptr || across(b) < across(*best)))
    {
      best = &b;
    }
  }
  if (best != nullptr || !had) {return best;}
  double best_m = kSameBayM;          // target_any: our bay, whatever it reads
  for (const auto & b : c.dock.frame_bays) {
    if ((c.target_any || b.indicator != dock::Colour::Red) && across(b) < best_m) {
      best = &b;
      best_m = across(b);
    }
  }
  return best;
}

/// Make `g` (a bay of the frame stamped t) our bay's newest sighting, and feed its face
/// centre to ctx.centre unless the image edge cut the face (its centre is biased). The
/// indicator only says the bay's colour. CALL UNDER ctx.mu.
inline void adoptBay(Context & c, const dock::FrameBay & g, double t)
{
  c.green.t = t;
  c.green.bay = g;
  if (!g.windows.empty()) {
    c.green.windows = g.windows;
    c.green.windows_t = t;
  }
  if (!g.truncated && std::isfinite(g.left_m)) {
    c.centre.sight(dock::bodyToWorld(c.boat, c.heading_deg, g.ahead_m, g.left_m),
      std::hypot(g.ahead_m, g.left_m), t, c.now_s);
  }
}

/// RoboCommand's course as the OCS relays it (/crsd/ocs_course: the RxCourse JSON, or
/// null): its id and its corners' lat/lons in order. False, with both cleared, when it has
/// fewer than three corners - no course, and then nothing is fenced.
inline bool parseCourse(const std::string & json, std::string & id,
  std::vector<nav::LatLon> & corners)
{
  id.clear();
  corners.clear();
  auto number = [&json](const char * key, std::size_t from, std::size_t to, double & v) {
      const std::size_t k = json.find(key, from);
      if (k == std::string::npos || k >= to) {return false;}
      const std::size_t c = json.find(':', k);
      if (c == std::string::npos || c >= to) {return false;}
      char * end = nullptr;
      v = std::strtod(json.c_str() + c + 1, &end);
      return end != json.c_str() + c + 1 && std::isfinite(v);
    };
  const std::size_t k = json.find("\"course_id\"");
  if (k != std::string::npos) {
    const std::size_t q1 = json.find('"', json.find(':', k));
    const std::size_t q2 = q1 == std::string::npos ? q1 : json.find('"', q1 + 1);
    if (q2 != std::string::npos) {id = json.substr(q1 + 1, q2 - q1 - 1);}
  }
  const std::size_t c = json.find("\"corners\"");
  const std::size_t open = c == std::string::npos ? c : json.find('[', c);
  const std::size_t close = open == std::string::npos ? open : json.find(']', open);
  for (std::size_t a = open; close != std::string::npos; ) {
    a = json.find('{', a + 1);
    if (a == std::string::npos || a > close) {break;}
    const std::size_t b = json.find('}', a);
    double lat = 0.0, lon = 0.0;
    if (b != std::string::npos && number("\"latitude\"", a, b, lat) &&
      number("\"longitude\"", a, b, lon))
    {
      corners.push_back({lat, lon});
    }
  }
  if (corners.size() < 3) {corners.clear(); return false;}
  return true;
}

/// The course boundary as the dock book's keep-in polygon, in the local frame, once per
/// course and again after resetTask3 (a new book). CALL UNDER ctx.mu.
inline void placeCourse(Context & c)
{
  if (c.dock.keep_in_seq == c.course_seq || !c.origin_set) {return;}
  c.dock.keep_in.clear();
  for (const auto & ll : c.course_corners) {c.dock.keep_in.push_back(nav::toLocal(ll, c.origin));}
  c.dock.keep_in_seq = c.course_seq;
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
    placeCourse(c);
    c.dock.ingest(f, c.boat, c.heading_deg, c.cam_mount,
      c.dock_heading >= 0.0 ? c.dock_heading : fire::kNaN);
    if (const dock::FrameBay * g = pickGreen(c)) {adoptBay(c, *g, f.t);}
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

/// The camera saw this bay within the last second: it is in the picture now.
inline bool inView(const Context & c, const dock::BayTrack & t)
{
  constexpr double kInViewS = 1.0;
  return c.dock_t - t.last_seen <= kInViewS;
}

/// A leaf's heading_deg port, else the GCS's dock heading, else NaN.
inline double dockHeading(const Context & c, double port)
{
  if (std::isfinite(port)) {return port;}
  return c.dock_heading >= 0.0 ? c.dock_heading : std::numeric_limits<double>::quiet_NaN();
}

/// A leaf's first port, else the GCS's search_first, else "left".
inline std::string searchFirst(const Context & c, const std::string & port)
{
  if (!port.empty()) {return port;}
  return c.search_first.empty() ? std::string("left") : c.search_first;
}

/// Forget the last attempt's shots. Called at goal start with resetTask3.
inline void resetFire(Context & c)
{
  c.cmd_speed = 0.0;
  c.last_keep_t = -1.0;
  c.hs_commanded = false;
  c.strafe_state = fire::StrafeState{};
  c.strafe = fire::StrafeCmd{};
  c.strafe_in = fire::StrafeInputs{};
  c.last_strafe_t = -1.0;
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
  c.search_dir = 0;
  c.search_target_track = -1;
  c.search_leg_ends = 0;
  c.chosen_track = -1;
  c.chosen_bay = 0;
  c.colour_rule = dock::ColourRule{};
  c.berth = dock::Berth{};
  c.readiness_confirmed = false;
  c.docking_report_s = -1.0;
  c.fires_out = 0;
  c.green = Context::GreenBay{};
  c.target_any = false;
  c.centre = fire::CentreTarget{};
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
/// Task 4's AvoidInvaders guard (src/task4_leaves.cpp).
void registerTask4Nodes(BT::BehaviorTreeFactory & factory);

}  // namespace crusader_bt

#endif  // CRUSADER_BT__CONTEXT_HPP_
