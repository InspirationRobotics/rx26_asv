// context.hpp — the one shared object every leaf reads, and the leaf base class.
//
// WHY A CONTEXT AND NOT BLACKBOARD PORTS FOR EVERYTHING.
//
// BehaviorTree.CPP can carry any type through a port, but every non-trivial one
// needs a convertFromString specialisation before it can appear in the XML, and
// a missing one is a RUNTIME error at tree-load — on the boat, at the dock. So
// ports here carry only strings and numbers, which the library handles natively,
// and world state (pose, buoys, entry/exit, the local origin) travels in this
// struct instead. It is the blackboard in the sense that matters: one object,
// written by the runner, read by every leaf.
//
// THE LEAVES NEVER SUBSCRIBE. The runner owns every subscription and refreshes
// this struct once per tick. A leaf that opened its own subscription could not
// be reasoned about off-boat, would double the DDS traffic for a topic already
// being read, and would see a different instant of the world than its siblings.
//
// LOCKING. The runner writes under `mu` from ROS callbacks; leaves read under
// it from the tick thread. Every access goes through a lock_guard — the tick
// runs on the executor thread and the callbacks may not.
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

namespace crusader_bt
{

/// World state, refreshed by the runner once per tick. Read-only to leaves
/// except for `waypoint` and `consumed`, which NextWaypoint writes, the gate
/// bookkeeping the Disruptive Task 1 leaves keep, and the Task 3 commitments
/// (chosen bay, berth, request) below `dock`.
struct Context
{
  std::mutex mu;
  rclcpp::Node * node = nullptr;              ///< borrowed; the runner owns it

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
  // `buoys` above is the FUSION of this and the tracker's targets, and the
  // beacon state in it always came from here. See nav::fusePassage.
  //
  // entry/exitp come from the plan too, not from findBeacon over the boat's own
  // targets: the EXIT sits 92 m out in a full-size field, far past the camera,
  // and a transit that waits to SEE it never starts.
  std::vector<nav::PlanBuoy> plan;
  std::uint32_t plan_version = 0;             ///< bumps on every new plan
  bool plan_fresh = false;                    ///< aged like pose_fresh
  double plan_age_s = 0.0;                    ///< seconds since the last plan
  std::vector<nav::Buoy> obstacles;           ///< tracked, but not in the plan

  // ---- the passage the BOAT planned ----
  //
  // The aircraft sends ten buoys and their colours. Which pair is a gate, and
  // what order to drive them in, is worked out here by nav::planPassage over
  // `buoys` -- the FUSED field, so the aircraft's colours and the tracker's
  // positions. Re-planned every tick, because a confirmation can bring new
  // colours and the answer must follow them.
  nav::Passage passage;

  /// Gates already driven, as (red_id, green_id).
  ///
  /// BY BUOY IDS, NOT BY COUNT. A confirmation can recolour the field, the plan
  /// re-runs, and the order can change underneath the boat; "I have done two,
  /// start at index two" then skips a gate that was never driven. Ids survive
  /// a reorder and survive the pair itself swapping colours.
  std::vector<std::pair<int, int>> cleared_gates;

  // ---- the confirmation handshake ----
  //
  // Having crossed a gate, the boat publishes gate_reached(gate_seq) and waits
  // for an acknowledgement carrying the SAME seq before driving the next one.
  // The question it is asking is "is the next pair still where you said it
  // was" -- the aircraft answers by acking and retransmitting the whole field.
  //
  // Matching on the sequence is what makes a retransmission on a lossy radio
  // distinguishable from the answer to the next request.
  std::uint8_t gate_seq = 0;
  int gate_red_id = -1;                       ///< the gate being driven now
  int gate_green_id = -1;
  bool have_confirmation = false;             ///< an ack for gate_seq arrived
  /// Every gate in `passage` is in `cleared_gates`: go to the exit.
  ///
  /// THE BOAT DECIDES THIS NOW, from its own plan -- it holds the whole field,
  /// so it can count. The aircraft used to end the transit with a 255/255
  /// answer, which it can no longer do because it no longer assigns gates.
  ///
  /// Proximity to the exit is still not a substitute, for the reason it never
  /// was: the boat must clear every gate first, and the exit buoy can sit well
  /// inside any sane arrival radius of the last one.
  bool passage_complete = false;
  /// Did the boat finish the gate it last asked about?
  ///
  /// RequestConfirmation advances the sequence ONLY when this is set, so a leg
  /// that failed re-asks under the SAME seq and the aircraft's idempotent
  /// answer gives the same reply. Without it every failure inside the leg --
  /// a confirmation timeout, an unusable pair, a PlanChanged re-task -- burns
  /// a sequence number, and back when the sequence number chose the gate that
  /// also SKIPPED one. SITL 2026-09-13.
  ///
  /// It is no longer what stops a gate being skipped -- cleared_gates is, and
  /// it is keyed on the buoys rather than on a counter -- but it still keeps
  /// the sequence numbering honest.
  bool gate_cleared = false;

  // ---- Task 3: the docking bays ----
  //
  // `dock` is written by the runner on every DockObservation, through
  // ingestDockObservation() below - the ONE function both runners call, so
  // the ROS node and the off-ROS sim runner cannot disagree about what a frame
  // means. Everything after it is written by the Task 3 leaves.
  int tier = 0;                               ///< goal tier: 0 Core, 1 Adv, 2 Disr
  dock::DockBook dock;                        ///< the bays, world-anchored
  dock::Mount cam_mount;                      ///< camera_link in base_link
  /// Face centre above the camera, m: places sightings from a PITCHED camera
  /// (dock::faceInBody). Copied into the book's params at every ingest.
  double dock_face_dz = 0.39;
  double dock_obs_age_s = 1e9;                ///< since the last DockObservation
  std::uint32_t dock_seq = 0;                 ///< bumps on every DockObservation
  double dock_t = 0.0;                        ///< that frame's stamp, seconds
  /// The book's track id for each bay of that frame (-1: not placed), as
  /// DockBook::ingest matched them. All -1 when the book did not take the frame.
  std::vector<int> dock_frame_tracks;
  // The timing layer's verdict on the latest frame, for the bay it tracks.
  std::string dock_pattern;                   ///< "", steady, flash, code, ...
  std::vector<dock::Colour> dock_colours;
  int dock_target_window = -1;                ///< DockWindow.index, -1 = none
  double dock_fps = 0.0;                      ///< the timing layer's observed rate
  /// "hit" events seen so far. COUNTED, because DockObservation.last_event is
  /// set on one frame only and a leaf ticking at 10 Hz can miss that frame.
  int dock_hits = 0;

  /// How much indicator evidence makes a verdict, and how many sightings make
  /// a bay. HERE rather than on each leaf's ports because SafeBayKnown and
  /// CommitSafeBay must apply the SAME rule - a condition that passes on
  /// looser numbers than the commit that follows it is a tree that stalls.
  dock::VoteParams dock_votes;
  int dock_min_obs = 5;
  /// How many bays the dock has: dock::layout() numbers bays only once this
  /// many are confirmed. 3 on the course; 1 to test the Task 3 tree against a
  /// single practice bay (bt_runner_node dock_bays).
  int dock_bays = 3;

  std::string task3_phase;                    ///< for feedback and the viewers
  int survey_attempt = 0;                     ///< PickVantage calls, all of them
  int survey_looks = 0;                       ///< ... of which placed from seen bays
  int chosen_track = -1;                      ///< committed by CommitSafeBay
  int chosen_bay = 0;                         ///< its 1-based number
  dock::Berth berth;                          ///< for the chosen bay
  int fired_window = -1;                      ///< DockWindow.index we put out
  /// RoboCommand's ReadinessConfirm for the docking report, from
  /// /crsd/ocs_command. The light is what the leaves wait on; this only stops
  /// the report being re-sent.
  bool readiness_confirmed = false;
  dock::Request request;                      ///< decoded, held, not yet sent
  bool have_request = false;

  /// Where the boat was when the goal was accepted. "Go home" in a mission
  /// means "back to where this attempt started", not the autopilot's HOME —
  /// those differ whenever the boat was driven out manually first.
  nav::Vec2 home;
  bool have_home = false;

  // ---- the fixed-nozzle shot (fire_math; src/fire_leaves.cpp) ----
  //
  // Stand off a wall at the calibrated range, pointed at a window, and fire
  // when still. The runner feeds the three streams through the ingest*()
  // functions below; now_s is the runner's clock (monotonic seconds) for this
  // tick, the SAME clock the samples are stamped with.
  double now_s = 0.0;
  std::string mode;                           ///< the autopilot's mode, upper case
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
  /// Set by StationKeep on every tick it commands; the runner clears it before
  /// each tick and sends a STOP if motion was commanded last tick and not this
  /// one - a leaf has no hook for "I stopped being ticked".
  bool hs_commanded = false;

  // ---- the strafe keep (MANUAL; fire_math's second half) ----
  // The camera's windows of the bay ahead, in the BODY frame, per
  // DockWindow.index (0 = upper-left, 1 = lower-right), stamped on now_s's
  // clock at receipt (ingestFireWindows). And the compass heading that squares
  // the bow to that face, from the two windows (or the face plane).
  fire::Series fire_win_x[2];
  fire::Series fire_win_y[2];
  fire::Series face_heading{true};
  /// The FACE CENTRE of the bay the hold steers on (dock::pickHoldBay: the
  /// committed bay, else the GREEN one, else the nearest), body frame, for
  /// StrafeKeep lateral_ref="face". face_pick names the rule that chose it.
  fire::Series fire_face_x;
  fire::Series fire_face_y;
  std::string face_pick;
  /// The book's track (resolved) of the bay the face hold last steered on, -1
  /// none: what SprayUntilHit aims at when no bay was committed (a hold-and-
  /// fire test in front of one practice bay).
  int hold_track = -1;
  /// What the keep holds on the line ("window", "face", "slip"), for the
  /// gate's reasons.
  std::string strafe_ref = "window";
  std::string face_src;                       ///< "windows" / "plane": the last source
  double face_target = fire::kNaN;            ///< the last good square heading, held
  double yaw_rate_dps = fire::kNaN;           ///< ATTITUDE.yawspeed, + = turning right
  fire::StrafeState strafe_state;             ///< the keep's slew and integrators
  fire::StrafeInputs strafe_in;               ///< what the keep saw last tick, for the gate
  fire::StrafeCmd strafe;                     ///< ... and what it made of it
  std::string strafe_block;                   ///< non-empty: the keep refuses (why)
  double last_strafe_t = -1.0;
  /// Live gain overrides, set by bt_runner_node's strafe.* parameter callback.
  /// NOT cleared per goal: it is the node's parameters, not the run's state.
  fire::StrafeTune strafe_tune;
  /// Heading by time (ingestHeading: /crsd/pose, the EKF yaw), so the lateral estimator can take
  /// each camera frame's yaw out with the heading AT that frame's capture time.
  fire::HeadingHistory heading_hist;
  /// The lateral estimator (strafe.est_enable): which window it tracks, and
  /// the newest camera sample it has taken in. Cleared per goal (resetFire).
  fire::LateralEstimator lat_est;
  int lat_est_widx = -1;
  double lat_est_seen_t = -1e18;
  /// Set by StrafeKeep on every tick it commands the sticks; the runner clears
  /// it before each tick and RELEASES the sticks if they were commanded last
  /// tick and not this one.
  bool sticks_commanded = false;

  // ---- the slip from the LiDAR (Task 3 with the pan/tilt cannon; SlotKeep) ----
  // /crsd/dock_slot, as a WallFilter: range_m = the LiDAR's standoff from the
  // back wall, angle_deg = the slip's axis (+ = turn left to square), lat_m =
  // the slip's centreline at the body origin (+ = left), each sample stamped
  // with the heading at receipt so the square heading survives a turn
  // (WallFilter::inward_deg). slot_lat is the same lateral as a Series, for
  // its rate. slot_why: the last sweep's reason, for the log.
  fire::WallFilter slot;
  fire::Series slot_lat;
  std::string slot_why;

  // ---- the whole-field Task 1 plan (global_passage.hpp; src/global_leaves.cpp) ----
  //
  // task1_global.xml plans the WHOLE passage at once (approach, entry orbit, transit, exit
  // orbit) and drives it phase by phase. Per mission: reset with the goal, like the gates.
  struct GlobalPassage
  {
    gp::Phase phase = gp::Phase::Approach;   ///< the phase being driven, or the next one
    gp::Plan plan;                           ///< ok = false until one was made
    gp::Params params;                       ///< PlanGlobalPassage's, reused by every replan
    gp::FieldSig field;                      ///< the aircraft's report the plan was made against
    double transit_s = 0.0;                  ///< progress along plan.legs[Transit] at the last stop
    std::vector<nav::Vec2> traj;             ///< the transit as driven, first point = its start
    std::vector<nav::Vec2> entry_ring;       ///< the ring the boat orbited (or is about to)
    bool at_checkpoint = false;              ///< the last transit stretch stopped at a gate
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
  // Spec: docs/nav2_avoidance_spec.md sections 2 and 5.4. Written by the runner,
  // read by NavigateTo and CircleBuoy; every runner-side rule that touches the
  // frame lives in ingestPose / applyDatum below so the ROS runner and the
  // off-ROS one cannot disagree about it.
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
  /// A different datum arrived after one was adopted. The origin never moves
  /// mid-run (the DockBook, home and the gate bookkeeping hold local coordinates),
  /// so every planned leg holds and the hazard publisher stops.
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
  std::function<void()> publish_report;
  std::function<void(int)> consume_buoy;      ///< mark a buoy dealt with
  /// Tell the aircraft a gate is cleared and ask for the next pair. Publishes
  /// std_msgs/UInt8 on crsd/gate_reached; rxl_link_node puts it on the air.
  std::function<void(std::uint8_t)> report_gate_reached;

  // ---- Task 3 outputs ----
  //
  // The three RoboCommand reports go out as JSON for the OCS to relay, like
  // the Task 1 report. Field names are rx_reports.proto's.
  std::function<void(int)> report_docking;          ///< DockingReport.bay_id
  /// One line of the run's story for the operator ("DOCKED", "ON FIRE: ...",
  /// "CODE: ..."): logged as "TASK3 | ..." and, on the boat, /crsd/task3_events.
  std::function<void(const std::string &)> announce;
  std::function<void(int)> report_firefighting;     ///< FirefightingReport.window_id
  /// ResourceDeliveryRequest to RoboCommand.
  std::function<void(const dock::Request &)> report_request;
  /// The same request to the UAV, over the radio.
  std::function<void(const dock::Request &)> relay_request;
  /// The water cannon: fire or not, aimed at a point in camera_link, at
  /// window (DockWindow.index; -1 = none) for that window's own tilt trim.
  std::function<void(bool, double, double, double, int)> cannon;

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
};

using ContextPtr = std::shared_ptr<Context>;

// ------------------------------------------------------------ the shared frame
//
// Spec section 2. The TF frame `map`, ctx.origin and every HazardArray
// coordinate are ONE local ENU plane, centred on one datum that nav_frames_node
// owns and publishes on /crsd/datum. These two functions are the whole of how a
// runner learns it, and BOTH runners call them (bt_runner_node.cpp and
// offros_runner.cpp): the rule lives here, not in two copies that drift.

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
/// With nav_mode off the origin is pinned at the first fix, as it always was. With
/// shadow or on it is NOT: a first-fix origin is not the TF frame, so the planner
/// would be handed coordinates in a plane the costmap does not share, and the
/// error is a silent offset. The boat is left unplaced until the datum arrives
/// (applyDatum places it from the fix kept here); pose_fresh stays false until
/// then because it requires origin_set.
inline PoseEvent ingestPose(Context & c, nav::LatLon fix, double heading_deg)
{
  if (!std::isfinite(fix.lat) || !std::isfinite(fix.lon)) {return PoseEvent::Bad;}
  c.fix = fix;
  c.have_fix = true;
  c.heading_deg = heading_deg;        // NaN when GPS yaw is unresolved
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
  Ignored     ///< non-finite, a mismatch already reported, or nav_mode off
};

/// A datum arrived (/crsd/datum, or the offros "datum" line). CALL UNDER ctx.mu.
///
/// NEVER RE-PINS MID-RUN. A datum that differs from the adopted origin by more than
/// 0.01 m sets datum_mismatch instead: the DockBook, `home` and the gate
/// bookkeeping all hold local coordinates, and moving the origin under them would
/// teleport the field. The leg holds until bt_runner is restarted.
///
/// NOT IN OFF: off pins the origin at the first fix, as it always did, and shares its
/// frame with no costmap, so a datum is none of its business. Without this a boat in
/// off whose container merely runs nav_frames_node would log a "datum changed,
/// restart bt_runner" ERROR against a pin nobody asked to be the datum.
inline DatumEvent applyDatum(Context & c, nav::LatLon datum)
{
  if (c.nav_mode == path::Mode::Off) {return DatumEvent::Ignored;}
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
  if (c.datum_mismatch) {return DatumEvent::Ignored;}
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
/// SIDE FENCES join the set ONLY in nav_mode on (path::sideFences). They exist for the planner,
/// which is the only thing that can steer round them. In off a known hazard on a straight leg
/// HOLDS the boat, and in shadow the legacy legs ignore the plan, so a fence there would stop
/// the boat at a wall nothing is going to go round. They come from the tree's own passage plan
/// (ctx.passage, re-planned every tick of the transit and cleared with the mission), so a
/// Task 1 field is the only thing that can produce one.
inline std::vector<path::Hazard> knownHazards(const Context & c)
{
  std::vector<path::Hazard> hz =
    path::buildHazards(c.buoys, c.obstacles, c.dock, c.dock_min_obs, c.nav);
  if (c.nav_mode == path::Mode::On && c.have_entry && c.have_exit) {
    const std::vector<path::Hazard> fences =
      path::sideFences(c.buoys, c.passage, c.entry, c.exitp, c.nav);
    hz.insert(hz.end(), fences.begin(), fences.end());
  }
  return hz;
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

/// Back to "no leg running". CALL UNDER ctx.mu.
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

inline void clearLeg(Context & c)
{
  c.leg = Context::LegStatus{};
  ++c.leg_seq;
}

/// Fold one DockObservation into the context. CALL UNDER ctx.mu.
///
/// Both runners convert their input (a crusader_msgs/DockObservation, or a
/// JSON line from the sim) into a dock::Frame and hand it here; nothing about
/// what a frame MEANS is decided in either runner.
///
/// The bays are placed only with a fresh pose and a resolved heading - a
/// sighting placed from a stale pose is a bay in the wrong place, forever. The
/// timing layer's verdict does not depend on the pose, so it is always taken.
inline void ingestDockObservation(Context & c, const dock::Frame & f)
{
  c.dock_frame_tracks.assign(f.bays.size(), -1);
  if (c.origin_set && c.pose_fresh && std::isfinite(c.heading_deg)) {
    c.dock.prm.face_dz = c.dock_face_dz;
    c.dock_frame_tracks = c.dock.ingest(f, c.boat, c.heading_deg, c.cam_mount);
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

/// One /crsd/dock_slot message (crusader_msgs/DockSlot). CALL UNDER ctx.mu.
/// Valid only when the whole slip was found: both side walls (or one and the
/// known width) and the back wall. Stamped with the heading at receipt, like
/// the wall range.
inline void ingestDockSlot(
  Context & c, double t, bool valid, double back_range_m, double angle_deg, double lateral_m,
  const std::string & why = "")
{
  fire::WallSample s;
  s.t = t;
  s.valid = valid && std::isfinite(back_range_m) && std::isfinite(angle_deg) &&
    std::isfinite(lateral_m);
  s.range_m = back_range_m;
  s.angle_deg = angle_deg;
  s.lat_m = lateral_m;
  s.heading_deg = c.heading_deg;
  c.slot.add(s);
  if (s.valid) {c.slot_lat.add(t, lateral_m);}
  c.slot_why = why;
}

/// The heading just set on ctx (heading_deg, from /crsd/pose), into the
/// history the lateral estimator reads. CALL UNDER ctx.mu, after setting it,
/// with `t` on now_s's clock. Both runners call it from their pose handler.
inline void ingestHeading(Context & c, double t)
{
  c.heading_hist.add(t, c.heading_deg);
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

/// The face the strafe keep can hold on the line instead of a window
/// (lateral_ref="face"). CALL UNDER ctx.mu, from ingestFireWindows. The bay is
/// dock::pickHoldBay's (committed, else GREEN, else nearest); its face centre
/// goes into the body frame through cam_mount.
///
/// A face cut at ONE side of the image is still used. Its bearing is the
/// visible part's, which lies on the same side of the boat as the true centre
/// and short of it by half the part cut off - so the hold pushes the right way,
/// a little too gently, and is exact again once the whole face is back in view.
/// Dropping it instead left a boat pushed sideways with no sideways error at
/// all, and it never came back (2026-10-09, on the water). Cut at BOTH sides
/// (the face wider than the view) the centre is anywhere: nothing.
inline void ingestFireFace(Context & c, double t, const dock::Frame & f)
{
  std::vector<int> ids = c.dock_frame_tracks;
  for (int & id : ids) {if (id >= 0) {id = c.dock.resolve(id);}}
  const int chosen = c.chosen_track >= 0 ? c.dock.resolve(c.chosen_track) : -1;
  const char * why = "";
  const int i = dock::pickHoldBay(f, ids, chosen, &why);
  if (i < 0) {return;}
  const dock::BaySighting & s = f.bays[static_cast<std::size_t>(i)];
  c.face_pick = why;
  if (static_cast<std::size_t>(i) < ids.size() && ids[static_cast<std::size_t>(i)] >= 0) {
    c.hold_track = ids[static_cast<std::size_t>(i)];
  }
  if (s.cut_left && s.cut_right) {
    c.face_pick += ", wider than the view";
    return;
  }
  if (s.cut_left || s.cut_right) {c.face_pick += s.cut_left ? ", cut at the left" : ", cut at the right";}
  dock::Vec2 p;
  if (!dock::faceCentreBody(s, c.cam_mount, c.dock_face_dz, p)) {return;}
  c.fire_face_x.add(t, p.x);
  c.fire_face_y.add(t, p.y);
}

/// The strafe keep's view of one DockObservation. CALL UNDER ctx.mu, after
/// ingestDockObservation, with `t` on now_s's clock (the frame's own stamp is
/// another clock). The face first (ingestFireFace). Then takes the bay nearest
/// the bow that has a positioned window; puts its windows into the body frame
/// through cam_mount; and, when both windows are there at the face's spacing,
/// the heading that would square the bow to it - else the same from the face
/// plane's normal.
inline void ingestFireWindows(Context & c, double t, const dock::Frame & f)
{
  ingestFireFace(c, t, f);
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
  c.lat_est.reset();
  c.lat_est_widx = -1;
  c.lat_est_seen_t = -1e18;
}

/// Forget everything Task 3 learned. Called at goal start: a second attempt
/// must not inherit the first one's bays, votes or commitments.
inline void resetTask3(Context & c)
{
  c.dock = dock::DockBook{};
  c.dock_hits = 0;
  c.task3_phase.clear();
  c.fired_window = -1;
  c.survey_attempt = 0;
  c.survey_looks = 0;
  c.chosen_track = -1;
  c.chosen_bay = 0;
  c.hold_track = -1;
  c.berth = dock::Berth{};
  c.readiness_confirmed = false;
  c.request = dock::Request{};
  c.have_request = false;
}

/// A place to go, passed BETWEEN LEAVES on the blackboard.
///
/// This is the channel perception uses to tell an action where to drive. A
/// compute leaf writes one to an output port, an action leaf reads it from an
/// input port, and the XML shows the wire:
///
///     <NextWaypoint buoys="{buoys}" out="{goal}"/>
///     <NavigateTo   goal="{goal}"/>
///
/// NO convertFromString SPECIALISATION IS NEEDED, and that is worth knowing
/// because the fear of needing one is why the first version of this package
/// smuggled every position through the Context struct instead. A specialisation
/// is only required to parse a LITERAL out of the XML (goal="1.28;103.85"). A
/// value that only ever travels between leaves as {ref} is moved as the real
/// type and never touches a string.
///
/// `why` is carried so a log line, a feedback message or a viewer can say WHICH
/// buoy this is and why we are steering there — a bare lat/lon in a log tells
/// you where the boat went and nothing about what it thought it was doing.
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

  rclcpp::Logger log() const
  {
    return ctx_->node ? ctx_->node->get_logger() : rclcpp::get_logger("crusader_bt");
  }
};

/// Base for a leaf that DOES something but finishes within one tick.
///
/// The distinction from CrusaderCondition is not decoration. BT.CPP's
/// SyncActionNode::executeTick THROWS if the leaf returns RUNNING, which is the
/// right guard for a leaf that is supposed to be instantaneous, and the two
/// report different NodeType — so Groot, and tools/bt_view.py, draw a thing that
/// publishes a topic differently from a thing that answers a question.
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

  rclcpp::Logger log() const
  {
    return ctx_->node ? ctx_->node->get_logger() : rclcpp::get_logger("crusader_bt");
  }
};

/// Base for leaves that answer in one tick and have NO side effects.
class CrusaderCondition : public BT::ConditionNode
{
public:
  CrusaderCondition(const std::string & name, const BT::NodeConfig & cfg)
  : BT::ConditionNode(name, cfg), ctx_(contextFrom(cfg, name)) {}

protected:
  ContextPtr ctx_;

  rclcpp::Logger log() const
  {
    return ctx_->node ? ctx_->node->get_logger() : rclcpp::get_logger("crusader_bt");
  }
};

/// Register every Crusader leaf with `factory`.
///
/// Registered DIRECTLY rather than loaded as plugins with registerFromPlugin().
/// Plugins buy hot-swapping a behaviour without relinking, which we will never
/// do mid-competition, and cost a shared-library search path that is wrong in
/// exactly one environment — the container — and fails at tree-load with a
/// message about a missing .so rather than about the tree.
void registerCrusaderNodes(BT::BehaviorTreeFactory & factory);

/// The Task 3 leaves, from src/task3_leaves.cpp. Called by
/// registerCrusaderNodes, so a runner registers everything with one call.
void registerTask3Nodes(BT::BehaviorTreeFactory & factory);
/// The fixed-nozzle shot's leaves (src/fire_leaves.cpp).
void registerFireNodes(BT::BehaviorTreeFactory & factory);
/// The whole-field Task 1 leaves (src/global_leaves.cpp, task1_global.xml). Also called by
/// registerCrusaderNodes.
void registerGlobalPassageNodes(BT::BehaviorTreeFactory & factory);

}  // namespace crusader_bt

#endif  // CRUSADER_BT__CONTEXT_HPP_
