"""mission_lib — the pure half of the mission planner: catalogue, list, load, validate, save.

The Mission tab edits behaviour trees as blocks and stores them as BehaviorTree.CPP XML, so
everything that has to be RIGHT lives here once: which blocks exist and what ports they take,
how `<include>` flattens, and whether a tree will load on the boat. Both servers
(`gcs_laptop.py` on the laptop, `gcs_node.py` on the Jetson) import this module, so the two
cannot disagree. Stdlib only, no ROS: it runs on Windows and inside the container, and
`test/test_mission_lib.py` exercises all of it off-boat.

THE CATALOGUE IS A TABLE, NOT A PARSE OF THE C++. The installed boat has no sources, so the
ports of every `factory.registerNodeType` leaf are written out below. The test compares the
table to `crusader_bt/src/*.cpp` (ids, port names, directions, kinds), so a port added in C++
and not here fails there, not on the water. Defaults and descriptions are copied from the
source; enum choices are the "a | b | c" lists its descriptions already carry.

A SAVED MISSION IS SELF-CONTAINED. Every tree it uses is inlined and there is no `<include>`:
the file that ran is the whole record of what ran. `flatten` is how a pre-built file that
includes `lib/*.xml` becomes one, and `validate` refuses text that still has an include.

EVERY `<SubTree>` NEEDS `_autoremap="true"`. A subtree gets its own blackboard, and every
Crusader block reads its context from the blackboard key `ctx`, so without the remap each
block throws "no 'ctx' on the blackboard" at tree load (crusader_bt/README.md).

DESCRIPTIONS ARE COMMENTS. A mission file's first XML comment, when it starts `MISSION:`, is
its description; a lib subtree file's starts `<SubtreeID>: `. ElementTree drops comments, so
those are read from the raw text.
"""
from __future__ import annotations

import copy
import math
import os
import re
import xml.etree.ElementTree as ET

CATALOGUE_VERSION = 1

# ocs_link.RX_TASKS, duplicated so this file imports nothing; the test compares the two.
_RX_TASKS = ["TASK_SAFE_PASSAGE", "TASK_INFRA_SURVEY_REPAIR", "TASK_COORDINATED_LOGISTICS",
             "TASK_DYNAMIC_INCIDENT", "TASK_NONE"]

_CATEGORIES = [
    ("flow", "Flow", "#e6a23c"),
    ("decorator", "Decorators", "#d9a521"),
    ("robocommand", "RoboCommand", "#3fa34d"),
    ("navigation", "Navigation", "#4c97ff"),
    ("task", "Tasks", "#9966ff"),
    ("condition", "Conditions", "#2ab3a6"),
    ("task1", "Task 1 internals", "#cf63cf"),
    ("task3", "Task 3 internals", "#ff8c1a"),
    ("fire", "Fire internals", "#e05d5d"),
    ("other", "Other", "#8a8a8a"),
]

_SAFE_NAME = re.compile(r"[A-Za-z0-9_-]{1,64}")
_DEVICE = re.compile(r"(?i)con|prn|aux|nul|com[1-9]|lpt[1-9]")
_BLACKBOARD = re.compile(r"\{[^{}]+\}")
_INT = re.compile(r"-?\d+")
_REAL = re.compile(r"-?\d+\.\d+")
_COMMENT = re.compile(r"<!--(.*?)-->", re.S)
_XML_DECL = '<?xml version="1.0"?>\n'

# BT.CPP lets <Action ID="Foo"/> stand for <Foo/>.
_GENERIC_TAGS = ("Action", "Condition", "Control", "Decorator")
# Controls that throw at tick unless they have exactly these many children.
_ARITY = {"IfThenElse": (2, 3), "WhileDoElse": (2, 3)}
# Which port picks "a GPS point" on the two blocks that take lat/lon.
_FIX_SELECTOR = {"NavigateTo": "target", "CircleBuoy": "anchor"}


# ----------------------------------------------------------------------------- the table

def _port(name, typ, default, desc, direction, required, choices=None):
    p = {"name": name, "type": typ, "default": default, "desc": desc,
         "direction": direction, "required": required}
    if typ == "enum":
        p["choices"] = list(choices)
    return p


def _in(name, typ, default, desc, choices=None, req=None):
    """An input. With no default it is required unless the description says what absence means."""
    return _port(name, typ, default, desc, "in", default is None if req is None else req, choices)


def _out(name, desc):
    return _port(name, "bb", None, desc, "out", False)


# (id, kind, category, advanced, description, ports)
_BUILTINS = [
    ("Sequence", "control", "flow", False,
     "Runs its children left to right; fails at the first child that fails, succeeds when all "
     "have succeeded.", []),
    ("SequenceWithMemory", "control", "flow", True,
     "Like Sequence, but after a child that is still running it carries on from that child "
     "instead of starting over.", []),
    ("ReactiveSequence", "control", "flow", False,
     "Re-checks every child above the running one each tick; use it to guard a long action "
     "with conditions.", []),
    ("Fallback", "control", "flow", False,
     "Tries its children left to right until one succeeds; fails only if all of them fail.",
     []),
    ("ReactiveFallback", "control", "flow", False,
     "Like Fallback, but re-tries the earlier children every tick, so a preferred option can "
     "interrupt a running one.", []),
    ("Parallel", "control", "flow", False,
     "Runs all children at once; succeeds when success_count of them have succeeded, fails "
     "when failure_count have failed.",
     [_in("success_count", "int", "-1", "children that must succeed (-1 = all of them)"),
      _in("failure_count", "int", "1", "children that fail the whole node")]),
    ("ParallelAll", "control", "flow", True,
     "Runs all children at once until every one has finished; fails if more than max_failures "
     "failed.",
     [_in("max_failures", "int", "1", "failures tolerated before the node fails")]),
    ("IfThenElse", "control", "flow", False,
     "Child 1 is the condition: if it succeeds run child 2, otherwise run child 3 (optional).",
     []),
    ("WhileDoElse", "control", "flow", True,
     "While child 1 (the condition) succeeds keep running child 2; when it fails run child 3 "
     "(optional).", []),
    ("Inverter", "decorator", "decorator", False,
     "Swaps its child's result: success becomes failure and failure becomes success.", []),
    ("ForceSuccess", "decorator", "decorator", False,
     "Reports success whatever its child returns (a child still running stays running).", []),
    ("ForceFailure", "decorator", "decorator", False,
     "Reports failure whatever its child returns (a child still running stays running).", []),
    ("RetryUntilSuccessful", "decorator", "decorator", False,
     "Restarts a failing child, up to num_attempts times; succeeds as soon as the child does.",
     [_in("num_attempts", "int", None, "tries before giving up (-1 = forever)")]),
    ("Repeat", "decorator", "decorator", False,
     "Runs its child num_cycles times in a row; fails if the child fails.",
     [_in("num_cycles", "int", None, "how many times to run the child (-1 = forever)")]),
    ("KeepRunningUntilFailure", "decorator", "decorator", False,
     "Re-runs its child every time it succeeds; fails, ending the loop, the first time the "
     "child fails.", []),
    ("Timeout", "decorator", "decorator", False,
     "Stops its child and fails if the child is still running after msec milliseconds.",
     [_in("msec", "int", None, "milliseconds the child may run")]),
    ("Delay", "decorator", "decorator", False,
     "Waits delay_msec milliseconds, then runs its child.",
     [_in("delay_msec", "int", None, "milliseconds to wait first")]),
    ("RunOnce", "decorator", "decorator", True,
     "Runs its child once; afterwards it is skipped (then_skip) or repeats the first result.",
     [_in("then_skip", "bool", "true", "true = skip later ticks, false = repeat the result")]),
    ("AlwaysSuccess", "action", "flow", False,
     "Succeeds immediately; a placeholder, or a condition that is always true.", []),
    ("AlwaysFailure", "action", "flow", False,
     "Fails immediately; a placeholder, or a condition that is always false.", []),
    ("Sleep", "action", "flow", False,
     "Waits msec milliseconds, then succeeds.",
     [_in("msec", "int", None, "milliseconds to wait")]),
]

# The crusader_bt blocks, one per factory.registerNodeType. Ports follow providedPorts();
# the first three are the RoboCommand blocks the run-level tree is built from.
_CRUSADER = [
    ("WaitForRunStart", "action", "robocommand", False,
     "Hold position until RoboCommand's RunStart reaches the boat; remove it to bench-test "
     "without RoboCommand",
     [_in("max_age_s", "double", "30",
          "a RunStart received up to this long before the block started still counts"),
      _in("timeout_s", "double", "0", "fail after waiting this long (0 = wait forever)"),
      _in("hold", "bool", "true", "hold station while waiting"),
      _out("run_id", "the RunStart's run id, once received")]),
    ("TaskAttempt", "decorator", "robocommand", False,
     "Reports this task in the heartbeat's current_task while its child runs, TASK_NONE after",
     [_in("token", "enum", None, "the task to report in the heartbeat", choices=_RX_TASKS),
      _in("none_dwell_s", "double", "1.5",
          "TASK_NONE is held this long before a new attempt is claimed, so the 2 Hz heartbeat "
          "carries it")]),
    ("SetTier", "action", "robocommand", False,
     "Sets the tier that TierIs and TierAtLeast read, for a mission that runs tasks at "
     "different tiers; always succeeds",
     [_in("tier", "enum", None, "the tier to run the following blocks at",
          choices=["core", "advanced", "disruptive"])]),
    ("IsAutonomous", "condition", "condition", False,
     "Succeeds while the autopilot is in an autonomous mode; fails the moment the pilot takes "
     "the boat back.",
     []),
    ("ModeIs", "condition", "condition", False,
     "Succeeds only while the autopilot is in exactly this mode (for example GUIDED).",
     [_in("mode", "string", "GUIDED", "the mode this subtree needs")]),
    ("TierIs", "condition", "condition", False,
     "Succeeds if the run's tier is exactly this one (core, advanced or disruptive).",
     [_in("tier", "enum", "disruptive",
          "the tier to test for",
          choices=["core", "advanced", "disruptive"])]),
    ("TierAtLeast", "condition", "condition", False,
     "Succeeds if the run's tier is at least this level (0 core, 1 advanced, 2 disruptive).",
     [_in("tier", "int", "1", "0 Core, 1 Advanced, 2 Disruptive")]),
    ("NavigateTo", "action", "navigation", False,
     "Drives to a target and succeeds on arrival (planned around obstacles, or straight); fails "
     "if blocked too long.",
     [_in("target", "enum", "waypoint",
          "where to drive: port = the {goal} point, fix = lat/lon, or a named point",
          choices=["port", "waypoint", "approach", "exit", "home", "fix"]),
      _in("goal", "bb", None,
          "a point written by an upstream block; used with target=port", req=False),
      _in("lat", "double", "0.0", "with target=fix: latitude"),
      _in("lon", "double", "0.0", "with target=fix: longitude"),
      _in("tolerance", "double", "2.0", "arrival radius, metres"),
      _in("resend_m", "double", "1.5",
          "re-send the setpoint when the goal moves further than this"),
      _in("avoid", "bool", "true",
          "plan around known hazards (needs nav_mode shadow or on; in off every leg is a "
          "guarded straight one); false = a straight leg"),
      _in("exempt", "string", "",
          "with avoid=false only: comma list of gate | dock, hazards this leg drives past"),
      _in("blocked_timeout_s", "double", "15.0", "FAILURE after being blocked this long")]),
    ("CircleBuoy", "action", "navigation", False,
     "Drives once around a point on a ring of waypoints; succeeds when the full circle is done.",
     [_in("anchor", "enum", "entry",
          "what to circle: the entry buoy, the exit buoy, or a GPS point (fix)",
          choices=["entry", "exit", "fix"]),
      _in("lat", "double", "0.0", "with anchor=fix: latitude"),
      _in("lon", "double", "0.0", "with anchor=fix: longitude"),
      _in("radius", "double", None,
          "orbit radius, metres (absent: nav_orbit_radius_m)", req=False),
      _in("points", "int", None,
          "waypoints around the circle (absent: nav_orbit_points)", req=False),
      _in("overshoot_deg", "double", "45.0",
          "degrees to keep going past one full turn, the same way round, so a boat that takes "
          "each hop early (tolerance 2 m is ~19 deg at 6 m) still sweeps >= 360; 45 = one extra"
          " point at 8 points; 0 = stop at exactly 360; clamped to 0..360"),
      _in("direction", "enum", "cw", "cw = clockwise, ccw = counter-clockwise",
          choices=["cw", "ccw"]),
      _in("tolerance", "double", None,
          "hop arrival radius, metres (absent: nav_orbit_tolerance_m)", req=False),
      _in("avoid", "bool", "true",
          "plan each hop around known hazards (needs nav_mode shadow or on)"),
      _in("blocked_timeout_s", "double", "15.0",
          "FAILURE after a hop has been blocked this long")]),
    ("HoldStation", "action", "navigation", False,
     "Holds the boat where it is and never succeeds, so wrap it in a Timeout or guard it with a "
     "sibling.",
     []),
    ("StopBoat", "action", "navigation", False,
     "Commands speed zero with the heading held; succeeds at once.",
     []),
    ("TargetAhead", "action", "navigation", True,
     "Writes a point at a set range and bearing from the bow to the blackboard (a stand-in for a"
     " detector).",
     [_in("range", "double", "50.0", "metres from the boat"),
      _in("bearing", "double", "0.0", "degrees relative to the bow, cw+"),
      _in("why", "string", "target", "what this is, for the log"),
      _out("out", "where the next action should drive")]),
    ("AvoidObstacles", "action", "navigation", True,
     "Deprecated: passes the goal through unchanged, because NavigateTo avoids obstacles itself.",
     [_in("in", "bb", None, "the goal an upstream leaf chose"),
      _out("out", "the same goal"),
      _in("clearance", "double", "5.0", "ignored"),
      _in("margin", "double", "2.0", "ignored")]),
    ("SetTask", "action", "robocommand", True,
     "Publishes a task token in the heartbeat; the course lights its beacons only after this. "
     "Prefer TaskAttempt.",
     [_in("token", "enum", "TASK_SAFE_PASSAGE",
          "the task to report in the heartbeat",
          choices=_RX_TASKS)]),
    ("EntryResolved", "condition", "task1", True,
     "Succeeds once the entry buoy has been identified.",
     []),
    ("ExitResolved", "condition", "task1", True,
     "Succeeds once the exit buoy has been identified.",
     []),
    ("NearExit", "condition", "task1", True,
     "Succeeds within a radius of the exit buoy; ends the Core transit.",
     [_in("radius", "double", "8.0", "metres; keep > orbit radius")]),
    ("ResolveBuoyStates", "action", "task1", True,
     "Refreshes buoy colours and states from perception; always succeeds.",
     []),
    ("PublishSafePassageReport", "action", "task1", True,
     "Sends the Safe Passage report (buoys seen and their states) to RoboCommand; always "
     "succeeds.",
     [_in("min_period_s", "double", "1.0", "floor between reports")]),
    ("NextWaypoint", "action", "task1", True,
     "Works out the next place to steer in the Core transit; fails when nothing is left.",
     [_in("offset", "double", "4.0", "metres to clear the buoy by")]),
    ("PassagePlanFresh", "condition", "task1", True,
     "Succeeds while the aircraft's passage plan is recent enough to trust; fails when the link "
     "goes stale.",
     [_in("max_age_s", "double", "15.0",
          "seconds; the plan arrives at ~0.2 Hz so this is not the pose timeout")]),
    ("PlanChanged", "condition", "task1", True,
     "Succeeds when the aircraft has sent a new passage plan since the last look.",
     []),
    ("PassageComplete", "condition", "task1", True,
     "Succeeds when the aircraft has said there are no more gates.",
     []),
    ("PlanPassage", "action", "task1", True,
     "Works out the gate order from the combined aircraft and LiDAR buoy field; fails while the "
     "field is unusable.",
     [_in("max_width", "double", "20.0", "widest a real gate can be, m"),
      _in("min_width", "double", "2.0", "narrowest a real gate can be, m"),
      _in("max_cross_angle", "double", "60.0",
          "a pair is a gate only if crossing it red-to-starboard is within this many degrees "
          "of the entry -> exit axis; otherwise both buoys are singles")]),
    ("SelectNextGate", "action", "task1", True,
     "Picks the first gate not yet driven; fails when the passage is done.",
     []),
    ("RequestConfirmation", "action", "task1", True,
     "Tells the aircraft a checkpoint is done and asks it to confirm the field.",
     [_in("what", "string", "gate", "what the boat just finished, for the log line")]),
    ("AwaitConfirmation", "action", "task1", True,
     "Waits for the aircraft to confirm the field, re-asking until it does or the timeout passes.",
     [_in("timeout_s", "double", "120.0", "give up waiting and drive on with the last field"),
      _in("retry_s", "double", "3.0", "how often to re-send the request while waiting"),
      _in("release_on_change", "bool", "false",
          "true: a CHANGED field also ends the wait - the aircraft re-tasked instead of acking,"
          " and the tree re-plans against it. Changed = a colour, a buoy, or a move beyond "
          "retask_move_m (gp::fieldChange); the aircraft's own re-report scatter is not one"),
      _in("retask_move_m", "double", "2.0",
          "with release_on_change: a buoy that moved less than this has not changed")]),
    ("GateWaypoint", "action", "task1", True,
     "Works out the approach or crossing point for the current gate and writes it to the "
     "blackboard.",
     [_in("point", "enum", "through", "approach = short of the gate, through = past it",
          choices=["approach", "through"]),
      _in("standoff", "double", None,
          "metres past the middle (absent: nav_gate_standoff_m)", req=False),
      _in("approach", "double", None,
          "metres short of the middle (absent: nav_gate_approach_m)", req=False),
      _in("offset", "double", "4.0", "metres to clear a LONE buoy by"),
      _out("out", "where the next action should drive")]),
    ("MarkGateCleared", "action", "task1", True,
     "Records the current gate as driven.",
     []),
    ("TransitComplete", "condition", "task1", True,
     "Succeeds once the whole transit (every gate and side) is behind the boat.",
     []),
    ("AtCheckpoint", "condition", "task1", True,
     "Succeeds if the last transit stretch stopped at a gate's checkpoint rather than at the end.",
     []),
    ("WaitForField", "action", "task1", True,
     "Waits until the aircraft has reported the entry and exit buoys and the boat has a fresh "
     "pose.",
     []),
    ("PlanGlobalPassage", "action", "task1", True,
     "Plans the whole passage from the current phase, keeping the plan while it stays good and "
     "re-planning when not.",
     [_in("orbit_radius_m", "double", None,
          "orbit radius wanted (absent: nav_orbit_radius_m); shrunk round a buoy "
          "until its neighbours keep nav_orbit_clear_m", req=False),
      _in("orbit_min_m", "double", "1.5", "smallest orbit normally accepted"),
      _in("overshoot_deg", "double", "45.0", "orbit past one full turn, at least"),
      _in("soft_m", "double", "2.0", "clearance beyond which more water buys nothing"),
      _in("w_clear", "double", "4.0", "a metre at nav_hard_m costs 1 + w_clear metres"),
      _in("relax1_m", "double", "0.7",
          "clearance tried when nothing fits at nav_hard_m (= nav_hard_m: never relax)"),
      _in("relax2_m", "double", "0.6", "... and then this"),
      _in("res_m", "double", "0.1", "planning grid, metres"),
      _in("retask_move_m", "double", "2.0",
          "a re-reported buoy that moved less than this is the aircraft's scatter, not a re-task"),
      _in("retry_s", "double", "1.0", "between tries while no plan fits"),
      _in("timeout_s", "double", "60.0", "FAILURE after this long without a plan")]),
    ("DrivePlanPhase", "action", "task1", True,
     "Drives one phase of the plan: approach, entry orbit, transit or exit orbit.",
     [_in("phase", "enum", "transit",
          "which stretch of the plan to drive",
          choices=["approach", "entry_orbit", "transit", "exit_orbit"]),
      _in("stop_at_checkpoints", "bool", "false",
          "transit only: end each stretch at the next gate's checkpoint (Disruptive)"),
      _in("stop_at_end", "bool", "false",
          "stop on the leg's last point instead of handing over to the next leg while moving"),
      _in("blocked_timeout_s", "double", "15.0", "FAILURE after being blocked this long"),
      _in("lookahead_m", "double", "1.5", "pure pursuit lookahead along the path"),
      _in("resend_m", "double", "0.3", "re-send the setpoint once it has moved this far"),
      _in("end_tol_m", "double", "0.8", "arrival radius at a stop")]),
    ("DockCameraAlive", "condition", "task3", True,
     "Succeeds while the dock detector is publishing frames.",
     [_in("max_age_s", "double", "3.0", "seconds without a DockObservation")]),
    ("SafeBayKnown", "condition", "task3", True,
     "Succeeds once exactly one dock bay reads GREEN.",
     [_in("need_others_red", "bool", "true", "also require every other bay to read RED")]),
    ("ChosenBaySafe", "condition", "task3", True,
     "Succeeds while the committed bay still reads GREEN on recent frames.",
     [_in("min_ema", "double", "0.3", "recent GREEN share below which we abort")]),
    ("DockedInBay", "condition", "task3", True,
     "Succeeds when the boat is in the berth: on the centreline, bow in, whole hull inside the "
     "slip.",
     [_in("along_tol", "double", "0.25", "metres either side of the berth depth"),
      _in("lateral_tol", "double", "0.3", "metres off the centreline"),
      _in("heading_tol_deg", "double", "15.0", "bow off straight-in"),
      _in("hull_length", "double", "1.0", "m, the boat (~1.0 x 0.6)"),
      _in("hull_beam", "double", "0.6", "m, the boat"),
      _in("finger_margin", "double", "0.35",
          "m in from the bay pitch's edge: half a 0.5 m finger, plus 0.1 m clear")]),
    ("LinedUp", "condition", "task3", True,
     "Succeeds if the boat is already on the committed bay's centreline, bow in, outside the "
     "line-up point.",
     [_in("min_along_m", "double", "2.7", "at least this far out from the face"),
      _in("lateral_tol", "double", "0.4", "metres off the centreline"),
      _in("heading_tol_deg", "double", "20.0", "bow off straight-in")]),
    ("UpdateDockBook", "action", "task3", True,
     "Logs what the boat believes about the dock bays when that changes; always succeeds.",
     []),
    ("PickVantage", "action", "task3", True,
     "Picks the next place to look at the dock from and writes it to the blackboard.",
     [_in("standoff", "double", "5.0",
          "m in front of the faces; MUST clear the fingers: finger length + half a hull"),
      _in("lead_m", "double", "2.0", "drive here first, further out, to arrive facing in"),
      _out("out", "where to look from"),
      _out("lead", "where to drive first")]),
    ("CommitSafeBay", "action", "task3", True,
     "Commits to the one GREEN bay so the berthing legs steer to it.",
     [_in("need_others_red", "bool", "false",
          "the strict rule; the tree has already applied it")]),
    ("DockWaypoint", "action", "task3", True,
     "Works out the lead-in, line-up or berth point for the committed bay and writes it to the "
     "blackboard.",
     [_in("point", "enum", "berth",
          "lead = lead-in, predock = line-up point, berth = in the slip",
          choices=["lead", "predock", "berth"]),
      _in("predock_m", "double", "3.0", "line-up point, m out from the face"),
      _in("berth_m", "double", "1.25",
          "face to BODY ORIGIN when docked: bow offset plus clearance"),
      _in("lead_m", "double", "2.0", "lead-in, m beyond the line-up point"),
      _out("out", "where the next action should drive")]),
    ("ReportDocking", "action", "task3", True,
     "Sends the docking report (the chosen bay number) to RoboCommand.",
     []),
    ("AwaitFireTarget", "action", "task3", True,
     "Waits for RoboCommand to start the fire (a window turning steady RED), re-sending the "
     "docking report until confirmed.",
     [_in("timeout_s", "double", "60.0", "give up waiting for the light"),
      _in("resend_s", "double", "10.0", "re-send the docking report this often")]),
    ("SprayUntilHit", "action", "task3", True,
     "Aims and sprays the target window every tick until it turns GREEN; fails on timeout.",
     [_in("timeout_s", "double", "60.0", "give up"),
      _in("min_green_s", "double", "0.4", "GREEN this long after RED is a hit"),
      _in("max_aim_age_s", "double", "1.0", "do not fire at an older aim point")]),
    ("ReportFirefighting", "action", "task3", True,
     "Sends the firefighting report (the window that was hit) to RoboCommand.",
     [_in("window_id_base", "int", "1",
          "RoboCommand's window_id for DockWindow.index 0 - CONFIRM with RoboNation")]),
    ("DecodeResourceRequest", "action", "task3", True,
     "Reads the colour code the target window flashes and holds it steady before believing it.",
     [_in("timeout_s", "double", "80.0", "the code runs for 60 s after the hit"),
      _in("hold_s", "double", "5.0", "the same answer for this long"),
      _in("min_frames", "int", "20", "and across this many new frames")]),
    ("ReportResourceRequest", "action", "task3", True,
     "Sends the decoded resource delivery request to RoboCommand and the UAV.",
     []),
    ("NotDropped", "condition", "fire", True,
     "Succeeds while the autonomy-drop latch (RC channel 9) is clear.",
     []),
    ("WallRangeAlive", "condition", "fire", True,
     "Succeeds while the LiDAR has measured a wall recently.",
     [_in("max_age_s", "double", "1.0", "seconds without a VALID wall range")]),
    ("AttitudeAlive", "condition", "fire", True,
     "Succeeds while attitude data is arriving.",
     [_in("max_age_s", "double", "0.5", "seconds without /crsd/attitude")]),
    ("WindowOut", "condition", "fire", True,
     "Succeeds once the shot has put the target window out (it turned GREEN).",
     []),
    ("ShotsFired", "condition", "fire", True,
     "Succeeds once at least this many bursts have gone out (dry ones count).",
     [_in("min", "int", "1", "bursts needed for success")]),
    ("SetAvoidance", "action", "fire", True,
     "Turns the autopilot's own obstacle avoidance on or off for the shot; always succeeds.",
     [_in("enable", "bool", "true", "true = avoidance on")]),
    ("StationKeep", "action", "fire", True,
     "Holds the firing range and aims at the window under GUIDED; always succeeds.",
     [_in("fire_range_m", "double", "3.22", "calibrated wall range for this window (squirt_cal)"),
      _in("window_lat_m", "double", "0.22", "window from the face centre, + LEFT (UL 0.22)"),
      _in("yaw_bias_deg", "double", "0.0", "calibrated: + aims further left"),
      _in("lateral", "enum", "centreline",
          "fingers | camera | centreline",
          choices=["fingers", "camera", "centreline"]),
      _in("window_index", "int", "0", "DockWindow.index for the camera bearing (0 = UL)"),
      _in("deadband_m", "double", "0.05", "inside this of the target: no thrust"),
      _in("kp", "double", "0.6", "m/s per m of range error, above v_min"),
      _in("v_min", "double", "0.12", "least speed the autopilot acts on"),
      _in("v_max", "double", "0.25", "speed cap, either way"),
      _in("accel", "double", "0.25", "m/s^2: gentle, thrust rocks the hull"),
      _in("min_range_m", "double", "1.5", "never drive forward closer than this"),
      _in("turn_first_deg", "double", "10.0", "heading error above this: turn only"),
      _in("square_until_m", "double", "0.5",
          "further than this from the firing range: approach square to the wall"),
      _in("range_check_m", "double", "0.5",
          "LiDAR vs camera range disagreement that stops the keep; 0 = off"),
      _in("face_setback_m", "double", "0.0",
          "the camera's faces stand this far behind the edge the LiDAR ranges")]),
    ("AwaitFiringSolution", "action", "fire", True,
     "Waits until range, aim and stillness have all held steady, then succeeds; fails on timeout.",
     [_in("range_tol_m", "double", "0.06", "+- about the range target"),
      _in("heading_tol_deg", "double", "2.0", "+- about the aim heading"),
      _in("hold_s", "double", "1.0", "everything true for this long"),
      _in("gap_s", "double", "2.0", "from the last burst's end"),
      _in("timeout_s", "double", "60.0", "give up after this")]),
    ("FireBurst", "action", "fire", True,
     "Fires one burst of water (a dry run unless fire_pump is on); succeeds when the bridge "
     "accepts it.",
     [_in("burst_s", "double", "0.5", "pump on for this long (the bridge caps it)"),
      _in("flight_s", "double", "0.8", "the water's flight after the pump stops"),
      _in("ack_timeout_s", "double", "1.5", "no answer from the bridge by then: fail")]),
    ("StrafeKeep", "action", "fire", True,
     "Holds the firing spot on the sticks under MANUAL: range, sideways and square; always "
     "succeeds.",
     [_in("fire_range_m", "double", "3.22", "LiDAR wall range to fire from (squirt_cal)"),
      _in("window_index", "int", "0", "DockWindow.index to hit (0 = upper-left)"),
      _in("lateral_bias_m", "double", "0.0",
          "calibrated: + puts the stream this much further LEFT at the window"),
      _in("nozzle_y_m", "double", "0.0", "nozzle LEFT of the centreline"),
      _in("cam_timeout_s", "double", "0.5", "window older than this: no sideways thrust"),
      _in("face_timeout_s", "double", "3.0", "square-up older than this: hold the last"),
      _in("range_check_m", "double", "0.5",
          "LiDAR vs camera disagreement that stops the keep; 0 = off"),
      _in("face_setback_m", "double", "0.0",
          "the face stands this far behind the edge the LiDAR ranges"),
      _in("deadband_range_m", "double", "0.05", "no thrust inside this range error, metres"),
      _in("deadband_lat_m", "double", "0.04", "no thrust inside this sideways error, metres"),
      _in("deadband_yaw_deg", "double", "3.0", "loose: strafing aims, not the heading"),
      _in("kp_fwd", "double", "90.0", "us per m"),
      _in("kd_fwd", "double", "60.0", "us per m/s"),
      _in("kp_lat", "double", "90.0", "us per m"),
      _in("kd_lat", "double", "30.0", "us per m/s"),
      _in("kp_yaw", "double", "4.0", "us per deg"),
      _in("kd_yaw", "double", "3.0", "us per deg/s"),
      _in("min_us", "double", "30.0", "added to every correction: the ESC deadband"),
      _in("max_us", "double", "120.0", "deflection cap (the bridge caps it too)"),
      _in("slew_us_s", "double", "200.0", "per axis"),
      _in("min_range_m", "double", "1.5", "never push ahead inside this"),
      _in("square_first_deg", "double", "10.0", "further off square: turn only"),
      _in("ki_fwd", "double", "20.0", "us per m.s (holds against a current)"),
      _in("ki_lat", "double", "30.0", "us per m.s"),
      _in("i_max_us", "double", "80.0", "integral cap")]),
    ("AwaitStrafeSolution", "action", "fire", True,
     "Waits until the strafing keep has held the firing spot steadily, then succeeds; fails on "
     "timeout.",
     [_in("range_tol_m", "double", "0.06", "+- about fire_range_m"),
      _in("lat_tol_m", "double", "0.05", "+- the window off the nozzle's line"),
      _in("yaw_tol_deg", "double", "5.0", "+- off square (loose: see StrafeKeep)"),
      _in("fire_range_m", "double", "3.22", "= StrafeKeep's"),
      _in("hold_s", "double", "1.0", "everything true for this long"),
      _in("gap_s", "double", "2.0", "from the last burst's end"),
      _in("timeout_s", "double", "60.0", "give up after this")]),
]

# (id, preset_of, fixed, label, category, desc, show_ports)
_PRESETS = [
    ("GpsWaypoint", "NavigateTo", {"target": "fix"}, "GPS waypoint", "navigation",
     "Drive straight to a GPS point, no perception (dead-reckoning waypoint)",
     ["lat", "lon", "tolerance", "avoid"]),
    ("ReturnHome", "NavigateTo", {"target": "home"}, "Return home", "navigation",
     "Drive back to the boat's home point and succeed on arrival", ["tolerance", "avoid"]),
    ("OrbitGpsPoint", "CircleBuoy", {"anchor": "fix"}, "Orbit a GPS point", "navigation",
     "Circle a GPS point once at a set radius, with no perception",
     ["lat", "lon", "radius", "points", "direction"]),
    ("HoldPosition", "HoldStation", {}, "Hold position", "navigation",
     "Hold the boat where it is; never succeeds, so a Timeout or a sibling has to end it", []),
]

_KNOWN = frozenset(t[0] for t in _BUILTINS + _CRUSADER)
_CHILDREN = {"control": "many", "decorator": "one"}


def _label(ident):
    """'WaitForRunStart' -> 'Wait for run start' (acronyms keep their case)."""
    words = re.findall(r"[A-Z]+(?![a-z])|[A-Z][a-z]*|[a-z]+|\d+", ident)
    return " ".join(w if i == 0 or (len(w) > 1 and w.isupper()) else w.lower()
                    for i, w in enumerate(words))


def _node_entries():
    nodes = []
    for nid, kind, cat, advanced, desc, ports in _BUILTINS + _CRUSADER:
        nodes.append({"id": nid, "kind": kind, "category": cat, "label": _label(nid),
                      "desc": desc, "children": _CHILDREN.get(kind, "none"),
                      "advanced": advanced, "ports": copy.deepcopy(ports)})
    nodes.append({"id": "SubTree", "kind": "subtree", "category": "task", "label": "Sub-tree",
                  "desc": "Runs another tree of this mission; it must carry _autoremap so the "
                          "blocks inside can reach the boat",
                  "children": "none", "advanced": True,
                  "ports": [_in("ID", "string", None, "the tree to run")]})
    return nodes


def _preset_entries():
    return [{"id": pid, "preset_of": base, "fixed": dict(fixed), "label": label,
             "category": cat, "desc": desc, "show_ports": list(show)}
            for pid, base, fixed, label, cat, desc, show in _PRESETS]


# ------------------------------------------------------------------------------ XML helpers

def safe_name(name):
    """The name if it is safe as a file stem ([A-Za-z0-9_-]{1,64}), else None.

    Windows device names (CON, NUL, COM1 ...) are refused too: `CON.xml` is the console there.
    """
    ok = isinstance(name, str) and _SAFE_NAME.fullmatch(name) and not _DEVICE.fullmatch(name)
    return name if ok else None


def _parse(data, comments=False):
    """The root element of `data` (str or bytes); ET.ParseError if it is not well-formed."""
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=comments))
    parser.feed(data)
    return parser.close()


def _read(path):
    with open(path, "rb") as f:
        return f.read().decode("utf-8-sig")


def _kids(el):
    """Element children only: with comments kept, `list(el)` also holds ET.Comment nodes."""
    return [c for c in el if isinstance(c.tag, str)]


def _node_id(el):
    return (el.get("ID") or el.tag) if el.tag in _GENERIC_TAGS else el.tag


def _ref(el, trees, known):
    """The id of the tree this node runs, or None when it is an ordinary block.

    `<SubTree ID="X"/>` and, in BT.CPP 4, plain `<X/>` where X is a defined tree.
    """
    if not isinstance(el.tag, str):
        return None
    if el.tag == "SubTree":
        return el.get("ID") or ""
    return el.tag if el.tag in trees and el.tag not in known else None


def _first_comment(raw):
    m = _COMMENT.search(raw)
    return m.group(1).strip() if m else ""


def _one_line(text):
    return " ".join(text.split())


def _mission_description(raw):
    first = _first_comment(raw)
    return _one_line(first[len("MISSION:"):]) if first.startswith("MISSION:") else ""


def _canon(el):
    """A whitespace-insensitive fingerprint, to tell a repeated include from a conflicting one."""
    return (el.tag, tuple(sorted(el.attrib.items())), (el.text or "").strip(),
            tuple(_canon(c) for c in _kids(el)))


def _model_ports(root):
    """{subtree id: [port dicts]} from the TreeNodesModel blocks of a parsed tree file."""
    models = {}
    for model in root.iter("TreeNodesModel"):
        for sub in model.findall("SubTree"):
            ports = [{"name": p.get("name"), "default": p.get("default"),
                      "desc": _one_line(p.text or ""), "type": _infer_type(p.get("default"))}
                     for p in sub if p.tag in ("input_port", "inout_port") and p.get("name")]
            if sub.get("ID"):
                models.setdefault(sub.get("ID"), []).extend(ports)
    return models


def _infer_type(default):
    if default is None:
        return "string"
    if default in ("true", "false"):
        return "bool"
    if _INT.fullmatch(default):
        return "int"
    return "double" if _number(default) is not None else "string"


def _number(text):
    try:
        v = float(text)
    except ValueError:
        return None
    return v if math.isfinite(v) else None


def _tunable_kind(port):
    """(kind, empty_ok) of a SubTree tunable from its TreeNodesModel port, or None for no rule.

    kind is "real", "int" or "text". BT.CPP 4.10 types a SubTree literal by its text (6 is an
    int, 6.0 a real) and refuses the load when that differs from the leaf port's type, so the
    default's own spelling says which it is. An EMPTY default is the "use the nav_* parameter"
    tunable: its value may be empty, and its type comes from the description ("decimal point",
    "whole number"). Anything else non-numeric (a direction) is text and never empty."""
    default = port.get("default")
    if default is None:
        return None
    if default == "":
        desc = (port.get("desc") or "").lower()
        kind = "real" if "decimal point" in desc else "int" if "whole number" in desc else "text"
        return kind, True
    if _REAL.fullmatch(default):
        return "real", False
    if _INT.fullmatch(default):
        return "int", False
    return "text", False


# --------------------------------------------------------------------------------- flatten

def _parse_file(path):
    try:
        return _parse(_read(path), comments=True)
    except OSError as e:
        raise ValueError("cannot read %s: %s" % (path, e.strerror or e))
    except ET.ParseError as e:
        raise ValueError("%s is not well-formed XML: %s" % (os.path.basename(path), e))


def _merge_duplicates(root, path):
    """One TreeNodesModel, and each tree once. A repeat that differs is an error, not a pick."""
    seen, model, kept = {}, None, []
    for child in list(root):
        if child.tag == "BehaviorTree":
            tid, fp = child.get("ID"), _canon(child)
            if tid in seen:
                if seen[tid] != fp:
                    raise ValueError("%s: tree '%s' is defined twice with different content"
                                     % (os.path.basename(path), tid))
                continue
            seen[tid] = fp
        elif child.tag == "TreeNodesModel":
            if model is not None:
                have = {(m.tag, m.get("ID")): _canon(m) for m in _kids(model)}
                for m in _kids(child):
                    key = (m.tag, m.get("ID"))
                    if key not in have:
                        model.append(m)
                    elif have[key] != _canon(m):
                        raise ValueError("%s: TreeNodesModel entry '%s' differs between files"
                                         % (os.path.basename(path), key[1]))
                continue
            model = child
        kept.append(child)
    root[:] = kept


def _inline(path, stack):
    real = os.path.realpath(path)
    if real in stack:
        raise ValueError("include cycle: " + " -> ".join(
            os.path.basename(p) for p in stack[stack.index(real):] + (real,)))
    root = _parse_file(path)
    if root.tag != "root":
        raise ValueError("%s: the root element is <%s>, not <root>"
                         % (os.path.basename(path), root.tag))
    spliced = []
    for child in list(root):
        if child.tag != "include":
            spliced.append(child)
            continue
        target = child.get("path")
        if not target or child.get("ros_pkg"):
            raise ValueError("%s: <include> needs a plain relative path (ros_pkg is not supported)"
                             % os.path.basename(path))
        parts = list(_inline(os.path.join(os.path.dirname(path), target), stack + (real,)))
        if parts:
            parts[-1].tail = child.tail
        spliced.extend(parts)
    root[:] = spliced
    _merge_duplicates(root, path)
    return root


def _flatten_root(path):
    return _inline(os.path.abspath(path), ())


def flatten(path):
    """The file at `path` as one self-contained XML text, every `<include>` inlined.

    BT.CPP 4 semantics: an include's path is relative to the file that holds it, and its trees
    and TreeNodesModel entries join the includer's. The including file's root attributes win.
    Raises ValueError for a missing or malformed file, an include cycle, or two different
    definitions of one tree.
    """
    root = _flatten_root(path)
    raw = _read(path)
    head = raw[:raw.find("<root")]
    lead = "".join("<!--%s-->\n" % c for c in _COMMENT.findall(head))
    return _XML_DECL + lead + ET.tostring(root, encoding="unicode") + "\n"


# ------------------------------------------------------------------------------ catalogue

def _plain_xml(el):
    """A tree as indented XML with its comments left out (they would bloat every drop)."""
    el = copy.deepcopy(el)
    for node in list(el.iter()):
        node[:] = _kids(node)
    el.tail = None
    ET.indent(el, space="  ")
    return ET.tostring(el, encoding="unicode")


def _closure(trees, start):
    """`start` and every tree it runs, directly or not, in discovery order."""
    order, todo = [], [start]
    while todo:
        tid = todo.pop(0)
        if tid in order or tid not in trees:
            continue
        order.append(tid)
        todo.extend(r for r in (_ref(el, trees, _KNOWN) for el in trees[tid].iter()) if r)
    return order


def _lib_subtree(path):
    """The catalogue entry for one lib file; ValueError if it cannot be read as a subtree."""
    root = _flatten_root(path)
    trees = {t.get("ID"): t for t in root.findall("BehaviorTree") if t.get("ID")}
    models = _model_ports(root)
    sid = next((s for s in models if s in trees), next(iter(trees), None))
    if sid is None:
        raise ValueError("no <BehaviorTree ID=...> in the file")
    first = _first_comment(_read(path))
    desc = _one_line(first[len(sid) + 1:]) if first.startswith(sid + ":") else ""
    return {"id": sid, "label": _label(sid), "desc": desc, "file": os.path.basename(path),
            "ports": models.get(sid, []),
            "xml": "\n".join(_plain_xml(trees[t]) for t in _closure(trees, sid))}


def _xml_files(directory):
    """(stem, path) for every *.xml directly in `directory`, by name; none if it is missing."""
    try:
        names = sorted(os.listdir(directory))
    except OSError:
        return []
    return [(n[:-4], os.path.join(directory, n)) for n in names
            if n.lower().endswith(".xml") and os.path.isfile(os.path.join(directory, n))]


def catalogue(bt_dir):
    """Everything the palette needs: categories, blocks, presets and the task subtrees.

    Subtrees come from `bt_dir/lib/*.xml`. A lib file that cannot be read is left out of
    `subtrees` and named in `problems`, so a broken file is visible and never half-listed.
    """
    subtrees, problems = [], []
    for stem, path in _xml_files(os.path.join(bt_dir, "lib")):
        try:
            subtrees.append(_lib_subtree(path))
        except ValueError as e:
            problems.append("lib/%s.xml: %s" % (stem, e))
    return {"version": CATALOGUE_VERSION,
            "categories": [{"id": i, "label": label, "color": color}
                           for i, label, color in _CATEGORIES],
            "nodes": _node_entries(), "presets": _preset_entries(),
            "subtrees": subtrees, "problems": problems}


# ----------------------------------------------------------------------------- list / load

def _mission_row(source, stem, path):
    row = {"name": stem, "source": source, "file": os.path.basename(path),
           "main_tree": "", "description": ""}
    try:
        raw = _read(path)
        row["description"] = _mission_description(raw)
        row["main_tree"] = _parse(raw).get("main_tree_to_execute", "")
    except (OSError, ValueError, ET.ParseError) as e:
        row["description"] = "unreadable: %s" % e
    return row


def list_missions(bt_dir, missions_dir):
    """Pre-built missions (`bt_dir/*.xml`, not lib/) then saved ones, each by name."""
    return [_mission_row(source, stem, path)
            for source, directory in (("prebuilt", bt_dir), ("saved", missions_dir))
            for stem, path in _xml_files(directory) if safe_name(stem)]


def load_mission(bt_dir, missions_dir, name, source):
    """One mission with its includes flattened, ready to edit or run."""
    res = {"ok": False, "name": name, "source": source, "xml": "", "description": "",
           "message": ""}
    directory = {"prebuilt": bt_dir, "saved": missions_dir}.get(source)
    if directory is None:
        res["message"] = "unknown source %r (use prebuilt or saved)" % (source,)
        return res
    if safe_name(name) is None:
        res["message"] = "%r is not a valid mission name" % (name,)
        return res
    path = os.path.join(directory, name + ".xml")
    if not os.path.isfile(path):
        res["message"] = "no %s mission named %s" % (source, name)
        return res
    try:
        res["xml"] = flatten(path)
        res["description"] = _mission_description(_read(path))
        res.update(ok=True, message="loaded %s" % name)
    except (OSError, ValueError) as e:
        res["message"] = str(e)
    return res


# ---------------------------------------------------------------------------------- validate

class _Check:
    """One validate() run: the findings so far and the tables every rule looks things up in."""

    def __init__(self, cat):
        self.errors, self.warnings = [], []
        self.nodes = {n["id"]: n for n in cat["nodes"]}
        self.trees = {}      # tree id -> its <BehaviorTree>
        self.declared = {}   # tree id -> tunable names its TreeNodesModel declares
        # tree id -> {tunable name: port}. The catalogue's task subtrees first (a saved mission
        # inlines their trees without the TreeNodesModel), then whatever the mission declares.
        self.tunables = {sub["id"]: {p["name"]: p for p in sub["ports"]}
                         for sub in cat.get("subtrees", [])}
        self.uses = {}       # tree id -> ids of the trees it runs
        self.ids = {}        # tree id -> ids of the blocks in it
        self.attempts = []   # (TaskAttempt element, where)

    def err(self, where, msg):
        self.errors.append(_located(where, msg))

    def warn(self, where, msg):
        self.warnings.append(_located(where, msg))

    def document(self, root):
        if root.tag != "root":
            return self.err("", "the root element must be <root>, not <%s>" % root.tag)
        if root.get("BTCPP_format") != "4":
            self.err("", 'root must carry BTCPP_format="4" (found %r)' % root.get("BTCPP_format"))
        for el in _kids(root):
            if el.tag == "BehaviorTree":
                self.define(el)
            elif el.tag == "TreeNodesModel":
                self.model(el)
            elif el.tag == "include":
                self.err("", "<include> is not allowed: a mission must be self-contained "
                             "(load it through load_mission, which flattens)")
            else:
                self.warn("", "unknown element <%s> under <root> is ignored" % el.tag)
        main = root.get("main_tree_to_execute")
        if not self.trees:
            self.err("", "no <BehaviorTree ID=...> is defined")
        if not main:
            self.err("", "root has no main_tree_to_execute")
        elif main not in self.trees:
            self.err("", "main_tree_to_execute '%s' is not a tree defined here" % main)
        for tid, el in self.trees.items():
            self.tree(tid, el)
        self.recursion()
        if main in self.trees:
            self.mission_rules(main)

    def define(self, el):
        tid = el.get("ID")
        if not tid:
            self.err("", "a <BehaviorTree> has no ID")
        elif tid in self.trees:
            self.err("", "tree '%s' is defined more than once" % tid)
        else:
            self.trees[tid] = el

    def model(self, model):
        for tid, ports in _model_ports(model).items():
            self.declared.setdefault(tid, set()).update(p["name"] for p in ports)
            self.tunables.setdefault(tid, {}).update((p["name"], p) for p in ports)

    def tree(self, tid, el):
        kids = _kids(el)
        if len(kids) != 1:
            self.err(tid, "a tree needs exactly one root block, found %d" % len(kids))
        for k in kids:
            self.node(k, [tid], tid)

    def node(self, el, path, tid):
        nid = _node_id(el)
        ref = _ref(el, self.trees, self.nodes)
        label = el.get("name") or (("SubTree " + ref) if el.tag == "SubTree" else nid)
        where = " > ".join(path + [label])
        spec = None
        self.ids.setdefault(tid, set()).add(nid)
        if ref is not None:
            self.subtree(el, ref, where, tid)
        elif nid in self.nodes:
            spec = self.nodes[nid]
            self.ports(el, spec, where)
            if nid == "TaskAttempt":
                self.attempts.append((el, where))
        else:
            self.err(where, "unknown node '%s' (not a built-in, not a Crusader block, not a "
                            "tree defined in this mission)" % nid)
        children = spec["children"] if spec else ("none" if ref is not None else None)
        self.arity(nid, children, len(_kids(el)), where)
        for k in _kids(el):
            self.node(k, path + [label], tid)

    def arity(self, nid, children, n, where):
        if nid in _ARITY:
            lo, hi = _ARITY[nid]
            if not lo <= n <= hi:
                self.err(where, "needs %d to %d children, has %d" % (lo, hi, n))
        elif children == "none" and n:
            self.err(where, "is a leaf and cannot have children (has %d)" % n)
        elif children == "one" and n != 1:
            self.err(where, "a decorator needs exactly one child, has %d" % n)
        elif children == "many" and n == 0:
            self.warn(where, "has no children, so it does nothing")

    def subtree(self, el, tid, where, owner):
        if not tid:
            self.err(where, "SubTree has no ID")
            return
        if tid in self.trees:
            self.uses.setdefault(owner, set()).add(tid)
        else:
            self.err(where, "SubTree '%s' is not defined in this mission (drop it from the Tasks "
                            "palette, which carries its tree along)" % tid)
        if el.get("_autoremap") != "true":
            self.err(where, '<SubTree> needs _autoremap="true": every Crusader block reads ctx '
                            'from the blackboard and a subtree has its own')
        declared = self.declared.get(tid)
        for attr, val in el.attrib.items():
            if attr == "ID" or _meta_attr(attr):
                continue
            if declared is not None and attr not in declared:
                self.warn(where, "'%s' is not a tunable that %s declares" % (attr, tid))
            elif attr in self.tunables.get(tid, {}):
                self.tunable(where, attr, self.tunables[tid][attr], val)

    def tunable(self, where, name, port, val):
        """A SubTree literal must be spelled as its tunable's type: BT.CPP types it by its text."""
        rule = _tunable_kind(port)
        if rule is None or _BLACKBOARD.fullmatch(val):
            return
        kind, empty_ok = rule
        if val == "":
            if not empty_ok:
                self.err(where, "'%s' must not be empty (only a tunable whose default is empty "
                                "means 'use the nav_* parameter'); its default is %r"
                         % (name, port["default"]))
        elif kind == "real" and not _REAL.fullmatch(val):
            how = ("write %s.0" % val) if _INT.fullmatch(val) else "write a number with a decimal point"
            self.err(where, "'%s' is a real, so %s (BT.CPP types a SubTree literal by its text: "
                            "%r would load as a whole number and the tree fails to load)"
                     % (name, how, val))
        elif kind == "int" and not _INT.fullmatch(val):
            how = ("write %s" % val.split(".")[0]) if _REAL.fullmatch(val) else "write a whole number"
            self.err(where, "'%s' is a whole number, so %s (BT.CPP types a SubTree literal by its "
                            "text: %r would load as a real and the tree fails to load)"
                     % (name, how, val))

    def ports(self, el, spec, where):
        by_name = {p["name"]: p for p in spec["ports"]}
        for attr, val in el.attrib.items():
            if _meta_attr(attr) or (attr == "ID" and el.tag in _GENERIC_TAGS):
                continue
            if attr in by_name:
                self.value(where, by_name[attr], val)
            else:
                self.warn(where, "unknown port '%s' (ignored)" % attr)
        for p in spec["ports"]:
            if p["required"] and p["name"] not in el.attrib:
                self.err(where, "required port '%s' is missing" % p["name"])
        sel = _FIX_SELECTOR.get(spec["id"])
        if sel and el.get(sel) == "fix":
            self.gps_point(el, where)
        if spec["id"] == "NavigateTo" and el.get("target") == "port" and "goal" not in el.attrib:
            self.err(where, 'target="port" needs goal="{entry}" naming the point to drive to')

    def value(self, where, p, val):
        name, typ = p["name"], p["type"]
        if _BLACKBOARD.fullmatch(val):
            return
        if p["direction"] == "out" or typ == "bb":
            want = "a blackboard entry like {goal}"
        elif typ == "enum" and val not in p["choices"]:
            want = "one of " + ", ".join(p["choices"])
        elif typ == "int" and not _INT.fullmatch(val):
            want = "a whole number"
        elif typ == "double" and _number(val) is None:
            want = "a number"
        elif typ == "bool" and val.lower() not in ("true", "false", "1", "0"):
            want = "true or false"
        else:
            return
        self.err(where, "port '%s' must be %s, not %r" % (name, want, val))

    def gps_point(self, el, where):
        found = {}
        for key, limit in (("lat", 90.0), ("lon", 180.0)):
            raw = el.get(key)
            if raw is None:
                self.err(where, "a GPS point needs %s" % key)
            elif not _BLACKBOARD.fullmatch(raw) and _number(raw) is not None:
                found[key] = _number(raw)
                if abs(found[key]) > limit:
                    self.err(where, "%s %s is outside -%g..%g" % (key, raw, limit, limit))
        if found.get("lat") == 0 and found.get("lon") == 0:
            self.err(where, "lat and lon are both 0: that is the default, not a place")

    def recursion(self):
        state = {}

        def visit(tid, stack):
            state[tid] = "open"
            for nxt in sorted(self.uses.get(tid, ())):
                if state.get(nxt) == "open":
                    self.err("", "SubTree recursion: "
                             + " -> ".join(stack[stack.index(nxt):] + [nxt]))
                elif nxt not in state:
                    visit(nxt, stack + [nxt])
            state[tid] = "done"

        for tid in self.trees:
            if tid not in state:
                visit(tid, [tid])

    def below(self, el, seen):
        """Every block under `el`, following SubTree references into their trees."""
        for k in _kids(el):
            yield k
            ref = _ref(k, self.trees, self.nodes)
            if ref in self.trees and ref not in seen:
                seen.add(ref)
                yield from self.below(self.trees[ref], seen)

    def mission_rules(self, main):
        reached, todo = set(), [main]
        while todo:
            tid = todo.pop()
            if tid not in reached:
                reached.add(tid)
                todo.extend(self.uses.get(tid, ()))
        ids = set().union(*(self.ids.get(t, set()) for t in reached))
        if "TaskAttempt" in ids and "WaitForRunStart" not in ids:
            self.warn("", "No WaitForRunStart: this mission starts without RoboCommand - fine for "
                          "a bench test")
        for el, where in self.attempts:
            if any(_node_id(k) == "TaskAttempt" for k in self.below(el, set())):
                self.warn(where, "TaskAttempt inside another TaskAttempt: the inner one reports "
                                 "TASK_NONE when it ends, which ends the outer attempt")


def _located(where, msg):
    return "%s: %s" % (where, msg) if where else msg


def _meta_attr(attr):
    """Attributes BT.CPP itself reads (name, _autoremap, _skipIf ...), never a port."""
    return attr == "name" or attr.startswith("_")


def validate(xml_text, cat):
    """Will this mission load and make sense? Errors block saving and starting; warnings do not."""
    check = _Check(cat)
    res = {"ok": False, "errors": check.errors, "warnings": check.warnings, "main_tree": None,
           "trees": []}
    try:
        root = _parse(xml_text.lstrip(chr(0xFEFF) + " \t\r\n"))
    except ET.ParseError as e:
        check.err("", "not well-formed XML: %s" % e)
        return res
    check.document(root)
    res.update(ok=not check.errors, main_tree=root.get("main_tree_to_execute") or None,
               trees=list(check.trees))
    return res


# -------------------------------------------------------------------------------------- save

def save_mission(missions_dir, name, xml_text, cat, overwrite=False):
    """Write a validated mission to `missions_dir/<name>.xml`; never an invalid or unsafe one."""
    res = {"ok": False, "message": "", "file": ""}
    if safe_name(name) is None:
        res["message"] = ("%r is not a valid mission name (letters, digits, _ and -, up to 64)"
                          % (name,))
        return res
    verdict = validate(xml_text, cat)
    if not verdict["ok"]:
        res["message"] = "not saved, the mission is invalid: " + "; ".join(verdict["errors"][:5])
        return res
    path = os.path.join(missions_dir, name + ".xml")
    if os.path.exists(path) and not overwrite:
        res["message"] = "%s already exists; save it under another name or overwrite" % name
        return res
    try:
        os.makedirs(missions_dir, exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8", newline="\n") as f:
            f.write(xml_text.rstrip() + "\n")
        os.replace(tmp, path)
    except OSError as e:
        res["message"] = "could not write %s: %s" % (path, e.strerror or e)
        return res
    warned = len(verdict["warnings"])
    note = " (%d warning%s)" % (warned, "" if warned == 1 else "s") if warned else ""
    res.update(ok=True, file=os.path.abspath(path), message="saved %s%s" % (name, note))
    return res
