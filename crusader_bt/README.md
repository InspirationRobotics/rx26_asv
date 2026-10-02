# crusader_bt — missions as behaviour trees

Crusader's `bt_navigator`. One coarse ROS action outside, a tree of primitive
leaves inside — the shape Nav2 uses, and the one that keeps a mission both
externally triggerable and internally editable.

```
crusader_msgs/action/SafePassage        the interface. Unchanged by any of this.
        |
bt_runner_node                          accepts the goal, owns the TIMEOUT and
        |                               CANCEL, ticks the tree at 10 Hz
behavior_trees/task1_safe_passage.xml   the mission. Editable without a compiler
        |                               (task3_disruptive.xml: Task 3, below)
src/leaves.cpp                          11 primitives: read a port, call
        |                               nav_math, publish, poll
include/crusader_bt/nav_math.hpp        ALL the geometry. stdlib only.
                                        (dock_math.hpp: the same, for Task 3;
                                        path_math.hpp + planned_leg.hpp: avoidance)
```

## The one thing to know before changing anything

**`nav_math.hpp` depends on nothing but the standard library, and it must stay
that way.** It holds every calculation that can be silently wrong — the side
rule, the orbit direction, the local projection, the "is this buoy ahead of us"
test — and because it has no dependencies it compiles and runs on a laptop in
about a second:

```bash
g++ -std=c++17 -O2 -I include -o /tmp/t test/test_nav_math.cpp && /tmp/t
```

125 checks. That loop is the difference between finding a mirrored side rule at a
desk and finding it by watching the boat pass a buoy on the wrong side, once, on
the water. If a leaf in `src/leaves.cpp` grows a formula, the formula belongs in
`nav_math.hpp` with a test, and the leaf keeps only the plumbing.

It has already earned it: the first `nextWaypoint` derived the direction of
travel from the nearest candidate buoy, which made every candidate "ahead" by
construction and would have turned the boat round to re-approach a buoy it had
already passed. Caught by `test_nav_math`, off-boat, in one run.

## The side rule, because it is inverted from what you expect

handbook 3.3.2: *a **FLASHING RED** light is passed on the vehicle's **starboard**
side; **FLASHING GREEN** on its **port**.* So for a RED buoy the boat goes down
**its port side**, and the waypoint is offset to **port** of the direction of
travel — the opposite of the buoy's required side. Swapping the two inverts the
task and still looks completely plausible on a plot, which is why the test
asserts against hand-worked cases with the compass directions spelled out.

## Leaves

| Leaf | Kind | Contract |
|---|---|---|
| `IsAutonomous` | condition | SUCCESS while the mode is in `autonomous_modes` |
| `ResolveBuoyStates` | compute | **always SUCCESS** — a dropped frame must not abort the run |
| `PublishSafePassageReport` | action | **always SUCCESS**, rate-limited internally |
| `EntryResolved` / `ExitResolved` | condition | SUCCESS once that buoy is classified |
| `NearExit` | condition | SUCCESS within `radius` of the exit — **ends the transit** |
| `SetTask` | action | publishes the task token; the course lights its beacons on this |
| `NavigateTo` | action | drive to a target and poll for arrival: planned around hazards, or straight (`avoid`), see *Planned legs* |
| `CircleBuoy` | action | one explicit hop onto the ring, then `points` waypoints once around, `cw` or `ccw`; each hop is a leg |
| `AvoidObstacles` | compute | **deprecated pass-through**: `out` = `in`. `NavigateTo` avoids by itself |
| `HoldStation` | action | **always RUNNING** — a Timeout or a sibling ends it |
| `NextWaypoint` | compute | **FAILURE = nothing left**, the loop's secondary exit |

The two "always SUCCESS" contracts are load-bearing. They sit in the reactive
guard band that is re-ticked ten times a second, and a FAILURE there propagates
to the root and ends the mission.

## Planned legs — obstacle avoidance

Avoidance lives in the tree, not in the autopilot. `NavigateTo` and `CircleBuoy`
plan around every known hazard through Nav2's `planner_server` and a costmap, and
**hold** when they cannot plan; ArduRover still flies the boat, from GUIDED
position setpoints (a carrot 3 to 5 m ahead on the path). ArduPilot's own simple
avoidance is an optional backstop, off in the baseline (`PRX1_TYPE` 0).
**Avoidance is not a safety system. The RC e-stop is.** Spec:
`docs/nav2_avoidance_spec.md`; operator view: `docs/OPERATIONS.md`.

```
include/crusader_bt/path_math.hpp       hazards, clearance, carrot, orbit ring, escape.
                                        stdlib only, test_path_math.cpp
include/crusader_bt/planner_port.hpp    the PlannerPort seam; StraightPlannerPort (a stub)
include/crusader_bt/planned_leg.hpp     the leg state machine. stdlib only,
                                        test_planned_leg.cpp, against a scripted fake planner
src/ros_planner_port.cpp                the ROS side: planner_server's action and services
src/leaves.cpp                          NavigateTo / CircleBuoy drive one PlannedLeg each
```

**`nav_mode`** (bt_runner_node, read at startup): `off | shadow | on`.

| Mode | What the legs do |
|---|---|
| `off` | Straight legs **with a guard**: every leg, `avoid` or not, drives the straight line to its goal, but **holds** (one hold, then FAILURE at `blocked_timeout_s`) if that line crosses a known hazard, and resumes only after the line has stayed clear for `nav_unblock_reset_s`. It never drives through. No planner, no Nav2 needed. **The boat default**: the `asv` image has no Nav2 until it is recreated. |
| `shadow` | The planner runs for real and the map shows its path and state, but the boat is driven by the unguarded legacy leg: **the goal setpoints are the legacy ones**, never a hold, never a failure (the state says `would HOLD`). The first thing to run on the water. |
| `on` | The planned path drives the boat. The sim rig passes this. |

`-p nav_mode:=on` and `-p nav_mode:=off` work as written: the command line parses a
bare on/off as a YAML boolean, and the node accepts that as the mode it spells.
A build **without `nav2_msgs`** (an image without Nav2) compiles a stub port and
refuses to start with anything but `off`. To check that on a machine that has Nav2:
`colcon build --packages-select crusader_bt --build-base /tmp/b_nonav --install-base /tmp/i_nonav --cmake-args -DCMAKE_DISABLE_FIND_PACKAGE_nav2_msgs=TRUE`.

**Ports.**

| Leaf | Port | Default | Meaning |
|---|---|---|---|
| `NavigateTo` | `avoid` | `true` | plan around hazards; `false` = a straight leg |
| | `exempt` | `""` | with `avoid="false"` only: `gate` (the pair being driven), `dock`, or both comma separated. With `avoid="true"` it logs a WARN and is ignored |
| | `blocked_timeout_s` | `15.0` | FAILURE after being blocked this long |
| `CircleBuoy` | `points` | `8` (was 5) | waypoints on the ring, after one explicit hop onto it |
| | `overshoot_deg` | `45` | degrees the ring runs past one full turn, the same way round (one extra point at 8 points); `0` = exactly 360. 2026-10-01: a hop counts as arrived within `tolerance` (2 m, about 19° at 6 m), so a 360° ring closed about 19° short and the referee scored 328 against its 330; the circle is complete only after the last (overshoot) point |
| | `avoid`, `blocked_timeout_s` | `true`, `15.0` | per hop |

**What a planned leg does** (`nav_mode on`, `avoid` true). Clearance is 0.8 m hard
and 2.0 m soft from a hazard's surface. The goal is pushed out of any hazard and
clipped into the 80 m costmap window; arrival is judged on the **true** goal.

| State | Meaning |
|---|---|
| `STRAIGHT` | A leg that never touches the costmap: every `off` leg, the gate crossing and the Task 3 predock and berth (`avoid="false"`), and the legacy leg under `shadow`. A straight leg whose line is blocked shows `BLOCKED`. |
| `PLANNING` | No path yet. BLOCKED follows after `nav_first_plan_wait_s`. |
| `FOLLOWING` | Steering at the carrot. Re-planned at up to 2 Hz; a new path replaces a valid one only if shorter by 20 % **and** 3 m. |
| `BLOCKED` | No valid path, or the planner is blind (`planner_server not available`, a timeout). **One hold**, the costmap cleared once at 5 s, **FAILURE at 15 s**. The 15 s only resets after 3 s of continuous FOLLOWING, so flip-flopping still times out. |
| `DEGRADED` | Stale pose, a NaN heading, or a changed datum. One hold, nothing requested, **never a failure** (the mission timeout is the backstop). Resumes by itself. |
| `ARRIVED` / `FAILED` | Terminal. |

The leg status is JSON on `/crsd/nav/leg_status` (`bt_runner_node.nav_leg_topic`),
drawn by the ground station; `/crsd/nav/hazards` is the known field the costmap
is drawn from. Both are built by the same two functions the leaves use
(`knownHazards`, `legStatusJson` in `context.hpp`), so the map, the costmap and the
leg's own checks cannot disagree about the field. A halted leg (a
reactive guard, a cancel) and the end of the mission reset the status to `IDLE`.

**Straight legs.** The gate crossing is `avoid="false" exempt="gate"`, and the Task 3
line-up and berth are `avoid="false" exempt="dock"`: those legs drive *through* a
pair or a slip that is itself a hazard, so they never ask the planner. They still
**hold** when anything else known is on the line.

**The frame.** The costmap's `map`, the tree's local plane and every hazard
coordinate are one ENU plane around one datum, owned by `nav_frames_node`
(`/crsd/datum`). With `shadow` or `on` the tree **waits for the datum** and does not
pin its origin at the first fix (a goal sent before it arrives fails at once: no fresh
pose, target not available); with `off` it pins at the first fix as it always did. A datum that later differs by more than 1 cm is never
adopted: planned legs go `DEGRADED` ("datum changed: restart bt_runner").

**`AvoidObstacles`** is a deprecated pass-through (`out` = `in`, once-per-run WARN):
`NavigateTo` avoids by itself. It stays registered so an old local XML still loads.

**Off-ROS.** `offros_runner --nav-mode off|shadow|on` (default `off`, so
`tools/task3_sim/test_e2e.py` is unchanged) swaps the planner for
`StraightPlannerPort`, an instant straight path. It is not a planner: it exists to
run the real `PlannedLeg` and the real leaves. With `shadow` or `on` the runner waits
for a `{"type":"datum","lat":..,"lon":..}` line, as the node waits for `/crsd/datum`.
It prints `{"type":"leg",...}` and `{"type":"hazards","n":k}`.

## Task 3 — Coordinated Logistics (`task3_disruptive.xml`)

One tree for all three tiers, run by the same `bt_runner_node` under the same
`SafePassage` action (the runner ticks whatever `tree_file` names; `tier` picks
how far it goes). Same split as Task 1:

```
behavior_trees/task3_disruptive.xml     find the GREEN bay, dock, fire, decode
src/task3_leaves.cpp                    15 leaves: read the Context, call dock_math
include/crusader_bt/dock_math.hpp       ALL of it: placing, the bay book, numbering,
                                        choice, berthing, the code, report JSON
test/test_dock_math.cpp                 136 checks, stdlib only, ~1 s
```

**Bays are known by where they are, not by `bay_index`.** The dock detector's
`DockObservation.bay_index` is "left to right in THIS frame": a boat that sees
bays 2 and 3 is told 0 and 1. `bt_runner_node` places every sighting in the
world (bearing + face plane + pose + `cam_x/y/yaw`) and folds it into a bay
track by position (`dock::DockBook`); bay NUMBERS come from sorting three
confirmed tracks along the dock, left to right **facing** it (`dock::layout`).

**Three numbering schemes meet here and disagree.** The CV's colours put OFF at 1
and RED at 2; RoboCommand's `Color` and `RXL_COLOR` put RED at 1. Every crossing
is a named function in `dock_math` with a test, and the mutation that casts one
into the other fails three of them.

| Leaf | Kind | Contract |
|---|---|---|
| `DockCameraAlive` | condition | the detector publishes every frame; silence = dead |
| `UpdateDockBook` | compute | **always SUCCESS**; logs when the belief changes |
| `SafeBayKnown` | condition | exactly one bay reads GREEN (strict: others RED) |
| `PickVantage` | compute | next place to look from — always outside the slips |
| `CommitSafeBay` | action | commits the GREEN bay and its number |
| `ChosenBaySafe` | condition | recent readings still say GREEN (guards the berthing legs) |
| `LinedUp` | condition | already on the centreline, facing in: skip the lead-in |
| `DockWaypoint` | compute | lead-in / line-up / berth, from the bay's CURRENT estimate |
| `DockedInBay` | condition | berth tolerances AND the whole hull inside the measured slip |
| `ReportDocking` | action | `DockingReport(bay_id)` |
| `AwaitFireTarget` | action | waits for the RED window; re-sends the report until confirmed |
| `SprayUntilHit` | action | aims every tick; cannon OFF on every exit |
| `ReportFirefighting` | action | `FirefightingReport(window_id)` |
| `TierAtLeast` | condition | Core stops after the fire |
| `DecodeResourceRequest` | action | the code (c1, c2), held 5 s before it is believed |
| `ReportResourceRequest` | action | `ResourceDeliveryRequest` to RoboCommand, and the relay to the UAV |

**Off-ROS, the whole tree runs against a simulated course**: `tools/task3_sim/`
builds these leaves and this XML with BehaviorTree.CPP from source and drives
them through `offros/offros_runner.cpp` — `bt_runner_node`'s loop with JSON lines
for topics, and a four-symbol `rclcpp` logging shim (`offros/shim`) so
`leaves.cpp` compiles unchanged. `offros/` is not in `CMakeLists.txt`.

The numbers in the tree come from the RobotX 2026 build guide's dock (1.5 m
slips between 2 m fingers, bays 2 m apart) and the boat (~1.0 × 0.6 m); the XML
header says which is which. Preconditions and limits the sim found are there and
in `tools/task3_sim/README.md`. Three need the boat: the camera, level, cannot
see either window from the berth (`cam_pitch_deg` ≈ -25, which the math
handles); `WP_RADIUS` 2.0 m parks the boat at the finger ends; and GUIDED cannot
hold a slip against a cross-current (`LOIT_RADIUS` 2 m, no strafing).

## The fixed-nozzle shot (`task3_fire_manual.xml`, `task3_fire_test.xml`)

The Task 3 nozzle is fixed, so the boat's position IS the aim. These trees are
that shot on its own - stand off the dock at the calibrated range, get the
window on the nozzle's line, wait until the hull is still, fire - to test before
it replaces `SprayUntilHit` in the full mission (docs/T3_coordinated_logistics.md).

```
behavior_trees/task3_fire_manual.xml    MANUAL, the sticks, STRAFING (the one to use)
behavior_trees/task3_fire_test.xml      GUIDED heading+speed (aims by turning; kept)
src/fire_leaves.cpp                     13 leaves
include/crusader_bt/fire_math.hpp       ALL of it: the wall filter, the aim, both keeps,
                                        the camera->body maths, steady, the gate, the
                                        burst book, the cross-check
test/test_fire_math.cpp                 117 checks, stdlib only
```

**MANUAL** (`StrafeKeep`): the sticks as RC overrides (`sticks` in the Context ->
`/crsd/rc_override`), mapped onto channels by `stick_channels` /
`stick_neutral_us` / `stick_reverse` (dp_hold's ch3/ch4/ch1). Range from the
LiDAR; sideways and square from the camera's `DockWindow` x, y, z
(`ingestFireWindows`, through `cam_mount`). `telemetry_bridge` forwards them only
in MANUAL, only on ch1/3/4, and releases them on silence. **GUIDED**
(`StationKeep`): heading + signed speed via `/crsd/guided_heading_speed`.

Inputs: `/crsd/wall_range`, `/crsd/attitude` (with yaw rate), `/crsd/pump_state`,
`/crsd/autonomy_drop`, and `dock/observations`, folded in by the runner. After
every tick the runner sends ONE stop (GUIDED) or ONE release (MANUAL) if motion
was commanded last tick and not this one - a leaf has no hook for "I stopped
being ticked".

| Leaf | Kind | Contract |
|---|---|---|
| `ModeIs` | condition | exactly this mode (heading+speed is ignored outside GUIDED) |
| `NotDropped` | condition | the drop latch (ch9) is clear (unknown counts as dropped) |
| `WallRangeAlive` / `AttitudeAlive` | condition | the streams the shot stands on |
| `StrafeKeep` | compute | MANUAL. **always SUCCESS**: surge to the range (LiDAR), sway onto the window and yaw square (camera); P+D with the ESC deadband compensated, I near the target, capped; zero while the LiDAR and camera disagree or water is in the air |
| `AwaitStrafeSolution` | action | MANUAL. RUNNING until range, window on the line, square, camera fresh, steady and not being moved all hold 1 s |
| `StationKeep` | compute | GUIDED. **always SUCCESS**: square to the wall until close, then the aim heading; banded speed; zero while the LiDAR and camera disagree or water is in the air |
| `SetAvoidance` | action | the autopilot's 2 m avoidance off for the shot, on after |
| `AwaitFiringSolution` | action | GUIDED. RUNNING until range, heading, steady, quiet and the gap all hold 1 s; FAILURE on timeout, saying which never held |
| `FireBurst` | action | one burst via `/crsd/pump_cmd`, keyed on the bridge's ACK; **DRY** unless `fire_pump`; OFF on halt |
| `WindowOut` | condition | the timing layer's hit |
| `ShotsFired` | condition | at least N bursts went out (dry ones count) |
| `StopBoat` | action | speed 0, heading held |

## Subtrees need `_autoremap="true"`

A `<SubTree>` gets its **own blackboard** and does not inherit the parent's
entries. Every leaf here reads the shared `Context` from the blackboard key
`ctx`, so without remapping, every leaf inside a subtree throws at tree-load:

    tree raised: IsAutonomous: no 'ctx' on the blackboard

Tested both ways against BT.CPP 4.9 on 2026-09-07:

| | |
|---|---|
| `<SubTree ID="Leg"/>` | `OUTCOME_FAULT`, no 'ctx' on the blackboard |
| `<SubTree ID="Leg" _autoremap="true"/>` | runs |

This matters the moment the run-level tree wraps each mission as a subtree,
which is the plan. The error message in `context.hpp` names the fix, because
this is not a thing anyone guesses.

## Read-only posture

`publish_setpoints: false` (the default) means the runner **cannot move the
boat**. Legs still poll for arrival, so a human can drive the mission by hand and
the tree follows along — that is the water test worth doing first, and it is
worth more than a version that faked arrival. Turning it true is a G1 gate.

`fire_pump: false` (the default) means it **cannot squirt**: `FireBurst` runs
dry. A separate switch, because moving and water are separate gates (G1, G7).
With both off the fire tree is a shadow that logs every solution it would act on.

## What is not here

- **Autopilot avoidance as the mechanism.** Avoidance is the tree's (*Planned
  legs*). ArduPilot's simple avoidance is an optional backstop, off in the baseline
  (`PRX1_TYPE` 0), and `OA_TYPE` stays 0: BendyRuler must never run alongside the
  planner. Where the tree cannot plan it holds; it does not hand the boat to the
  autopilot.
- **The mission timeout and cancel.** They are in `bt_runner_node`, not the XML,
  so a tree that somehow never terminates still cannot strand the boat.
- **`behaviortree_ros2`.** Not released for Humble, and not needed: the leaves
  publish a setpoint and poll a topic rather than calling an action — the same
  shape OUXT-Polaris used at RobotX 2022.
- **`buoys_passed_correctly`** in the result is still 0. `nav::passedCorrectly`
  exists and is tested, but scoring it needs the logged track, which needs the
  recorder wired in.
