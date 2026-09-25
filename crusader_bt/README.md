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
                                        (dock_math.hpp: the same, for Task 3)
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

57 checks. That loop is the difference between finding a mirrored side rule at a
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
| `NavigateTo` | action | publish a setpoint, poll for arrival |
| `CircleBuoy` | action | `points` waypoints once around, `cw` or `ccw` |
| `HoldStation` | action | **always RUNNING** — a Timeout or a sibling ends it |
| `NextWaypoint` | compute | **FAILURE = nothing left**, the loop's secondary exit |

The two "always SUCCESS" contracts are load-bearing. They sit in the reactive
guard band that is re-ticked ten times a second, and a FAILURE there propagates
to the root and ends the mission.

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

## What is not here

- **Obstacle avoidance.** It runs in the autopilot (`PRX1_TYPE=2`,
  `AVOID_ENABLE`) and keeps working whether or not any of this is healthy.
- **The mission timeout and cancel.** They are in `bt_runner_node`, not the XML,
  so a tree that somehow never terminates still cannot strand the boat.
- **`behaviortree_ros2`.** Not released for Humble, and not needed: the leaves
  publish a setpoint and poll a topic rather than calling an action — the same
  shape OUXT-Polaris used at RobotX 2022.
- **`buoys_passed_correctly`** in the result is still 0. `nav::passedCorrectly`
  exists and is tested, but scoring it needs the logged track, which needs the
  recorder wired in.
