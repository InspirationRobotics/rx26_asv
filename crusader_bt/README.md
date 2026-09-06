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
        |
src/leaves.cpp                          11 primitives: read a port, call
        |                               nav_math, publish, poll
include/crusader_bt/nav_math.hpp        ALL the geometry. stdlib only.
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
