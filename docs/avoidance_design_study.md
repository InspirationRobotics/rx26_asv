# Obstacle avoidance for Crusader: design study (2026-09-30)

Status: **decided 2026-09-30: Option A, see `docs/nav2_avoidance_spec.md`.** This study
is the background and the requirements; the spec is what gets built. At the time of
writing nothing in the boat stack had been changed. Written from a read-only study of
branch `sim/gazebo`, which is `task3-disruptive` plus the Gazebo sim. It was triggered by
a sim finding: with the EXIT moved behind an unpaired buoy, the boat rammed the buoy on its
way to the exit orbit.

## 1. Current state

Every position move takes one path:

    ctx_->send_setpoint              bt_runner_node.cpp:566-573
      -> telemetry_bridge._guided_cb telemetry_bridge.py:681-705
      -> SET_POSITION_TARGET_GLOBAL_INT, position only

There is no velocity path. The heading+speed path, `/crsd/guided_heading_speed`,
is clamped to 0.4 m/s.

| Leaf | Where | Moves by | Avoidance |
|---|---|---|---|
| NavigateTo | leaves.cpp:219-362 | one setpoint, re-sent when the goal moves more than `resend_m` | only when AvoidObstacles feeds it |
| CircleBuoy | leaves.cpp:425-521 | ring built once (nav_math.hpp:218-232), one setpoint per point | **none** |
| GateWaypoint, NextWaypoint, TargetAhead, PickVantage, DockWaypoint | compute only | written as goals, driven by NavigateTo | no |
| StationKeep, StopBoat, StrafeKeep | fire_leaves.cpp | heading+speed or RC override | off on purpose (the shot) |

The only avoided leg in any tree is the Task 1 Disruptive gate run-in
(task1_disruptive.xml:129-133). The entry orbit (:81) and exit orbit (:197) are
not avoided. Neither are Core Task 1 nor Task 3.

**CircleBuoy ends short.** Its first ring point is 360/n degrees from the boat's
current bearing, however far out the boat is. With n = 5 and a far entry, the
sim measured 313° swept within 8 m. The rule is "fully circle".

### What AvoidObstacles does (leaves.cpp:800-860, nav_math.hpp:538-580)

**Inputs.** Its hazards are:
- the fused plan buoys, except the current gate pair;
- confirmed tracks that match no plan buoy. That list is cleared in Core tier.

It never sees LiDAR-only objects: `target_tracker use_lidar: false`
(crusader_params.yaml:494).

**Algorithm.**
- It checks only the straight segment from the boat to the goal.
- It takes the nearest hazard closer than 5 m to that segment.
- It returns a detour point 7 m out, perpendicular to the segment.
- It recomputes every tick.

**Limits.**
- One blocker at a time. The two detour legs are never checked.
- Hazards are points, with no size.
- No blocked behaviour.
- With a stale pose it passes the goal straight through.
- It ignores red/green sides.

### Other avoidance in the stack

**ArduPilot baseline** (params/working_crusader.params):

| Param | Value | Effect |
|---|---|---|
| OA_TYPE | 0 | no path planner |
| PRX1_TYPE | 0 | no proximity input |
| AVOID_ENABLE | 3 | simple avoidance on, but with PRX1_TYPE 0 it has no data and does nothing |
| AVOID_MARGIN | 2.0 | |

G5_obstacle_avoidance.md claims PRX1_TYPE=2 was live on 2026-09-08, which
contradicts the baseline. core.launch.py:36-50 and crusader_params.yaml:413-415
assume 2. **Check the live value with param_guard.**

**LiDAR to autopilot.** lidar_cluster_node -> proximity_bridge (72 sectors,
5 Hz) -> telemetry_bridge -> OBSTACLE_DISTANCE. It only matters when PRX1_TYPE
is 2.

**Removed code.** An occupancy grid and APF were removed in v0.5. They are in
git at `8c4ffa5` and never ran on the boat.

**Nav2 and TF.** Nav2 is not installed or used anywhere. There is no TF tree:
the BT uses its own east/north frame pinned at the first GPS fix
(bt_runner_node.cpp:393-409).

**Docs that contradict each other:**
- crusader_bt/README.md:207-208 says the autopilot avoids.
- nav_math.hpp:446-447 says the tree does not.
- task1_disruptive.xml:61-64 says the tree does.

## 2. Requirements (Task 1 and Task 3)

**What is an obstacle.** Every course element (handbook 3.3.1):
- black buoys (3.3.2:23);
- other gates' buoys;
- ENTRY and EXIT when they are not the orbit anchor;
- unknown LiDAR objects, with 5 m off a vessel (hidden-3.6:13);
- the dock, its fingers and the platforms.

**Must still be possible:**
- **Gate crossing:** 2-20 m wide, red to starboard. A 2 m gate leaves about
  1.57 m of water for a 0.6 m beam, so the crossing stays a straight leg that
  exempts only its own pair.
- **Orbit:** a 6 m ring. The anchor counts only at its physical size.
- **Berth:** a 1.5 m slip, about 0.45 m per side. The line-up and berth legs are
  exempt; every earlier leg avoids the dock.

**Sensors.**
- The LiDAR covers the front 180°. `r_max` is 10 m today, a pool value; use
  25-40 m on open water.
- The camera sees to about 25 m. The UAV field arrives every 5 s.
- GPS yaw can be NaN. Then LiDAR returns must not be placed.

**Boat.** About 1.0 x 0.6 m, circumscribed radius about 0.58 m. It drives
bow-first in GUIDED. WP_SPEED is 1 m/s.

**Replanning.** Check the path every tick at 10 Hz. Replan fully when it is
invalid, or at most at 2 Hz. Use hysteresis so the boat keeps one side of an
obstacle. Remember hazards that leave the FOV: expire them only when they are in
view and unseen for a few seconds, or after about 30 s out of view.

**When blocked.** Never drive into a hazard. Hold, retry for 10-15 s, then
return FAILURE. The Task 1 gate leg already restarts on failure; the orbits need
a Retry around them.

## 3. Options

### A. Nav2 planner_server + costmap_2d called from the BT; ArduPilot follows the path

**New pieces:**
- a map->base_link TF node, with its origin shared with the BT;
- a levelled PointCloud2 from lidar_cluster_node;
- a custom costmap layer for UAV-plan buoys and per-leg exemptions;
- Nav2 params, lifecycle and launch files;
- the first ROS action client in crusader_bt.

**Install.** A new image layer, `asv_add_nav2.Dockerfile`, and **recreating the
`asv` container.** The current image `asv:socket-20260903` cannot be rebuilt
from its Dockerfile.

**Inflation problem.** Inflation is one setting per costmap. It cannot both keep
3-4 m off black buoys and fit a 2 m gate, so gates still need exemptions.

**Effort:** 2.5-3.5 weeks.

**Risks:** the container rebuild, the first TF tree (an origin mismatch is a
silent offset), and lifecycle failures. The Nav2 docs do not cover the hard
parts here (known-buoy layer, exemptions, GUIDED hand-off).

### B. Full Nav2 (bt_navigator + MPPI omni): rejected

**Reasons:**
- Humble's bt_navigator is BT.CPP v3; crusader_bt is v4.
- It needs a cmd_vel path, and the boat has only position setpoints.
- It would put two control loops on a drifting hull.
- It adds CPU load on the Orin Nano.
- It has the same container cost as A.

### C. A shared local planner inside crusader_bt (recommended)

    plan buoys + unmatched tracks --+     crsd/lidar_clusters -> bt_runner HazardBook
                                    +--> hazards[] <-------------  (world-anchored, FOV-aware decay)
                                    v
    path_math.hpp: grid A* (0.25-0.5 m cells; hard radius + soft cost band), smoothing,
                   pathClear, a moving target point along the path
                                    v
    PlannedLeg helper used by NavigateTo, every CircleBuoy hop and the Task 3 legs:
      replan when invalid or at <=2 Hz, hysteresis, blocked -> hold -> timeout -> FAILURE,
      arrival judged on the TRUE goal
                                    v
    send_setpoint (unchanged) -> telemetry_bridge -> GUIDED position

**New and changed files:**
- **new `include/crusader_bt/path_math.hpp`**, pure C++ like nav_math:
  - hazards and exemptions;
  - A* planning and path checks;
  - the target point;
  - a hazard-aware orbit ring, with an explicit hop to the ring start and
    8 points, which fixes the short circle;
  - HazardBook merge and expiry.
- **new `test/test_path_math.cpp`.**
- **bt_runner_node.cpp:** subscribes to `crsd/lidar_clusters`, projecting with
  pose and heading and dropping them when heading is NaN or the pose is stale.
- **context.hpp:** the new hazard fields.
- **leaves.cpp:** NavigateTo gets ports `avoid`, `clearance`, `soft_clearance`,
  `exempt` and `blocked_timeout_s`, and CircleBuoy plans each hop.
- **XML:** the gate crossing gets `avoid="false"` plus a pathClear guard, and
  AvoidObstacles is removed.
- **offros_runner.cpp:** fed hazards so task3_sim still works.
- **crusader_params.yaml:** new keys under `bt_runner_node`.

**Dependencies.** No new ones, no image change and no container recreation.

**Frames and setpoints.** No TF. Setpoints stay GUIDED position. The target
point sits about 5 m ahead on the path, beyond WP_RADIUS 2.0, and is re-sent
through the `resend_m` deadband.

**Obstacle map:**
- plan buoys, which never decay while listed;
- unmatched tracks;
- LiDAR clusters taken directly. `use_lidar` stays false for its pool-wall
  reason.

The hard radius is the hazard radius + 0.58 + about 0.4 m. A soft cost band
runs out to 3-4 m. Clusters wider than about 1.5 m get 5 m (the vessel rule).
Clusters within 1 m of an exempt buoy are exempt too.

**Effort.** About 1.5-2 weeks. Phase 1 alone (known buoys only) takes 3-4 days
and fixes the sim finding.

**Risks, with their mitigations:**

| Risk | Mitigation |
|---|---|
| A* correctness | off-ROS unit tests, as for nav_math |
| LiDAR clutter on water; crusader_perception has never run on the water | tuning on the water |
| Pool walls make goals unreachable | a `lidar_hazards` switch |
| The boat flipping between sides | hysteresis |

### D. ArduPilot OA_TYPE=1 BendyRuler

It is a black box, has routed round the wrong side of gate buoys (G5 §4), and
needs a reboot to take effect. At most a last-resort guard. Never run it
together with C.

## 4. Recommendation: C, in phases

**0. Sign-off.** The behavioural contract:

> Every NavigateTo and every orbit hop routes around known buoys and LiDAR
> objects, keeps at least X m clear, stops and waits when boxed in, and fails
> the leg after N seconds. Gate crossings and berth legs stay straight and
> exempt only their own two buoys or the chosen slip.

**1. path_math.hpp and its unit tests** (laptop g++). Cases:
- an empty field;
- a single blocker;
- a gap too narrow;
- a 2 m gate passable only when exempt;
- an orbit ring with a hazard on it;
- a goal inside a hazard;
- a start inside the inflation;
- hysteresis;
- expiry.

**2. PlannedLeg in NavigateTo and CircleBuoy, with known hazards; XML
updates.** Sim tests:
- task1_core regression: the referee still passes, with no contact and run time
  at most +20%;
- a new course, `task1_blocked_exit.yaml`;
- a black buoy on the entry approach;
- the panel moving the EXIT behind an unpaired buoy.

**3. LiDAR hazards.** Sim tests:
- an open_water course with a platform on the direct line (a LiDAR-only
  object);
- `r_max` at 10 and at 30;
- Task 3 with a buoy before the vantage point.

**4.** The blocked behaviour, a path display, a minimum-clearance figure in
task1_judge, and fixing the stale docs listed in §1.

**5. The boat.** Deploy with the crusader-deploy skill:
1. scp crusader_bt and crusader_params.yaml.
2. Strip CRLF.
3. Run `rebuild.sh` on the HOST.
4. Restart crsd-ros.

Before that, check that the `asv` image has behaviortree_cpp
(`docker inspect asv --format '{{.Config.Image}}'`).

Water test order:
1. Shadow mode (`publish_setpoints:=false`).
2. One buoy.
3. A 3 m gate.
4. An orbit with a buoy near the ring.

Avoidance is not a safety system; the RC e-stop is.

### Decisions for the team
1. C or A, and where avoidance lives (the README and G5 disagree).
2. The clearance numbers: hard radius, soft band, the 5 m vessel standoff, and
   which legs exempt what.
3. LiDAR hazards bypass target_tracker, with a pool profile that turns them off.
4. The blocked policy (wait time, give-up, Retry around the orbits).
5. ArduPilot: check the live PRX1_TYPE. If it is 2 with AVOID_MARGIN 2.0, the
   boat will stop in narrow gates and in the berth. Keep OA_TYPE 0.
6. `r_max` for competition (25-40 m), and CircleBuoy points 5 -> 8.
