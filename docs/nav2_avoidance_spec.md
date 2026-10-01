# Nav2 obstacle avoidance for Crusader: implementation spec (2026-09-30)

Status: **implementation spec for Option A**, decided 2026-09-30. Its predecessor is
`docs/avoidance_design_study.md`, which has the current state and the requirements.
Nothing is implemented yet.

Per `CLAUDE.md`, ROS-node changes need a proposal before they are made, and this spec
is that proposal. Each package below says what changes in the boat's **behaviour**.
Implementation starts only when the team says go. Recreating the boat's `asv`
container needs separate, explicit approval (§8.4).

Source tags used below:
- **[H:file]** means read in the Nav2 `humble` branch source,
  `github.com/ros-navigation/navigation2/blob/humble/<file>`, on 2026-09-30.
- **[D]** means docs.nav2.org (rolling).
- **[STVL]** means `github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/humble/README.md`.
- **[apt]** means checked with `apt-cache` in WSL Ubuntu-22.04, plus the
  packages.ros.org pool listing.
- File:line references are to branch `sim/gazebo` as it stood on 2026-09-30,
  including the uncommitted sim changes in the working tree.

## 0. Decisions in one table

| Topic | Decision |
|---|---|
| Architecture | **Nav2 `planner_server` + `costmap_2d`**, called from the BT. ArduPilot keeps control through GUIDED **position** setpoints on the existing `ctx_->send_setpoint` path (bt_runner_node.cpp:566-573). No `bt_navigator`, no `controller_server`, no velocity path. |
| Planner | **SmacPlanner2D**, id `GridBased`. NavFn is installed as a fallback but not configured. Reasons in §3.4. |
| Costmap | One global costmap: **rolling window 80 x 80 m at 0.1 m**, `global_frame: map`, `robot_base_frame: base_footprint`, `track_unknown_space: false`. |
| Known hazards | A **custom costmap layer**, `crusader_nav_layers::HazardLayer`, fed by `/crsd/nav/hazards` (`crusader_msgs/HazardArray`), which **bt_runner publishes**. Options compared in §3.3. |
| LiDAR hazards | The **Spatio-Temporal Voxel Layer (STVL)**, fed a levelled, water-gated, DBSCAN-filtered cloud from `lidar_cluster_node` in `base_footprint`. Objects decay about 3 s after they are in view and unseen, and 30 s after they leave view. This choice follows Bumblebee (§11). Fallback: `nav2_costmap_2d::ObstacleLayer`. |
| Clearance | "0.8 m hard, 2 m soft". Hazards are drawn at physical size. `robot_radius: 0.82` gives a **0.804 m inscribed radius**, because Nav2 builds the circle as a 16-gon. `inflation_radius: 2.0`, `cost_scaling_factor: 2.0`. The planner never enters cells of cost ≥ 253. |
| Frames | **One local ENU frame.** TF `map`, the BT's east/north frame and the hazard map are the same frame. `nav_frames_node` owns the datum and publishes `/crsd/datum` latched. bt_runner adopts it. TF is `map -> base_footprint` only: no `odom`, no `base_link` in TF. |
| Planned legs | One pure helper, `path::PlannedLeg`, used by NavigateTo, every CircleBuoy hop and the Task 3 legs. It drives a **carrot 3-5 m ahead** on the path and judges arrival on the **true goal**. Blocked: hold once, keep replanning, clear the costmap at 5 s, FAILURE at 15 s. Stale pose or NaN heading: hold, no FAILURE. |
| Straight legs | Gate crossing: `avoid="false" exempt="gate"`. Task 3 predock and berth: `avoid="false" exempt="dock"`. Straight setpoints, plus a BT-side check of the segment against known hazards minus the exempt ones. They never touch the costmap. |
| Rollout | `bt_runner_node.nav_mode`: `off` (straight legs with the known-hazard guard, no planner; §5.5, 2026-10-01), `shadow` (plans and displays, but drives legacy), `on`. Boat default `off`, sim `on`. Everything **builds without Nav2**, so the boat can take this code before its container is recreated. |

## 1. Architecture and data flow

```
 telemetry_bridge ──/crsd/pose (LatLonHead, 20 Hz, stamp = MAVLink receipt)──┬───────────────┬──────────────┐
        │  └─/crsd/attitude (Attitude)──────────────────────────────┐        │               │              │
        │                                                           v        v               v              v
        │                                             lidar_cluster_node  nav_frames_node   bt_runner_node  ground_station
        │   /livox/lidar (PointCloud2, livox_frame, 10 Hz) ───────────►│ (NEW, crusader_nav)  │
        │                                                           │   │ /crsd/datum ──────►│ adopts datum (latched)
        │                                                           │   │ /tf map->base_footprint (pose rate, finite heading only)
        │                     /crsd/nav/obstacle_cloud (PointCloud2,   │   v
        │                     base_footprint, 10 Hz) ───────┐        │ ┌─────────── planner_server (Nav2 humble) ──────────────┐
        │                                                    └───────┼►│ global_costmap  rolling 80 m @0.1 m, frame map          │
        │                                                            │ │  1 hazard_layer  crusader_nav_layers::HazardLayer ◄─────┼── /crsd/nav/hazards (HazardArray, 2 Hz, latched) ◄── bt_runner
        │                                                            │ │  2 stvl_layer    STVL (LiDAR, time-decayed)             │
        │                                                            │ │  3 inflation     inscribed 0.804 m, radius 2.0 m        │
        │                                                            │ │ SmacPlanner2D "GridBased"                              │
        │                                                            │ │  action /compute_path_to_pose ◄────────────────────────┼── bt_runner (≤2 Hz, + on invalid)
        │                                                            │ │  srv    /is_path_valid ◄───────────────────────────────┼── bt_runner (≤10 Hz)
        │                                                            │ │  srv    /global_costmap/clear_entirely_global_costmap ◄┼── bt_runner (once per blocked episode)
        │                                                            │ └── managed by nav_lifecycle (our own, with timeouts) ───┘
        │◄── /crsd/guided_setpoint (GuidedSetpoint: carrot / hold / straight goal) ── bt_runner
        └──► SET_POSITION_TARGET_GLOBAL_INT (GUIDED, position only) ──► ArduRover
 bt_runner ── /crsd/nav/leg_status (String JSON, ≤2 Hz + on change) ──► ground_station map (path, carrot, state)
```

| Name | Type | Producer → consumer | Frame | Rate | QoS |
|---|---|---|---|---|---|
| `/crsd/pose` | crusader_msgs/LatLonHead | telemetry_bridge → nav_frames_node, bt_runner, lidar_cluster_node, GCS | WGS84, heading in compass degrees, NaN when unresolved | 20 Hz; the stamp repeats until the next MAVLink sample (telemetry_bridge.py:196,479-482) | default reliable, depth 10 |
| `/crsd/datum` **NEW use** | crusader_msgs/LatLonHead (heading NaN, ground_speed 0, `header.frame_id: map`) | nav_frames_node → bt_runner (and any other node that wants it) | WGS84 | once, latched | reliable, transient_local, depth 1 |
| `/tf` | tf2_msgs/TFMessage: `map -> base_footprint` | nav_frames_node → STVL, costmap, planner | — | at each new pose stamp **with a finite heading** (typically 10-20 Hz) | tf2 default |
| `/crsd/nav/obstacle_cloud` **NEW** | sensor_msgs/PointCloud2 (x, y, z float32) | lidar_cluster_node → STVL | `base_footprint`: levelled body axes, origin at the hull-bottom datum | every LiDAR window (10 Hz), also when empty; **withheld while attitude is stale** | sensor_data (best effort, volatile, depth 5) |
| `/crsd/nav/hazards` **NEW** | crusader_msgs/HazardArray | bt_runner → HazardLayer | `map` | 2 Hz, only once the origin came from `/crsd/datum` | reliable, transient_local, depth 1 |
| `/compute_path_to_pose` | nav2_msgs/action/ComputePathToPose | bt_runner → planner_server | `map` | ≤ 2 Hz periodic, plus immediate replans when the path turns invalid (min gap 0.2 s) | action defaults |
| `/is_path_valid` | nav2_msgs/srv/IsPathValid | bt_runner → planner_server | `map` | ≤ 10 Hz, one call in flight | service defaults |
| `/global_costmap/clear_entirely_global_costmap` | nav2_msgs/srv/ClearEntireCostmap | bt_runner → global_costmap | — | at most once per blocked episode | service defaults |
| `/plan` | nav_msgs/Path | planner_server → rviz (debug) | `map` | each successful plan | Nav2 default |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | costmap → costmap_probe and rviz (debug) | `map` | 1 Hz, then updates | Nav2 default |
| `/crsd/nav/leg_status` **NEW** | std_msgs/String, JSON (§4.3) | bt_runner → GCS | WGS84 lat/lon | ≤ 2 Hz, plus every state change | reliable, depth 10 |
| `/crsd/nav/frames_health` **NEW** | std_msgs/String, JSON | nav_frames_node → humans | — | 1 Hz | reliable, depth 10 |
| `/crsd/guided_setpoint` | crusader_msgs/GuidedSetpoint | bt_runner → telemetry_bridge | WGS84 | event-driven, through the deadband rules in §5.3 | unchanged |

**Contexts.** All of these nodes run in the `crsd-sim` container in the sim, and in the
`asv` container on the boat. The livox driver stays in `crusader_legacy`. Nothing new
runs on the Jetson host or the laptop.

## 2. One frame: the datum

**One rule.** The TF frame `map`, the BT's local frame (`ctx_->origin`, a Vec2 with x
east and y north) and every `HazardArray` coordinate are **the same equirectangular
ENU plane**. It is the one `nav::toLocal` implements (nav_math.hpp:92-104,
R = 6371000 m), centred on one datum. A plan pose (x, y) in `map` is therefore a
`nav::Vec2` with no conversion. A second projection anywhere in the chain is a silent
offset. `crusader_common.geo` (111139 m/deg) and SITL (111318.845 m/deg) differ from it
by 0.05 % and 0.11 %, so **neither may be used to make map coordinates**.

**Owner: `nav_frames_node`.** It is new, in crusader_nav, and specified in §3.2.
- `datum_source: param` uses `datum_lat` / `datum_lon` from parameters. It is
  restart-stable; use it in competition and in the sim.
- `datum_source: first_fix` uses the first `/crsd/pose` with a finite lat/lon. The
  heading may be NaN.
- The datum is published once on `/crsd/datum`, latched, and never changes for the
  life of the process.
- The TF that the same node publishes uses that same datum, by construction.

**bt_runner adopts it.** This replaces the first-fix pin at bt_runner_node.cpp:390-409.
1. If a `/crsd/datum` arrives **before** `origin_set`: set `ctx_->origin` to it, set
   `origin_set`, set `datum_from_topic_ = true`, and call `refuseLocked()`, the same as
   the current pin path does.
2. If a pose arrives while there is still no origin:
   - with `nav_mode == off`, pin at the first fix (the legacy behaviour) and log INFO
     `no /crsd/datum: pinned at the first fix (nav_mode off, frame not shared)`;
   - otherwise ignore the pose for pinning (it still updates `boat`, which is
     meaningless until the origin is set, and stays guarded by `origin_set`). Log
     `WARN waiting for /crsd/datum — is crusader_nav nav.launch.py running?`,
     throttled to once per 5 s.
3. If a later `/crsd/datum` differs from the adopted origin by more than 0.01 m (through
   `toLocal`): log ERROR once and set `ctx_->datum_mismatch = true`. That stops the
   hazard publisher, and every planned leg goes DEGRADED (hold) with the reason
   `datum changed: restart bt_runner`. **Never re-pin mid-run.** The DockBook, `home`
   and the gate bookkeeping all hold local coordinates.
4. **Hazards are published only when `datum_from_topic_ && !datum_mismatch`.** A
   first-fix origin is not the TF frame.

**offros_runner** gets the same rule. A new stdin line type, `{"type":"datum","lat":..,"lon":..}`,
is handled as in step 1; onPose (offros_runner.cpp:267-282) follows step 2.

**The sim** passes the course origin, which is SITL home (`courses/*.yaml` `origin`),
as `datum_source:=param`. The map frame then equals the Gazebo world frame to within
0.11 % of scale. That is harmless, but do not compare `map` coordinates to Gazebo truth
below about 5 cm per 50 m.

**TF content.** `map -> base_footprint`:
- translation: `toLocal(lat, lon, datum)`, with z = 0;
- rotation: yaw only, `yaw_enu = radians(90 - heading_deg)`;
- stamp: `/crsd/pose.header.stamp`, published only when it is strictly greater than
  the last stamp sent (tf2 rejects repeated stamps);
- published **only when the heading is finite**. A NaN heading means no TF, so the
  LiDAR cannot be placed: STVL's tf2 MessageFilter drops the cloud
  ([H:nav2_costmap_2d/plugins/obstacle_layer.cpp] shows the same pattern).

`base_footprint` here means "base_link levelled": REP-103 axes with no roll or pitch.
Its origin is the hull-bottom datum that `lidar_x/y/z` are measured from
(crusader_params.yaml:310-316), so the waterline sits at z = +0.24. That breaks
REP-120's "on the ground", and §9 records it. Nothing publishes `odom` or `base_link`
into TF: planner_server and the costmap need only `global_frame` and
`robot_base_frame` ([H:nav2_costmap_2d/src/costmap_2d_ros.cpp]: the TF wait in
on_configure is for exactly that pair).

**What Nav2 does with a stale pose or a NaN heading, and why the BT must not rely
on it** (all [H]):
- `Costmap2DROS::getRobotPose` asks tf2 for the **latest** transform (stamp 0), with no
  age check (nav2_util/robot_utils.hpp: `stamp = rclcpp::Time()`). After TF stops, the
  costmap keeps rolling around the last pose.
- `PlannerServer::isPathValid` returns **`is_valid = true`** when getRobotPose fails, and
  only then. In Humble it never fills `invalid_pose_indices` (planner_server.cpp).
- `computePlan` calls `waitForCostmap()`, which spins until `isCurrent()`. With
  STVL's `expected_update_rate` set, a missing cloud or missing TF leaves the request
  **hanging**. It neither fails nor plans.

So the **BT itself** detects a stale pose (`pose_fresh`), a NaN heading and a datum
mismatch, and holds (§5.3). It times out plan requests at 1.0 s, and treats a timeout as
"cannot plan".

## 3. New packages

### 3.1 File list

| Package | Build type | Files |
|---|---|---|
| `crusader_nav` | ament_python | `package.xml`, `setup.py`, `setup.cfg`, `resource/crusader_nav`, `crusader_nav/__init__.py`, `crusader_nav/frames_core.py`, `crusader_nav/nav_frames_node.py`, `crusader_nav/costmap_probe.py`, `crusader_nav/lifecycle_core.py`, `crusader_nav/nav_lifecycle.py`, `launch/nav.launch.py`, `config/nav2_params.yaml`, `test/test_frames_core.py`, `test/test_lifecycle_core.py`, `README.md` |
| `crusader_nav_layers` | ament_cmake | `package.xml`, `CMakeLists.txt`, `hazard_layer_plugin.xml`, `include/crusader_nav_layers/raster.hpp` (pure), `include/crusader_nav_layers/hazard_layer.hpp`, `src/hazard_layer.cpp`, `test/test_raster.cpp`, `README.md` |

**Build without Nav2.** The boat's `asv` image lacks Nav2 until it is recreated.

- crusader_nav_layers' CMake does `find_package(nav2_costmap_2d QUIET)`.
  - Found: it builds and installs the plugin.
  - Not found: it prints `message(WARNING "nav2_costmap_2d not found: crusader_nav_layers installs nothing (image without Nav2)")`
    and calls only `ament_package()`.
  - `test_raster` always builds, because it is pure.
- crusader_nav is pure Python, so its Nav2 `exec_depend`s do not affect colcon.
- Both packages become `exec_depend`s of `crusader_bringup` (package.xml) so that
  `check_config` reachability passes (check_config.py:551-612). The boat's
  `rebuild.sh --packages-up-to crusader_bringup` still succeeds.

### 3.2 `crusader_nav`: the nodes

**`frames_core.py`** is pure, with no rclpy. It holds:
- `to_local(lat, lon, dlat, dlon) -> (x, y)`: nav_math's formula, verbatim. Use
  `R = 6371000.0` and `DEG = math.pi/180`; `x = (lon-dlon)*DEG*R*cos(dlat*DEG)` and
  `y = (lat-dlat)*DEG*R`.
- `yaw_from_heading(deg) -> rad`: `radians(90 - deg)`, NaN in gives NaN out.
- `quat_from_yaw(yaw) -> (x, y, z, w)`.
- `class StampGate`: `accept(stamp_ns) -> bool`, strictly increasing.

**Parity literal.** WP1's C++ test and WP3's Python test both pin this. With datum
(1.2806, 103.8557) and the point (1.2806 + 0.0009, 103.8557 + 0.0009), expect
`x = 100.050439`, `y = 100.075434`, within ±1e-6.

**`nav_frames_node.py`** (node name `nav_frames_node`) follows the house pattern:
`declare_from_config` + `PARAM_SPEC` + `run_node`, as in lidar_cluster_node.py:54-90.

| Param | Type | Default | RO/DYN | Meaning |
|---|---|---|---|---|
| `datum_source` | string | `"first_fix"` | RO | `param` or `first_fix` |
| `datum_lat` | double | 0.0 | RO | used when `datum_source: param` |
| `datum_lon` | double | 0.0 | RO | — |
| `pose_topic` | string | `"/crsd/pose"` | RO | — |
| `datum_topic` | string | `"/crsd/datum"` | RO | — |
| `map_frame` | string | `"map"` | RO | — |
| `base_frame` | string | `"base_footprint"` | RO | — |
| `health_period_s` | double | 1.0 | DYN | — |

Behaviour:
- With `param`, publish the datum in `__init__`. With `first_fix`, publish it on the
  first pose with finite lat/lon. A `param` datum with a lat/lon of (0, 0) is a
  config error: raise at startup. (0, 0) is a real place, and the default is not a
  datum.
- On each pose: if a datum is set, the heading is finite and `StampGate` accepts the
  stamp, send the TF (§2). Count NaN-heading samples for the health line.
- Health JSON every `health_period_s`:
  `{"datum":[lat,lon],"source":"param|first_fix","tf_hz":x,"last_tf_age_s":x|null,"nan_heading":n}`.
  Use null rather than a stale number when there is no TF yet.
- QoS for `/crsd/datum`: `QoSProfile(depth=1, reliability=RELIABLE, durability=TRANSIENT_LOCAL)`.

**`costmap_probe.py`** is a read-only console tool, and also the bench and water check.
It runs as `ros2 run crusader_nav costmap_probe` inside asv or crsd-sim. It subscribes
to `/global_costmap/costmap` (OccupancyGrid, where a value ≥ 99 means LETHAL or
INSCRIBED once scaled), `/crsd/datum` and `/crsd/pose`. At 1 Hz it prints each
connected blob of lethal cells within 40 m as `lat, lon, range m, bearing deg, size m`.
No ROS params; argparse takes `--max-range`.

**`lifecycle_core.py` and `nav_lifecycle.py`** (node name `nav_lifecycle`) replace
`nav2_lifecycle_manager`. *Note 2026-10-01 (why):* in the full sim rig the manager asked
planner_server to configure, planner_server did, and rmw_fastrtps 6.2.10 logged
`failed to send response to /planner_server/change_state (timeout): client will not
receive response`. Fast DDS had not matched the server's response writer with the new
client's response reader within 100 ms, so the reply was dropped. Humble 1.1.20's manager
calls `change_state` with **no timeout**, so it waited for ever, planner_server stayed
`inactive [2]`, and every planned leg held and FAILed. It depends on load and discovery
timing (WP3's isolated test never hit it), and it can happen on the boat at boot.

`lifecycle_core.py` is pure (no rclpy; `test/test_lifecycle_core.py`): `next_action(state_id)`
gives `configure` for 1 (unconfigured), `activate` for 2 (inactive), `none` for 3 (active) and
4 (finalized), and `wait` for 10..15 (transitions), 0/None (no answer) and any other id;
`TimeoutPolicy` says "recreate the clients" on every Nth consecutive timeout;
`RepeatGate` lets a log line through once and then at most every 30 s.

| Param | Type | Default | RO/DYN | Meaning |
|---|---|---|---|---|
| `node_names` | string[] | `["planner_server"]` | RO | lifecycle nodes to configure and activate |
| `check_period_s` | double | 1.0 | RO | period of the `get_state` check |
| `call_timeout_s` | double | 3.0 | RO | a call unanswered after this is dropped and asked again |
| `recreate_after_timeouts` | int | 3 | RO | consecutive timeouts before the two clients are rebuilt |

Behaviour: one timer, one call in flight per node, never a blocking call. Each period it
asks `/<node>/get_state`; on an answer it applies `next_action` through
`/<node>/change_state` (transition 1 configure, 3 activate), each call with a
`call_timeout_s` deadline. A call that misses it is dropped
(`Client.remove_pending_request`), counted, and `get_state` is asked again on the next
period; after `recreate_after_timeouts` in a row the node's two clients are destroyed and
created again (fresh discovery) and a WARN is logged. A missing service (process down or
respawning) is logged once and checked again every period. It never exits, so a
planner_server respawned by `nav.launch.py` is configured and activated again with no
other help. INFO on every state it sees change, then `planner_server active`; a repeated
condition is logged once and then at most every 30 s. planner_server waits in `activating`
(not `configuring`) for TF map to base_footprint, so calls to it time out while it waits;
that is expected and harmless.

*Bond.* Nav2's lifecycle nodes call `createBond()` on activate. Humble 1.1.20 has no
parameter to switch it off (`nav2_util/lifecycle_node.hpp` and the strings of
`libnav2_util_core.so` have none), and with no manager the bond never connects. Measured in `crsd-sim:nav2`: after the `Creating bond`
line nothing follows (no log, no deactivate) through 50 s of idle, past the bond's 10 s
connect timeout, so nothing is set in `nav2_params.yaml` for it.

### 3.3 Known hazards into the costmap: options and choice

| Option | Replacement and removal | Exemptions | Geometry | Cost | Verdict |
|---|---|---|---|---|---|
| (1) Our own OccupancyGrid → `StaticLayer` (`map_topic`, `map_subscribe_transient_local` default true, `subscribe_to_updates` default false [H:plugins/static_layer.cpp]) | a new full grid replaces the layer, and `processMap` sets `current_` | only by leaving cells out | we must pick a fixed geometry: about 1.4 MB per message for 120 m at 0.1 m, sent on every change, plus a grid-geometry coupling with the costmap | no C++ | **fallback**: workable, but heavy and coupled |
| (2) Custom layer reading a small `HazardArray` | atomic: the whole set is replaced on every message, and removed hazards' bounds are re-dirtied | possible, but not needed (§5.5) | exact circles and convex polygons, drawn in the costmap's own geometry; works with a rolling window | about 300 lines of C++ plus pluginlib | **chosen** |
| (3) Synthetic PointCloud2 → ObstacleLayer | marks persist until raytrace clears them, and clearing needs a ray through the old cell from the sensor, so a buoy the UAV moves away from **stays** | none | points only | none | **rejected**: removal is unreliable |

**`HazardLayer`** (`crusader_nav_layers::HazardLayer : public nav2_costmap_2d::Layer`)
is **stateless**: it keeps no grid of its own.
- `onInitialize()` declares `enabled` (true), `topic` (`"/crsd/nav/hazards"`) and
  `max_age_s` (5.0). It subscribes with reliable, transient_local, depth-1 QoS. The
  callback stores the message under a `std::mutex`, sets `dirty_`, saves the previous
  set's AABBs as `removed_`, and records the receipt time on the node clock.
- `updateBounds(rx, ry, ryaw, *min_x, *min_y, *max_x, *max_y)`:
  - If `!enabled_`, return.
  - Set `current_ = have_msg && (now - rx_time) <= max_age_s`. A dead bt_runner then
    makes the costmap non-current.
  - If `header.frame_id != layered_costmap_->getGlobalFrameID()`, log ERROR (throttled
    to 5 s), ignore the message and set `current_ = false`.
  - If the costmap is rolling or `dirty_`: expand the bounds by every current hazard's
    AABB (circle `r + keepout`, polygon vertices) and by `removed_`. Then clear
    `dirty_` and `removed_`.
- `updateCosts(master, i0, j0, i1, j1)`: rasterise every hazard, clipped to
  `[i0, i1) x [j0, j1)`, by calling `master.setCost(i, j, LETHAL_OBSTACLE)`. Stateless
  means each cycle redraws whatever falls inside the bounds that the cycle reset.
- `reset()` sets `dirty_ = true` and keeps the message. This matters because
  `clear_entirely` resets **every** layer ([H:src/clear_costmap_service.cpp]
  `resetLayers`).
- `isClearable()` returns false. `matchSize()` and `onFootprintChanged()` do nothing.
- `raster.hpp` is pure, templated on a `mark(i, j)` functor:
  - `rasterCircle(cx, cy, r, res, ox, oy, nx, ny, i0, j0, i1, j1, mark)` marks a cell
    when its centre is within `r + 0.7072*res` of `(cx, cy)`, **or** when the cell
    contains `(cx, cy)`. That is conservative: it never undershoots the surface.
  - `rasterConvex(poly, keepout, res, ...)` marks a cell when its centre's signed
    distance to the polygon is ≤ `keepout + 0.7072*res`.
- Plugin XML:
  `<library path="crusader_nav_layers"><class type="crusader_nav_layers::HazardLayer" base_class_type="nav2_costmap_2d::Layer"/></library>`.
  The YAML refers to it as `crusader_nav_layers::HazardLayer`.

### 3.4 Planner: SmacPlanner2D or NavFn

The Humble facts, from [H:nav2_smac_planner/src/smac_planner_2d.cpp, a_star.cpp,
collision_checker.cpp, node_2d.cpp] and [H:nav2_navfn_planner/src/navfn_planner.cpp]:

| | SmacPlanner2D | NavFn |
|---|---|---|
| Search | A*, 8-connected, Euclidean heuristic: explores a corridor | Dijkstra over the **whole** costmap by default (`use_astar: false`) |
| Time bound | `max_planning_time` (default 2.0 s); the search and the smoother share it | none |
| Soft-band knob | traversal cost `1 + cost_travel_multiplier * cost/252` per step, ×√2 on diagonals (node_2d.cpp). Humble's code default is **1.0**; [D] recommends 2.0 | fixed, `COST_NEUTRAL + 0.8*cost` |
| Untraversable | cell cost ≥ 253 (`INSCRIBED`), and `UNKNOWN` unless `allow_unknown` | ≥ 253 |
| Start in the inscribed zone | **throws** "Starting point in lethal space! Cannot create feasible plan." | clears only the start cell, and its neighbours stay blocked: in practice fails too |
| Goal off the costmap | **not bounds-checked**: `worldToMap`'s return value is ignored, which is undefined behaviour | checked, returns an empty path |
| Goal occupied | with `tolerance` > 0, returns the best node within tolerance after `max_on_approach_iterations` | searches outward by `tolerance` |
| Defaults | `tolerance` 0.125, `allow_unknown` true, `max_iterations` 1e6, `max_on_approach_iterations` 1000, `use_final_approach_orientation` false | `tolerance` 0.5, `use_astar` false, `allow_unknown` true |

**Smac 2D wins on the Orin Nano.** Its time is bounded, and the soft band is tunable.
Its two weaknesses are covered **in the BT, which must do this anyway**:
- every goal is clipped inside the window (`nav_clip_radius_m` 35 < 40 - 5);
- every goal is pushed out of known hazards;
- a start inside a known hazard's hard zone gets an escape point (§5.2).

### 3.5 `config/nav2_params.yaml`: full content

```yaml
# Crusader Nav2: planner_server + global costmap only. No bt_navigator, no controller.
# This spec is docs/nav2_avoidance_spec.md. Frames: docs §2. Never set use_sim_time
# (the sim runs on wall clock).
planner_server:
  ros__parameters:
    expected_planner_frequency: 2.0          # warn if one plan takes >0.5 s
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.5                         # m; only used when the exact goal is unreachable
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: true                    # open water is never "seen free" by a LiDAR
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 0.5                 # s, search + smoothing. BT times out at 1.0
      cost_travel_multiplier: 2.0            # Humble code default 1.0; docs advise 2.0
      use_final_approach_orientation: false
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10

global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_footprint
      rolling_window: true
      width: 80                              # m (int). BT clips goals to 35 m
      height: 80
      resolution: 0.1
      # 0.82 * cos(pi/16) = 0.804 m INSCRIBED radius: Nav2 makes the circle a
      # 16-gon (footprint.cpp makeFootprintFromRadius / calculateMinAndMaxDistances).
      # This IS the "0.8 m hard" clearance from an obstacle surface. check_config
      # asserts it against bt_runner_node.nav_hard_m.
      robot_radius: 0.82
      footprint_padding: 0.0
      track_unknown_space: false
      update_frequency: 5.0
      publish_frequency: 1.0
      always_send_full_costmap: false
      transform_tolerance: 0.5
      plugins: ["hazard_layer", "stvl_layer", "inflation_layer"]
      hazard_layer:
        plugin: "crusader_nav_layers::HazardLayer"
        enabled: true
        topic: "/crsd/nav/hazards"
        max_age_s: 5.0                       # bt_runner publishes at 2 Hz
      stvl_layer:
        plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
        enabled: true
        voxel_decay: 30.0                    # s, linear: OUT-OF-VIEW memory
        decay_model: 0
        voxel_size: 0.1                      # = resolution
        track_unknown_space: false
        max_obstacle_height: 5.0
        mark_threshold: 0
        update_footprint_enabled: false      # never erase a real return near the hull
        combination_method: 1                # max: never lowers HazardLayer's LETHAL
        obstacle_range: 40.0
        origin_z: 0.0
        publish_voxel_map: false
        transform_tolerance: 0.5
        mapping_mode: false
        map_save_duration: 60.0
        observation_sources: lidar_mark lidar_clear
        lidar_mark:
          data_type: PointCloud2
          topic: /crsd/nav/obstacle_cloud
          marking: true
          clearing: false
          min_obstacle_height: 0.0           # cloud is already water-gated upstream
          max_obstacle_height: 5.0
          expected_update_rate: 1.0          # s; silence -> costmap not current -> no plans
          observation_persistence: 0.0
          inf_is_valid: false
          clear_after_reading: true
          filter: "passthrough"              # downsampled upstream
          voxel_min_points: 0
        lidar_clear:
          enabled: true
          data_type: PointCloud2
          topic: /crsd/nav/obstacle_cloud
          marking: false
          clearing: true
          model_type: 1                      # 3D lidar (hourglass frustum)
          horizontal_fov_angle: 3.14         # rad: the 180 deg forward sector
          vertical_fov_angle: 1.82           # rad: +/-52 deg covers the inverted MID360's -52..+7
          vertical_fov_padding: 0.3          # m
          min_z: 0.5                         # m, = lidar_cluster_node r_min
          max_z: 20.0                        # m: in-view decay only where a 0.3 m buoy is reliably seen
          decay_acceleration: 5.0            # 1/s^2: in-view & unseen -> gone in ~3 s
          expected_update_rate: 1.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 2.0                # "2 m soft", from the obstacle surface
        cost_scaling_factor: 2.0             # cost 208@0.9 m, 170@1.0, 114@1.2, 63@1.5, 23@2.0
        inflate_unknown: false
        inflate_around_unknown: false

nav_lifecycle:                               # replaces lifecycle_manager_crsd_nav, 3.2
  ros__parameters:
    node_names: ["planner_server"]
    check_period_s: 1.0
    call_timeout_s: 3.0                      # floats stay floats: rcl takes no int for a double
    recreate_after_timeouts: 3

# FALLBACK if STVL misbehaves (unverified decay on empty clouds, CPU): replace
# "stvl_layer" in `plugins` with "obstacle_layer" and add:
#   obstacle_layer:
#     plugin: "nav2_costmap_2d::ObstacleLayer"
#     enabled: true
#     footprint_clearing_enabled: false
#     combination_method: 1
#     observation_sources: lidar_mark
#     lidar_mark: {topic: /crsd/nav/obstacle_cloud, data_type: PointCloud2, marking: true,
#                  clearing: false, min_obstacle_height: 0.0, max_obstacle_height: 5.0,
#                  obstacle_max_range: 40.0, observation_persistence: 0.0, expected_update_rate: 1.0}
# Marks then never decay; only the BT's blocked-recovery clear removes them. ObstacleLayer
# per-source max_obstacle_height defaults to 0.0 [H] - it MUST be set.
```

**The numbers.**
- Inflation cost at distance d from a lethal cell is 253 for d ≤ 0.804, and
  `252·exp(-2.0·(d - 0.804))` for 0.804 < d ≤ 2.0, and 0 beyond
  ([H:include/nav2_costmap_2d/inflation_layer.hpp] `computeCost`).
- With `cost_travel_multiplier` 2 at 0.1 m cells, passing 3 m of the band at an
  average cost of 100 costs as much as about 2.4 m of extra path. The planner will
  therefore detour up to a few metres to stay out of the soft band. Tune this in the
  sim (§10.2).
- Raster and voxel quantisation makes the hard limit **0.80 m, -0.07 / +0.07 m**
  measured from the true surface.

**STVL facts and their sources.**
- Parameter names: [STVL] README, humble branch.
- "If laser scanner MUST be 0" in the README refers to `decay_acceleration` for
  2D-laser frustums; we use a PointCloud2 with `model_type: 1`.
- **Verify in the sim** that frustum clearing runs when the cloud is empty. That is
  acceptance test N3 in §10.1, and it is why the fallback exists.
- Packages: `ros-humble-spatio-temporal-voxel-layer` 2.3.4 exists for amd64
  (`1jammy.20260908`) and for arm64 (`1jammy.20260910`) [apt]. It pulls in OpenVDB
  (`ros-humble-openvdb-vendor`) and the **libpcl-1.12** set, which is a large image
  delta; see §12.

### 3.6 `launch/nav.launch.py`

The launch arguments are `nav2_params` (default `<share crusader_nav>/config/nav2_params.yaml`),
`crsd_params` (default `<share crusader_bringup>/config/crusader_params.yaml`),
`datum_source` (default `""`, which means use the value in `crsd_params`), `datum_lat`
(`"0.0"`) and `datum_lon` (`"0.0"`). An `OpaqueFunction` builds three nodes:

1. `crusader_nav/nav_frames_node`, name `nav_frames_node`. Parameters: `[crsd_params]`,
   then, when `datum_source != ""`, `{datum_source, datum_lat: float, datum_lon: float}`.
   Set `respawn = (effective datum_source == "param")`: a respawn with `first_fix`
   would move the datum.
2. `nav2_planner/planner_server`, name `planner_server`. Parameters: `[nav2_params]`,
   with `respawn=True` and `respawn_delay=2.0`. `nav_lifecycle` configures and activates a
   respawned one again (measured 2026-10-01: `kill -9`, active again 7 s later).
3. `crusader_nav/nav_lifecycle`, name `nav_lifecycle`. Parameters: `[nav2_params]`, with
   `respawn=True` and `respawn_delay=2.0`. It replaces `nav2_lifecycle_manager/lifecycle_manager`
   (`lifecycle_manager_crsd_nav`) *as of 2026-10-01*: Humble's manager hung for ever on one
   lost `change_state` reply; see 3.2.

All three use `output='screen'`.

**Startup.** `planner_server` stays in *activating* until TF `map -> base_footprint`
exists (measured 2026-10-01: configure returns at once, `Activating` then waits for the
first transform). That needs a fix **with a finite heading**, because the costmap loops on
`canTransform` at 2 Hz [H:costmap_2d_ros.cpp]. Check it with
`ros2 lifecycle get /planner_server`, run in the same container.

**Where it is launched.**
- **Sim:** `gz_rig_up.sh`, between target_tracker (line 65) and bt_runner (line 71).
  WP5, §10.
- **Boat:** by hand, inside `asv`, **before** bt_runner. bt_runner is not in
  core.launch.py either. **Do not add nav.launch.py to core.launch.py** until it has
  run on the water; core.launch.py is for what has. The order is:
  `ros2 launch crusader_nav nav.launch.py datum_source:=param datum_lat:=<lat> datum_lon:=<lon>`,
  then `ros2 run crusader_bt bt_runner_node --ros-args --params-file <crusader_params.yaml> -p nav_mode:=shadow -p tree_file:=...`.
  Both run inside asv over ssh, after
  `source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash`.

## 4. Messages

### 4.1 New in crusader_msgs

Add two lines to `crusader_msgs/CMakeLists.txt` `rosidl_generate_interfaces` (after
line 40): `"msg/Hazard.msg"` and `"msg/HazardArray.msg"`. There is no new dependency:
polygons are parallel float arrays, which keeps geometry_msgs out.

`msg/Hazard.msg`:
```
# One known hazard for the costmap's HazardLayer, in the `map` frame of the
# enclosing HazardArray: ENU metres from /crsd/datum, the BT's local frame.
# Drawn LETHAL at its PHYSICAL size plus keepout_m; inflation adds the 0.8 m
# hard / 2.0 m soft clearance. Nothing here is a guess: a hazard nobody can
# place is left out, not drawn somewhere plausible.
uint8 CIRCLE=0
uint8 POLYGON=1
uint8 kind

# Where it came from - for logs and the ground station; the layer ignores it.
uint8 SRC_PLAN_BUOY=0   # a passage buoy (UAV plan, position fused with the tracker)
uint8 SRC_TRACK=1       # a confirmed track no plan buoy claimed (Core: every track)
uint8 SRC_DOCK=2        # a dock finger or deck segment, from the DockBook
uint8 SRC_KEEPOUT=3     # reserved: a RoboCommand keep-out zone (Task 4); no producer yet
uint8 source
int32 id                # plan buoy id / track id / bay track id; -1 = none

float64 x               # CIRCLE centre [m]
float64 y
float64 radius_m        # CIRCLE physical radius [m]
float64[] polygon_x     # POLYGON vertices [m], convex, counter-clockwise; len == polygon_y
float64[] polygon_y
float64 keepout_m       # extra LETHAL padding beyond the surface [m]; 0 = physical size only
```

`msg/HazardArray.msg`:
```
# The WHOLE known-hazard set. Each message REPLACES the previous one, so a hazard
# that is gone (the UAV moved a buoy, a track expired) disappears with no
# delete message. frame_id MUST be "map"; the layer refuses anything else.
std_msgs/Header header
uint32 seq              # bumps on every publish
Hazard[] hazards
```

### 4.2 No new message for the datum

`/crsd/datum` reuses `LatLonHead`, which is already the bare position carrier for
goals and origins (telemetry_bridge.py:58-64).

### 4.3 The leg status is JSON, not a message

It follows the house style of `/crsd/bt_status`. Published on `/crsd/nav/leg_status`:

```json
{"t": 12.3, "leaf": "NavigateTo", "name": "<BT node name>", "mode": "on",
 "avoid": true, "state": "FOLLOWING", "why": "", "blocked_s": 0.0,
 "goal": [lat, lon], "target": [lat, lon] | null,
 "path": [[lat, lon], ...], "plan_ms": 41.0 | null, "hop": 3, "hops": 9}
```

- `state` is one of IDLE, STRAIGHT, PLANNING, FOLLOWING, BLOCKED, DEGRADED, ARRIVED, FAILED.
- `path` is decimated to at most 60 points.
- `hop` and `hops` appear for CircleBuoy only, and are absent otherwise.
- With no leg running, publish `{"t":..,"state":"IDLE"}`, once per change.

## 5. crusader_bt changes

### 5.1 Files

| File | Change | WP |
|---|---|---|
| `include/crusader_bt/path_math.hpp` | **NEW**, pure. Hazards, geometry, carrot, orbit ring, dock polygons, NavParams | 1 |
| `include/crusader_bt/planner_port.hpp` | **NEW**, pure. The `PlannerPort` interface and `StraightPlannerPort` | 1 |
| `include/crusader_bt/planned_leg.hpp` | **NEW**, pure. The `PlannedLeg` state machine | 1 |
| `test/test_path_math.cpp`, `test/test_planned_leg.cpp` | **NEW**. g++ only, in test_nav_math's style | 1 |
| `include/crusader_bt/nav_math.hpp` | Delete `Detour` and `avoidObstacles` (504-580); fix the comment at 446-447 (§9) | 1 |
| `test/test_nav_math.cpp` | Delete the avoidObstacles block (369-425) | 1 |
| `include/crusader_bt/ros_planner_port.hpp`, `src/ros_planner_port.cpp` | **NEW**. The ROS implementation, compiled only with nav2_msgs | 2 |
| `include/crusader_bt/context.hpp` | New fields, `knownHazards()` and `LegStatus` (§5.4) | 2 |
| `src/bt_runner_node.cpp` | Datum, params, hazards publisher, leg-status publisher, planner port (§5.4) | 2 |
| `src/leaves.cpp` | NavigateTo, CircleBuoy, HoldStation helper, AvoidObstacles as a pass-through (§5.5) | 2 |
| `offros/offros_runner.cpp` | Datum line, `--nav-mode`, leg JSON (§5.7) | 2 |
| `behavior_trees/*.xml` | §5.6, including a new `nav_test_line.xml` | 2 |
| `CMakeLists.txt`, `package.xml` | Optional nav2_msgs; register the two new tests | 2 |
| `tools/task3_sim/build.py` | `run_test("test_path_math")` and `run_test("test_planned_leg")` (lines 253-255); add the three headers to the staleness list (line 169) | 2 |
| `README.md` | §9 | 2 |

### 5.2 `path_math.hpp`: the exact API (namespace `crusader_bt::path`)

```cpp
// Pure: <algorithm> <cmath> <cstdint> <limits> <string> <vector>, nav_math.hpp, dock_math.hpp.
using nav::Vec2;
enum class HazardKind { Circle = 0, Polygon = 1 };          // == Hazard.msg kind
enum class HazardSource { PlanBuoy = 0, Track = 1, Dock = 2, Keepout = 3 };  // == SRC_*
struct Hazard {
  HazardKind kind = HazardKind::Circle; HazardSource source = HazardSource::PlanBuoy;
  int id = -1; Vec2 c; double r = 0.0; std::vector<Vec2> poly; double keepout = 0.0;
};
struct NavParams {                         // defaults == crusader_params.yaml bt_runner_node nav_*
  double hard_m = 0.8, soft_m = 2.0;
  double buoy_radius_m = 0.30, track_radius_m = 0.30, exempt_radius_m = 1.0;
  double lookahead_m = 5.0, lookahead_min_m = 3.0, max_chord_dev_m = 0.3, wp_radius_m = 2.0;
  double replan_period_s = 0.5, min_request_gap_s = 0.2, check_period_s = 0.1;
  double plan_timeout_s = 1.0, first_plan_wait_s = 1.0; int invalid_confirm = 2;
  double hysteresis_frac = 0.2, hysteresis_m = 3.0, goal_replan_m = 1.0, clip_radius_m = 35.0;
  double clear_after_s = 5.0, unblock_reset_s = 3.0, escape_margin_m = 0.5, goal_margin_m = 0.3;
  double local_check_tol_m = 0.1, orbit_max_push_m = 3.0, orbit_clear_m = 1.4;
  double dock_finger_len_m = 2.0, dock_finger_w_m = 0.5, dock_slip_w_m = 1.5, dock_deck_depth_m = 1.0;
};
/// Signed distance from p to the hazard's LETHAL boundary (surface + keepout); < 0 inside.
double clearance(const Hazard & h, Vec2 p);
double minClearance(const std::vector<Hazard> & hz, Vec2 p);           // +inf when empty
/// Plan buoys (radius buoy_radius_m), unmatched confirmed tracks (track_radius_m), and the
/// dock (dockHazards). Consumed buoys are NOT dropped - a buoy dealt with is still there.
std::vector<Hazard> buildHazards(const std::vector<nav::Buoy> & buoys,
  const std::vector<nav::Buoy> & tracks, const dock::DockBook & dock, int dock_min_obs,
  const NavParams & p);
/// Per DockBook track with n >= min_obs and a usable outward(): two finger rectangles
/// (lateral +/-(slip/2 + finger_w/2) along right = starboardOf(-out), 0..finger_len out from
/// the face) and one deck rectangle (-deck_depth..0 along out, lateral +/-(slip/2 + finger_w)).
std::vector<Hazard> dockHazards(const dock::DockBook & b, int min_obs, const NavParams & p);
/// Removes PlanBuoy hazards whose id is listed, Track hazards within exempt_radius of a
/// listed buoy, and every Dock hazard when exempt_dock.
std::vector<Hazard> exempt(const std::vector<Hazard> & hz, const std::vector<int> & buoy_ids,
  bool exempt_dock, double exempt_radius_m);
bool segmentClear(Vec2 a, Vec2 b, const std::vector<Hazard> & hz, double clearance_m,
  double step_m = 0.1);
/// First index >= from_i whose point is within clearance_m of any hazard; -1 = none.
int firstConflict(const std::vector<Vec2> & path, std::size_t from_i,
  const std::vector<Hazard> & hz, double clearance_m);
double pathLength(const std::vector<Vec2> & path, std::size_t from_i = 0);
/// Closest vertex, searching forward from `hint` (never backwards by more than 2 m of arc).
std::size_t closestIndex(const std::vector<Vec2> & path, Vec2 p, std::size_t hint);
struct Moved { Vec2 p; bool moved = false; bool ok = true; };
/// If goal is within hard + goal_margin of any hazard, walk it toward `from` in 0.1 m steps
/// until clear; ok=false if the whole segment is blocked (goal returned unchanged).
Moved pushGoalOut(Vec2 goal, Vec2 from, const std::vector<Hazard> & hz, double hard,
  double margin);
Vec2 clipToWindow(Vec2 goal, Vec2 from, double clip_r);
struct Escape { bool needed = false; bool ok = false; Vec2 p; };
/// needed when minClearance(boat) < hard. Candidates: radii wp_radius + {0.5, 1.0, 2.0},
/// 16 bearings. Valid: clearance >= hard + escape_margin AND along boat->cand (0.1 m steps)
/// no hazard gets closer than min(its clearance at boat, hard) - 0.05. Pick min
/// |cand - goal| + 0.5 |cand - boat|. ok=false if none.
Escape escapeStart(Vec2 boat, Vec2 goal, const std::vector<Hazard> & hz, const NavParams & p);
struct Carrot { Vec2 p; std::size_t idx = 0; bool is_end = false; };
/// Farthest point within [lookahead_min, lookahead] of arc from the boat's projection
/// such that every path vertex between is within max_chord_dev of the chord AND the chord
/// boat->p is segmentClear at hard - local_check_tol. Falls back to lookahead_min. The end
/// of the path when less than lookahead_min remains (is_end = true).
Carrot carrot(const std::vector<Vec2> & path, Vec2 boat, std::size_t hint,
  const std::vector<Hazard> & hz, const NavParams & p);
/// Switch to the new path only if the current one is invalid, or the new one is shorter by
/// BOTH >= frac AND >= abs_m. Lengths are path length + |path end - true goal|.
bool preferNew(bool cur_valid, double cur_len, double new_len, double frac, double abs_m);
/// n+1 points: ring[0] = on the anchor->from bearing at `radius` (the explicit hop target),
/// ring[k] = a0 + k*step, ring[n] == ring[0]. cw = decreasing ENU angle (as nav::orbit).
std::vector<Vec2> orbitRing(Vec2 anchor, Vec2 from, double radius, int n, bool cw);
/// Each ring point (not 0 or n) with minClearance < orbit_clear_m: push radially out in
/// 0.25 m steps up to orbit_max_push_m; still short -> drop it (logged by the caller).
std::vector<Vec2> adjustRing(const std::vector<Vec2> & ring, Vec2 anchor,
  const std::vector<Hazard> & hz, const NavParams & p, int * dropped = nullptr);
```

### 5.3 `planner_port.hpp` and `planned_leg.hpp`

```cpp
namespace crusader_bt { namespace path {
enum class Reply { None, Pending, Ok, Failed, Unavailable };
struct PlanReply { std::uint64_t seq = 0; Reply status = Reply::None;
                   std::vector<Vec2> path; double planning_s = -1.0; std::string why; };
struct CheckReply { std::uint64_t seq = 0; Reply status = Reply::None; bool valid = false; };
/// Non-blocking. Called from the tick thread; implementations lock their OWN mutex and
/// never ctx->mu. A new request supersedes any in flight (its reply is dropped by seq).
class PlannerPort {
 public:
  virtual ~PlannerPort() = default;
  virtual bool ready() = 0;                                          // server + service up
  virtual std::uint64_t requestPlan(Vec2 start, Vec2 goal) = 0;      // 0 = not sent
  virtual PlanReply lastPlan() = 0;
  virtual std::uint64_t requestCheck(const std::vector<Vec2> & path) = 0;
  virtual CheckReply lastCheck() = 0;
  virtual void clearCostmap() = 0;                                   // fire and forget
};
/// offros and tests: instant straight [start, goal] path; checks always valid.
class StraightPlannerPort : public PlannerPort { /* trivial, header-only */ };

enum class Mode { Off, Shadow, On };
enum class LegState { Idle, Straight, Planning, Following, Blocked, Degraded, Arrived, Failed };
const char * legStateName(LegState s);             // "IDLE", "STRAIGHT", ...
struct LegConfig { bool avoid = true; double tolerance = 2.0; double resend_m = 1.5;
                   double blocked_timeout_s = 15.0; std::vector<int> exempt_buoys;
                   bool exempt_dock = false; };
struct LegInputs { double now_s = 0.0; bool pose_fresh = false; Vec2 boat;
                   double heading_deg = std::nan(""); bool goal_ok = false; Vec2 goal;
                   std::vector<Hazard> hazards; bool datum_ok = true; };
enum class Result { Running, Success, Failure };
struct LegOutput { Result result = Result::Running; bool send = false; Vec2 setpoint;
                   LegState state = LegState::Idle; std::string why; double blocked_s = 0.0;
                   std::vector<std::string> log; };
class PlannedLeg {
 public:
  PlannedLeg(const NavParams & p, Mode m, PlannerPort * port);   // port may be null (Off)
  LegOutput start(const LegConfig & c, const LegInputs & in);    // = onStart
  LegOutput step(const LegInputs & in);                           // = onRunning, each tick
  void halt();                                                    // forget everything
  const std::vector<Vec2> & path() const; bool hasTarget() const; Vec2 target() const;
  LegState state() const; double planMs() const;
};
}}
```

**Leg behaviour.** This is the contract the unit tests pin. It is specified per tick;
`dt` is the time since the last step.

**A. Straight in Shadow mode.** (Off mode was A until 2026-10-01. It is **B** now: see the
decision at the end of §5.5.)
- This is **exactly the legacy NavigateTo** (leaves.cpp:241-305). `start` sends the
  goal. `step` resends when the goal has moved more than `resend_m`. With a fresh
  pose and `|goal - boat| <= tolerance`, the result is Success. With a stale pose,
  Running.
- The state is STRAIGHT.

**B. On mode with `avoid=false`, and Off mode with any `avoid`: a straight leg with a guard.**
1. Send exactly as in A.
2. Each tick, take `guard = exempt(hazards, exempt_buoys, exempt_dock, exempt_radius_m)`
   and check `segmentClear(boat, goal, guard, hard_m - local_check_tol_m)`.
3. If the check fails, enter BLOCKED: send a **hold** (`setpoint = boat`) once and
   accumulate `blocked_s`. When the segment has stayed clear for `unblock_reset_s`
   (3 s, the rule a planned leg uses; so a track that flickers on and off the line
   cannot make the boat lurch at it), resend the goal and return to STRAIGHT.
   `blocked_s` counts while the leg is held, clear or not, and only that resume zeroes
   it, so a flicker still ends in FAILURE at `blocked_timeout_s`. A straight leg makes
   no planner calls and no costmap clears.
4. The heading is not needed, so there is no DEGRADED here. A stale pose returns
   Running, as legacy does.
5. In Shadow mode, the same check is logged as `would hold`, and nothing more.

**C. On mode with `avoid=true`: a planned leg.**

*0. Degraded.* A stale pose, a NaN heading or `!datum_ok` puts the leg in DEGRADED.
- On entry only, send a hold at the last fresh boat position, if one exists.
- Request nothing.
- `blocked_s` does not grow, and there is no FAILURE. The mission timeout and the
  guard band are the backstop.
- On recovery, go to PLANNING and force a replan.

*1. Arrival.* A fresh pose with `|goal_true - boat| <= tolerance` gives ARRIVED and
Success.

*2. Replies.* Read `port->lastPlan()`. If its seq is the pending one and it is no
longer Pending:
- Ok:
  - The candidate path is `[escape?] + reply.path`.
  - Reject it if `firstConflict(path, 0, hazards, hard - tol) >= 0`, with the reason
    `plan crosses a known hazard`; the costmap lagged the field.
  - With no current path, or a current path that is invalid, **adopt** it.
  - Otherwise adopt it only if `preferNew(...)`.
- Failed or Unavailable: count it as a plan failure.
- If the request has been Pending longer than `plan_timeout_s`: count a plan failure
  with the reason `planner timeout (costmap not current?)` and clear the pending seq.

*3. Validity of the current path, while FOLLOWING.*
- Local check: `firstConflict(path, closest, hazards, hard - tol) >= 0` makes it
  **invalid now**.
- Remote check: when nothing is in flight and `check_period_s` has passed, call
  `requestCheck(path[closest..])`. On the reply, valid resets `invalid_streak`;
  invalid increments it.
- Invalid is confirmed when the local check fails, or when
  `invalid_streak >= invalid_confirm`.
- Confirmed invalid: drop the path, enter BLOCKED (hold once) and set
  `need_replan`.
- A plan **failure** (Failed, Unavailable, or a timeout) while FOLLOWING:
  - Failed with a currently valid path: keep following and log it.
  - Timeout or Unavailable: BLOCKED. The planner is blind, so we hold.

*4. Request.* With nothing pending, request when:
- `need_replan` is set, or there is no path, or `now - last_req >= replan_period_s`;
- and `now - last_req >= min_request_gap_s`.

Steps:
1. If `!port->ready()`, count a plan failure with the reason
   `planner_server not available`.
2. Otherwise:
   - `g = clipToWindow(pushGoalOut(goal).p, boat, clip_radius_m)`;
   - `e = escapeStart(boat, g, hazards, p)`;
   - `start = e.needed && e.ok ? e.p : boat`;
   - `requestPlan(start, g)`.
3. If `e.needed && !e.ok`, count a plan failure with the reason `boxed in`.
4. The goal moving more than `goal_replan_m` since the last request sets `need_replan`.

*5. States.*
- **PLANNING** has no path yet. A plan failure, or `now - start >= first_plan_wait_s`,
  moves it to BLOCKED (hold once). It keeps waiting for, and requesting, plans.
- **FOLLOWING**:
  - If an escape is active, send the escape point. It is at least `wp_radius + 0.5` m
    away, so ArduRover does not count it as reached. The escape ends once
    `minClearance(boat) >= hard`.
  - Otherwise take `c = carrot(...)` and **send** when there is no carrot yet, or
    `|c.p - last_sent| > resend_m`, or
    `(!c.is_end && |boat - last_sent| < wp_radius_m + 0.5)`. The last condition stops
    ArduRover reaching an intermediate carrot and loitering. WP_RADIUS is 2.0
    (params/working_crusader.params:913).
  - Arrival is still judged in step 1, on the **true** goal, never on the path end or
    the carrot.
- **BLOCKED**:
  - `blocked_s += dt`.
  - At `clear_after_s`, call `port->clearCostmap()` once per episode.
  - At `blocked_timeout_s`, FAILED and Failure. The hold stands, and the tree decides
    what happens next.
  - An adopted valid path returns the leg to FOLLOWING.
  - `blocked_s` resets to 0 only after `unblock_reset_s` of continuous FOLLOWING, so
    flip-flopping still times out.

**D. Shadow mode with `avoid=true`.**
- Run C completely, with every port call, so the plans, checks and states are real.
- But `send` follows **A** (legacy).
- There is never a Failure from blocking, and never a hold.
- `why` carries `shadow: would HOLD (...)` when C would have held.

`halt()` sets `state = Idle` and drops the path and the pending seqs. In-flight replies
are then ignored by seq.

### 5.4 Runner and context

**`context.hpp`** adds the following after line 251, beside `publish_setpoints`:

```cpp
  // ---- obstacle avoidance (path_math.hpp / planned_leg.hpp) ----
  path::Mode nav_mode = path::Mode::Off;
  path::NavParams nav;                          // bt_runner_node nav_* params
  std::shared_ptr<path::PlannerPort> planner;   // null when nav_mode off or no Nav2 build
  bool datum_mismatch = false;
  struct LegStatus { std::string leaf, name, state = "IDLE", why; bool avoid = false;
    double blocked_s = 0.0; bool have_goal = false; nav::Vec2 goal; bool have_target = false;
    nav::Vec2 target; std::vector<nav::Vec2> path; double plan_ms = -1.0; int hop = -1, hops = 0; };
  LegStatus leg;
  std::uint32_t leg_seq = 0;                    // bumped by a leaf whenever it writes `leg`
```

It also adds `inline std::vector<path::Hazard> knownHazards(const Context & c)` (CALL
UNDER mu), which returns
`path::buildHazards(c.buoys, c.obstacles, c.dock, c.dock_min_obs, c.nav)`. It is **the
one function** that both the hazard publisher and the leaves call, so the costmap and
the BT's local checks can never disagree about the known field.

**`bt_runner_node.cpp`:**

1. **Params.** Declare these in the constructor, after line 219, with defaults from
   `path::NavParams{}`. The keys and types are in §5.8. Validate `nav_mode`, which
   must be one of `off|shadow|on`, and throw otherwise.
2. **Planner port.**
   - If `nav_mode != off`: `ctx_->planner = makeRosPlannerPort(this, ...)`.
   - That factory returns nullptr when built without nav2_msgs. Then **throw**
     `bt_runner was built without nav2_msgs (image without Nav2); nav_mode must be off`.
3. **Datum.** Subscribe to `/crsd/datum` (`nav_datum_topic`) with
   `QoS(1).reliable().transient_local()`, implementing §2. onPose (390-409) changes as
   §2 describes.
4. **Hazard publisher.**
   - A wall timer at `nav_hazard_rate_hz` takes `lock(ctx_->mu)`.
   - If `!datum_from_topic_ || datum_mismatch`, it returns.
   - Otherwise it calls `knownHazards`, fills the `HazardArray` (`frame_id = nav_map_frame`,
     `stamp = now()`, `++seq`) and publishes on `nav_hazards_topic` with
     `QoS(1).reliable().transient_local()`.
   - It runs whether or not a goal is active, so the costmap is current before the
     first leg.
5. **Leg status.**
   - Publish in the tick loop, right after `stopIfSilent(false)` (around line 950),
     when `ctx_->leg_seq` changed and at least `1/nav_status_hz` has passed, or when
     `leg.state` changed. A state change always publishes.
   - Convert to lat/lon with `nav::toLatLon(.., origin)`, and decimate `path` to at
     most 60 points.
   - At goal start (execute(), near line 900) and at every exit, set `leg = {}` and
     `leg.state = "IDLE"`.
6. **Feedback.** `publishFeedback` adds `fb->warning = "nav BLOCKED <why>"` when
   `leg.state == "BLOCKED"`. The field already exists, so no message change is needed.

**`ros_planner_port.{hpp,cpp}`.** Compile these only when `CRSD_HAVE_NAV2` is defined.
The header always declares
`std::shared_ptr<path::PlannerPort> makeRosPlannerPort(rclcpp::Node *, const std::string & map_frame, const std::string & planner_id, const std::string & action, const std::string & valid_srv, const std::string & clear_srv);`.
Without Nav2, a stub `.cpp` path returns nullptr.

- **Callback group.** One dedicated `MutuallyExclusive` group for
  `rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>` and the
  `IsPathValid` / `ClearEntireCostmap` clients. The node already spins on a
  MultiThreadedExecutor (line 1213), so the callbacks never touch the tick thread
  (worker_).
- **`ready()`** is `action_server_is_ready() && valid_client->service_is_ready()`.
  It is non-blocking.
- **`requestPlan`:**
  - Set `goal.goal` and `goal.start` with `header.frame_id = map_frame` and
    `stamp = 0` (latest). Positions are the Vec2 x/y unchanged; that is the shared
    datum. Orientation is identity (Smac 2D ignores it).
  - Set `use_start = true` and `planner_id = "GridBased"`.
  - Call `async_send_goal` with goal-response and result callbacks.
  - On SUCCEEDED with at least 2 poses: Ok. ABORTED: Failed, with
    `why = "planner aborted (no path, start in lethal space, or costmap)"`.
    Rejected: Failed. A stale seq: ignore.
  - Supersede: `async_cancel_goal` on the previous handle, if there is one.
- **`requestCheck`:** an `IsPathValid` request with the path in `map_frame`. The reply
  sets `valid`.
- **`clearCostmap`:** `ClearEntireCostmap` through `async_send_request`; log the result.

### 5.5 Leaves (`leaves.cpp`)

The helper-function pass is mandatory. Extract **`sendSetpoint(ctx, Vec2, const char * what)`**,
which converts with `toLatLon` under the lock, publishes when `publish_setpoints`, and
logs with `[NOT SENT]` otherwise. Use it in NavigateTo, CircleBuoy and HoldStation;
today three copies exist, at lines 253-260, 503-517 and 539-541.

Also extract **`legInputs(ctx, goal_ok, goal) -> LegInputs`**. It fills
`now_s`, `pose_fresh`, `boat`, `heading_deg` and `datum_ok = !datum_mismatch`, plus
`knownHazards`, all under one lock.

**NavigateTo** (lines 219-362) keeps every existing port and its behaviour in Off mode.
The new ports:

| Port | Type | Default | Meaning |
|---|---|---|---|
| `avoid` | bool | true | plan around hazards (needs `nav_mode` shadow or on) |
| `exempt` | string | `""` | comma list: `gate` (ctx gate_red_id/gate_green_id), `dock`. Used **only** with `avoid=false`; with `avoid=true` it logs WARN once and is ignored |
| `blocked_timeout_s` | double | 15.0 | FAILURE after this long blocked |

- **onStart:** resolve as now. Build the LegConfig from the ports and construct
  `leg_ = PlannedLeg(ctx->nav, ctx->nav_mode, ctx->planner.get())`. Call
  `out = leg_.start(cfg, legInputs(...))` and apply the output (send, log, write
  `ctx->leg`, `++leg_seq`).
- **onRunning:** re-resolve the goal every tick (unchanged; ports can move). Call
  `out = leg_.step(...)` and apply it. On Success, consume the buoy (lines 299-301,
  unchanged).
- **onHalted (new):** `leg_.halt()`, then set `ctx->leg.state = "IDLE"` and bump
  `leg_seq`.

**CircleBuoy** (lines 425-521):

| Port | Type | Default | Change |
|---|---|---|---|
| `points` | int | **8** (was 5) | — |
| `avoid` | bool | true | new |
| `blocked_timeout_s` | double | 15.0 | new, per hop |

- **onStart:** the anchor as now. Then `ring_ = adjustRing(orbitRing(a, boat, radius, points, cw), a, knownHazards, nav, &dropped)`.
  Log the sweep as `nav::sweepDeg(a, ring[1..], ring[0])`, which must be ±360, and the
  number dropped. Hop `i_ = 0` is **the explicit hop to the ring start**. Each hop is
  one `PlannedLeg` (restarted per hop) with goal `ring_[i_]` and tolerance from the
  port.
- **onRunning:** step the leg. On Success, increment `i_`; when `i_ > points`, return
  SUCCESS. On Failure, return FAILURE, and the XML Retry restarts the orbit.
- **This applies in all modes.** In Off mode the hops are straight legacy hops, so the
  313° short circle is fixed even without Nav2.
- Delete `nav::orbit`? **No.** test_nav_math pins it, and it stays as dead-but-tested
  code. CircleBuoy simply stops calling it.

**HoldStation** uses `sendSetpoint`; its behaviour is unchanged.

**AvoidObstacles** (lines 800-860) becomes a **pass-through**: `out = in`, SUCCESS.
On the first tick it logs `WARN AvoidObstacles is deprecated and does nothing: NavigateTo avoids by itself`
once. It stays registered (line 1126), so an old local XML still loads. `nav::avoidObstacles`
is deleted (WP1).

**Decision, 2026-10-01: in `nav_mode off` every leg is a guarded straight leg.**
Behaviour A (the unguarded legacy leg) now exists only for `shadow` with `avoid=false`. In
Off, a `NavigateTo` or `CircleBuoy` hop with `avoid=true` behaves as B: if the straight line
to the goal crosses a known hazard it holds once, accumulates `blocked_s`, resumes only after
`unblock_reset_s` of clear line, and FAILS at `blocked_timeout_s` (15 s). `avoid=false` legs,
which are straight in every mode, get the same guard in Off with their `exempt` lists.
- *Why.* Off is the boat's default until the `asv` container has Nav2. The run-in's
  `AvoidObstacles` ReactiveSequence has been removed from `task1_disruptive.xml` and
  `AvoidObstacles` is a pass-through, as this section specifies, so an Off leg that only
  drove its line would go straight through any black buoy or LiDAR track on it. Putting the
  legacy detour back is not the answer: it was removed per this spec, and it also caused the
  2026-09-30 collision, through the 3 m `run_in` tolerance. With no planner to steer round
  a hazard, **holding is safer than driving through**, and the tree then decides what a
  failed leg means, exactly as for a blocked planned leg.
- *What it costs.* A hazard the boat could have steered round now stops it until the line
  clears or the leg fails. The leg makes no planner call and no costmap clear (so the clear
  step at `clear_after_s` does not exist in Off), and builds and runs without `nav2_msgs`.
  The leg status says STRAIGHT or BLOCKED honestly, and a hold is labelled as one in the log.
- *Unchanged.* Shadow still sends the legacy goal and only logs `would HOLD`; On is as above;
  a stale pose is still Running, as legacy. Tests: `test_planned_leg.cpp` sections 18 and 19.

**Why no costmap exemption is needed.** Every leg that comes near its own pair or
the dock is `avoid=false`. Those legs never consult the costmap, so a 2 m gate's
LiDAR-marked pair cannot block its own crossing. Every planned leg treats **every**
known object as a hazard. Its exemption check is BT-side, against the known list only.

**Limitation.** An object seen by the LiDAR only, sitting **inside** a gate or slip, is
not guarded on those straight legs. Handbook 3.3.1 requires the water inside a gate to
be clear, and the run-in has already planned around everything it could see.

### 5.6 XML changes per tree

Every tree is checked by `check_config` for XML legality. A `--` inside a comment is
illegal.

**`task1_disruptive.xml`:**
1. Line 81 becomes `<RetryUntilSuccessful num_attempts="3"><CircleBuoy anchor="entry" radius="6.0" points="8" direction="cw"/></RetryUntilSuccessful>`.
2. Lines 129-133: the `run_in` ReactiveSequence (AvoidObstacles plus NavigateTo steer)
   becomes `<NavigateTo target="port" goal="{goal}" tolerance="3.0"/>`.
3. Line 141 becomes `<NavigateTo target="port" goal="{goal}" tolerance="2.5" avoid="false" exempt="gate"/>`.
4. Line 197 becomes `<RetryUntilSuccessful num_attempts="3"><CircleBuoy anchor="exit" radius="6.0" points="8" direction="ccw"/></RetryUntilSuccessful>`.
5. Rewrite the comments at lines 61-64, 117-128 and 135-139 (§9).
6. A blocked run-in FAILs, which fails the leg Sequence. Then the ForceSuccess at line
   109 means **the leg restarts against the same gate**, with no other change.

**`task1_safe_passage.xml` (Core):**
1. Line 50 becomes `<RetryUntilSuccessful num_attempts="3"><NavigateTo target="approach" tolerance="4.0"/></RetryUntilSuccessful>`,
   kept inside the ReactiveFallback.
2. Lines 81-82 become `<NextWaypoint offset="4.0"/>` followed by
   `<ForceSuccess><NavigateTo target="waypoint" tolerance="2.5"/></ForceSuccess>`.
   Without the ForceSuccess, a blocked leg would end the transit through
   KeepRunningUntilFailure and skip the gates.
3. Lines 63 and 98 become CircleBuoy with `points="8"`, each wrapped in
   `RetryUntilSuccessful num_attempts="3"`.
4. Rewrite the comment at lines 30-31.

**`task3_disruptive.xml`:**
- Lines 122-123 (look) and 150-151 (lead) are unchanged: they plan with
  `avoid="true"` and the dock is a hazard.
- Line 157 becomes `<NavigateTo target="port" goal="{goal}" tolerance="0.4" resend_m="0.2" avoid="false" exempt="dock"/>`.
- Line 176 becomes `<NavigateTo target="port" goal="{goal}" tolerance="0.25" resend_m="0.2" avoid="false" exempt="dock"/>`.
- Add a comment at the line-up: "planned legs end outside the fingers; the line-up and
  berth stay straight because 0.45 m per side is inside the 0.8 m hard clearance".

**New `nav_test_line.xml`,** for the LiDAR-only sim test:
`IsAutonomous` guard band, then `<NavigateTo target="approach" tolerance="2.0"/>`. The
goal's approach lat/lon is the target.

`demo_waypoints.xml`, `demo_perception.xml` and the two `task3_fire_*.xml` trees are
unchanged. The demo legs plan through the defaults.

### 5.7 offros_runner and task3_sim

**offros_runner** (`crusader_bt/offros/offros_runner.cpp`):
- A new stdin type, `datum`. onPose follows §2. The `Params` struct (line 151) gains
  `std::string nav_mode = "off"`, set from a new CLI flag `--nav-mode off|shadow|on`
  in main (line 686).
- With shadow or on: `ctx_->planner = std::make_shared<path::StraightPlannerPort>()`.
  The straight path is rejected locally whenever it crosses the dock: a stub, not a
  planner. Its only purpose is to exercise the PlannedLeg state machine with the real
  leaves.
- New stdout types:
  - `{"type":"leg", ...}`: the §4.3 schema, emitted on change.
  - `{"type":"hazards","n":k}`.
- **Default `off`.** `tools/task3_sim/test_e2e.py` must pass unchanged.
- The header comment's in/out list (lines 29-45) gains the new types.

**`tools/task3_sim/build.py`** adds two `run_test` lines and three headers (§5.1).
`ros_planner_port.cpp` is **never** in its source list (lines 205-210).

### 5.8 crusader_params.yaml: the `bt_runner_node` section (WP2's hunk, after line 1276)

Everything here is [RO]: the runner reads it at startup.

| Key | Type | Default | Note |
|---|---|---|---|
| `nav_mode` | string | `"off"` | `off`, `shadow` or `on`. **Boat `off`** until the container has Nav2; the sim passes `on` |
| `nav_map_frame` | string | `"map"` | — |
| `nav_datum_topic` | string | `"/crsd/datum"` | — |
| `nav_hazards_topic` | string | `"/crsd/nav/hazards"` | must equal nav2_params `hazard_layer.topic` (check_config) |
| `nav_leg_topic` | string | `"/crsd/nav/leg_status"` | = ground_station.nav_leg_topic (check_config TOPIC_PAIRS) |
| `nav_planner_action` | string | `"/compute_path_to_pose"` | — |
| `nav_valid_service` | string | `"/is_path_valid"` | — |
| `nav_clear_service` | string | `"/global_costmap/clear_entirely_global_costmap"` | — |
| `nav_planner_id` | string | `"GridBased"` | — |
| `nav_hazard_rate_hz` | double | 2.0 | — |
| `nav_status_hz` | double | 2.0 | — |
| `nav_hard_m` | double | 0.8 | ≤ robot_radius·cos(π/16) (check_config) |
| `nav_soft_m` | double | 2.0 | = inflation_radius (check_config) |
| `nav_buoy_radius_m` | double | 0.30 | RoboBuoy footprint 0.432 m square, circumscribed (gen_world.py:49) |
| `nav_track_radius_m` | double | 0.30 | — |
| `nav_exempt_radius_m` | double | 1.0 | — |
| `nav_lookahead_m` | double | 5.0 | — |
| `nav_lookahead_min_m` | double | 3.0 | — |
| `nav_max_chord_dev_m` | double | 0.3 | — |
| `nav_wp_radius_m` | double | 2.0 | = WP_RADIUS in working_crusader.params:913 (check_config) |
| `nav_replan_period_s` | double | 0.5 | — |
| `nav_min_request_gap_s` | double | 0.2 | — |
| `nav_check_period_s` | double | 0.1 | — |
| `nav_plan_timeout_s` | double | 1.0 | — |
| `nav_first_plan_wait_s` | double | 1.0 | — |
| `nav_invalid_confirm` | int | 2 | — |
| `nav_hysteresis_frac` | double | 0.2 | — |
| `nav_hysteresis_m` | double | 3.0 | — |
| `nav_goal_replan_m` | double | 1.0 | — |
| `nav_clip_radius_m` | double | 35.0 | ≤ width/2 - 5 (check_config) |
| `nav_clear_after_s` | double | 5.0 | — |
| `nav_unblock_reset_s` | double | 3.0 | — |
| `nav_escape_margin_m` | double | 0.5 | — |
| `nav_goal_margin_m` | double | 0.3 | — |
| `nav_local_check_tol_m` | double | 0.1 | — |
| `nav_orbit_max_push_m` | double | 3.0 | — |
| `nav_orbit_clear_m` | double | 1.4 | — |
| `nav_dock_finger_len_m` | double | 2.0 | build guide (task3_disruptive.xml:22-25) |
| `nav_dock_finger_w_m` | double | 0.5 | — |
| `nav_dock_slip_w_m` | double | 1.5 | — |
| `nav_dock_deck_depth_m` | double | 1.0 | — |

### 5.9 Unit tests, all off-ROS

Build each with `g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include test/X.cpp`.
Run them on the laptop through tools/task3_sim/build.py, and in colcon through
CMake `add_test`.

**`test_path_math.cpp`:**
1. Circle and polygon `clearance` signs, and keepout.
2. `buildHazards`: counts and positions for plan buoys, tracks and a dock track. Dock
   fingers at ±1.0 m, 2 m long.
3. `exempt`: gate ids, a track 0.6 m from an exempt buoy dropped, a track 1.5 m away
   kept, and `dock`.
4. `segmentClear` and `firstConflict`.
5. `pushGoalOut`: a goal at a buoy centre ends up at least `hard + margin` from the
   surface, on the line toward `from`. A free goal is unchanged. A fully blocked line
   gives `ok = false`.
6. `clipToWindow`.
7. `escapeStart`: a boat 0.5 m from a buoy surface gets a candidate at least 2.5 m away
   that is clear and non-approaching. A boat in the middle of a 2 m gate escapes along
   the gate axis. A boat boxed in by four buoys gets `ok = false`.
8. `carrot`:
   - a straight path gives a carrot 5 m ahead;
   - a path arcing around a buoy gives a shorter carrot with its chord clear at
     `hard - tol`;
   - less than 3 m left gives `is_end`.
9. `preferNew`: 10 % shorter is kept; 30 % and 5 m shorter is adopted; an invalid
   current path is always replaced.
10. `orbitRing` with n = 8: 9 points, `ring[0] == ring[8]`, and the sweep from ring[0]
    over ring[1..8] is -360 for cw and +360 for ccw. Also the **far-start case**: from
    (60, 0) with the anchor at the origin, the sweep is still ±360.
11. `adjustRing`: a buoy on the ring pushes that point out to a clearance of at least
    1.4 m. An unfixable point is dropped. The endpoints are kept.
12. The projection parity literal from §3.2, through `nav::toLocal`.

**`test_planned_leg.cpp`** uses a `FakePlannerPort` with scripted replies and a manual
clock:
1. Off mode reproduces the legacy send sequence: start sends; a 1.0 m goal move does
   not resend; a 2.0 m move does; arrival gives Success.
2. On: a plan arrives, FOLLOWING, the carrot is 5 m ahead. The carrot is resent per
   the rules as the boat advances. A goal pushed out of a buoy succeeds on the
   **true-goal** tolerance.
3. No plan ever arrives: BLOCKED at 1.0 s with exactly one hold, `clearCostmap` called
   once at 5 s, Failure at 15 s.
4. Blocked, then a valid plan at 8 s: FOLLOWING, `blocked_s` held until 3 s of
   continuous following, then reset. A flip-flop every 2 s still fails at 15 s.
5. IsPathValid invalid once does nothing; invalid twice gives BLOCKED and an immediate
   request.
6. A new known hazard on the path gives BLOCKED in the same tick. A plan that crosses a
   known hazard is rejected.
7. NaN heading: DEGRADED, one hold, zero requests, `blocked_s` stays 0 for 60 s.
   Recovery leads to PLANNING.
8. A stale pose holds at the last fresh pose, with no Failure.
9. `ready() == false`: BLOCKED with the reason `planner_server not available`, Failure
   at 15 s.
10. A timeout at 1.0 s; the late reply is ignored by seq.
11. Shadow: the only sends are the legacy goal setpoints; the state shows BLOCKED;
    there is never a Failure.
12. `avoid=false`, `exempt` gate: the own pair 0.9 m off the segment is ignored. A
    third buoy on the line gives a hold, then Failure at 15 s. Removing it resumes with
    the goal resent.
13. A 100 m goal: the requested goal sits 35 m out and advances as the boat advances.
14. A start inside the hard zone sends the escape point first, and the plan start is
    the escape point.

## 6. lidar_cluster_node: the new navigation cloud (crusader_perception)

**`lidar_cluster_core.py`** gets a new
`process_body_nav(pts_body, p, roll, pitch, levelled, st, nav_leaf) -> (clusters, st, nav_pts)`.
It is today's `process_body` (lines 375-411), plus:

- `nav_pts` is the **levelled** points (`lvl`) whose DBSCAN label belongs to a cluster
  with at least `min_points` members, **regardless of `max_extent_m`**. Docks,
  platforms and shorelines are hazards even though they are not trackable objects.
- The points are voxel-downsampled with an edge of `nav_leaf`, one point per occupied
  voxel (the centroid), as an (M, 3) float32 array.
- `process_body` becomes a wrapper that returns `(clusters, st)`, so `lidar_view.py`
  and the tests are untouched.
- `st.n_nav` is added to `ClusterStats.as_dict()`.

**`lidar_cluster_node.py`** gets new PARAM_SPEC entries and matching YAML in its
section (lines 302-358, WP4's hunk):

| Key | Type | Default | RO/DYN | Meaning |
|---|---|---|---|---|
| `nav_cloud_enable` | bool | true | DYN | **The pool switch.** False publishes EMPTY clouds, which keep the costmap current and mark nothing |
| `nav_cloud_topic` | string | `"/crsd/nav/obstacle_cloud"` | RO | — |
| `nav_cloud_frame` | string | `"base_footprint"` | RO | = nav_frames_node.base_frame (check_config) |
| `nav_cloud_leaf_m` | double | 0.10 | DYN, range 0.05-0.5 | — |

**Behaviour, in `_on_cloud` (line 196):**
1. Take `t_rx = self.get_clock().now()` **first**.
2. After processing, if `att is None`, **do not publish** the nav cloud, and log WARN
   at most every 10 s: `attitude stale: nav cloud withheld; the costmap goes non-current and planned legs hold`.
   An unlevelled water gate is a guess.
3. Otherwise publish a PointCloud2 with fields x, y, z (float32), `is_dense`,
   `header.frame_id = nav_cloud_frame` and `stamp = t_rx`. It holds `nav_pts`, or an
   empty cloud when `nav_cloud_enable` is false.

The stamp is **receipt time, not the driver stamp**. The Livox driver's clock is not
proven to be the host clock, and TF is stamped on the host clock (telemetry_bridge's
receipt stamps). Use sensor-data QoS. The cluster topic and its behaviour are
unchanged.

**The LiDAR source, raw versus filtered.** Feeding STVL the raw `/livox/lidar`
(20k points at 10 Hz in `livox_frame`) would mean:
- publishing roll and pitch into TF;
- a second copy of the [RO] extrinsic, upside down, as a static TF;
- height-gating a mount that sits 0.28 m above the water and mostly sees water.

`lidar_cluster_node` already levels with attitude, gates the water at 0.39 m above the
hull datum, rejects DBSCAN noise and motion-compensates 5 sweeps, which gives roughly
20 m of range on a buoy. **Chosen: the filtered cloud.** It keeps one owner of the
extrinsic and costs STVL little CPU.

**The pool profile.** Run `ros2 param set /lidar_cluster_node nav_cloud_enable false`
in the asv container, or use the GCS Tuning tab, since the parameter is DYN. Use it
together with `r_max 10` (YAML comment at line 352). The study warns that pool walls
make goals unreachable. The YAML comment must say so.

**Test: `crusader_perception/test/test_lidar_nav_cloud.py`** (unittest):
1. A synthetic 0.3 m buoy at 8 m gives nav points at body x ≈ 8 and z above the water
   gate.
2. A 20 m wall gives nav points but no Cluster3D (`max_extent_m`).
3. Sparse noise gives none.
4. A 10° pitched cloud: a known point lands at its levelled z.
5. The downsample bounds the count.

## 7. Ground station path display and the sim referee

**GCS (`crusader_groundstation`, WP5).**
- `gcs_node.py`:
  - PARAM_SPEC (around lines 91-115) and YAML gain `nav_leg_topic` (RO,
    `"/crsd/nav/leg_status"`) and `nav_leg_timeout_s` (DYN, 2.0, range 0.5-30).
  - Subscribe to the String into a `StreamCache`.
  - `_snapshot` adds:
    `"nav": {"ok", "age", "state", "why", "blocked_s", "mode", "path": [[x,y],..], "target": [x,y]|null, "goal": [x,y]|null}`.
    Points are converted with `geo.latlon_to_xy(.., self._origin)`, as `_target_items`
    does (line 668), so the path sits in the same frame as the trail.
  - A stale cache gives `ok: false` and empty arrays: a blank, not the last path.
- `gcs_page.py` `draw()` (line 1881), after the trail and before the clusters:
  - Draw the path as a dashed 2 px polyline, coloured by state: FOLLOWING
    `--lbl-green`, BLOCKED `--lbl-red`, PLANNING/DEGRADED `--lbl-yellow`, STRAIGHT
    `--muted`.
  - Draw the target as a 5 px ring and the goal as a cross.
  - Draw a top-left text badge: `NAV <STATE> <blocked_s>s <why>`.
  - No new tab, no new control.

**Referee (`crusader_sim/crusader_sim/task1_judge.py`, WP5).**
- Footprints: every robobuoy uses the existing `BUOY_HALF_M` box. Add `platform`
  (2 x 2 m), `dock` (deck and fingers, mirroring gen_world.py:183-246) and
  `launch_pad` (2 x 2 m) from `course["elements"]`. They live in a local helper in
  the judge; do not touch course.py.
- `_update` tracks, per object, the minimum of:
  - **centre-to-surface**, the boat centre to the object's box. This is the planner's
    contract.
  - **hull gap**, the hull rectangle to the box, using the existing `_touching`
    geometry. It is computed only when yaw is known.
  - the time each minimum happened.
- **Gate-corridor exclusion.** For each judged gate, samples inside the corridor are
  excluded **for that pair's two buoys only**. The corridor is the points whose
  projection on the red→green axis lies within the segment and whose distance from
  the line is at most 3 m.
- `verdict()` adds:
  `"min_clearance": {"centre_m": x, "worst": name, "non_gate_centre_m": x, "hull_m": x|null, "by_object": {name: {"centre_m", "hull_m", "t"}}, "clearance_ok": bool}`.
  `clearance_ok` is `non_gate_centre_m >= 0.70`, the 0.8 m contract less 0.1 for
  quantisation. It is **reported, not part of `pass`**: the referee grades the task,
  and clearance is our engineering metric.
- `format_verdict` adds the line
  `[judge]   min clearance <x> m (<worst>), non-gate <y> m, hull <z> m`.
- Add a small self-test under `if __name__` or in the module docstring: a straight
  track passing 0.5 m from a black buoy reports `centre_m` = 0.5 - 0.22 ± 0.01.

## 8. Docker

### 8.1 Apt packages

All exist for Humble on jammy, amd64 and arm64 [apt]:

```
ros-humble-nav2-planner ros-humble-nav2-smac-planner ros-humble-nav2-navfn-planner
ros-humble-nav2-costmap-2d ros-humble-nav2-lifecycle-manager ros-humble-nav2-msgs
ros-humble-nav2-util ros-humble-nav2-core ros-humble-spatio-temporal-voxel-layer
ros-humble-tf2-ros-py
```

`nav2-*` is 1.1.20 in both builds: `1jammy.20260908` for amd64 and `1jammy.2026091x`
for arm64. STVL is 2.3.4. `tf2-ros-py` is normally present in ros-base already; listing
it makes the image say so. `navfn` is installed only as the A/B fallback.

### 8.2 `crusader_sim/docker/crsd-sim.Dockerfile` (WP5)

Add one `RUN apt-get update && apt-get install -y --no-install-recommends <8.1 list> && rm -rf /var/lib/apt/lists/*`
after the behaviortree line (line 33). Add a build-time assert:

```
RUN bash -c 'source /opt/ros/humble/setup.bash && ros2 pkg prefix nav2_planner && ros2 pkg prefix spatio_temporal_voxel_layer'
```

Then rebuild the image and **recreate the container**, because `gz_sim_up.sh` reuses an
existing one (lines 145-150). This runs in WSL:

```bash
cd ~/robotx_ws/src/rx26_asv && docker build -t crsd-sim:humble -f crusader_sim/docker/crsd-sim.Dockerfile . && docker rm -f crsd-sim
```

The next `gz_sim_up.sh` recreates the container, and its first run does a full colcon
build (lines 151-158).

### 8.3 `setup/asv_add_nav2.Dockerfile` (WP5, new)

It follows `asv_add_bt.Dockerfile`: it adds Nav2 to **exactly the image the boat runs**,
and nothing else.

```dockerfile
# asv_add_nav2.Dockerfile - Nav2 planner + costmap (+ STVL) on top of the image the boat RUNS.
# WHY A LAYER: see asv_add_bt.Dockerfile. Spec: docs/nav2_avoidance_spec.md section 8.
# Jetson host:
#   cd ~/robotx_ws/src/rx26_asv/setup
#   docker build -f asv_add_nav2.Dockerfile --build-arg BASE="$(docker inspect asv --format '{{.Config.Image}}')" -t asv:nav2-YYYYMMDD .
ARG BASE=asv:bt-20260928
FROM ${BASE}
ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
# Record what the base had, so the package delta is reviewable after the fact.
RUN dpkg -l 'ros-humble-*' > /opt/crsd_pre_nav2_dpkg.txt
# BehaviorTree.CPP too, idempotently: crusader_bt is what calls the planner, and the
# boat image may still be asv:socket-20260903 (study section 4, step 5).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-behaviortree-cpp \
        ros-humble-nav2-planner ros-humble-nav2-smac-planner ros-humble-nav2-navfn-planner \
        ros-humble-nav2-costmap-2d ros-humble-nav2-lifecycle-manager ros-humble-nav2-msgs \
        ros-humble-nav2-util ros-humble-nav2-core ros-humble-spatio-temporal-voxel-layer \
        ros-humble-tf2-ros-py \
    && rm -rf /var/lib/apt/lists/*
RUN L=/opt/ros/humble/lib; A="$L/$(uname -m)-linux-gnu/libbehaviortree_cpp.so"; \
    if [ ! -e "$L/libbehaviortree_cpp.so" ] && [ -e "$A" ]; then ln -s "$A" "$L/libbehaviortree_cpp.so"; fi
RUN dpkg -l 'ros-humble-*' > /opt/crsd_post_nav2_dpkg.txt
# Fail the BUILD, not the boat.
RUN source /opt/ros/humble/setup.bash \
    && ros2 pkg prefix nav2_planner && ros2 pkg prefix nav2_smac_planner \
    && ros2 pkg prefix nav2_costmap_2d && ros2 pkg prefix spatio_temporal_voxel_layer \
    && ros2 pkg prefix behaviortree_cpp && python3 -c "import tf2_ros" \
    && test -f /opt/ros/humble/share/behaviortree_cpp/cmake/behaviortree_cppConfig.cmake \
    && echo "nav2 layer ok"
```

The comments in the real file must not contain `--`; check_config enforces that. If
`apt-get update` fails on an expired ROS key, the base's apt source is stale; install
`ros2-apt-source` as in the ROS docs. That is a risk, not a step.

### 8.4 Recreating the `asv` container: prepared, not executed

**This needs explicit team approval.** Run it on the **Jetson host** over
`ssh crusader@192.168.100.109`, in bash, with the boat on the stand and **disarmed**.
`sudo` needs Chris's password. WP5 writes it into `setup/README.md` as the missing
section "Recreating the asv container", which `asv_add_bt.Dockerfile` already
references and which does not exist.

```bash
# 0. Record the truth: the flags the live container was created with are NOT written down anywhere else.
docker inspect asv --format '{{.Config.Image}}'
docker inspect asv > ~/asv_pre_nav2.inspect.json
docker inspect asv --format 'binds={{json .HostConfig.Binds}} priv={{.HostConfig.Privileged}} net={{.HostConfig.NetworkMode}} runtime={{.HostConfig.Runtime}} restart={{json .HostConfig.RestartPolicy}} tty={{.Config.Tty}} stdin={{.Config.OpenStdin}} cmd={{json .Config.Cmd}}'
```
```bash
# 1. Build the layer to a NEW tag. The running container is untouched.
cd ~/robotx_ws/src/rx26_asv/setup && docker build -f asv_add_nav2.Dockerfile --build-arg BASE="$(docker inspect asv --format '{{.Config.Image}}')" -t asv:nav2-YYYYMMDD .
```
```bash
# 2. Smoke-test the image in a throwaway container.
docker run --rm --network host asv:nav2-YYYYMMDD bash -lc 'source /opt/ros/humble/setup.bash && ros2 pkg prefix nav2_planner && ros2 pkg prefix spatio_temporal_voxel_layer'
```
```bash
# 3. Stop the stack, then RENAME, not rm. Anything living only inside the old container is kept.
sudo systemctl stop crsd-ros crsd-container && docker rename asv asv_pre_nav2
```
```bash
# 4. Create the new asv with the flags step 0 printed. This template matches OPERATIONS.md section 15.
#    Adding the power-socket mount fixes the known gap in Boat/CLAUDE.md "Open now"; that is a team decision.
docker create -it --name asv --network host --privileged --runtime nvidia -v /dev:/dev -v /home/crusader/robotx_ws:/root/robotx_ws -v /run/crsd-power.sock:/run/crsd-power.sock asv:nav2-YYYYMMDD
```
```bash
# 5. Start, rebuild the workspace IN the new container, start the stack.
sudo systemctl start crsd-container && bash ~/robotx_ws/src/rx26_asv/tools/scripts/rebuild.sh && sudo systemctl start crsd-ros
```
```bash
# 6. Verify: core topics as before, then the nav stack by hand (section 3.6).
docker exec asv bash -lc 'source /opt/ros/humble/setup.bash && ros2 pkg prefix crusader_nav_layers && ls /root/robotx_ws/install/crusader_nav_layers/lib'
```

**Rollback,** also on the Jetson host:

```bash
sudo systemctl stop crsd-ros crsd-container && docker rename asv asv_nav2_failed && docker rename asv_pre_nav2 asv
```
```bash
# install/ and build/ are bind-mounted and SHARED. CMake caches from the Nav2 image must go before rebuilding in the old one.
rm -rf ~/robotx_ws/build/crusader_bt ~/robotx_ws/build/crusader_nav_layers ~/robotx_ws/install/crusader_nav_layers
```
```bash
sudo systemctl start crsd-container && bash ~/robotx_ws/src/rx26_asv/tools/scripts/rebuild.sh && sudo systemctl start crsd-ros
```

Delete `asv_pre_nav2` only after a successful water day. `crsd-container.service`
starts the container by name (`docker start -a asv`), so no unit file changes.

## 9. Docs to fix

| Where | Today | Fix | WP |
|---|---|---|---|
| crusader_bt/README.md:207-208 | "avoidance runs in the autopilot (PRX1_TYPE=2)" | Avoidance lives in the tree: NavigateTo and CircleBuoy plan through Nav2's planner_server (`nav_mode`). ArduPilot's simple avoidance is an optional backstop, off by baseline (PRX1_TYPE 0). Add a "Planned legs" section covering ports, states, `nav_mode` and the shadow posture | 2 |
| nav_math.hpp:446-447 | "The tree does no avoidance anyway" | "Obstacles become hazards for the planned legs (path::buildHazards) and the costmap" | 1 |
| task1_disruptive.xml:61-64, 117-128, 135-139 | AvoidObstacles is local and reactive | Planned legs through Nav2. The crossing is straight with `exempt=gate`, and why | 2 |
| task1_safe_passage.xml:30-31 | "avoidance runs in the autopilot" | as above | 2 |
| docs/G5_obstacle_avoidance.md | claims PRX1_TYPE=2 live, contradicting the baseline (study §1) | Banner: avoidance is the tree's (this spec). The live PRX1_TYPE is unverified; check it with param_guard. If it is 2 with AVOID_MARGIN 2.0, ArduPilot **stops** the boat inside the planner's 0.8-2 m band and in the berth. OA_TYPE stays 0 | 5 |
| crusader_bringup/launch/core.launch.py:36-50 | comment: the LiDAR gates arming (PRX1_TYPE=2) | "only if the autopilot has PRX1_TYPE=2; baseline 0 (working_crusader.params:544)". This is a **comment-only** launch-file change, but it still needs the team's OK | 5 |
| crusader_params.yaml:413-415 | proximity_bridge "feeds the autopilot's avoidance" as if it were the mechanism | Note that the tree's planner is the mechanism and proximity_bridge is optional | 3 |
| crusader_sim/README.md:124-128 | "not fixed here" | point to this spec and the new courses; add a "Nav2 avoidance in the sim" section (`NAV_MODE`, image rebuild) | 5 |
| docs/avoidance_design_study.md:3 | "proposal, awaiting sign-off" | "Decided 2026-09-30: Option A, see nav2_avoidance_spec.md" | 5 |
| setup/README.md | the "Recreating the asv container" section that asv_add_bt references is missing | add it from §8.4 | 5 |
| docs/OPERATIONS.md | — | grep it first, since it is 44 KB. Add a short section "Obstacle avoidance (Nav2)": start order, `nav_mode`, the pool switch, `costmap_probe`, and BLOCKED/DEGRADED meanings | 5 |
| crusader_perception/README.md, crusader_nav/README.md, crusader_nav_layers/README.md | — | the nav cloud; the new packages | 4, 3, 3 |
| Boat/.claude/skills/crusader-triage/SKILL.md "Avoidance appears to do nothing" | — | one paragraph pointing to `/crsd/nav/leg_status` and `costmap_probe` | 5 |

**REP-120 note** for crusader_nav/README.md: `base_footprint` here is levelled
base_link at the hull datum, not on the water. It is named for Nav2 familiarity.

## 10. Work breakdown

**File ownership is disjoint.** `crusader_params.yaml` is shared **by section only**,
and each work package (WP) edits only its own section's hunk:

| Section of crusader_params.yaml | WP |
|---|---|
| `bt_runner_node` | WP2 |
| `lidar_cluster_node` | WP4 |
| `ground_station` | WP5 |
| new `nav_frames_node` section, and the proximity_bridge comment | WP3 |

A YAML key **must land with its code**: `declare_from_config` raises on an unknown
key (param_utils.py:129-131).

| WP | Owner files | Depends on | Can start |
|---|---|---|---|
| **WP1 Planning core** (pure C++) | path_math.hpp, planner_port.hpp, planned_leg.hpp, test_path_math.cpp, test_planned_leg.cpp, nav_math.hpp (deletions and comment), test_nav_math.cpp (deletions) | none | day 0 |
| **WP2 BT integration** | context.hpp, bt_runner_node.cpp, leaves.cpp, ros_planner_port.{hpp,cpp}, offros_runner.cpp, behavior_trees/*.xml (plus nav_test_line.xml), crusader_bt/CMakeLists.txt, package.xml, README.md, tools/task3_sim/build.py, YAML `bt_runner_node` | the WP1 API (§5.2-5.3 is exact); the WP3 msgs | day 0, coding against the headers. WP1 lands first for the tests |
| **WP3 Nav stack and config** | crusader_msgs/msg/Hazard*.msg and CMakeLists.txt; the crusader_nav and crusader_nav_layers packages; crusader_bringup/package.xml; tools/scripts/check_config.py; YAML `nav_frames_node` section and the proximity_bridge comment | none. **Land the msgs first**: they take about 10 minutes and unblock WP2 | day 0 |
| **WP4 Perception** | lidar_cluster_core.py, lidar_cluster_node.py, test_lidar_nav_cloud.py, crusader_perception/README.md, YAML `lidar_cluster_node` | none | day 0 |
| **WP5 Sim, Docker, GCS, docs** | crsd-sim.Dockerfile, setup/asv_add_nav2.Dockerfile, setup/README.md, crusader_sim/scripts/{gz_rig_up.sh, gz_sim_up.sh, gz_rig_down.sh, gz_rig_processes.txt}, the new crusader_sim/courses/*.yaml, task1_judge.py, crusader_sim/README.md, gcs_node.py, gcs_page.py, YAML `ground_station`, the docs marked WP5 in §9, core.launch.py (comment only) | the topic names from this spec | day 0. Integration (§10.2) needs all WPs |

Every WP that touches vehicle code (WP1-WP4, plus WP5's GCS and judge) does the
**helper-function pass**: re-read its own diff for repeated blocks and extract them,
as Boat/CLAUDE.md requires.

**Per-WP acceptance:**
- **WP1:** both tests and `test_nav_math` pass under g++ on the laptop (through
  `python tools/task3_sim/build.py`) and under `-Wall -Wextra -Wpedantic` with no
  warnings. Every behaviour in §5.3 has a numbered test.
- **WP2:**
  - In `crsd-sim`, `colcon build --packages-select crusader_msgs crusader_bt` succeeds
    **with** Nav2.
  - It also succeeds **without** Nav2, either in an old crsd-sim image or by
    configuring with `-DCMAKE_DISABLE_FIND_PACKAGE_nav2_msgs=TRUE`.
  - `tools/task3_sim/test_e2e.py` passes with the default nav mode `off`, and a
    manual task3_sim run with `--nav-mode on` completes.
  - `check_config.py` passes.
  - bt_runner with `nav_mode:=on` and no planner holds and FAILs a leg at 15 s with
    `planner_server not available`.
- **WP3:**
  - `test_frames_core` and `test_raster` pass.
  - In `crsd-sim`, with a 10 Hz fake `/crsd/pose`
    (`ros2 topic pub -r 10 /crsd/pose crusader_msgs/msg/LatLonHead ...`), `nav.launch.py`
    brings `planner_server` to **active**.
  - `ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose ...`
    with `use_start: true` returns a path.
  - With a HazardArray holding a 0.3 m circle on the line, the path keeps at least
    0.8 m from the circle's surface, checked by a script over `/plan`.
  - Re-publishing the array without that circle restores the straight path within 1 s.
  - `/is_path_valid` returns false for a path through the circle.
  - `clear_entirely` followed by the next update redraws the circle.
  - `check_config` passes, including the new cross-checks: `nav_hard_m` ≤
    robot_radius·cos(π/16); `nav_soft_m` == inflation_radius; `nav_clip_radius_m` ≤
    width/2 - 5; the hazard topic pair; `nav_wp_radius_m` == WP_RADIUS in the
    baseline; `nav_cloud_frame` == base_frame; nav_frames_node in
    CONFIG_DRIVEN_NODES.
- **WP4:**
  - `test_lidar_nav_cloud` and the existing tests pass.
  - In the sim, `ros2 topic hz /crsd/nav/obstacle_cloud` reads about 10 Hz in frame
    `base_footprint`.
  - A buoy 8 m ahead appears at body x ≈ 8 ± 0.3.
  - With attitude stopped, the cloud stops and the WARN appears.
- **WP5:**
  - Both Dockerfiles build.
  - `docker run` on the new crsd-sim image shows the nav packages.
  - The judge self-test passes.
  - The GCS draws a path from a hand-published `/crsd/nav/leg_status` JSON line.
  - `gz_rig_up.sh` starts the nav stack when Nav2 is present, and prints the banner
    and runs `nav_mode:=off` when it is absent.

### 10.1 Sim rig changes (WP5)

**`gz_rig_up.sh`**, after target_tracker (line 65):

```bash
if ros2 pkg prefix nav2_planner >/dev/null 2>&1 && ros2 pkg prefix crusader_nav_layers >/dev/null 2>&1; then
  read -r DLAT DLON < <(python3 -c "from crusader_sim import course as C; o=C.load('$COURSE')['origin']; print(o['lat'], o['lon'])")
  up nav /tmp/nav.log ros2 launch crusader_nav nav.launch.py datum_source:=param datum_lat:=$DLAT datum_lon:=$DLON
  NAV_MODE="${NAV_MODE:-on}"
else
  echo "  *** AVOIDANCE OFF: this image has no Nav2. Rebuild crsd-sim (docs/nav2_avoidance_spec.md 8.2)"
  NAV_MODE=off
fi
```

The bt_runner line (71-73) gains `-p nav_mode:=$NAV_MODE`.

**`gz_sim_up.sh`:**
- Pass `-e NAV_MODE="${NAV_MODE:-}"` alongside TREE (line 165).
- Add `crusader_nav crusader_nav_layers` to the per-run `--packages-select` (line 162).
  `crusader_bt` and `crusader_perception` still need a manual rebuild after a change,
  as today.

**`gz_rig_down.sh` and `gz_rig_processes.txt`:** add `planner_server`,
`nav_lifecycle` (was `lifecycle_manager`), `nav_frames_node` and `ros2 launch crusader_nav`.

**Sim-only acceptance checks for the nav stack:**
- **N1:** `ros2 lifecycle get /planner_server` reports active.
- **N2:** the `tf2_echo map base_footprint` translation matches `toLocal` of
  `/crsd/pose` to within 0.05 m.
- **N3:** STVL decay. A buoy marked in view, then removed from the course while still
  in view, must disappear within 5 s. Out of view, it must persist for at least 20 s
  and be gone by 35 s. **If N3 fails, switch to the ObstacleLayer fallback (§3.5) and
  record it in this spec.**

**New courses**, all copies of `task1_core.yaml` unless noted:
- `task1_blocked_exit.yaml`: EXIT moved to `x: 62.0, y: 0.0`, plus
  `{type: robobuoy, name: black3, x: 53.0, y: -0.6, beacon: "off"}`. That puts black3
  on the line from gate 3's through point (≈48, -1) to the EXIT ring start (≈56, -0.4).
- `task1_entry_black.yaml`: plus `{type: robobuoy, name: black_entry, x: 3.5, y: 0.9, beacon: "off"}`,
  on the line from the start (0, 0) to the ENTRY ring start (≈6.2, 1.5).
- `open_water_platform.yaml`: `open_water.yaml` plus
  `{type: platform, name: plat, x: 20.0, y: 0.0}` on the line to a goal 40 m east.
  **Check its freeboard first.** The sim platform deck tops out at 0.15 m (gen_world.py:251-262,
  where the value is labelled GUESS). That is the same height as the water gate
  (`water_z` 0.24 + `water_margin` 0.15 above the hull datum, which is 0.15 m above
  the water). It may be invisible to the LiDAR by design.

### 10.2 Sim integration test plan

These run in WSL. Start the rig with `NAV_MODE=on bash crusader_sim/scripts/gz_sim_up.sh <course>`,
then run `bash crusader_sim/scripts/gz_task1.sh <course>`. Every run is logged to
`~/.cache/crusader_sim/task1_last.log`; keep a copy per run. **Baseline:** the same
commit with `NAV_MODE=off`.

| # | Course / setup | Pass criteria |
|---|---|---|
| S1 | `task1_core`, Disruptive tree (default) | referee **PASS** (gates 3/3, ENTRY cw and EXIT ccw ≥ 330°, no contact); `non_gate_centre_m` ≥ 0.70; median run time over 3 runs ≤ **1.2 ×** the baseline median; no BLOCKED in the log |
| S2 | `task1_core` with `TREE=task1_safe_passage.xml` and `--no-uav` (Core) | PASS; clearance ≥ 0.70; time ≤ 1.2 × baseline |
| S3 | `task1_blocked_exit` | PASS; `black3` centre ≥ 0.80 - 0.07; the GCS shows a detour; exit orbit ≥ 330° ccw |
| S4 | `task1_entry_black` | PASS; `black_entry` ≥ 0.73 m; entry orbit complete |
| S5 | Task 1 panel (`TASK1_PANEL.cmd`): move the EXIT behind an unpaired buoy mid-run (the 2026-09-30 finding) | no contact; exit orbit ≥ 330° ccw; the panel's referee is clean |
| S6 | `open_water_platform` with `TREE=nav_test_line.xml`, goal `approach_latitude: 1.28060, approach_longitude: 103.8560594` (40 m east, SITL scale), sent with `ros2 action send_goal /crsd/safe_passage crusader_msgs/action/SafePassage "{tier: 0, timeout_s: 300, approach_latitude: 1.2806, approach_longitude: 103.8560594}"` inside crsd-sim | **Precondition:** `crsd/lidar_cluster_health` shows the platform clustered, otherwise it is a perception finding (freeboard); rerun with `ros2 param set /lidar_cluster_node water_margin 0.08`. Then: arrives; platform centre-to-surface ≥ 0.73 m. Run at `r_max` 10 and at 30 |
| S7 | `task3` with `TREE=task3_disruptive.xml` | the docking and fire reports as on the baseline; look and lead legs keep ≥ 0.73 m from the fingers and deck; predock and berth unchanged (STRAIGHT in leg status) |
| S8 | Blocked: in the panel, ring the boat with 4 black buoys 2 m off | one hold; costmap clear at 5 s; FAILURE at 15 s; Retry / leg restart visible in the tree; mission ends with a clear result, never driving into a buoy |
| S9 | Degraded: mid-transit, set `SIM_GPS_HDG 0` on SITL from QGC or MAVProxy (SITL runs in WSL, not in crsd-sim) | DEGRADED and hold within 1 tick of NaN heading; no FAILURE; resumes after `SIM_GPS_HDG 1` |
| S10 | Shadow: `NAV_MODE=shadow`, `task1_core` | setpoint log identical to baseline (`grep "NavigateTo\|orbit" /tmp/bt.log` diff modulo the timestamps); leg status shows plans |

**Recommended, adopted from Bumblebee:** a hybrid run with the real Ekko over the real
radio against the Gazebo boat, the way §11 describes Bumblebee pairing a real UAV
with a simulated ASV.

### 10.3 Real-boat water-test order

The container is recreated first (§8.4). Each step needs `publish_setpoints:=true`,
which is a G1 gate, with the RC e-stop in hand. **Avoidance is not a safety system;
the RC e-stop is.**

0. **Bench, on the stand, disarmed.**
   - With nav.launch.py running, check `frames_health`: `tf_hz` > 5 and the datum
     printed.
   - Rotate the boat by hand through 4 × 90° near a fixed object (a dock post).
     `costmap_probe` must keep the object's lat/lon within 0.3 m. This catches yaw-sign
     and mirror errors, which are silent; Bumblebee's sensor-rig lesson in §11.
   - Run `tegrastats` with the nav stack running.
1. **Shadow mode** on the water: `nav_mode:=shadow`, a full Task 1 field driven
   legacy. Leg status shows sane paths; no false BLOCKED; the costmap blobs match the
   buoys.
2. `nav_mode:=on`: one black buoy on the line at 1 m/s. Check the min clearance from
   the GPS track against the buoy fix.
3. A 3 m gate: run-in, then the crossing (`exempt=gate`).
4. An orbit with a buoy 1 m outside the ring.
5. Blocked: a two-buoy box. Hold, then FAILURE at 15 s.
6. The full Task 1 field. Then the Task 3 dock approach: look and lead planned,
   line-up and berth straight.

**Before step 2,** check the live `PRX1_TYPE` with `tools/scripts/param_guard.py`.
If it is 2 with `AVOID_MARGIN` 2.0, ArduPilot's simple avoidance will stop the boat
inside the planner's band. That is decision D2 in §12.

## 11. What Bumblebee does, and what we adopt or deliberately don't

Sources:
- Team Bumblebee (NUS) RobotX 2026 paper, https://bumblebee.sg/pdf/Bumblebee_RobotX_Paper_2026.pdf
  (43 pages; the PDF page numbers are cited below).
- Their blog, https://bumblebee.sg/competitions/robotx/2026/blog/ (post titles
  and dates are cited below).

Their navigation is described only briefly. Where this section says "they", read it as
our paraphrase.

| What they do | Where | Our decision |
|---|---|---|
| ASV navigation on **Nav2**. Obstacles live in an **STVL** layer, not an occupancy grid, because wave crests, spray and thruster wash make noisy surface returns that would persist in a static costmap. STVL time-decays voxels, and its 3D voxels separate a buoy's hull from the water | paper p.4 §II-B-1-d "Obstacle Avoidance" | **Adopted: STVL** for our LiDAR layer (§3.5), with **decay semantics that are exactly the study's** "in view and unseen a few seconds; about 30 s out of view". We keep our **upstream** water gate and DBSCAN in lidar_cluster_node (§6), because our MID360 is inverted 0.28 m above the water and mostly sees water |
| Nav2's **controller server** outputs a body-frame velocity; their own holonomic controller and thrust allocator track it | paper p.4 | **Not adopted.** User decision: ArduPilot GUIDED position setpoints, no velocity path (study Option B rejected). They own the dynamics-aware controller; we do not |
| RoboCommand keep-out zones and moving contacts are **added to the costmap**, and the planner routes round them with no mission-logic change (Task 4) | paper p.4 | **Hook adopted:** `Hazard.SRC_KEEPOUT` and `keepout_m` (§4.1). No producer yet; the 5 m distractor-vessel rule (hidden-3.6:13) can use `keepout_m` too, once something classifies vessels |
| Dock entry: the LiDAR resolves the bay side walls, which go into the costmap as **hard obstacles**; holonomic **sway** corrects lateral offset in the final approach | paper p.4-5 §e "Dock Detection and Entry" | **Not adopted for the berth.** 0.45 m per side is inside our 0.8 m hard clearance, and we have no sway on the GUIDED path. Predock and berth stay straight (`exempt=dock`). The dock **is** a hazard on every earlier leg, from the DockBook plus the LiDAR |
| The UAV flies a lawnmower survey, classifies beacon state over a window of frames, sends positions and states **in one message**, then hovers to broadcast changes so the ASV can **re-plan** | paper p.6 §3-b | Matches our Ekko plan, `PlanChanged` and per-gate confirmation design. No change |
| UAV-to-ASV relative pose comes from **AprilTags** on the ASV rather than shared GNSS, so neither vehicle's GNSS error nor link latency enters | paper p.6 §3-c | **Flagged, not adopted (R6):** unassociated UAV plan buoys become hazards at the UAV's GNSS positions. An inter-vehicle GNSS bias shifts them. Proposal for later: estimate the plan→tracker offset from associated pairs and apply it to the unassociated ones |
| Localisation is a UKF fusing **RTK GNSS with DLIO** LiDAR odometry; a dual-UKF local/global split | paper p.10 §2, p.37 table; blog "Why does the boat think it is somewhere else?" (20 Aug 2026) | **Not adopted.** ArduPilot's EKF (RTK plus GPS yaw) stays the single estimator, with no `odom` frame (§2). STVL decay bounds the smear from a GNSS jump |
| They test buoy fields in a **Gazebo** model, then the pool, then open water; tune GNSS-LiDAR fusion on a **sensor rig** before the hull; run a **real UAV against a simulated ASV** over the real link | paper p.27 test-plan table; p.8 §A; p.10 §4 | **Adopted:** the same sim → bench → water ladder (§10.2-10.3); the **bench rotation test** before water (§10.3 step 0); the hybrid Ekko-plus-Gazebo run is recommended (§10.2) |
| ROS 2 **Jazzy**, Py Trees | paper p.37 | We stay on Humble and BT.CPP v4. That is why Humble's planner_server has no error codes and why we time out plans ourselves (§2) |

**Bottom line.** Their experience supports Nav2 for planning and costmaps on a small
RobotX USV, and it supports a **decaying** obstacle representation on water. Nothing
in it argues against the user's decisions. The one design change it prompted is
**STVL instead of a plain ObstacleLayer**.

## 12. Risks and open questions for the team

**Risks:**

| # | Risk | Mitigation |
|---|---|---|
| R1 | Container recreation loses hand-made state | rename, never rm; capture `inspect` first; rollback in §8.4; team approval |
| R2 | The STVL image delta: PCL 1.12 plus OpenVDB, likely several hundred MB, on the Jetson's disk; also unproven STVL behaviour on empty clouds | measure `docker images` before and after; the N3 test; the ObstacleLayer fallback is one YAML edit |
| R3 | The Nav2 debs (built 2026-09) on an older base: apt pulls newer ROS libraries under existing nodes | Humble's ABI-stability promise; `/opt/crsd_{pre,post}_nav2_dpkg.txt` give the reviewable delta; core-topic check in step 6 |
| R4 | Origin mismatch, a silent offset | one datum owner; `datum_mismatch` hold; projection parity literal in both languages; N2 test; bench rotation test |
| R5 | CPU on the Orin Nano: Smac at ≤ 2 Hz under 0.5 s, costmap at 5 Hz over 640k cells, STVL, plus TensorRT detection | `tegrastats` at bench step 0; fall back by lowering `update_frequency` to 2, or the resolution to 0.15 m with `robot_radius` re-derived |
| R6 | UAV GNSS bias on unassociated plan buoys (Bumblebee p.6) | the planner keeps 0.8 m off the UAV position; within camera range the tracker position wins (nav_math fusePassage). Bias estimation is a later item |
| R7 | LiDAR-only objects inside a gate or slip are not guarded on straight legs | by design (§5.5); run-ins planned; the handbook says gates are clear |
| R8 | A start inside the hard zone of a **LiDAR-only** object: the escape knows only known hazards | blocked, clear at 5 s, FAILURE, retry. Watch for it in S8 and on the water |
| R9 | ArduRover GUIDED: WP_RADIUS 2.0 and LOIT_RADIUS 2.0 versus the carrot logic and Task 3's 0.25-1.0 m tolerances (an existing issue, task3_disruptive.xml:66-70) | the carrot resend rule; Task 3's precondition is unchanged and still open |
| R10 | Low-freeboard objects (≤ 0.15 m) are invisible by the water gate | documented; S6 precondition; `water_margin` is DYN |

**Decisions needed:**
- **D1. The boat datum.** Who sets `datum_lat`/`datum_lon` on competition day: the
  launch area, or the first fix with no respawn?
- **D2. ArduPilot.** Set `PRX1_TYPE` 0 (or `AVOID_MARGIN` ≤ 0.8) whenever `nav_mode`
  is on? Keep OA_TYPE 0 regardless; BendyRuler must never run alongside this.
- **D3. Degraded.** Should a NaN heading lasting more than N s fail the leg instead of
  holding until the mission timeout? The spec holds, per the user's rule.
- **D4. The 5 m vessel standoff** (hidden-3.6:13). Needs a vessel classifier feeding
  `keepout_m = 4.2`, which is 5.0 minus the 0.8 hard radius. Who owns it?
- **D5. `r_max` for competition,** 25-40 m (study D6). The STVL clearing frustum is
  capped at 20 m regardless.
- **D6. The power-socket mount** in the recreation (§8.4 step 4): add it now?
- **D7. Orbits.** Is a Retry of 3 then mission FAILURE right? Or should an orbit that
  cannot complete fall through to the next phase, scoring partial?

**Not verified, and to be confirmed in the sim before relying on it:**
- STVL frustum clearing with empty clouds, and `decay_acceleration` with
  `model_type: 1` (N3).
- STVL's exact per-source parameter set in 2.3.4, beyond the humble-branch README
  example.
- The arm64 behaviour of every package; existence is verified, behaviour is not.
- Whether the boat image is `asv:socket-20260903` or `asv:bt-20260928` (step 0 of
  §8.4 prints it).
- The live `PRX1_TYPE`.

## Appendix: the Nav2 Humble facts this spec relies on

| Fact | Source |
|---|---|
| `ComputePathToPose`: goal = `goal`, `start` (PoseStamped), `planner_id`, `use_start`; result = `path`, `planning_time`; no feedback; **no error code** in Humble | [H:nav2_msgs/action/ComputePathToPose.action] |
| `IsPathValid`: request `nav_msgs/Path path`; response `bool is_valid`, `int32[] invalid_pose_indices` | [H:nav2_msgs/srv/IsPathValid.srv] |
| planner_server serves `compute_path_to_pose`, `compute_path_through_poses` and the `is_path_valid` service; isPathValid checks from the pose closest to the robot onward; with a radius footprint, LETHAL or INSCRIBED is invalid; **valid=true if the robot pose is unavailable**; indices never filled | [H:nav2_planner/src/planner_server.cpp] |
| `computePlan`: `waitForCostmap()` spins until current; the start comes from `goal->start` when `use_start`; an exception aborts the goal (`terminate_current`); params `planner_plugins`, `expected_planner_frequency` (1.0) | same file |
| The costmap node is `global_costmap` in namespace `global_costmap`, on its own thread | same file |
| Costmap2DROS params and defaults (`rolling_window` false, `robot_radius` 0.1, `footprint_padding` 0.01, `track_unknown_space` false, `transform_tolerance` 0.3, `update_frequency` 5, `publish_frequency` 1, …); on_configure waits on `canTransform(global, base)`; `getRobotPose` takes the latest TF | [H:nav2_costmap_2d/src/costmap_2d_ros.cpp], [H:nav2_util/src/robot_utils.cpp, include/nav2_util/robot_utils.hpp] |
| A radius footprint is a 16-gon, so inscribed = r·cos(π/16); padding is applied per axis | [H:nav2_costmap_2d/src/footprint.cpp] |
| Inflation cost: 254 on the obstacle, 253 within the inscribed radius, `252·exp(-k(d-r_in))` beyond; params `inflation_radius` 0.55, `cost_scaling_factor` 10, `inflate_unknown`, `inflate_around_unknown`; `isClearable` false | [H:plugins/inflation_layer.cpp, include/.../inflation_layer.hpp] |
| ObstacleLayer per-source params and defaults (`max_obstacle_height` **0.0**, `expected_update_rate` 0, `observation_persistence` 0, `obstacle_max_range` 2.5, `raytrace_max_range` 3.0, `clearing` false); PointCloud2 through a tf2 MessageFilter with sensor-data QoS | [H:plugins/obstacle_layer.cpp] |
| StaticLayer params (`map_subscribe_transient_local` true, `subscribe_to_updates` false, `map_topic`); `reset()` sets `current_` false | [H:plugins/static_layer.cpp] |
| Clear services `clear_except_`, `clear_around_` and `clear_entirely_` + costmap name; clear-entirely resets **all** layers | [H:src/clear_costmap_service.cpp] |
| SmacPlanner2D defaults; no `worldToMap` check; "Starting point in lethal space" throw; tolerance and approach logic; collision at cost ≥ 253; traversal cost formula | [H:nav2_smac_planner/src/{smac_planner_2d,a_star,collision_checker,node_2d}.cpp]; semantics [D] configuration guide, "Smac 2D Planner" |
| NavFn defaults; start and goal off-map return an empty path; `clearRobotCell` | [H:nav2_navfn_planner/src/navfn_planner.cpp] |
| lifecycle_manager params (`node_names`, `autostart` false, `bond_timeout` 4.0, `attempt_respawn_reconnection` true, `bond_respawn_max_duration` 10.0); startup failure aborts bring-up; a broken bond resets, then respawn-reconnects | [H:nav2_lifecycle_manager/src/lifecycle_manager.cpp] |
| Every package in §8.1 at 1.1.20 (Nav2) and 2.3.4 (STVL), for jammy amd64 and arm64 | [apt] |
