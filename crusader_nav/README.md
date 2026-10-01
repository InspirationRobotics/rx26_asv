# crusader_nav

Nav2 obstacle avoidance for Crusader, planner side. Spec: `docs/nav2_avoidance_spec.md`.
Pure Python: the Nav2 packages are needed to *run* `nav.launch.py`, not to build, so a
workspace (or the boat's `asv` image before it is recreated) without them still builds.

| Piece | What it does |
|---|---|
| `nav_frames_node` | Owns the **datum**. Publishes `/crsd/datum` once, latched, and `map -> base_footprint` TF from `/crsd/pose`. |
| `launch/nav.launch.py` | `nav_frames_node` + Nav2 `planner_server` (global costmap, SmacPlanner2D) + `nav_lifecycle`. |
| `nav_lifecycle` | Configures and activates `planner_server`, and again after a respawn. Replaces `nav2_lifecycle_manager`: every call has a timeout (see below). |
| `config/nav2_params.yaml` | The Nav2 parameters. `check_config.py` cross-checks them against `crusader_params.yaml` and the autopilot baseline. |
| `costmap_probe` | Read-only console tool: lethal blobs in the costmap, as lat/lon, range, bearing. |

## The one frame

TF `map`, bt_runner's east/north frame and every `HazardArray` coordinate are **one**
equirectangular ENU plane, centred on one datum (`frames_core.to_local`, R = 6371000 m, the
same formula as `crusader_bt/nav_math.hpp`). `crusader_common.geo` and SITL use different
metres-per-degree, so **neither may be used to make map coordinates**. The parity literal
(datum 1.2806, 103.8557 + 0.0009 on both axes gives x = 100.050439, y = 100.075434) is pinned in
`test/test_frames_core.py` and in WP1's C++ test; change one and change both.

`datum_source`:

* `param` (`datum_lat`, `datum_lon`): restart-stable. Use it in the sim and, unless decided
  otherwise (spec D1), in competition. `(0, 0)` is a startup error: the default is not a datum.
* `first_fix`: the first `/crsd/pose` with a usable position (finite, and not exactly `0, 0`,
  which is what the autopilot reports before it has a position). **Never respawned**: a
  restart would pick a different datum and shift the map under a running bt_runner.

TF is sent at each new pose stamp **with a finite heading** (`yaw = radians(90 - heading)`).
A NaN heading means no TF, which makes STVL drop the cloud and the costmap go non-current, so
planned legs hold. `/crsd/nav/frames_health` (1 Hz JSON) shows `tf_hz`, `last_tf_age_s`
(null until the first transform) and the NaN-heading count.

**REP-120 note.** `base_footprint` here is *levelled base_link at the hull-bottom datum*, the
origin `lidar_x/y/z` are measured from, not "on the ground": the waterline sits at z = +0.24.
It is named `base_footprint` for Nav2 familiarity. Nothing publishes `odom` or `base_link`.

## Running it

By hand, inside `asv` (or `crsd-sim`), **before** `bt_runner_node`. It is not in
`core.launch.py` until it has run on the water.

    source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash
    ros2 launch crusader_nav nav.launch.py datum_source:=param datum_lat:=<lat> datum_lon:=<lon>
    ros2 lifecycle get /planner_server          # active once a pose with a finite heading exists

## Why nav_lifecycle and not Nav2's lifecycle manager (2026-10-01)

In the full sim rig `nav2_lifecycle_manager` (Humble 1.1.20, rmw_fastrtps 6.2.10) asked `planner_server`
to configure; it did, and logged `failed to send response to /planner_server/change_state (timeout):
client will not receive response`. Fast DDS had not matched the server's response writer with the new
client's reader within 100 ms, so the reply was dropped. Humble's manager calls `change_state` with **no
timeout**, so it waited for ever and `planner_server` stayed `inactive [2]`: every planned leg held and
FAILed. It depends on load and discovery timing, and it can happen on the boat at boot.

`nav_lifecycle` (params: the `nav_lifecycle` section of `nav2_params.yaml`) asks `get_state` every
`check_period_s` (1 s), configures or activates as needed, and gives every call `call_timeout_s` (3 s). A
call that misses it is dropped and asked again; `recreate_after_timeouts` (3) in a row rebuild the service
clients (fresh discovery). It never exits, so a respawned `planner_server` is brought up again by itself.
Its log (INFO on each state change, a repeated condition at most every 30 s):

    planner_server: unknown -> unconfigured
    planner_server: asking it to configure
    planner_server: unconfigured -> inactive
    planner_server: asking it to activate
    planner_server active (was inactive)

`planner_server` waits in `activating` (not `configuring`) for TF, so `nav_lifecycle` warns "no answer to
activate" while there is no pose with a finite heading; that is expected. Nav2's nodes create a bond on
activate that, with no manager, never connects. In Humble 1.1.20 that is harmless (nothing follows the
`Creating bond` line, no deactivate; there is no parameter to turn it off), so nothing is set for it.

## What `planner_server` also needs

`planner_server` also needs `/crsd/nav/hazards` (bt_runner, 2 Hz) and `/crsd/nav/obstacle_cloud`
(lidar_cluster_node, 10 Hz) to be *current*: a costmap that has heard nothing from either does
not plan, it hangs. That is deliberate (a dead feed must not plan blind).

## What was measured in the sim (2026-10-01, `crsd-sim:nav2`)

* With a 10 Hz fake pose, planner_server is active in about 2 s; a 40 m plan takes about 25 ms.
* A 0.3 m circle on the line gives a detour at 2.0 m from its surface (the planner prefers to
  leave the 2 m soft band). A 2.0 m gap between two polygons is threaded at 0.95 m clearance.
  A removed hazard restores the straight path in 0.16 s; `clear_entirely` is redrawn in 0.07 s.
* **STVL, as the spec configures it, did not work out of the box:**
  * the `lidar_clear` source with `expected_update_rate: 1.0` is never marked updated, so the
    costmap is non-current forever. It is set to `0.0` in `nav2_params.yaml`; `lidar_mark`
    still catches silence;
  * with that fix, a dense cloud cluster produced **no** lethal cells under `stvl_layer`, while
    the same cloud under the `ObstacleLayer` fallback produced the right blob. The cause was not
    found. Run acceptance test N3 with the real cloud; the fallback block in
    `nav2_params.yaml` is verified.

## Tests

    python crusader_nav/test/test_frames_core.py
    python crusader_nav/test/test_costmap_probe.py
    python crusader_nav/test/test_lifecycle_core.py
