# Lake mode: the REAL Crusader at a lake, with you playing Ekko

Sunday 2026-10-04: 10 buoys on a lake, no UAV in the air. The boat runs its real Task 1 tree; **you are the UAV**:
you tell it where the buoys are and what colour each one is, and you answer its checkpoints, from a browser. This is
the sim's Task 1 panel (`task1_panel`) with the simulator taken out: same radio handshake, same buttons, real boat.

By default the boat runs the **whole-field planner** (`task1_global.xml`, Advanced and Disruptive tier) with `nav_mode off`:
it drives its own plan with GUIDED setpoints and never calls Nav2, so the `asv` container needs no Nav2 for it. START picks
the tier ([Advanced vs Disruptive](#advanced-vs-disruptive)). Saturday's pool test is `POOL=1`
([Pool test](#pool-test-saturday)).

## Safety, before anything else

- **The RC SB switch is the only e-stop.** Neither this page nor any script here is one. The page says so on every screen.
- **Nothing here arms the boat, disarms it, selects a mode, publishes `/crsd/set_mode` or `/crsd/rc_override`, opens a
  MAVLink port (14550/14551/14552) or stops a protected node.** The **pilot** arms with the RC and selects GUIDED. START
  is greyed until the boat itself reports *armed + GUIDED*, and `lake_goal` checks the same thing again before it sends
  anything. `test/test_lake.py` reads the code of `lake_goal.py` / `lake_panel.py` / `goal_client.py` to keep it that way.
- **ABORT GOAL cancels the tree's goal; it does not stop the boat.** The tree stops commanding, but the autopilot stays
  armed and keeps its last command. After ABORT the page shows a banner: **switch SC to HOLD / use the RC.**
- **Dead-man.** The field is resent every 5 s *only while a browser has polled the page in the last 3 s*. Close the tab,
  lose the WiFi, sleep the laptop: the resends stop, and the boat's own plan-freshness guard (`plan_timeout_s` 15)
  aborts the mission about 15 s later. Auto-ACK is held back the same way. The page shows the dead-man state on every
  screen. **Keep the page open and visible:** a browser throttles a tab in a background window.
  **Exception: the Advanced tier on the whole-field tree has no freshness guard** (it plans once and "the UAV is not
  required to remain on station"): close the page there and the boat KEEPS DRIVING its plan. The page says so instead of
  promising an abort; the RC is the stop.
- The panel is unauthenticated HTTP on the boat's WiFi. START still needs the pilot's arm + GUIDED, but anyone on that
  WiFi can ABORT. Use the lake's WiFi, not a shared one.
- PUBLISH defaults to **false** (stand test): the tree plans and ticks but sends no setpoints.

## Where each command runs (three places, constantly confused)

| Context | What it is |
|---|---|
| **laptop** | Windows PowerShell 5.1: SSH out, a browser, copy files |
| **Jetson host** | `crusader@192.168.100.109` over SSH (`Boat/CLAUDE.md`, "Access"; `crusader-net` skill: `plink`, not OpenSSH). systemd, docker |
| **asv** | the ROS 2 Humble container on the Jetson (`docker exec -it asv bash`): *all* of lake mode runs here. `~` is `/root` |

The laptop only runs a browser at `http://<jetson>:8095`. It pulls everything over HTTP; no UDP goes to the laptop.

## Ports

| Port | Who | Note |
|---|---|---|
| tcp 8095 | the panel (`task1_panel --lake`) | `PANEL_PORT` |
| tcp 8090 | ground station | core runs it; the rig starts one only if none is up |
| tcp 8085 | `bt_view` (the tree) | |
| udp 14555 loopback | `rxl_link_node` (the panel talks to it as the UAV) | `-p rxl_endpoint:=udpin:127.0.0.1:14555` on the command line; the YAML's `/dev/crsd-rfd` is never edited |
| udp 14556 loopback | `panel_feed` -> the panel | `LAKE_FEED_PORT` moves it, only to run beside another panel (the sim's Task 1 panel holds 14556) |
| 14550, 14551 | untouched | 14551 is `telemetry_bridge`'s, 14550 stays free for tooling |

## Before the day (at home, once)

1. **Code on the Jetson.** Lake mode lives on the local branch `sim/gazebo`; the boat's checkout is `main`. Copy the
   package, strip CR from the shell scripts, prove them:
   - laptop: `pscp -r <checkout>\crusader_sim crusader@192.168.100.109:robotx_ws/src/rx26_asv/` (see `crusader-deploy`)
   - Jetson host: `sed -i 's/\r$//' ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/*.sh`
   - Jetson host: `bash -n ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh` (and `lake_rig_down.sh`)

   Nothing needs a colcon build for the panel, the feed or `lake_goal`: they run from source (`python3 -m`, `PYTHONPATH`).
   `crusader_sim` is not built on the boat and `rebuild.sh` does not build it.
2. **What the container must have** (the rig checks and says what is missing; run it with `--check` first):
   - core up (`systemctl status crsd-ros` on the host): `telemetry_bridge` is the one hard requirement
   - built in the workspace: `crusader_bt`, `crusader_world_model` (`target_tracker`), `crusader_link`,
     `crusader_groundstation`. **Nav2 is NOT needed for the default (the global tree, `nav_mode off`)**: `crusader_nav`,
     `crusader_nav_layers` and the Nav2 packages (`nav2_planner`, the STVL layer) are needed only for `NAV_MODE=shadow|on`
     (the per-gate tree, or to watch a planner drawn). Without them such a run falls back to `off` and says so; with
     `nav_mode off` the rig says they are not needed and goes on. The `crsd-sim:nav2` image has Nav2; the `asv`
     container may not.
   - `crusader_bt`/`crusader_world_model` new enough for the tree you run (`task1_global.xml` is in
     `install/crusader_bt/share/crusader_bt/behavior_trees/`; the colour-vote tracker; `nav_mode`; `n_gates` in
     `/crsd/safe_passage_report`). The rebuild is on the **host**: `tools/scripts/rebuild.sh`
   - python in `asv`: `yaml`, `pymavlink` (both are what the boat's own nodes use)
3. **`asv`: check readiness without starting anything.** Jetson host: `docker exec -it asv bash`, then in asv:

       LAKE_DATUM=1.3000000,103.8500000 bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh --check

## On the lake

**1. Pick the datum.** One `lat,lon` you can name again (a pier corner, the boat's start). It is the map's origin, the
nav datum and the panel's origin: the field, the saved course and every boat layer are metres east/north of it.
Use the same value every time you restart the rig.

**2. Start the rig.** In `asv`:

    LAKE_DATUM=<lat>,<lon> bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh

First time: the **stand test** (`PUBLISH=false`): the tree plans and ticks, but no setpoint is sent. For the water, restart it
with `PUBLISH=1` (re-running restarts the rig cleanly; it stops only what it started, by recorded pid):

    PUBLISH=1 LAKE_DATUM=<lat>,<lon> bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh

The banner (and `--check`) says plainly **which tree and nav_mode it chose, and why**: with the default global tree and no
`NAV_MODE` given it says `nav_mode off <- task1_global.xml drives GUIDED setpoints itself and never calls Nav2`. The
per-gate tree keeps its old behaviour: `TREE=task1_disruptive.xml` defaults to `NAV_MODE=shadow`, and `NAV_MODE=on` makes it
drive the planner's paths.

| Env | Default | |
|---|---|---|
| `LAKE_DATUM` | **required** | `lat,lon` |
| `TREE` | `task1_global.xml` | the whole-field planner (Advanced + Disruptive). `task1_disruptive.xml` = the per-gate tree (always Disruptive behaviour) |
| `NAV_MODE` | not given: `off` with the global tree, `shadow` with a per-gate tree | `off` no Nav2 / `shadow` plan + draw, drive legacy / `on` drive the plans. With the global tree shadow/on only start the nav stack and draw its planner: it does not drive the boat (the banner says so) |
| `PUBLISH` | false | `1` lets the tree send setpoints (the pilot still has to arm + GUIDED) |
| `LAKE_TUNING` | none | a planner-tuning file, see [Planner tuning](#planner-tuning-overlay-lake_tuning) |
| `POOL` | off | `1` = the pool test, see [Pool test](#pool-test-saturday) |
| `PANEL_PORT`, `LAKE_LOGDIR`, `LAKE_SRC` | 8095, `~/.cache/crusader_lake`, `/root/robotx_ws/src/rx26_asv` | logs: `<LAKE_LOGDIR>/<name>.log` |
| `LAKE_FEED_PORT` | 14556 | the udp port `panel_feed` sends to; change it only to run beside the sim's own panel |
| `LAKE_RXL` | `replace` | see below |

It starts only what `core.launch.py` does not run: `rxl_link_node` (loopback), `target_tracker`, `nav` (not with
`nav_mode off`), `bt_view`, `ground_station` (only if none is up), `panel_feed`, `bt_runner`, `task1_panel --lake`. **`rxl_link_node`**: core runs one
with respawn on the RFD900's serial port. `LAKE_RXL=replace` stops it once and starts the loopback one; core respawns its serial
one 5 s later (harmless noise if the RFD900 is not on USB; if it is, it runs beside ours and hears nothing: unplug
the RFD900 for a cleaner day). Stop everything it started: `bash .../lake_rig_down.sh` (by pid, never by name).

**3. Start `oak_detector` from the GCS Nodes tab** (`http://<jetson>:8090`). It owns the OAK-D (one client only), so the
rig cannot start it. No detector, no camera tracks to click.

**4. Open the page** on the laptop: `http://<jetson>:8095` (the Jetson's WiFi address, `crusader-net`). The header shows the
autopilot (`GUIDED . ARMED`, or why not), the dead-man, the feed and whether the UAV is transmitting.

**5. Build the field** (up to 10 buoys, exactly one ENTRY and one EXIT). Pick the role (keys 1-5: RED, GREEN, ENTRY,
EXIT, BLACK), then any of:
- **click a camera track** on the map (the squares: outline = the boat's own colour vote, grey `?` = unknown), or the
  `-> buoy` button in the *Camera tracks* table. Clicking a buoy's track again recolours it;
- **PIN AT BOAT**: a buoy where the boat is. It is the boat's GPS position, not the buoy's: bring the boat alongside and
  expect ~1 m. A camera track is the better source; the boat fuses your position with its own track within 5 m
  (`assoc_radius_m`), and the UAV error requirement is < 1 m. The pin is the **average of the boat's fresh pose over the
  last ~2 s** (the page shows how many samples, the spread and the heading's age under the button). It refuses, and says
  why, rather than guess: no fresh pose; fewer than 4 samples in the window; a boat that moved more than 1.5 m within it
  ("hold it still alongside the buoy"; a wander over 0.5 m is allowed with a warning). **Bow offset** (metres, default 0,
  up to 10): puts the pin that far **ahead of the boat along its heading**, so you can nose the bow up to a buoy and
  pin the buoy rather than the boat. It needs a fresh heading (the GPS yaw of the RTK pair; the compass is disabled):
  a blank heading, or one whose newest sample is older than 1 s, **refuses the offset** (the pin at offset 0 still works);
- **ADD lat/lon**: typed coordinates;
- **Load template / Load saved**: a course YAML as the starting field, placed from the datum whatever its own origin.
The "map click" choice switches between *camera track -> buoy*, *place a buoy at the click* and *set approach point*.
Drag a buoy to move it, Del removes the selected one. The field in progress survives a panel restart (same datum).

**6. Approach point.** Click it on the map ("set approach point"), or "at the boat". `START` first drives there; with
none the goal skips the drive ("already on station").

**7. COMMIT FIELD.** Puts the field on the air (this is the sim's LAUNCH SIM). Buoy positions lock (ids are list
indices, as in the sim); colours change through SEND. The boat's *fused passage* (diamonds) and its tracks should now
agree with what you sent. UAV position error defaults to **0** here (R in the card sets the sim's < 1 m error).

**8. The pilot arms and selects GUIDED with the RC.** The header pill reads `GUIDED . ARMED`; START enables.

**9. START TASK 1.** Pick the tier next to the button (**Disruptive** by default, or **Advanced**). START rewinds the radio,
sends the field, then runs `lake_goal --tier ...`: your colours, your approach point, 600 s. `lake_goal` refuses unless the
autopilot reports armed + GUIDED, in either tier. The select is locked while a goal runs.

**10. Answer the checkpoints (Disruptive only; in Advanced the boat asks none and the page says so).** The boat asks after
the ENTRY orbit and after each gate, and waits:

| Ask | Label |
|---|---|
| seq 1 | `ENTRY orbit done - confirm gate 1` |
| seq k+1 | `gate k cleared - confirm gate k+1` |
| seq n_gates+1 | `EXIT gate - confirm exit` |

`n_gates` is the boat's own count of **paired** gates (`/crsd/safe_passage_report`), else `min(#red, #green)` of what you sent.
**ACK** confirms the field as last sent. To play the UAV changing its mind: recolour a buoy (the *Field on the air* table's
role dropdown, or click it on the map), then **SEND CHANGES** (a changed field alone releases a gate checkpoint, not the
ENTRY one) or **SEND CHANGES + ACK**. The boat replans. Unanswered, it re-asks every 3 s and gives up after 120 s.
Auto-ACK is **off** by default.

**11. Finish or stop.** `ABORT GOAL` cancels the goal (then the banner: SC to HOLD / use the RC). `STOP UAV` closes the
radio: nothing is transmitted; a running goal aborts on the boat's own guard in ~15 s. The pilot ends with the RC.

**12. Keep the layout.** *SAVE AS COURSE* writes `<LAKE_LOGDIR>/layouts/<name>.yaml` (origin = the datum, metres east/north
of it, `boat_start` = where the boat was, `approach` = your point, tier disruptive) and offers it as a download.

## Advanced vs Disruptive

START chooses the goal's tier; the field you build and COMMIT is the same.

| | Advanced | Disruptive (default) |
|---|---|---|
| the boat | waits for your field, plans the whole passage **once**, drives approach, ENTRY orbit, transit, EXIT orbit | the same plan, but it **stops and asks** after the ENTRY orbit and after each gate |
| checkpoints | **none**: the Checkpoints card says so, nothing waits for you, the table stays empty by design | seq 1 = ENTRY orbit, seq k+1 = gate k, the last = the EXIT confirmation; ACK, or SEND CHANGES + ACK |
| a changed field | the plan is re-made only if what is left of it stops being good (a hazard on it, the boat pushed off) | any change you SEND re-plans the rest from where the boat is |
| the UAV may leave | **yes**: no freshness guard on your field. Closing the page does **not** stop the boat | **no**: `PassagePlanFresh` aborts ~15 s after your last field (the dead-man above) |

Only the whole-field tree has an Advanced behaviour. A rig running `TREE=task1_disruptive.xml` is always the Disruptive
behaviour: the page then disables Advanced (and the panel refuses it) rather than let you expect no checkpoints.
A real ask is never hidden by the tier: the ask card follows what the boat asked.

## Planner tuning overlay (`LAKE_TUNING`)

`LAKE_TUNING=<yaml>` layers a planner-tuning file on the boat's own params for this rig run. The format is the sim's
tuning profile (`config/tuning_profiles/*.yaml`, e.g. `tight_3to5m.yaml`): nested ROS params, **only the keys that change**.
`bt_runner` gets it as a **2nd `--params-file`** (after the boat's own, so it wins); Nav2 gets it as `nav2_overlay:=` and
**only when the nav stack is started** (a profile's `planner_server` / `global_costmap` keys are listed as IGNORED with
`nav_mode off`, never silently dropped). `crusader_params.yaml` and `nav2_params.yaml` are never edited.

The rig checks the file against the boat's own params files **before it starts anything** and stops with every problem at
once: no such file, not YAML, a node it cannot reach, a key the boat's file does not have (ROS would ignore a typo
silently), a value of another type (the node would refuse to start). The banner lists every key applied, `<LAKE_LOGDIR>/
tuning_applied.yaml` is the exact file the nodes got, and once `bt_runner` is up the rig reads the keys back from the running
node (`overlay read back from /bt_runner_node: N/N key(s) match`, or which ones did not take). By hand:
`ros2 param get /bt_runner_node nav_orbit_radius_m`.

Most keys are read once at start: change them by restarting the rig. With the global tree the keys that matter are the
orbit ones (`nav_orbit_radius_m`: the largest orbit, shrunk to fit the water round each blue buoy); the gate/fence keys are the
per-gate tree's.

## Pool test (Saturday)

A small outdoor pool, **camera only**: the LiDAR must feed nothing that plans.

    POOL=1 PUBLISH=1 LAKE_DATUM=<lat>,<lon> bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh

`POOL=1` does four things, and the banner says `POOL: camera-only, LiDAR not used for planning`:
- **`NAV_MODE` is forced off** (a `NAV_MODE=` you give is ignored and the banner says so): no Nav2, no STVL, no planner,
  nothing from the LiDAR reaches a costmap.
- **`target_tracker` starts with `use_lidar:=false` stated on its command line.** It is already `false` in
  `crusader_params.yaml` and in the tracker's code (`test_lake_rig.py` pins both); the explicit flag makes the promise survive
  a changed default. A `target_tracker` already running with `use_lidar` true (someone started it from the GCS Nodes tab) is
  **refused** before anything starts: stop it first.
- **`LAKE_TUNING` defaults to `tuning_profiles/tight_3to5m.yaml`** (orbit 3 m, 3 m gate points ...) unless you give one. It was
  tuned for a 3-5 m field in the sim, not for any particular pool: read the keys the banner lists.
- The tree is the global tree unless you say otherwise.

**The pool is not measured yet, so there is no pool profile.** When the satellite image (or a tape) gives the dimensions, a
profile belongs in `crusader_sim/config/tuning_profiles/pool_<name>.yaml` in `tight_3to5m.yaml`'s format, and the run is
`POOL=1 LAKE_TUNING=<that file> ...`. Numbers in it must come from the pool, not from a guess. The `lidar_cluster_node` and
the LiDAR driver still run (they are core's): only their use in tracks and plans is off. With `nav_mode off` the page's hazards
and costmap layers are blank (no nav datum): the camera tracks, the fused passage, the field and the boat still draw.

## Replaying the lake layout in Gazebo ("sim-real")

1. laptop: put the downloaded file in `Boat\rx26_asv\crusader_sim\courses\<name>.yaml`.
2. WSL: `NAV_MODE=on bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_sim_up.sh <name> --no-uav --no-gui`, then the sim's
   Task 1 panel (`TASK1_PANEL.cmd`), where `<name>` is now in *Load template*. (`crusader-sim` skill.)
The frame is the same: the course's metres east/north of its origin are the sim's world metres.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| the rig stops before starting anything with `LAKE_TUNING ...` | the tuning file is missing, a key is not in the boat's own params file (a typo ROS would ignore silently), or a value has the wrong type (3 for a double). The message lists every problem |
| the rig stops with `NAV_MODE must be off, shadow or on` | `NAV_MODE=` anything else |
| `PIN refused: only N fresh pose sample(s)` | the feed has just started or is dropping out; wait 2 s. `no fresh boat pose`: `panel_feed` / `telemetry_bridge` (the pose layer is blank, never the last value) |
| `PIN refused: ... hold it still` | the boat moved more than 1.5 m in the last 2 s: stop alongside the buoy |
| bow offset refused: `no heading` / `heading is stale` | no GPS yaw (RTK moving baseline not fixed): pin with offset 0 |
| hazards / costmap layers are blank with `nav_mode off` | by design: no nav stack, so no nav datum and no costmap; the camera tracks, the boat, the fused passage and the field still draw |
| Advanced refused: `this rig runs the per-gate tree` | `TREE=task1_disruptive.xml` has no Advanced behaviour: START Disruptive, or restart with the default tree |
| `PANEL UNREACHABLE` | the page cannot poll: WiFi, or the panel died (`<LAKE_LOGDIR>/panel.log`). The boat aborts ~15 s after its last field |
| no camera tracks | `oak_detector` not running (GCS Nodes tab), or `target_tracker` down (`tt.log`) |
| `feed OFFLINE` | `panel_feed` down (`panel_feed.log`), or something else holds udp 14556 |
| "panel_feed's origin is not this panel's datum" | the rig and the panel got different `LAKE_DATUM`s: restart the rig with one |
| START greyed | the reason is under the button: not committed / not armed / not GUIDED / no FCU status from the feed |
| `planner_server is not active` | Nav2 needs a pose with a finite **heading** (GPS yaw from the RTK pair; the compass is disabled): wait for the moving baseline |
| the boat never asks checkpoint 1 | it has not finished the ENTRY orbit; check `bt.log` and the tree on :8085 |
| `ros2 node list` raises `!rclpy.ok()` | the container's ros2 daemon is wedged (seen in crsd-sim 2026-10-02). The rig's own checks are `--no-daemon`; use `ros2 node list --no-daemon`, or `ros2 daemon stop` |
| a field was sent but the boat ignores it | `rxl_link_node` is not the loopback one: `ps -ef | grep rxl_link` in asv; `rxl.log` |
| "udp 14555/14556 is taken" | an old rig process: `lake_rig_down.sh`, then `ps -ef | grep -E 'rxl_link|panel_feed'` |
| page works but a layer is blank | a layer older than 2 s is drawn as nothing and says so under the map (blank, never the last value) |

## What was verified, and where (rehearsal record)

Checked 2026-10-02, never on the real boat (no ssh was used):

| Check | How to re-run | Result |
|---|---|---|
| Offline unit tests: `goal_client` cancel on SIGINT/SIGTERM, `lake_goal` refuses unless armed + GUIDED (fake rclpy), the code of `lake_goal` / `lake_panel` / `goal_client` has no arm / mode / RC-override / MAVLink-port use, dead-man gating of resends and auto-ACK, field <-> course YAML round trip, checkpoint labels, only the lake routes exist, panel_feed's pose / fcu / hazards / datum / `n_gates` layers and the < 60 KB packet, gzip | WSL: `cd crusader_sim; python3 -m unittest discover -s test` (`-p test_lake.py` for these) and `python3 -m crusader_sim.panel_feed --selftest` | 54 lake tests + the existing 50 pass; mutation-checked (removing the dead-man gate, the FCU guard, the cancel call or the START guard each fails a test) |
| `lake_goal` with REAL rclpy against a fake boat (isolated `ROS_DOMAIN_ID=87`, private code copy) | `test/ros_lake_goal.py` (the docstring has the command) | 6/6: SIGINT and SIGTERM (also from a parent that ignores SIGINT) cancel the goal and the server sees it; HOLD, disarmed and no status are refused with no goal sent |
| LakePanel + the REAL `panel_feed` node + the REAL `rxl_link_node` (private ports 34555/34556, domain 87) | `test/ros_lake_integration.py` (the docstring has the command) | 26/26: every new layer from real ROS messages, the plan arrives with the right buoy colours and lat/lon, a resend does not bump the plan version, a recolour does, an ask shows with its label, ACK reaches `/crsd/next_gate`, START follows the FCU stream |
| The lake page in a browser against a dry-run panel and a synthetic feed (HTTP + DOM, no screenshots) | `test/lake_testkit.py` builds the pieces | field built by clicking tracks, PIN AT BOAT, COMMIT, checkpoint banner, recolour + SEND CHANGES + ACK, START refused in HOLD, START + ABORT banner, STOP UAV, SAVE AS COURSE + download |
| The sim page after the shared-JS extraction | same, `task1_panel` dry-run | layers, mouse placement, keys, launch, staging, send, stop: as before |
| `lake_rig_up.sh --check` against the live sim container (read-only), `lake_rig_down.sh` and `up()` on dummy process groups | `bash scripts/lake_rig_up.sh --check` | preflight ok; only recorded pids are stopped; a non-leader pid and a stale pid are left alone |
| **Gazebo rehearsal** of the whole procedure (sim as the boat; `lake_rig_up.sh` unmodified; the pilot's arm + GUIDED played by `pilot_standin.py`, never by the panel) | `test/lake_rehearsal/reh_run.sh <tag> pass|abort|deadman` (WSL; the sim must be FREE, it brings it up and down) | task1_core, PUBLISH=1 NAV_MODE=on, 2026-10-02. **pass**: START refused until armed + GUIDED, then the goal completed (outcome 0, 174 s), all 4 checkpoints answered with their labels incl. `EXIT gate - confirm exit`; the standalone referee says gates 3/3 and both circles correct, no contact, min clearance 1.02 m, but `g2_red` WRONG SIDE: the boat does this on the SIM panel too (`sim_flow_run.sh task1_core`, same buoy), so it is the planner, not lake mode. **abort**: recolour b9 + SEND CHANGES + ACK at checkpoint 1 gave plan v2 (replan); ABORT in transit: outcome 3 CANCELLED, ground speed 0.02 m/s 6 s later, banner set. **deadman**: browser stopped in transit: bt_runner `passage plan is stale (age 15.1s)`, tree FAILURE 15 s after the last field. Found and fixed on the way: a stale `crusader_world_model` install crashed `target_tracker` (no camera tracks), now in `gz_sim_up.sh`'s build list; a zombie `nav_frames_node` read as "already running", now skipped |

Not verified anywhere, because only the boat can: that the `asv` container has Nav2, the newer `crusader_bt` /
`crusader_world_model`, `pymavlink`; that `oak_detector` produces tracks over the lake; that the RTK heading is valid
for `planner_server`; the WiFi's behaviour with the dead-man.
