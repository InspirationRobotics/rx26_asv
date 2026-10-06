# crusader_sim: Crusader in Gazebo, flown by the boat's own autopilot and code

The headless SITL rig (`tools/sitl/`) proves the plumbing. This adds the
water. It runs Gazebo Harmonic physics, the **real ArduRover 4.6.3** running
**the boat's own parameters** (OmniX, servo map, reversals), simulated MID360 and
OAK-D LR, and the RobotX course. Everything downstream of the sensors is the
boat's unmodified code: `telemetry_bridge`, `lidar_cluster_node`,
`rxl_link_node`, `target_tracker`, `ground_station` and `bt_runner`, with the
boat's params file.

**Status (2026-09-29):**
- **Task 1 runs end to end.** `task1_disruptive.xml` finished `task1_core` with
  outcome SUCCESS, 10/10 buoys classified, in 142 s at real-time factor 1.0 with
  the GUI open.
- **The independent referee gave PASS**: ENTRY circled clockwise, all three
  gates with red to starboard, EXIT counter-clockwise, no contact. The tree's own
  `buoys_passed_correctly` read 0 on the same run, so that counter isn't being
  filled. It's a tree issue for the team; the boat's path was correct.
- **MANUAL** (`/crsd/rc_override`) and **GUIDED** (setpoints) both drive the hull
  correctly. `check_motion` passes all four checks.
- **The hull is a placeholder** until the team sends CAD and a weight. See
  [MODEL_REQUEST.md](MODEL_REQUEST.md).
- **Task 3 runs end to end with the pan/tilt water cannon (2026-10-05).** `task3_cannon.xml`
  passes the referee on `task3` (bay 2), the GREEN bay moved to 1 and 3, and `task3_ul` with
  the camera tilted up 5 deg: docked on the LiDAR 1.1 m from the back wall (the whole hull in
  the slip), backed out to 1.6 m to watch, the fire out after 2 s of water from the aimed
  cannon, the reports and the code right. Checked at the GUI's ~0.45x as well as headless. One double-click:
  `scripts/TASK3_SIM.cmd`. The whole story: [docs/T3_cannon_sim.md](../docs/T3_cannon_sim.md).

```
 Windows                    WSL2 Ubuntu-22.04                         crsd-sim container (ROS 2 Humble)
 ───────                    ─────────────────                         ─────────────────────────────────
 GZ_SIM_UP.cmd ──►  gz sim (Harmonic) ◄──JSON/lockstep──► ArduRover SITL 4.6.3 ─tcp5760─► MAVProxy
 QGC (udp 14550)      hull, thrusters, water,                ▲ RC (udp 5501)          │ 14551 ──► telemetry_bridge
 browser :8090/:8085  MID360, OAK-D, course                  sim_transmitter          │ 14550 ──► tools (check_motion…)
                          │ gz-transport (GZ_PARTITION=crusader_sim)
                          └──────────────────────────────────────► ros_gz_bridge ─► /sim/*
                                                                     livox_shim ─► /livox/lidar ─► lidar_cluster_node
                                                                     sim_camera ─► oak/rgb, oak/depth, crsd/oak/detections
                                                                     sim_uav ─RXL udp 14555─► rxl_link_node
                                                                     target_tracker, ground_station, bt_runner (the boat's)
```

## Run it

**Windows, one click each:**

| Double-click | What it does |
|---|---|
| `crusader_sim/scripts/GZ_SIM_UP.cmd` | syncs your checkout into WSL, builds `crsd-sim` the first time, starts everything, locks the Gazebo camera on the boat, opens the ground station (:8090) and tree viewer (:8085), then **runs Task 1 in its own window after a 20 s countdown** (press N to skip it and drive yourself). Running it again restarts from scratch, with the boat back at the start |
| `crusader_sim/scripts/TASK3_SIM.cmd` | the same on the Task 3 dock (`GZ_SIM_UP.cmd task3`): after the countdown it runs the whole Task 3 with the pan/tilt cannon (`gz_task3.sh`) and prints the referee's verdict. Log: `~/.cache/crusader_sim/task3_last.log` (WSL), the world's `/tmp/task3_world.log` (container) |
| `crusader_sim/scripts/GZ_SIM_DOWN.cmd` | stops the rig and container, SITL, the transmitter and Gazebo, then releases the hidden WSL keep-alive (`gz_keepalive.sh`). WSL itself is left to idle out, because the distro is shared |

The Gazebo window appears on the desktop through WSLg. The last Task 1 run's
output is kept in `~/.cache/crusader_sim/task1_last.log` (WSL; not `/tmp`, which WSL wipes at every distro start).

**WSL** (the same thing, with options):

```bash
bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_sim_up.sh task1_core
bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_sim_up.sh task1_core --no-gui --no-uav
bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_sim_down.sh
```

| Flag | Effect |
|---|---|
| `<course>` | any `courses/*.yaml`: `task1_core`, `task3`, `open_water`, and the avoidance tests `task1_avoid` (4 obstacles on the legs, the panel's default), `task1_unpaired` (red/green buoys that are not pairs: the side fences), `task1_blocked_exit`, `task1_entry_black`, `task1_boxed_in` (S8), `open_water_platform` |
| `--no-gui` | Gazebo server only. The sim is identical, you just can't watch it |
| `--no-uav` | no Ekko stand-in; the boat has only its own camera (Core-tier test) |
| `--no-rig` | stop after Gazebo + SITL, for `check_motion` or your own nodes |
| `--recreate-container` | remove the `crsd-sim` container if it was made from a different image than `RX26_IMAGE` and make a new one. Only with the sim down; see "Nav2 avoidance in the sim" |

Environment, set in front of the command: `NAV_MODE=off`, `shadow` or `on` (the tree's planning, below), `TREE=<xml>` (a name in `crusader_bt/behavior_trees` or a path), `SIM_DETECTOR=truth` or `yolo` (the camera's boxes: the oracle, or the real YOLO; see "The real YOLO detector in the sim"), `RX26_IMAGE` (the image a *new* container is made from; default `crsd-sim:nav2` when it exists, else `crsd-sim:humble`).

### Task 1

```bash
bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_task1.sh task1_core
```

(which is `python3 -m crusader_sim.task1_goal --course task1_core` in `crsd-sim`,
with its output also written to `~/.cache/crusader_sim/task1_last.log`)

It arms, switches to GUIDED through the bridge's `/crsd/set_mode`, and sends the
`SafePassage` goal (approach point = 6 m short of ENTRY). It then prints the
tree's phases until the result. Use `--tier core|advanced|disruptive` to override
the course's tier. To run a different tree, put
`TREE=task1_safe_passage.xml` in front of `gz_sim_up.sh`; a bare name means
`crusader_bt/behavior_trees/`.

Every run is also scored by **`task1_judge`**, an independent referee that sees
only the boat's *true* path (Gazebo ground truth) and the course file. It checks
that EVERY red was kept to starboard and every green to port (each buoy on its own,
paired or not: it is judged at its closest pass), ENTRY was circled clockwise, EXIT
counter-clockwise, and that no buoy was touched. The gate lines in its output are
informational; the verdict is `buoys N/N on their side`. The tree's own
`buoys_passed_correctly` is the tree grading itself; this referee can disagree
with it, which is the point. It also runs standalone:
`python3 -m crusader_sim.task1_judge --course task1_core` (Ctrl-C for the
verdict; live JSON on `/sim/task1_judge`).

### Side fences: red and green buoys that do not pair

`task1_unpaired` (10 buoys: ENTRY, EXIT, 2 red, 3 green, 3 black) has one gate, a lone green on
the course line's right-hand side and a red/green pair with the red on the LEFT of its green.
With `nav_mode on` bt_runner walls each red and green with a row of hazard circles running
outward from it (`nav_fence_*` in crusader_params.yaml, source 4 in `/crsd/nav/hazards`), so the
planner can only pass it on the handbook's side. Scored run (2026-10-02): PASS 5/5 on their
side, 1.4 m clearance, 190 s; the same run with `nav_fence_len_m: 0` FAILs 2/5. crusader_bringup
is not rebuilt by `gz_sim_up.sh` and the rig reads the INSTALLED params yaml: after editing it,
`colcon build --packages-select crusader_bringup` in `crsd-sim` or the run keeps the old values.
The report `/crsd/safe_passage_report` carries `n_gates` (paired gates in the boat's plan),
`gates_cleared` and `single_count`.

### Task 1 Disruptive: you are the UAV (the panel)

Double-click `scripts/TASK1_PANEL.cmd` (desktop shortcut "Crusader Task 1
panel"). It starts `task1_panel` hidden in WSL and opens http://localhost:8095.
The panel is the UAV: in the Disruptive tier the colours are visible only from
the air, so every beacon in the world is unlit and the colours exist only here.

1. **Setup.** Pick a state (RED, GREEN, ENTRY, EXIT, BLACK), click the water to
   place a buoy, drag to move, or edit the table; or load a template. A panel
   that has never launched starts on **`task1_avoid`** (the Load template list
   offers it first): ENTRY, 2 gates, EXIT and 4 black obstacles standing ON the
   straight lines between them, 77 m long, for watching Nav2 route round buoys
   (the obstacle positions and why they are where they are: the header of
   `courses/task1_avoid.yaml`). A panel that launched before keeps its last layout;
   press Load template. Save and load layouts (kept in `~/.cache/crusader_sim/panel/`).
   Exactly one ENTRY and one EXIT are needed.
2. **LAUNCH SIM** builds the world from the layout (`gz_sim_up.sh --course-file
   ... --no-uav`: no auto-acking stand-in) and starts sending your field over
   RXL: the whole field, resent every 5 s, because the boat aborts the mission
   if it is more than 15 s old.
3. **START TASK 1** runs `task1_goal --no-judge`. The boat then asks at each
   checkpoint: **1 = ENTRY orbit done** (asked after the clockwise circle, not
   before), **k+1 = gate k cleared**. Answer ACK, or click buoys to change their
   state (staged until sent) and SEND CHANGES + ACK. A changed field makes the
   boat replan; auto-ACK answers every checkpoint for you. The yellow banner says
   plainly what you are confirming (which gate, as it stands in the field you last
   sent), and what each button does.
4. **The ask after the last gate is the EXIT checkpoint**: with n gates in the field
   you last sent (n = the smaller of the red and the green count), checkpoint n+1
   reads "EXIT gate - confirm exit", and the boat waits for it before it circles the
   EXIT (a sim log, 3 gates: checkpoint 4 asked and awaited). The boat circles the
   EXIT in the latest field, so move the EXIT at that ask at the latest.
5. The referee is the panel's own `task1_judge`, which grades each gate with
   the colours in force when it was crossed.
6. **STOP SIM** stops the sim and keeps the panel (and layout); `GZ_SIM_DOWN.cmd`
   stops everything, panel included. Log: `/tmp/task1_panel.log` (WSL).

Sensor views, each off until toggled and rendered only while shown: RGB
480x300 and depth 320x200 from preview cameras at the OAK-D's pose (the OAK-D
sensors themselves stay full resolution), and a LiDAR top-down view ±25 m in
the boat frame. All three on cost no measurable real-time factor (2026-09-30).

**Two maps, one pan/zoom.** The top map is the truth (solid circles) against what the UAV sent (hollow
circles, joined to the truth), with the referee's gate pairs and the boat's trail. **Boat's map**, below it, draws
only what the BOAT believes (four layers, default on, a toggle each under it; the choice is remembered per
browser), with the true buoys as faint dots to read it against. Pan or zoom either and the other follows; Fit
resets both; "boat's layers here too" overlays the layers on the top map. Buoys are placed, moved and staged on
the top map only. The layers, bottom to top:

| Layer | Topic | Drawn as |
|---|---|---|
| Costmap | Nav2 local costmap, via panel_feed's `costmap` layer `{res_m, cells, lidar, stamp}` | translucent violet squares of side `res_m` where the costmap is lethal or inscribed, and cyan squares for the LiDAR (STVL) voxels; under everything else, so the path and tracks stay readable. A rig whose panel_feed predates the layer sends no `costmap` key: nothing is drawn and the line under the map says so |
| Planned path | `/crsd/nav/leg_status` | the leg dashed in the ground station's colours (green FOLLOWING, red BLOCKED, yellow PLANNING/DEGRADED, grey STRAIGHT), the carrot as a ring, the goal as a cross, and a `NAV <state> <s> <why>` badge on the map's top left |
| Boat's camera tracks | `/crsd/world_targets` | squares, `#id label`, a cross at the estimate, dashed while tentative, fading with time since last seen (`seen N s ago` after 2 s). The **outline** is the boat's own colour vote from the label (`red_buoy`, `green_buoy`, `flashing_blue_buoy`, `steady_blue_buoy`; `black_buoy` from older data), and a grey outline with a `?` is `unknown_buoy`: not confidently seen. Where the fused passage gives that buoy a colour, the **fill** is the UAV's colour, so a grey `?` filled red reads "boat says ?, UAV says red"; a red outline filled green is a disagreement. The pairing is by position (a fused buoy within 0.75 m of the track), because the report carries no track id |
| Boat's fused passage | `/crsd/safe_passage_report` | small diamonds in the colour the UAV gave, at the tracker's position where a track matched and at the UAV's where none did: the tree's own association of the UAV field to its tracks. The tree publishes it only while a Task 1 run is ticking, so it is empty before START. A diamond with no hollow square on it is a buoy the boat has not matched to a track |

With the side beacons off (Disruptive) the colours exist only in the UAV's report, so the tracker's label is
normally `unknown_buoy` and the colour shows only as the fill and in the diamonds (older builds labelled
everything `black_buoy`, which still draws). A square sitting well away from its faint truth dot is the boat's
own mapping error; one with no dot under it is a ghost track. Path over costmap: a dashed path through violet
squares is the planner driving through what the costmap calls lethal, which it should never do.

The panel has no ROS, so `crusader_sim/panel_feed.py` (a sim-only node in `crsd-sim`, started by every
`gz_rig_up.sh`, stopped with the rig) subscribes those topics, converts their lat/lon to course metres
(the inverse of `course.enu_to_latlon`, from the course file's origin) and sends one JSON datagram every 0.25 s
to **udp 127.0.0.1:14556** (`--feed-port` on the panel). 14556 because the boat owns `1455x` and 14550-14553
and 14555 are taken (tooling, `telemetry_bridge`, `batt_watchdog`, the RFD900 shim, RXL); the panel binds it,
so no flight-stack datagram can be stolen. A second panel on the same port says so in the error bar and
runs without the layers. A layer older than 2 s is **stale**: the server returns its age and no data, the
page draws nothing and the line under the map says `STALE 7.3 s, not drawn` (an empty list means "heard,
nothing held"; no line at all means never heard). `leg_status` is sent while a leg runs and once on IDLE, so
between legs the NAV badge reads `stale`. The same data is under `feed` in `curl localhost:8095/api/state`.
Check the arithmetic and the staleness rules, plain python3, no ROS:

```bash
PYTHONPATH=~/robotx_ws/src/rx26_asv/crusader_sim python3 -m crusader_sim.panel_feed --selftest
```

**Found by the panel on 2026-09-30:** `CircleBuoy` drove its orbit points with no obstacle
avoidance (`AvoidObstacles` lived only in the gate leg), so moving the EXIT so that the approach
crosses a buoy ended in a collision. The fix is the tree's planned legs, which plan through
Nav2: `docs/nav2_avoidance_spec.md`, run and tested as "Nav2 avoidance in the sim" below, with
`task1_blocked_exit` and `task1_entry_black` as the reproductions. An orbit entered from far
away can still end short of a full circle (313 degrees measured); the tests below require 330 or more.

### Lake mode: the real boat, you are the UAV

The same panel against the **real** Crusader at a lake, with no simulator (`task1_panel --lake`,
`lake_panel.py`, `lake_panel.html`). It runs inside the `asv` container on the Jetson and is a browser page
for the laptop. `lake_rig_up.sh` / `lake_rig_down.sh` start only what `core.launch.py` does not run; `lake_goal.py`
sends the goal and refuses unless the pilot has armed the boat and chosen GUIDED; the field is resent only while
a browser polls (dead-man); `SAVE AS COURSE` writes a course YAML that replays in this sim. Procedure, safety rules
and troubleshooting: **[`LAKE_MODE.md`](LAKE_MODE.md)**. Tests: `test/test_lake.py`.

### Nav2 avoidance in the sim

The boat's tree plans its legs around known hazards through Nav2's `planner_server` and a
costmap (spec: `docs/nav2_avoidance_spec.md`). In the sim the whole stack runs in `crsd-sim`:
`nav_frames_node` (the datum and TF `map -> base_footprint`), `planner_server`, `nav_lifecycle`
(our own node that configures and activates it, with call timeouts; it replaced Nav2's lifecycle
manager, which hung on one lost reply), and bt_runner with `nav_mode`. It needs the Nav2 image and the nav packages built.

**1. Build the image once** (WSL; the Dockerfile copies nothing from the context, so the small
`docker/` directory is the context). It goes to a NEW tag and touches nothing that is running:

```bash
cd ~/robotx_ws/src/rx26_asv && docker build -t crsd-sim:nav2 -f crusader_sim/docker/crsd-sim.Dockerfile crusader_sim/docker
```

```bash
docker run --rm crsd-sim:nav2 bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | grep -E 'nav2|spatio'"
```

The second line must list `nav2_planner`, `nav2_smac_planner`, `nav2_costmap_2d`,
`nav2_lifecycle_manager` and `spatio_temporal_voxel_layer` among others. The image is about
3.6 GB against 1.8 GB for the old one, mostly the Nav2 and STVL dependencies.

**2. Switch the container.** `docker start` never changes a container's image, so an existing
`crsd-sim` keeps the one it was created from. `gz_sim_up.sh` compares image IDs (not tags) and says so
when they differ; the rig then runs `nav_mode off` and prints `*** AVOIDANCE OFF` rather than start a
tree that waits for a planner. To switch, with the sim **down** (`GZ_SIM_DOWN.cmd`, or
`gz_sim_down.sh`):

```bash
bash /mnt/c/Users/Chaser/Documents/dev/RobotX_2026/Boat/rx26_asv/crusader_sim/scripts/gz_sim_up.sh task1_core --recreate-container
```

Run it from the Windows checkout this first time: the copy under `~/robotx_ws/src` does not have the
flag until step 1 of this very script has synced it.

`--recreate-container` removes the old container, deletes the build and install directories of `crusader_bt` and
`crusader_nav_layers` (their CMake caches hold the old image's `find_package` answers for Nav2),
creates the container from `crsd-sim:nav2` and does one full `colcon build` (a few minutes). The
workspace itself is the bind mount and is not touched otherwise. The by-hand equivalent is
`docker rm -f crsd-sim`, the two `rm -rf`, then a normal `gz_sim_up.sh`. The Task 1 panel runs
`gz_sim_up.sh` without the flag, so do this once first. Anything that lived only inside the old
container (files under `/tmp`, logs) goes with it.

**3. Run.** `NAV_MODE` is `on` by default when Nav2 is present:

```bash
NAV_MODE=on bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_sim_up.sh task1_core
```

```bash
bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_task1.sh task1_core
```

`gz_rig_up.sh` starts `crusader_nav`'s `nav.launch.py` (datum = the course origin) before bt_runner,
waits up to 40 s for `planner_server` to report `active` and says so if it does not, and passes
`-p nav_mode:=$NAV_MODE` to bt_runner. `NAV_MODE=off` starts no nav stack and drives guarded straight legs (a leg holds, then fails at 15 s, rather than cross a known hazard): that is the baseline.
`shadow` plans and displays but drives the legacy legs. Logs: `/tmp/nav.log` and `/tmp/bt.log`
in `crsd-sim`. Each run's output is in `~/.cache/crusader_sim/task1_last.log`; **copy it per run**.
`gz_sim_up.sh` also rebuilds every package a run exercises (`crusader_msgs`, `crusader_bt`,
`crusader_perception`, `crusader_sim`, `crusader_nav`, `crusader_nav_layers`,
`crusader_groundstation`) every run: seconds when unchanged, about 40 s after a `crusader_bt` change.
A stale `crusader_bt` would ignore `nav_mode` and drive the legacy legs. `gz_nav_test.sh` (below) also
checks the mode bt_runner announces and marks a run INVALID when it differs from the one asked for.

**What to look at.** The ground station map (http://localhost:8090) draws the planned path dashed
(green FOLLOWING, red BLOCKED, yellow PLANNING/DEGRADED, grey STRAIGHT), the carrot as a ring, the goal
as a cross, and a top-left badge `NAV <STATE> <s> <why>`; the same JSON is `ros2 topic echo
/crsd/nav/leg_status`. The judge's verdict gains a line, `min clearance <x> m (<object>), non-gate
<y> m, hull <z> m`: centre-to-surface is the planner's contract (0.8 m), non-gate leaves out the
samples inside a gate corridor for that gate's own two buoys, and `clearance_ok` is non-gate at least
0.70 m. It is reported, never part of PASS. Check the arithmetic any time with:

```bash
PYTHONPATH=~/robotx_ws/src/rx26_asv/crusader_sim python3 -m crusader_sim.task1_judge --selftest
```

**The tests** (spec section 10.2). The baseline for every timing is the same commit with
`NAV_MODE=off`; the median of 3 runs must be at most 1.2 times the baseline. The Task 1 rows run
unattended, N fresh runs each, with one scored line per run (result, verdict, clearance, mission
seconds, BLOCKED count) and every log kept in `~/.cache/crusader_sim/runs/<tag>/`:

```bash
tr -d '\r' < /mnt/c/Users/Chaser/Documents/dev/RobotX_2026/Boat/rx26_asv/crusader_sim/scripts/gz_nav_test.sh > /tmp/gz_nav_test.sh && bash /tmp/gz_nav_test.sh task1_core --mode on --runs 3 --tag s1_on
```

(`--mode off|shadow|on`, `--tree X.xml`, `--no-uav`, `-- <task1_goal flags>`. It runs from a `/tmp`
copy because its first step syncs the checkout, which rewrites changed scripts in place.)

| # | Course and setup | Pass |
|---|---|---|
| S1 | `task1_core`, the default Disruptive tree | PASS, `non_gate_centre_m` at least 0.70, no BLOCKED in `/tmp/bt.log` |
| S2 | `task1_core`, `TREE=task1_safe_passage.xml`, `gz_sim_up.sh ... --no-uav` | PASS, clearance at least 0.70 |
| S3 | `task1_blocked_exit` | PASS, `black3` at least 0.73 m, a visible detour, exit orbit at least 330 degrees ccw |
| S4 | `task1_entry_black` | PASS, `black_entry` at least 0.73 m, entry orbit complete |
| S5 | the panel: move the EXIT behind an unpaired buoy mid-run | no contact, exit orbit at least 330 degrees ccw |
| S6 | `open_water_platform`, `TREE=nav_test_line.xml`, a SafePassage goal 40 m east (`approach_latitude 1.2806, approach_longitude 103.8560594`) | arrives, platform at least 0.73 m. Read `/crsd/lidar_cluster_health` first: at the default `water_margin` 0.15 the platform's deck is AT the water gate, so it may not be seen at all, and then run `ros2 param set /lidar_cluster_node water_margin 0.08`. Run at `r_max` 10 and 30 |
| S7 | `task3`, `TREE=task3_disruptive.xml` | as the baseline; look and lead legs at least 0.73 m from fingers and deck; predock and berth STRAIGHT |
| S8 | `task1_boxed_in`: 4 black buoys 1.3 m round the start (at 2 m the planner plans out between them; see the course header) | one hold, costmap clear at 5 s, FAILURE at 15 s, never into a buoy |
| S9 | mid-transit, `SIM_GPS_HDG 0` on SITL (WSL, not `crsd-sim`), then `1` | DEGRADED and hold, no FAILURE, resumes |
| S10 | `NAV_MODE=shadow`, `task1_core` | setpoints identical to the baseline; leg status shows plans |

Stack checks in `crsd-sim`: **N1** `ros2 lifecycle get /planner_server` says `active`; **N2**
`ros2 run tf2_ros tf2_echo map base_footprint` matches the boat's pose to 0.05 m; **N3** a buoy marked
in view and then removed while still in view leaves the costmap within 5 s, and out of view it
persists at least 20 s and is gone by 35 s (if not, switch to the ObstacleLayer fallback in the spec).

### The real YOLO detector in the sim

By default `sim_camera` boxes the buoys from geometry (an oracle that reports what a *working* detector
would). `SIM_DETECTOR=yolo` swaps that for **the team's real detector and LED classifier running on the
rendered 1920x1200 frames** (`crusader_vision/runs/crusader_det_yolo26n.pt` + `crusader_led_cls.pt`), so
you can see what they do on Gazebo renders and what the boat's stack then does with their output.

```bash
# once (WSL): venv + model copy; a 1.5 GB venv under `~/robotx_ws/venvs/yolo`, 2-3 minutes. Never touches the host python
bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/setup_yolo_venv.sh

SIM_DETECTOR=yolo bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_sim_up.sh task1_core --no-gui
bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_sim_up.sh task1_core --detector yolo      # same
bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_nav_test.sh task1_core --detector yolo --tag c5_yolo
```

`gz_nav_test.sh --detector yolo` adds the run's score line to `summary.txt`, and marks the run INVALID if
the rig fell back to the oracle. Extra `sim_camera` ROS args go in `SIM_CAMERA_ARGS`, e.g.
`SIM_CAMERA_ARGS="-p yolo_log:=/tmp/yolo.csv -p yolo_imgsz:=1920 -p yolo_conf:=0.1"`.

**What runs.** `crusader_sim/yolo_detect.py` is `oak_detector._run_pipeline` with the TensorRT engines
replaced by the `.pt` files they are exported from (ultralytics 8.4.144 = the version they were trained
with, CPU torch, 4 threads, ~5 Hz on a worker thread). The decisions are `oak_detector_core`'s own functions,
imported, not copied: `led_patch` (the classifier's crop), `TrackTable`, `FlashTracker`, and the label words
(`flash_red_diamond`, `red_diamond`, `off_diamond`, `diamond`). Every tunable is read from the
`oak_detector` block of `crusader_params.yaml`. The box then goes through the same depth-median patch,
stereo noise and `Detection3DArray` as the oracle's, on `crsd/oak/detections`. The oracle's occlusion check
is not applied (a network does not box what it cannot see).

Deliberate differences from the boat: `marking_shape` is skipped (the shape in the label is the detector's
class; Task 1 never branches on shape); the flash tracker runs on **sim time** (the beacons blink on the sim
clock, and a wall-clock tracker would mis-time them whenever RTF is not 1); inference is on the CPU because the
container has no GPU. If the venv, the models or the label order are wrong the node says
`detector:=yolo is UNAVAILABLE` and runs the oracle (`gz_rig_up.sh` prints the same as a banner).

**The score.** The sim knows where every buoy is, so every ~10 s `/tmp/sim_camera.log` (container) gets:

```
yolo vs truth | last window 50 frames: recall 0.80 (40/50) precision 0.91 (40/44) | colour 36 right / 2 wrong / 2 unresolved | ENTRY/EXIT 3/4 (1 undecided) | mean IoU 0.71 || run 300 frames: ... | recall by range 0-10 m 12/12, 10-20 m 20/28, >20 m 0/0
```

Recall counts buoys at least `min_bbox_px` (24) tall that are not hidden behind another buoy; precision counts
a box on any buoy in view (down to 6 px) as right; colour and ENTRY/EXIT are judged on the matched boxes
(a label with no colour in it is *unresolved*, not wrong; `n/a` when there is nothing to judge: a blank, not a 0). Matching is by IoU >= 0.3 against the idealised
0.43 x 0.41 m silhouette. `yolo_log:=<csv>` writes one row per truth buoy per frame.

**Offline, on any image:** `docker exec crsd-sim /root/robotx_ws/venvs/yolo/bin/python -m crusader_sim.yolo_detect frame.png`
(`--imgsz`, `--conf` as below). Tests without torch: `python3 -m unittest discover -s test -p "test_yolo*.py"`.

| `sim_camera` param | Default | |
|---|---|---|
| `detector` | `truth` | `truth` or `yolo` |
| `yolo_rate_hz` | 5.0 | inferences per sim second, at most; a busy worker skips frames instead of queueing |
| `yolo_threads` | 4 | torch CPU threads |
| `yolo_imgsz`, `yolo_conf` | 0, 0.0 | experiments only. 0 = the boat's `det_imgsz_*` (640) and `det_conf_min` (0.60) |
| `yolo_model_dir`, `yolo_device`, `yolo_log` | `~/robotx_ws/models/sim_yolo`, `cpu`, none | |

**Cost.** The RGB stream is rendered and bridged for the whole run (1920x1200 at 15 Hz is the expensive
thing in this sim), plus ~100 ms of CPU per inference at 640. Watch RTF the first time.

**What it does on Gazebo renders (first measurement, 2026-10-01).** `task1_core`, with the UAV stand-in,
`gz_nav_test.sh task1_core --mode on --detector yolo --tag c5_yolo`: the judge said PASS (3/3 gates, no
contact, 163 s; the same run on the oracle took 169 s with 0.78 m minimum clearance, this one 0.52 m), but
the passage came from the UAV's plan. The boat's own camera contributed little:

- **Detection.** At the boat's own settings (640 input, conf 0.60) recall was 0.23 (194/850) with precision
  0.98: 58 % of buoys closer than 10 m (178/307), 3 % at 10-20 m (16/543). A buoy 14-40 px tall in the
  1920x1200 frame is 5-13 px at the detector's 640 input, and at the start line the model boxed nothing at all.
  Raising the input to 1920 and lowering the floor to 0.1 (`yolo_imgsz`, `yolo_conf`; not what the boat runs)
  found 2 of 6 buoys at ~290 ms a frame. On the boat's own 640x400 footage from 2026-09-14
  (`Boat/20260914-010054/camera`) the same detector boxes buoys 11-84 px tall (median 33): size matters, not
  only render style.
- **Colour.** 0 of the 194 matched boxes carried a colour: the labels stayed bare (`diamond`), the flash
  tracker never resolved a light state (it wants 8 samples over 4 s on one track, and the classifier hedges).
  Given perfect boxes on one frame, the LED classifier read the two green buoys right, one red buoy as
  off (possibly its dark phase) and the other as blue, and both unlit buoys as blue or green (0.4-0.98
  confidence). The sim's LED crops are 4-15 px; the ones it was trained on are ~87x48.

Treat recall by range in the log line as the answer to "from how far does the model see a Gazebo buoy",
and `colour ... unresolved` as the flash tracker's, not the model's, verdict.

### Sensor model (checked against the spec pages and Resources.md, 2026-09-30)

**MID-360 LiDAR:**
- Scans only the 180° in front of the sensor (−90..+90° about its forward
  axis), because the real rear half sees only the boat's own hull. The boat's
  clustering already uses 180° (`crusader_params.yaml:321`).
- Keeps the 0.6° sample step, which gives about 10k points per scan at 10 Hz.
- Vertical FOV −7..+52°, range 0.1–40 m, 2 cm noise.
- Mounted upside down at the bow, so the raw frame is x forward, y starboard,
  z down.
- Its mount matches Resources.md:112-115.

**OAK-D LR camera:**
- HFOV 82°. Colour 1920x1200 and depth 640x400, both at 15 Hz.
- Depth is aligned to the colour camera (`oak_pipeline.py:170`), so one HFOV
  serves colour, depth and the preview cameras.
- Depth range 0.58–30 m. The 0.58 m is derived from the 15 cm baseline and
  95 px of disparity.
- Its mount matches Resources.md:131-135. Its yaw and pitch are still assumed
  to be 0.

The panel's LiDAR view hatches everything aft of the sensor as "no coverage".

### GUIDED and MANUAL, and switching between them

The sim switches modes exactly as the boat does. There is no sim-only mode path.

| Mode | How it's commanded | Topic (in `crsd-sim`) |
|---|---|---|
| switch mode | the bridge's own path | `ros2 topic pub --once /crsd/set_mode std_msgs/msg/String "{data: MANUAL}"` (or `GUIDED`, `HOLD`) |
| switch mode, by "the pilot" | the SC switch on the simulated transmitter | `python3 -m crusader_sim.sim_transmitter set mode manual` (`hold`, `auto`) |
| GUIDED position | `SET_POSITION_TARGET_GLOBAL_INT` | `/crsd/guided_setpoint` (what `bt_runner` sends) |
| GUIDED heading + speed | `SET_ATTITUDE_TARGET` | `/crsd/guided_heading_speed` |
| **MANUAL direct** | `RC_CHANNELS_OVERRIDE` ch1 steer / ch3 throttle / ch4 lateral | `/crsd/rc_override` (`crusader_msgs/RcChannels`) |

**MANUAL direct control** is what the Task 3 fixed-nozzle shot uses on the real
boat, and it's the path you asked for:

```bash
docker exec -it crsd-sim bash -lc "python3 -m crusader_sim.manual_drive demo"   # scripted, measured
docker exec -it crsd-sim bash -lc "python3 -m crusader_sim.manual_drive keys"   # drive it: w/s a/d q/e
```

`telemetry_bridge` applies its override rules here just as on the water:
MANUAL only, ch1/3/4 only, ±150 µs, released after 0.5 s of silence, blocked
while the autonomy-drop latch is tripped. Measured: surge 5.7 m forward, sway
3.1 m to starboard, yaw 184° clockwise, each in 6 s at full deflection.

> **"Thruster commands" means surge/sway/yaw, not four separate motors.** That's
> what the real boat can do through ArduPilot today: the OmniX mixer owns the
> four outputs, and ArduPilot refuses `DO_SET_SERVO` on motor channels.
> Per-thruster control on the real boat would need one of these, a **team
> decision** because it changes the boat:
> (a) `SERVO1..4_FUNCTION` = RCIN9..12 plus overrides on ch9–12, which
> bypasses the mixer in *every* mode;
> (b) a Lua script taking the outputs with `SRV_Channels:set_output_pwm_chan_timeout`
> in a chosen mode (check that Pixhawk1 has the flash and RAM for scripting);
> (c) a Jetson-side mixer feeding (a).
> Whichever you pick, the sim will run it unchanged, because it runs the real
> firmware.

### The transmitter

`sim_transmitter` is the RadioMaster: raw RC into SITL, below MAVLink, as the
real receiver does.

```bash
python3 -m crusader_sim.sim_transmitter set estop on|off     # SB (ch7)
python3 -m crusader_sim.sim_transmitter set mode manual|hold|auto   # SC (ch8)
python3 -m crusader_sim.sim_transmitter set drop on|off      # autonomy-drop (ch9)
python3 -m crusader_sim.sim_transmitter show
```

(From WSL, set `PYTHONPATH=~/robotx_ws/src/rx26_asv/crusader_sim` first.)
After SITL boots, `gz_sim_up.sh` flicks SB e-stop → run once, because
ArduPilot latches whatever e-stop state it read at boot.

### Checks

```bash
PYTHONPATH=~/robotx_ws/src/rx26_asv/crusader_sim python3 -m crusader_sim.check_motion
```

MANUAL surge/sway/yaw pulses on the bridge's override channels, then a 20 m
GUIDED leg, each judged in the body frame. Run it after any change to
`crusader_hull.yaml`. A wrong sign here is the sim's version of "perfect in
manual, spins in AUTO". Fix it in the YAML, never in the params.

## What's faithful, and what isn't (yet)

| Faithful | Not (yet) |
|---|---|
| ArduRover **4.6.3**, the boat's params (`sitl_params.py` whitelist, every difference listed in `config/sitl_overlay.parm`) | **Hull shape, mass, drag**: placeholders ([MODEL_REQUEST.md](MODEL_REQUEST.md)) |
| OmniX mixing and **lateral motion**, with SERVO1/4 reversal | Thruster positions and angles are *derived from the mixer*, not measured |
| Yaw from GPS, compass off (NMEA HDT instead of Unicore moving baseline) | Waves, wind, current (flat water) |
| MAVProxy port map, 5 Hz HEARTBEAT, `--streamrate=-1` | MID360's non-repetitive scan pattern (a 600 × 34 grid instead) |
| MID360 mount from `lidar_*` params, **upside down**, Livox point format | Camera **detections** come from ground truth (the OAK-D NN can't run in a sim) unless `SIM_DETECTOR=yolo` runs the real `.pt` models on the frames (CPU, not the OAK's engines). The **frames** are real renders on `oak/rgb` |
| OAK-D mount from `cam_*` params; RGB 1920×1200 and depth 640×400 as `oak_detector` runs them | Beacons are lit steadily; the 1 s on/off flash isn't animated yet |
| RC e-stop, mode switch and autonomy-drop, on their real channels | Colour indicator, pump/water stream, Task 3 not yet flown |
| The RobotX 2026 light beacon (new 2026-09-14 design), RoboBuoy, dock from `task3_sim` | Task 1 layout is **illustrative**; the handbook gives it only as an image |

## Files

| Path | Runs in | What |
|---|---|---|
| `scripts/GZ_SIM_UP.cmd`, `GZ_SIM_DOWN.cmd` | Windows | double-click launchers |
| `scripts/gz_sim_up.sh`, `gz_sim_down.sh` | WSL | the order of operations |
| `scripts/gz_task1.sh`, `gz_keepalive.sh` | WSL | one Task 1 run, logged; the hidden client that stops WSL idling out |
| `scripts/gz_sync.sh` | WSL | Windows checkout -> `~/robotx_ws/src` (shared by the sim and the panel) |
| `scripts/TASK1_PANEL.cmd`, `task1_panel_up.sh` | Windows / WSL | double-click the panel; sync, then run it on :8095 |
| `crusader_sim/task1_panel.py`, `.html`, `panel_sensors.py` | WSL host (no ROS) | the UAV panel: layout, RXL link (`tools/bench/uav_link.py`), live referee, sensor views over gz-transport |
| `crusader_sim/panel_common.js`, `.css` | (served by the panel) | the map, layers, field editing and look both pages share; `gz_sync.sh` copies them |
| `crusader_sim/lake_panel.py`, `lake_panel.html`, `lake_goal.py`, `goal_client.py` | `asv` on the Jetson | LAKE MODE (`LAKE_MODE.md`): the real boat, the panel plays the UAV; `goal_client` is the goal-sending/cancel code `task1_goal` shares |
| `scripts/lake_rig_up.sh`, `lake_rig_down.sh` | `asv` on the Jetson | start / stop what lake mode adds to `core.launch.py` (by recorded pid) |
| `scripts/gz_rig_up.sh`, `gz_rig_down.sh` | `crsd-sim` | the ROS rig (the headless rig's `task1_sim_up.sh`, with sensors from Gazebo) |
| `config/crusader_hull.yaml` | — | **the placeholder boat.** Edit this when CAD arrives |
| `config/sitl_overlay.parm` | — | every place SITL differs from the boat, and why |
| `config/gz_bridge.yaml` | — | Gazebo → ROS topic map |
| `courses/*.yaml` | — | course layouts: buoys, beacon states, dock. `task1_avoid`, `task1_blocked_exit`, `task1_entry_black` and `open_water_platform` are the avoidance tests |
| `crusader_sim/gen_crusader.py`, `gen_world.py` | WSL | build model and world into `~/.cache/crusader_sim` |
| `crusader_sim/livox_shim.py`, `sim_camera.py` | `crsd-sim` | Gazebo sensors → the boat's driver topics |
| `crusader_sim/yolo_detect.py`, `scripts/setup_yolo_venv.sh` | `crsd-sim` / WSL | the real YOLO + LED classifier on sim frames and its score; the venv + model-copy script (`SIM_DETECTOR=yolo`) |
| `crusader_sim/sim_uav.py` | `crsd-sim` | Ekko's Task 1 radio, from the course's truth |
| `crusader_sim/task3_world.py` | `crsd-sim` | Task 3: RoboCommand's lights, the dock detector's DockObservation from geometry (`tools/task3_sim/world.py`'s), the water, the referee |
| `crusader_sim/task3_gz_agent.py` | WSL | Task 3: recolours the dock's windows and draws the water in Gazebo, on task3_world's word (udp 14558) |
| `crusader_sim/task3_goal.py`, `scripts/gz_task3.sh` | `crsd-sim` / WSL | the Task 3 operator: arm, WP_RADIUS 0.3, GUIDED, latch reset, goal, MANUAL at the hand-over, the verdict |
| `crusader_sim/sim_transmitter.py` | WSL | the RC transmitter |
| `crusader_sim/task1_goal.py`, `manual_drive.py`, `check_motion.py` | `crsd-sim` / WSL | operator tools |
| `crusader_sim/task1_judge.py` | `crsd-sim` | independent Task 1 referee, from ground truth; also the minimum-clearance figure (`--selftest`) |
| `docker/crsd-sim.Dockerfile` | WSL | the x86 stand-in for the boat's `asv` image, with Nav2 + STVL (`crsd-sim:nav2`) |
| `setup/install_wsl.sh` | WSL | Gazebo, ArduPilot `Rover-4.6.3`, ardupilot_gazebo (`apt` stage as root, `user` stage as you) |

## Troubleshooting, all learned the hard way on 2026-09-28/29

| Symptom | Cause |
|---|---|
| Real-time factor ~0.2, "fcu_status stale", the tree aborts with OUTCOME_NOT_AUTONOMOUS | **Rendering on the CPU.** WSLg's Mesa defaults to `llvmpipe` on this PC (`glxinfo -B`: "Accelerated: no"). `gz_sim_up.sh` sets `GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`, which gives "D3D12 (NVIDIA GeForce RTX 5060 Ti)". If a driver update breaks that, `GZ_GPU=cpu` forces software rendering: slow, but it runs |
| No Gazebo window; `/tmp/gz_gui.log` ends in a segfault under `glXChooseFBConfig` / `libnvwgf2umx.so` | NVIDIA's WSL driver occasionally crashes the GUI at startup (the server is unaffected). `gz_sim_up.sh` checks and relaunches it, and the third try renders only the window on the CPU. By hand: `GZ_PARTITION=crusader_sim GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA gz sim -g` |
| Arms, but motors sit at neutral; "Motors Emergency Stopped" | SITL's default RC holds ch7 (SB, `RC7_OPTION=165`) at 1000 µs = e-stop, and ArduPilot latches it at boot. `sim_transmitter set estop on`, then `off` |
| "PreArm: Gyros inconsistent" for ~10–20 s after boot | normal; `check_motion`/`task1_goal` retry. Don't arm the instant SITL starts |
| An override is silently ignored | `RC_CHANNELS_OVERRIDE` is only accepted from sysid 255 (`SYSID_MYGCS`) |
| `/livox/lidar` lists but never delivers, or `/sim/*` is empty | `GZ_PARTITION` must be `crusader_sim` on both sides; the container needs `--ipc=host` |
| Everything dies about a minute after the last terminal closes | WSL2 stops an idle VM. The `.cmd` launchers hold it open; from a terminal, keep one WSL shell open |
| The boat capsizes in pitch | don't un-segment the pontoons (`buoyancy_segments`): gz's box buoyancy has no pitch restoring moment for one long box |
| A `pkill -f` in a one-liner kills its own shell | put patterns in a file, as `tools/sitl/rig_processes.txt` and `scripts/gz_rig_processes.txt` do |
