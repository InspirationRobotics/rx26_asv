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
- The Task 3 dock world is built and valid, but not yet flown.

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
| `<course>` | any `courses/*.yaml`: `task1_core`, `task3`, `open_water`, and the avoidance tests `task1_blocked_exit`, `task1_entry_black`, `task1_boxed_in` (S8), `open_water_platform` |
| `--no-gui` | Gazebo server only. The sim is identical, you just can't watch it |
| `--no-uav` | no Ekko stand-in; the boat has only its own camera (Core-tier test) |
| `--no-rig` | stop after Gazebo + SITL, for `check_motion` or your own nodes |
| `--recreate-container` | remove the `crsd-sim` container if it was made from a different image than `RX26_IMAGE` and make a new one. Only with the sim down; see "Nav2 avoidance in the sim" |

Environment, set in front of the command: `NAV_MODE=off`, `shadow` or `on` (the tree's planning, below), `TREE=<xml>` (a name in `crusader_bt/behavior_trees` or a path), `RX26_IMAGE` (the image a *new* container is made from; default `crsd-sim:nav2` when it exists, else `crsd-sim:humble`).

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
that red was kept to starboard, green to port, ENTRY was circled clockwise, EXIT
counter-clockwise, and that no buoy was touched. The tree's own
`buoys_passed_correctly` is the tree grading itself; this referee can disagree
with it, which is the point. It also runs standalone:
`python3 -m crusader_sim.task1_judge --course task1_core` (Ctrl-C for the
verdict; live JSON on `/sim/task1_judge`).

### Task 1 Disruptive: you are the UAV (the panel)

Double-click `scripts/TASK1_PANEL.cmd` (desktop shortcut "Crusader Task 1
panel"). It starts `task1_panel` hidden in WSL and opens http://localhost:8095.
The panel is the UAV: in the Disruptive tier the colours are visible only from
the air, so every beacon in the world is unlit and the colours exist only here.

1. **Setup.** Pick a state (RED, GREEN, ENTRY, EXIT, BLACK), click the water to
   place a buoy, drag to move, or edit the table; or load the `task1_core`
   template. Save and load layouts (kept in `~/.cache/crusader_sim/panel/`).
   Exactly one ENTRY and one EXIT are needed.
2. **LAUNCH SIM** builds the world from the layout (`gz_sim_up.sh --course-file
   ... --no-uav`: no auto-acking stand-in) and starts sending your field over
   RXL: the whole field, resent every 5 s, because the boat aborts the mission
   if it is more than 15 s old.
3. **START TASK 1** runs `task1_goal --no-judge`. The boat then asks at each
   checkpoint: **1 = ENTRY orbit done** (asked after the clockwise circle, not
   before), **k+1 = gate k cleared**. Answer ACK, or click buoys to change their
   state (staged until sent) and SEND CHANGES + ACK. A changed field makes the
   boat replan; auto-ACK answers every checkpoint for you.
4. **There is no EXIT checkpoint** in the boat's tree: it uses the EXIT in the
   latest field when it starts its exit orbit, so move the EXIT at the last
   gate's checkpoint at the latest.
5. The referee is the panel's own `task1_judge`, which grades each gate with
   the colours in force when it was crossed.
6. **STOP SIM** stops the sim and keeps the panel (and layout); `GZ_SIM_DOWN.cmd`
   stops everything, panel included. Log: `/tmp/task1_panel.log` (WSL).

Sensor views, each off until toggled and rendered only while shown: RGB
480x300 and depth 320x200 from preview cameras at the OAK-D's pose (the OAK-D
sensors themselves stay full resolution), and a LiDAR top-down view ±25 m in
the boat frame. All three on cost no measurable real-time factor (2026-09-30).

**The boat's own picture** (three map layers, default on, a toggle each under the map; the choice is
remembered per browser). Solid circles are the true buoys and hollow circles the UAV's report, as before;
these show what the BOAT thinks:

| Layer | Topic | Drawn as |
|---|---|---|
| Planned path | `/crsd/nav/leg_status` | the leg dashed in the ground station's colours (green FOLLOWING, red BLOCKED, yellow PLANNING/DEGRADED, grey STRAIGHT), the carrot as a ring, the goal as a cross, and a `NAV <state> <s> <why>` badge on the map's top left |
| Boat's camera tracks | `/crsd/world_targets` | hollow squares in the colour of the track's label, `#id label`, a cross at the estimate, dashed while tentative, fading with time since last seen (`seen N s ago` after 2 s) |
| Boat's fused passage | `/crsd/safe_passage_report` | small diamonds in the colour the UAV gave, at the tracker's position where a track matched and at the UAV's where none did: the tree's own association of the UAV field to its tracks. The tree publishes it only while a Task 1 run is ticking, so it is empty before START. A diamond with no hollow square on it is a buoy the boat has not matched to a track |

With all beacons unlit (Disruptive) the tracks read `black_buoy`: that is correct, the colours exist only in
the UAV's report and so only in the diamonds. A hollow square sitting well away from its solid circle is
the boat's own mapping error; one with no circle under it is a ghost track.

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
| MID360 mount from `lidar_*` params, **upside down**, Livox point format | Camera **detections** come from ground truth (the OAK-D NN can't run in a sim). The **frames** are real renders on `oak/rgb` |
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
| `scripts/gz_rig_up.sh`, `gz_rig_down.sh` | `crsd-sim` | the ROS rig (the headless rig's `task1_sim_up.sh`, with sensors from Gazebo) |
| `config/crusader_hull.yaml` | — | **the placeholder boat.** Edit this when CAD arrives |
| `config/sitl_overlay.parm` | — | every place SITL differs from the boat, and why |
| `config/gz_bridge.yaml` | — | Gazebo → ROS topic map |
| `courses/*.yaml` | — | course layouts: buoys, beacon states, dock. `task1_blocked_exit`, `task1_entry_black` and `open_water_platform` are the avoidance tests |
| `crusader_sim/gen_crusader.py`, `gen_world.py` | WSL | build model and world into `~/.cache/crusader_sim` |
| `crusader_sim/livox_shim.py`, `sim_camera.py` | `crsd-sim` | Gazebo sensors → the boat's driver topics |
| `crusader_sim/sim_uav.py` | `crsd-sim` | Ekko's Task 1 radio, from the course's truth |
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
