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
  outcome SUCCESS, 10/10 buoys classified, in 176 s.
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

**Windows:** double-click `crusader_sim/scripts/GZ_SIM_UP.cmd`. It syncs your
checkout into WSL, builds the workspace in `crsd-sim` the first time (about a
minute), starts everything, and opens the ground station (:8090) and the tree
viewer (:8085). The Gazebo window appears on the desktop through WSLg. Stop with
`GZ_SIM_DOWN.cmd`.

**WSL** (the same thing, with options):

```bash
bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_sim_up.sh task1_core
bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_sim_up.sh task1_core --no-gui --no-uav
bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_sim_down.sh
```

| Flag | Effect |
|---|---|
| `<course>` | any `courses/*.yaml`: `task1_core`, `task3`, `open_water` |
| `--no-gui` | Gazebo server only. The sim is identical, you just can't watch it |
| `--no-uav` | no Ekko stand-in; the boat has only its own camera (Core-tier test) |
| `--no-rig` | stop after Gazebo + SITL, for `check_motion` or your own nodes |

### Task 1

```bash
docker exec -it crsd-sim bash -lc "python3 -m crusader_sim.task1_goal --course task1_core"
```

It arms, switches to GUIDED through the bridge's `/crsd/set_mode`, and sends the
`SafePassage` goal (approach point = 6 m short of ENTRY). It then prints the
tree's phases until the result. Use `--tier core|advanced|disruptive` to override
the course's tier. To run a different tree, put
`TREE=task1_safe_passage.xml` in front of `gz_sim_up.sh`; a bare name means
`crusader_bt/behavior_trees/`.

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
| `scripts/gz_rig_up.sh`, `gz_rig_down.sh` | `crsd-sim` | the ROS rig (the headless rig's `task1_sim_up.sh`, with sensors from Gazebo) |
| `config/crusader_hull.yaml` | — | **the placeholder boat.** Edit this when CAD arrives |
| `config/sitl_overlay.parm` | — | every place SITL differs from the boat, and why |
| `config/gz_bridge.yaml` | — | Gazebo → ROS topic map |
| `courses/*.yaml` | — | course layouts: buoys, beacon states, dock |
| `crusader_sim/gen_crusader.py`, `gen_world.py` | WSL | build model and world into `~/.cache/crusader_sim` |
| `crusader_sim/livox_shim.py`, `sim_camera.py` | `crsd-sim` | Gazebo sensors → the boat's driver topics |
| `crusader_sim/sim_uav.py` | `crsd-sim` | Ekko's Task 1 radio, from the course's truth |
| `crusader_sim/sim_transmitter.py` | WSL | the RC transmitter |
| `crusader_sim/task1_goal.py`, `manual_drive.py`, `check_motion.py` | `crsd-sim` / WSL | operator tools |
| `docker/crsd-sim.Dockerfile` | WSL | the x86 stand-in for the boat's `asv` image |
| `setup/install_wsl.sh` | WSL | Gazebo, ArduPilot `Rover-4.6.3`, ardupilot_gazebo (`apt` stage as root, `user` stage as you) |

## Troubleshooting, all learned the hard way on 2026-09-28/29

| Symptom | Cause |
|---|---|
| Arms, but motors sit at neutral; "Motors Emergency Stopped" | SITL's default RC holds ch7 (SB, `RC7_OPTION=165`) at 1000 µs = e-stop, and ArduPilot latches it at boot. `sim_transmitter set estop on`, then `off` |
| "PreArm: Gyros inconsistent" for ~10–20 s after boot | normal; `check_motion`/`task1_goal` retry. Don't arm the instant SITL starts |
| An override is silently ignored | `RC_CHANNELS_OVERRIDE` is only accepted from sysid 255 (`SYSID_MYGCS`) |
| `/livox/lidar` lists but never delivers, or `/sim/*` is empty | `GZ_PARTITION` must be `crusader_sim` on both sides; the container needs `--ipc=host` |
| Everything dies about a minute after the last terminal closes | WSL2 stops an idle VM. The `.cmd` launchers hold it open; from a terminal, keep one WSL shell open |
| The boat capsizes in pitch | don't un-segment the pontoons (`buoyancy_segments`): gz's box buoyancy has no pitch restoring moment for one long box |
| A `pkill -f` in a one-liner kills its own shell | put patterns in a file, as `tools/sitl/rig_processes.txt` and `scripts/gz_rig_processes.txt` do |
