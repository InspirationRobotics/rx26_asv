# Task 3 with the pan/tilt water cannon: the Gazebo sim and the boat

Task 3 (Coordinated Logistics) redone for a 2-DOF pan/tilt cannon, the way Team Bumblebee
aim theirs: the boat docks on the **LiDAR** and holds still, and the **camera** aims the
cannon at the burning window every frame while the water runs. Branch `task3-gazebo`
(Chase's `sim/gazebo` + `task3-disruptive`), 2026-10-05.

## Run it

**In the sim (Windows):** double-click `crusader_sim/scripts/TASK3_SIM.cmd`. It brings up
Gazebo on the Task 3 dock, counts down 20 s, then runs the whole task and prints the
referee's verdict. `GZ_SIM_DOWN.cmd` stops everything. Any dock course from a terminal,
e.g. `TASK3_SIM.cmd task3_bay1` (the same as `GZ_SIM_UP.cmd task3_bay1`):

| Course | GREEN bay | The fire |
|---|---|---|
| `task3` (the default) | 2, the middle | lower-right window |
| `task3_bay1` | 1, the left (as seen from the water) | lower-right window |
| `task3_bay3` | 3, the right | lower-right window |
| `task3_ul` | 2 | UPPER-LEFT window: needs the camera tilted up ~5 deg |

The GREEN bay is fixed by the course file, not random: the indicator colours are baked
into the Gazebo world when it is generated.

**On the boat** (inside the `asv` container, boat facing the dock, GUIDED on SC):

```bash
docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/run.sh dry
```

`shadow` logs only, `dry` takes the boat and aims but every burst is DRY, `live` fires the
pump. `dock_view` must be running first (ground station, Nodes tab). When the tree says
**"waiting for the pilot to select MANUAL (SC)"**, flip SC to MANUAL: the tree never changes
the mode itself. `TIER=0|1|2` (default 2) and `TIMEOUT_S` (900) in front of it if needed.

## What runs

| | Where | What |
|---|---|---|
| `task3_cannon.xml` | bt_runner_node | the tree, one run (below) |
| `dock_slot_node` | crusader_perception | `/livox/lidar` -> `/crsd/dock_slot`: the slip's side walls and back wall (`slot_fit_core`) |
| `cannon_aim_node` | crusader_fcu | `/crsd/water_cannon` (the window's aim point) -> pan/tilt on `/crsd/cannon_cmd`, bursts on `/crsd/pump_cmd` (`cannon_aim_core`) |
| telemetry_bridge | crusader_fcu | `/crsd/cannon_cmd` -> `MAV_CMD_DO_SET_SERVO` on SERVO11 (AUX3, pan) and SERVO12 (AUX4, tilt), clamped 1100-1900 us; `/crsd/cannon_state` from SERVO_OUTPUT_RAW |
| `task3_world` | crusader_sim (sim only) | RoboCommand's lights, the dock detector's `DockObservation` from geometry, the water, the referee |
| `task3_gz_agent` | crusader_sim, WSL host (sim only) | recolours the windows and draws the water in Gazebo |

### The tree (`crusader_bt/behavior_trees/task3_cannon.xml`)

1. **Approach (GUIDED)**: `task3_part1_approach.xml`'s legs. Survey, pick the one GREEN
   bay (strict GREEN + RED + RED), stop on its centreline 3.5 m out.
2. **Hand-over**: `AwaitMode MANUAL`. The pilot flips SC; in the sim `task3_goal` does it.
   The dock book is kept, so no LookAround. Then `FaceBay` turns the bow into the slip on
   the yaw stick, because GUIDED setpoints carry no heading (bay 1 arrived 74 deg off). The
   LiDAR then gets 4 s to find the slip.
3. **Dock (MANUAL)**: `SlotKeep` on the LiDAR alone. It holds the LiDAR **1.1 m from the
   back wall** (the deck edge between the fingers), the hull on the slip's centreline, and
   the bow square to the side walls. At 1.1 m the stern is ~8 cm inside the 2 m fingers'
   tips, so the whole hull stays in the slip across the ±6 cm tolerance (the sim's reports
   came at 1.03-1.17 m: stern 1-15 cm inside; at 1.2 m one run reported with it 8 cm out).
   Same controller and gains as StrafeKeep, plus a brake inside the deadband
   (`brake_mps`). Docked = the standoff to ±6 cm, the hull within 10 cm of the centreline,
   square to 5 deg, for 2 s; then `ReportDocking`.
4. **Fire**: the hold backs out to the **1.6 m watch range** (bow still in the slip).
   `AwaitFireTarget` (the timing layer's steady RED), then `SprayUntilHit`: every tick it
   puts the window's camera-frame aim point on `/crsd/water_cannon`, and `cannon_aim_node`
   turns it into pan/tilt plus bursts. `ReportFirefighting` when GREEN.
5. **Request** (Advanced/Disruptive), still from 1.6 m: `DecodeResourceRequest`, then
   `ReportResourceRequest` to RoboCommand and the UAV.

The standoff is one blackboard value, `standoff`, set by `Script` nodes in the tree
(1.1 to dock, 1.6 to watch). `SlotKeep` reads it every tick.

**Why it backs out.** The CV masks the top 188 rows (`hull_band_rows`), so the camera sees
nothing more than ~19 deg above its axis. How much of each window the sim's camera sees
(its 5x5 sample of the opening; under 50% is not reported):

| LiDAR standoff | camera | lower-right | upper-left |
|---|---|---|---|
| 1.1 m | level | 40% | 0% |
| 1.1 m | tilted up 5 deg | 80% | 0% |
| 1.1 m | tilted up 12 deg | 100% | 60% |
| 1.2 m | level | 60% (the fire yes; the code no: the hold's ±5 cm drops it) | 0% |
| 1.6 m | level | 100% | 20% |
| 1.6 m | tilted up 5 deg | 100% | 80% |

So the boat docks fully in, then backs out to 1.6 m for the fire and the code. A first
version fought the fire from the berth whenever it saw it there; bay 3 settled at 1.16 m,
saw the fire, put it out, but never saw the window turn GREEN, and sprayed until the
timeout. Tilted up 12-15 deg, the camera could do everything from the berth: then drop the
`standoff := 1.6` line.

New leaves (`src/fire_leaves.cpp`): `DockSlotAlive`, `SlotKeep`, `AwaitMode`, `FaceBay`.
New in `fire_math.hpp`: `StrafeParams.brake_mps` and `bandBrake()`. NaN by default, so the
fixed-nozzle trees behave exactly as before.

### The slip from the LiDAR (`DockSlot`)

| Field | Meaning |
|---|---|
| `valid`, `why` | both side walls (or one plus the 1.5 m slip width) and the back wall found |
| `angle_deg` | the slip's axis; + = it points LEFT of the bow (turn left to square up) |
| `lateral_m` | the slip's centreline at the body origin; + = LEFT of the boat |
| `width_m`, `left_m`, `right_m`, `n_left`, `n_right` | the two walls |
| `back_range_m`, `n_back` | **LiDAR -> back wall** along the slip: the standoff |

Side walls: per side, the nearest strong line running along the bow. Back wall: the
nearest band across the slip that fills at least half its width and stands at least 10 cm
tall. That rejects stray water returns, which once made a "wall" 0.23 m ahead. On ray-cast
sweeps of the build-guide dock with Gazebo's 2 cm noise, over 480 random poses: all valid,
standoff within 1.4 cm. In Gazebo, with the boat still: LiDAR 1.581 m, truth 1.573 m.

### The aim (`crusader_fcu/cannon_aim_core.py`)

```
camera point --(cam_x/y/z, cam_pitch)--> body --(minus the pivot)--> level by roll/pitch
  --> low-arc drag-free throw at v --> back to the hull --> pan, tilt --> PWM (ServoMap)
```

- `v = sqrt(g * 3.0 / sin 90) = 5.42 m/s`, from the 3.0 m throw at 45 deg.
- Pan + = LEFT, tilt + = UP.
- Checked by flying the water along the solved angles: lands within 1 cm, rolled and
  pitched up to 6 deg.
- Fires once SERVO_OUTPUT_RAW says both servos have sat on the command for 0.4 s; then
  1 s bursts every 2.1 s while the tree asks.

**Assumed until measured** (all in `crusader_params.yaml`, `cannon_aim_node`):

- the pivot at (0.47, 0.30, 0.75) m in the body frame: the camera + 10 cm ahead, 30 cm
  left, 10 cm up;
- 1000-2000 us = -90..+90 deg on both servos, signs +1;
- the exit speed.

## In the sim

- `task3_world` runs the team's own `tools/task3_sim/world.py` (Lights, Judge, the Camera
  that emits the CV team's DockObservation with their timing stage) on **Gazebo's** truth.
- The water is a drag-free parabola from the cannon's true pivot and joint angles, crossed
  with the face.
- RoboCommand lights the window when the docking report names the GREEN bay **and** the
  bow is in it; GREEN after 2.0 s of water on the window.
- The deck is 0.3 m high.
- The pan/tilt joints are driven by SITL's SERVO11/12 through the same bridge path as the
  boat, so the sim tests the MAVLink plumbing too.

Final runs, all on the same code (2026-10-05), with the sim slowed to the GUI's 0.45x
(headless, `set_physics real_time_factor 0.45`):

| Run | Result |
|---|---|
| `task3` (bay 2, lower-right window, level camera) | **PASS** in 115 s: docked at LiDAR 1.03 m, centred to 1 cm, 1.1 deg off square; out after 2.0 s of water; code RED -> BLUE reported and relayed |
| GREEN bay = 1 | **PASS** in 161 s (docked at 1.15 m) |
| GREEN bay = 3 | **PASS** in 159 s (docked at 1.16 m) |
| `task3_ul`, camera tilted up 5 deg (`cam_pitch_deg -5`) | **PASS** in 107 s (docked at 1.17 m) |
| unit tests: crusader_bt (9 suites), fcu 64, perception 62, sim 273, squirt_cal 52, task3_sim, check_config | **all pass** |

The LiDAR figures are the referee's truth when the docking report went out. Across these runs
and the ones before them it was 1.03-1.17 m: the stern 1.85-1.99 m out, inside the 2 m
fingers every time. At 0.45x the hold swings ±15 cm before it settles (see "The sim's
speed" below), so the report lands anywhere in the ±6 cm band.

Before the 1.1 m berth (LiDAR 1.6 m, headless at ~1.0x) the same courses passed, plus
`task1_core` (Chase's Task 1, 6/6 buoys) and `check_motion` as regressions; those two
were not re-run after the change, which touched only the Task 3 tree and its comments.
`task3_ul` with a LEVEL camera fails at any of these ranges: the upper-left window is
above the CV's hull band, so the fire is never seen.

The bay 1 and bay 3 runs are `courses/task3_bay1.yaml` and `task3_bay3.yaml` (`task3`
with `green_bay` and `lit_window.bay` changed).

**So: tilt the camera up ~5 deg** (and set `cam_pitch_deg: -5.0` in both places in
crusader_params.yaml), or a fire in the upper-left window is never seen.

## Found on the way

- **The pump never fired, on the boat either.** `pump_core.MAV_CMD_DO_REPEAT_SERVO` was 211,
  which is `MAV_CMD_DO_GRIPPER`; ArduRover answered FAILED to every burst. It is 184 now,
  and `test_pump_core` checks both ids against pymavlink. This would have shown up at the
  G7 bench.
- **The drop latch starts tripped.** telemetry_bridge blocks overrides until a person
  resets it (`go.sh` does). The sim's operator script now resets it before the goal.
  Without that, the MANUAL guard band ended the run on its first tick.
- **Nav2 in open water.** At the Task 3 start the costmap sees an empty obstacle cloud,
  calls itself "not current" and refuses to plan. Task 3 courses default to `NAV_MODE=off`
  (the straight, guarded legs); `NAV_MODE=on` still asks for Nav2.
- **The boat needs `SR0_RC_CHAN 10`** (G7 already asks for it; the params dump still has
  4). At 4 Hz, SERVO_OUTPUT_RAW is stale enough that the bridge's pump watchdog sees ON
  0.3 s after a burst ends. It then latches the pump path off for the rest of the run,
  after the first burst. The sim now uses 10.
- **Sim fidelity: SITL used its own stick trims and dead zones** (1500 µs and a 30 µs
  steering dead zone; the boat has RC1_TRIM 1489 and RC1_DZ 0). Small yaw overrides then
  did nothing. `sitl_params` now carries RC1-4 TRIM/MIN/MAX/DZ, and `sim_transmitter`
  rests at the boat's trims.
- **With the Gazebo window open, the docking report never went out** (so RoboCommand never
  lit the fire). The GUI slows the sim to ~0.45x real time, SITL's ATTITUDE stream
  (SR0_EXTRA1, on the sim's clock) reached the tree at 5-6 Hz of wall time, and
  `AwaitStrafeSolution`'s "steady" check (attitude rates quiet, needs >= 10 Hz) never
  passed. Two fixes: `SR0_EXTRA1 30` in the SITL overlay (the boat already intends 30), and
  the cannon tree docks with `steady="false"`: the hold's own 2 cm corrections never pass
  "steady", and only a fixed nozzle needs it. At 0.45x the pump's return also reads
  late, so the sim gives the bridge `pump_return_timeout_s 3.0` (0.3 on the boat).
- **The hold hunted.** Inside its deadband StrafeKeep's law commands nothing, so a
  low-drag hull coasts through and gets kicked back: ±11 cm on a 9 s period in bay 3.
  `brake_mps` (SlotKeep: 0.03 m/s) brakes with the D term inside the band.

## To measure on the boat

| What | Where it goes |
|---|---|
| nozzle pivot x, y, z (body frame, hull-bottom datum) | `cannon_aim_node` `nozzle_*` |
| each servo: PWM at 0 deg, us per deg, which way + turns | `pan_*`, `tilt_*` |
| exit speed: one level throw at a known angle | `throw_range_m`, `throw_elev_deg` |
| the camera's real tilt | `cam_pitch_deg` (target_tracker + bt_runner_node) |
| the pump's Pixhawk output (SERVOn_FUNCTION = 60, RCIN10) | telemetry_bridge `pump_servo_channel` |
| `SR0_RC_CHAN 10` on the autopilot (G7) | the Pixhawk's params; without it the pump path latches off after the first burst |

## Open

- "Fully docked": at the 1.1 m berth the whole hull is in the slip (stern ~8 cm inside the
  fingers' tips). At the 1.6 m watch range the stern is ~0.4 m out. The referee judges the
  docking report where the boat is when it is sent, and counts the bow inside
  (`docked_rule bow_in`); `whole_hull` is the strict reading. If
  the judges want the boat to stay fully in the slip until the end, tilt the camera up
  12-15 deg and drop the `standoff := 1.6` line (above).
- **The sim's speed.** With the Gazebo window open the sim runs at ~0.45x real time (GPU
  bound), and the stack runs on wall-clock time, so to the controllers the boat is ~2x
  slower and its hold about half as damped. The approach into the slip overshoots (LiDAR
  down to 0.54 m, the bow ~0.35 m off the deck; no contact) and the hold swings ±15 cm
  for ~40 s before it settles. Headless (`--no-gui`) it runs at ~1.0x and holds to a few
  cm. Not tuned out: the boat runs at 1.0x, and its gains are the boat's.
- The real dock detector on Gazebo renders: the CV team's colour and geometry cores are
  not in this repo, so the sim's `DockObservation` comes from geometry. The windows ARE
  recoloured in Gazebo, so the real pipeline can be put on `oak/rgb` later.
