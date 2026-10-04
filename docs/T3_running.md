# Task 3: running the trees on the boat

How to run the Task 3 behaviour trees on Crusader: the **fixed-nozzle fire tree** (strafe onto a
window in MANUAL and shoot) and the **GUIDED approach** (find the GREEN bay and line up on it).
Written from the 2026-10-02 pool session. For what the trees *do*, see
[T3_coordinated_logistics.md](T3_coordinated_logistics.md) and the headers of the XML files in
`crusader_bt/behavior_trees/`.

All commands run from a laptop that can `ssh crusader@crusader-asv`. Inside the container the
scripts are at `/root/robotx_ws/src/rx26_asv/tools/scripts/task3/`, abbreviated `T3` below:

```bash
T3=/root/robotx_ws/src/rx26_asv/tools/scripts/task3
```

Their logs go to `~/robotx_ws/t3tools/` (outside `src/`): `bt.log` is the tree, `go.log` the
action client when detached.

## What has to be running

| what | started by | check |
|---|---|---|
| core (telemetry_bridge, ground station, LiDAR chain, rxl_link) | systemd `crsd-ros` at boot | ground station page loads |
| `dock_view`: the dock detector, owns the OAK-D | ground station **Nodes** tab, or `task3.launch.py` | `status.sh` → `dock_view: up`, fps ~12 |
| `wall_range_node`: LiDAR range to the dock face | `task3.launch.py`, or by hand (below) | `status.sh` → `wall: valid ...` |
| `bt_runner_node`: the tree | `tree.sh`, or `task3.launch.py` | `status.sh` → `bt_runner: RUNNING` |

`wall_range_node` is **not** in the boot launch and not on the Nodes tab. After any reboot,
start it again:

```bash
ssh crusader@crusader-asv "docker exec asv bash -lc 'source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash && setsid nohup ros2 run crusader_perception wall_range_node --ros-args --params-file /root/robotx_ws/install/crusader_bringup/share/crusader_bringup/config/crusader_params.yaml > /root/robotx_ws/t3tools/wall.log 2>&1 < /dev/null &'"
```

Or start all three with the launch file: `ros2 launch crusader_bringup task3.launch.py
tree:=fire_manual` (args: `tree`, `publish_setpoints`, `fire_pump`, `dock_bays`).

One look at everything:

```bash
ssh crusader@crusader-asv "docker exec asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/status.sh"
```

## The scripts

| script | does |
|---|---|
| `tree.sh SETPOINTS PUMP [TREE] [-p name:=value ...]` | (re)starts the tree in the background. Moves nothing by itself. |
| `go.sh [TIMEOUT_S] [--here] [tier=N] [-d]` | checks the mode, resets the drop latch, sends the goal: **this starts the run**. `tier=1`/`tier=2` (Advanced/Disruptive) makes the Task 3 trees read the resource request too |
| `status.sh` | tree, mode, latch, SD, wall, dock_view, at a glance |
| `watch.sh [SECONDS] [N]` | the last N lines of what the tree is doing |

`SETPOINTS` (`publish_setpoints`, Gate G1: may the tree move the boat) and `PUMP` (`fire_pump`,
Gate G7: may it squirt) are required and spelled out every time. With both `false` the tree is a
**shadow**: it logs what it would do and touches nothing.

## Taking the boat back

Any of these, at any time:

- **SC** to the other mode (out of MANUAL for the fire trees; back to MANUAL for the GUIDED trees).
  The tree ends the run and releases the boat.
- **SD up**: the autonomy-drop latch; the bridge drops every override. `go.sh` resets it, and only
  with SD **down**.
- **SB**: e-stop.

Ctrl+C on `go.sh` stops only the watching, never the tree.

## The fire tree (MANUAL, strafing)

`task3_fire_manual.xml` is the competition shot from 1.0 m. `task3_fire_manual_1p6.xml` is the
pool-tuning variant from 1.6 m with no attempt limit, used while the detector can't hold the
windows at 1.0 m.

1. **Shadow first.** Boat in MANUAL about 2 m out, roughly square, SD down.

   ```bash
   ssh crusader@crusader-asv "docker exec asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/tree.sh false false task3_fire_manual_1p6.xml"
   ssh -t crusader@crusader-asv "docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/go.sh 86400"
   ```

   Drive it by hand. Each strafe line says what it *would* command. **Signs:** window to your left
   (`window +0.20 m left`) → `lat` negative (port). Square off by more than 10° → it only turns.
2. **Hold (dry).** The same, with `tree.sh true false ...`: the tree takes the sticks, and the pump
   stays off. It lines up, waits until the hull is still, and logs `FIRE #n (DRY ...)`.
3. **Fire.** `tree.sh true true ...`, only once the dry hold is clean, and only with the nozzle
   calibrated for that range (squirt_cal).

**What the log says:**

```text
strafe: range 1.62 m, window -0.04 m left, square +1.0 deg | sticks fwd +2 lat +5 yaw +0 us (on the spot) [live: kp_lat 100 ...]
lining up: not steady: rocking            <- what is still blocking the shot
firing solution held 1.1 s
FIRE #100001 (DRY: fire_pump is off) 0.50 s
```

**The shot needs all of these** (`AwaitStrafeSolution`), held for `hold_s` 1 s:

- range within `fire_range_m ± range_tol_m` (0.06)
- window within `lat_tol_m` (0.05 m) of the nozzle line
- square within 5°
- camera fresh
- hull steady (roll/pitch rate under 4°/s; no thrust in the last moment)

**Aim offset:** `lateral_bias_m` in the tree XML. **+ moves the shot further RIGHT** (the hold keeps
the window that far left of the nozzle's line; checked in the sim, 2026-10-03). The 1.6 m variant
has +0.03, 3 cm right. On 2026-10-02 the sign was believed the other way round, so the values
tried by eye that day (-0.10, +0.10, 0) moved the shot the opposite way to what was asked, and the
"3 cm right" was entered as -0.03 (3 cm LEFT). Check +0.03 by eye before trusting it.

### Tuning the gains live

`bt_runner_node` has `strafe.*` parameters that override the tree's gains **while it runs**:

- **Change them** from the ground station **Tuning** tab (`bt_runner_node`), or with
  `ros2 param set`.
- **−1 = the tree's own value.**
- Every strafe log line ends with `[live: ...]` naming the overrides in force.
- **They last until the node restarts.** Copy a good tune into the tree file.

```bash
ssh crusader@crusader-asv "docker exec asv bash -lc 'source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash && ros2 param set /bt_runner_node strafe.kd_lat 200.0'"
```

Write the value as a float (`200.0`, not `200`). To start the tree with a tune already set, pass
`-p` to `tree.sh`:

```bash
ssh crusader@crusader-asv "docker exec asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/tree.sh true false task3_fire_manual_1p6.xml -p strafe.kp_lat:=100.0 -p strafe.kd_lat:=200.0 -p strafe.ki_lat:=10.0 -p strafe.kp_yaw:=5.0 -p strafe.kd_yaw:=0.5 -p strafe.window_median_s:=0.1 -p strafe.rate_window_s:=0.5 -p strafe.ki_fwd:=40.0 -p strafe.i_max_us:=100.0"
```

That is the tune that held best on 2026-10-02. **Tree defaults:**

- `kp_lat 90`, `kd_lat 30`, `ki_lat 30`
- `kp_yaw 4`, `kd_yaw 3`
- `kp_fwd 90`, `kd_fwd 60`, `ki_fwd 20`
- `i_max_us 80`
- camera smoothing: a 0.6 s median, and a 0.8 s rate fit

| param | unit | does |
|---|---|---|
| `strafe.kp_lat` | µs per m | push back toward the window line |
| `strafe.kd_lat` | µs per m/s | brake on the sideways slide |
| `strafe.ki_lat` | µs per m·s | build-up against a steady drift (capped by `i_max_us`) |
| `strafe.kp_fwd` / `kd_fwd` / `ki_fwd` | same, on range | |
| `strafe.kp_yaw` / `kd_yaw` | µs per deg (/s) | square up |
| `strafe.window_median_s` | s | median on the camera's window position; lower = quicker, twitchier |
| `strafe.rate_window_s` | s | fit for the sideways speed the brake acts on (needs ≥4 frames) |
| `strafe.coast_s` | s | lateral stops pushing when arrival looks under this far away (tree: 1.5; 0 = never coast) |
| `strafe.lat_min_us` | µs | smallest lateral command that isn't zero (tree: the shared `min_us`) |
| `strafe.est_enable` | 0/1 | **sideways estimator** (below); 0 = the median/rate fit above |
| `strafe.est_q` / `est_r` | m²/s³ / m | estimator: how fast the drift may change / camera noise (0.01 / 0.03) |
| `strafe.track_s` | s | estimator: how long it coasts with no camera frame before it gives up (1.0) |

**The sideways estimator (`strafe.est_enable 1`, off by default).** The camera sees the window in
the bow's frame, so every degree of yaw wobble at 1.6 m looks like ~3 cm of sideways slide, and
the brake fights it. The estimator:

- undoes the yaw at each frame's **capture time**, using the heading history (the autopilot's EKF
  yaw: gyro plus dual-antenna GPS yaw; the compass is off), and filters in the square-to-wall frame
- uses `dock_view`'s frame-in-hand stamp, so the 45–100 ms detection delay is corrected
- adds the **current** yaw back, so `lat_err` is where the nozzle points now
- gives the brake a smoothed speed with no yaw-made motion in it
- rides out camera dropouts up to `track_s`

In the unit test (1.6 m, ±2° yaw wobble at 0.5 Hz, 3 cm camera noise), the fake sideways speed
fell from 0.22 m/s RMS (bow frame) to 0.02 m/s. **Not yet tried on the water.** A/B it live with
`est_enable` 0 ↔ 1 under the same gains. The strafe log line shows `{est p v age | cam}`: the
estimate next to the raw camera value.

**What the 2026-10-02 tuning showed:**

- The hull slides at about 0.1 m/s, so the brake needs a big `kd_lat` (200–350) before it does
  anything: 30 µs per m/s × 0.1 m/s is under the ESC deadband.
- Small lateral commands (under about ±50 µs) barely move the boat, so a weak P lets a gust
  carry it 0.3–0.5 m.
- The camera smoothing windows are real lag. At ~12 fps detections, 0.1–0.25 s median is
  enough.

## The GUIDED approach (find the GREEN bay, line up)

`task3_approach_test.xml` is the mission's own find-and-line-up legs, ending at a line-up point
**4.0 m** from the face (the mission uses 3.0 m). It doesn't berth, fire or report.

**Before:**

- **`WP_RADIUS` at 0.3**: the boat is normally 2.0, which parks GUIDED 2 m short of every point.
  Set it back afterwards.
- **The dock in view** from where you start, and its **indicator readable as GREEN**. On
  2026-10-02 at 4 m in late-afternoon sun the indicator read `unknown` in every frame, and the
  tree cannot commit without GREEN. Check before sending:

  ```bash
  ssh crusader@crusader-asv "docker exec asv bash -lc 'source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash && timeout 3 ros2 topic echo /dock/observations crusader_msgs/msg/DockObservation | grep -cE \"indicator_colour: 3\"'"
  ```

  This counts GREEN indicator frames in 3 s. It should be most of ~36.

**Run:**

```bash
ssh crusader@crusader-asv "docker exec asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/tree.sh true false task3_approach_test.xml -p dock_bays:=1"
# SC to GUIDED, then:
ssh -t crusader@crusader-asv "docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/go.sh 600 --here"
```

- `dock_bays:=1` lets the tree number a single practice bay. The course has 3; `dock::layout()`
  won't number fewer than `dock_bays`.
- `--here` makes the boat's position the approach point. Without an approach point the survey
  fails at once when nothing is seen yet. With one, it looks from here first, then runs a 4 m
  ring search around it: a 4 m drive in the pool.

**Expected log:**

```text
dock: 1:G safe=1
CommitSafeBay ...
NavigateTo ... bay 1 lead      (6 m out on the centreline)
NavigateTo ... bay 1 predock   (4 m out)
```

**Open question:** where the approach point comes from at the competition. The handbook (3.3.4)
says only that the USV must *locate* the dock. No coordinates are mentioned. Ask RoboNation, or
plan to GPS-log the dock during practice.

## Task 3 in two parts (GUIDED approach, then MANUAL dock and fire)

| part | tree | mode | does |
|---|---|---|---|
| 1 | `task3_part1_approach.xml` | GUIDED | finds the GREEN bay, commits, lines up **3 m** out on its centreline, outside the fingers. Logs `committed to bay N`. |
| 2 | `task3_part2_dock_fire.xml` | MANUAL | drives in on the camera and LiDAR, docks at the **1.4 m watch range**, reports docking, waits for the fire, goes in to shoot, reports, backs out, reads and reports the request |

The mission's legs and numbers are `task3_disruptive.xml`'s; the firing spots are
`task3_fire_manual.xml`'s. Each header explains its choices. Between the parts, **you** move SC from
GUIDED to MANUAL, and you carry the bay number across.

**Run:**

```bash
ssh crusader@crusader-asv "docker exec asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/tree.sh true false task3_part1_approach.xml"
# SC to GUIDED (WP_RADIUS 0.3, the dock in view), then:
ssh -t crusader@crusader-asv "docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/go.sh 600 --here"
# it stops lined up 3 m out; note N in "committed to bay N"
ssh crusader@crusader-asv "docker exec asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/tree.sh true false task3_part2_dock_fire.xml -p task3_bay:=N -p strafe.window_median_s:=0.1 -p strafe.rate_window_s:=0.5"
# SC to MANUAL, then:
ssh -t crusader@crusader-asv "docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/go.sh 600 tier=2"
```

At the pool (one bay) add `-p dock_bays:=1` to both; `task3_bay` can then be left out, because part
2 can number one bay itself. With three bays it can't: from 3 m the outer faces are cut off at the
image edge. `true false` keeps the pump dry; `true true` only after a clean dry run.

**Why part 2 docks at 1.4 m and goes in only to shoot.** From the firing spots (1.0 / 0.8 m) the
camera cannot see the upper-left window. In the 2026-10-02 close-range frames with the camera 1.0 m
or less from the face, it showed in at most 65% of frames and was never whole, and the indicator
drops out of view too. So the boat watches from 1.4 m (stern still inside the 2.0 m fingers), goes
in to shoot, and comes back out to read the code. The hold always tracks the **lower-right**
window. The upper-left is shot 0.45 m to its left (`aim_bias` -0.45; **measure that spacing on the
dock**).

**Known gap: the upper-left shot.**
- **Today's calibration (1.0 m):** on the upper-left's line at 1.0 m, the lower-right is at the edge
  of the camera's view. In the sim the hold lost it, and no shot was taken.
- **Calibrated for 1.4 m instead:** the sim passed, with the shot 2 cm from target. Calibrate the
  upper-left shot from the watch range (a lower nozzle angle), then change `fire_range := 1.0` in
  the `upper_left` branch to that range.

**Sim checks (2026-10-03).** The sim needs `cam_pitch_deg -10` (it only reports windows wholly in
view) and a stand-in nozzle, so these test the tree's logic, not the aim.

| case | result |
|---|---|
| part 1, GREEN bay 1, 2 or 3 | committed to the right bay in ~18 s, no contacts |
| part 2, lower-right lit | docking, fire out, request and UAV all right; no contacts |
| part 2, upper-left lit, shot calibrated at 1.4 m | all right; no contacts |
| part 2, upper-left lit, shot at 1.0 m | docked, then lost the lower-right window at 1.0 m: no shot |

```bash
python tools/task3_sim/sim.py --headless --timeout 400 --fire-pump --task3-bay 2 --tree crusader_bt/behavior_trees/task3_part2_dock_fire.xml --scenario "{\"start_n\": 17.0, \"start_mode\": \"MANUAL\", \"target_window\": 1, \"cam_pitch_deg\": -10.0, \"deck_z\": 0.0, \"nozzle_elev_deg\": 65.0, \"nozzle_hit_range_m\": 0.8, \"extinguish_s\": 0.3}"
```

**For the competition** the two parts must become one run. A person switching SC or typing the bay
mid-run is an intervention. That means one tree, with the mode change made by the tree on
`/crsd/set_mode` (the bridge already accepts it; SC still outranks it). It needs a SetMode leaf and
the team's sign-off. Not built yet.

## When things go wrong (all seen on 2026-10-02)

| symptom | cause | fix |
|---|---|---|
| `go.sh: REFUSED: reset refused ... switch still in drop position (ch9=1995)` | SD up | SD down; `status.sh` shows ch9 |
| goal ABORTED at 0.0 s, `WallRangeAlive` | `wall_range_node` not running (after a reboot), or the LiDAR lost the wall (start 3+ m out and >15° off square) | start it (above); start ~2 m out, roughly square |
| `bt_runner: not running` after a while | the node got SIGTERM, a reboot or a stop elsewhere | `tree.sh` again. **Live gains are lost**: pass them with `-p`. |
| dock_view 1 fps up close, CPU pegged | fixed: OpenBLAS limited to 1 thread, colour on capped crops | if it comes back, check `/crsd/dock_view_health` `chain_ms` |
| GUIDED run: `PickVantage: no bay seen and no approach point` | no `--here`, bay not yet seen | `go.sh ... --here`, boat facing the dock |
| QGC won't connect after changing Wi-Fi | MAVProxy sends to a fixed laptop IP; the network drops the broadcast | update `--out=udp:<laptop ip>:14550` in `crsd-mavproxy` and restart it (sudo) |
