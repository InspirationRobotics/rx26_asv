# Pool test, backyard pool (~15 × 6 m), Sat 3 Oct

Replaces blocks P1–P3 of `TEST_PLAN.md` for this pool. Written Sat afternoon from a bench session on the
real boat (branch `lake-20261004` = `sim/gazebo` 43f2bfa, built and running).

## Why the plan's pool block does not fit here

- The plan's P3 runs a full autonomous Task 1 with the tight profile: **3 m orbits**. 2 m was tried in the sim
  and the hull touched the ENTRY buoy, so 3 m is the floor (`crusader_sim/config/tuning_profiles/tight_3to5m.yaml`).
- The boat's footprint radius is 0.82 m (`crusader_nav/config/nav2_params.yaml:38`). A 3 m orbit needs about
  2 × (3.0 + 0.82) ≈ **7.6 m of width**. This pool has 6 m at best, so the orbit hits a wall.
- LiDAR in a walled pool sees walls everywhere, so anything LiDAR-based would be noise. The plan already handles
  that with `POOL=1` (camera only), and this test keeps that.

**So in the pool:** test what has never been proven on water (the boat has barely been driven), plus the
whole Task 1 GUI flow with setpoints OFF. Autonomous Task 1 runs wait for the lake.

## Layout

Plan for **13 × 5 m of usable water** (the pool is smaller than 15 × 6 in practice). Measure it with a tape and
write it in Readings.

- **Start wall** = the end you launch from. **Centreline** = the long axis, midway between the side walls.
- Every autonomous point stays **≥ 2 m from the end walls and ≥ 1.5 m from the side walls**
  (0.82 m hull radius + slack for GPS error). If the water is narrower than 5 m, drop every point that is off
  the centreline.

```
 start wall                                                      far wall
 |  0     2     3        5           7           9          11    13 m |
 |        .     A                    B                       C         |   centreline (y = 0)
 |              E(ntry)         R  (gate)  G?                X(it)     |
 |  side walls at y = ±2.5 m (±3 m if the pool really is 6 m wide)     |
```

Points (x along the centreline from the start wall, y to the left facing the far wall):

| Point | x (m) | y (m) | Used in |
|---|---|---|---|
| A | 3 | 0 | P2 |
| B | 7 | 0 | P2 |
| C | 11 | 0 | P2 |
| Rectangle | (3, −1) → (11, −1) → (11, +1) → (3, +1) | | P2 AUTO |
| ENTRY buoy | 3 | 0 | P4 |
| Gate: RED / GREEN | 7, −1.5 / 7, +1.5 | (3 m gate; red on the RIGHT going toward the far wall) | P4, P5 |
| EXIT buoy | 11 | 0 | P4 |

Heading toward the far wall, starboard is on your right: **RED on the right, GREEN on the left**
(handbook 3.3.2). On a 5 m-wide pool the gate buoys are 1 m from the side walls; the boat threading the middle
has about 0.4 m each side (1.5 − 0.82 − 0.3 buoy radius). If that is too tight, open the gate to the full width
and accept that the buoys sit at the walls.

## P0 · Safety, before the boat touches the water (10 min)

- [ ] RC on, SB e-stop tested: drops the relay in MANUAL **and** in GUIDED (`docs/G1_bench_procedure.md`).
  The RC is the only stop; nothing in the GUI stops the boat, and ABORT only cancels the mission.
- [ ] A pole to fend the boat off the walls. `WP_SPEED` is 0.5 m/s on the boat (read today), which is
  pool-friendly; leave it.
- [ ] LED strip plugged in and showing RED / YELLOW / GREEN (handbook 5.3.1).
- [ ] Battery: meter reading written down. **The autopilot reports no voltage at all: `BATT_MONITOR = 0`**
  (read today). Nothing on the boat watches the pack; swap on a timer.

## P1 · GPS and MANUAL (20 min)

- [ ] **[gcs]** Telemetry tab: **RTK fixed** and **GPS yaw valid**, heading matches where the bow points.
  On the garage floor today it read a plain 3D fix (h_acc 1.9 m), `GPS_RAW_INT.yaw = 65535` (invalid) and the
  EKF in constant-position mode. That is normal under a roof, but it is also exactly what the 2026-09-08 UM982
  baseline fault looked like. **If yaw is still invalid outdoors after 5 minutes of open sky, stop: do P1
  MANUAL and P3 camera only, and no GUIDED/AUTO.** Without yaw the boat cannot navigate.
  The moving-baseline offsets on the FC (0.94, 0.50 → 1.065 m) agree with the measured 1.07 m baseline.
- [ ] **[pool]** RC in MANUAL: each thruster pushes the right way; translate forward, back, left, right; rotate
  both ways; nothing spins on its own.
- [ ] **[pool]** The RC mode switch (ch8) gives **MANUAL / HOLD / GUIDED only**. AUTO has to be set from QGC.

## P2 · Waypoints: GUIDED and AUTO (30 min, setpoints from QGC only)

- [ ] **[qgc]** GUIDED click-to-go A → B → C → A (4 m hops on the centreline).
  **Pass:** stops at each within ~0.5 m (`WP_RADIUS` is 0.3 on the boat), no spin, no stall short of a point,
  never closer than 1 m to a wall.
- [ ] **[qgc]** GUIDED to a point 1 m left of B, then 1 m right of B. The hull is holonomic, so it should crab
  sideways with the bow held, not pirouette.
- [ ] **[qgc]** AUTO: the 8 × 2 m rectangle, started from QGC. **Pass:** all four corners in order, holds after
  the last. Fine in MANUAL but spinning in AUTO means steering inversion, fixed in `SERVOx_*`, never `RCx_REVERSED`.
- [ ] **[qgc]** Stall probe (from the plan): GUIDED target 1.5 m away; once it arrives push it ~1 m with the pole.
  With `LOIT_RADIUS` 2.0 (read today) it will not come back until pushed past 2 m: that is the drift you would get
  at a Task 1 checkpoint hold. Write down what you see.

## P3 · Camera on real buoys (20 min)

- [ ] **[gcs]** Nodes tab: start `oak_detector`. Its log should say `engines ready` (new YOLO26 detector and LED
  classifier engines, built on the Jetson today).
- [ ] **[pool]** One buoy at 3, 6 and 10 m ahead of the bow (pool length permitting), beacon lit red, then green.
  **Pass:** boxes on the buoy, right colour; a camera track appears in the Task 1 tab. Note false boxes on walls,
  the house or reflections. Write the max range in Readings.

## P4 · Task 1 GUI flow with setpoints OFF (30 min)

Exercises everything on the real boat except the autopilot following the plan: perception, capture, commit,
the drawn plan, checkpoints UI, ABORT, the dead-man, the tuning path.

- [ ] **[gcs]** Task 1 tab, **rig** strip: tick **pool (camera only)**, leave **setpoints ON** off, press
  **START RIG** (about 40 s). The datum is the boat's position, so start it with the boat where you
  want the map origin. No SSH needed. By hand, the old way still works:
  `POOL=1 LAKE_DATUM=<lat>,<lon> bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh` in `asv`.

  **Pass:** banner `POOL: camera-only, LiDAR not used for planning`; no `planner_server`. If the preflight says
  "no /crsd/fcu_status within 8 s", run it again before believing it (it did that once right after a boot today
  and passed on the retry).
- [ ] **[pool] [gcs]** Buoys at ENTRY, gate, EXIT as in the layout table. Capture each in the Task 1 tab (camera
  track, or bow alongside + PIN AT BOAT with the bow offset set). COMMIT FIELD, then SAVE AS COURSE.
- [ ] **[gcs]** Pilot arms and selects **GUIDED** (with setpoints off the autopilot just holds where it is).
  START, tier **Disruptive**. **Pass:** the drawn path keeps RED to starboard and GREEN to port, orbits ENTRY
  clockwise and EXIT counter-clockwise. (The drawn orbits will be wider than the pool. Nothing is sent, so that
  is fine, and it is the reason the autonomous run waits for the lake.)
- [ ] **[gcs]** ABORT. **Pass:** banner says the boat is still armed and the pilot takes over.
- [ ] **[gcs]** START again, then close the Task 1 tab. **Pass:** the mission aborts within ~18 s (dead-man).
- [ ] **[gcs]** Tuning tab, `/bt_runner_node`, Planner group: change `nav_orbit_radius_m`, START, read
  `~/.cache/crusader_lake/bt.log`. **Pass:** `nav_*: 1 changed since the previous goal: ...`. Set it back.

## P5 · Optional: thread the gate by GUIDED waypoints (15 min)

Only if P2 was clean, RTK is fixed, and the pilot is happy. GUIDED click-to-go (x = 4, y = 0) → (x = 10, y = 0)
through the real gate, both directions. This is the same autopilot path the tree uses, through a real gate.
**Pass:** passes between the buoys without touching, both ways.

## Not in the pool, and why

- `PUBLISH=1` Task 1 runs (P-A, P-D in the plan): the orbits do not fit (see the top).
- Anything with LiDAR or Nav2: walls everywhere; and Sunday's tree (`task1_global.xml`) never uses Nav2.

## Corrections to TEST_PLAN.md found on the bench

- **R0 (lake, setpoints off, "the pilot drives in MANUAL while you watch the plan") cannot work as written.**
  `task1_global.xml:52,79` has `IsAutonomous`, so the mission ends the moment the pilot leaves GUIDED
  (`bt_runner_node.cpp:1362`), and START is refused unless armed + GUIDED (`lake_panel.py:222-225`). The
  whole-field plan is drawn at START, so R0 becomes: START in GUIDED with setpoints off, read the plan, ABORT.
- **S5 stand rehearsal "ACK the checkpoints":** with setpoints off the boat never reaches the ENTRY orbit, so no
  checkpoint will be asked on the stand. START, plan, ABORT and the dead-man are what S5 can test.
- **AUTO square (P2):** start it from QGC; the RC mode switch has no AUTO.
- **S2.4 "check_config PASS inside rebuild.sh":** this `rebuild.sh` no longer runs it. Run
  `python3 tools/scripts/check_config.py` inside `asv` separately (PASS today).
- **S2.4 restart:** `sudo systemctl restart crsd-ros` needs the password. Without it, the GUI's power control
  (crsd-power socket, `reboot`) restarts everything cleanly; that is what was used today.

## Bench readings taken today (garage floor, boat disarmed, RC off)

| Reading | Value |
|---|---|
| Branch / HEAD on the boat | `lake-20261004` @ 43f2bfa; previous state kept as branch `boat-local-20261003` |
| Build | `rebuild.sh` PASS, 13 packages, 3 min; `check_config` PASS |
| WP_RADIUS / LOIT_RADIUS / GUID_OPTIONS | 0.3 / 2.0 / 0 (pass) |
| WP_SPEED / CRUISE_SPEED / CRUISE_THROTTLE | 0.5 / 2.0 / 15 |
| BATT_MONITOR | 0 (no voltage reported) |
| param_guard vs baseline | no protected drift; 65 tunable diffs (baseline is stale) |
| HEARTBEAT | 5 Hz, sysid 2 |
| Accelerometer | z = −999 mg at rest (≈ 1 g; the old 85 %-of-g fault is gone) |
| GPS (garage) | 3D fix, 28 sats, h_acc 1.9 m, yaw invalid — recheck outdoors |
| Lake rig `--check` | PASS: task1_global.xml, nav_mode off, Nav2 not needed |
| Engines | `export_engines.py --check-only` OK (TensorRT 10.7, ultralytics 8.4.96); build started |
| Param dump | `Boat/QGC params/2026-10-03_crusader_lake20261004.params` (899/899) |

## Nav2 status (tested Sat evening)

Sunday's tree (`task1_global.xml`) never calls Nav2. Nav2 was tested anyway, at the user's request.

**On the boat (garage floor, disarmed), side by side.** The live `asv` container and its `install/` were not
changed. That was checked inside the container: `bt_runner` still resolves to `build/`, links no Nav2 library,
and asv has no costmap plugin library.
- Image `asv:nav2-20261003`, built from `setup/asv_add_nav2.Dockerfile` on `asv:bt-20260928`: "nav2 layer ok",
  smoke test OK, +0.4 GB.
- Container `asv_nav2`: same mounts and host network as `asv`. It is STOPPED and kept.
- The Nav2 build lives in its own workspace folders (`build_nav2/`, `install_nav2/`, `log_nav2/`, each with a
  `COLCON_IGNORE`). It passed: 5 packages, 3.5 min.
- `nav.launch.py`, with `install/` sourced first and `install_nav2/` on top: `planner_server` was active about
  2 s after configure. All three costmap layers loaded (team hazard layer, STVL, inflation).
  `/crsd/nav/obstacle_cloud` ran at 9.9 Hz, and the costmap filled from the live LiDAR.
- **The planner plans only while `bt_runner` publishes `/crsd/nav/hazards`** (frame `map`). Until then the
  hazard layer is "not current" and `planner_server` waits indefinitely. That is the design, not a fault, but
  a bench test needs a stand-in publisher.
- With an empty hazard list standing in: paths between clear points SUCCEEDED (30 m, 301 poses, 1–2 ms). From
  the boat's own position the planner ABORTED ("Starting point in lethal space"), because of garage clutter
  0.8–1.2 m from the boat. The cloud already drops a 0.5 m sphere round the sensor and everything behind the
  180° forward view, so this is very likely not the hull. **If Nav2 is ever used on the water, first confirm in
  open water that the boat's own cell is free.**

**In the laptop sim (Gazebo, per-gate tree `task1_disruptive.xml`):**

| Run | Result |
|---|---|
| Obstacles course, Nav2 on (task1_avoid) | PASS: 4/4 buoys, 2/2 gates, 1.20 m clear of the black buoy on the leg line |
| Boxed-in start (S8), Nav2 on | Behaves per its spec: blocked → costmap cleared at 5 s → leg failed at 15 s → clean failure, no contact |
| Basic course (S1) / black buoy at entry (S4), Nav2 on | Completed, 3/3 gates, both orbits correct, no contact. The per-buoy referee scores 4/6: after a gate the per-gate tree loops back past a gate buoy on the wrong side (~2.9 m) |
| Plan-only mode (S10, "shadow") | Timed out after 900 s: stuck in the legacy (non-Nav2) leg after the ENTRY orbit |

Verdict: the Nav2 machinery works on the Jetson and in the sim. The per-gate tree that uses it breaks the
per-buoy side rule; the whole-field tree exists to fix that. Keep Sunday on `task1_global.xml`.

## Camera and GUI checks on the boat (Sat evening)

**oak_detector with the new engines: PASS.**
- Started through the GUI's own `/node/start`. Its log: `engines ready — shapes: circle, diamond | colours: blue, green, off, red`.
  These are the new YOLO26 detector and LED classifier, built on the Jetson today (detector 475 s, classifier 269 s).
  The old engines are kept as `*.bak-20261003-214045`.
- OAK-D on USB 3 at 1920×1200. `/crsd/oak/detections` publishes at 15.0 Hz, and the :8080 stream serves.
- **0 false detections in 151 frames** with no buoys in view (garage).

**Lake rig in pool mode (POOL=1, setpoints off) and the GUI: PASS after one fix.**
- The rig banner says camera-only, `task1_global.xml`, nav_mode off. The tight profile is applied: 19/19 keys
  read back from the running `bt_runner`, and the Nav2-only keys are correctly skipped. `target_tracker`
  confirms `use_lidar is FALSE`.
- GUI on :8090: the Task 1 tab frames :8095. The camera source is oak_detector. The Tuning tab lists 58 `nav_*`
  keys from `/bt_runner_node` with the profile's values (orbit 3.0, lookahead 3.0/2.5), and profile
  `tight_3to5m` is listed.
- **Found and fixed: `panel_feed` crashed on the boat** (`No module named 'map_msgs'`: the asv image has no
  Nav2). The panel then showed the feed and FCU offline, with no boat pose, so PIN AT BOAT and START were impossible.
  It was fixed two ways, with Chase's OK:
  - Code: commit `1059ccb` makes the import optional. It is on `sim/gazebo` (laptop, not pushed) and on the
    boat's `lake-20261004`.
  - Container: `ros-humble-map-msgs` apt-installed in `asv` (1 package, nothing upgraded or removed). This is
    logged in `~/asv_container_changes.txt` on the Jetson. **It is not in any Dockerfile yet**, so it is lost if
    `asv` is recreated. The code fix covers that case.

  Both paths were verified on the boat: feed up, 0 bad packets, FCU fresh, boat pose 0.05–0.15 s old, PIN ready
  (8 samples).
- **COMMIT FIELD end to end:**
  1. Four typed buoys landed at the intended ENU spots.
  2. The radio sent the field, and the dead-man was satisfied by a 1 s poller.
  3. The boat logged `rxl_link_node: plan v1: 4 buoys, entry ..., exit ...`.
  4. START's block changed to "the autopilot is NOT ARMED: the pilot arms it with the RC", and START was
     refused with that message.
- **Known false alarm:** `lake_rig_up.sh`'s preflight said "no /crsd/fcu_status within 8 s … START will stay
  disabled" in 3 of 5 starts, mostly right after `lake_rig_down.sh`, while the FCU status was fresh every time.
  The 8 s `--no-daemon` probe is too short on a busy Jetson. **If the panel shows FCU "fresh", ignore the line.**
- **Harmless:** the rig also starts a second `ground_station`. It fails with "Address already in use" because
  core's own instance already serves :8090 with the new build. The GUI is unaffected.

## START RIG from the GUI (added Sat night)

The Task 1 tab has a **rig** strip: **pool (camera only)**, **setpoints ON** and **new datum here** switches,
**START RIG / RESTART RIG / STOP RIG**, the datum the next start will use and why, and the rig script's own
output under *rig output*. The ground station runs the same `lake_rig_up.sh` / `lake_rig_down.sh` itself.

- **Datum:** the boat's position, **except** when a field in progress is within 1 km. Then that field's
  datum is kept, because the panel restores the field only for the same datum. So capture with
  setpoints off, then RESTART RIG with setpoints ON, and the buoys you pinned come back. Tick
  **new datum here** for a new site that happens to be close. With no fresh boat pose (GPS yaw), START RIG
  is refused.
- **Safety:** START RIG starts the rig, not a mission. START in the panel still needs the pilot to arm and
  select GUIDED. START/STOP RIG are refused while the autopilot is armed in GUIDED or AUTO, and setpoints ON
  asks you to confirm.
- **Tested on the boat:** START (pool) up in ~38 s with the datum from the boat. A second press while busy
  is refused. A pinned buoy turns the preview to "the field in progress, N m from the boat". RESTART with
  setpoints ON is up in ~35 s, keeps the datum, restores the pinned buoys, and its banner says PUBLISH=true.
  STOP brings everything down in about 1 s.
- **Two pre-existing rig-script bugs fixed on the way:**
  - `running()` never detected anything, which made the rig start a second ground_station.
  - A POOL rig could not be restarted (exit 3: "use_lidar is not false (no answer)"). The rig's own previous
    tracker reached the POOL check first, and its parameter query raced discovery. The second command of
    TEST_PLAN.md P3 would have hit this by hand too.
- The rig's logs and layouts (`~/.cache/crusader_lake/`) live **inside** the `asv` container, not on the
  host. Copy them out with `docker cp asv:/root/.cache/crusader_lake <dest>` after each run, as the plan
  asks.
