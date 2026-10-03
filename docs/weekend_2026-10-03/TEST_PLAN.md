# Crusader Test Weekend: Test weekend, Sat 3 – Sun 4 October

The published page has the shared checklist, run log and readings: https://claude.ai/artifact/D3qvajKn8JSXXH7qX4u42g. This file is the same plan in plain Markdown, for offline use and the repo. The page source is `crusader_weekend.html` next to this file.

Saturday gets the boat ready: latest code and vision engines on the Jetson, sensors and radio checked, then a camera-only Task 1 in the pool. Sunday is the lake: you play Ekko from the operator GUI and the boat runs the same whole-field planner you ran in the sim.

> **The RC SB switch is the only e-stop.** It cuts the relay in every mode. Nothing on the laptop or in the GUI is a stop, and ABORT does not stop the boat: the pilot takes over with the RC.

## Your list, item by item

What got done on the PC tonight (Fri 2 Oct), and where the rest happens. Everything is committed on branch `sim/gazebo` and pushed to GitHub (`InspirationRobotics/rx26_asv`) on Sat 3 Oct. It is not merged into `main`.

| Item | State | What it means |
|---|---|---|
| Vision model as .engine | On the boat | Script ready: `tools/scripts/export_engines.py` checks class order against the YAML, exports, smoke-tests and keeps the old engines. Verified on the laptop with the real .pt files. A .engine must be built on the Jetson itself, so the build is step S2.5. |
| Latest code on the boat | On the boat | Tonight's work is on `sim/gazebo` (code as of `2997b02`; later commits on the branch are documents only). Deploy is a git bundle into a new branch, steps S2.1–S2.4, or a fetch from GitHub if the Jetson has internet. |
| Rocket Prism AC ↔ Bullet AC | Needs you | Nothing in the repo records the radios' IPs, SSID or frequency. Block S4 records them, then measures ping, throughput and range. |
| Pool test, camera only | On the boat | `POOL=1` on the lake rig: the tracker runs camera-only, there is no Nav2 and no LiDAR in the plan, and the tight-field profile is on (3 m orbits). Rehearsed in Gazebo on a 3–5 m field: PASS in 162 s. The pool layout stays blank until you send the satellite image. |
| Basic waypoint nav by hand | Needs you | Pool block P2: GUIDED clicks, then a 4-point AUTO square in QGC. |
| All sensors reading correctly | Needs you | Block S3, using the GUI's Telemetry, LiDAR and Camera tabs. |
| Look-ahead vs WP_RADIUS stall check | Done tonight | Analysed (ArduRover 4.6.3 source + our follower). With the boat's last known `WP_RADIUS` 0.5 it will not stall. At 2.0 (the baseline file and the sim) it can stall at the end of a stopping leg. Confirm the live value in S2.6. |
| Operator GUI: Task 1 tab | On the boat | Built and tested on the PC. The GUI at `:8090` has a **Task 1** tab that shows the lake panel: capture buoys by driving, send positions and colours as the UAV, recolour for Disruptive, answer checkpoints. **Leaving the tab during a run stops the UAV heartbeat and the boat aborts about 15 s later.** That is deliberate: a hidden panel would hide the dead-man and the checkpoint prompts. To watch other tabs during a run, use the tab's *open on its own* link and keep the panel in its own window. |
| Planner tuning in the Tuning page | On the boat | Tuning tab, `/bt_runner_node`: a **Planner (applies at the next START)** group, plus Load profile and Save as profile. Before tonight a live change to these knobs did nothing until the node restarted. Now the boat re-reads them each time a goal is accepted, never mid-mission, and logs which ones changed. NaN, infinite and out-of-range values are refused. Note: a bad value in the YAML now stops the node at startup instead of being accepted silently. |
| Sim run against Sunday's GUI | Done tonight | Sunday's rig and panel were driven against Gazebo, start to finish, with setpoints on and the whole-field tree: **Disruptive** PASS (4 checkpoints incl. EXIT, 246 s), **Advanced** PASS (228 s), **recolour** at a checkpoint replanned and PASSED (247 s), **dead-man** aborted 15 s after the page stopped, **pool mode** PASS (162 s). After merging tonight's GUI and lake branches, one more Disruptive run on the final branch: PASS, 253 s, 6/6 buoys on their side, no contact. 273 offline tests pass. |
| Sim in the GUI / GUI on localhost | Later | In the sim the same GUI already runs on the laptop: the sim starts the ground station at `localhost:8090`, and its new Task 1 tab frames the sim's panel at `localhost:8095`. For the real boat the GUI stays on the Jetson for Sunday. Moving it onto the laptop needs boat-side node changes, so it waits until the S4 range walk shows the link actually needs it. See Decisions. |

*Context tags on every step say where it runs: laptop Windows PowerShell 5.1 · jetson Jetson host over SSH · asv inside the Jetson's `asv` container (`docker exec -it asv bash`) · qgc QGroundControl · gcs the operator GUI at `:8090` · yard pool lake physical work.*

## Saturday 3 Oct · get the boat ready

Order matters: code and engines first, then sensors and radio on the stand, then the yard, then water. Ticks are shared with everyone who opens this page.

### S1 · Laptop, before you leave (≈ 20 min)

- [ ] **[laptop]** Make the code bundle from the committed branch.

  *powershell:*

  ```powershell
  cd C:\Users\Chaser\Documents\dev\RobotX_2026\Boat\rx26_asv
  ```

  *powershell:*

  ```powershell
  git bundle create C:\Users\Chaser\crusader-20261003.bundle sim/gazebo
  ```

  **Pass:** the file exists; `git bundle verify` on it says okay.

- [ ] **[laptop]** Take the models: `crusader_det_yolo26n.pt` (detector, circle/diamond) and `crusader_led_cls.pt` (LED colour, blue/green/off/red) from `Boat\crusader_vision\runs\`, plus the fallback detector `det_yolo11n\weights\best.pt`.

- [ ] **[laptop]** Send Claude the pool's satellite image (where the boat launches, the pool's size, where you stand). The pool block below has no distances until then.

### S2 · Deploy on the stand (≈ 1.5 h · stop at 1 pm if stuck)

Riskiest part of the day. If the build fails, keep what is running and skip to S3: the GUI and sensors still get checked.

- [ ] **[jetson]** Record the starting state before touching anything.

  *jetson bash:*

  ```bash
  cd ~/robotx_ws/src/rx26_asv; { git branch --show-current; git rev-parse HEAD; git status --short; docker ps -a; df -h /; } > ~/state-20261003.txt 2>&1; cat ~/state-20261003.txt
  ```

  If `git status` lists changes, **stop**: commit them to a branch `boat-local-20261003` first. Never reset them.

- [ ] **[laptop]** Copy the bundle and the two models. Use `pscp`; OpenSSH password login fails on this PC (crusader-net skill).

  *powershell:*

  ```powershell
  pscp C:\Users\Chaser\crusader-20261003.bundle crusader@192.168.100.109:/home/crusader/
  ```

  *powershell:*

  ```powershell
  pscp C:\Users\Chaser\Documents\dev\RobotX_2026\Boat\crusader_vision\runs\crusader_det_yolo26n.pt C:\Users\Chaser\Documents\dev\RobotX_2026\Boat\crusader_vision\runs\crusader_led_cls.pt crusader@192.168.100.109:/home/crusader/robotx_ws/models/
  ```

- [ ] **[jetson]** Fetch into a **new** branch, check for boat-only commits, switch.

  *jetson bash:*

  ```bash
  cd ~/robotx_ws/src/rx26_asv; git fetch ~/crusader-20261003.bundle sim/gazebo:lake-20261004
  ```

  *jetson bash (with internet, instead of the bundle):*

  ```bash
  cd ~/robotx_ws/src/rx26_asv; git fetch origin sim/gazebo:lake-20261004
  ```

  *jetson bash:*

  ```bash
  git log --oneline lake-20261004..HEAD | head -20
  ```

  *jetson bash:*

  ```bash
  git switch lake-20261004
  ```

  Use the bundle fetch or the GitHub fetch, not both. The log command lists commits the boat has and the branch does not. **Empty is expected.** If not, merge them into `lake-20261004` before building. A git checkout writes LF endings here, so no CRLF strip is needed for files that came through git.

- [ ] **[jetson]** Build on the host (it fails inside asv with a misleading message), then restart core.

  *jetson bash:*

  ```bash
  cd ~/robotx_ws/src/rx26_asv; tools/scripts/rebuild.sh
  ```

  *jetson bash:*

  ```bash
  sudo systemctl restart crsd-ros; systemctl status crsd-ros --no-pager | head -5
  ```

  **Pass:** `check_config` PASS inside rebuild.sh, build without errors, `crsd-ros` active.

- [ ] **[asv]** Vision engines. Check first, then build (several minutes each). Never `pip install` in asv: protobuf is pinned at 5.29 and the detector stack breaks if it moves.

  *asv bash:*

  ```bash
  python3 /root/robotx_ws/src/rx26_asv/tools/scripts/export_engines.py --det /root/robotx_ws/models/crusader_det_yolo26n.pt --cls /root/robotx_ws/models/crusader_led_cls.pt --check-only
  ```

  *asv bash:*

  ```bash
  python3 /root/robotx_ws/src/rx26_asv/tools/scripts/export_engines.py --det /root/robotx_ws/models/crusader_det_yolo26n.pt --cls /root/robotx_ws/models/crusader_led_cls.pt
  ```

  If the YOLO26 file will not load (ultralytics in asv too old), copy `det_yolo11n\weights\best.pt` over and pass it as `--det`. Then restart `oak_detector` from the GUI's Nodes tab. **Pass:** its log says `engines ready`. The previous engines stay as `*.bak-<time>`.

- [ ] **[qgc]** Pull a fresh parameter file into `Boat\QGC params\` (named 2026-10-03…) and fill in Readings: `WP_RADIUS`, `LOIT_RADIUS`, `GUID_OPTIONS`, `WP_SPEED`, `CRUISE_SPEED`, `CRUISE_THROTTLE`.

  **Pass:** `WP_RADIUS ≤ 0.5` and `GUID_OPTIONS = 0`. Why: in GUIDED a setpoint counts as reached once the boat is within WP_RADIUS, and then a boat loiters. The whole-field follower ends a stopping leg only within 0.8 m, so a larger WP_RADIUS can leave the boat parked short of the end with nothing re-sending: a stall until the mission timeout. The last dump (7 Sep) had 0.5. The baseline file and the sim use 2.0.

- [ ] **[asv]** Lake rig preflight. It starts nothing and lists what is missing.

  *asv bash:*

  ```bash
  LAKE_DATUM=1.3000000,103.8500000 bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh --check
  ```

  **Pass:** it reports tree `task1_global.xml` with nav_mode `off`, says Nav2 is not needed, and lists nothing missing. The datum is a placeholder; `--check` only parses it. The Nav2 container rebuild in the old plan is no longer needed for Sunday.

### S3 · Sensors, in the GUI (≈ 30 min)

Open the operator GUI at `http://<jetson>:8090`. Every reading goes into the Readings section.

- [ ] **[gcs]** Telemetry tab: GPS fix type is RTK fixed, GPS yaw valid, and the heading matches where the bow points (the compass is disabled by design). HEARTBEAT about 5 Hz.

  No valid GPS yaw = no heading = the boat cannot plan. Check this before blaming the planner.

- [ ] **[gcs]** LiDAR tab: about 10 Hz, about 20k points. Stand off the **starboard** side: you must appear to starboard (the MID360 is mounted upside down; both y and z are negated).

- [ ] **[gcs]** Camera tab with the new engines: walk a RoboBuoy past the bow at 5, 10 and 15 m. Boxes on the buoy; with its beacon lit, the right colour. Note the range where detections stop.

- [ ] **[yard]** RC: the SB e-stop shows in RC_CHANNELS (ch7) and actually drops the relay in MANUAL and in GUIDED. Do the G1 bench checks (`docs/G1_bench_procedure.md`) before any run with setpoints on.

- [ ] **[yard]** Battery: measure the pack with a meter and write it next to the reported voltage. The reported number is disputed and nothing protects the pack automatically.

- [ ] **[yard]** LED strip plugged back in and showing RED / YELLOW / GREEN. Handbook 5.3.1 makes it mandatory, and it has been unplugged since 3 Sep.

### S4 · Radio: Rocket Prism AC ↔ Bullet AC (≈ 45 min)

The repo has no record of either radio's settings. What you write here decides whether the GUI needs to move to the laptop. The Jetson side of the Bullet is `192.168.8.109` (wired port `enP8p1s0`, gateway 192.168.8.1).

- [ ] **[laptop]** Both radios on the bench, within 5 m. Open each web UI and record: management IP, mode (Rocket = access point, Bullet = station), SSID, frequency and channel width, output power, firmware.

  Watch out: the airMAX factory address 192.168.1.20 sits on the boat's LiDAR subnet (192.168.1.x shares the same cable). Give the radios addresses on 192.168.8.x.

- [ ] **[laptop]** Laptop on the Rocket's LAN, 100 pings to the Jetson through the link.

  *powershell:*

  ```powershell
  ping -n 100 192.168.8.109
  ```

  **Pass:** 0 % loss and an average under 10 ms at bench range.

- [ ] **[jetson] [laptop]** Throughput. `iperf3` is not known to be installed on either side. If it is: run the server on the Jetson host, the client on the laptop. If not, time a `pscp` of the bundle back to the laptop instead.

  *jetson bash:*

  ```bash
  iperf3 -s -1
  ```

  *powershell:*

  ```powershell
  iperf3 -c 192.168.8.109 -t 10
  ```

- [ ] **[laptop]** Through the radio only (laptop WiFi off): the GUI at `http://192.168.8.109:8090` loads, and QGC gets telemetry. MAVProxy broadcasts to 192.168.8.255; if QGC stays silent, the laptop's address must be added to `GCS_IPS`.

- [ ] **[yard]** Range walk with the Bullet end (or the boat on its cart): 50, 100, 200 m in line of sight. At each: signal (dBm) on both radios, ping loss and average, and whether the GUI's Map tab still updates smoothly.

- [ ] **[yard]** Unplug the Rocket for 20 s during the S5 stand test: the GUI reconnects by itself afterwards, and the boat's mission aborts about 15 s after the last field (the dead-man). The RC stays the only stop.

### S5 · Yard, RC in hand, setpoints off (≈ 45 min)

- [ ] **[asv] [gcs]** Start the rig in stand mode and open the GUI's **Task 1** tab.

  *asv bash:*

  ```bash
  LAKE_DATUM=<lat>,<lon> bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh
  ```

  Defaults: the whole-field tree, nav_mode off, setpoints off (stand test). The panel header shows which tree and mode the rig chose and why.

- [ ] **[gcs] [yard]** Stand rehearsal: PIN AT BOAT a few fake buoys (walk the boat, or type lat/lon), COMMIT FIELD, the pilot arms and selects GUIDED, START, ACK the checkpoints, ABORT. Repeat once and close the tab: the mission must abort within about 18 s.

  Keep the Task 1 tab in front. A browser slows a hidden tab, and slow polls look like a lost operator to the dead-man.

- [ ] **[gcs]** Tuning tab, `/bt_runner_node`, **Planner** group: change `nav_orbit_radius_m` (e.g. 6 → 4), START, and read `~/.cache/crusader_lake/bt.log`. Then set it back. Also try Load profile `tight_3to5m`: it previews which keys it applies and which Nav2 keys it skips.

  **Pass:** the log shows `nav_*: 1 changed since the previous goal: nav_orbit_radius_m 6 -> 4`. A red warning in the tab means the running bt_runner is an old build: S2.4 did not take.

## Saturday afternoon · pool

Small pool, so the boat uses the camera only: no Nav2, no LiDAR in the plan, small orbits. The pool layout is still a blank: send Claude the satellite image and this block gets distances.

### P1 · Pool setup (≈ 30 min)

Distances are blank until the satellite image arrives. Size the field to the pool: gate 3–5 m wide, at least 3 m from ENTRY/EXIT to anything else.

- [ ] **[gcs] [pool]** At the pool: RTK fixed and GPS yaw valid. Walls, fences and buildings can spoil RTK, and Task 1 needs it.

- [ ] **[pool]** RC in MANUAL: each thruster pushes the right way; translate forward, back, left, right; rotate both ways. Nothing spins on its own.

### P2 · Basic waypoint navigation, by hand (≈ 30 min)

- [ ] **[qgc] [pool]** GUIDED: click-to-go to three points about 4 m apart.

  **Pass:** it drives to each and stops within WP_RADIUS; no spin; no stall short of the point.

- [ ] **[qgc] [pool]** AUTO: a 4-waypoint square sized to the pool.

  **Pass:** all four reached in order, holds after the last. Perfect in MANUAL but spinning in AUTO means steering inversion: it is fixed in `SERVOx_*`, never `RCx_REVERSED`.

- [ ] **[qgc] [pool]** Stall probe: a GUIDED target 1.5 m away (the follower's look-ahead). After it arrives, push the boat about 1 m with a pole.

  Note whether it comes back. With `LOIT_RADIUS` 2.0 it will not: after arriving, a boat only corrects once it is pushed outside that radius. That is the drift you would see at a Task 1 checkpoint hold.

### P3 · Task 1, camera only (≈ 1 h)

`POOL=1` forces nav off, starts the tracker with `use_lidar:=false`, and loads the tight-field profile unless `LAKE_TUNING` names another file. Capture the field with setpoints off, then restart with `PUBLISH=1` for the runs.

- [ ] **[asv]** Restart the rig in pool mode.

  *asv bash:*

  ```bash
  POOL=1 LAKE_DATUM=<lat>,<lon> bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh
  ```

  *asv bash:*

  ```bash
  POOL=1 PUBLISH=1 LAKE_DATUM=<lat>,<lon> bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh
  ```

  **Pass:** the banner says `POOL: camera-only, LiDAR not used for planning` and lists the profile keys it read back from the running node. No `planner_server` runs. Use the same datum for both commands.

- [ ] **[pool] [gcs]** Buoys: ENTRY, one red–green gate, EXIT (a black between if there is room). Capture each in the Task 1 tab: click its camera track, or bring the bow alongside and PIN AT BOAT. COMMIT FIELD.

- [ ] **[pool] [gcs]** Advanced run: setpoints on, the pilot arms and selects GUIDED, START with tier **Advanced**. Log it as run P-A in the run log.

- [ ] **[pool] [gcs]** Disruptive run: ACK each checkpoint; on the second run, recolour one buoy at a checkpoint and SEND CHANGES + ACK. Log P-D.

- [ ] **[asv]** After each run keep `~/.cache/crusader_lake/` (logs, layouts) and a bag from the GUI's Record tab.

### P4 · Pack for Sunday (evening)

- [ ] **[yard]** LED strip in · charged packs · RC · laptop and charger · Rocket Prism AC with mast or tripod, PoE and power · 10 buoys with anchors and lines · tape measure · kayak or safety boat · phone GPS. **Unplug the RFD900**: on lake day RXL runs on loopback.

## Sunday 4 Oct · the lake, sim to real

No drone today. You drive the boat to each buoy to capture the field, then act as the UAV: send the buoy positions and colours, answer checkpoints, recolour for Disruptive.

### U1 · Set up (first hour)

- [ ] **[lake] [laptop]** Rocket on the shore mast aimed at the field; laptop on its LAN; `ping -n 20 192.168.8.109` clean.

- [ ] **[lake]** Pick one datum you can name again (dock corner, pier). Write it in Readings. Use the same `LAKE_DATUM` every time the rig restarts.

- [ ] **[lake]** Lay the field: ENTRY, gate 1 (8–10 m wide), two blacks between, gate 2, two blacks, EXIT, about 70 m long. Later variants: swap one red for a green (unpaired: 3 green + 2 red), then a tight 3–5 m version.

### U2 · Capture the field by driving (≈ 45 min)

You are the UAV. The positions you pin are what the boat plans around, so take them carefully. The UAV error requirement is under 1 m.

- [ ] **[asv] [gcs]** Rig up in stand mode (setpoints off), start `oak_detector` from the Nodes tab, open the **Task 1** tab.

  *asv bash:*

  ```bash
  LAKE_DATUM=<lat>,<lon> bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh
  ```

- [ ] **[lake] [gcs]** Drive in MANUAL to each buoy, nose up to it, and press PIN AT BOAT. Set its role and colour. The pin is the boat's position averaged over 2 s, so hold still a moment. It refuses a stale pose, fewer than 4 samples, or more than 1.5 m of movement. Set *bow offset* to the distance from the boat's reference point to the bow (measured Saturday) and the pin lands at the bow; that needs a heading less than 1 s old. Cross-check against the camera track if one shows.

  **Pass:** each pin within about 1 m of where the buoy really is (the camera track agrees, or you measured).

- [ ] **[gcs]** COMMIT FIELD, then **SAVE AS COURSE** and download the YAML.

### U3 · Sim to real (≈ 30 min, if the sim PC is with you)

- [ ] **[laptop]** Put the YAML in `Boat\rx26_asv\crusader_sim\courses\`, load it in the sim's Task 1 panel and run it Advanced, then Disruptive. If the sim fails on your real layout, fix that before the water. Without the sim PC, do this Sunday night and note it.

### U4 · Runs (rest of the day)

Go to the run log. Before each run the pilot confirms SB is in reach. After each run keep the logs and a bag.

- [ ] **[asv]** For runs with setpoints, restart the rig with PUBLISH on.

  *asv bash:*

  ```bash
  PUBLISH=1 LAKE_DATUM=<lat>,<lon> bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh
  ```

  Choose the tier at START: **Disruptive** (default, checkpoints) or **Advanced** (no checkpoints). **In Advanced the dead-man does not apply:** closing the page or losing the radio does not end the mission. The pilot ends it with the RC.

- [ ] **[asv]** After every run: copy `~/.cache/crusader_lake/`, take the bag from the Record tab, write the result here.

## Run log

In order. Do not start the next run until the previous one is clean. Record the result and a one-line note; the bag and `~/.cache/crusader_lake/` logs carry the rest.

### P-A · pool · camera only · Advanced

ENTRY orbit, the gate, EXIT orbit.

**Pass when:** Full clockwise orbit round ENTRY, gate crossed with red to starboard, full counter-clockwise orbit round EXIT, no contact.

Result: ____  Note: ________________________

### P-D · pool · camera only · Disruptive

Same field, ACK each checkpoint; second try with a recolour.

**Pass when:** It asks after the ENTRY orbit, after the gate and before EXIT, and replans after the recolour.

Result: ____  Note: ________________________

### R0 · lake · stand mode, setpoints off · MANUAL

START, then the pilot drives while you watch the plan in the Task 1 tab.

**Pass when:** The drawn path keeps every red to starboard and every green to port, and goes round the blacks.

Result: ____  Note: ________________________

### R1 · lake · PUBLISH=1 · Advanced

START; ABORT right after the ENTRY orbit.

**Pass when:** A full clockwise orbit; after ABORT the banner says to take over, and the pilot does.

Result: ____  Note: ________________________

### R2 · lake · PUBLISH=1 · Advanced

Full run.

**Pass when:** Both orbits in the right direction, every gate on the right side, no contact, no stall at any stop.

Result: ____  Note: ________________________

### R3 · lake · PUBLISH=1 · Disruptive

Full run, ACK every checkpoint by hand.

**Pass when:** Asks after the ENTRY orbit, after each gate and once before EXIT; continues on each ACK.

Result: ____  Note: ________________________

### R4 · lake · PUBLISH=1 · Disruptive

At a gate checkpoint, recolour a buoy, then SEND CHANGES + ACK.

**Pass when:** The boat replans to the new colours and passes the recoloured buoy on its new side.

Result: ____  Note: ________________________

### R5 · lake · unpaired field (3 green, 2 red)

Disruptive or Advanced.

**Pass when:** Every red to starboard and every green to port, single buoys included.

Result: ____  Note: ________________________

### R6 · lake · tight 3–5 m field · tuning profile

Only if R2–R5 are clean.

**Pass when:** No contact; orbits still complete.

Result: ____  Note: ________________________

### RF · fallback · TREE=task1_disruptive.xml NAV_MODE=off

Only if the whole-field tree misbehaves.

**Pass when:** The older per-gate tree's guarded straight legs complete.

Result: ____  Note: ________________________

## Readings to take

Type what you measure. A field left empty stays empty: a blank is a fact, a remembered number is a guess. Each reading shows who entered it and how long ago.

| Reading | Hint | Value | Who / when |
|---|---|---|---|
| WP_RADIUS (m) | must be ≤ 0.5 |  |  |
| LOIT_RADIUS (m) | 7 Sep dump: 2.0 |  |  |
| GUID_OPTIONS | must be 0 |  |  |
| WP_SPEED (m/s) |  |  |  |
| CRUISE_SPEED / CRUISE_THROTTLE | 7 Sep dump: 0.1 / 1 (baseline 2.0 / 50) |  |  |
| Branch and HEAD on the boat after deploy |  |  |  |
| Engines built (detector, classifier) | which .pt, FP16, time to build |  |  |
| L4T / TensorRT / ultralytics in asv | printed by export_engines.py |  |  |
| GPS fix and yaw (yard) | e.g. RTK fixed, yaw valid |  |  |
| GPS fix and yaw (pool) |  |  |  |
| LiDAR rate and starboard check |  |  |  |
| Detector range on a RoboBuoy (m) | where boxes stop |  |  |
| Pack: meter vs reported (V) |  |  |  |
| Radio IPs, SSID, frequency, width | Rocket / Bullet |  |  |
| Bench: ping loss, avg RTT, throughput |  |  |  |
| 50 m: dBm both ends, ping |  |  |  |
| 100 m: dBm both ends, ping |  |  |  |
| 200 m: dBm both ends, ping |  |  |  |
| Pool size and buoy spacing used |  |  |  |
| Bow offset: reference point to bow (m) | for PIN AT BOAT |  |  |
| Lake datum (lat, lon) and what it is |  |  |  |

## Decisions for the team

Things tonight's work found that need a person to decide. None of them was changed on the boat.

- **WP_RADIUS in the baseline.** The boat had 0.5 on 7 Sep; `params/working_crusader.params` and the sim say 2.0, and `check_config.py` enforces `nav_wp_radius_m` = 2.0. Rule from tonight's analysis: WP_RADIUS must stay below min(stop tolerance 0.8, orbit tolerance 0.8, look-ahead minus re-send 1.2). Proposal: set the baseline and `nav_wp_radius_m` to 0.5 so the sim matches the boat.
- **LOIT_RADIUS 2.0.** After every arrival and every hold the boat may drift up to 2 m with no correction, including toward a buoy at a checkpoint. Proposal: 0.75–1.0, tried in the pool first.
- **CRUISE_SPEED 0.1 / CRUISE_THROTTLE 1** on the boat (7 Sep) against 2.0 / 50 in the baseline. That leaves almost no throttle feed-forward, so the boat lags its target. Find out who set it and why before changing it.
- **Follower stall guard (code, not changed).** The whole-field follower re-sends a setpoint only when the target moves 0.3 m; nothing re-sends on time or watches for no progress. Proposal: in `global_leaves.cpp`, forget the last sent point if 1 s has passed, and log a warning when progress stalls about 5 s. It is a node change, so it waits for your yes.
- **Merging `sim/gazebo` into `main`.** The branch holds the sim, the whole-field planner, lake mode and tonight's GUI work. It was pushed to GitHub on 3 Oct but not merged. Merging is the team's call, ideally after Sunday shows it works on the water.
- **GUI on the laptop for the real boat (not built).** The lake panel needs no ROS except START/ABORT, so it could run on the laptop: the boat would send its feed to the laptop and accept the UAV radio messages over the link, the way a real Ekko would talk to it. That means changing `panel_feed` and adding a small goal relay on the boat. Worth it only if S4 shows the GUI struggling at range.
- **Advanced has no dead-man.** The Advanced tree has no plan-freshness check, so a lost page or link does not end the run. Fine for Sunday because the RC is the safety path. Decide whether competition Advanced should get one.
- **Two cosmetic boat bugs (not fixed).** The goal feedback says TRANSIT from the start (`bt_runner_node.cpp:1349`), and the result line says *passed correctly 0* with the whole-field tree (`bt_runner_node.cpp:1373`). Ignore both on Sunday; trust the run log.
- **Real Ekko's radio protocol.** The team's RXL link is still not what Ekko speaks (TUNNEL 32772/32773). Sunday is unaffected because you play the UAV. It blocks competition.

---

Source: `Boat/rx26_asv` branch `sim/gazebo`; lake operator guide `crusader_sim/LAKE_MODE.md`; handbook 3.3.2 (ENTRY clockwise, EXIT counter-clockwise) and 5.3.1 (kill switch, visual feedback). Written Fri 2 Oct 2026, night.
