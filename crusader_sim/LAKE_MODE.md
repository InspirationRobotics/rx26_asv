# Lake mode: the REAL Crusader at a lake, with you playing Ekko

Sunday 2026-10-04: 10 buoys on a lake, no UAV in the air. The boat runs its real Task 1 tree; **you are the UAV**:
you tell it where the buoys are and what colour each one is, and you answer its checkpoints, from a browser. This is
the sim's Task 1 panel (`task1_panel`) with the simulator taken out: same radio handshake, same buttons, real boat.

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
| udp 14556 loopback | `panel_feed` -> the panel | |
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
     `crusader_groundstation`, and for avoidance `crusader_nav`, `crusader_nav_layers` and the **Nav2 packages**
     (`nav2_planner`, the STVL layer). **Without them the rig runs `NAV_MODE=off` (legacy straight legs) and says so.**
     The `crsd-sim:nav2` image has Nav2; the `asv` container may not. Settle that before the day.
   - `crusader_bt`/`crusader_world_model` new enough for the tree you run (`task1_disruptive.xml`, the colour-vote
     tracker, `nav_mode`, `n_gates` in `/crsd/safe_passage_report`). The rebuild is on the **host**: `tools/scripts/rebuild.sh`
   - python in `asv`: `yaml`, `pymavlink` (both are what the boat's own nodes use)
3. **`asv`: check readiness without starting anything.** Jetson host: `docker exec -it asv bash`, then in asv:

       LAKE_DATUM=1.3000000,103.8500000 bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh --check

## On the lake

**1. Pick the datum.** One `lat,lon` you can name again (a pier corner, the boat's start). It is the map's origin, the
nav datum and the panel's origin: the field, the saved course and every boat layer are metres east/north of it.
Use the same value every time you restart the rig.

**2. Start the rig.** In `asv`:

    LAKE_DATUM=<lat>,<lon> bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh

First time: the **stand test** (defaults `NAV_MODE=shadow`, `PUBLISH=false`): the tree plans and the planner is drawn, but
no setpoint is sent. For the water, restart it: `PUBLISH=1 NAV_MODE=on LAKE_DATUM=... bash .../lake_rig_up.sh`
(re-running restarts the rig cleanly; it stops only what it started, by recorded pid).

| Env | Default | |
|---|---|---|
| `LAKE_DATUM` | **required** | `lat,lon` |
| `NAV_MODE` | `shadow` | `off` legacy legs / `shadow` plan + draw, drive legacy / `on` drive the plans |
| `PUBLISH` | false | `1` lets the tree send setpoints (the pilot still has to arm + GUIDED) |
| `TREE` | `task1_disruptive.xml` | |
| `PANEL_PORT`, `LAKE_LOGDIR`, `LAKE_SRC` | 8095, `~/.cache/crusader_lake`, `/root/robotx_ws/src/rx26_asv` | logs: `<LAKE_LOGDIR>/<name>.log` |
| `LAKE_RXL` | `replace` | see below |

It starts only what `core.launch.py` does not run: `rxl_link_node` (loopback), `target_tracker`, `nav`, `bt_view`,
`ground_station` (only if none is up), `panel_feed`, `bt_runner`, `task1_panel --lake`. **`rxl_link_node`**: core runs one
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
- **PIN AT BOAT**: a buoy where the boat is now (fresh pose only). It is the boat's GPS position, not the buoy's: bring the
  boat alongside and expect ~1 m. A camera track is the better source; the boat fuses your position with its own track
  within 5 m (`assoc_radius_m`), and the UAV error requirement is < 1 m;
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

**9. START TASK 1.** Rewinds the radio, sends the field, then runs `lake_goal`: tier 2 (Disruptive, your colours),
your approach point, 600 s. `lake_goal` refuses unless the autopilot reports armed + GUIDED.

**10. Answer the checkpoints.** The boat asks after the ENTRY orbit and after each gate, and waits:

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

## Replaying the lake layout in Gazebo ("sim-real")

1. laptop: put the downloaded file in `Boat\rx26_asv\crusader_sim\courses\<name>.yaml`.
2. WSL: `NAV_MODE=on bash ~/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_sim_up.sh <name> --no-uav --no-gui`, then the sim's
   Task 1 panel (`TASK1_PANEL.cmd`), where `<name>` is now in *Load template*. (`crusader-sim` skill.)
The frame is the same: the course's metres east/north of its origin are the sim's world metres.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `PANEL UNREACHABLE` | the page cannot poll: WiFi, or the panel died (`<LAKE_LOGDIR>/panel.log`). The boat aborts ~15 s after its last field |
| no camera tracks | `oak_detector` not running (GCS Nodes tab), or `target_tracker` down (`tt.log`) |
| `feed OFFLINE` | `panel_feed` down (`panel_feed.log`), or something else holds udp 14556 |
| "panel_feed's origin is not this panel's datum" | the rig and the panel got different `LAKE_DATUM`s: restart the rig with one |
| START greyed | the reason is under the button: not committed / not armed / not GUIDED / no FCU status from the feed |
| `planner_server is not active` | Nav2 needs a pose with a finite **heading** (GPS yaw from the RTK pair; the compass is disabled): wait for the moving baseline |
| the boat never asks checkpoint 1 | it has not finished the ENTRY orbit; check `bt.log` and the tree on :8085 |
| a field was sent but the boat ignores it | `rxl_link_node` is not the loopback one: `ps -ef | grep rxl_link` in asv; `rxl.log` |
| "udp 14555/14556 is taken" | an old rig process: `lake_rig_down.sh`, then `ps -ef | grep -E 'rxl_link|panel_feed'` |
| page works but a layer is blank | a layer older than 2 s is drawn as nothing and says so under the map (blank, never the last value) |

## What was verified, and where

See the bottom of this file ("Rehearsal record").
