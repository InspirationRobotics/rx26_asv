# Crusader Task 1: Saturday prep and Sunday lake test (3–4 Oct 2026)

> **Superseded on Fri 2 Oct, night.** The current plan is the published page *Crusader Test Weekend* (claude.ai
> artifacts), with a shared checklist, run log and readings. What changed since this file was written: the lake rig
> now defaults to the whole-field tree `task1_global.xml` with `nav_mode off`, so **the asv container does NOT need
> Nav2** (step 6 below is no longer required for Sunday); START has an Advanced/Disruptive select; `POOL=1` runs
> camera-only for the Saturday pool; the operator GUI (`:8090`) has a Task 1 tab and planner knobs in Tuning that apply
> at the next START; vision engines are built with `tools/scripts/export_engines.py`. Keep `WP_RADIUS <= 0.5` on the
> boat (stall analysis in the page). This file is kept as the record of the overnight state.


Written overnight on 2 Oct, from the state of local branch `sim/gazebo`, which has **not been pushed**.
Each step names where it runs: **[laptop]** (Windows, PowerShell 5.1), **[WSL]**, **[Jetson]** (host, over SSH),
**[asv]** (inside the Jetson's container), or **[yard]** / **[lake]** (physical work with the boat).
The operator guide for the lake panel is [crusader_sim/LAKE_MODE.md](../../crusader_sim/LAKE_MODE.md). This plan says
*when* to do things; LAKE_MODE.md says *how*.

## What the overnight work gives you

| Piece | State | Evidence |
|---|---|---|
| Nav2 + STVL avoidance | STVL really marks the LiDAR now. The per-source `obstacle_range` defaulted to 2.5 m and silently dropped every return | sim: LiDAR-only platform avoided (1.35–1.49 m clearance, 0.27 m contact without the fix); `c4_S1_on` PASS |
| Colour-vote tracker | Every RoboBuoy detection (`diamond`, `flash_red_diamond`, `off_diamond`… and the old `red_buoy` names) joins **one** track. Colour = vote: ≥5 lit votes and ≥60 % of lit votes, else `unknown_diamond`; the UAV field supplies the colour | 32 unit tests |
| Unpaired fields (e.g. 3 green + 2 red) | Each red/green gets a "side fence" in the costmap. Pairs form only when they can be crossed forwards. The referee scores every buoy's side | `task1_unpaired`: PASS ×3 (5/5 sides); same course with no fences: FAIL 2/5 |
| EXIT confirmation | The boat asks after the entry orbit, after each gate, and once more before the exit orbit; the panel labels it "EXIT gate - confirm exit" | sim logs |
| Sim panel | Truth vs UAV-sent map (<1 m error), boat's-map canvas (tracks with vote outline / UAV fill, fused passage, Nav2 path, costmap + LiDAR voxels), sensor views, default course `task1_avoid` (4 obstacles between the gates) | live panel-flow run: see "Still open" |
| Real YOLO in the sim | `--detector yolo` runs `crusader_det_yolo26n.pt` and `crusader_led_cls.pt` on Gazebo frames | PASS on the UAV plan, but recall is only 0.23 (3 % at 10–20 m) and there are no colours: the domain gap is large, so this is not a measure of lake performance |
| Lake mode | `task1_panel --lake` on the Jetson; you play the UAV over loopback RXL; START is gated on ARMED + GUIDED; never arms or changes mode; dead-man; SAVE AS COURSE for sim-real | 104 offline tests, real rclpy/rxl integration 26/26. **Gazebo rehearsal not yet run** |

## Saturday: get the boat ready

### Morning, at the laptop (about 1.5 h)

1. **[laptop]** Read `git log --oneline 5db4241^..sim/gazebo` (the overnight commits). Decide whether to push
   `sim/gazebo` or bundle it. Nothing has been pushed; that is the team's call.
2. **[WSL]** Run the lake rehearsal in Gazebo if I have not already (about 10 min each):
   `bash crusader_sim/test/lake_rehearsal/reh_run.sh sat_pass pass`, then `... sat_abort abort`, then `... sat_dm deadman`.
   Pass means: the referee PASSes the first; ABORT leaves the boat holding; with the browser closed the boat aborts
   within about 18 s.
3. **[laptop]** Optional: double-click the desktop shortcut "Crusader Task 1 panel". Load `task1_avoid`, then
   `task1_unpaired`, and watch one run of each. This is the clearest demo of fences and avoidance.

### Midday, at the boat on its stand: deploy (about 3 h; the riskiest part)

4. **[Jetson]** Record the starting state: branch, HEAD, `git status`, `docker inspect asv`, `df -h`, and whether apt
   reaches the internet. Save it to a file.
5. **[laptop → Jetson]** Get the code over: `git bundle create sim.bundle sim/gazebo`, copy it, fetch it into a **new**
   branch `lake-20261004`. Strip CRLF from the scripts (`crusader-deploy` skill).
6. **[Jetson]** Recreate `asv` with Nav2 + STVL following [setup/README.md](../../setup/README.md) "Recreating the asv
   container". **Rename the old container; do not remove it.** Then run `tools/scripts/rebuild.sh` on the host, and
   `sudo systemctl restart crsd-ros`.
   - **Time-box: 3 pm.** If the image will not build (ROS apt key, disk, network), stop and keep the old `asv`.
     Lake mode then runs `NAV_MODE=off` (guarded straight legs) and says so in a banner. You can still test the
     whole UAV/checkpoint flow, without planned avoidance.
7. **[asv]** Run `LAKE_DATUM=<any lat,lon> bash .../crusader_sim/scripts/lake_rig_up.sh --check`. It lists everything
   missing: Nav2 packages, a tree new enough for `n_gates`, pymavlink, yaml. Also run `tools/scripts/check_config.py`,
   which must PASS.
8. **[asv]** Sensor rates: `/livox/lidar` 10 Hz, `/crsd/nav/obstacle_cloud` present, `/crsd/world_targets` once
   `oak_detector` is started from the GCS Nodes tab (`:8090`).

### Afternoon, in the yard, RC in hand

9. **[yard]** E-stop: SB down cuts the relay in any mode. Do the G1 checks
   ([docs/G1_bench_procedure.md](../G1_bench_procedure.md)) **before `PUBLISH=1` ever runs**.
10. **[yard]** RTK heading valid (G4). **The planner stays inactive until GPS yaw is valid**, so check this before
    blaming Nav2. Also check frames health, the 4×90° rotation check, and `tegrastats` with the rig up (Orin Nano
    CPU load).
11. **[yard]** **Real LiDAR into STVL.** Put a buoy or person 5–15 m from the bow and open the lake panel on the
    boat's map with the costmap layer on: cyan cells should appear there and fade 2–5 s after the object leaves.
    This is the first time STVL sees the real MID360, which is mounted upside down.
12. **[yard]** **Camera tracks.** Walk a RoboBuoy past the bow at 5, 10 and 15 m. Expect one track per buoy, shown as
    `unknown_diamond` (grey "?") unless its side beacon is lit. Note the range where tracks stop appearing.
13. **[yard] Stand rehearsal of lake mode**, defaults `NAV_MODE=shadow` and `PUBLISH` off: PIN AT BOAT ten fake buoys
    (walk the boat around, or type lat/lon), then:
    - COMMIT FIELD, START (pilot arms and selects GUIDED first), ACK each checkpoint, ABORT.
    - Repeat once and **close the browser tab**: the mission must abort within about 18 s.
    - Keep the panel tab in the foreground. A background tab can trip the dead-man.
14. **Pack:** the **LED strip plugged back in** (handbook 5.3.1, competition-blocking), charged packs, RC, laptop,
    field WiFi bridge, 10 buoys with anchors and lines, tape, kayak or safety boat, phone GPS.
    **Unplug the RFD900** for lake day (see LAKE_MODE.md "rxl_link_node").

## Sunday: the lake

Rule for every run: **the RC pilot holds SB and is the only safety path.** The panel is never a stop.

1. **[lake] Datum.** Pick one named point (the launch dock or a pier corner) and use the same `LAKE_DATUM` all day.
2. **[lake] Lay the field** to mirror `task1_avoid`: ENTRY, gate 1 (8–10 m wide), 2 black between, gate 2,
   2 black between, EXIT, about 70 m long. Later, swap a gate's red for a green to make it unpaired (3 green, 2 red).
3. **[asv]** Start the rig in shadow mode (step 2 of LAKE_MODE.md), then `oak_detector` from the GCS. **[laptop]** Open
   `http://<jetson>:8095`.
4. **[lake] Capture the field.** Drive past every buoy in MANUAL. On the map, click each camera track to make it a
   buoy and assign its role. Use PIN AT BOAT or typed lat/lon for any the camera misses. Then COMMIT FIELD and
   **SAVE AS COURSE**.
5. **[laptop] Sim-real.** Copy the saved YAML into `crusader_sim/courses/`, load it in the sim panel, and run it in
   Gazebo with `--mode on`. If the sim fails on your real layout, fix that before the water.
6. **Runs, in order.** Do not move to the next until the previous one is clean.

| # | Mode | What | Pass when |
|---|---|---|---|
| R0 | `NAV_MODE=shadow`, PUBLISH off, MANUAL | START, watch the plan and the path on the panel while the pilot drives | path keeps reds to starboard / greens to port, goes round blacks |
| R1 | `PUBLISH=1 NAV_MODE=on` | START; after the ENTRY orbit, ABORT at checkpoint 1 | full clockwise orbit, ABORT holds |
| R2 | same | full run, ACK each checkpoint by hand | gates crossed, blacks avoided, EXIT circled anticlockwise |
| R3 | same | at a gate checkpoint, recolour a buoy, then SEND CHANGES + ACK | boat replans to the new colours |
| R4 | same, unpaired field | 3 green + 2 red | every red to starboard, every green to port |
| R5 | fallback | `NAV_MODE=off` if R1–R2 misbehave | the old guarded straight legs complete |

7. **[asv] Keep the data** after every run: `~/.cache/crusader_lake/` (logs, layouts) and a bag from the GCS recorder.
   Write down the run number, the time, and what you saw.

## Still open (known limits, honestly)

- The Gazebo **rehearsal of lake mode** and a **full panel-flow sim run** on the merged branch were still pending
  when this was written. Check the morning report.
- **STVL on the real MID360**, and **YOLO on real buoys at the lake**, are both first-time tests. The sim YOLO
  numbers say nothing about the lake (rendering domain gap).
- `beaconFromLabel` maps oak's `blue_circle` (solid blue = EXIT) to Unknown. Harmless in Disruptive (the UAV supplies
  ENTRY/EXIT); it matters for Core tier. Team decision.
- Red and green no longer split by label, so `assoc_radius_m` (3 m) must stay below the narrowest gate.
- The panel's HTTP on :8095 has no authentication. Use it only on the team network.
- The team's `uav_link`/RXL is still not what the real Ekko speaks (TUNNEL 32772/32773). Sunday uses you as the UAV,
  so this does not block Sunday. It does block competition.
