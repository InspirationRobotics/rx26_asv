# Task 3 — Coordinated Logistics: the USV half

handbook 3.3.4. Three bays, numbered **left to right facing them**; one shows a GREEN docking
indicator. Dock in it, report `DockingReport(bay_id)`; *then* one of that bay's two windows lights
RED. Spray it until GREEN (5 s), report `FirefightingReport(window_id)`. Disruptive: off 1 s, then
`c1 1 s · off 1 s · c2 1 s · off 2 s` for 60 s — c1 the tin, c2 the circle. Report
`ResourceDeliveryRequest` and relay it to the UAV.

Code: `crusader_bt/behavior_trees/task3_disruptive.xml`, `src/task3_leaves.cpp`,
`include/crusader_bt/dock_math.hpp`. Simulator: `tools/task3_sim/`.

## The course and the boat, in numbers

| | | Source |
|---|---|---|
| Dock | 0.5 m cubes: a 13 × 2-cube main deck, four 1 × 4-cube fingers | build guide, *Docking Bay Structure* |
| Slips | **1.5 m wide**, fingers **2.0 m long**, 0.5 m wide; bays **2.0 m apart** | same |
| Face | 1.0 × 1.0 m panel on the deck, at the back of each slip, facing in | same, front-panel drawing |
| Windows | upper-left opening 210 × 290 mm, centre ~0.75 m up the panel; lower-right ~230 × 310, centre ~0.51 m up; indicator at the bottom centre | same |
| Deck height | ~0.05 m above water (face panel bottom) | every bay is the practice dock's height (2026-10-04); windows ~0.80 m (UL) / ~0.56 m (LR) up, **estimated** - measure |
| USV limit | fits a 2 × 1 × 1 m box | handbook 5.3.1 |
| The boat | ~1.0 × 0.6 m; camera 0.37 m ahead of centre, 0.41 m above water | team; `Resources.md` |

## What perception hands us

The dock detector (firefighting-cv; not in this repo yet) is a colour-blind **3-class** YOLO —
`bay_face`, `window`, `dock_indicator` — followed by rules for geometry, colour and timing. The
tree never sees a class. It sees one `crusader_msgs/DockObservation` per camera frame:

```
DockObservation            every frame, empty or not; stamped at the camera instant
├─ bays[]: DockBay         one per face seen, sorted left→right IN THIS FRAME
│   ├─ bay_index             0 = leftmost in the image — NOT a persistent id
│   ├─ truncated             cut by the image edge or the hull band
│   ├─ indicator_present, indicator_colour (0 unk, 1 off, 2 red, 3 green, 4 blue), _confidence
│   ├─ windows[]: DockWindow
│   │    index (slot, from geometry), slot ("UL"/"LR"), state (same numbering), aim point x,y,z
│   ├─ lit_window_index, lit_state
│   ├─ has_plane, plane_normal, plane_offset      face plane from stereo, camera frame
│   ├─ range_from_size_m                          fallback range from window height
│   └─ bearing_deg                                face centre, + LEFT, camera frame
├─ target_pattern          "", steady, flash, code, unresolved   ← the timing layer
├─ target_colours          ["red"] or ["red","blue"]; c1 = the colour AFTER the 2 s off
├─ target_window_index
├─ last_event              hit / hit_done / lost — ON ONE FRAME ONLY
└─ observed_fps            < 4: flashes cannot be resolved
```

Three things in it are easy to misuse, and the tree guards all three:

- **`bay_index` is per frame.** Seeing bays 2 and 3 yields indices 0 and 1. The runner places
  every sighting in the world and associates by position (`dock::DockBook`); the bay *number*
  comes from sorting three confirmed tracks along the dock (`dock::layout`).
- **Its colour numbers are not RoboCommand's.** CV: RED 2, GREEN 3. RoboCommand `Color` and
  `RXL_COLOR`: RED 1, GREEN 2. Casting one to the other reports RED as GREEN.
- **Everything is in the camera frame, which a pitched camera tilts.** `dock::faceInBody` undoes
  the pitch exactly (`cam_pitch_deg`, `dock_face_dz_m`); ignoring it costs ~6 % of range and, off
  to the side, tens of centimetres.

## The plan

| Phase | How | Why this way |
|---|---|---|
| Survey | Look at the whole dock from **5 m** in front (the outer faces sit ~22° off the bow); then head-on to the least-read bay from the same 5 m; a ring search if nothing is in view | outside the 2 m fingers with the whole hull; indicators inside the 6 m the CV reads them well at |
| Choose | Per-bay indicator **votes**, not a single reading: GREEN = ≥ 8 readings, ≥ 80 % green, and recent readings agree. Strict first (GREEN + RED + RED), lenient only when out of places to look | the CV abstains sometimes; one misread is a wrong bay |
| Berth | Lead-in 5 m → line-up 3 m → berth 1.25 m (face to body origin), all on the bay's own centreline; re-planned every tick; the choice re-checked on the way in | line up outside the fingers; berth leaves the stern inside them and the bow clear of the face |
| Docked? | Berth tolerances **and** every hull corner inside the slip the boat measured | a centre-point test passed a hull over a finger |
| Fire | Wait for the timing layer's steady RED, re-sending the docking report until RoboCommand confirms; spray at the CV aim point every tick until GREEN | a lost report is a fire that never lights |
| Code | Wait for `code` (c1, c2), then the same answer for 5 s more; report and relay | a report cannot be taken back; 5 s is one period out of sixty |

**No strafing.** The hull can strafe in MANUAL, but a tree driving MANUAL means RC overrides from
the companion, which the README gates behind G1 — and nothing in the plan needs it. The one place
lateral thrust would help is *holding* the berth against a cross-current (below).

## The camera cannot see the windows from the berth, as mounted

Berthed, the camera is ~0.9 m from the face and 0.41 m up. The upper window's top is ~1.2 m up —
41° above level; the lower window's top ~0.97 m — 32°. The OAK-D LR sees 27° above its axis, and
19° once the CV's 188-row hull band is masked. **Level, it sees neither window whole from inside
the slip**, and backing off does not help: the stern would leave the 2 m fingers first.

| Berth (face → body origin) | Lower window needs | Upper window needs |
|---|---|---|
| 1.25 m, with the hull band | ~13° pitch-up | ~22° pitch-up |
| 1.25 m, without the band | ~5° | ~15° |
| 1.5 m (stern at the finger ends), with the band | ~7° | ~15° |

So the mount wants **~25° of pitch-up** (`cam_pitch_deg ≈ -25`), a higher mount, or a second
camera. The tree already handles a pitched camera; the sim defaults to -25 and pins the level case
failing (`test_e2e level_camera`). Pitched up, the camera still sees the indicators from the survey
standoff (they sit at about its own height) but loses the near water — worth weighing against
Task 1.

## What goes out

| To | Topic (JSON) | Body |
|---|---|---|
| RoboCommand (via the OCS) | `/crsd/docking_report` | `{"bay_id": 2}` — 1..3, left to right facing the bays |
| | `/crsd/firefighting_report` | `{"window_id": 1}` — `DockWindow.index + 1` (**to confirm**) |
| | `/crsd/resource_delivery_request` | `{"task": "TASK_COORDINATED_LOGISTICS", "resource_color": "COLOR_RED", "delivery_circle_color": "COLOR_BLUE"}` |
| the UAV (via the radio) | `/crsd/uav_resource_request` | `{"seq": 1, "resource_color": 1, "delivery_color": 3}` (RXL numbering) |
| the pump | `/crsd/water_cannon` | `{"fire": true, "frame_id": "camera_link", "x":…, "y":…, "z":…}` |

RoboCommand's `ReadinessConfirm` arrives on `/crsd/ocs_command` (`ocs_client` republishes OCS
commands and acts on none).

## The fixed-nozzle shot

The nozzle is fixed at **30°** (since 2026-09-28; it was ~45°), so **the boat is the aim**: its
range to the dock sets how high the water lands, its sideways position where. The upper-left
window takes water from LiDAR wall range **1.0 m** (squirt_cal, 2026-09-28) **to 1.6 m** (shot
outside, 2026-10); the trees use **1.4 m**. The lower-right wants **0.7–0.8 m** (the trees: 0.8).
The old 45° nozzle wanted 3.1–3.4 m.

**`behavior_trees/task3_fire_manual.xml` — MANUAL, on the sticks.** This hull is OmniX: it
strafes, but only in MANUAL (GUIDED turns and drives). So the tree drives the sticks the way the
team's `dp_hold` did — RC override on ch1 steer, ch3 throttle, ch4 lateral — and holds three things:

| Axis | Held to | Measured by |
|---|---|---|
| surge (ch3) | the calibrated range, 1.4 m (upper-left) | the LiDAR wall range (`/crsd/wall_range`) |
| sway (ch4) | the window on the nozzle's line | the **camera**: `DockWindow` x, y, z of the upper-left window, in the body frame |
| yaw (ch1) | square to the face | the **camera**: the line through the face's two windows (or the face plane), as a compass heading held between frames |

The LiDAR's wall angle is not used — the wall may not be flat, the face is — and the heading
barely matters once the aim is by strafing: the window's sideways position in the *body* frame
is where the stream lands whatever the heading. Each axis is P + D with the ESC deadband
compensated (Blue Robotics ESCs do nothing within ±25 µs), an integrator for a steady current
(only near the target and nearly still, so the approach does not wind it up), a 120 µs cap and a
gentle slew on growing thrust (never on cutting it). dp_hold's gains on this hull are the start.

```
ReactiveSequence   guard band, every tick: ModeIs MANUAL, NotDropped, WallRangeAlive,
                   AttitudeAlive
  StrafeKeep       ALWAYS SUCCESS: the three sticks; zero while the LiDAR and camera
                   disagree or water is in the air
  Sequence         up to 5 × [AwaitStrafeSolution → FireBurst → WindowOut] → ShotsFired ≥ 1
```

It fires when range ±6 cm, the window within 5 cm of the line, within 5° of square, the camera
fresh, the hull still (attitude ≥ 8 Hz, rates < 4°/s, ±1.5°) and **not being moved** (no
correction for 1.5 s; a steady holding push against a current is not a correction) — all held
1 s. Everything is in `crusader_bt/include/crusader_bt/fire_math.hpp` (117 checks), the leaves
in `src/fire_leaves.cpp`, every number an XML port.

**The camera half is the dock detector.** `tools/dock_view.py` runs the CV team's model and
colour/timing chain and now publishes `DockObservation` on `/dock/observations`
(`crusader_perception/dock_obs_core.py`): it fits a plane to the OAK-D's stereo depth over the
bay face (window and indicator openings cut out) and puts each window's box centre on that
plane — x, y, z in `camera_link`. A few hundred bytes a frame; the image and the depth never
leave the Jetson. `--no-publish` makes it a viewer again.

**Taking it back.** `telemetry_bridge` forwards the sticks **only in MANUAL**, **only on ch1/3/4**
(SB, SC and the pump can never be overridden), clamped to ±150 µs, and **releases** them after
0.5 s of silence (`crusader_fcu/override_core.py`). So:
- **SC to the middle (HOLD)** or up (GUIDED): the tree ends, the bridge refuses and releases —
  the boat is the pilot's at once (sim: `fire_sc_hold`, 0.13 → 0.01 m/s in 1 s).
- **SB**: the e-stop, whatever the software says.
- **The drop latch (ch9)**: no switch is mixed to ch9 yet (SE is the pump; **SD** is the
  proposal), so today it never trips. Until it is wired, SC and SB are the kills.

**Two switches on `bt_runner_node`, both off by default:** `publish_setpoints` (may it take the
sticks — G1) and `fire_pump` (may it squirt — G7). Both off, the tree is a **shadow** in MANUAL:
you drive, it logs what it would do every second (`strafe: range 3.41 m, window +0.12 m left,
square +0.4 deg | sticks fwd +48 lat -41 yaw +0 us (moving in) [shadow ...]`).

**Hand over inside 4 m.** `wall_range_node.r_max` is 4 m: further out it cannot see the dock and
the guard band ends the run at once.

**`task3_fire_test.xml` is the GUIDED version** (heading + signed speed through
`/crsd/guided_heading_speed`, `guided_hs_core`): it cannot strafe, so it aims by turning. It is
kept, tested (`guided_calm`, `guided_drop`), for when GUIDED can strafe.

**Where it fires from.** With the 30° nozzle, inside the slip: at 1.4 m (upper-left) the
stern is ~0.1 m inside the 2 m fingers, at 0.8 m (lower-right) well inside.
`task3_part2_dock_fire.xml` docks at 1.4 m and fires from there or from 0.8 m. (With the old 45°
nozzle the boat would have had to back out to 3.2 m.)

### Before it moves the real boat, in this order

1. `colcon build` of `crusader_msgs`, `crusader_fcu`, `crusader_perception`, `crusader_bt` in the
   container — the first real build of `bt_runner_node`'s Task 3 and fire code.
2. `dock_view` on the boat: `ros2 topic echo /dock/observations` in front of the bay — the
   windows' x, y, z against a tape measure, `has_position` true.
3. [G1](G1_bench_procedure.md), **the RC override rows O1–O8**, props off — including the stick
   DIRECTIONS (`stick_reverse` if one is backwards).
4. [G7](G7_pump_bench.md) — the pump bench, `SERVO9_TRIM` = the OFF value first.
5. On the water, in MANUAL: a **shadow run** (both switches off; you drive, read the log), then
   **hold** (`publish_setpoints` on, `fire_pump` off — bursts run DRY), then **fire**.

The sim runs all of it first: `python tools/task3_sim/sim.py --fire --fire-pump`, and 20 fire
scenarios in `test_e2e.py` (tools/task3_sim/README.md).

## Open questions

| Question | Who | What depends on it |
|---|---|---|
| Camera mount: pitch it up ~25°, raise it, or add a camera? | boat | whether the fire can be seen at all |
| `WP_RADIUS` 2.0 → ~0.2 for Task 3 (or a docking mode) | boat params | the berth approach parks at the finger ends otherwise |
| Holding the berth against a cross-current: GUIDED loiters (`LOIT_RADIUS` 2 m) and does not strafe | boat / autonomy | hull contact within ~10 s at 5 cm/s |
| `FirefightingReport.window_id` numbering (1-based left-to-right? top/bottom?) | RoboNation | `ReportFirefighting window_id_base` |
| "Fully docked" defined how; is touching a finger penalised? | RoboNation | `DockedInBay`, the berth depth |
| The deck's height above water on the day | on site | the window heights, so the pitch |
| A USV→UAV resource-request message in the RXL dialect (only UAV→USV `RXL_RESOURCE_DELIVERY` exists) | UAV team | `rxl_link_node` putting `/crsd/uav_resource_request` on the air |
| The water cannon: fixed or pan/tilt, where, what it takes | **answered 2026-09-25: FIXED**, ~45°, ~3 m; on a Pixhawk pass-through output, the pilot's SE on ch10. The boat's position is the aim. `tools/squirt_cal` calibrates the range per window against `/crsd/wall_range`; the pump path is `/crsd/pump_cmd` → `telemetry_bridge` ([G7](G7_pump_bench.md)). The shot itself is `task3_fire_manual.xml` (above); `task3_disruptive.xml`'s `SprayUntilHit` (pan/tilt aim point) is still to be replaced by it | `SprayUntilHit` |
| A drop switch on ch9 (SD proposed) | team | until it exists the drop latch never trips; SC and SB are the kills |
| Firing from outside the slip: may the boat back out of the bay to fire, after reporting docked? | RoboNation | where the fixed nozzle can fire from (3.2 m is outside the fingers) |
| The real dock's deck height, and whether the face panels stand at the deck edge or set back | on site | re-calibrate: 5 cm of height is ~12 cm of range; `face_setback_m` for the camera cross-check |
| The dock detector node itself (draft spec in firefighting-cv) | CV team | everything above; the tree fails fast without it (`DockCameraAlive`) |

Undocking is not implemented: leaving a slip is stern-first and the setpoint path is
position-only.
