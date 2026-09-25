# Task 3 — Coordinated Logistics: the USV half

handbook 3.3.4. Three bays, numbered **left to right facing them**; one shows a GREEN docking
indicator. Dock in it, report `DockingReport(bay_id)`; *then* one of that bay's two windows lights
RED. Spray it until GREEN (5 s), report `FirefightingReport(window_id)`. Disruptive: off 1 s, then
`c1 1 s · off 1 s · c2 1 s · off 2 s` for 60 s — c1 the tin, c2 the circle. Report
`ResourceDeliveryRequest` and relay it to the UAV.

Code: `crusader_bt/behavior_trees/task3_disruptive.xml`, `src/task3_leaves.cpp`,
`include/crusader_bt/dock_math.hpp`. Simulator: `tools/task3_sim/`.

## What perception hands us

The dock detector (firefighting-cv; not in this repo yet) is a colour-blind **3-class** YOLO —
`bay_face`, `window`, `dock_indicator` — followed by rules for geometry, colour and timing. The
tree never sees a class. It sees one `crusader_msgs/DockObservation` per camera frame:

```
DockObservation            every frame, empty or not; stamped at the camera instant
├─ bays[]: DockBay         one per face seen, sorted left→right IN THIS FRAME
│   ├─ bay_index             0 = leftmost in the image — NOT a persistent id
│   ├─ truncated             cut by the image edge (centre biased)
│   ├─ indicator_present, indicator_colour (0 unk, 1 off, 2 red, 3 green, 4 blue), _confidence
│   ├─ windows[]: DockWindow
│   │    index (slot, from geometry), slot ("UL"/"LR"), state (same numbering), aim point x,y,z
│   ├─ lit_window_index, lit_state
│   ├─ has_plane, plane_normal, plane_offset      face plane from stereo, camera frame
│   ├─ range_from_size_m                          fallback range from window height
│   └─ bearing_deg                                face centre, + LEFT
├─ target_pattern          "", steady, flash, code, unresolved   ← the timing layer
├─ target_colours          ["red"] or ["red","blue"]; c1 = the colour AFTER the 2 s off
├─ target_window_index
├─ last_event              hit / hit_done / lost — ON ONE FRAME ONLY
└─ observed_fps            < 4: flashes cannot be resolved
```

Two things in it are easy to misuse, and the tree guards both:

- **`bay_index` is per frame.** Seeing bays 2 and 3 yields indices 0 and 1. The runner places
  every sighting in the world and associates by position (`dock::DockBook`); the bay *number*
  comes from sorting three confirmed tracks along the dock (`dock::layout`).
- **Its colour numbers are not RoboCommand's.** CV: RED 2, GREEN 3. RoboCommand `Color` and
  `RXL_COLOR`: RED 1, GREEN 2. Casting one to the other reports RED as GREEN.

## The plan

| Phase | How | Why this way |
|---|---|---|
| Survey | Look at the whole dock from **10 m** in front (all three bays fit the ~79° FOV); then head-on to the least-read bay; a ring search if nothing is in view | fingers ~6 m long: anything closer is inside a slip. GUIDED cannot strafe this hull, and nothing needs to |
| Choose | Per-bay indicator **votes**, not a single reading: GREEN = ≥ 8 readings, ≥ 80 % green, and recent readings agree. Strict first (GREEN + RED + RED), lenient only when out of places to look | the CV abstains sometimes and indicator recall falls past 6 m; one misread is a wrong bay |
| Berth | Lead-in 13 m → line-up 9 m → berth 3 m, all on the bay's own centreline; re-planned every tick from the latest estimate; the choice is re-checked on the way in | arriving from the side means turning 90° inside the run-in; the bay is read best close in |
| Docked? | Berth tolerances **and** every hull corner inside the slip width the boat measured | a centre-point test passed a hull 0.3 m over a finger |
| Fire | Wait for the timing layer's steady RED, re-sending the docking report until RoboCommand confirms; spray at the CV aim point every tick until GREEN | a lost report is a fire that never lights |
| Code | Wait for `code` (c1, c2), then the same answer for 5 s more; report and relay | a report cannot be taken back; 5 s is one period out of sixty |

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

## Open questions — not ours to decide

| Question | Who | What depends on it |
|---|---|---|
| `FirefightingReport.window_id` numbering (1-based left-to-right? top/bottom?) | RoboNation | `ReportFirefighting window_id_base` |
| Bay width and finger length; "fully docked" defined how; is touching a finger penalised? | RoboNation / course drawings | every standoff in the tree; `DockedInBay` |
| A USV→UAV resource-request message in the RXL dialect (only UAV→USV `RXL_RESOURCE_DELIVERY` exists) | UAV team | `rxl_link_node` putting `/crsd/uav_resource_request` on the air |
| The water cannon: fixed or pan/tilt, where, what it takes | hardware | whatever subscribes `/crsd/water_cannon` |
| `WP_RADIUS` 2.0 → ~0.3 for Task 3 (or a docking mode) | boat params | the berth approach parks 2 m short otherwise |
| Holding the berth against a cross-current: GUIDED loiters (`LOIT_RADIUS` 2 m) and does not strafe | boat / autonomy | hull contact within ~15 s at 5 cm/s |
| The dock detector node itself (draft spec in firefighting-cv) | CV team | everything above; the tree fails fast without it (`DockCameraAlive`) |

Undocking is not implemented: leaving a slip is stern-first and the setpoint path is
position-only.
