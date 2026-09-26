# `tools/task3_sim/` — Task 3 against the real tree, with no ROS

```bash
python tools/task3_sim/build.py      # once: BT.CPP from source, the math tests, the runner
python tools/task3_sim/sim.py        # then open http://localhost:8088
python tools/task3_sim/sim.py --fire --fire-pump    # the fixed-nozzle shot on its own
```

Plain Python ≥ 3.10 and g++ (MSYS2's mingw-w64 on Windows). **No WSL, no Docker, no ROS.**
First build fetches BehaviorTree.CPP 4.9.0 (~1 MB) into `.deps/` and takes about two minutes.
After that a rebuild is seconds.

## What runs

```
 sim.py  (Python, this folder)                         offros_runner  (C++, child process)
 ┌──────────────────────────────┐   JSON lines   ┌──────────────────────────────────────────┐
 │ world.py                     │  pose, status, │ crusader_bt/offros/offros_runner.cpp      │
 │  course: 3 bays, indicators  │  dock_obs,     │   bt_runner_node's tick loop, no ROS      │
 │  boat:   ArduRover GUIDED    │  ocs_command   │ crusader_bt/src/leaves.cpp       UNCHANGED│
 │  camera: DockObservation     │ ─────────────► │ crusader_bt/src/task3_leaves.cpp          │
 │  lights: fire → green → code │ ◄───────────── │ crusader_bt/behavior_trees/               │
 │  judge:  RoboCommand + UAV   │  setpoints,    │         task3_disruptive.xml              │
 │ page.html on :8088           │  reports,      │ + BehaviorTree.CPP 4.9.0, + a 4-symbol    │
 └──────────────────────────────┘  cannon, tree  │   rclcpp logging shim (offros/shim)       │
                                                 └──────────────────────────────────────────┘
```

**The tree, the leaves and `dock_math.hpp` are the files the boat runs.** Only
`bt_runner_node`'s ROS plumbing is swapped for JSON lines. What a `DockObservation` *means*
is decided in one place — `ingestDockObservation()` in `context.hpp` — which both runners call.

The camera emits the CV team's **draft `DockObservation`, field for field**, and its
`target_pattern` comes from their own timing stage (`vendor/dock_sequence_core.py`, pinned to a
commit), so the tree sees the code appear with the real latency.

## Why not the Task 1 rig

The Task 1 sim (`tools/sitl/`) is WSL2 → Docker → ArduRover SITL + ROS 2. It needs hardware
virtualisation, and it would buy little here: SITL runs a skid-steer `motorboat`, not this hull,
and cannot model a slip. What Task 3 needs tested is the *decision logic* — which bay, which
number, when it is docked, which colour came first — and that is all in the tree. So this rig
runs the tree for real and models the boat kinematically, from `params/working_crusader.params`.

When ROS is available, the tree runs unchanged under `bt_runner_node` (see
`crusader_bt/README.md`); nothing here is needed on the boat.

## The page (:8088)

Its own server, **not** a ground-station tab: the ground station is the boat's operator page on
the Jetson, and a desk tool that invents a world should never be one click away from it on
competition day. 8088 is clear of 8085 (bt_view), 8086 (Task 1 aircraft), 8087 (rx26_uav's
`sim_ekko`), 8090–8093 (the ground stations) and 8080/8081 (camera/LiDAR).

- **Course** — truth (faces, fingers, indicators, the lit window, the hull) and the boat's
  **belief** drawn over it: its bay tracks with their numbers and votes, the berth it planned.
- **Camera** — each `DockObservation` redrawn as an image: faces, window states, indicator,
  and the timing layer's verdict.
- **RoboCommand & UAV** — every report, marked against the truth.
- **Behaviour tree** — live, like `tools/bt_view.py`.
- **Scenario** — green bay, fire window, code colours, dock orientation, **camera pitch**,
  WP_RADIUS; live: camera off, mis-colour rate, a cross-current, the pilot taking MANUAL. The
  facts panel says whether the fire window is even in view from the berth.

## The fixed-nozzle shot (`--fire`)

`sim.py --fire` runs `behavior_trees/task3_fire_test.xml` (docs/T3_coordinated_logistics.md) on
a course where the upper-left window is already burning and the boat starts 3.8 m off the green
bay's slip. What it adds to the world:

- **The nozzle**: a drag-free 45° arc, fitted so a level, square-on boat at `nozzle_hit_range_m`
  (3.22, the pool's answer) puts the stream on the window's top edge. The water goes where the
  boat's true pose, pitch and roll send it; a shot is IN if it crosses the opening (±3 cm).
- **The sea**: `tools/squirt_cal/fake_boat.py`'s rocking (`sea` 0–3), plus a pitch kick when the
  boat accelerates.
- **The LiDAR**: `/crsd/wall_range` from the true pose with `wall_range_node`'s limits (r_max
  4 m, ±35°), the fingers' offset, noise, spray returns; `wall_on_fingers` fits the tips instead.
- **The autopilot**: GUIDED heading+speed as `guided_hs_core` remembers it (turns toward the
  heading, speed as asked, loiters 3 s after the last target), avoidance at 2 m while enabled.
- **The bridge**: its gates, running `guided_hs_core` and `pump_core` themselves.

Without `--fire-pump` the bursts run DRY, as on the boat before G7. `--set NODE.port=value`
runs a copy of the tree with one port changed (`--set StationKeep.lateral=fingers`). The page
adds a side view of the arc with each shot's crossing, the bridge's counts, a sea slider and
the pilot's SE switch.

## Tests

| | What | Time |
|---|---|---|
| `build.py test` | `test_nav_math` (140), `test_dock_math` (136), `test_fire_math` (79), stdlib only | 3 s |
| `test_world.py` | the simulated world on its own: build-guide geometry, numbering, boat, lights, camera view, judge; the nozzle, LiDAR, heading+speed, bridge, pump and sea | < 1 s |
| `test_e2e.py` | the real trees against 30 scenarios, 6 at a time, **real time** (`test_e2e.py fire` = the 16 fire ones) | ~8 min |

Every e2e scenario states what *should* happen, failures included:

| Scenario | Expect | Proves |
|---|---|---|
| `bay1` `bay2` `bay3` | pass | every bay, every window, c1 = c2 included |
| `east_facing` `north_facing` | pass | numbering and approach for a rotated dock |
| `core` `advanced` | pass | one tree serves all three tiers |
| `noisy` | pass | 5 % wrong colours + 15 % abstentions do not move the vote |
| `lost_report` | pass, report re-sent | RoboCommand missing the docking report is recovered |
| `current` | pass **with contact** | the known limit below, pinned |
| `level_camera` | docks, **never sees the fire** | the camera as mounted today, pinned |
| `wp_radius_2` | **no docking** | the precondition below is real |
| `no_camera` | fails **in < 10 s** | a dead dock detector stops the run |
| `no_fire` | fails, **no fire report** | a fire that never lights is never claimed out |

And the fixed-nozzle shot:

| Scenario | Expect | Proves |
|---|---|---|
| `fire_calm` `fire_close_start` | window out | the approach, backing out from inside the slip (2 m), one burst |
| `fire_rocking` | window out, later | waits for a calm spell (sea 2) |
| `fire_rough` | **no burst** | never calls sea 3 steady |
| `fire_offset` `fire_current` | window out (`lateral=fingers`) | 0.3 m off the centreline, 3 cm/s across: aimed from the fingers' offset |
| `fire_far_start` | window out | from 8 m, square first then aim - if `r_max` were raised |
| `fire_beyond_lidar` | **no burst**, at once | from 5 m the LiDAR (r_max 4 m) cannot see the dock |
| `fire_no_wall` `fire_slow_att` | **no burst** | no wall; attitude at 4 Hz cannot judge steady |
| `fire_finger_lock` | **no burst**, boat still | the LiDAR on the finger tips disagrees with the camera by 2 m |
| `fire_drop` `fire_mode_manual` | **stopped within 1 s** from 0.25 m/s | SE: the bridge's own stop (take it out and the boat carries on); SC: the mode |
| `fire_pump_off` | **fails**, no water | no pump output: FireBurst refuses, loudly |
| `fire_cal_error` | 5 shots, **all miss** 16 cm high | a wrong calibration is visible shot by shot |
| `fire_dry` | "fires" 5 × DRY, no pump command | the shadow posture |

## The course and the boat

From the RobotX 2026 build guide (*Docking Bay Structure*): 0.5 m dock cubes; three **1.5 m**
slips between **0.5 m** fingers **2.0 m** long, so the bays are **2.0 m** apart; a 1 m square face
at the back of each slip with the two windows and the indicator where the front-panel drawing
puts them. The boat is ~1.0 × 0.6 m with the camera 0.37 m ahead of centre and 0.41 m above the
water. **Assumed:** the deck's height above the water (0.3 m).

## What the sim found (and the tree now handles)

1. **Vantages inside a slip.** A "close look" nearer than the finger ends is between the
   fingers, and moving between two crossed one. Every look is outside them (5 m), and the line-up
   point too (3 m).
2. **The docked check was looser than the slip.** A centre-point test passed a hull with a
   corner over a finger. `DockedInBay` now tests the whole hull against the slip the boat
   *measured* from its own bay tracks.
3. **A lead-in point behind the boat turned it round twice.** `LinedUp` skips it.
4. **`HoldStation` at a vantage overshot and turned the boat away from the dock** (it re-sends
   the current position while the boat is still moving). The survey waits with `Sleep` instead.
5. **One bay tracked as two** stalled the numbering for 70 s. Tracks closer than a bay pitch now
   merge, and ids stay resolvable.
6. **A pitched camera placed bays wrongly** - 6 % long, and tens of cm sideways off-axis.
   `dock::faceInBody` undoes the pitch exactly.

## What it found that is NOT fixed (needs the boat, or a decision)

- **The camera, level, cannot see either window from the berth.** 0.41 m up and ~0.9 m from
  the face, with the CV's 188-row hull band masked, it sees 19° above its axis; the windows' tops
  are 32° and 41° up. It needs ~25° of pitch-up (or a higher mount). The sim defaults to -25 so
  the tree can be seen working; `level_camera` pins today's mount failing.
- **`WP_RADIUS` is 2.0 m on the boat.** ArduRover calls a GUIDED destination reached inside it
  and loiters where it stopped - at the finger ends, for this berth. Docking needs ~0.2 m (a param
  change), or a docking mode of its own. `wp_radius_2` reproduces it.
- **The berth cannot be held against a cross-current.** GUIDED loiters with `LOIT_RADIUS` 2.0 m
  and does not strafe on this frame; the slip leaves 0.45 m either side. 5 cm/s puts the hull on
  a finger in ~10 s. Holding in a slip needs the hull's lateral thrust, which GUIDED does not use.
- **No undocking.** Leaving the slip is stern-first, and the setpoint path is position-only.

## What the fire sim found (and is now fixed)

1. **Driving the whole approach on the aim heading walks the boat sideways** - 2-4° over 4 m is
   15-25 cm, most of a 21 cm window. `StationKeep` now approaches square to the wall and turns
   to the aim only within 0.5 m of the firing range (`fire_far_start` missed every shot before).
2. **`wall_range_node` cannot see the dock beyond ~4 m** (`r_max`): a hand-over further out
   ends the run at once. Hand over inside 4 m, or raise `r_max`.
3. **From outside the slip the wall fit locked onto the finger tips** (2 m short) when the
   panel stands back from the deck edge - found by a ray-cast test in
   `crusader_perception/test/test_wall_fit_core.py`, fixed in `wall_fit_core` (a line you can
   see through is not the wall; the nearest solid line is). The camera cross-check stays as the
   second guard.
4. **`wall_fit_core`'s refine did a full SVD** - an N×N matrix per sweep, 0.5 s on a desktop
   near a wall. Now 30 ms.

## What it is not

Not hydrodynamics, not a camera image, not the water stream's flight, not wind or waves. The
fingers do not stop the hull — contact is *recorded*. The boat's dimensions are rough, and the
deck's freeboard is assumed; the window heights, and so the pitch the camera needs, follow from
it.

## Files

| File | Is |
|---|---|
| `build.py` | fetch BT.CPP, compile it, run the math tests, link the runner |
| `sim.py` | the loop, the runner child, the page server, `--headless` |
| `world.py` | course, boat, camera, lights, judge; nozzle, sea, LiDAR, bridge — pure Python (+ `guided_hs_core`, `pump_core`, `nozzle_model` from the repo) |
| `page.html` | the page |
| `vendor/dock_sequence_core.py` | the CV team's timing stage, unmodified, pinned |
| `test_world.py`, `test_e2e.py` | the tests above |
