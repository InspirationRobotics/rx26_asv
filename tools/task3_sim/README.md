# `tools/task3_sim/` — Task 3 against the real tree, with no ROS

```bash
python tools/task3_sim/build.py      # once: BT.CPP from source, the math tests, the runner
python tools/task3_sim/sim.py        # then open http://localhost:8088
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
- **Scenario** — green bay, fire window, code colours, dock orientation, bay width, WP_RADIUS;
  live: camera off, mis-colour rate, a cross-current, the pilot taking MANUAL.

## Tests

| | What | Time |
|---|---|---|
| `build.py test` | `test_nav_math` (140) and `test_dock_math` (121), stdlib only | 2 s |
| `test_world.py` | the simulated world on its own: projection, numbering, boat, lights, camera, judge | < 1 s |
| `test_e2e.py` | the real tree against 13 scenarios, 6 at a time, **real time** | ~5 min |

Every e2e scenario states what *should* happen, failures included:

| Scenario | Expect | Proves |
|---|---|---|
| `bay1` `bay2` `bay3` | pass | every bay, every window, c1 = c2 included |
| `east_facing` `north_facing` | pass | numbering and approach for a rotated dock |
| `core` `advanced` | pass | one tree serves all three tiers |
| `noisy` | pass | 5 % wrong colours + 15 % abstentions do not move the vote |
| `lost_report` | pass, report re-sent | RoboCommand missing the docking report is recovered |
| `current` | pass **with contact** | the known limit below, pinned |
| `wp_radius_2` | **no docking** | the precondition below is real |
| `no_camera` | fails **in < 10 s** | a dead dock detector stops the run |
| `no_fire` | fails, **no fire report** | a fire that never lights is never claimed out |

## What the sim found (and the tree now handles)

1. **The first vantage and every "close look" were inside a slip.** With fingers ~6 m long,
   5 m from the face is between them; moving between two such looks crossed a finger. Every look
   is now from outside the fingers (10 m), and the line-up point too (9 m).
2. **The docked check was looser than the slip.** 0.5 m off and 15° skewed passed it, with a
   corner 0.3 m over the finger. `DockedInBay` now tests the whole hull against the slip width
   the boat *measured* from its own bay tracks.
3. **A lead-in point behind the boat turned it round twice.** `LinedUp` skips it.
4. **`HoldStation` at a vantage overshot and turned the boat away from the dock** (it re-sends
   the current position while the boat is still moving). The survey waits with `Sleep` instead.

## What it found that is NOT fixed (needs the boat, or a decision)

- **`WP_RADIUS` is 2.0 m on the boat.** ArduRover calls a GUIDED destination reached inside it
  and loiters where it stopped, 2 m short of the berth. Docking needs ~0.3 m (a param change), or
  a docking mode of its own. `wp_radius_2` reproduces it.
- **The berth cannot be held against a cross-current.** GUIDED loiters with `LOIT_RADIUS` 2.0 m
  and does not strafe on this frame; the slip leaves ~0.8 m either side. 5 cm/s puts the hull on a
  finger in ~15 s. Holding in a slip needs the hull's lateral thrust, which GUIDED does not use.
- **No undocking.** Leaving the slip is stern-first, and the setpoint path is position-only.

## What it is not

Not hydrodynamics, not a camera image, not the water stream's flight, not wind or waves. The
fingers do not stop the hull — contact is *recorded*. Bay width and depth are **assumed**
(4.0 × 6.0 m) until the course drawings are out; every standoff in the tree depends on the
depth, so check them when they are.

## Files

| File | Is |
|---|---|
| `build.py` | fetch BT.CPP, compile it, run the math tests, link the runner |
| `sim.py` | the loop, the runner child, the page server, `--headless` |
| `world.py` | course, boat, camera, lights, judge — pure Python |
| `page.html` | the page |
| `vendor/dock_sequence_core.py` | the CV team's timing stage, unmodified, pinned |
| `test_world.py`, `test_e2e.py` | the tests above |
