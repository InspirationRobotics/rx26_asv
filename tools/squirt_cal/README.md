# `tools/squirt_cal`: finding where the boat has to sit for the fixed nozzle to hit

The Task 3 nozzle is fixed: about 45°, about 3 m range, no pan or tilt. The aim is
therefore where the boat sits:
- **distance** to the face sets the hit height;
- **sideways** position sets left/right.

This tool finds both for each target. You fire a short burst, and one person
answers on a phone: "the boat should move **forward / back / left / right** / on".
The tool logs every shot against the LiDAR's range to the wall and says where to
go next ("7 cm BACK"). The result is a range per target for the Task 3 tree.

```
python3 tools/squirt_cal/squirt_cal.py            # on the Jetson (ROS)
python  tools/squirt_cal/squirt_cal.py --fake     # anywhere: a simulated boat
```

Then open **`http://<jetson>:8094`** on the phone, on the boat's WiFi.
- With `--fake` it runs on a laptop with no ROS, and a *Fake boat* panel appears.
- Every run is one **session**: `<log_dir>/<YYYYmmdd-HHMMSS>/shots.jsonl`, plus
  one camera frame per shot when `snapshot_url` is up.

## One person, one phone

1. **Pick the target**: upper-left or lower-right window, top edge.
2. **Drive to the readout.** The big number is the LiDAR range now, and the line
   under it says which way to move and how far. "square up: turn left 4°" means
   the boat is not square to the wall.
3. **Hands off the sticks.** The steady light goes green once the boat has
   stopped rocking and the sticks have been still for 1.5 s.
4. **Hold SQUIRT** (0.6 s, so a pocket tap cannot fire).
   - In *Fire when steady* mode it waits up to 15 s for the steady light, then
     fires.
   - In *Fire now* mode it fires at once, and the shot is logged as not steady.
5. **Tap where the BOAT should go.** It is a 3×3 grid, wall at the top: ↑ is
   "move forward" (toward the wall). The corners combine two answers, for
   example move forward *and* left. (The first pool session's buttons said
   "too far FORWARD" and were answered as "move forward"; its log is read with
   fore/aft flipped. The buttons now say what people mean.)
   - *Didn't see* discards the shot.
   - *Undo* takes back a mis-tap.
6. Repeat. After the first hit it probes half a step past it on each side, to
   find the band that hits, then aims for the band's middle.
7. **Copy the YAML** at the bottom: the range (and sideways offset) per target,
   with the band it came from.

**Log only.** Until the pump path passes [G7](../../docs/G7_pump_bench.md), the
tool cannot fire (the page says LOG ONLY). Fire with your own switch instead; the
tool sees the pump output go on and logs a shot the same way.
- With *Chime when steady* on, it beeps when to flick the switch.
- **No LiDAR?** Type a tape-measured range under *No LiDAR?*. It is used for every
  shot until cleared.

## Why it is built this way

- **Verdicts are about the boat, not the water.** With a fixed nozzle, "the water
  hit high" means move closer on the rising part of the arc and further back on
  the falling part. "The boat should move back" means the same thing on both
  parts. So each verdict is a one-sided bound on the range, and the estimate is a
  bracket, not a physics fit.
- **The range is the one measured BEFORE the burst.** It is the median over
  0.5 s, because the spray returns LiDAR points while it is in the air. The tests
  prove the logged range ignores them.
- **It calibrates against `/crsd/wall_range`, the same number the tree will use.**
  The MID360 sits low, so on a raised dock it ranges the dock's EDGE, not the
  panel. That offset cancels as long as calibration and firing use the same
  measurement. `face_setback_m` only affects the starting guess.
- **Steady before firing.** At ~1 m, a degree of pitch moves the hit ~2.4 cm. The
  gate needs attitude at ≥ 10 Hz (`SR0_EXTRA1`), rates under 4°/s, roll and pitch
  inside ±1.5° of their mean, and no stick movement for 1.5 s. Each shot logs the
  peak-to-peak pitch and roll while the water was in the air, so rocking shows up
  in the data.
- **It cannot hold the pump on.** It sends `/crsd/pump_cmd` bursts that
  `telemetry_bridge` checks (`crusader_fcu/pump_core.py`), and the autopilot times
  each one (DO_REPEAT_SERVO, one cycle). It never opens MAVLink (README rule 1).
  It refuses to fire while a mission is running (`/crsd/current_task`).
- **A bench tool, not a launch-file node.** It has no entry point and is not in
  `core.launch.py`: nothing ships until it has run on the boat. It reuses
  `crusader_groundstation`'s `GcsServer`, so every action is a POST re-checked
  on the server.

## Before the first real session

Measure these and put them in `crusader_params.yaml` → `squirt_cal`. They only
move the starting guess, but a bad guess costs shots:
- `nozzle_height_m`: above the waterline, boat loaded as it runs.
- `nozzle_x_m`: ahead of the LiDAR's body origin.
- `nozzle_elev_deg`, `nozzle_range_m`.
- `deck_height_m`: the face panel's bottom edge above the water, **on the real
  dock**.

The page warns when the nozzle model cannot reach a target at all. With the
default numbers (nozzle 0.40 m, deck 0.30 m, 45°, 3 m) **the upper window's top
edge is about 4.5 cm out of reach.** An elevated dock makes that worse. If a
session closes its bracket with no hits, the page says so: steepen or raise the
nozzle.

## Files

| File | What |
|---|---|
| `squirt_core.py` | the session: config, the shot state machine, the estimator, the log. No ROS, no HTTP |
| `steady_core.py` | "is the boat still enough to shoot" |
| `nozzle_model.py` | drag-free arc: the starting guess and the fake boat's truth |
| `fake_boat.py` | a boat, a pilot, a pump, a LiDAR and an honest operator, for `--fake` |
| `ros_adapter.py` | the topics (rclpy is imported only here) |
| `squirt_cal.py` | entry point: config, adapter, server, tick |
| `page.html` | the phone page |

## Tests (no ROS)

```bash
python tools/squirt_cal/test_nozzle_model.py   # the arc, hand-worked
python tools/squirt_cal/test_steady_core.py    # rocking, sticks, slow attitude
python tools/squirt_cal/test_squirt_core.py    # bracket, state machine, log, undo
python tools/squirt_cal/test_fake_e2e.py       # whole sessions against the fake boat
```

`test_fake_e2e` runs whole sessions against a fake nozzle that is **not** the one
in the config, with rocking, and with an operator who is wrong 10% of the time:

| Target, conditions | Shots | Sessions hitting within |
|---|---|---|
| lower-right, calm | 12 | 10/10 within 2 cm |
| lower-right, rough | 12 | 10/10 within 3.5 cm |
| lower-right, noisy operator | 15 | 10/10 within 2.5 cm |
| upper-left, of the draws that can reach it | 12 | 7/7 within 2 cm |

"Within" means the stream height at the estimated range on a level boat. The test
also checks that sessions which cannot reach the edge end with the "cannot reach"
warning.

## Not done here

- **Moving the boat itself.** The dp_hold-style hold in git history is RC override
  in MANUAL, which is G1-gated.
- **The Task 3 tree using the result.** `SprayUntilHit` still assumes a pan/tilt
  cannon.
- **A nozzle model in `tools/task3_sim`.**
