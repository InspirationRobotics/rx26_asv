# Handoff: calibrate the water cannon and the hold's PID in the Gazebo sim

Written 2026-10-12 for the chat that built the Task 3 Gazebo sim, from the chat that has
been running Task 3 on the water with Dominick. Branch `task3-gazebo`, commit `66e60e3` or
later. Read [T3_cannon_sim.md](T3_cannon_sim.md) (your own doc) and
[T3_cannon_cal.md](T3_cannon_cal.md) (the on-water calibration) alongside this.

## What Dominick wants

1. **The water cannon calibrated in the sim as far as the sim can take it**, before the next
   water session, so less is tuned by hand at the dock.
2. **PID values for heading, strafing and forward/back** that hold smoothly, worked out in
   the sim.
3. **Less hard-coding.** A taller dock, a different range or a different window should not
   need a retune.

Two facts decide how much of that the sim can deliver today. Both are yours to fix first.

- **The sim's water cannot miss.** `task3_world.py` flies the water from the same
  `cannon_aim_node` parameters the aim uses (lines 227 and 365-367: nozzle, exit speed,
  servo map). Whatever the aim believes, the sim agrees with. Nothing can be calibrated
  against that. Part A starts by splitting "what is true" from "what the aim believes".
- **The sim's hull is a placeholder.** `crusader_hull.yaml` tags mass (30 kg), CG and every
  damping term `GUESS`; the thruster layout is assumed (`MODEL_REQUEST.md`). Gains tuned on
  it are gains for a different boat. Part B starts by fitting the sim's boat to the real one.

## What changed on the water since the sim was built (2026-10-05 to 10-11)

| Change | Where | What it means for the sim |
|---|---|---|
| Water stays ON until GREEN (no penalty for too much water) | `cannon_aim_core.FireGate`, `pump_core` | `gap_s` is gone from `crusader_params.yaml`; `resend_s: 0.3` replaces it. A burst asked while one runs extends it. Re-run your courses: the pump path changed |
| Water model from one measured throw | `cannon_aim_node` params | 2.74 m out at 30 deg, landing 0.51 m below the nozzle = 4.85 m/s (was 5.42 from 3.0 m at 45 deg). `throw_drop_m` is new |
| The tilt servo is mounted reversed | `tilt_center_us: 1680`, `tilt_sign: -1` | `gen_crusader.py` reads these, so the Gazebo joint should already follow. Confirm it does: level at 1680 us, higher PWM = nozzle DOWN |
| Per-window trims | `pan_trim_ul_deg`, `tilt_trim_ul_deg`, `pan_trim_lr_deg`, `tilt_trim_lr_deg` | The common trims are gone. All four are 0 in the repo |
| Pump output | `pump_servo_channel: 9` (AUX1) | The sim overrides it to 10 in `gz_rig_up.sh`. Leave that; just know the YAML value is the boat's |
| A fire needs 2.5 s of RED on the window itself | `AwaitFireTarget` `confirm_s`, `dropout_s` | The timing layer called a code (blue, then red) "steady RED"; the boat went back in to spray. Fires are called ~1 s later now |
| A second, forgiving code reader | `dock::readCode`, `DecodeResourceRequest` | Each 2 s dark ends a cycle; two cycles agreeing is the answer. The strict reader read the code once in three runs |
| The on-water tree | `task3_cannon_fire.xml` | Practice dock: ONE bay, NO fingers. `StrafeKeep` on `wall_range_node` plus the camera's face (`lateral_ref="face"`, `range_fallback="face"`), not `SlotKeep`. Approach bow-on, dock at 1.2 m (1.4 m was set live on 10-11), back off to 1.7 m to watch, move in to spray, back out to read the code |
| Step lines | `/crsd/task3_events`, "TASK3 \| ..." in the log | APPROACHING, DOCKING, DOCKED, WATCHING, ON FIRE, FIRE OUT, CODE |
| The launch reads the YAML | `task3.launch.py` | No `publish_setpoints` / `fire_pump` launch arguments any more |

`task3_cannon.xml` (your tree, with the slip) was not touched. Dominick wants to end up on
it; `task3_cannon_fire.xml` is what the practice dock allows today. Both must pass.

**Not in the repo:** the boat's own `crusader_params.yaml` carries Dominick's edits and was
never copied back. Known differences: `nozzle_x: 0.0` on the boat (Dominick says 0.0 is
right; the repo says 0.47) and `exit_speed_mps: 4.8` (the repo derives 4.85 from the throw).
Ask him which `nozzle_x` is true before building anything on it.

## Part A: the water cannon

### A1. Split the truth from the belief

Give the sim its own truth, in a sim-only file (say `crusader_sim/config/cannon_truth.yaml`),
defaulting to the belief so today's runs pass unchanged:

- true exit speed, and a drag term (the model is drag-free; real water is not)
- true pivot position (x, y, z)
- true servo zero and us-per-degree, pan and tilt
- true camera pitch and position (the aim point is measured in the camera frame)
- bias and noise on the aim point's depth
- pump spin-up (the first ~0.3 s of a burst is slower) and stream spread

### A2. Report the miss

While it sprays, publish where the stream crosses the window's plane relative to the window
centre: cm left/right and up/down. On the water this is Dominick's eye; in the sim it is
exact. Log it and show it in the referee's output.

### A3. A fit, instead of trims

Per-window trims are a fixed angle that patches the model at one range and one height. Two
windows needing different trims means the model is wrong, not the servo. Build a tool that
turns hits into model numbers:

- **In:** a list of hits. Each is the window's aim point (camera frame x, y, z), the pan and
  tilt that hit it (trims included), and roll/pitch.
- **Out:** exit speed, tilt zero offset, pan zero offset (camera pitch as an option), written
  to `crusader_params.yaml`; the four trims go back to 0.
- **Say what it cannot tell apart.** Two windows at one range fix two unknowns. A second
  range is the check.
- **It must run on boat data too.** Add a "mark hit" to `tools/scripts/task3/cannon_cal.py`
  that records the current solution, or parse the cannon log (`cannon: water ON | UL: pan ...
  tilt ... deg (trims ...), target ... m away ... m up, throw ... deg`).

### A4. Prove the fit in the sim

1. Inject errors into the truth: speed -15 %, tilt zero +5 deg, pan zero -3 deg, camera pitch
   2 deg, pivot 0.2 m off in x.
2. Do what Dominick does at the dock: trim until each window is hit, at two ranges.
3. Run the fit.
4. Spray at ranges and heights that were NOT used: 1.0 to 1.8 m, both windows, the deck
   raised and lowered 0.3 m (the "taller dock" question), roll and pitch up to 6 deg.
5. Report the miss table. Pass = within 5 cm of the centre of a 25 cm window everywhere.

### A5. What to measure carefully

A sensitivity table: cm of miss at 1.2 m and at 1.7 m per unit of each error (1 % of speed,
1 deg of servo zero, 1 deg of camera pitch, 5 cm of pivot, 5 cm of depth). It tells Dominick
which numbers deserve a tape measure and which do not matter.

### A6. Reach and view

- At 4.85 m/s, where can each window be reached without clipping? On 2026-10-11 the
  upper-left window was out of reach from the 1.7 m watch range (1.98 m from the nozzle,
  0.42 m up), which is why the tree moves in to spray. Tilt limits are -10..+60 deg in
  software, 1100-1900 us at the servo, centred on 1680: not symmetric.
- Where can the camera see each window? The upper-left is cut off at 1.2 m with a level
  camera. Redo your visibility table for the practice dock and say what camera tilt would
  let the boat watch, spray and read the code from ONE range.

### A7. While it moves

Continuous water while the hold drifts +-5 cm and the hull rocks: what fraction of the time
is the stream on the window? Include servo slew and SERVO_OUTPUT_RAW at 10 Hz.

### Real data so far (rough: by eye and protractor)

- One throw: about 9 ft (2.74 m) out at about 30 deg, landing on the water 0.51 m below the
  nozzle.
- Nozzle 11 in (0.279 m) left of the camera's centre, 0.75 m above the hull bottom.
- Tilt level at about 1680 us, reversed. Pan + = left (confirmed).
- With the OLD model (5.42 m/s, pivot 0.47, 0.30, 0.75) hits at about 1.0-1.2 m needed tilt
  trims of about +25 deg (upper-left) and +35 deg (lower-right). Earlier, from about 1.6 m,
  it was right side to side and about a foot low.
- With the NEW model and the upper-left trim still +25, it aimed too high. The boat's file
  later showed both tilt trims saved as 0. How it aims at 0 has not been reported yet.

## Part B: PID for heading, strafe and forward/back

### B1. Fit the sim's boat to the real one first

- **Ask Dominick for** the `MODEL_REQUEST.md` items (mass, thruster positions and angles,
  3S or 4S) and for logs: the tree's logs from the water (`/root/.ros/log/
  bt_runner_node_*.log` in the boat's `asv` container; every tick has range, lateral offset,
  squareness and the three stick commands in us) and the Pixhawk's dataflash logs.
- **Write a step test he can run in 10 minutes** in open water, MANUAL, through RC override
  (a tree, say `task3_sysid.xml`): forward +80 us for 4 s then coast 6 s, the same in
  reverse, the same sideways each way, yaw +-60 us; twice each. Log body velocity and yaw
  rate at 30 Hz.
- **Fit** mass, the surge/sway/yaw damping terms, the thrust curve (the T200 dead band
  included) and the thruster arms. Show sim against real, per axis.

Until that is done, say plainly that any gains are for the placeholder hull.

### B2. Make the loop's senses real

- The camera at the boat's real frame rate (take it from the logs; the code reader needs
  at least 4 fps), 45-100 ms late, with dropouts (the upper-left window drops out at
  1.2-1.4 m).
- `wall_range_node` at its real rate and noise; ATTITUDE at 30 Hz.
- Run at 1.0x real time (headless), or put the stack on sim time. At the GUI's 0.45x the
  hold is half as damped and tells you nothing about gains.

### B3. Disturb it

A steady current or wind of 0.1-0.3 m/s, wave rocking, and a 0.5 m shove.

### B4. Tune

The controller is `strafeKeep` / `axisLaw` in `crusader_bt/include/crusader_bt/fire_math.hpp`,
used by `StrafeKeep` and `SlotKeep`: per axis P + D + I, plus `min_us` (jump over the ESC
dead band), `max_us`, a slew limit, deadbands, `coast_s` and `brake_mps`. Units are
microseconds of RC override per metre, per m/s, per degree.

| | `task3_cannon_fire.xml` (water) | `SlotKeep` defaults (sim) |
|---|---|---|
| forward/back kp / kd / ki | 90 / 60 / 40 | 90 / 60 / 20 |
| strafe kp / kd / ki | 100 / 200 / 10 | 90 / 60 / 20 |
| heading kp / kd | 5 / 0.5 | 4 / 3 |
| `i_max_us`, `min_us`, `max_us`, `slew_us_s` | 100, 30, 120, 200 | 80, 30, 120 (the tree sets 90), 200 |
| deadbands range / lateral / yaw | 0.05 m / 0.04 m / 3 deg | see the tree |

All of them can be changed live from the ground station's Tuning tab as `strafe.*` on
`bt_runner_node` (-1 = the tree's value): `kp_fwd`, `kd_fwd`, `ki_fwd`, `kp_lat`, `kd_lat`,
`ki_lat`, `kp_yaw`, `kd_yaw`, `i_max_us`, `coast_s`, `lat_min_us`, `fire_range_m`,
`watch_range_m`, and the sideways estimator (`est_enable`, `est_q`, `est_r`, `track_s`; off
by default, and I have no record of it being compared on and off on the water).

- **Tests, per axis:** a 0.5 m range step, a 0.3 m sideways step, a 20 deg heading step;
  then the full hold under B3.
- **Measure:** rise time, overshoot, settling time, steady error, RMS error in the hold,
  stick chatter, and whether it hunts (your sim hunted +-11 cm until `brake_mps`).
- **Aim for:** range and lateral within 5 cm and heading within 3 deg in the hold; under
  10 cm of overshoot; back from a 0.5 m shove in under 8 s; never inside `min_range_m`.
- **Tune for the uncertainty, not the nominal boat.** Check the chosen gains with mass
  +-30 %, drag +-50 % and thrust +-30 %, and report the worst case.

### B5. The approach

`StrafeKeep`'s approach mode points the bow at the bay (yaw error = minus the camera's
bearing, no strafing) and slows as the bearing grows. On 2026-10-11 it came in pointing
somewhat right of the bay. Reproduce it with the camera's lag and bearing noise, then tune
`kp_yaw` / `kd_yaw` and the forward scaling.

### What was seen on the water

- The hold drifts 0.3-0.5 m and is sometimes slow to come back (2026-10-02).
- Pushed right, it did not come back left once the face's edge left the picture (fixed:
  a face cut on one side is still used).
- Docked and held well enough on 2026-10-11 to put the fire out in 15-28 s of spray.

## Added 2026-10-16: the practice slip

The team built a real slip (side walls and a back wall). A new tree,
`task3_cannon_slip.xml` (`task3.launch.py tree:=cannon_slip`), uses it:

1. The camera approach from `cannon_fire` brings the bow to the slip's mouth (2.4 m wall
   range).
2. `AwaitDockSlot` waits until `dock_slot_node` has had the whole slip for 1 s.
3. `SlotKeep` then centers, squares and holds 1.1 m from the back wall, backing to 1.4 m
   to watch.
4. If the slip is lost for 2 s, the camera approach takes over again (three tries).

`SlotKeep` gained `watch` / `watch_range_m`, and it now publishes the standoff it holds to
the gates (`strafe_target_range`), which `task3_cannon.xml` also goes through.

Before the water test, in Gazebo:

- **A practice-slip course** with the team's measured slip (width, wall length, wall height
  above the water; ask Dominick) and the dock face with its two windows. Run
  `cannon_slip` on it:
  - the hand-over at the mouth
  - straight in, and from 0.5 m off to one side at 20 deg
  - a 0.3 m shove while docked
  - the slip lost for 3 s (does the retry back out and come in again?)
- **Re-run `task3_cannon.xml`** on your four courses: `SlotKeep` changed.
- **The new fire and code steps** against the sim's lights: the 2.5 s RED confirm, and
  `readCode`'s trace lines.
- **Continuous water through SITL:** the pump output stays ON across the re-sent bursts,
  and the watchdog does not latch.

## Rules

- **One params file, two boats.** Sim-only values go in `gz_rig_up.sh` overrides or
  `crusader_sim/config`, as `pump_return_timeout_s` already does. Do not change the boat's
  values in `crusader_params.yaml` to make the sim pass.
- **Gains live in the tree files**, not the YAML. Deliver them for both trees.
- Stay on `task3-gazebo`. Nothing goes to `main` without the team's sign-off on node and
  parameter changes.
- Keep the tests passing: `python crusader_fcu/test/test_pump_core.py`,
  `python crusader_fcu/test/test_cannon_aim_core.py`,
  `g++ -std=c++17 -O2 -I crusader_bt/include -o t crusader_bt/test/test_dock_math.cpp && ./t`
  (175 checks), `test_fire_math.cpp` the same way, and the sim's own suite.
- Say what was run and what was not. A sim result is not a boat result.
- Dominick wants plain explanations, American spelling, and small steps he can test.

## What to hand back

1. The truth/belief split and the miss report, with your four courses still passing.
2. A practice-dock course (one bay, no fingers) that runs `task3_cannon_fire.xml` in Gazebo.
3. The fit tool, proven by A4, and the sensitivity table.
4. The reach-and-view table, with a recommended docking range, watch range and camera tilt.
5. The step-test tree and the model fit script (they need his data to finish).
6. Gains for both trees with before/after numbers, and how they hold up across B4's ranges.
7. A 15-minute checklist for the next water session: what to measure, in what order, and
   what result means the sim was right.

## Questions only Dominick can answer

- Is the nozzle really 0.0 m forward of the body origin, or 0.47?
- The boat's mass ready to run, the thruster positions and angles, 3S or 4S.
- Can he pull the tree logs and the Pixhawk logs from 2026-10-11?
- Will he tilt the camera up, and by how much?
- How does it aim now, with all four trims at 0?
