# Task 3 in the Gazebo sim, with the water's commands

In the sim you run Task 3 with the **same commands as on the Jetson**: `t3 start`, `t3 go`,
`t3 steps`, `t3 log`, `slip`, `cal`. They run the same scripts
(`tools/scripts/task3/`), in the sim's container instead of the boat's. The only additions
are the things a sim has to stand in for:
- `tx`: the pilot's transmitter;
- `sim`: the boat being on the water, the weather and the referee.

## Once: install the commands

In an **Ubuntu-22.04** (WSL) terminal:

```bash
bash /mnt/c/Users/Owner/Downloads/college_stuff/robotx/boat/rx26_asv/crusader_sim/scripts/sim_aliases.sh --install
```

Then open a new Ubuntu terminal (or run `source ~/.bash_aliases`).

## Side by side

| Step | On the water | In the sim |
|---|---|---|
| The boat is on, in the water | power on (the stack starts itself) | `sim up` (course `task3_slip`), `sim up task3_slip_angle`, `sim up task3_wp` |
| The transmitter, all of it | the RadioMaster | `tx keys` (a keyboard transmitter, below) |
| Arm | the pilot, SB | `tx arm` |
| MANUAL | the pilot, SC | `tx manual` |
| GUIDED | QGC or the ground station | `tx guided` (or QGC) |
| Load a GPS waypoint | QGC: Plan view, one waypoint, Upload | QGC the same way, or `sim wp right` |
| Start the Task 3 launch (leave it running) | `t3 start` | `t3 start` |
| Start a run | `t3 go` | `t3 go` |
| Start a run from the waypoint (GUIDED first) | `t3 go wp` | `t3 go wp` |
| Watch the steps | `t3 steps` | `t3 steps` |
| The tree's log | `t3 log` | `t3 log` |
| What the LiDAR makes of the slip | `slip` | `slip` |
| RoboCommand's log (each report and the reply) | RoboCommand's own screen | `sim robocommand` |
| The light sequence (ARMING, FIRE, GREEN, GAP, CODE) | look at the dock | `sim lights` |
| Both of those, live | | `sim log` |
| Take the boat back | SC out of MANUAL; SD up; SB e-stop | `tx hold`; `tx drop on`; `tx estop on` |
| Wind, current | the weather | `sim wind 6 90`, `sim current 0.2 0`, `sim weather rough`, `sim calm` |
| The cannon's aim alone | `cal window lr` at the dock | `sim aimtest lr` (and `ul`) |
| Did it pass? | RoboCommand | `sim verdict` |
| Off | | `sim down` |

**Three terminals, as on the boat:**
1. `t3 start`, left running.
2. `t3 steps`, to watch. (A fourth, with `sim log`, shows the referee's side.)
3. Everything else: `tx ...`, `t3 go`, `sim ...`. Or `tx keys` here, and the rest in another one.

## The keyboard transmitter: `tx keys`

```bash
tx keys
```

It's a full-screen transmitter. Each key is a switch on the RadioMaster and goes into the
autopilot as a real RC channel, so it acts as the boat's switches do:

| Key | Switch | What it does |
|---|---|---|
| `m` / `h` / `a` | SC | MANUAL / HOLD / AUTO |
| `g` | (the ground station) | GUIDED |
| `d` | SD | the autonomy-drop latch, on/off: on = the tree lets go of the boat |
| `e` | SB | the motors' e-stop, on/off (motors only: the pump stops through telemetry_bridge's pump watchdog, as on the boat) |
| `p` | SE | the pump, on/off |
| `r` / `x` | | arm / disarm |
| arrows | sticks | throttle (up/down) and steer (left/right), 25 µs a press |
| `z` / `c` | stick | slide left / right |
| space | | all sticks back to centre |
| `q` | | quit (the sticks centre; the switches stay where they are) |

The bottom line shows the autopilot's side: the flight mode, armed, and whether the drop
latch has tripped. The single-shot commands (`tx manual`, `tx drop on`, ...) do the same
things from any terminal.

The Gazebo window opens with `sim up`, on a view of the dock you can zoom and turn.

## A run, start to finish

```bash
sim up task3_slip_angle
```

Wait for `=== ready ===` (about 2 minutes), then:

```bash
tx arm
```

```bash
tx manual
```

```bash
t3 start
```

In a second terminal:

```bash
t3 steps
```

In a third terminal:

```bash
t3 go
```

You'll see APPROACHING, IN THE SLIP, DOCKING, DOCKED, WATCHING, ON FIRE, MOVING IN,
FIRE OUT, BACKING OUT, CODE. Then:

```bash
sim verdict
```

To run again from the start: `sim down`, then `sim up ...`. The boat goes back to its
starting spot.

## A run from a GPS waypoint (GUIDED, then the search for GREEN)

This is the run with the GUIDED leg first:
1. The boat drives to a waypoint you load, in GUIDED.
2. At the waypoint the TREE switches to MANUAL itself. Leave the transmitter as it is. SD
   (the drop latch) still takes the boat back, and so does moving SC.
3. It slides sideways along the dock until the camera has the GREEN indicator in view.
4. Then it carries on as above (approach, slip, dock at 1.25 m, fire, code).

Why MANUAL: ArduPilot's GUIDED can't strafe on this boat, so everything after the waypoint
is RC override in MANUAL, as `dp_hold` did.

The course `task3_wp` starts the boat 20 m south of the dock, and **the GREEN bay is
random** at every `sim up`.

```bash
sim up task3_wp
```

Wait for `=== ready ===`, then:

```bash
tx arm
```

```bash
tx guided
```

Now load **one** waypoint, about 5 m off the dock. Either:
- **QGC** (it connects to the sim by itself): Plan view, click one waypoint in front of the
  dock, Upload. Or:
- the same upload from a terminal:

```bash
sim wp right
```

`sim wp left`, `sim wp middle`, `sim wp right 3` (3 m off instead of 5), or
`sim wp LAT LON` for anywhere. QGC shows the uploaded waypoint either way.

```bash
t3 start
```

In another terminal:

```bash
t3 go wp
```

`t3 go wp` reads the waypoint back off the autopilot (the first one in the mission) and
puts it in the goal. The steps:

- `TO THE WAYPOINT in GUIDED`: the boat drives there. You can watch it in QGC.
- `AT THE WAYPOINT: SWITCHING TO MANUAL`: the tree asks the autopilot for MANUAL itself
  (`/crsd/set_mode`, which telemetry_bridge already takes on the boat). It only switches
  from GUIDED, never from a mode the pilot chose.
- `SEARCHING for the GREEN bay, sliding along the dock`: if the camera can't already tell
  which bay is GREEN, the boat slides left up to 4 m with the bow held, then back to 4 m
  right of where it started. It stops as soon as it has the GREEN indicator and the two RED
  ones, and commits that bay (`committed to bay N` in `t3 log`).
- Often it already knows: the camera sees the whole dock on the way in, so it commits at
  once, with no slide.
- `SLIDING to bring the GREEN bay into view`: if the committed bay is out of the camera's
  picture now (e.g. the waypoint is in front of another bay), the boat slides toward it
  until it's within 20° of the bow. The camera approach can only steer on a bay it can see.
- Then APPROACHING, IN THE SLIP, DOCKING ... CODE, as in the run above.

Which bay was GREEN this time: `sim verdict`, or `cat ~/.cache/crusader_sim/random_choices.txt`.
To force one for a repeatable test:

```bash
SIM_GREEN_BAY=1 sim up task3_wp
```

**Keep the waypoint at least 5 m off the faces.** The fingers stand 2 m out, and the GUIDED
leg coasts about 0.7 m past the waypoint. From 3 m out, the bow ended up on a finger tip and
the boat was stuck there.

To see a slide, put GREEN on the far bay from the waypoint:

```bash
SIM_GREEN_BAY=1 sim up task3_wp
```

then `sim wp right`. Bay 1 is then about 43° off the bow, outside the 82° view, so the boat
slides left until it's within 20°.

`sim up` also sets `WP_RADIUS` to 0.3, as for a Task 3 GUIDED approach on the water. With
the boat's 2.0, GUIDED stops about 2 m short of the waypoint.

## The one-click run: `TASK3_SIM.cmd`

Double-click `crusader_sim\scripts\TASK3_SIM.cmd` (course `task3`, GREEN bay random). It
brings the sim up, waits 20 s, and runs the whole of Task 3 by itself with the sim's own
tree, `task3_cannon_sim.xml` (the water's is `task3_cannon_slip.xml`; they're kept apart):

1. `TO THE WAYPOINT in GUIDED`: 5 m off the middle bay's face.
2. `SWITCHING TO MANUAL`: the tree's own switch.
3. `SEARCHING` (if the camera doesn't have the GREEN bay yet) and `LINING UP on the GREEN
   bay`: it slides along the dock, the bow square to it and at least 3.5 m out, at 0.2 m/s
   whatever the wind and current do, until the boat is lined up on the GREEN bay (within 0.4 m of its centreline, the bow within 10° of square).
4. `APPROACHING the bay, square to it`: on the camera, strafing onto the bay's centreline
   all the way in.
5. `IN THE SLIP`, `DOCKING` at 1.25 m (LiDAR to the back wall), `DOCKED`: the docking
   report.
6. `WATCHING` from 1.7 m, `SPRAYING` from there (no trims: the aim is calculated), the
   firefighting report, the code, the resource request: RoboCommand's PASS. From 1.7 m
   because a camera ~0.22 m above the water sees half of the upper-left window only from
   ~1.65 m, and the water reaches it only to ~1.82 m.

The old all-GUIDED tree (`task3_cannon.xml`) circled any bay but the middle one, because
GUIDED can't strafe. It's still there for comparison: `TREE=task3_cannon.xml` before the
bring-up.

**With your own QGC waypoint:** after the bring-up, upload one waypoint in QGC (or
`sim wp left`), then in an Ubuntu terminal:

```bash
bash /mnt/c/Users/Owner/Downloads/college_stuff/robotx/boat/rx26_asv/crusader_sim/scripts/gz_task3.sh task3 --wp
```

## The cannon's aim test: `sim aimtest`

```bash
sim aimtest lr
```

This tests the aim on its own, with no trims. It brings up `task3_aim_lr` (about 3 min)
and pins the boat in front of the middle bay at a grid of places and headings: the LiDAR
1.25, 1.6 and 1.9 m from the back wall, centred and ±8 cm off the centreline, square and
±6°. At each spot it fires at the lower-right window and reads where the water crosses the
face, against the window's centre. `sim aimtest ul` does the upper-left from 1.7 and 1.75 m.
`--quick` does the centreline only.

Each line shows the miss (right/high, cm), and under it where the miss comes from:
- the aim point against the true window centre (forward, left, up, cm);
- the commanded pan and tilt against the ideal ones;
- the servos' actual angles against the command;
- the hull's roll and pitch, as the autopilot says and as Gazebo has it.

The table is also written to `/tmp/aim_test.csv` in the container.

What it can't show: the sim's water is the aim's own model (the same exit speed), so it
tests the AIM (camera, LiDAR, geometry, servos, the hull's attitude), not the stream. The
stream's real speed needs one `cal speed` shot on the water.

Results (2026-10-18, no trims, camera 0.225 m above the water):

| Window | Poses | Within ±5 cm | Worst | RMS |
|---|---|---|---|---|
| lower-right | 27 (LiDAR 1.25 / 1.6 / 1.9 m × ±8 cm × ±6°) | **27** | 1.2 cm | 0.7 cm |
| upper-left | 18 (LiDAR 1.7 / 1.75 m × ±8 cm × ±6°) | **17** | 2.0 cm | 0.9 cm |

The one upper-left miss was out of reach, so the cannon correctly refused to fire. The
upper-left only works in a narrow band: half of it is in the picture from ~1.65 m, and the
water reaches it only to ~1.78 m. The lower-right is in view from ~1.2 m and in reach to
~2.1 m.

Two things the first runs taught (both fixed):
- **The hull's attitude matters more than the camera.** Pinning the boat level fought the
  hull's own small rock (the cannon swinging moves it), so the autopilot's roll/pitch were
  a blur and the aim corrected for a tilt the water never saw: misses up to 5.7 cm. The
  pinner now keeps the hull's own roll and pitch.
- **One camera frame is noisy (~2 cm):** the aim point is now the median of the last
  0.4 s of frames (`aim_smooth_s`).

## Courses

| Course | Start |
|---|---|
| `task3_slip` (the default) | 4 m out, lined up with the green bay's slip (bay 1, on the left) |
| `task3_slip_angle` | 5.3 m out, 20° off the slip's line, bow turned so the green indicator is just out of the camera's view |
| `task3_wp` | 20 m south of the dock, facing it; **GREEN bay random**; for `t3 go wp` (above) |
| `task3`, `task3_bay1`, `task3_bay3`, `task3_ul` | the one-click run (`TASK3_SIM.cmd`, below). `task3`'s GREEN bay is random too |
| `task3_aim_lr`, `task3_aim_ul` | the cannon's aim test (`sim aimtest`, below) |

The practice slip on the water is ONE bay. For that, start the launch with
`t3 start slip dock_bays:=1`: the search then takes a lone GREEN indicator.

The sim's dock is the course's: three slips, each 1.5 m wide with 2 m walls, 0.3 m above the
water. The practice slip is one bay, with walls about 0.4 m above the water.

## Weather

`sim wind M/S TOWARD`, `sim current M/S TOWARD`, `sim calm`, `sim weather`:
- **TOWARD** is the compass direction the air or water moves toward. On the Task 3 courses the
  slips open south, so **0 pushes the boat into the slip, 180 out of it, 90 and 270 across
  it.**
- **Wind** is a force on the hull with gusts (±30% by default; `--gust 0.5` for more). As a
  rough guide: 6 m/s is about 11 N, 10 m/s about 31 N.
- **Current** moves the water itself, so the boat drifts with it and holding still costs
  thrust.
- **Variable** (2026-10-18): `sim wind 6 90 --swing 30` makes the wind's direction wander
  ±30° (slowly, over ~15 s); `sim current 0.15 0 --vary 0.5 --swing 40` makes the
  current's speed wander ±50% and its direction ±40° (over ~20 s).
- **Presets:** `sim weather rough` (wind 6 m/s across, gusts ±40%, swinging ±30°, plus a
  current 0.15 m/s into the dock, ±50%, swinging ±40°) and `sim weather light` (3 m/s
  across and 0.05 m/s in, both varying).
- `sim weather` shows what it's set to and what it is right now; the actual values also go
  to `/tmp/weather.log` every 5 s.

A hard one to try:

```bash
sim weather rough
```

That's a gusty, swinging crosswind plus a wandering current pushing into the slip.

The one-click run, every GREEN bay in every weather (2026-10-18, `task3_cannon_sim.xml`):

| GREEN bay | calm | wind (6 m/s across, gusts ±40%, swinging ±30°) | current (0.15 m/s in, ±50%, swinging ±40°) | `rough` (both) |
|---|---|---|---|---|
| 1 | PASS | PASS | PASS | PASS |
| 2 | PASS | PASS | PASS | PASS |
| 3 | PASS | PASS | PASS | PASS |

PASS is RoboCommand's: docked in the GREEN bay, the fire out, the right window reported,
the code's request, the UAV tasked, no contacts.

Two fixes it took (the first sweep was 10 of 12):
- **Lined up means lined up.** Gusts had the boat turning at the waypoint, and "the GREEN
  bay in the middle of the picture" came true while it turned, with the bay still 2–3 m
  to the side. The slide now ends only when the boat is on the bay's centreline (0.4 m)
  and square (10°).
- **Centre first.** It went into a slip 0.5 m off the middle and stuck on a finger. The
  slip hold now won't move in while it's more than 0.2 m off the centreline (0.25 m in the
  water's tree); it slides over first.

Before these (2026-10-17, the water's tree with its bow-pointing approach): a 6 m/s
crosswind plus a 0.15 m/s current into the dock failed on the approach. A current pushing
into the dock is still the case to watch on the water.

## What the sim is not

- **The camera:** the dock detector is a geometric stand-in for the CV team's model. It sees
  what a camera there would see, with their measured error rates, but it's not YOLO on
  images.
- **The boat:** a 60 kg estimate on a placeholder hull. The masses, drag and thrusters are
  guesses until they're fitted to a log from the water (`docs/T3_sim_calibration_handoff.md`,
  Part B).
- **What `sim:=true` changes:** the tree may move the simulated boat, the cannon may ask
  for the (simulated) water, and there's no `dock_view`. On the boat those come from
  `crusader_params.yaml` as always.
