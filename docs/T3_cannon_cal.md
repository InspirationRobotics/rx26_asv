# Task 3 cannon: aim calibration on the water

The short version: check the servos, measure the water's speed with one shot, then shoot
at the windows and trim until they hit. About 40 minutes, two people: the **pilot** on
the transmitter and the pump switch **SE (ch10)**, and the **operator** on the laptop.

Everything you change goes into a working copy of the params on the Jetson
(`~/robotx_ws/t3tools/cannon_cal.yaml`), not the boat's real file. `cal show` lists the
changes at the end.

## Before you go

1. On the Jetson host, get this branch and rebuild:
   ```bash
   cd ~/robotx_ws/src/rx26_asv && git fetch && git checkout task3-gazebo && bash tools/scripts/rebuild.sh
   ```
2. Restart `telemetry_bridge` (ground station, Nodes tab). Its log should say
   `cannon path: pan SERVO11, tilt SERVO12, 1100-1900 us`. Check it still shows the
   boat's mode and arming as usual.
3. The servos: pan on **AUX3**, tilt on **AUX4**, and the AUX rail powered.
4. Once per shell on the Jetson:
   ```bash
   alias cal='docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/cal.sh'
   ```
5. Bring: a tape measure, a board about 1 m × 1 m (plywood or foam board) you can stand
   upright, painter's tape.

## Safety

- **The water only comes from the pilot's switch SE.** The tool never fires the pump.
  **The e-stop (SB) does not stop the pump; SE does.**
- **The nozzle moves by itself** while a `cal` command runs. Keep faces and hands out of
  its line, and nobody between it and the target when SE could go on.
- **The boat is tied up and disarmed.** If the servos don't move while disarmed, press the
  Pixhawk's safety switch. Don't arm for this.

## 1. Start and check the servos (5 min)

```bash
cal node
```

It prints `cannon_aim_node up ... (pump DRY)`. Then:

| Command | The nozzle should | If not |
|---|---|---|
| `cal deg 0 0` | point straight ahead, level | find the PWM that does with `cal pwm <pan> <tilt>` (start at 1500 1500), then `cal set pan_center_us <it>` / `cal set tilt_center_us <it>` |
| `cal deg 30 0` | turn **LEFT** about 30° | `cal set pan_sign -1` |
| `cal deg 0 30` | point **UP** about 30° | `cal set tilt_sign -1` |

After any `cal set`, run `cal node` again.

**Is the nozzle where we assumed?** That's about 10 cm ahead of the camera's middle lens,
30 cm to its left and 10 cm above it. If it's more than ~5 cm off, set the real position.
The numbers are the lens's position plus your tape measurements:

```bash
cal set nozzle_x <0.37 + ahead>
```

```bash
cal set nozzle_y <0.0 + left>
```

```bash
cal set nozzle_z <0.65 + above>
```

Then `cal node`.

## 2. One shot for the water's speed (10 min)

**Setup.**
- Stand the board upright, facing the boat, **1.5 m in front of the nozzle**.
- Put a strip of tape on it **at the nozzle's height**.

**The shot.**
1. `cal deg 0 25` (straight ahead, 25° up).
2. Pilot: **SE on for about 1 s**.
3. Measure how far **above the tape** the stream hits, in metres (below = negative).
4. Then:

```bash
cal speed 1.5 <height> 25
```

It prints the speed and the `cal set exit_speed_mps ...` command. Run that, then
`cal node`.

Today's guess (5.42 m/s) predicts a hit about 0.25 m above the tape. Do this on the
battery you'll race with: the stream gets weaker as it drains.

## 3. Shoot the windows and trim (20 min)

**Setup.**
- The boat in a slip at the practice dock, the LiDAR about 1.6 m from the back wall
  (where the task fires from), tied.
- **dock_view** running (ground station, Nodes tab).

```bash
cal window lr
```

**Each shot:**
1. Wait for **ON TARGET**.
2. Pilot: SE about 1 s.
3. Look where the stream hits the lower-right window.
4. Ctrl-C.

| The stream hits | Fix (live, no restart; 1° ≈ 2.5 cm) |
|---|---|
| to the RIGHT | `cal set pan_trim_deg <bigger>` (+ = left) |
| to the LEFT | `cal set pan_trim_deg <smaller>` |
| LOW | `cal set tilt_trim_deg <bigger>` (+ = up) |
| HIGH | `cal set tilt_trim_deg <smaller>` |

Repeat until it hits the middle of the window. Then check the other one:

```bash
cal window ul
```

**If the upper-left misses while the lower-right hits,** the trims aren't the problem:
the water's speed is.
- Upper-left lands LOW: the real stream is slower. Lower `exit_speed_mps` by ~5%.
- Upper-left lands HIGH: raise it by ~5%.
- After either change, `cal node`, then shoot both windows again.

**No dock today?** Aim at a mark on the board instead. Give its position from the
camera's middle lens, in metres: forward, left (+) or right (−), up (+) or down (−).

```bash
cal point 1.6 -0.25 0.15
```

That point is roughly where the lower-right window would be.

**Done when** both windows get hit three times in a row.

## 4. Keep it

```bash
cal show
```

Copy every line marked `CHANGED` into `crusader_params.yaml`, in the `cannon_aim_node`
section, and commit it.

## If you have extra time

- Shoot from a bit closer and further (LiDAR ~1.3 m and ~2.0 m) to check it still hits.
  That's the slack in the hold.
- Rock the boat gently by hand while SE is on for ~3 s. The stream should stay on the
  window: the aim corrects for roll and pitch.
