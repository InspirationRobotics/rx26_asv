# What the sim needs from the team — and exactly how to hand it over

The sim runs today on a **placeholder Crusader**: a 1.0 × 0.6 m catamaran of
boxes, 30 kg, floating at the measured 0.24 m draft, with the four T200s where
ArduRover's OmniX mixer says they must be. Everything a guess is tagged `GUESS`
in [`config/crusader_hull.yaml`](config/crusader_hull.yaml). Each item below
replaces one guess. They are ordered by how much each changes what the sim
tells you.

**Nothing below needs code.** Every number goes into `crusader_hull.yaml` and
every mesh goes into `crusader_sim/meshes/`. The model regenerates on the next
`gz_sim_up.sh`.

---

## 1. Thruster geometry. Highest value, about 15 minutes with the CAD open

| Requirement | Why | What to send |
|---|---|---|
| Position of each T200 (centre of the prop) | Yaw and sway authority are thrust × lever arm. A wrong arm is a wrong turn rate in every mode. | x, y, z in metres from the **datum** (see box below), one row per thruster |
| Thrust axis of each T200 | OmniX assumes 45°. If yours are 30° or 60°, forward speed and sway speed trade off differently. | The angle from the bow line, degrees, **+ = the thrust points toward port** |
| Which way each one pushes at PWM > 1500 | The boat runs `SERVO1_REVERSED=1` and `SERVO4_REVERSED=1`. The sim assumes those two push *backward* along their axis above 1500 µs. That is the only way the params can drive correctly. | Bench test: boat on the stand, disarmed, **QGC → Motor Test**, one motor at a time, ~10 %. Write down "forward-left", "aft-right" etc. per SERVO output (1–4). |

> **The datum** is the one `Resources.md` and `crusader_params.yaml` already use.
> It's the geometric centre of the boat in plan view, at the **hull-bottom plane**,
> with x forward, y to port, z up. `lidar_x/y/z` and `cam_x/y/z` are measured
> from it, and the sim reads them from the params file automatically.

## 2. Mass and balance. 10 minutes, one bathroom scale

| Requirement | Why | What to send |
|---|---|---|
| **Total mass**, ready to run (battery in, everything on) | Acceleration, coasting and turn rate all scale with it. 30 kg is a guess. | kg |
| CG height above the hull bottom (optional) | Roll and pitch stability, and how much the sensors rock | metres. Onshape *Mass properties* is fine **if materials are assigned**; otherwise skip it |
| Battery: 3S or 4S | T200 thrust depends on voltage: about 35 N at 12 V, about 50 N at 16 V. The `crusader-battery` skill says the cell count is still unresolved. | "3S" or "4S" |

## 3. The hull mesh (Onshape). For looks and for docking contact

**Format: glTF binary (`.glb`) preferred, STL accepted.** Gazebo Harmonic reads
both. glTF keeps the colours, which matters because the camera renders the hull
into its own frames. STL comes out one colour.

Onshape steps:

1. Open the **Crusader assembly**, not a Part Studio: the export has to include
   the pontoons, deck, frame and mast together.
2. **No linking or merging is needed.** Crusader is one rigid body, so one mesh
   is exactly right. If some parts are *derived* from other documents, make sure
   the assembly is at the version you want (Onshape exports what the assembly
   shows).
3. **Hide what doesn't matter**: fasteners, wires, PCBs, the T200s themselves
   (the sim draws those), the LiDAR and the camera (the sim draws those at the
   params-file position). Fewer triangles means a faster sim in WSL; aim for under ~200k.
4. Right-click the assembly tab → **Export**:
   - Format: **GLTF** (binary `.glb`), or **STL** (binary, **units: meters**)
   - **Export as a single file**
   - If Onshape offers a coordinate-system / mate-connector option, put a
     **mate connector at the datum** (centre, hull-bottom plane, x forward,
     z up) and export relative to it.
     If it doesn't, **tell me where the datum is in the exported frame** (x, y, z of
     that point, and whether the model's +x is the bow). I'll put the offset in the
     YAML.
5. Send the file. It goes to `crusader_sim/meshes/crusader.glb`, and
   `hull.visual_mesh: "meshes/crusader.glb"` switches the sim to it.

**Optional, better buoyancy:** a second export of **only the pontoons/hull
skin**, as a watertight STL in the same frame. Right now buoyancy comes from boxes
sized to float at the measured draft, which gets the height right but not the
exact shape. With the hull skin I can size the buoyancy segments to the real
cross-section.

## 4. A 5-minute log for tuning the water. Makes the sim *move* like Crusader

The damping numbers in `crusader_hull.yaml` are sized for "2 m/s at 50 %
throttle", which is `CRUISE_SPEED`/`CRUISE_THROTTLE` from your params. That's
plausible, not measured. One calm-water session fixes it. Send the **dataflash
`.BIN`** (not just the tlog), with the boat in **MANUAL** throughout:

| Run | How | Gives |
|---|---|---|
| Straight line | throttle only, hold 30 %, then 50 %, then 70 %, ~10 s each | surge drag (`x_u`, `x_uu`) |
| Coast-down | from the 70 % run, sticks to centre, let it glide to a stop | surge drag, checked from the other side |
| Spin | steering only, ~50 %, 3–4 full turns | yaw drag (`n_r`, `n_rr`) |
| Crab | lateral only, ~50 %, 10 s each way | sway drag (`y_v`, `y_vv`); only an omni boat can do this |

`python3 -m crusader_sim.check_motion` then compares the sim's response to the
same inputs.

## 5. Task elements. Only if you have better than the handbook

The course models are built from handbook text: RoboBuoy panels, the new
2026-09-14 light beacon, the dock from `tools/task3_sim/world.py`. Several
handbook dimensions are **image only**, and the window size conflicts three ways
(25 cm square / 210 × 280 mm / world.py's 210 × 290). If the team has RoboNation's
**Box CAD files** or its own models of the RoboBuoy, the light beacon or the dock
structure, send them the same way as §3 (glTF/STL, metres, origin at the base
centre).

---

## Summary

| # | What | Format | Effort |
|---|---|---|---|
| 1 | Thruster positions, angles, push direction | numbers + a motor-test note | 15 min |
| 2 | Mass, battery cells, (CG) | numbers | 10 min |
| 3 | Hull mesh from Onshape | `.glb` (or binary STL in metres), one file, origin at the datum | 15 min |
| 4 | Tuning log | `.BIN` from a MANUAL session | 5 min on the water |
| 5 | Better task-element CAD | `.glb`/STL | only if it exists |
