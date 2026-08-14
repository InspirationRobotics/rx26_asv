# G2 — LiDAR orientation and extrinsic sign check

**Purpose:** determine, by observation rather than by assumption, what the MID360 actually
emits — so every position derived from it afterwards is anchored to something measured.

**Nothing downstream of the LiDAR is trustworthy until this is signed off.** Clustering
against a mirrored frame produces confident, well-formed, wrong answers. It is not a
failure you notice by watching the numbers.

## Why this gate exists

`Resources.md` describes the LiDAR frame as **x forward, y left, z down**. That frame is
**left-handed**: with x forward and y left, the right-hand rule puts z *up*. A rigid sensor
rotated 180° about its forward axis — which is what "mounted upside down" means — gives
**x forward, y RIGHT, z down**.

So the written description and the physical mount disagree about the y axis, and exactly
one of these is true:

| Hypothesis | Raw `/livox/lidar` for a target on the **port bow** | `lidar_sign_y` |
|---|---|---|
| **A** — frame as written | y **positive** | `+1` |
| **B** — plain 180° roll about the forward axis | y **negative** | `-1` |

Getting it wrong swaps port and starboard for every object on the map. In Mission Task 1
that inverts every red/green pass-side decision — the boat drives confidently through the
wrong side of every gate.

> ## ✅ SETTLED — 2026-08-14: hypothesis **B**
>
> Observed on the boat: **starboard reads +y** in the raw frame. So the mount is the
> ordinary 180° roll about the forward axis — x forward, y right, z down, right-handed and
> physically consistent — and `Resources.md`'s original "y left, z down" was the impossible
> left-handed reading.
>
> **`lidar_sign_y = -1.0`, `lidar_sign_z = -1.0`.** Both negate into REP-103 body.
> `Resources.md` and the tool's fallback are updated to match.
>
> Re-run the rest of this procedure after any remount. The steps below stand as the method;
> the boxed values above are the current answer.

## Prerequisites

- [ ] Boat on the cart or at the dock, **stationary** (the viewer's `--accumulate` is not
      motion-compensated).
- [ ] livox container running; `ros2 topic hz /livox/lidar` shows ~10 Hz from inside `asv`.
- [ ] **Message type checked.** Run this first — it is the failure that actually happened:

      ```bash
      ros2 topic info -v /livox/lidar
      ```

      The driver publishes `sensor_msgs/PointCloud2` when `xfer_format: 0` and
      `livox_ros_driver2/CustomMsg` when `xfer_format: 1`. **ROS 2 matches nothing across
      types**, so a subscriber on the wrong one receives zero messages while `ros2 topic
      list` and `ros2 topic info` both look perfectly healthy — the type list simply shows
      two entries, one per endpoint, which is easy to read as "it supports both".
      `lidar_view` resolves the publisher's type before subscribing and logs which it
      found, so this should not bite twice.

      `xfer_format: 0` is preferred for the stack: PointCloud2 is the standard type, `asv`
      needs no livox package to deserialise it, rviz speaks it, and it decodes as a numpy
      stride view rather than a Python loop over 20k objects per sweep.

- [ ] QoS noted from the same command. A **RELIABLE subscriber matches a BEST_EFFORT
      publisher not at all**; the reverse (our BEST_EFFORT subscriber, a RELIABLE
      publisher) is compatible and fine.
- [ ] A distinct, isolated target ~1 m tall at **5 m on the port bow** — roughly 45° off the
      bow to the **left**. A traffic cone, a buoy, or a person standing still. It must be the
      only thing at that bearing; a target in front of a wall proves nothing.
- [ ] Tape measure. Record the actual placement: range ____ m, bearing ____° to port.

## Procedure

### 1. Look at the raw frame first

```bash
python3 tools/lidar_view.py --frame raw
```

Open `http://<JETSON_IP>:8081`. Axes are labelled `+x/+y/+z` with no boat semantics, because
in raw frame those words have no agreed meaning yet.

Find your target in the plan panel. Record which quadrant it lands in:

- **x positive** (upper half of the plan panel) — expected; the target is ahead.
- **y positive** (left half) → hypothesis **A**, `lidar_sign_y = +1`
- **y negative** (right half) → hypothesis **B**, `lidar_sign_y = -1`

Record: target appeared at plan quadrant ______________, so `lidar_sign_y` = ______

Now the elevation panel. The ground plane is the dense flat band.

- ground at **negative z** (below centre) → `lidar_sign_z = -1` (as shipped)
- ground at **positive z** → `lidar_sign_z = +1`

Record: ground plane at ______ z, so `lidar_sign_z` = ______

### 2. Confirm in body frame

```bash
python3 tools/lidar_view.py --frame body
```

With the correct signs, and **only** with the correct signs:

| Panel | Must be true |
|---|---|
| Plan | Target sits in the **upper-left** quadrant — forward of the boat marker and to its left, under the `PORT +y` label |
| Plan | Target's radius matches the tape measure against the range rings (5/10/15/20 m) |
| Elevation | The ground band sits **below** the cyan `LiDAR` marker, near the green `z=0 hull datum` line |
| Elevation | Nothing substantial above the target's real height — a mirrored z puts the whole world in the sky |

If the target lands upper-**right**, the y sign is wrong. Try it immediately without editing
config:

```bash
python3 tools/lidar_view.py --frame body --sign-y -1
```

If that fixes it, the boat is hypothesis **B** and `lidar_sign_y` must be changed to `-1.0`
in `crusader_bringup/config/crusader_params.yaml` (`lidar_cluster_node` section). It is
`[RO]`, so edit the file and restart the node — `ros2 param set` is rejected by design.

### 3. Check the translation

Stand the target at a tape-measured **10.0 m dead ahead** on the centreline.

- Plan panel: the target should sit on the vertical axis, on the 10 m ring.
- The extrinsic adds +0.32 m forward, so a target measured from the *geometry centre* reads
  ~10.0 m; measured from the *sensor* it reads ~10.32 m. Know which one you measured before
  calling a 30 cm discrepancy an error.

### 4. Record the water/ground plane

With the boat **floating**, measure the waterline height above the hull-bottom datum (the
surface the mounting heights were taken from). That number is `water_z`.

```bash
python3 tools/lidar_view.py --frame body --water-z 0.10
```

Adjust until the orange waterline sits on the dense water return band. Record: `water_z` =
______ m. This is the parameter the clustering filter uses to discard water returns; until
it is measured, that filter is guesswork.

## Results

| Item | Value | Confirmed by | Date |
|---|---|---|---|
| `lidar_sign_y` | **−1.0** | starboard reads +y in raw frame | 2026-08-14 |
| `lidar_sign_z` | **−1.0** | mounted upside down; ground below sensor | 2026-08-14 |
| Hypothesis | **B** (180° roll about forward) | | 2026-08-14 |
| `water_z` | ______ m | step 4 — **still outstanding** | |
| Range agrees with tape to | ______ m | step 3 — **still outstanding** | |

- Performed by: ____________
- `crusader_params.yaml` updated to match: **n/a until `lidar_cluster_node` exists**; the
  values above are carried in `tools/lidar_view.py`'s `FALLBACK_EXTRINSIC` and go into the
  params file when that node lands.

**`water_z` is the one number still blocking the clustering filter.** Without it, the
water-return rejection threshold is a guess, and on this mount most of the field of view is
water.

## Notes

**Expect few points on a small target at range.** The MID360 returns ~200k points/s spread
over 360°×59°, so a 0.3 m × 1 m buoy gives roughly 37 points per sweep at 5 m, 9 at 10 m,
and 2 at 20 m. Use `--accumulate 5` to hold five sweeps for a clearer picture at the bench.
Livox's non-repetitive scan pattern means each sweep adds genuinely new coverage.

**Upside-down mounting flips the vertical field of view** from `-7°..+52°` to `-52°..+7°`,
so the great majority of returns are ground or water. A plan view that looks like a solid
disc of returns with a few objects in it is normal, not a fault.
