# `crusader_world_model` — fusion, tracking and the occupancy grid

Per-sensor detections in, one consistent picture of the world out. In the architecture
diagram this is the pair of boxes drawn *outside* the Perception group, because it belongs
to neither the sensors nor the missions.

**State: two nodes, neither has run on the boat.** Both are excluded from `core.launch.py`
and both carry a `NOTE:` at the top of the file. The occupancy grid is still unwritten.

| Node | Job | State |
|---|---|---|
| `target_tracker` | Camera + LiDAR + pose → earth-anchored target tracks on `crsd/world_targets` | bench only |
| `map_server` | Those tracks and the vessel state, live in a laptop browser on `:8082` | bench only |
| *(occupancy grid)* | World-anchored obstacle map with time decay | not written |

## Why this is a separate package from `crusader_perception`

Because it is testable and perception isn't. Everything here is geometry and time decay:
it can be exercised with invented detections, on a laptop, with no camera, no GPU and no
boat. Perception can't — it needs a model, a calibration and a scene.

That is not an aspiration. `target_tracker_core.py` and `map_server_core.py` have **no ROS
imports at all**, and [`tools/bench/bench_world_model.py`](../tools/bench/README.md) drives
the whole chain from an invented buoy field. If a change here can only be checked on the
water, it is in the wrong file.

## `target_tracker` — three stages

```
oak/detections (camera_link) ─┐
                              ├─ 1. FUSE (body) ─ 2. PROJECT (world) ─ 3. TRACK ─→ crsd/world_targets
crsd/lidar_clusters (base_link)┘         ▲                    ▲
      counted, then dropped     /crsd/attitude ──────── /crsd/pose
      unless use_lidar
```

**`use_lidar` is `false` by default, so the camera is the only thing that can create a
track.** Read [the switch](#the-switch-use_lidar) before anything below about fusion — five
of the six knobs in stage 1 are inert while it is off.

**1. Fuse.** The camera knows *where to look* and *what it is*: its bearing is an angle read
off a rectified image, good to a fraction of a degree. The LiDAR knows *how far*: a
time-of-flight range, centimetres, against stereo disparity that is metres out past 15 m.
So fusion is not an average — it keeps the camera's **direction** and the LiDAR's
**distance** and throws away each sensor's weak number. Both inputs are body-frame already,
so this stage has no attitude in it.

With `use_lidar: false` this stage is a pass-through: camera detections above
`min_confidence` become sightings with their own stereo range, clusters are counted into
`lidar_ignored` and discarded, and nothing else in the stage runs.

**2. Project.** One call to `geo.body_to_world_ypr` per sighting. This is the only place
attitude enters, and it is not a refinement: at 20 m, 5° of uncompensated roll moves a
target 1.7 m, which is most of a buoy gate.

**3. Track.** Nearest-neighbour association in the world frame, an EMA whose gain decays as
`1/hits`, and two decay timeouts. This is what turns a stream of sightings into an object
that persists while the boat looks away — the thing a mission actually needs.

### The switch: `use_lidar`

Default **`false`**. The question it answers is not "which sensor is better" — it is **what
an unlabelled cluster means in the water you are actually in.**

On open water, the nearest hard surface in range is the object you wanted to see. In a pool,
or alongside a dock, it is the wall. The wall clusters beautifully. It arrives with no
label, survives `track_unlabeled`, clears `confirm_hits` in three sightings, and — with
`track_timeout_s` at `0` — never leaves the map. `crusader_params.yaml` records the
measurement next to `lidar_cluster_node.r_max`: at 40 m the map filled with **100+ permanent
tracks at 15–28 m**, which were the pool walls, the deck and the room beyond them. Cutting
`r_max` to 10 m took clusters per cycle from 60+ to about 10 — better, and still ten
buildings a mission has to treat as buoys.

None of that is a tracker bug. It is the LiDAR honestly reporting a room, and a consumer of
`crsd/world_targets` has no way to tell that object from a buoy: `bt_runner_node` skips
tentative tracks and takes every confirmed one, so a confirmed wall is a confirmed obstacle
in the Task 1 tree.

**What it costs.** Every range becomes stereo — roughly 6 % systematic bias plus 4 % random,
so a buoy at 25 m sits about 1.5 m beyond truth and wanders by a metre. Inside a pool that
is invisible; at competition ranges it is most of a gate. The never-drop invariant narrows
with it: what is never dropped is every *camera* detection, and an object the camera cannot
see is no longer carried by the LiDAR.

**So turn it on for open water**, together with `lidar_cluster_node.r_max` back at 25–40. It
is `[DYN]` precisely so that is a checkbox in the ground station's Tuning tab and not a
redeploy — and unlike the extrinsic, flipping it invalidates nothing already on the map:
camera-derived targets stay exactly as true as they were.

**How to see it working.** `crsd/world_model_health` carries `lidar_in` and `lidar_ignored`
side by side, so "10 arriving, 10 discarded" is a configuration and "0 arriving" is a dead
sensor — the node also says so on `/rosout` once a minute rather than leaving a healthy,
connected, contributing-nothing LiDAR invisible.

### Remembering the course

`track_timeout_s: 0` (the default) means a **confirmed track never expires**: the boat
remembers the whole field for the run, and a buoy re-entering the camera's FOV updates the
*original* track instead of spawning a second one beside it. Set a positive number of
seconds to go back to forgetting.

Tentative tracks still expire on `tentative_timeout_s`, always — that guard cannot be
switched off, because without it one wave crest that clears `confirm_hits` would sit on the
map permanently. **With immortal tracks, `confirm_hits` is the only thing between a
reflection and a permanent phantom.** Raise it if phantoms accumulate.

The knob that decides whether a re-sighting *lands* on the remembered track is
`assoc_radius_m`: the new observation must fall within it of the stored position. At the
3.0 m default a re-sighting up to 2.9 m off re-associates and 3.5 m splits into a second
track. If returning to a buoy reliably produces a duplicate a few metres from the original,
that is this gate, not the memory.

### Why not a Kalman filter

The targets are mostly **static** objects seen from a moving platform whose own position is
RTK GPS. The dominant error is not process noise to be modelled, it is **misassociation** —
feeding one track sightings of two different buoys — and no amount of filter sophistication
fixes a wrong association. So the effort goes into the gate (a hard radius **plus** label
compatibility) and into making a bad gate visible (`position_stddev` grows loudly when one
track is being fed two objects) rather than into covariance propagation.

### The invariants this node must keep

1. **A detection with no supporting range from a second sensor is passed through with what
   it has — never dropped.** Losing an obstacle is worse than carrying a coarse range for
   it. With `use_lidar` on this cuts both ways: an unlabelled LiDAR cluster is a thing that
   is *there* and reaches the map with an empty label rather than being filtered for being
   anonymous. **`use_lidar: false` narrows it to the camera** — every camera detection above
   `min_confidence` still reaches the map with whatever range it has, and the LiDAR no longer
   contributes an object of its own. That is a deliberate scope change, taken once at the top
   of `fuse()` and counted in `lidar_ignored`; it is not a detection quietly failing a gate,
   which is what this invariant exists to forbid.
2. **Two different labels never merge**, however close. A red buoy and a green buoy 2 m
   apart are a gate; averaging them into one object at the midpoint puts a waypoint through
   the middle of nothing.
3. **Pose or attitude stale → no observations are ingested at all.** There is no honest
   place to put a detection when the boat's own position is unknown, and guessing writes
   targets at coordinates the boat has already left. Existing tracks still age and still
   expire; this is the one case where doing nothing is the correct action.
4. **Published every tick, empty or not.** Silence means the node is dead, not that the
   water is clear.
5. **Track ids are never reused**, not even after a track is dropped, so a consumer that
   stored "target 7" cannot have that reference silently re-pointed at a different buoy.

### The parameters that matter most

`use_lidar` (default `false`) decides whether anything anonymous can reach the map at all,
and is the first thing to check when the map is crowded. See [the switch](#the-switch-use_lidar).

Then `assoc_radius_m` (default 3.0). Too wide merges a gate pair into one object at the midpoint;
too narrow splits one buoy into a new track every few seconds. Watch `position_stddev` on
the map — small and steady is a solid fix, growing means the gate is wrong.

It matters more in camera-only mode, not less: with no LiDAR range, a target's distance
carries the full stereo error, so a buoy at 25 m can re-project a metre or more along the
bearing ray between sightings and split into a second track. At pool ranges that noise is
~0.4 m and 3.0 m is comfortable; if duplicates appear strung out along a bearing at long
range, this gate is the one to widen.

Then `confirm_hits` (default 3). With `track_timeout_s: 0` it is **the only thing between a
detector false-positive and a permanent phantom.** Raise it if phantoms accumulate even with
the LiDAR off.

`fuse_bearing_deg` is inert while `use_lidar` is false. When it is on, it is loose at 6.0° to
absorb the **unmeasured camera extrinsic** (see below); tighten it once `cam_*` are real
numbers.

## The camera extrinsic: translation measured, orientation assumed

From `Resources.md`'s mounting section: the OAK-D is **37 cm forward, on the centreline,
65 cm up** — against the same hull-bottom datum as `lidar_x/y/z` (0.32 / 0.05 port / 0.52).
That shared datum is what makes a camera z and a LiDAR z of the same buoy agree; with
`cam_z` left at zero they would have disagreed by 65 cm and z would have stopped being the
sanity check this package leans on.

**The two mount angles are still an assumption.** Resources.md records the camera as "already
REP-103; NOT yet bench-confirmed the way the LiDAR was", so `cam_yaw_deg` and `cam_pitch_deg`
are zero because the camera is *believed* to point dead ahead and dead level. That is
precisely the kind of belief that had the LiDAR's y-sign recorded as "left" — a left-handed
frame that could not describe a rigid sensor — until [docs/G2](../docs/G2_lidar_orientation.md)
settled it on the bench. Run the same check for the camera and the angles stop being a guess.

Until then `fuse_bearing_deg` stays wide at 6°. The translations no longer justify that: the
two sensors are 5 cm apart in both x and y, which is 0.3° of parallax at 10 m. It is the
unconfirmed **angles** holding the gate open, and it should come down toward 2–3° once they
are confirmed.

The tracker says this out loud: if both sensors are producing and *nothing* fuses, it logs
that every target is being tracked twice and names the mount angles as the suspect.

## `map_server` — the display

```bash
ros2 run crusader_world_model map_server
# then, from the laptop:  http://<JETSON_IP>:8082
```

Vessel position, heading, speed, roll/pitch, mode and armed state as text; a north-up plan
map with the boat as a heading arrow, its wake, range rings centred on the boat, and every
tracked target coloured by its label. Pan, zoom, follow-boat toggle.

**It sends data, not pixels.** `tools/oak_view.py` and `tools/lidar_view.py` ship MJPEG
because their subject *is* an image. A map is a boat position and a few dozen targets —
well under a kilobyte — so the page is HTML+canvas polling a JSON endpoint. The Jetson
spends no CPU drawing or encoding, the link carries ~1 KB per update instead of ~100 KB,
the laptop can zoom without a round trip, and the readouts are selectable text. The shared
rule survives either way: **the laptop needs a browser and nothing else.** No ROS on
Windows, no rviz2, no X forwarding.

**Port 8082**, because 8080 is `buoy_detector`'s annotated view and `tools/oak_view.py`, and
8081 is `tools/lidar_view.py`. All three are useful at once — they are three different
questions about one moment.

### What it costs on the radio link

Measured on the real `/state` path, at the 5 Hz default poll, **per connected browser**:

| Scenario | `/state` | Per client |
|---|---|---|
| Idle at the dock | 0.3 KiB | 0.02 Mbps |
| Typical run, 6 targets, trail saturated | 12.2 KiB | 0.51 Mbps |
| Busy course, 20 targets | 15.5 KiB | 0.64 Mbps |
| Worst case, `max_tracks: 64` | 25.5 KiB | 1.05 Mbps |

Plus 13 KiB once, when the page loads. Against a 150 Mbps link that is 0.34% typical and
0.7% worst case — but the number worth remembering is the **0.5 Mbps**, not the percentage,
because a WiFi link's rated speed is a close-range PHY rate and what matters is what is
left at the far end of a course.

**The trail is 86% of every poll** — 600 points of unchanged history, resent five times a
second. `trail_length` is `[DYN]`, so it is the one knob that moves this number without a
restart: halving it roughly halves the bandwidth. Sending only new points would be the real
fix and has not been done, because 0.5 Mbps has not yet been worth the protocol.

The page holds **one poll in flight at a time**. `setInterval` fires on a wall clock and
does not care whether the last request returned, so on a link that has gone slow the
requests pile up faster than they drain, hit the browser's per-host connection cap, and
freeze the map on stale data while the banner that should be warning about it waits behind
the queue. Skipping a tick instead costs one frame of a 5 Hz display and keeps the request
rate matched to what the link can carry — measured at 4 requests rather than 20 across four
seconds of a two-second-latency link.

**It has no publishers, no services and no timers that touch anything but its own trail
buffer**, so it can be started and killed at any point in a session, including under way.
A display that can affect the vehicle is a display nobody dares restart when it misbehaves.

**Staleness is the feature.** A dead pose does not freeze the boat marker at its last
position and keep looking healthy — the marker greys out, the readout goes red, and a
banner says so. A moving map is the most convincing thing on a screen, and a convincing map
of a lie is worse than a blank one.

It anchors its own display origin rather than reusing the tracker's, and re-projects targets
from the lat/lon they carry. The two nodes start at different moments, so their origins
differ; lat/lon is the frame they genuinely share, and metres are always relative to
somebody's choice.

## Verifying it before the sensors are trustworthy

```bash
python3 tools/bench/bench_world_model.py
```

An invented buoy field around the **real** boat: it subscribes to `/crsd/pose` and
`/crsd/attitude` and publishes only `oak/detections` and `crsd/lidar_clusters`, computing
what the two sensors would have reported. Real RTK noise, real moving-baseline yaw with its
real latency, real hull motion — a simulated boat moves perfectly, which is the one kind
this tracker will never see. It does not contend with `telemetry_bridge` for `/crsd/pose`.

The field is anchored at the first fix and rotated to the heading at that instant, so the
buoys land **ahead of the bow** wherever the boat is. **It prints the ground truth when it
anchors** — that is what makes it a check rather than a demo.

`--sim-pose` invents the vessel too, for a desk run with no boat; `--no-camera` and
`--no-lidar` force the single-sensor paths.

**What it cannot prove:** the bench builds detections with `geo.world_to_body_ypr`, the exact
transpose of the transform the tracker runs, so a sign error shared by both cancels and the
map looks perfect anyway. It proves plumbing, association, decay, fusion arbitration and the
display. It does **not** prove the frame convention — that is a bench exercise against real
hardware, the way [docs/G2](../docs/G2_lidar_orientation.md) did it for the LiDAR.

## If the map fills up with anonymous targets, check `use_lidar` first

With `use_lidar: false` — the default — this cannot happen: no cluster becomes an
observation, so every track on the map has a label from the camera. A map full of
unlabelled targets means the switch is on. The rest of this section is what to do about it
when you have deliberately turned it on.


`lidar_cluster_node`'s `water_z` is still the 0.10 m placeholder — the waterline above the
hull datum, to be measured floating (G2 step 4). Until it is, water returns can survive the
water gate, cluster, and arrive here as perfectly good unlabelled `Cluster3D`s. This node
cannot tell them from an anonymous obstacle and, per its never-drop invariant, will happily
track them.

So a map crowded with unlabelled targets at short range is a **`water_z` symptom, not a
tracker bug** — check `crsd/lidar_cluster_health`'s `n_water` first. `track_unlabeled: false`
suppresses them, but only as a temporary measure while `water_z` is wrong: it also throws
away every genuine obstacle the camera cannot name, which is the exact trade the package
README forbids making permanently.

`water_z` has since been measured at 0.24 m (2026-09-05), which closes the placeholder half
of this. What it does not close is a hard surface *above* the waterline — a wall, a dock,
a hull — which is a real return at a real height and is exactly what `use_lidar` is for.

## What is still missing

- **The occupancy grid.** `occupancy_core.py` and `occupancy_grid_node.py` were removed
  unverified in v0.5 and are recoverable at `8c4ffa5`. The core is worth reading before
  rewriting: it encoded a distinction between perception cells (which decay) and
  externally-commanded keep-out cells (which persist until explicitly cleared and can never
  be overwritten by perception). Its `Occupancy`/`Grid`/`Cell` messages are in the same
  commit — re-add them when the node that fills them is landing.
- **The measured camera extrinsic**, above. Everything fused is provisional until then.
- ~~**A consumer.**~~ `crusader_bt`'s `bt_runner_node` now subscribes `/crsd/world_targets`
  and fills the behaviour tree's buoy field from it. It takes **confirmed tracks only** —
  which is why a confirmed phantom is a mission problem and not just a map problem, and most
  of the argument for `use_lidar` defaulting off.

## Change impact

| You changed | Re-run |
|---|---|
| `target_tracker_core.py` | `tools/bench/bench_world_model.py` and compare against the printed truth; it needs no hardware, so there is no excuse for skipping it |
| the fusion or association gates in `crusader_params.yaml` | `python3 tools/scripts/check_config.py`, then the bench with `--chop` — watch `position_stddev` on the map |
| `use_lidar` | nothing to rebuild — it is `[DYN]`. Confirm in `crsd/world_model_health` that `lidar_ignored` moved, and re-run the bench with `--no-lidar` to exercise the same path it now takes by default. Note the default bench field's unlabelled member is invisible with it off: that buoy exists to prove the LiDAR-only path, and there is no longer one |
| `cam_x/y/z/yaw/pitch` | every fused position shifts; re-check against real buoys at known ranges, and tighten `fuse_bearing_deg` |
| `map_server_core.py`'s page | open it in a browser and confirm the **stale** path still fires — feed it a snapshot with `ok: false` and check the banner, not just the healthy case |
| `TrackedTarget`/`TrackedTargetArray` fields | full `tools/scripts/rebuild.sh`; a mismatched message is a silent deserialization failure |
| topic names in either section | `python3 tools/scripts/check_config.py` — it pins producer against consumer, because a mismatch there starts both nodes cleanly and delivers nothing |
