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
                               /crsd/attitude ──────── /crsd/pose
```

**1. Fuse.** The camera knows *where to look* and *what it is*: its bearing is an angle read
off a rectified image, good to a fraction of a degree. The LiDAR knows *how far*: a
time-of-flight range, centimetres, against stereo disparity that is metres out past 15 m.
So fusion is not an average — it keeps the camera's **direction** and the LiDAR's
**distance** and throws away each sensor's weak number. Both inputs are body-frame already,
so this stage has no attitude in it.

**2. Project.** One call to `geo.body_to_world_ypr` per sighting. This is the only place
attitude enters, and it is not a refinement: at 20 m, 5° of uncompensated roll moves a
target 1.7 m, which is most of a buoy gate.

**3. Track.** Nearest-neighbour association in the world frame, an EMA whose gain decays as
`1/hits`, and two decay timeouts. This is what turns a stream of sightings into an object
that persists while the boat looks away — the thing a mission actually needs.

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
   it. This cuts both ways: an unlabelled LiDAR cluster is a thing that is *there* and
   reaches the map with an empty label rather than being filtered for being anonymous.
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

### The parameter that matters most

`assoc_radius_m` (default 3.0). Too wide merges a gate pair into one object at the midpoint;
too narrow splits one buoy into a new track every few seconds. Watch `position_stddev` on
the map — small and steady is a solid fix, growing means the gate is wrong.

Second is `fuse_bearing_deg`, currently loose at 6.0° to absorb the **unmeasured camera
extrinsic** (see below). Tighten it once `cam_*` are real numbers.

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

## If the map fills up with anonymous targets, that is the water

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

## What is still missing

- **The occupancy grid.** `occupancy_core.py` and `occupancy_grid_node.py` were removed
  unverified in v0.5 and are recoverable at `8c4ffa5`. The core is worth reading before
  rewriting: it encoded a distinction between perception cells (which decay) and
  externally-commanded keep-out cells (which persist until explicitly cleared and can never
  be overwritten by perception). Its `Occupancy`/`Grid`/`Cell` messages are in the same
  commit — re-add them when the node that fills them is landing.
- **The measured camera extrinsic**, above. Everything fused is provisional until then.
- **A consumer.** Nothing reads `crsd/world_targets` yet. Cognition has no package.

## Change impact

| You changed | Re-run |
|---|---|
| `target_tracker_core.py` | `tools/bench/bench_world_model.py` and compare against the printed truth; it needs no hardware, so there is no excuse for skipping it |
| the fusion or association gates in `crusader_params.yaml` | `python3 tools/scripts/check_config.py`, then the bench with `--chop` — watch `position_stddev` on the map |
| `cam_x/y/z/yaw/pitch` | every fused position shifts; re-check against real buoys at known ranges, and tighten `fuse_bearing_deg` |
| `map_server_core.py`'s page | open it in a browser and confirm the **stale** path still fires — feed it a snapshot with `ok: false` and check the banner, not just the healthy case |
| `TrackedTarget`/`TrackedTargetArray` fields | full `tools/scripts/rebuild.sh`; a mismatched message is a silent deserialization failure |
| topic names in either section | `python3 tools/scripts/check_config.py` — it pins producer against consumer, because a mismatch there starts both nodes cleanly and delivers nothing |
