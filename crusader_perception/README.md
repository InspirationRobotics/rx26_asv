# `crusader_perception` — sensors, detection and ranging

Sensor-**coupled** processing: raw data in, detections out. This package owns the OAK-D
and everything that turns its frames into positioned objects.

| Node | Consumes | Publishes |
|---|---|---|
| `oakd_publisher` | OAK-D LR (depthai) | `oak/rgb` (`bgr8`), `oak/depth` (`16UC1`, mm, aligned) |
| `buoy_detector` | OAK-D LR + TensorRT engine | `oak/detections` (`crusader_msgs/Detection3DArray`, `camera_link`) |
| `lidar_cluster_node` | `/livox/lidar` + `/crsd/attitude` + `/crsd/pose` | `crsd/lidar_clusters` (`crusader_msgs/Cluster3DArray`, `base_link`) |

Code that is device-coupled, model-coupled, frame-coupled or calibration-coupled goes
here. Anything that could be checked with made-up numbers and no camera belongs in
[`crusader_world_model`](../crusader_world_model/README.md) instead — that split is the
point of having two packages, and it is the one boundary in this area that still holds.

## This package was `crusader_sensors` until the detection node landed

The original split had three tiers: a sensor package that owned devices and published raw
frames, a perception package that consumed those frames and detected things, and the world
model — with the first two in *different containers*. `buoy_detector` collapsed all of it.
It opens the camera *and* runs inference in one process, because shipping 1.28 MB frames
across a container boundary to detect a few hundred bytes of buoy is the wrong trade —
~38 MB/s at 30fps versus a `Detection3DArray`.

Once detection runs beside the device, "sensors" and "perception" are one package, and the
name that describes it is the output, not the hardware. The empty `crusader_perception`
scaffold this replaced was written against the old plan; so was a lot of surrounding
documentation, which claimed a sensor container owned the OAK-D and that `asv` must never
contain depthai. Neither is true — there is no such container, and `asv` needs depthai to
run this package at all.

## The two nodes are alternatives, not a pipeline

The OAK-D admits exactly **one** client. Whichever of these starts first gets the camera
and the other fails to open it — by design, not by accident.

- `oakd_publisher` — frames on the wire, for a human looking at pixels. 1.28 MB per frame,
  ~38 MB/s at 30fps, and that traffic is the reason the raw-frame path is hard to scale
  across containers.
- `buoy_detector` — **raw in, detections out.** Inference runs beside the device, so
  nothing large ever leaves the process. This is the shape the rest of the stack should
  consume.

`buoy_detector` has `publish_frames` for bring-up, when you want detections *and* a picture
from one process. It costs exactly the bandwidth `oakd_publisher` costs, and it defaults
off.

Both build their device pipeline from
[`oak_pipeline.py`](crusader_perception/oak_pipeline.py) — one builder, because depth is
aligned to the RGB camera at one specific geometry and two copies would drift into ranges
that are quietly wrong rather than obviously broken. `check_config.py` pins their shared
camera params too.

`buoy_detector` imports **both** depthai (camera) and ultralytics/TensorRT (engine), so
`asv` is the only image that can run it — it is the one with the CUDA stack, and the
[Dockerfile](../Dockerfile) now installs depthai for exactly this reason. It used to
*assert depthai was absent*; that guard was written for a three-tier plan that this node
replaced, and it would now fail the build it exists to protect.

Engine path defaults to `/root/robotx_ws/models/buoy_v16.engine` and is a parameter.

## Detection output

`oak/detections` carries positions in `camera_link`, REP-103 body axes — **x forward,
y left, z up** — converted from the camera's optical frame (z forward, x right, y down)
inside the node, so no consumer has to remember which convention it holds. If your URDF
puts `camera_link` somewhere other than the RGB sensor's optical centre, that offset
belongs in TF, not here.

These positions are **body-frame and boat-relative**. Turning one into a map position takes
boat pose *and* attitude: `crusader_common.geo.body_to_world_ypr` consumes an (x, y, z)
straight off `Detection3D` together with `/crsd/attitude` and `/crsd/pose`. Without roll
and pitch a 20 m bearing is only correct on flat water — see
[`crusader_fcu`](../crusader_fcu/README.md).

Two properties fusion depends on:

- **Published every frame, empty or not.** An empty array means "alive, saw nothing";
  silence means the producer is dead. A consumer that cannot tell those apart will steer
  on a ten-second-old detection.
- **`header.stamp` is the camera instant** — device timestamp with transport latency
  removed, not publish time — because that is what a LiDAR sweep gets associated against,
  and what an attitude sample gets matched to.

Boxes that the detector sees but depth cannot range are **dropped, not published with a
guessed position**, and counted in the health line as `no_depth`. A detector that sees
buoys but cannot range them otherwise looks identical, from downstream, to one that sees
nothing.

## Watching it work

`buoy_detector` serves its own annotated MJPEG view — open `http://<JETSON_IP>:8080` in a
browser on the boat network. No ROS on the laptop, no topic subscription, no second client
on the camera.

What's drawn, and why each part is there:

| Overlay | Tells you |
|---|---|
| Box, coloured by class family | what the engine found, and whether red/green are the right way round — a gate is defined by which colour is on which side |
| `red_buoy 87% 12.4m [12.4,-1.8,+0.2]` | range first (check it against a tape measure), then the `camera_link` position published on the topic |
| `NO DEPTH` in place of a range | the engine saw it, stereo could not range it — this box is **not** on the detections topic |
| Small rectangle + dot inside the box | exactly where depth was sampled, with the count of pixels that survived the range gate |

That last one is the reason this view exists rather than a plain `oak_view`. When
`no_depth` climbs, the picture tells you immediately whether the sample patch is sitting on
sky above a buoy (raise `SAMPLE_V_RATIO`), on water past `range_max_m`, or on a genuinely
featureless surface that stereo cannot match.

**Frames are only drawn while a browser is connected.** With no viewer the annotation and
copy are skipped entirely — that CPU belongs to inference. `stream_enable: false` removes
the server altogether.

Port 8080 is also `tools/oak_view.py`'s default. They cannot both bind it, and they are
never both useful at once: this one already has the frames.

If the port is busy the node **warns and keeps detecting** rather than failing to start.
The detections topic is the product; the view is a convenience, and a bring-up viewer must
never be able to stop the boat from seeing buoys.

## Where it runs, and why it is not in the launch file

In `asv`, with everything else in this repo. There are two containers on the Jetson and
the other one drives the MID360 and nothing else.

`crusader_bringup` exec_depends on this package, so `--packages-up-to crusader_bringup`
builds it with the rest and a compile error here surfaces at build time rather than on the
boat. It builds even without the SDK present: `depthai` is imported *inside* the node's
`_open_device`, never at module scope, so `colcon build` and CI's import check pass on a
machine with no camera and no depthai.

Neither node is in `core.launch.py`, and that is deliberate: **the OAK-D admits exactly one
client**, so the two contend. Which one runs is a per-session operator choice — frames for
a human, detections for the stack — and that is not something a launch file can decide.
Start the one you want by hand.

This is also why `crusader_world_model` is a separate package rather than more files here.
Everything there is geometry and time decay — exercisable with invented detections, on a
laptop, with no camera, no GPU and no boat. Nothing here is. Merge them and every fusion
check needs a camera.

## Parameters

From `crusader_bringup/config/crusader_params.yaml`, sections `oakd_publisher` and
`buoy_detector`, like every other node in the repo. `crusader_common/config.py` resolves it
env (`$CRUSADER_PARAMS`) → the installed share dir → the source tree. If it resolves none
of them the node fails loudly at startup rather than inventing defaults.

All `[RO]` — structural. Change them in the YAML and restart; `ros2 param set` is rejected.

| Param | Default | Meaning |
|---|---|---|
| `rgb_topic` / `depth_topic` | `oak/rgb` / `oak/depth` | relative topic names |
| `frame_id` | `oak_rgb_camera_optical_frame` | shared by both images |
| `fps` | `30.0` | camera rate |
| `isp_denominator` | `3` | 1/N of 1920x1200 → `3` gives the stereo-native 640x400 |
| `sync_threshold_ms` | `50` | max RGB/depth gap the device will pair |
| `subpixel` / `lr_check` | `true` / `true` | StereoDepth quality modes |
| `poll_period_s` | `0.01` | output-queue poll; must stay well under `1/fps` |
| `queue_size` | `4` | device queue depth, non-blocking |
| `health_period_s` | `5.0` | warns when no frames arrived since the last check |

## The topic contract

```
oak/rgb    sensor_msgs/Image   bgr8    640x400   BEST_EFFORT (qos_profile_sensor_data)
oak/depth  sensor_msgs/Image   16UC1   640x400   BEST_EFFORT, millimetres
```

Both names are **relative**, so a namespace or a remap moves the pair together, and both
carry the same `frame_id` — depth is aligned to the RGB camera, so they are the same
optical frame and the RGB intrinsics apply to both images unchanged.

Three properties consumers may rely on, and one they may not:

- **Same instant.** RGB and depth are paired *on the device* by `dai.node.Sync`, within
  `sync_threshold_ms`. A pair that cannot be matched is dropped, never half-published — so
  `depth[v, u]` for a `(u, v)` off an RGB detection box is legal.
- **Same size.** The 1200p sensor is ISP-scaled 1/3 to 640x400 and `StereoDepth` outputs
  at that size; the node hard-errors if the two ever disagree.
- **Same stamp.** One `header.stamp` for the pair, derived from the device timestamp minus
  measured transport latency — not "now" at publish time.
- **Not reliable delivery.** `BEST_EFFORT`. A `RELIABLE` subscriber matches a `BEST_EFFORT`
  publisher *not at all*: `ros2 topic list` looks perfect and zero frames flow. This is the
  single most likely reason a consumer sees nothing.

The geometry is ported from the prequal-proven `gate_navigator` in
[robotx_2026](https://github.com/InspirationRobotics/robotx_2026), so a node written
against that pipeline works against these topics without re-deriving intrinsics.

## Running and checking it

In `asv`, with the workspace sourced. No `--params-file` needed: the node loads its
declaration defaults from the YAML itself (that is what `declare_from_config` does), so the
file is honoured whether or not a launch system passes it.

```bash
ros2 run crusader_perception oakd_publisher
```

If `crusader_bringup` is not built in that workspace, point the node at the file directly:

```bash
CRUSADER_PARAMS=/root/robotx_ws/src/rx26_asv/crusader_bringup/config/crusader_params.yaml ros2 run crusader_perception oakd_publisher
```

```bash
ros2 topic hz /oak/rgb
```

Watch it in a browser:

```bash
python3 tools/oak_view.py --topic /oak/rgb
```

`oak_view.py` on a **raw** topic needs `cv2` + `numpy`; it defaults to a `/compressed`
topic, which this node does not publish — pass `--topic` as above.

## What is NOT published, deliberately

- **`camera_info`.** Depth in millimetres is not a 3D position without `fx`/`cx`; today a
  consumer reads them with `device.readCalibration()` for itself, as `gate_navigator` does.
  Publishing `oak/camera_info` (`sensor_msgs/CameraInfo`, same stamp and frame) is the
  obvious next addition and would let consumers drop their depthai dependency entirely.
- **Compressed transports.** No `image_transport` republishing here; add a `republish` node
  if a laptop-side viewer needs JPEG.

## `lidar_cluster_node` — the LiDAR half

The MID360 is driven by the **livox container** (the only other container on the Jetson,
and the only device boundary this repo still has). It publishes `/livox/lidar` as a
`sensor_msgs/PointCloud2`; this node clusters it into 3D objects. It opens no device.

**The geometry lives in [`lidar_cluster_core.py`](crusader_perception/lidar_cluster_core.py)
— no rclpy, exercised against invented clouds on a laptop.** `tools/lidar_view.py` imports
the transform and the filters *from that same module*, so the bench orientation check
(docs/G2) proves something about what this node actually does rather than about a second
copy of the maths.

The pipeline, and the order is load-bearing:

| Stage | Frame | Why here |
|---|---|---|
| `near_mask` (0.5 m sphere), `fov_mask` (forward 180°) | **sensor** | The origin here *is* the sensor, which is what "0.5 m from the LiDAR" means. Both describe what the sensor can physically see: the mount and deck beneath it, and the hull blocking the view aft. |
| `to_body` — sign flips, then translation | body | Offsets are measured in body directions, so translating before flipping moves the sensor the wrong way along every flipped axis. |
| `level` — de-rotate roll/pitch, keep yaw | levelled | The water gate **must** run here. Under power the boat pitches bow-up, and a z gate in the tilted body frame slices the water at a different height on every wave. |
| water / sky / range gates | levelled | Most of this mount's FOV is water — see below. |
| voxel-grid DBSCAN | levelled | Isotropic metric: "0.4 m apart" means the same at any attitude. |

Centroids come back out in **body** frame, not levelled. The camera is bolted to the same
hull so `oak/detections` is body-frame too, and fusion compares them directly; whoever
projects into the world applies attitude once, there. Publishing levelled positions would
apply it twice.

### Why DBSCAN, and why `eps` stays small

Plain Euclidean cluster extraction is pure connectivity, which fails two ways on water: a
thin trail of spray can **chain** a buoy to the shoreline behind it into one cluster centred
on neither, and every isolated glint becomes its own tiny cluster. Requiring a density
before a point may *seed* a cluster kills both and labels the rest noise.

The tempting mistake is to widen `eps` for distant objects. It is backwards: a buoy is
0.3 m wide at every range, so its returns are always ~0.3 m apart, while the neighbourhood
volume grows as `eps³` and sweeps in proportionally more scattered noise. Widening helps the
noise more than the buoy. What actually changes with range is the *spacing* between returns
on one object, which the gentle `eps(r) = eps_0 · max(1, r/eps_r_ref)` growth covers.

Density is counted in **raw points, not occupied voxels** — voxelising discards exactly the
quantity the core test needs, so each voxel carries its point count as a weight.

### Accumulation is not optional

The MID360 returns ~200k points/s spread over 360°×59°, so a 0.3 m × 1 m buoy gives roughly:

| Range | Points per sweep | 1 sweep | 5 sweeps |
|---|---|---|---|
| 5 m | ~37 | ✅ | ✅ |
| 10 m | ~9 | ✅ | ✅ |
| 15 m | ~4 | ❌ | ✅ |
| 20 m | ~2 | ❌ | ✅ |

(✅/❌ measured against this core, not estimated.) One *sweep* is one `PointCloud2` message —
the driver bundles ~100 ms of scanning at 10 Hz. Livox's **non-repetitive** pattern means
each sweep covers different parts of the FOV instead of retracing the same rings, so
stacking five genuinely triples the reach rather than resampling the same returns.

**But only with motion compensation.** Each retained sweep is stored with the pose it was
taken at and transformed into the current body frame before clustering. Without it, a boat
moving 1.5 m between sweeps reports one buoy as **two phantom objects 1.5 m apart**; moving
slower it smears one cluster to more than double its true width. If `/crsd/pose` goes stale
the node falls back to a single sweep and says so — a sparser correct cloud beats a denser
smeared one, and range drops to ~10 m.

Accumulation also smears anything *actually moving*, which matters for Mission Task 4's
moving surface object. Lower `accumulate_sweeps` when tracking movers.

### Upside-down mounting: most of the FOV is water

The 180° roll flips the vertical FOV from `−7°..+52°` to `−52°..+7°`, so the great majority
of returns are water or ground. The `water_z` gate therefore does real work, and
**`water_z` is currently a placeholder of 0.10 m** — measure it floating (G2 step 4) before
trusting the filter.

### Health

`crsd/lidar_cluster_health` carries JSON: how many points each stage dropped, sweeps in the
window, whether the cloud was compensated and levelled. A stage silently eating the whole
cloud is indistinguishable from a dead sensor downstream, so the node also logs loudly when
it produces zero clusters from a non-empty cloud.

### Still to come

The pre-v0.5 `lidar_fusion.py` is recoverable from git at `8c4ffa5` and is worth reading
before fusion is rebuilt — particularly its bearing/range gating, which encodes a real
lesson about the MID360's 360° field of view voting a shoreline over a buoy.

## Change impact

| You changed | Re-run / re-check |
|---|---|
| the `oakd_publisher` / `buoy_detector` sections of `crusader_params.yaml` | `python3 tools/scripts/check_config.py`, then restart the node — params are `[RO]` |
| added/removed a node in this package | `CONFIG_DRIVEN_NODES` in `tools/scripts/check_config.py`, or the section check fails |
| `isp_denominator`, `fps` | `ros2 topic hz` on both topics; every consumer's assumed image size and any pixel-space threshold |
| `frame_id` | anything doing TF lookups against the camera |
| topic names / QoS | every subscriber — a QoS change is silent, not an error |
| the pipeline (sync, alignment, `setOutputSize`) | re-verify `depth[v, u]` against a known-distance target before trusting any range downstream |
| the `camera_link` axis convention | `crusader_common.geo.body_to_world_ypr`, and anything mapping detections into the world |
