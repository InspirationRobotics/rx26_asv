# `crusader_sensors` — device drivers

Nodes that **own hardware** and publish its raw output as ROS topics. Nothing here
interprets anything: no detection, no fusion, no filtering. Raw data out, and that is all.

| Node | Owns | Publishes |
|---|---|---|
| `oakd_publisher` | OAK-D LR (depthai) | `oak/rgb` (`bgr8`), `oak/depth` (`16UC1`, mm, aligned) |

## Builds everywhere, runs in the sensor container

`crusader_bringup` exec_depends on this package, so `--packages-up-to crusader_bringup`
builds it with everything else and a compile error here surfaces at build time rather than
on the boat. Building it in `asv` is safe: `depthai` is imported *inside* the node's
`_open_device`, never at module scope, so `colcon build` and the import smoke in
[`setup/install_container.sh`](../setup/install_container.sh) pass in a container with no
SDK and no camera.

**Running** it is a different matter, and only the sensor container may. The `asv` image
fails its own build if `depthai` appears in it ([Dockerfile](../Dockerfile)), because the
OAK-D admits exactly one client: a second process opening the device takes the camera away
from this node.

## Parameters

From `crusader_bringup/config/crusader_params.yaml`, section `oakd_publisher`, like every
other node in the repo — one params file for the boat, whichever container a node lives in.
The sensor container therefore needs `crusader_bringup` built in its workspace, or
`$CRUSADER_PARAMS` pointed at the file; `crusader_common/config.py` resolves env → the
installed share dir → the source tree, and a mounted repo satisfies the last one. If it
resolves none of them the node fails loudly at startup rather than inventing defaults.

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

In the sensor container, with that workspace sourced. No `--params-file` needed: the node
loads its declaration defaults from the YAML itself (that is what `declare_from_config`
does), so the file is honoured whether or not a launch system passes it.

```bash
ros2 run crusader_sensors oakd_publisher
```

If `crusader_bringup` is not built in that workspace, point the node at the file directly:

```bash
CRUSADER_PARAMS=/root/robotx_ws/src/rx26_asv/crusader_bringup/config/crusader_params.yaml ros2 run crusader_sensors oakd_publisher
```

```bash
ros2 topic hz /oak/rgb
```

From any ROS container on the boat network (`asv` included), watch it in a browser:

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

## Change impact

| You changed | Re-run / re-check |
|---|---|
| the `oakd_publisher` section of `crusader_params.yaml` | `python3 tools/scripts/check_config.py`, then restart the node — params are `[RO]` |
| added/removed a node in this package | `CONFIG_DRIVEN_NODES` in `tools/scripts/check_config.py`, or the section check fails |
| `isp_denominator`, `fps` | `ros2 topic hz` on both topics; every consumer's assumed image size and any pixel-space threshold |
| `frame_id` | anything doing TF lookups against the camera |
| topic names / QoS | every subscriber — a QoS change is silent, not an error |
| the pipeline (sync, alignment, `setOutputSize`) | re-verify `depth[v, u]` against a known-distance target before trusting any range downstream |
