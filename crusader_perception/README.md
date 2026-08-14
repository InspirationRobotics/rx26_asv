# `crusader_perception` — detection and ranging

**This package is empty.** It is scaffolded so the first node has somewhere to land and
the dependency graph is already correct — not because anything works yet.

## What belongs here

Sensor-**coupled** processing: raw data in, detections out.

```
sensor container            THIS package                 crusader_world_model
─────────────────           ────────────────             ────────────────────
OAK-D  → Image        →     detect + range        →      fuse → 3D positions
MID360 → PointCloud2  →     (per-sensor)          →      occupancy grid
```

The **sensor-driver container** owns the devices and publishes their raw output as ROS
topics. It does not detect anything. Inference runs *here*, on this container's GPU —
which is why the `asv` image carries CUDA/TensorRT and `cv_bridge`, and why it
deliberately carries no depthai or Livox SDK. If you need a device SDK, you are writing a
node for the wrong container.

Code that is model-coupled, frame-coupled or calibration-coupled goes in this package.
Anything that could be checked with made-up numbers and no camera belongs in
`crusader_world_model` instead — that split is the point of having two packages.

## What was here before, and why it isn't

The pre-v0.5 pipeline (`perception_node`, `detector`, `depth_association`,
`oakd_guard`, `pipeline_stats`, `lidar_fusion`) was deleted: it had never been verified
end-to-end, and its buoy model was trained on another team's buoys. It is all recoverable
from git at `8c4ffa5` and is worth reading before rewriting — particularly
`lidar_fusion.py`'s bearing/range gating, which encodes a real lesson about the MID360's
360° field of view voting a shoreline over a buoy.

The messages it used (`Detection`, `DetectionArray`) are in the same commit. Re-add them
to `crusader_msgs` when the node that fills them is landing, not before.

## Before the first node lands

Agree the **topic contract with the sensor container** first: topic names, message types,
QoS, frame ids, and what the camera's intrinsics/extrinsics look like on the wire. A
mismatched QoS profile is silent — a BEST_EFFORT publisher and a RELIABLE subscriber match
nothing at all, `ros2 topic list` looks perfect, and no data flows.
