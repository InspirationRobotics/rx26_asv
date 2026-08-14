# `crusader_world_model` — fusion and the occupancy grid

**This package is empty.** It is scaffolded so the first node has somewhere to land and
the dependency graph is already correct — not because anything works yet.

## What belongs here

Everything sensor-**agnostic**: per-sensor detections in, one consistent picture of the
world out. In the architecture diagram this is the pair of boxes drawn *outside* the
Perception group — **Fusion (3D object positions)** and **Occupancy grid** — because they
belong to neither the sensors nor the missions.

| Piece | Job |
|---|---|
| Fusion | Combine detections of the same object from different sensors into one 3D position. Camera contributes reliable bearing, LiDAR contributes accurate range. |
| Occupancy grid | World-anchored obstacle map assembled from fused objects and pose, with time decay. |

## Why this is a separate package from `crusader_perception`

Because it is testable and perception isn't. Everything here is geometry and time decay:
it can be exercised with invented detections, on a laptop, with no camera, no GPU and no
boat. Perception can't — it needs a model, a calibration and a scene.

Keeping them apart means the untestable half never becomes a dependency of the testable
half. Merge them and every fusion check needs a camera.

**Safety invariant to preserve when fusion is rebuilt:** a detection with no supporting
range from a second sensor is passed through with what it has — **never dropped**. Losing
an obstacle is worse than carrying a coarse range for it.

## What was here before

`occupancy_core.py`, `occupancy_grid_node.py` and the fusion half of `lidar_fusion.py`,
all removed unverified in v0.5 and recoverable from git at `8c4ffa5`. The occupancy core
is worth reading before rewriting: it encoded a distinction between perception cells
(which decay) and externally-commanded keep-out cells (which persist until explicitly
cleared and can never be overwritten by perception).

Its messages — `Occupancy`, `Grid`, `Cell` — are in the same commit. Re-add them to
`crusader_msgs` when the node that fills them is landing.
