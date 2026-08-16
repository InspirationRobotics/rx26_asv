# `tools/bench/` — bench harnesses

Two unrelated jobs live here. The four `bench_*_{no,}ros.py` scripts measure **where the
frames go**; `bench_world_model.py` **invents** frames so the world model can be driven
with no hardware at all.

## `bench_world_model.py` — a boat and a buoy field, invented

```bash
python3 tools/bench/bench_world_model.py
```

Publishes `/crsd/pose`, `/crsd/attitude`, `/crsd/fcu_status`, `oak/detections` and
`crsd/lidar_clusters` for a simulated boat circling an invented buoy field, at roughly the
real rates and in the real frames. Run `target_tracker` and `map_server` alongside it and
open `http://localhost:8082`.

**It prints the ground truth at startup**, which is what makes it a check rather than a
demo: the tracker's output is a number you can compare against a number you chose. A track
that settles within a metre of its truth row, keeps its id, and does not split in two when
the boat circles it, is a tracker that works.

The circle is not decoration. A straight run past a buoy never tests the two things most
likely to be wrong — whether a track survives leaving the field of view, and whether it is
still *one* track when it comes back into view from a different bearing.

| Flag | Reproduces |
|---|---|
| `--no-camera` | LiDAR only — every track goes anonymous (empty label) |
| `--no-lidar` | camera only — ranges carry the stereo bias, targets sit beyond truth and wander. Nothing is dropped; that is the package invariant |
| `--drop-pose-after N` | a dead bridge. The tracker must stop ingesting, the map must go red and grey the boat out, and existing tracks must expire on schedule |
| `--chop` | roll and pitch. With compensation working the targets hold still; without it they breathe in and out by a metre or two at 20 m |

**What it cannot prove:** it builds detections with `geo.world_to_body_ypr`, the exact
transpose of the transform the tracker runs, so a sign error shared by both cancels and the
map looks perfect anyway. It proves plumbing, association, decay, fusion arbitration and the
display — never the frame convention. That is bench work against real hardware, the way
[docs/G2](../../docs/G2_lidar_orientation.md) did it for the LiDAR.

## The four throughput benches — where do the frames go?

Four scripts, one variable at a time, to answer a question the running node cannot:
**is the camera slow, is ROS slow, or is the transport slow?** Written to settle whether
the OAK-D should be wrapped in ROS at all.

Run them in the container that owns the camera, in order. Each prints one `RATE=` line.

| Script | Cameras | Depth | ROS | Isolates |
|---|---|---|---|---|
| `bench_rgb_noros.py` | 1 | no | no | the floor — what the device can do at its cheapest |
| `bench_stereo_noros.py` | 3 | yes | no | cost of stereo, alignment and Sync |
| `bench_rgb_ros.py` | 1 | no | yes | the ROS tax on one stream |
| `bench_stereo_ros.py` | 3 | yes | yes | the whole `oakd_publisher` path |

```bash
python3 tools/bench/bench_rgb_noros.py
```

```bash
python3 tools/bench/bench_stereo_noros.py
```

```bash
python3 tools/bench/bench_rgb_ros.py
```

```bash
python3 tools/bench/bench_stereo_ros.py
```

## Reading the result

Four rates, and the drop between consecutive ones names the cost:

- **rgb_noros low (< 25)** — the device or USB is the limit. Nothing downstream matters
  until this is fixed. Check `usb=SUPER` in the header line.
- **stereo_noros ≪ rgb_noros** — stereo compute. Re-run with `--no-subpixel`, then
  `--no-lr-check`, then `--isp 4`; the flag that moves the number is your trade.
  Then `--no-sync` to see whether colour or depth is the slow half — with Sync the rate
  is `min(rgb, depth)`, so a slow depth stream drags colour down with it invisibly.
- **rgb_ros ≪ rgb_noros** — ROS itself. Read the per-frame breakdown; `publish=` far
  above `build=` means a blocking DDS write, not Python overhead.
- **stereo_ros ≪ stereo_noros** — same, at 1.28 MB per pair instead of 768 kB.
  `--rgb-only` shows how much of it is the depth image alone.

`--no-publish` on both ROS scripts does everything except the DDS write. If the rate is
unchanged with and without it, the transport is free; if it jumps, the write is the wall.

## Two ways to be fooled

**Nobody is subscribed.** With no subscriber, DDS discards cheaply and the publish looks
free. The number that matters for the fusion node is measured with a real consumer —
run `ros2 topic hz /bench/rgb` from the container that will do the consuming, and expect
a different (lower) result than the empty-network run.

**A one-second warmup is skipped** in all four scripts, on purpose: the first frames land
while the device is still ramping, and counting them understates a healthy pipeline.

## Reference point

robotx_2026's `oak_view.py` is 1 camera, ISP 1/2, **15 fps**, JPEG-encoded to a browser —
no stereo and no ROS at all. To compare like with like:

```bash
python3 tools/bench/bench_rgb_noros.py --isp 2 --fps 15 --cv
```

That, not 30 fps of raw RGB + depth, is what "way faster" was measured against.
