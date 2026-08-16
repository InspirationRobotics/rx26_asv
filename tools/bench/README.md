# `tools/bench/` — bench harnesses

Two unrelated jobs live here. The four `bench_*_{no,}ros.py` scripts measure **where the
frames go**; `bench_world_model.py` **invents** frames so the world model can be driven
with no hardware at all.

## `bench_world_model.py` — an invented buoy field around the REAL boat

```bash
python3 tools/bench/bench_world_model.py
```

**It uses the real vessel.** It subscribes to `/crsd/pose` and `/crsd/attitude` and
publishes only the two sensor topics — `oak/detections` and `crsd/lidar_clusters` —
computing what the camera and LiDAR *would* have reported for a field of buoys that isn't
there. Run `core.launch.py` as normal, then `target_tracker`, `map_server` and this.

That is a far better test than a simulated boat: the pose carries real RTK noise, the yaw
is the real moving-baseline solution with its real latency, and the roll and pitch are the
real hull in the real water. A synthetic boat moves perfectly, which is the one kind of
boat the tracker will never see. It also means the bench does not contend with
`telemetry_bridge` for `/crsd/pose` — two publishers on that topic make the boat teleport.

**The field is anchored at the first fix and rotated to the heading at that instant**, so
the buoys are laid out ahead of the bow wherever the boat happens to be — pointing the boat
anywhere and starting the bench puts targets in front of it. `FIELD` is written as
`(right, ahead)` in metres so the layout reads the way you would describe it from the helm.
Anchoring to fixed lat/lon would put the field in Florida while the boat sits in a car park.

**It prints the ground truth when it anchors**, which is what makes it a check rather than
a demo: the tracker's output is a number you can compare against a number you chose. A
track that settles within a metre of its truth row, keeps its id, and does not split in two
as the boat swings past it, is a tracker that works.

| Flag | Reproduces |
|---|---|
| `--no-camera` | LiDAR only — every track goes anonymous (empty label) |
| `--no-lidar` | camera only — ranges carry the stereo bias, targets sit beyond truth and wander. Nothing is dropped; that is the package invariant |
| `--sim-pose` | invent the **vessel** too: a boat circling the field, publishing `/crsd/pose` itself. Desk mode, for a laptop with no boat. **Do not use it while the core stack is up** |

`--chop` and `--drop-pose-after` apply only under `--sim-pose` and the script **errors**
rather than ignoring them — a `--chop` run that quietly did nothing because the real
attitude was in use would read as "roll compensation works", which is the opposite of what
it proved. On the real boat, rock the hull for chop and stop `telemetry_bridge` for a
dropout.

Under `--sim-pose` the boat circles rather than running straight, because a straight run
past a buoy never tests the two things most likely to be wrong — whether a track survives
leaving the field of view, and whether it is still *one* track when it comes back into view
from a different bearing. On the real boat you get that by driving a loop.

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
