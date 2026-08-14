# `tools/bench/` — where do the frames go?

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
