# Gate G2 — perception trust gate (bench)

**Purpose:** objective-1 (collision) metrics are officially untrusted until the buoy
model is retrained on Crusader's own buoys AND the end-to-end pipeline meets its
budget on real hardware. G2 flips `objective1.trusted` in the episode evaluator.

**Pass criteria (all):**
1. Regression gate: `eval_regression.py` exits 0 (per-class precision & recall ≥ 0.80
   for all 5 LED buoy states, on held-out sessions never seen in training).
2. Live throughput: ≥ 15 fps sustained end-to-end (`/crsd/perception_health`).
3. Live latency: p95 capture→publish ≤ 100 ms (same topic).
4. Position accuracy: mean error < 0.5 m at 10 m range vs RTK ground truth.

## A. Model retrain path (repeat per footage batch)

1. Collect: `tools/scripts/collect_footage.py` on bench + water days — vary range
   (3/10/20 m), sun angle, chop. Target ≥ 1500 labeled frames across ≥ 4 sessions
   before first retrain; ALL five LED states must appear.
2. Label in YOLO format (session `labels/` dir + `classes.txt`).
3. `tools/training/prep_dataset.py` — hold out ≥ 1 full session (`--holdout`).
4. `tools/training/train_buoy.py` — fine-tune from V16 base.
5. `tools/training/eval_regression.py` — **must pass before export**.
6. `train_buoy.py --export-only` **on the target Jetson** (engines are per-Jetson/
   JetPack). Copy to `/root/robotx_ws/models/`, update `engine_path` param.
7. First live run: `Detector` will refuse to start if the engine's class names
   aren't covered by `perception/config/class_map.json` — fill in the real class
   list on first bench bring-up (see TODO in that file).

## B. Live bench measurement

1. Preflight passes (USB `SUPER` is checked there and again at node start).
2. Survey the buoy position: RTK fix with the boat's bow at the buoy, record
   lat/lon. Move boat to a surveyed pose ~10 m away, static, GPS yaw resolved.
3. Launch `perception_node` + `telemetry_bridge` + `frame_transform`.
4. Run `tools/bench/g2_error_logger.py --truth-lat ... --truth-lon ...
   --label <state under test>`; let it accumulate ≥ 200 detections.
5. Record: mean/p95 error, fps, p95 latency, engine file, `training_manifest.json`
   contents, git SHA. Repeat for at least 3 of the 5 LED states and 2 ranges (5 m,
   10 m).
6. OAK-D LR socket check: first bring-up must verify CAM_A/B/C assignment
   (`perception_node` TODO) — wrong sockets fail loudly (no depth), not subtly.

## C. Flip the trust flag

After sign-off, episode runs may set `--perception-trusted` (the
`assemble(..., perception_trusted=True)` path). The flag is per-run and recorded
in every metrics JSON — never edit the evaluator default.

- Perception lead: ____________  date: ________
- Safety lead:     ____________  date: ________
- Engine file + sha: __________________________
