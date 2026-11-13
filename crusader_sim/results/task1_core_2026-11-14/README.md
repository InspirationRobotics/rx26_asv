# Task 1 Core in Gazebo — overnight 2026-11-13/14 (Chris's laptop)

Goal: Task 1 Core done consistently in the sim before the real run, every attempt scored the way
the handbook scores it, so our branch and Dominick's can be compared on one number.

## How every attempt is run and scored

```bash
# WSL Ubuntu-22.04, with the sim installed (crusader_sim/setup/install_wsl.sh + crsd-sim image)
RX26_WIN_SRC=/mnt/c/<path>/rx26_asv bash crusader_sim/scripts/t1_attempt.sh LABEL COURSE TREE [key=value ...]
```

- A **fresh sim per attempt** (`gz_sim_up.sh COURSE --no-uav --no-gui`): boat at the start, empty tracker.
- `key=value` = the GCS **Run panel** inputs, set on `/bt_runner_node` before Start
  (e.g. `t1_field_lat`/`t1_field_lon`, the field centre an operator clicks on the map).
- **Score /325** = the handbook's Core table (section 4) by `crusader_sim/task1_score.py`:
  heartbeat 25, ENTRY/EXIT reported 10+10, ENTRY cw / EXIT ccw circles 25+25, greens/reds
  reported at their true positions 10 each (≤4 each), all greens to port 25, all reds to
  starboard 25, bonus 100 for everything. A reported buoy counts within **1.5 m** of the true one
  with the right state (the handbook gives no tolerance; 1.5 m is under the field's 1.86 m
  smallest spacing). The path is graded by the existing `task1_judge`.
- Course **task1_sg** = the real Singapore field (Chase's LiDAR map of 2026-11-10), ENTRY south,
  EXIT north. Detector = the truth oracle (perfect colours, stereo-depth positions): this tests
  tracking, planning and driving, not the YOLO model.
- Per attempt here: `LABEL.log` (the run and the judge), `LABEL.score.txt`, `LABEL.png` (true track,
  true field, reported buoys as ×).

## Attempts

| # | tree | course | Run panel | score | judge | notes |
|---|---|---|---|---|---|---|
| a01 | task1_core_scan (main 2ae0ce4, as on the boat 13 Nov) | task1_sg | field centre only | **175/325** | FAIL 6/8 sides | report perfect (ENTRY, EXIT, 4/4 G, 4/4 R within 0.33 m); both circles correct; b08 red kept to port, b09 green to starboard at the north end; 13 re-plans |
| a02 | same (repeat) | task1_sg | field centre only | **200/325** | FAIL 6/8 sides | report perfect, both circles; all greens correct; b08 and b10 red to port. See below |
| a03 | same | task1_sg | + LiDAR fusion (`use_lidar` true, r_max 25) | **165/325** | FAIL 4/8, no EXIT circle | worse: 4 false reports (one GREEN 0.88 m from b06 red), tree FAILED at PlanGlobalPassage. Fusion as it stands mixes neighbouring buoys; not pursued |
| a04 | same | task1_sg | + `assoc_radius_m` 1.5 (default 3.0) | **125/325** | FAIL, no circles | 23 tracks for 11 buoys (9 duplicate reports), as on the water on 13 Nov: far buoys' stereo error > 1.5 m. One fixed radius cannot serve this field (see below) |
| a05 | same | task1_sg | + range-aware tracking (`range_sigma_k` 0.0036, `vote_ambiguity_m` 1.0) | **150/325** | FAIL, touched b08 | 20 tracks: the stereo-only error model is too optimistic up close (pose/frame timing adds error), its 0.9 m gate split buoys. Kept in the code (off), not used |
| a06 | baseline repeat | task1_sg | field centre only | **200/325** | FAIL 7/8 | report perfect, both circles, only b03 red to port. **Baseline over a01/a02/a06: 175, 200, 200** |
| a07 | same tree | task1_sg | + **mapped field prior** (`target_tracker/prior_field` = the LiDAR map, `prior_field_task1_sg.txt`) | **325/325** | **PASS 8/8** | everything: report, both circles, every side, the b09/b10 gap threaded, bonus. 11 tracks for 11 buoys (no duplicates), 13 plans (baseline 76-120) |

### One association radius cannot serve this field (a03, a04)

The tracker's own docstring says `assoc_radius_m` must stay below the narrowest gate, or buoys of
different colours merge. Here buoys are 1.86 m apart, but stereo range error grows with range
squared (sim: ~0.1 m at 5 m, 0.36 m at 10 m, 0.8 m at 15 m, 1.4 m at 20 m). At 3.0 m (default)
up-close neighbours merge: positions drag, colours mix, the planner sees the gap wobble. At 1.5 m
every far buoy splits into duplicates. **Range-aware tracking** (target_tracker, new, OFF by
default): per sighting sigma(r) = k r^2; the association gate is 2.5 x the combined sighting and
track sigma (clamped 0.9-3.0 m); positions are fused by inverse variance (a far sighting barely
moves a buoy fixed from close in; a close one corrects a buoy first placed from far); a sighting
with another track within 1.0 m of its own casts no colour vote. All live parameters on the GCS
Params tab: `range_sigma_k` (0 = off; sim 0.0036, a real OAK-D ~2x), `vote_ambiguity_m`.

### What goes wrong at the north end (a02, from its track and the runner log)

The only legal route past b08/b09/b10 is through the b09 (green) / b10 (red) gap: 1.86 m centre to
centre, 1.43 m between buoy surfaces, ~0.71 m from the boat's centre to each buoy when centred.
Camera-only, the tracker's buoy positions wobble 0.3-0.4 m (stereo depth), so the transit
re-planned five times ("a buoy moved 0.4 m toward the path ahead"), already relaxed to 0.50 m.
One wobble left **no plan at any clearance** (0.80 / 0.70 / 0.50 m: "no passage keeps every
side"), and right after that failed re-plan the runner logged **"transit complete -> exit_orbit"**
with the boat still short of the gap. The exit orbit's lead-in then went round the WEST of b09
and the turn-back near b08/b10 put both reds to port.

## Status at 01:25 (session stopped at its usage limit)

**Best so far: a07, 325/325**, the boat's own `task1_core_scan` tree, unchanged, plus the tracker's
mapped field prior. Run it on the boat, after review, by setting on the GCS Params tab
(target_tracker, live): `prior_field` = the contents of `prior_field_task1_sg.txt`, and on the Run
panel the field centre 1.2806335, 103.8556542 as before. Everything else is main as run on 13 Nov.

**Not yet done** (each is one `t1_attempt.sh` line; the map string is in `prior_field_task1_sg.txt`):
- a08 (shifted 1.8 m E / 1.2 m S + 0.12 m noise map, to prove the alignment) was running when the
  session stopped; its result is in `~/t1_runs/a08_scan_prior_shifted_sg.*` in WSL, not committed.
- The prior on `task1_sg_rev` and `task1_sg_black` (same buoy positions, other lights), and repeats.
- Dominick had pushed no Task 1 branch by 01:18, so no comparison yet.
- No screen recording yet (`T1_GUI=1` keeps the Gazebo window for one).

**Risk to weigh before the water:** the prior drops any sighting within 2 m of a mapped buoy that
it cannot place confidently. A buoy re-moored 1-2 m from the map could go unseen. Off = `prior_field ""`.

## Sim fixes made on the way (sim only, nothing that runs on the boat)

- `sim_camera.py`: frames are handed to ROS as `array('B')`, not `bytes`. rosidl's setter checked
  every byte of a 1920×1200 frame in Python (~90 % of the node's CPU by py-spy); on this laptop the
  sim camera fell to ~1 frame/s, lost odometry sync and the sim boat went blind. Now ~12 frames/s,
  0 dropped, worst pose skew 47 ms.
