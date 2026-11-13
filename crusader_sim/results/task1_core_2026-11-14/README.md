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

## Sim fixes made on the way (sim only, nothing that runs on the boat)

- `sim_camera.py`: frames are handed to ROS as `array('B')`, not `bytes`. rosidl's setter checked
  every byte of a 1920×1200 frame in Python (~90 % of the node's CPU by py-spy); on this laptop the
  sim camera fell to ~1 frame/s, lost odometry sync and the sim boat went blind. Now ~12 frames/s,
  0 dropped, worst pose skew 47 ms.
