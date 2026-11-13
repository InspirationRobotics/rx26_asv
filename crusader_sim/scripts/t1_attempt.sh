#!/usr/bin/env bash
# t1_attempt.sh — ONE scored Task 1 attempt from a fresh sim. Runs in: WSL2 Ubuntu-22.04.
#
#     RX26_WIN_SRC=/mnt/c/.../rx26_asv bash t1_attempt.sh LABEL COURSE TREE [key=value ...]
#
#   1. gz_sim_up.sh COURSE --no-uav --no-gui: a fresh world, the boat at the course's start and
#      the tracker empty, so attempts do not leak into each other. T1_GUI=1 keeps the Gazebo
#      window (to watch or record); T1_UP_ARGS adds gz_sim_up flags (e.g. "--tuning FILE").
#   2. Every key=value is set on /bt_runner_node before the goal: the GCS Run panel's inputs
#      (t1_field_lat, t1_field_lon, t1_scan_dwell_s, t1_approach_lat ...), read at Start.
#      node/key=value sets another node's live parameter (target_tracker/pos_alpha=0.1), as
#      the GCS Params tab would. SIM_TRACKER_LIDAR=1 in the environment = LiDAR fusion on.
#   3. task1_score record in the background, task1_goal --tree TREE in the foreground.
#   4. task1_score score. Kept in ~/t1_runs/: LABEL.log (task1_goal + judge), LABEL.report.json
#      (the last safe_passage report), LABEL.track.csv (the true track), LABEL.png (track,
#      field and report drawn), LABEL.score.txt, LABEL.bt.log and LABEL.tt.log (the runner's
#      and the tracker's logs), and one JSON line appended to summary.jsonl.
#
# The score is the handbook's Core table (task1_score.py says how it is computed).
set -uo pipefail
[ $# -ge 3 ] || { echo "usage: t1_attempt.sh LABEL COURSE TREE [key=value ...]" >&2; exit 2; }
LABEL="$1"; COURSE="$2"; TREE="$3"; shift 3
WIN_SRC="${RX26_WIN_SRC:?set RX26_WIN_SRC to the Windows checkout, as a WSL path}"
CT="${RX26_CONTAINER:-crsd-sim}"
OUT="$HOME/t1_runs"; mkdir -p "$OUT"
ROS='source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash'
GUI_ARG="--no-gui"; [ "${T1_GUI:-0}" = 1 ] && GUI_ARG=""

echo "=== $LABEL: $TREE on $COURSE ($(date '+%F %T'))"
tr -d '\r' < "$WIN_SRC/crusader_sim/scripts/gz_sim_up.sh" > /tmp/gz_sim_up.sh
RX26_WIN_SRC="$WIN_SRC" TREE="$TREE" bash /tmp/gz_sim_up.sh "$COURSE" --no-uav $GUI_ARG ${T1_UP_ARGS:-} \
  > "$OUT/$LABEL.up.log" 2>&1 || { echo "*** sim did not come up: $OUT/$LABEL.up.log"; tail -5 "$OUT/$LABEL.up.log"; exit 2; }

for kv in "$@"; do
  key="${kv%%=*}"; node=bt_runner_node
  case "$key" in */*) node="${key%%/*}"; key="${key#*/}" ;; esac   # node/param=value
  docker exec "$CT" bash -c "$ROS && ros2 param set /$node $key '${kv#*=}'" 2>&1 \
    | grep -v '^Set parameter successful' | sed "s|^|  param $node/$key: |"
done

docker exec "$CT" rm -f /tmp/t1_report.json /tmp/t1_report.json.track.csv
docker exec -d "$CT" bash -c "$ROS && exec python3 -m crusader_sim.task1_score record --out /tmp/t1_report.json"
docker exec "$CT" bash -c "$ROS && python3 -u -m crusader_sim.task1_goal --course $COURSE --tree $TREE" \
  > "$OUT/$LABEL.log" 2>&1
docker exec "$CT" pkill -INT -f "task1_score record" 2>/dev/null; sleep 1

docker cp "$CT:/tmp/t1_report.json" "$OUT/$LABEL.report.json" >/dev/null 2>&1 || echo '{}' > "$OUT/$LABEL.report.json"
docker cp "$CT:/tmp/t1_report.json.track.csv" "$OUT/$LABEL.track.csv" >/dev/null 2>&1
docker cp "$CT:/tmp/bt.log" "$OUT/$LABEL.bt.log" >/dev/null 2>&1
docker cp "$CT:/tmp/tt.log" "$OUT/$LABEL.tt.log" >/dev/null 2>&1
docker cp "$OUT/$LABEL.log" "$CT:/tmp/t1_run.log" >/dev/null 2>&1
docker cp "$OUT/$LABEL.report.json" "$CT:/tmp/t1_run.report.json" >/dev/null 2>&1
docker cp "$OUT/$LABEL.track.csv" "$CT:/tmp/t1_run.report.json.track.csv" >/dev/null 2>&1
docker exec "$CT" bash -c "$ROS && python3 -m crusader_sim.task1_score score --course $COURSE --log /tmp/t1_run.log --report /tmp/t1_run.report.json" \
  | tee "$OUT/$LABEL.score.txt"
# the picture on the WSL host: the image has no matplotlib, the host has (install_wsl.sh apt)
cp "$OUT/$LABEL.track.csv" "$OUT/$LABEL.report.json.track.csv" 2>/dev/null
PYTHONPATH="$HOME/robotx_ws/src/rx26_asv/crusader_sim" python3 -m crusader_sim.task1_score score \
  --course "$COURSE" --log "$OUT/$LABEL.log" --report "$OUT/$LABEL.report.json" --plot "$OUT/$LABEL.png" >/dev/null
rm -f "$OUT/$LABEL.report.json.track.csv"
docker exec "$CT" bash -c "$ROS && python3 -m crusader_sim.task1_score score --course $COURSE --log /tmp/t1_run.log --report /tmp/t1_run.report.json --json" \
  | python3 -c "import json,sys; r=json.loads(sys.stdin.read()); r.update(label='$LABEL', tree='$TREE', params='$*'.split()); print(json.dumps(r))" \
  >> "$OUT/summary.jsonl"
grep -E '^\[result\]|^\[judge\] VERDICT' "$OUT/$LABEL.log" | cut -c1-200
