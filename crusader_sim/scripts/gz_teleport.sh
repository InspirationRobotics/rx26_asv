#!/usr/bin/env bash
# gz_teleport.sh — move the sim boat instantly. SIM ONLY. Runs in: WSL2 (where Gazebo runs).
#
#     bash gz_teleport.sh                       back to the course's start (= start)
#     bash gz_teleport.sh start                 the same
#     bash gz_teleport.sh LAT LON [HEADING]     to a GPS point; HEADING in compass degrees
#                                               (0 = north, 90 = east), default: keep the bow
#
# The start is the running world's own crusader pose (gen_world.py: the course's boat_start);
# lat/lon are turned into Gazebo metres from the world's <spherical_coordinates> with the same
# 111318.845 m/deg SITL uses (course.enu_to_latlon), so the boat's GPS reads the point given.
# GZ_SIM_DOWN/UP is the full reset; this only moves the hull. The autopilot sees its GPS jump:
# give its position estimate ~10-30 s to settle before starting a mission, and do not teleport
# while one is running (the leg it is on would steer from the new spot).
set -u
export GZ_PARTITION=crusader_sim
die() { echo "gz_teleport: $*" >&2; exit 2; }

WORLD_FILE="$(pgrep -af "gz sim -s" | grep -o '/[^ ]*\.sdf' | head -1)"
[ -n "$WORLD_FILE" ] || die "no Gazebo server running (start the sim first)"
[ "$(pgrep -fc "gz sim -s")" -le 1 ] || die "more than one Gazebo server is running: GZ_SIM_DOWN, then GZ_SIM_UP once"
WORLD="$(grep -o '<world name="[^"]*"' "$WORLD_FILE" | head -1 | cut -d'"' -f2)"
POSE_NOW="$(timeout 5 gz topic -e -n 1 -t "/world/$WORLD/pose/info" 2>/dev/null)"

REQ="$(python3 - "$WORLD_FILE" "$POSE_NOW" "$@" <<'PY'
import math, re, sys
world_file, pose_now, args = sys.argv[1], sys.argv[2], sys.argv[3:]
sdf = open(world_file).read()
M_PER_DEG = 0.011131884502145034 * 1e7          # = crusader_sim.course.EARTH_M_PER_DEG
lat0 = float(re.search(r"<latitude_deg>([-\d.]+)", sdf).group(1))
lon0 = float(re.search(r"<longitude_deg>([-\d.]+)", sdf).group(1))
start = re.search(r"<name>crusader</name>\s*<pose>([^<]+)</pose>", sdf)
sx, sy, sz, _r, _p, syaw = (float(v) for v in start.group(1).split())

def yaw_now():
    m = re.search(r'name: "crusader".*?orientation \{(.*?)\}', pose_now, re.S)
    if not m:
        return syaw
    q = {k: float(v) for k, v in re.findall(r"([xyzw]): ([-\d.e]+)", m.group(1))}
    return math.atan2(2 * (q.get("w", 1) * q.get("z", 0) + q.get("x", 0) * q.get("y", 0)),
                      1 - 2 * (q.get("y", 0) ** 2 + q.get("z", 0) ** 2))

if not args or args[0].lower() == "start":
    x, y, yaw, what = sx, sy, syaw, "the course start"
else:
    if len(args) < 2:
        sys.exit("gz_teleport: give LAT LON [HEADING], or start")
    lat, lon = float(args[0]), float(args[1])
    x = (lon - lon0) * M_PER_DEG * math.cos(math.radians(lat0))
    y = (lat - lat0) * M_PER_DEG
    if math.hypot(x, y) > 1000:
        sys.exit(f"gz_teleport: {lat}, {lon} is {math.hypot(x, y):.0f} m from the course: a typo?")
    yaw = math.radians(90.0 - float(args[2])) if len(args) > 2 else yaw_now()
    what = f"{lat:.7f}, {lon:.7f}"
heading = (90.0 - math.degrees(yaw)) % 360.0
print(f"# {what}: x {x:.2f} m E, y {y:.2f} m N, heading {heading:.0f} deg", file=sys.stderr)
print(f'name: "crusader", position: {{x: {x:.4f}, y: {y:.4f}, z: {sz:.4f}}}, '
      f"orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}")
PY
)" || exit 2

OUT="$(gz service -s "/world/$WORLD/set_pose" --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
        --timeout 3000 --req "$REQ" 2>&1)"
echo "$OUT" | grep -q "data: true" || die "Gazebo refused the move: $OUT"
echo "moved. Give the autopilot ~10-30 s before starting a mission."
