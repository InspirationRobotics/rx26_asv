"""gui_view — a starting camera view for the Gazebo GUI on a dock course.

    python3 -m crusader_sim.gui_view <course.yaml>     # prints a gz.msgs.GUICamera request

Runs in: WSL (gz_sim_up.sh), no ROS. On a dock course the GUI camera is NOT locked onto
the boat (/gui/follow re-centres it every frame, so a zoom snaps straight back out);
instead it starts here, once: off to one side of the slips and out over the water,
looking at where the boat docks and sprays, with the approach in the frame. The person
watching then zooms and turns as they like.
"""
import math
import sys

from crusader_sim import course as C

BACK_M = 9.0     # out from the dock face, over the water
SIDE_M = 6.0     # to one side of the slips
UP_M = 4.5       # above the water
LOOK_OUT_M = 2.5 # look at the berth: this far out from the dock
LOOK_UP_M = 0.6


def quat_from_yaw_pitch(yaw, pitch):
    """(x, y, z, w) for a camera looking along +x, turned yaw about z, then pitched
    down by `pitch` (radians, + = looking down)."""
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    return (-sp * sy, sp * cy, cp * sy, cp * cy)


def view(course):
    """(camera position, (qx, qy, qz, qw)) for the course's dock, or None without one."""
    d = next((e for e in course["elements"] if e.get("type") == "dock"), None)
    if d is None:
        return None
    psi = math.radians(float(d.get("facing_deg", 180.0)))
    out = (math.cos(psi), math.sin(psi))
    side = (-out[1], out[0])
    tx, ty = float(d["x"]) + out[0] * LOOK_OUT_M, float(d["y"]) + out[1] * LOOK_OUT_M
    tz = LOOK_UP_M
    cx = tx + out[0] * BACK_M + side[0] * SIDE_M
    cy = ty + out[1] * BACK_M + side[1] * SIDE_M
    cz = UP_M
    yaw = math.atan2(ty - cy, tx - cx)
    pitch = math.atan2(cz - tz, math.hypot(tx - cx, ty - cy))
    return (cx, cy, cz), quat_from_yaw_pitch(yaw, pitch)


def request(course):
    """The text of a gz.msgs.GUICamera request for /gui/move_to/pose, or ""."""
    v = view(course)
    if v is None:
        return ""
    (x, y, z), (qx, qy, qz, qw) = v
    return (f"pose: {{position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, "
            f"orientation: {{x: {qx:.5f}, y: {qy:.5f}, z: {qz:.5f}, w: {qw:.5f}}}}}")


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print(__doc__.split("\n\n")[1].strip(), file=sys.stderr)
        return 2
    print(request(C.load(argv[0])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
