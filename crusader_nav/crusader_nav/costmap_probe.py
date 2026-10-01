"""costmap_probe - what the costmap thinks is lethal, as lat/lon. Read-only.

    ros2 run crusader_nav costmap_probe [--max-range 40]

Run it inside the same container as nav.launch.py (asv on the boat, crsd-sim in
the sim). It is also the bench and water check of docs/nav2_avoidance_spec.md
10.3 step 0: rotate the boat by hand near a fixed object and the object's
lat/lon must not move by more than 0.3 m. That catches yaw-sign and mirror
errors, which are silent.

Once a second it prints one line per connected blob of LETHAL cells (value 100
in the published grid: only HazardLayer and STVL mark lethal, the inflation
layer stops at 99) within --max-range of the boat:

    blob  lat 1.2806123  lon 103.8557123  range 12.3 m  bearing 045 deg  size 0.6 m

Blanks over guesses: with no datum there is no lat/lon, so it says so and prints
nothing; with a stale or missing pose the blob is still located (the grid and
the datum are enough) but range and bearing print as '-'.

The grid arrives whole when the rolling window moves and as patches
(/global_costmap/costmap_updates) otherwise; both are applied.
"""
import argparse
import math
import time

from crusader_nav import frames_core as fc

LETHAL = 100                 # nav_msgs/OccupancyGrid value for cost 254
POSE_FRESH_S = 1.0           # older than this and range/bearing are blank


def find_blobs(data, width, height, res, ox, oy, threshold=LETHAL):
    """Connected (8-neighbour) blobs of cells >= threshold in a row-major grid.

    Returns [(cx, cy, size_m, n_cells)] with the centroid in the grid's frame
    (cell centres, origin (ox, oy) = lower-left corner of cell (0, 0)) and
    size_m the longer side of the bounding box. Pure: no numpy needed.
    """
    marked = {k for k, v in enumerate(data) if v >= threshold}
    blobs = []
    while marked:
        stack = [marked.pop()]
        cells = []
        while stack:
            k = stack.pop()
            cells.append(k)
            i, j = k % width, k // width
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    ni, nj = i + di, j + dj
                    if 0 <= ni < width and 0 <= nj < height:
                        nk = nj * width + ni
                        if nk in marked:
                            marked.remove(nk)
                            stack.append(nk)
        xs = [k % width for k in cells]
        ys = [k // width for k in cells]
        cx = ox + (sum(xs) / len(xs) + 0.5) * res
        cy = oy + (sum(ys) / len(ys) + 0.5) * res
        size = max(max(xs) - min(xs) + 1, max(ys) - min(ys) + 1) * res
        blobs.append((cx, cy, size, len(cells)))
    return blobs


def blob_line(blob, datum, boat_xy):
    """One output line. boat_xy is the boat in map metres, or None (no fresh pose)."""
    cx, cy, size, _ = blob
    lat, lon = fc.to_latlon(cx, cy, *datum)
    if boat_xy is None:
        rng = brg = "-"
    else:
        dx, dy = cx - boat_xy[0], cy - boat_xy[1]
        rng = f"{math.hypot(dx, dy):.1f}"
        brg = f"{fc.bearing_deg(dx, dy):03.0f}"
    return (f"blob  lat {lat:.7f}  lon {lon:.7f}  range {rng} m  bearing {brg} deg  "
            f"size {size:.1f} m")


def main(args=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--max-range", type=float, default=40.0,
                    help="only blobs this close to the boat [m] (default 40)")
    opts, ros_args = ap.parse_known_args(args)

    import rclpy
    from map_msgs.msg import OccupancyGridUpdate
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    from crusader_msgs.msg import LatLonHead

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class Probe(Node):
        def __init__(self):
            super().__init__("costmap_probe")
            self.grid = None                 # [msg, mutable list of cells, t_rx]
            self.datum = None
            self.pose = None                 # (lat, lon, t_rx)
            self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                     self._on_grid, latched)
            self.create_subscription(OccupancyGridUpdate, "/global_costmap/costmap_updates",
                                     self._on_update, 10)
            self.create_subscription(LatLonHead, "/crsd/datum", self._on_datum, latched)
            self.create_subscription(LatLonHead, "/crsd/pose", self._on_pose, 10)
            self.create_timer(1.0, self._tick)

        def _on_grid(self, m):
            self.grid = [m, list(m.data), time.monotonic()]

        def _on_update(self, u):
            if self.grid is None:
                return
            m, cells = self.grid[0], self.grid[1]
            w = m.info.width
            for row in range(u.height):
                a = (u.y + row) * w + u.x
                cells[a:a + u.width] = list(u.data[row * u.width:(row + 1) * u.width])
            self.grid[2] = time.monotonic()

        def _on_datum(self, m):
            self.datum = (m.latitude, m.longitude)

        def _on_pose(self, m):
            self.pose = (m.latitude, m.longitude, time.monotonic())

        def _tick(self):
            if self.datum is None:
                print("waiting for /crsd/datum (is nav_frames_node running?)", flush=True)
                return
            if self.grid is None:
                print("waiting for /global_costmap/costmap (is planner_server active?)",
                      flush=True)
                return
            m, cells, t_rx = self.grid
            now = time.monotonic()
            boat = None
            if self.pose is not None and fc.is_fix(self.pose[0], self.pose[1]) \
                    and now - self.pose[2] <= POSE_FRESH_S:
                boat = fc.to_local(self.pose[0], self.pose[1], *self.datum)
            blobs = find_blobs(cells, m.info.width, m.info.height, m.info.resolution,
                               m.info.origin.position.x, m.info.origin.position.y)
            if boat is not None:
                blobs = [b for b in blobs
                         if math.hypot(b[0] - boat[0], b[1] - boat[1]) <= opts.max_range]
            blobs.sort(key=lambda b: (math.hypot(b[0] - boat[0], b[1] - boat[1])
                                      if boat else b[0]))
            print(f"-- {len(blobs)} lethal blob(s), costmap age {now - t_rx:.1f} s, "
                  f"boat {'fresh' if boat else 'NO FRESH POSE'}", flush=True)
            for b in blobs:
                print(blob_line(b, self.datum, boat), flush=True)

    rclpy.init(args=ros_args)
    node = Probe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
