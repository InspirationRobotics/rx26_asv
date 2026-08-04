"""gate_navigator — GUIDED-mode buoy-gate transit (Mission 1). REWIRED port.

The operational gate_navigator owned its own MAVLink connection (mission download +
setpoint TX), its own OAK-D + YOLO + depth pipeline, and an MJPEG stream. This
version keeps the gate-pairing/midpoint/exit ALGORITHM but consumes this repo's
topics and actuates through the sanctioned GUIDED path:

  in:  /crsd/pose            (LatLonHead)      boat lat/lon/heading
       /crsd/fcu_status      (FcuStatus)       flight mode (act only in GUIDED)
       /crsd/detections_body (DetectionArray)  perception_node output (body frame)
  out: /crsd/guided_setpoint (GuidedSetpoint)  -> telemetry_bridge forwards it as
                             SET_POSITION_TARGET_GLOBAL_INT (latch-gated). This node
                             NEVER opens its own MAVLink connection.

WORKFLOW CHANGE: waypoints no longer come from a QGC mission download over MAVLink
(this node has no MAVLink link). They are read from `mission_file`
(JSON {"waypoints": [[lat,lon], ...]}), the same source mission_planner_node uses.
When gate_navigator is later wrapped as a mission-planner task (plan §3.4), the
planner supplies the waypoints instead.

Detections are body-frame positions (x=starboard+, y=forward+) with canonical
labels from class_map.json — so pairing uses geometry, not pixel coordinates as
the camera-coupled original did. Still UNTESTED end-to-end (CLAUDE.md): hardening
it against the fixed scenario suite is the Mission-1 priority.
"""
import json
import math
import time
from collections import deque
from pathlib import Path
from statistics import median

from rclpy.node import Node

from interfaces.msg import LatLonHead, FcuStatus, DetectionArray, GuidedSetpoint

from rx26_asv.api.common import config as crsd_config
from rx26_asv.api.common import geo
from rx26_asv.api.common.detection_input import DetectionInput
from rx26_asv.api.common.node_main import run_node
from rx26_asv.api.common.param_utils import declare_from_config
from rx26_asv.api.common.stream_cache import StreamCache

# Canonical perception labels (class_map.json): red/green flashing gate buoys.
RED_CLASSES = {"buoy_flash_red"}
GREEN_CLASSES = {"buoy_flash_green"}

# --- gate geometry: the operational tune, kept as constants (not per-run knobs) ---
CONF_MIN = 0.60             # confidence floor for a buoy to count in a gate pair
GATE_MIN_WIDTH = 1.0        # m
GATE_MAX_WIDTH = 8.0        # m
PAIR_MAX_DEPTH_DIFF = 5.0   # m; rejects unlikely red/green pairings
MAX_CORRECTION = 15.0       # m; midpoint must be this close to the planned waypoint
MIDPOINT_SHIFT_RATIO = 0.5  # shift target toward the RIGHT-side buoy by this fraction
PAIR_CONFIRM_FRAMES = 6
PAIR_SAMPLE_TIMEOUT = 0.60  # s; clear samples if detections stop being consecutive
PAIR_MAX_SPREAD = 1.00      # m; max world-position spread before locking
GATE_EXIT_DISTANCE = 0.2    # m beyond the gate for the exit target
MIDPOINT_SWITCH_RADIUS = 1.0
EXIT_ARRIVE_RADIUS = 1.0
NORMAL_WP_ARRIVE_RADIUS = 1.0

PARAM_SPEC = {
    "mission_file": dict(read_only=True,
                         description='JSON {"waypoints": [[lat,lon],...]}'),
    "gate_wp_indices": dict(read_only=True,
                            description="zero-based waypoint indices treated as gates"),
    "pose_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                           description="stale pose -> stop commanding setpoints"),
    "cmd_period_s": dict(read_only=True, lo=0.1, hi=5.0,
                         description="min seconds between guided setpoints"),
    "rate_hz": dict(read_only=True, lo=1.0, hi=50.0),
}


def dist_m(lat1, lon1, lat2, lon2):
    x, y = geo.latlon_to_xy(lat2, lon2, (lat1, lon1))
    return math.hypot(x, y)


class GateNavigator(Node):
    def __init__(self):
        super().__init__("gate_navigator")
        p = declare_from_config(self, crsd_config.node_params("gate_navigator"),
                                PARAM_SPEC)
        self.gate_wp_indices = set(p["gate_wp_indices"])
        self.cmd_period_s = p["cmd_period_s"]

        self.waypoints = self._load_waypoints(p["mission_file"])
        if not self.waypoints:
            raise RuntimeError(
                f"no waypoints in mission_file {p['mission_file']!r} — gate_navigator "
                "needs a JSON {\"waypoints\": [[lat,lon],...]}")
        if self.gate_wp_indices and max(self.gate_wp_indices) >= len(self.waypoints):
            raise RuntimeError(
                f"gate_wp_indices={sorted(self.gate_wp_indices)} but only "
                f"{len(self.waypoints)} waypoint(s) loaded")
        self.get_logger().info(f"loaded {len(self.waypoints)} waypoints; "
                               f"gates at {sorted(self.gate_wp_indices)}")

        self.setpoint_pub = self.create_publisher(GuidedSetpoint,
                                                  "/crsd/guided_setpoint", 10)
        self.create_subscription(LatLonHead, "/crsd/pose", self._pose_cb, 10)
        self.create_subscription(FcuStatus, "/crsd/fcu_status", self._fcu_cb, 10)
        # prefer fused (LiDAR range) when the fusion node is up, else camera-only
        self._det_input = DetectionInput(self, self._det_cb)

        self.lat = self.lon = self.heading = None
        # Gate midpoints come from rotating BODY detections through the boat
        # heading, so a stale heading aims the setpoint where the gate is not.
        self.pose_age = StreamCache(p["pose_timeout_s"])
        self.mode = ""
        self.wp_index = 0
        self.done = False

        # gate state
        self.gate_samples = deque(maxlen=PAIR_CONFIRM_FRAMES)
        self.last_pair_sample_time = 0.0
        self.gate_midpoint = None
        self.gate_exit = None
        self.gate_locked = False
        self.gate_clearing = False

        self.last_cmd = 0.0
        self.last_waiting_log = 0.0
        self.create_timer(1.0 / p["rate_hz"], self._tick)

    # ---------- waypoint source ----------

    @staticmethod
    def _load_waypoints(mission_file):
        """Parse the mission file, or raise naming the exact problem.

        Every failure here is FATAL by design. gate_wp_indices is positional, so
        a partially-loaded course renumbers which waypoints are gates — the node
        would hunt for a gate at the wrong place instead of failing. Same reason
        the .plan converter refuses to emit a short list.
        """
        if not mission_file:
            return []
        path = Path(mission_file)
        try:
            data = json.loads(path.read_text())
        except FileNotFoundError:
            raise RuntimeError(
                f"mission_file {path} does not exist. Generate one from a QGC "
                f".plan:\n  python3 tools/scripts/plan_to_mission.py "
                f"<course.plan> -o {path}")
        except json.JSONDecodeError as e:
            raise RuntimeError(f"mission_file {path} is not valid JSON: {e}")

        if not isinstance(data, dict) or "waypoints" not in data:
            raise RuntimeError(
                f'mission_file {path} has no "waypoints" key — expected '
                '{"waypoints": [[lat, lon], ...]}')

        waypoints = []
        for i, pair in enumerate(data["waypoints"]):
            try:
                lat, lon = (float(v) for v in pair)
            except (TypeError, ValueError):
                raise RuntimeError(
                    f"mission_file {path} waypoint {i} is not a [lat, lon] "
                    f"pair: {pair!r}")
            if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
                raise RuntimeError(
                    f"mission_file {path} waypoint {i} = ({lat}, {lon}) out of "
                    "range — lat/lon transposed?")
            waypoints.append((lat, lon))
        return waypoints

    # ---------- inputs ----------

    def _pose_cb(self, msg: LatLonHead):
        if math.isnan(msg.heading):
            return       # GPS yaw unresolved: a position without a heading
                         # cannot place a gate, and a STALE heading is worse
        self.lat, self.lon = msg.latitude, msg.longitude
        self.heading = msg.heading
        self.pose_age.set(True, time.monotonic())

    def _fcu_cb(self, msg: FcuStatus):
        self.mode = msg.mode

    def _det_cb(self, msg: DetectionArray):
        if (msg.frame != "body" or self.lat is None or self.heading is None
                or self.pose_age.get(time.monotonic()) is None
                or self.wp_index not in self.gate_wp_indices or self.gate_locked):
            return
        candidate = self._select_gate_candidate(msg.detections)
        if candidate is not None:
            self._record_gate_candidate(candidate)
        elif (self.gate_samples
              and time.time() - self.last_pair_sample_time > PAIR_SAMPLE_TIMEOUT):
            self.gate_samples.clear()

    # ---------- control ----------

    def _tick(self):
        mono = time.monotonic()
        if self.pose_age.went_stale(mono):
            self.get_logger().error(
                f"/crsd/pose stale ({self.pose_age.age(mono):.1f}s) — "
                "HOLDING: no further guided setpoints until pose returns")
        if (self.done or self.lat is None or self.heading is None
                or self.pose_age.get(mono) is None
                or self.mode != "GUIDED"):
            return
        if self.wp_index >= len(self.waypoints):
            self._finish()
            return

        target = self._current_target()
        if target is None:
            return
        distance = dist_m(self.lat, self.lon, *target)

        if self.wp_index in self.gate_wp_indices:
            if self.gate_locked:
                if not self.gate_clearing and distance < MIDPOINT_SWITCH_RADIUS:
                    self.gate_clearing = True
                    self.get_logger().info("gate midpoint reached; commanding exit")
                    target = self.gate_exit
                    distance = dist_m(self.lat, self.lon, *target)
                elif self.gate_clearing and distance < EXIT_ARRIVE_RADIUS:
                    self.get_logger().info(f"gate cleared at waypoint {self.wp_index + 1}")
                    self._advance()
                    return
            else:
                planned = self.waypoints[self.wp_index]
                if dist_m(self.lat, self.lon, *planned) < NORMAL_WP_ARRIVE_RADIUS:
                    now = time.time()
                    if now - self.last_waiting_log > 2.0:
                        self.get_logger().info(
                            "at planned gate waypoint; waiting for a stable "
                            "same-frame red/green detection")
                        self.last_waiting_log = now
        else:
            if distance < NORMAL_WP_ARRIVE_RADIUS:
                self.get_logger().info(f"waypoint {self.wp_index + 1} reached")
                self._advance()
                return

        now = time.time()
        if now - self.last_cmd >= self.cmd_period_s:
            self._send_target(*target)
            self.last_cmd = now

    def _current_target(self):
        if self.wp_index in self.gate_wp_indices and self.gate_locked:
            return self.gate_exit if self.gate_clearing else self.gate_midpoint
        return self.waypoints[self.wp_index]

    def _advance(self):
        self.wp_index += 1
        self._reset_gate_state()
        if self.wp_index >= len(self.waypoints):
            self._finish()

    def _finish(self):
        if self.done:
            return
        self.get_logger().info("mission complete — commanding final waypoint hold")
        self.done = True

    def _reset_gate_state(self):
        self.gate_samples.clear()
        self.last_pair_sample_time = 0.0
        self.gate_midpoint = None
        self.gate_exit = None
        self.gate_locked = False
        self.gate_clearing = False

    def _send_target(self, lat, lon):
        m = GuidedSetpoint()
        m.header.stamp = self.get_clock().now().to_msg()
        m.latitude, m.longitude = lat, lon
        m.yaw = float("nan")
        self.setpoint_pub.publish(m)
        if self.wp_index in self.gate_wp_indices and self.gate_locked:
            tag = "gate-exit" if self.gate_clearing else "gate-midpoint"
        else:
            tag = "planned"
        self.get_logger().info(
            f"WP {self.wp_index + 1}/{len(self.waypoints)} ({tag}) "
            f"dist={dist_m(self.lat, self.lon, lat, lon):.1f}m",
            throttle_duration_sec=1.0)

    # ---------- gate pairing (body-frame geometry) ----------

    def _select_gate_candidate(self, detections):
        reds, greens = [], []
        for d in detections:
            if d.confidence < CONF_MIN:
                continue
            item = (d.y, d.x)            # (forward, right) in BODY
            if d.label in RED_CLASSES:
                reds.append(item)
            elif d.label in GREEN_CLASSES:
                greens.append(item)
        if not reds or not greens:
            return None

        planned = self.waypoints[self.wp_index]
        best, best_score = None, float("inf")
        for red_fwd, red_right in reds:
            for grn_fwd, grn_right in greens:
                depth_diff = abs(red_fwd - grn_fwd)
                if depth_diff > PAIR_MAX_DEPTH_DIFF:
                    continue
                gate_fwd = grn_fwd - red_fwd
                gate_right = grn_right - red_right
                gate_width = math.hypot(gate_fwd, gate_right)
                if not GATE_MIN_WIDTH <= gate_width <= GATE_MAX_WIDTH:
                    continue

                mid_fwd = (red_fwd + grn_fwd) / 2.0
                mid_right = (red_right + grn_right) / 2.0
                # empirical: raw midpoint lands on the left buoy; shift toward the right one
                if red_right >= grn_right:
                    mid_fwd += MIDPOINT_SHIFT_RATIO * (red_fwd - grn_fwd)
                    mid_right += MIDPOINT_SHIFT_RATIO * (red_right - grn_right)
                else:
                    mid_fwd += MIDPOINT_SHIFT_RATIO * (grn_fwd - red_fwd)
                    mid_right += MIDPOINT_SHIFT_RATIO * (grn_right - red_right)
                if mid_fwd <= 0.3:
                    continue

                mid_world = self._body_to_latlon(mid_fwd, mid_right)
                correction = dist_m(*mid_world, *planned)
                if correction > MAX_CORRECTION:
                    continue

                # exit point: gate-perpendicular, pointing away from the boat
                nfwd, nright = -gate_right / gate_width, gate_fwd / gate_width
                if nfwd * mid_fwd + nright * mid_right < 0.0:
                    nfwd, nright = -nfwd, -nright
                exit_world = self._body_to_latlon(
                    mid_fwd + nfwd * GATE_EXIT_DISTANCE,
                    mid_right + nright * GATE_EXIT_DISTANCE)

                score = correction + 0.20 * depth_diff
                if score < best_score:
                    best_score = score
                    best = {"width": gate_width, "mid_world": mid_world,
                            "exit_world": exit_world, "mid_body": (mid_fwd, mid_right)}
        return best

    def _record_gate_candidate(self, candidate):
        now = time.time()
        if self.gate_samples and now - self.last_pair_sample_time > PAIR_SAMPLE_TIMEOUT:
            self.gate_samples.clear()
        self.last_pair_sample_time = now

        mid, exit_pt = candidate["mid_world"], candidate["exit_world"]
        self.gate_samples.append((mid[0], mid[1], exit_pt[0], exit_pt[1]))
        mf, mr = candidate["mid_body"]
        self.get_logger().info(
            f"gate pair: width={candidate['width']:.2f}m mid=({mf:.2f}m fwd, "
            f"{mr:+.2f}m right) samples={len(self.gate_samples)}/{PAIR_CONFIRM_FRAMES}",
            throttle_duration_sec=0.5)
        if len(self.gate_samples) < PAIR_CONFIRM_FRAMES:
            return

        mid_lat = median(s[0] for s in self.gate_samples)
        mid_lon = median(s[1] for s in self.gate_samples)
        exit_lat = median(s[2] for s in self.gate_samples)
        exit_lon = median(s[3] for s in self.gate_samples)
        mid_spread = max(dist_m(mid_lat, mid_lon, s[0], s[1]) for s in self.gate_samples)
        exit_spread = max(dist_m(exit_lat, exit_lon, s[2], s[3]) for s in self.gate_samples)
        if max(mid_spread, exit_spread) > PAIR_MAX_SPREAD:
            self.get_logger().warning(
                f"gate estimate unstable: mid spread={mid_spread:.2f}m "
                f"exit spread={exit_spread:.2f}m")
            return

        self.gate_midpoint = (mid_lat, mid_lon)
        self.gate_exit = (exit_lat, exit_lon)
        self.gate_locked = True
        self.gate_clearing = False
        self.get_logger().info(
            f"gate locked: midpoint=({mid_lat:.7f}, {mid_lon:.7f}) "
            f"exit=({exit_lat:.7f}, {exit_lon:.7f})")

    def _body_to_latlon(self, forward_m, right_m):
        heading_rad = math.radians(self.heading)
        wx, wy = geo.body_to_world(right_m, forward_m, 0.0, 0.0, heading_rad)
        return geo.xy_to_latlon(wx, wy, (self.lat, self.lon))


def main(args=None):
    run_node(GateNavigator, args=args)


if __name__ == "__main__":
    main()
