"""nav_frames_node - owns the datum and the map -> base_footprint transform.

docs/nav2_avoidance_spec.md sections 2 and 3.2.

Subscribes:
  /crsd/pose            crusader_msgs/LatLonHead  - WGS84, heading in compass degrees
Publishes:
  /crsd/datum           crusader_msgs/LatLonHead  - the datum, ONCE, latched
  /tf                   map -> base_footprint     - at each new pose stamp, finite heading only
  /crsd/nav/frames_health  std_msgs/String (JSON) - 1 Hz, for humans

THE ONE RULE. The TF frame `map`, bt_runner's east/north frame and every
HazardArray coordinate are the same equirectangular ENU plane centred on one
datum (frames_core.to_local). This node is the only place the datum is chosen;
bt_runner adopts it from /crsd/datum, so there is no second origin to disagree.

WHY NO TRANSFORM WITHOUT A HEADING. A NaN heading means GPS yaw is unresolved,
and a transform with a made-up yaw would place every LiDAR point wrongly while
looking perfectly healthy. No transform makes STVL's tf2 filter drop the cloud
and the costmap go non-current, which holds the boat (blanks over guesses).

WHY THE STAMP GATE. /crsd/pose repeats a stamp until the next MAVLink sample;
tf2 rejects repeated stamps, so only strictly increasing stamps are sent.

LAUNCH. nav.launch.py respawns this node only when the datum comes from
parameters: a respawn with `first_fix` would pick a NEW datum and move the map.
"""
import json
import math
import time

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from crusader_msgs.msg import LatLonHead

from crusader_common import config as crsd_config
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config, make_set_callback

from crusader_nav import frames_core as fc

DATUM_SOURCES = ("param", "first_fix")

# [RO] = structural: change the YAML (or the launch argument) and restart.
# [DYN] = `ros2 param set`, range-validated.
PARAM_SPEC = {
    "datum_source": dict(read_only=True, choices=DATUM_SOURCES,
                         description="param: datum_lat/datum_lon; first_fix: first /crsd/pose"),
    "datum_lat": dict(read_only=True, lo=-90.0, hi=90.0,
                      description="datum latitude [deg], datum_source=param"),
    "datum_lon": dict(read_only=True, lo=-180.0, hi=180.0,
                      description="datum longitude [deg], datum_source=param"),
    "pose_topic": dict(read_only=True, description="LatLonHead in"),
    "datum_topic": dict(read_only=True, description="LatLonHead out, latched"),
    "map_frame": dict(read_only=True, description="TF parent frame"),
    "base_frame": dict(read_only=True, description="TF child frame: levelled base_link"),
    "health_period_s": dict(read_only=False, lo=0.1, hi=60.0),
}
DYNAMIC_RANGES = {k: (v["lo"], v["hi"]) for k, v in PARAM_SPEC.items()
                  if not v["read_only"] and "lo" in v}


def datum_config_error(source, lat, lon):
    """Why this (source, lat, lon) cannot start a node, or None. Pure."""
    if source not in DATUM_SOURCES:
        return f"datum_source {source!r} is not one of {DATUM_SOURCES}"
    if source == "param" and not fc.is_fix(lat, lon):
        # (0, 0) is a real place and the default is not a datum: a node that
        # started anyway would put the whole map in the Gulf of Guinea.
        return (f"datum_source is 'param' but datum_lat/datum_lon = ({lat}, {lon}) is "
                "not a usable position; set them in the launch arguments or the YAML")
    return None


def _datum_msg(lat, lon, frame):
    m = LatLonHead()
    m.header.frame_id = frame
    m.latitude, m.longitude = float(lat), float(lon)
    m.heading = math.nan             # a datum has a position and no heading
    m.ground_speed = 0.0
    return m


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class NavFramesNode(Node):

    def __init__(self):
        super().__init__("nav_frames_node")
        p = declare_from_config(self, crsd_config.node_params("nav_frames_node"), PARAM_SPEC)
        err = datum_config_error(p["datum_source"], p["datum_lat"], p["datum_lon"])
        if err:
            raise ValueError(f"nav_frames_node: {err}")
        self._source = p["datum_source"]
        self._map = p["map_frame"]
        self._base = p["base_frame"]
        self._datum = None                        # (lat, lon), set once, never changed
        self._gate = fc.StampGate()
        self._stats = fc.TfStats(time.monotonic())

        self._tf = TransformBroadcaster(self)
        self._datum_pub = self.create_publisher(
            LatLonHead, p["datum_topic"],
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._health_pub = self.create_publisher(String, "/crsd/nav/frames_health", 10)
        self.create_subscription(LatLonHead, p["pose_topic"], self._on_pose, 10)
        self._health_timer = self.create_timer(p["health_period_s"], self._health)
        self.add_on_set_parameters_callback(
            make_set_callback(self, DYNAMIC_RANGES, self._apply))

        if self._source == "param":
            self._set_datum(p["datum_lat"], p["datum_lon"])
        else:
            self.get_logger().info("waiting for the first /crsd/pose fix to pin the datum")

    def _apply(self, changes):
        if "health_period_s" in changes:
            self._health_timer.cancel()
            self._health_timer = self.create_timer(changes["health_period_s"], self._health)

    def _set_datum(self, lat, lon):
        self._datum = (lat, lon)
        self._datum_pub.publish(_datum_msg(lat, lon, self._map))
        self.get_logger().info(
            f"datum {lat:.7f}, {lon:.7f} from {self._source}: frame '{self._map}' "
            f"-> '{self._base}'")

    def _on_pose(self, msg: LatLonHead):
        if self._datum is None:
            if not fc.is_fix(msg.latitude, msg.longitude):
                return                            # no position yet, not a datum
            self._set_datum(msg.latitude, msg.longitude)
        if not math.isfinite(msg.heading):
            self._stats.note_nan_heading()
            return                                # no heading, no transform
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        if not self._gate.accept(_stamp_ns(msg.header.stamp)):
            return
        x, y = fc.to_local(msg.latitude, msg.longitude, *self._datum)
        qx, qy, qz, qw = fc.quat_from_yaw(fc.yaw_from_heading(msg.heading))
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._map
        t.child_frame_id = self._base
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.translation.z = 0.0
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self._tf.sendTransform(t)
        self._stats.note_sent(time.monotonic())

    def _health(self):
        hz, age, nan_n = self._stats.snapshot(time.monotonic())
        out = String()
        out.data = json.dumps({
            "datum": list(self._datum) if self._datum else None,
            "source": self._source,
            "tf_hz": round(hz, 2),
            "last_tf_age_s": None if age is None else round(age, 2),
            "nan_heading": nan_n,
        })
        self._health_pub.publish(out)


def main(args=None):
    run_node(NavFramesNode, args)


if __name__ == "__main__":
    main()
