#!/usr/bin/env python3
"""bench_world_model.py — an invented buoy field around the REAL boat, so the
world model can be driven before the camera and LiDAR are trustworthy.

    # on the boat, with core.launch.py already running:
    ros2 run crusader_world_model target_tracker      # terminal 1
    ros2 run crusader_world_model map_server          # terminal 2
    python3 tools/bench/bench_world_model.py          # terminal 3

DEFAULT MODE USES THE REAL VESSEL. It subscribes to `/crsd/pose` and
`/crsd/attitude` and publishes only the two SENSOR topics, computing what the
camera and LiDAR would have reported for a field of buoys that isn't there.
That is a far better test than a simulated boat: the pose carries real RTK
noise, the yaw is the real moving-baseline solution with its real latency, and
the roll and pitch are the real hull in the real water. A synthetic circle
tests the tracker against a boat that moves perfectly, which is the one kind of
boat it will never see.

It also means the bench does NOT contend with `telemetry_bridge` for
`/crsd/pose`. Two publishers on that topic interleave, and the tracker sees the
boat teleporting between two positions — which is why the simulated-vessel mode
now has to be asked for by name.

    --sim-pose      invent the vessel too: a boat circling the field, publishing
                    /crsd/pose, /crsd/attitude and /crsd/fcu_status itself. For
                    a DESK run with no boat and no telemetry_bridge. Do not use
                    it while the core stack is up.

WHERE THE BUOYS GO. The field is anchored at the boat's FIRST FIX and rotated
to its heading at that instant, so the buoys are laid out AHEAD OF THE BOW
wherever the boat happens to be. Anchoring to fixed lat/lon would put the field
in Florida while the boat sits in a car park, and nothing would ever come into
view. The resulting true lat/lon of every buoy is printed once, at anchor time.

THE GROUND TRUTH IS PRINTED. That is what makes this a check rather than a
demo: the tracker's output is a number you can compare against a number you
chose. A track that settles within a metre of its truth row, keeps its id, and
does not split in two as the boat swings past it, is a tracker that works.

Publishes (topic names READ FROM crusader_params.yaml, never restated here):
  target_tracker.detections_topic  10 Hz  camera_link, EVERY frame, empty or not
  target_tracker.clusters_topic     2 Hz  lidar_cluster_node's accumulation windows
and, ONLY under --sim-pose:
  /crsd/pose          20 Hz   /crsd/attitude 30 Hz   /crsd/fcu_status 1 Hz

WHAT IT CANNOT PROVE. The body/world rotation here is
crusader_common.geo.world_to_body_ypr, the exact transpose of the one the
tracker runs — so a sign error shared by both cancels and this bench will show
a perfect map anyway. It proves the plumbing, the association, the decay, the
fusion arbitration and the display. It does NOT prove the frame convention, and
it never will; that is a bench exercise against real hardware, the way
docs/G2_lidar_orientation.md did it for the LiDAR.

REQUIREMENTS: rclpy + crusader_msgs + crusader_common + crusader_world_model —
i.e. the `asv` container, or any sourced workspace. No numpy, no cv2, no device
SDKs, and no camera or LiDAR.
"""
import argparse
import math
import random
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from crusader_msgs.action import SafePassage

from crusader_msgs.msg import (Attitude, Cluster3D, Cluster3DArray, Detection3D,
                               Detection3DArray, FcuStatus, GuidedSetpoint,
                               LatLonHead)

from crusader_common import config as crsd_config
from crusader_common import geo
from crusader_common.stream_cache import StreamCache

from crusader_world_model import target_tracker_core as core

# A sibling file, not a package: running this script puts tools/bench on
# sys.path, which is the only reason a bare name works here.
import field_gui                                       # noqa: E402
import uav_link                                        # noqa: E402

# label -> RXL beacon state, for the --uav radio path. The labels are the ones
# field_gui's palette places and beaconFromLabel() parses; this is the third
# consumer of the same five strings, so it is a lookup rather than a parse.
BEACON_OF = {
    "flashing_blue_buoy": 4,     # rxl_codec.BEACON_FLASHING_BLUE  (ENTRY)
    "red_buoy": 2,               # BEACON_FLASHING_RED
    "green_buoy": 3,             # BEACON_FLASHING_GREEN
    "black_buoy": 1,             # BEACON_OFF
    "steady_blue_buoy": 5,       # BEACON_STEADY_BLUE  (EXIT)
}
ENTRY_LABEL = "flashing_blue_buoy"
EXIT_LABEL = "steady_blue_buoy"

# Fallback origin for --sim-pose only: St Petersburg, FL — RoboNation's water.
# In the default (real vessel) mode the origin is the boat's own first fix and
# this is never used.
SIM_ORIGIN = (27.7745, -82.6320)

# The buoy field, in metres RELATIVE TO THE BOAT AT ANCHOR TIME:
#   right  = + to starboard of where the bow was pointing
#   ahead  = + along the bow
# A gate to run, a pair of channel markers beyond it, and one unlabelled object
# the camera will never name — that last one is there to prove an anonymous
# LiDAR cluster still reaches the map instead of being quietly filtered for
# having no label.
#
# TWO FIELDS, chosen with --field. "tracker" is the original and stays the
# default: changing what an existing bench invents would silently change what
# every previous run of it meant.
#
# Labels are not free text. bt_runner_node's beaconFromLabel() matches on
# substrings, in this order: red, green, blue, off/black. So "flashing_blue"
# and "steady_blue" are the only way to say ENTRY and EXIT, and a label may
# never contain two colour words.
FIELDS = {
    # The original: a gate to run, a pair of channel markers beyond it, and one
    # unlabelled object the camera will never name — that last one is there to
    # prove an anonymous LiDAR cluster still reaches the map instead of being
    # quietly filtered for having no label.
    "tracker": [
        #  label          right  ahead
        ("red_buoy",       -3.0, 25.0),
        ("green_buoy",      3.0, 25.0),
        ("red_buoy",      -12.0, 48.0),
        ("green_buoy",     -4.0, 52.0),
        ("yellow_buoy",    18.0, 35.0),
        ("",               22.0, 60.0),   # LiDAR sees it; the camera never will
    ],

    # Task 1 Safe Passage, handbook 3.3.2: ten buoys, one flashing-blue ENTRY,
    # one steady-blue EXIT, RED passed to STARBOARD and GREEN to PORT, plus two
    # unlit BLACK buoys that carry no side constraint and are pure obstacles.
    #
    # RED sits to +right and GREEN to -right because the boat transits along
    # +ahead: that is what puts red on its starboard hand. Mirror the two
    # columns and the field becomes a test that the boat drives the wrong side
    # of every buoy, which is worth doing deliberately and never by accident.
    #
    # 92 m from the anchor to the EXIT buoy. The ENTRY sits at 20 m so it is
    # inside CAM_MAX_M at the moment the field anchors — the rest is discovered
    # on the way, which is the honest version of the problem.
    "task1": [
        #  label                right  ahead
        ("flashing_blue_buoy",    0.0, 20.0),   # ENTRY — circle it CLOCKWISE
        ("red_buoy",             +6.0, 38.0),
        ("green_buoy",           -6.0, 38.0),
        ("black_buoy",          +14.0, 45.0),   # unlit: obstacle, either side
        ("red_buoy",             +7.0, 56.0),
        ("green_buoy",           -5.0, 56.0),
        ("black_buoy",          -16.0, 62.0),   # unlit: obstacle, either side
        ("red_buoy",             +5.0, 74.0),
        ("green_buoy",           -7.0, 74.0),
        ("steady_blue_buoy",      0.0, 92.0),   # EXIT — circle it ANTICLOCKWISE
    ],
}

# Height of a buoy's centre above the hull-bottom datum that base_link sits on
# [m]: roughly the placeholder waterline (0.10) plus what floats above it. Not
# zero, because zero would put the buoys exactly in the base_link plane and let
# a wrong cam_z pass unnoticed.
BUOY_Z = 0.25

CAM_FOV_DEG = 40.0       # OAK-D half-angle, roughly
CAM_MAX_M = 25.0         # = buoy_detector.range_max_m
LIDAR_FOV_DEG = 90.0     # = lidar_cluster_node.fov_deg / 2
LIDAR_MAX_M = 40.0       # = lidar_cluster_node.r_max

# Sensor error models, deliberately DIFFERENT in character, because that
# asymmetry is the entire argument for fusing them. The camera's bearing is
# excellent and its range is a biased, range-dependent mess; the LiDAR's range
# is excellent and it knows nothing about what it hit.
CAM_BEARING_SD = 0.004   # rad — sub-quarter-degree
CAM_RANGE_BIAS = 0.06    # fraction of range, systematic (stereo baseline error)
CAM_RANGE_SD = 0.04      # fraction of range, random
LIDAR_RANGE_SD = 0.03    # m
LIDAR_MISS_P = 0.15      # windows where a small target returns nothing

POSE_TIMEOUT_S = 1.0     # = shared.pose_timeout_s


class WorldModelBench(Node):
    """Invents the two sensors. Invents the vessel too, only under --sim-pose."""

    def __init__(self, args):
        super().__init__("bench_world_model")
        self.args = args
        self.t0 = time.monotonic()
        random.seed(args.seed)

        # The camera extrinsic is READ FROM crusader_params.yaml, not restated
        # here — the same discipline tools/lidar_view.py follows for the LiDAR's.
        # A bench that models the mount differently from the node under test
        # passes and fails for reasons that have nothing to do with the code
        # being checked, which is worse than not checking at all.
        p = crsd_config.node_params("target_tracker")
        self.extrinsic = core.TrackerParams(
            cam_x=p["cam_x"], cam_y=p["cam_y"], cam_z=p["cam_z"],
            cam_yaw_deg=p["cam_yaw_deg"], cam_pitch_deg=p["cam_pitch_deg"])

        self.origin = None          # (lat, lon) of the field frame's zero
        self.field = []             # [(label, east, north)] once anchored
        self.anchor_heading = 0.0   # rad, the bow at anchor time
        # self.field is read by the synthesis timers on the executor thread and
        # rewritten by the GUI's HTTP thread. It is only ever REPLACED whole,
        # never mutated in place, and both sides take this lock around that one
        # assignment — so a reader always iterates a complete field, never one
        # caught halfway through an edit.
        self._field_lock = threading.Lock()
        self._nan_heading = 0       # fixes dropped for unresolved GPS yaw

        # The radio, under --uav. This bench then plays BOTH halves of the
        # world: the boat's own sensors (camera + LiDAR, as always) and the
        # aircraft's transmissions. Running both at once is the honest
        # Disruptive setup and is what exercises nav::fusePassage.
        # The mission action, so the page can start and stop a run without
        # anybody opening a terminal. This is a CLIENT of bt_runner_node's
        # server; the bench never decides anything about the mission itself.
        self.mission = {"state": "idle", "phase": "", "progress": 0.0,
                        "outcome": None, "detail": "", "plan_version": 0,
                        "buoys_known": 0, "elapsed_s": 0.0}
        self._goal_handle = None
        self.action = ActionClient(self, SafePassage, "/crsd/safe_passage")

        # Putting the simulated boat back on the start line, so a second run
        # does not mean restarting seven nodes.
        self.reset = {"state": "idle", "detail": ""}
        self._sp_pub = self.create_publisher(
            GuidedSetpoint, "crsd/guided_setpoint", 10)

        self.uav = None
        if getattr(args, "uav", False):
            self.uav = uav_link.UavLink(args.uav_endpoint)
            # 20 Hz: the boat asks for a gate once per leg, so this only has to
            # be faster than a person notices, not faster than the radio.
            self.create_timer(0.05, lambda: self.uav.poll())
            # AND RETRANSMIT, the way a real aircraft does. /crsd/passage_plan
            # is not latched -- deliberately, because a latched topic would hand
            # a late subscriber a stale plan stamped with the moment it arrived,
            # which is exactly the frozen-world lie the staleness check exists to
            # catch. So the aircraft repeats itself instead, and a bt_runner that
            # starts after the operator clicked Transmit picks the passage up
            # within one period. Clicking Transmit once and getting a tree that
            # fails on its first tick is what this fixes.
            self.create_timer(1.0 / 0.2, lambda: self.uav.resend())
        self._warned_no_pose = False

        # The TOPIC NAMES come from the same params file as the extrinsic, and
        # for the same reason. They were hardcoded here as "oak/detections"
        # while target_tracker subscribed "crsd/oak/detections", so the bench
        # published into a topic nothing read: the tracker kept publishing an
        # EMPTY target array at its usual 10 Hz, which looks exactly like a
        # bench that is running and a tracker that sees nothing.
        self.det_pub = self.create_publisher(Detection3DArray,
                                             p["detections_topic"], 10)
        self.clu_pub = self.create_publisher(Cluster3DArray,
                                             p["clusters_topic"], 10)
        self.create_timer(1 / 10.0, self._detections)
        self.create_timer(1 / 2.0, self._clusters)

        if args.sim_pose:
            self._start_simulated_vessel()
        else:
            self._start_real_vessel()

    # ---------- vessel: the real one ----------

    def _start_real_vessel(self):
        """Subscribe to the boat. Publish nothing about it."""
        self._pose = StreamCache(POSE_TIMEOUT_S)
        self._att = StreamCache(POSE_TIMEOUT_S)
        self.create_subscription(LatLonHead, "/crsd/pose", self._on_pose, 10)
        self.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)
        self.create_timer(5.0, self._waiting_check)
        self.get_logger().info(
            "using the REAL vessel: /crsd/pose + /crsd/attitude. Publishing "
            f"only {self.det_pub.topic_name} and {self.clu_pub.topic_name}. "
            "The buoy field will be anchored ahead of the bow at the first fix.")

    def _on_pose(self, msg: LatLonHead):
        """Cache the fix, anchoring the field on the first usable one.

        A NaN heading is GPS yaw unresolved, and the field's orientation is
        defined by the heading at anchor time — so an unresolved first fix must
        not be allowed to anchor it. Dropped rather than defaulted to north:
        laying the field out on a guessed heading puts the buoys somewhere the
        boat may never look, and the ground-truth table would be wrong.
        """
        if math.isnan(msg.heading):
            self._nan_heading += 1
            return
        if self.origin is None:
            self._anchor(msg.latitude, msg.longitude,
                         math.radians(msg.heading))
        e, n = geo.latlon_to_xy(msg.latitude, msg.longitude, self.origin)
        self._pose.set((e, n, math.radians(msg.heading), msg.ground_speed),
                       time.monotonic())

    # ---------- field access, and the GUI's two callables ----------

    def _snapshot(self):
        """The field as it is right now. Callers iterate this, not self.field."""
        with self._field_lock:
            return list(self.field)

    # --------------------------------------------------------- SITL reset

    def _return_to_start(self):
        """Drive the boat back to the anchor so the next run starts where the
        last one did.

        NOT a reboot. MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN was the obvious thing
        and it does not work: tried 2026-09-13, the autopilot restarted and
        re-armed but the simulated vehicle STAYED where it was -- reported "on
        the start line" from 64 m up the field, which is worse than failing. A
        genuine position reset means restarting the SITL process itself, and
        that lives on the WSL host where this container cannot reach it
        (tools/sitl/start_sitl.sh, by hand).

        So this drives home under GUIDED instead, using the same setpoint path
        the mission uses. Publishing on /crsd/guided_setpoint while bt_runner is
        also publishing would make the boat argue with itself, which is why this
        refuses to run while a mission is active.
        """
        if self.mission["state"] in ("active", "starting", "cancelling"):
            raise RuntimeError("cancel the mission first")
        if self.reset["state"] == "running":
            raise RuntimeError("already on the way")
        if self.origin is None:
            raise RuntimeError("not anchored yet")
        self.reset = {"state": "running", "detail": "driving back"}
        threading.Thread(target=self._return_worker, daemon=True).start()
        return "returning to the start line"

    def _return_worker(self):
        try:
            lat, lon = self.origin          # the anchor IS the start line
            deadline = time.monotonic() + 180.0
            arrived_within = 3.0
            while time.monotonic() < deadline:
                sp = GuidedSetpoint()
                sp.header.stamp = self.get_clock().now().to_msg()
                sp.latitude, sp.longitude = lat, lon
                sp.yaw = float("nan")       # position only, as the mission does
                self._sp_pub.publish(sp)

                now = time.monotonic()
                v = self._pose.get(now) if self._pose else None
                if v is None:
                    self.reset["detail"] = "no fresh pose"
                    time.sleep(0.5)
                    continue
                e, n = v[0], v[1]
                d = math.hypot(e, n)
                self.reset["detail"] = "%.0f m to run" % d
                if d <= arrived_within:
                    break
                time.sleep(0.5)
            else:
                raise RuntimeError("did not get back within 180 s")

            # Re-arm the AIRCRAFT too: set_gates rewinds which gates have been
            # served and forgets the answers already given, so the next run
            # starts at gate one instead of replying PASSAGE COMPLETE to its
            # first request.
            if self.uav is not None:
                self.uav.set_gates(self.uav.status()["gates"])

            self.mission = {"state": "idle", "phase": "", "progress": 0.0,
                            "outcome": None, "detail": "", "plan_version": 0,
                            "buoys_known": 0, "elapsed_s": 0.0}
            self.reset = {"state": "done", "detail": "on the start line"}
        except Exception as exc:                           # noqa: BLE001
            self.reset = {"state": "failed", "detail": str(exc)}
            self.get_logger().error("return to start failed: %s" % exc)

    # ------------------------------------------------------- the mission

    def _mission_send(self, tier=2, timeout_s=400.0):
        """Start a run. Called from the HTTP thread, so it never BLOCKS.

        send_goal_async returns a future the executor resolves on its own
        thread. Waiting on it here would deadlock the one spin that is supposed
        to resolve it, and the page would hang instead of the mission starting.
        """
        if self._goal_handle is not None and self.mission["state"] == "active":
            raise RuntimeError("a mission is already running")
        if not self.action.server_is_ready():
            # One second, not forever: if bt_runner is not up, say so on the
            # page rather than leaving a button that looks like it did nothing.
            if not self.action.wait_for_server(timeout_sec=1.0):
                raise RuntimeError(
                    "no /crsd/safe_passage server -- is bt_runner_node running?")
        goal = SafePassage.Goal()
        goal.tier = int(tier)
        goal.timeout_s = float(timeout_s)
        self.mission = {"state": "starting", "phase": "", "progress": 0.0,
                        "outcome": None, "detail": "", "plan_version": 0,
                        "buoys_known": 0, "elapsed_s": 0.0}
        fut = self.action.send_goal_async(goal, feedback_callback=self._on_fb)
        fut.add_done_callback(self._on_accepted)
        return "goal sent (tier %d)" % tier

    def _on_accepted(self, fut):
        gh = fut.result()
        if not gh.accepted:
            # bt_runner rejects a second goal rather than queueing it.
            self.mission.update(state="done", outcome=-1,
                                detail="goal REJECTED (one already running?)")
            return
        self._goal_handle = gh
        self.mission["state"] = "active"
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_fb(self, fb):
        f = fb.feedback
        self.mission.update(phase=f.phase, progress=float(f.progress),
                            plan_version=int(f.plan_version),
                            buoys_known=int(f.buoys_known))

    def _on_result(self, fut):
        r = fut.result().result
        self.mission.update(state="done", outcome=int(r.outcome),
                            detail=r.detail, elapsed_s=float(r.elapsed_s))
        self._goal_handle = None

    def _mission_cancel(self):
        if self._goal_handle is None:
            raise RuntimeError("nothing to cancel")
        self._goal_handle.cancel_goal_async()
        self.mission["state"] = "cancelling"
        return "cancel sent"

    # ---------------------------------------------- the radio, under --uav

    def _uav_transmit(self):
        """Send the CURRENT field as the passage. Called from the HTTP thread.

        Buoy ids are the index in the field list, which is exactly the number
        the page draws on each marker -- so a gate authored by clicking two
        markers names the same two buoys the boat will look up.
        """
        if self.uav is None:
            raise RuntimeError("no radio")
        with self._field_lock:
            field = list(self.field)
            origin = self.origin
        if origin is None:
            raise RuntimeError("not anchored yet")
        if not field:
            raise RuntimeError("nothing placed")

        buoys, entry, exit_ = [], None, None
        for i, (label, e, nn) in enumerate(field):
            lat, lon = geo.xy_to_latlon(e, nn, origin)
            buoys.append((i, lat, lon, BEACON_OF.get(label, 0)))
            if label == ENTRY_LABEL and entry is None:
                entry = (lat, lon)
            if label == EXIT_LABEL and exit_ is None:
                exit_ = (lat, lon)

        # A passage with no ENTRY or no EXIT is refused here rather than sent.
        # The boat would accept it -- the fields are just numbers -- and then
        # fail much later with "entry buoy not available", a long way from the
        # thing that was actually wrong.
        if entry is None or exit_ is None:
            missing = []
            if entry is None:
                missing.append("ENTRY (flashing blue)")
            if exit_ is None:
                missing.append("EXIT (steady blue)")
            raise RuntimeError("place an " + " and an ".join(missing) + " first")

        self.uav.send_plan(buoys, entry, exit_)
        return "sent %d buoys" % len(buoys)

    def _uav_set_gates(self, gates):
        if self.uav is not None:
            self.uav.set_gates(gates)

    def _gui_set_field(self, buoys):
        """Replace the field. Called from the HTTP thread, never the executor."""
        with self._field_lock:
            self.field = [(str(l), float(e), float(n)) for l, e, n in buoys]

    def _gui_state(self):
        """What the page polls. Every number here is either fresh or absent.

        The boat is omitted entirely once its pose goes stale rather than being
        sent at its last known position: a marker frozen on a map is read as a
        stationary boat, which is the one failure this bench must not imitate.
        """
        if self.origin is None:
            return {"anchored": False, "boat": None, "buoys": [],
                    "radio": self.uav.status() if self.uav is not None else None,
                    "mission": dict(self.mission), "reset": dict(self.reset),
                    "nan_heading": self._nan_heading}
        now = time.monotonic()
        v = self._pose.get(now) if self._pose else None
        boat = None
        if v is not None:
            e, n, yaw, _speed = v
            boat = {"east": round(e, 2), "north": round(n, 2),
                    "heading_deg": round(math.degrees(yaw), 1),
                    "age_s": round(self._pose.age(now), 2)}
        out = []
        for label, e, n in self._snapshot():
            lat, lon = geo.xy_to_latlon(e, n, self.origin)
            out.append({"label": label, "east": round(e, 2), "north": round(n, 2),
                        "lat": lat, "lon": lon})
        radio = self.uav.status() if self.uav is not None else None
        return {"anchored": True, "origin": list(self.origin), "radio": radio,
                "mission": dict(self.mission), "reset": dict(self.reset),
                "anchor_heading_deg": round(math.degrees(self.anchor_heading), 1),
                "boat": boat, "buoys": out, "nan_heading": self._nan_heading}

    def _on_att(self, msg: Attitude):
        self._att.set((msg.roll, msg.pitch), time.monotonic())

    def _waiting_check(self):
        """Say loudly when nothing is arriving, and name the two likely causes."""
        if self.origin is not None:
            return
        self.get_logger().warn(
            "no usable /crsd/pose yet after "
            f"{time.monotonic() - self.t0:.0f}s — is telemetry_bridge running, "
            "and does the fix carry a non-NaN heading? (NaN = GPS yaw "
            "unresolved.) For a desk run with no boat, use --sim-pose.",
            throttle_duration_sec=5.0)

    # ---------- vessel: the simulated one ----------

    def _start_simulated_vessel(self):
        """Publish pose/attitude/status for a boat circling the field.

        Desk mode. A circle rather than a straight line because a straight run
        past a buoy never tests the two things most likely to be wrong —
        whether a track survives leaving the field of view, and whether it is
        still ONE track when it comes back into view from a different bearing.
        """
        self.pose_pub = self.create_publisher(LatLonHead, "/crsd/pose", 10)
        self.att_pub = self.create_publisher(Attitude, "/crsd/attitude", 10)
        self.status_pub = self.create_publisher(FcuStatus, "/crsd/fcu_status", 10)
        self.create_timer(1 / 20.0, self._sim_pose)
        self.create_timer(1 / 30.0, self._sim_attitude)
        self.create_timer(1.0, self._sim_status)
        # The simulated boat starts at the field frame's origin heading "ahead",
        # so one FIELD table serves both modes unchanged.
        self._anchor(SIM_ORIGIN[0], SIM_ORIGIN[1], 0.0)
        self.get_logger().warn(
            "--sim-pose: publishing /crsd/pose MYSELF. If telemetry_bridge is "
            "running, both are on that topic and the tracker will see the boat "
            "teleporting. Stop the core stack, or drop this flag.")

    def _sim_state(self):
        """Simulated (east, north, yaw, speed, roll, pitch) at this instant."""
        t = time.monotonic() - self.t0
        r, v = self.args.radius, self.args.speed
        w = v / r                                   # rad/s around the circle
        # Centre the circle on the field, and start the boat at the frame
        # origin heading along the bow — matching the anchor above.
        cx, cy = 0.0, r
        e = cx + r * math.sin(w * t)
        n = cy - r * math.cos(w * t)
        yaw = (w * t) % (2 * math.pi)
        roll = pitch = 0.0
        if self.args.chop:
            roll = math.radians(8.0) * math.sin(t * 1.7)
            pitch = math.radians(4.0) * math.sin(t * 2.3 + 1.0)
        return e, n, yaw, v, roll, pitch

    def _sim_pose(self):
        if (self.args.drop_pose_after
                and time.monotonic() - self.t0 > self.args.drop_pose_after):
            return                        # silence, exactly as a dead bridge
        e, n, yaw, v, _r, _p = self._sim_state()
        lat, lon = geo.xy_to_latlon(e, n, self.origin)
        m = LatLonHead()
        m.header.stamp = self.get_clock().now().to_msg()
        m.latitude, m.longitude = lat, lon
        m.heading = math.degrees(yaw) % 360.0
        m.ground_speed = v
        self.pose_pub.publish(m)

    def _sim_attitude(self):
        _e, _n, yaw, _v, roll, pitch = self._sim_state()
        m = Attitude()
        m.header.stamp = self.get_clock().now().to_msg()
        m.roll, m.pitch, m.yaw = roll, pitch, yaw
        self.att_pub.publish(m)

    def _sim_status(self):
        m = FcuStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.mode, m.armed, m.system_status = "MANUAL", False, 3
        self.status_pub.publish(m)

    # ---------- the field ----------

    def _anchor(self, lat0: float, lon0: float, heading0: float):
        """Place the buoy field ahead of the bow and print the ground truth.

        FIELD is written as (right, ahead) so the layout reads the way a person
        describes it from the helm. Rotating by the heading at anchor time is
        what puts the buoys where the boat can actually see them: the same table
        works whether the dock faces north or south-west, and the operator does
        not have to recompute a field every time the boat is put in the water at
        a different angle.
        """
        self.origin = (lat0, lon0)
        self.anchor_heading = heading0
        ch, sh = math.cos(heading0), math.sin(heading0)
        built = []
        for label, right, ahead in FIELDS[self.args.field]:
            east = right * ch + ahead * sh
            north = -right * sh + ahead * ch
            built.append((label, east, north))
        # --gui starts from empty water on purpose: a page that opened with a
        # 92 m open-water field already laid out would have the operator
        # deleting ten buoys before placing the first one they wanted.
        if self.args.gui:
            built = []
        with self._field_lock:
            self.field = built

        log = self.get_logger()
        log.info(f"field anchored at {lat0:.7f}, {lon0:.7f} "
                 f"heading {math.degrees(heading0):.1f} deg")
        log.info("GROUND TRUTH — compare the tracker's output against this:")
        for label, e, n in self._snapshot():
            lat, lon = geo.xy_to_latlon(e, n, self.origin)
            log.info(f"  {label or '(unlabelled)':>14}  "
                     f"E{e:+7.1f} N{n:+7.1f}   {lat:.7f}, {lon:.7f}")
        if self.args.no_camera:
            log.warn("--no-camera: every track will be ANONYMOUS (empty label)")
        if self.args.no_lidar:
            log.warn(f"--no-lidar: ranges carry the stereo bias "
                     f"(+{CAM_RANGE_BIAS:.0%}); targets will sit BEYOND truth")

    def _vessel(self):
        """(east, north, yaw, roll, pitch) now, or None if the boat is unknown.

        None is a real answer and callers must handle it. Substituting a last
        known pose would put the invented buoys at bearings computed from a
        position the boat has left, which is the frozen-pose failure this repo
        keeps having to design out — and here it would be self-inflicted.
        """
        if self.args.sim_pose:
            e, n, yaw, _v, roll, pitch = self._sim_state()
            return e, n, yaw, roll, pitch
        now = time.monotonic()
        pose = self._pose.get(now)
        if pose is None or self.origin is None:
            return None
        att = self._att.get(now)
        # A stale attitude falls back to level rather than blocking. It costs
        # nothing: the tracker refuses to ingest anything without a fresh
        # attitude of its own, so these detections are computed and then
        # correctly ignored. Blocking here would instead make a dead ATTITUDE
        # stream look like a dead CAMERA, which is the wrong diagnosis to hand
        # someone at the dock.
        roll, pitch = att if att is not None else (0.0, 0.0)
        return pose[0], pose[1], pose[2], roll, pitch

    def _visible(self, half_fov_deg, max_m):
        """Field members inside a sensor's cone, as (label, x, y, z, range).

        One function for both sensors because the difference between them is
        the cone and the error model, not the geometry — and two copies of a
        visibility test drift until one sensor "sees" something the other
        cannot, which looks exactly like a fusion bug.

        Returns None (not an empty list) when the boat's position is unknown:
        "no buoys in view" and "no idea where the boat is" are different
        answers and the callers publish differently for each.
        """
        v = self._vessel()
        if v is None:
            return None
        e, n, yaw, roll, pitch = v
        out = []
        for label, be, bn in self._snapshot():
            x, y, z = geo.world_to_body_ypr(be, bn, BUOY_Z, roll, pitch, yaw,
                                            e, n)
            r = math.hypot(x, y)
            if r > max_m or r < 0.5:
                continue
            if abs(math.degrees(math.atan2(y, x))) > half_fov_deg:
                continue
            out.append((label, x, y, z, r))
        return out

    def _no_pose(self):
        """Warn once per dropout, then stay quiet. Returns True if unknown."""
        if not self._warned_no_pose:
            self._warned_no_pose = True
            self.get_logger().warn(
                "vessel position unknown — NOT publishing detections. Cannot "
                "compute what a sensor would have seen without knowing where "
                "the boat is, and publishing empty arrays would tell the "
                "tracker the water is clear.")
        return True

    # ---------- the sensors ----------

    def _detections(self):
        """What buoy_detector would publish: camera_link, EVERY frame.

        The unlabelled field member is skipped here and only here — a camera
        that cannot name a thing does not report it, which is precisely why the
        LiDAR-only path has to work.

        Positions are converted out of base_link into TRUE camera_link before
        publishing. Without that step the mount offset would be missing from
        the bench and present in the tracker, and every camera-derived position
        would land 37 cm forward and 65 cm high of truth — a clean, constant
        error that looks exactly like a fusion bug and is not one.
        """
        if self.args.no_camera:
            return
        seen = self._visible(CAM_FOV_DEG, CAM_MAX_M)
        if seen is None:
            return self._no_pose()
        self._warned_no_pose = False

        array = Detection3DArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = "camera_link"
        for label, x, y, z, r in seen:
            if not label:
                continue
            # Perturb the BEARING slightly and the RANGE badly, then rebuild
            # the position from the pair — applying independent noise to x and
            # y instead would give the camera a range accuracy it does not have.
            b = math.atan2(y, x) + random.gauss(0, CAM_BEARING_SD)
            rr = r * (1 + CAM_RANGE_BIAS + random.gauss(0, CAM_RANGE_SD))
            cx, cy, cz = core.body_to_camera(rr * math.cos(b), rr * math.sin(b),
                                             z, self.extrinsic)
            d = Detection3D()
            d.label, d.confidence = label, round(random.uniform(0.62, 0.95), 2)
            d.x, d.y, d.z = cx, cy, cz
            d.bbox = [0, 0, 0, 0]
            array.detections.append(d)
        self.det_pub.publish(array)       # every frame, empty or not

    def _clusters(self):
        """What lidar_cluster_node would publish: base_link, every window."""
        if self.args.no_lidar:
            return
        seen = self._visible(LIDAR_FOV_DEG, LIDAR_MAX_M)
        if seen is None:
            return self._no_pose()
        self._warned_no_pose = False

        array = Cluster3DArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = "base_link"
        found = []
        for label, x, y, z, r in seen:
            if random.random() < LIDAR_MISS_P:
                continue                  # a small target missed this window
            k = 1 + random.gauss(0, LIDAR_RANGE_SD) / max(r, 1e-3)
            c = Cluster3D()
            c.x, c.y, c.z = x * k, y * k, z
            c.extent_x = c.extent_y = 0.32
            c.extent_z = 0.45
            # Roughly the return count the real sensor gives a 0.3 m buoy:
            # ~37 at 5 m, falling off as 1/r^2 (Cluster3D.msg).
            c.n_points = max(2, int(37 * (5.0 / max(r, 1.0)) ** 2))
            c.range = math.hypot(c.x, c.y)
            found.append(c)
        found.sort(key=lambda c: c.range)   # nearest first, per the contract
        array.clusters = found
        self.clu_pub.publish(array)        # every window, empty or not


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim-pose", action="store_true",
                    help="invent the VESSEL too and publish /crsd/pose myself "
                         "(desk mode; do NOT use while core.launch.py is up)")
    ap.add_argument("--no-camera", action="store_true",
                    help="LiDAR only — every track goes anonymous")
    ap.add_argument("--no-lidar", action="store_true",
                    help="camera only — ranges carry the stereo bias")
    ap.add_argument("--field", choices=sorted(FIELDS), default="tracker",
                    help="which invented field to lay out: 'tracker' (default, "
                         "the original mixed field) or 'task1' (handbook 3.3.2 "
                         "Safe Passage: 10 buoys with an ENTRY and an EXIT)")
    ap.add_argument("--gui", action="store_true",
                    help="place the field by hand in a browser instead of "
                         "using a --field table. Starts empty; serves on "
                         "--gui-port. This is the pool mode: a table sized for "
                         "open water does not fit in the water you have")
    ap.add_argument("--gui-port", type=int, default=field_gui.DEFAULT_PORT,
                    help=f"port for --gui (default {field_gui.DEFAULT_PORT}; "
                         "8080/8081/8085/8090 are already taken on the boat)")
    ap.add_argument("--uav", action="store_true",
                    help="also play the AIRCRAFT: open the RXL radio and serve "
                         "Transmit and the gate handshake from the page. Use "
                         "with --gui; the field you place becomes the passage "
                         "the boat is told about.")
    ap.add_argument("--uav-endpoint", default="udpout:127.0.0.1:14555",
                    help="rxl_link_node's rxl_endpoint")
    ap.add_argument("--seed", type=int, default=1, help="RNG seed")
    sim = ap.add_argument_group("--sim-pose only")
    sim.add_argument("--radius", type=float, default=30.0,
                     help="circle radius around the buoy field [m]")
    sim.add_argument("--speed", type=float, default=2.0,
                     help="ground speed [m/s]")
    sim.add_argument("--chop", action="store_true",
                     help="add roll/pitch; targets must NOT move on the map")
    sim.add_argument("--drop-pose-after", type=float, default=0.0,
                     help="stop publishing /crsd/pose after N s")
    args = ap.parse_args()

    # Fail loudly rather than silently ignoring a flag. A --chop run that
    # quietly did nothing because the real attitude was in use would be read as
    # "roll compensation works", which is the opposite of what it proved.
    if not args.sim_pose:
        used = [name for name, on in (("--chop", args.chop),
                                      ("--drop-pose-after", args.drop_pose_after))
                if on]
        if used:
            ap.error(f"{', '.join(used)} only applies with --sim-pose; the real "
                     "vessel supplies its own attitude and its own dropouts "
                     "(stop telemetry_bridge to test a dropout for real)")
    if args.no_camera and args.no_lidar:
        ap.error("--no-camera and --no-lidar together publish nothing at all")
    # Refused rather than quietly ignored: --gui empties the field at anchor
    # time, so a --field given alongside it would name a table that never gets
    # laid out, and the run would look like the table was wrong.
    if args.gui and args.field != "tracker":
        ap.error("--gui replaces the --field table (it starts from empty "
                 "water); pass one or the other, not both")

    rclpy.init()
    node = WorldModelBench(args)
    gui = None
    if args.gui:
        gui = field_gui.FieldGui(
            node._gui_state, node._gui_set_field, args.gui_port,
            transmit=node._uav_transmit if args.uav else None,
            set_gates=node._uav_set_gates if args.uav else None,
            mission_send=node._mission_send, mission_cancel=node._mission_cancel,
            sitl_reset=node._return_to_start)
        gui.start()
        node.get_logger().info(
            f"field GUI on http://<this-host>:{args.gui_port} — the field "
            "starts EMPTY and cannot be edited until a fix with a resolved "
            "heading anchors it")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if gui:
            gui.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
