"""ArduPilot Rover SITL backend — the primary evaluation target (plan §4.6).

Connects ONLY to a UDP/TCP MAVLink endpoint (SITL's own output, or MAVProxy's
rebroadcast). It is structurally incapable of opening a serial device — the
"never a second Pixhawk consumer" rule is enforced at the constructor, not by
convention. Additionally it refuses to arm anything unless RX26_SITL_OK=1 is set
in the environment, so accidentally pointing it at the real boat's rebroadcast
cannot arm/drive the vehicle.

Runs in the crusader container next to docker/sitl/run_sitl.sh. Not importable
without pymavlink (lazy import) so the rest of the orchestrator stays stdlib-only.
"""
import math
import os
import time


class SitlBackend:
    def __init__(self, endpoint="udp:127.0.0.1:14550", timeout_s=60.0):
        if not (endpoint.startswith("udp:") or endpoint.startswith("udpin:")
                or endpoint.startswith("tcp:")):
            raise ValueError(
                f"refusing endpoint {endpoint!r}: only udp:/tcp: allowed — this backend "
                "must never open a serial device (single-Pixhawk-owner rule)")
        if os.environ.get("RX26_SITL_OK") != "1":
            raise RuntimeError(
                "RX26_SITL_OK=1 not set. This backend arms and drives the vehicle it "
                "connects to; set the variable only when the endpoint is SITL.")
        from pymavlink import mavutil  # lazy: keeps orchestrator stdlib-only elsewhere
        self._mavutil = mavutil
        self.endpoint = endpoint
        self.timeout_s = timeout_s
        self.conn = None
        self.origin = None           # (lat, lon) from scenario
        self._last = (0.0, 0.0, 0.0, 0.0)
        self.t = 0.0
        self._t0 = None

    # --- frame conversion (equirectangular, fine at course scale) ---

    def _to_latlon(self, x, y):
        lat0, lon0 = self.origin
        lat = lat0 + (y / 111_139.0)
        lon = lon0 + (x / (111_139.0 * math.cos(math.radians(lat0))))
        return lat, lon

    def _to_xy(self, lat, lon):
        lat0, lon0 = self.origin
        y = (lat - lat0) * 111_139.0
        x = (lon - lon0) * 111_139.0 * math.cos(math.radians(lat0))
        return x, y

    # --- backend contract ---

    def reset(self, scenario, seed):
        # seed is unused (SITL has its own SIM_* seeds; set via params if needed)
        m = self._mavutil
        self.origin = scenario.origin
        self.conn = m.mavlink_connection(self.endpoint)
        if not self.conn.wait_heartbeat(timeout=self.timeout_s):
            raise TimeoutError(f"no heartbeat on {self.endpoint}")
        # GUIDED mode
        mode_id = self.conn.mode_mapping()["GUIDED"]
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            m.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        # arm (SITL only — see constructor guard)
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            m.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        self.conn.motors_armed_wait()
        self._t0 = time.time()
        self.t = 0.0
        self._poll()

    def set_target(self, x, y):
        lat, lon = self._to_latlon(x, y)
        m = self._mavutil.mavlink
        # position-only target; velocity/accel/yaw fields masked out
        type_mask = (m.POSITION_TARGET_TYPEMASK_VX_IGNORE
                     | m.POSITION_TARGET_TYPEMASK_VY_IGNORE
                     | m.POSITION_TARGET_TYPEMASK_VZ_IGNORE
                     | m.POSITION_TARGET_TYPEMASK_AX_IGNORE
                     | m.POSITION_TARGET_TYPEMASK_AY_IGNORE
                     | m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                     | m.POSITION_TARGET_TYPEMASK_YAW_IGNORE
                     | m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        self.conn.mav.set_position_target_global_int_send(
            0, self.conn.target_system, self.conn.target_component,
            m.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, type_mask,
            int(lat * 1e7), int(lon * 1e7), 0.0,
            0, 0, 0, 0, 0, 0, 0, 0)

    def step(self, dt):
        time.sleep(dt)               # SITL runs wall-clock (lock-step later if needed)
        self.t = time.time() - self._t0
        self._poll()

    def _poll(self):
        msg = self.conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if msg is None:
            return                    # keep last state; runner's timeout catches death
        x, y = self._to_xy(msg.lat / 1e7, msg.lon / 1e7)
        heading = math.radians(msg.hdg / 100.0) if msg.hdg != 65535 else self._last[2]
        speed = math.hypot(msg.vx / 100.0, msg.vy / 100.0)
        self._last = (x, y, heading, speed)

    def state(self):
        return self._last

    def shutdown(self):
        if self.conn is not None:
            try:
                # disarm and close; idempotent
                self.conn.mav.command_long_send(
                    self.conn.target_system, self.conn.target_component,
                    self._mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0)
                self.conn.close()
            except Exception:
                pass
            self.conn = None
