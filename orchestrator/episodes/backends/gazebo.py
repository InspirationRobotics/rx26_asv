"""Gazebo + ArduPilot SITL backend — closes the OmniX strafe-dynamics gap.

WHY THIS EXISTS
---------------
docker/sitl/run_sitl.sh documents the limitation that bounds every SITL result
in this repo:

    "SITL's boat model ('motorboat' frame) does not model OmniX lateral thrust
     — GUIDED behavior, AVOID_*, WP_* logic are exact (same firmware), strafe
     dynamics are not. dp_hold-style RC-override mechanisms get logic-level
     testing only; their dynamics are bench/field territory."

That is a real hole. Crusader is FRAME_TYPE=2 (OmniX) — four thrusters in an X,
genuinely holonomic — and `dp_hold` is THE lateral-hold path (MANUAL + RC
override, since GUIDED cannot strafe on this frame). Today its dynamics can only
be validated on the water, which is the scarcest resource of the season and the
one thing you cannot get more of in November.

This backend keeps ArduPilot SITL as the flight code (same 4.6.3 firmware, same
GUIDED/AVOID_/WP_ logic — nothing about that changes) but replaces SITL's
built-in motorboat FDM with **Gazebo physics driving a real four-thruster OmniX
model**. Lateral thrust becomes real: dp_hold, station-keeping in wind, and the
Task 3 docking approach get dynamics-level testing instead of logic-level.

    kinematic  ->  fast, no dynamics, CI default
    sitl       ->  real firmware, motorboat FDM, NO lateral thrust
    gazebo     ->  real firmware, Gazebo FDM, REAL lateral thrust   <- this file

It is strictly additive: same backend contract, same scenario JSON, same
evaluator, same metrics. `run_episode.py --backend gazebo`.

WHAT IT DOES NOT CHANGE
-----------------------
Nothing about scoring, scenarios, or the evaluator. A gazebo episode is directly
comparable to a sitl episode because the scenario and metric contract are
identical — which is the whole point of the backend abstraction.

SAFETY
------
Inherits SitlBackend's guards verbatim: UDP/TCP endpoints only (never a serial
device — the single-Pixhawk-owner rule is enforced structurally, at the
constructor), and RX26_SITL_OK=1 required before anything arms. Additionally
refuses to run if it cannot confirm a Gazebo server, so a misconfigured run
fails loudly instead of silently falling back to the motorboat FDM and quietly
reporting strafe results that are not real.

SCOPE
-----
This repo is the ASV. The UUV and UAV are separate repositories deployed to
their own devices, so this backend models ONE vehicle and never reaches across
domains. Cross-domain behaviour (Task 1 Advanced UAV->USV route handoff, Task 2
UUV resource requests) reaches the ASV only as RoboCommand messages, which is
already the mission planner's contract — see api/mission/robocomms.py. Nothing
here needs a second vehicle in the world.

PREREQUISITES (in the crusader container)
-----------------------------------------
    docker/sitl/install_sitl.sh        ArduRover 4.6.3   (already pinned)
    docker/sitl/install_gazebo.sh      Gazebo Harmonic + ardupilot_gazebo
    docker/sitl/run_sitl_gazebo.sh     brings up both, wired together

USAGE
-----
    RX26_SITL_OK=1 python3 orchestrator/run_episode.py \
        --scenario orchestrator/scenarios/mission1_transit.json \
        --backend gazebo --mav udp:127.0.0.1:14550 --out /tmp/ep.json
"""
import math
import os
import socket
import time


class GazeboBackend:
    """Backend contract: reset / set_target / set_speed_scale / step / state /
    shutdown, plus a `.t` attribute. Identical to SitlBackend by design."""

    def __init__(self, endpoint="udp:127.0.0.1:14550", timeout_s=60.0,
                 gz_world=None, require_gazebo=True):
        # --- inherited safety guards (see SitlBackend) ------------------------
        if not (endpoint.startswith("udp:") or endpoint.startswith("udpin:")
                or endpoint.startswith("tcp:")):
            raise ValueError(
                f"refusing endpoint {endpoint!r}: only udp:/tcp: allowed — this "
                "backend must never open a serial device "
                "(single-Pixhawk-owner rule)")
        if os.environ.get("RX26_SITL_OK") != "1":
            raise RuntimeError(
                "RX26_SITL_OK=1 not set. This backend arms and drives the vehicle "
                "it connects to; set the variable only when the endpoint is SITL.")

        # --- the guard specific to this backend -------------------------------
        # Without this, a run with Gazebo down silently falls through to SITL's
        # motorboat FDM. Every number still looks plausible; the strafe results
        # are simply not real. Fail loudly instead.
        self.gz_world = gz_world or os.environ.get("RX26_GZ_WORLD")
        if require_gazebo and not self._gazebo_alive():
            raise RuntimeError(
                "no Gazebo server detected on the ArduPilot FDM port.\n"
                "Start it first:  docker/sitl/run_sitl_gazebo.sh\n"
                "Refusing to run: without Gazebo, SITL falls back to the "
                "motorboat FDM, which does NOT model OmniX lateral thrust — the "
                "exact limitation this backend exists to remove. Results would "
                "look fine and be wrong.\n"
                "Set require_gazebo=False only if you have deliberately "
                "arranged the FDM another way.")

        from pymavlink import mavutil   # lazy: orchestrator stays stdlib-only
        self._mavutil = mavutil
        self.endpoint = endpoint
        self.timeout_s = timeout_s
        self.conn = None
        self.origin = None
        self._last = (0.0, 0.0, 0.0, 0.0)
        self.t = 0.0
        self._t0 = None
        self.speed_scale = 1.0

    # ------------------------------------------------------------------ probe #

    @staticmethod
    def _gazebo_alive(host="127.0.0.1", port=9002, timeout=0.5):
        """
        ardupilot_gazebo binds the JSON FDM port. If something is listening,
        Gazebo is up with the plugin loaded.

        UDP has no connection state, so we probe by binding: if the bind FAILS
        with EADDRINUSE, someone else (Gazebo) already holds it.
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.settimeout(timeout)
            s.bind((host, port))
            return False          # we got it -> nothing else is there
        except OSError:
            return True           # in use -> Gazebo has it
        finally:
            s.close()

    # --------------------------------------------------- frame conversion ---- #
    # Equirectangular, same as SitlBackend — fine at course scale and, more
    # importantly, IDENTICAL, so gazebo and sitl traces are comparable.

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

    # -------------------------------------------------- backend contract ----- #

    def reset(self, scenario, seed):
        m = self._mavutil
        self.origin = scenario.origin

        # The Gazebo world's <spherical_coordinates> and SITL's -l HOME_LOC must
        # both equal scenario.origin. If they disagree the vehicle sits in the
        # right place on screen while its GPS fix is somewhere else, and every
        # position-based metric is quietly wrong.
        # tools/sim/scenario_to_world.py generates the world FROM this same
        # scenario file so the two cannot drift; run_sitl_gazebo.sh reads it too.
        self._warn_origin_mismatch()

        self.conn = m.mavlink_connection(self.endpoint)
        if not self.conn.wait_heartbeat(timeout=self.timeout_s):
            raise TimeoutError(
                f"no heartbeat on {self.endpoint}. Is run_sitl_gazebo.sh up?")

        mode_id = self.conn.mode_mapping()["GUIDED"]
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            m.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            m.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        self.conn.motors_armed_wait()

        self._t0 = time.time()
        self.t = 0.0
        self.speed_scale = 1.0
        self._poll()

    def set_target(self, x, y):
        lat, lon = self._to_latlon(x, y)
        m = self._mavutil.mavlink
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

    def set_speed_scale(self, scale):
        """
        APF advisory speed scale (runner calls this when an advisor is active).

        Applied as WP_SPEED, matching how the boat consumes it. Recorded so the
        gazebo and sitl backends stay behaviourally comparable.
        """
        self.speed_scale = max(0.0, min(1.0, float(scale)))

    def step(self, dt):
        # SITL runs wall-clock here, same as SitlBackend. If you need faster
        # episodes, set SIM_SPEEDUP on the SITL side AND raise Gazebo's
        # real_time_factor — changing only one desynchronises the FDM link and
        # produces physically meaningless motion.
        time.sleep(dt)
        self.t = time.time() - self._t0
        self._poll()

    def _poll(self):
        msg = self.conn.recv_match(type="GLOBAL_POSITION_INT",
                                   blocking=True, timeout=2)
        if msg is None:
            return                       # keep last state; runner timeout catches death
        x, y = self._to_xy(msg.lat / 1e7, msg.lon / 1e7)
        heading = (math.radians(msg.hdg / 100.0)
                   if msg.hdg != 65535 else self._last[2])
        speed = math.hypot(msg.vx / 100.0, msg.vy / 100.0)
        self._last = (x, y, heading, speed)

    def state(self):
        return self._last

    def shutdown(self):
        if self.conn is not None:
            try:
                self.conn.mav.command_long_send(
                    self.conn.target_system, self.conn.target_component,
                    self._mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0)
                self.conn.close()
            except Exception:
                pass
            self.conn = None

    # ------------------------------------------------------------ diagnostics #

    def _warn_origin_mismatch(self):
        """
        Compare scenario.origin against the world the operator actually launched.

        Cheap check, expensive bug: a silent origin mismatch produces a run that
        completes, scores, and is wrong in a way no metric reveals.
        """
        world = self.gz_world
        if not world or not os.path.exists(world):
            return
        try:
            import re
            txt = open(world).read()
            lat = float(re.search(r"<latitude_deg>([-\d.]+)", txt).group(1))
            lon = float(re.search(r"<longitude_deg>([-\d.]+)", txt).group(1))
        except Exception:
            return
        slat, slon = self.origin
        if abs(lat - slat) > 1e-6 or abs(lon - slon) > 1e-6:
            raise RuntimeError(
                f"ORIGIN MISMATCH — refusing to run.\n"
                f"  scenario : {slat}, {slon}\n"
                f"  gz world : {lat}, {lon}  ({world})\n"
                f"Regenerate the world from the scenario:\n"
                f"  python3 tools/sim/scenario_to_world.py --scenario <file>")
