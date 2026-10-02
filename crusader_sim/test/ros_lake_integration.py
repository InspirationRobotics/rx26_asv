"""ros_lake_integration — LakePanel + the REAL panel_feed node + the REAL rxl_link_node, in an isolated ROS
domain (87) on private ports (rxl udp 34555, feed udp 34556): what test_lake.py fakes (message objects, the
radio peer) is real here. A manual check, not part of `unittest discover` (it needs ROS).

Runs in: the crsd-sim container (or `asv`), after sourcing /opt/ros/humble and the workspace. ROS_DOMAIN_ID=87
keeps it off a running sim's graph and the private ports keep it off 14555/14556:

    docker exec -e ROS_DOMAIN_ID=87 crsd-sim bash -lc 'source /opt/ros/humble/setup.bash;
      source /root/robotx_ws/install/setup.bash;
      PYTHONPATH=/root/robotx_ws/src/rx26_asv/crusader_sim:$PYTHONPATH       python3 -u /root/robotx_ws/src/rx26_asv/crusader_sim/test/ros_lake_integration.py'

Prints PASS/FAIL per check and exits 1 if any failed. Checked 2026-10-02: 26/26.
"""
import json
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String, UInt8

from crusader_msgs.msg import (FcuStatus, GatePair, Hazard, HazardArray, LatLonHead, PassagePlan, TrackedTarget,
                               TrackedTargetArray)
from crusader_sim import course as C
from crusader_sim.lake_panel import LakePanel

DATUM = {"lat": 1.3, "lon": 103.85}
FIELD = [{"x": 12.0, "y": 3.0, "state": "flash_blue"}, {"x": 22.0, "y": -3.0, "state": "flash_red"},
         {"x": 22.0, "y": 3.0, "state": "flash_green"}, {"x": 42.0, "y": 0.0, "state": "steady_blue"},
         {"x": 27.0, "y": 9.0, "state": "off"}]
RXL_PORT, FEED_PORT = 34555, 34556
results = []


def check(what, ok, detail=""):
    results.append(bool(ok))
    print("  %s %s %s" % ("PASS" if ok else "FAIL", what, detail), flush=True)


def wait(cond, timeout=8.0):
    t = time.time()
    while time.time() - t < timeout:
        if cond():
            return True
        time.sleep(0.05)
    return False


class BoatStandIn(Node):
    """Publishes what telemetry_bridge / bt_runner would; records what rxl_link_node publishes."""

    def __init__(self):
        super().__init__("boat_standin")
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(LatLonHead, "/crsd/pose", 10)
        self.fcu_pub = self.create_publisher(FcuStatus, "/crsd/fcu_status", 10)
        self.haz_pub = self.create_publisher(HazardArray, "/crsd/nav/hazards", latched)
        self.datum_pub = self.create_publisher(LatLonHead, "/crsd/datum", latched)
        self.rep_pub = self.create_publisher(String, "/crsd/safe_passage_report", 10)
        self.tgt_pub = self.create_publisher(TrackedTargetArray, "/crsd/world_targets", 10)
        self.gate_pub = self.create_publisher(UInt8, "crsd/gate_reached", 10)
        self.plans, self.gates = [], []
        self.create_subscription(PassagePlan, "crsd/passage_plan", lambda m: self.plans.append(m), 10)
        self.create_subscription(GatePair, "crsd/next_gate", lambda m: self.gates.append(m), 10)
        self.mode, self.armed, self.pos = "HOLD", False, (5.0, 2.0)
        self.create_timer(0.2, self.tick)
        d = LatLonHead()
        d.latitude, d.longitude = DATUM["lat"], DATUM["lon"]
        self.datum_pub.publish(d)

    def tick(self):
        la, lo = C.enu_to_latlon(self.pos[0], self.pos[1], DATUM)
        p = LatLonHead()
        p.latitude, p.longitude, p.heading, p.ground_speed = la, lo, 90.0, 0.4
        self.pose_pub.publish(p)
        f = FcuStatus()
        f.mode, f.armed, f.system_status = self.mode, self.armed, 4
        self.fcu_pub.publish(f)
        t = TrackedTarget()
        tla, tlo = C.enu_to_latlon(22.1, -2.9, DATUM)
        t.id, t.label, t.latitude, t.longitude, t.confirmed, t.hits, t.time_since_seen = 7, "red_buoy", tla, tlo, True, 20, 0.1
        arr = TrackedTargetArray()
        arr.targets = [t]
        self.tgt_pub.publish(arr)
        h = HazardArray()
        h.header.frame_id = "map"
        c = Hazard()
        c.kind, c.source, c.id, c.x, c.y, c.radius_m, c.keepout_m = Hazard.CIRCLE, 0, 3, 27.0, 9.0, 0.4, 0.3
        poly = Hazard()
        poly.kind, poly.source, poly.id = Hazard.POLYGON, 2, 9
        poly.polygon_x, poly.polygon_y = [0.0, 6.0, 6.0, 0.0], [-12.0, -12.0, -9.0, -9.0]
        h.hazards = [c, poly]
        self.haz_pub.publish(h)
        self.rep_pub.publish(String(data=json.dumps({"buoys": [], "n_gates": 1, "gates_cleared": 0, "single_count": 0})))


def main():
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    procs = []
    cfg = "/root/robotx_ws/install/crusader_bringup/share/crusader_bringup/config/crusader_params.yaml"
    procs.append(subprocess.Popen(["ros2", "run", "crusader_link", "rxl_link_node", "--ros-args", "--params-file", cfg,
                                   "-p", "rxl_endpoint:=udpin:127.0.0.1:%d" % RXL_PORT], env=env,
                                  stdout=open("/tmp/ri_rxl.log", "w"), stderr=subprocess.STDOUT, start_new_session=True))
    procs.append(subprocess.Popen([sys.executable, "-u", "-m", "crusader_sim.panel_feed", "--ros-args",
                                   "-p", "origin:=%s,%s" % (DATUM["lat"], DATUM["lon"]), "-p", "course:=lake",
                                   "-p", "panel_port:=%d" % FEED_PORT], env=env,
                                  stdout=open("/tmp/ri_feed.log", "w"), stderr=subprocess.STDOUT, start_new_session=True))
    rclpy.init()
    boat = BoatStandIn()
    ex = MultiThreadedExecutor()
    ex.add_node(boat)
    threading.Thread(target=ex.spin, daemon=True).start()
    a = NS(dry_run=True, dry_run_mission_s=1.0, rxl_endpoint="udpout:127.0.0.1:%d" % RXL_PORT, feed_port=FEED_PORT,
           origin=dict(DATUM), lake_dir=tempfile.mkdtemp(prefix="ri_"), container="x", lake=True)
    panel = LakePanel(a)
    panel.feed.start()
    threading.Thread(target=panel.loop, daemon=True).start()
    try:
        print("== the feed: real panel_feed node -> the panel's receiver")
        check("the feed comes up", wait(lambda: panel.feed.view()["up"]), panel.feed.view().get("error"))
        v = lambda: panel.feed.view()
        check("pose layer fresh, x/y/yaw right", wait(lambda: v()["pose"]["status"] == "fresh") and
              abs(v()["pose"]["data"]["x"] - 5.0) < 0.05 and abs(v()["pose"]["data"]["y"] - 2.0) < 0.05 and
              abs(v()["pose"]["data"]["yaw"]) < 0.01, str(v()["pose"]["data"]))
        check("fcu layer: HOLD, disarmed", wait(lambda: v()["fcu"]["status"] == "fresh") and
              v()["fcu"]["data"]["mode"] == "HOLD" and v()["fcu"]["data"]["armed"] is False, str(v()["fcu"]["data"]))
        check("hazards layer: circle and polygon, in course metres", wait(lambda: v()["hazards"]["status"] == "fresh") and
              len(v()["hazards"]["data"]["items"]) == 2, str(v()["hazards"]["data"] and v()["hazards"]["data"]["items"][:1]))
        h0 = [i for i in (v()["hazards"]["data"] or {"items": []})["items"] if i["kind"] == 0]
        check("... the circle at 27.0 x 1.0011", h0 and abs(h0[0]["x"] - 27.03) < 0.05 and abs(h0[0]["y"] - 9.01) < 0.05, str(h0))
        check("datum in the packet", v()["datum"] == {"lat": DATUM["lat"], "lon": DATUM["lon"]}, str(v()["datum"]))
        check("tracks layer: the camera track", wait(lambda: v()["tracks"]["status"] == "fresh") and
              v()["tracks"]["data"]["items"][0]["label"] == "red_buoy", str(v()["tracks"]["data"]))
        check("passage layer carries n_gates", wait(lambda: v()["passage"]["status"] == "fresh") and
              v()["passage"]["data"]["n_gates"] == 1, str(v()["passage"]["data"] and v()["passage"]["data"].get("n_gates")))
        st = panel.state({})
        check("panel: boat + fcu, START blocked (nothing committed yet)", st["boat"] is not None and
              st["fcu"]["data"]["mode"] == "HOLD" and "COMMIT" in (st["start_block"] or ""), str(st["start_block"]))
        check("panel: no origin problem with the real feed", "origin" not in st["errors"], str(st["errors"]))

        print("== the field -> the real rxl_link_node -> /crsd/passage_plan")
        r = panel.act_pin({"state": "off"})
        check("PIN AT BOAT from the real pose", r["ok"], str(r))
        panel.act_clear({})
        check("layout accepted", panel.act_layout({"buoys": FIELD})["ok"])
        check("COMMIT", panel.act_commit({})["ok"])
        check("the plan arrives on /crsd/passage_plan", wait(lambda: boat.plans), "")
        pl = boat.plans[-1] if boat.plans else None
        if pl:
            got = sorted((b.id, b.beacon) for b in pl.buoys)
            check("5 buoys, beacon codes ENTRY 4, RED 2, GREEN 3, EXIT 5, OFF 1", got == [(0, 4), (1, 2), (2, 3), (3, 5), (4, 1)], str(got))
            la, lo = C.enu_to_latlon(12.0, 3.0, DATUM)
            b0 = [b for b in pl.buoys if b.id == 0][0]
            check("buoy 0 lat/lon exact to 1e-6 deg (~0.1 m)", abs(b0.latitude - la) < 1e-6 and abs(b0.longitude - lo) < 1e-6,
                  "%.7f,%.7f vs %.7f,%.7f" % (b0.latitude, b0.longitude, la, lo))
            ela, elo = C.enu_to_latlon(12.0, 3.0, DATUM)
            check("entry position is buoy 0's", abs(pl.entry_latitude - ela) < 1e-6 and abs(pl.entry_longitude - elo) < 1e-6)
        v0 = pl.plan_version if pl else -1
        n0 = len(boat.plans)
        time.sleep(0.5)
        # the dead-man: resends only while polled; plan_version must NOT bump on identical resends
        for _ in range(60):                      # ~6 s polling at 10 Hz: one 5 s resend goes out
            panel.state({})
            time.sleep(0.1)
        check("a periodic resend arrived and the plan version did not change", len(boat.plans) > n0 and boat.plans[-1].plan_version == v0,
              "plans %d->%d version %s->%s" % (n0, len(boat.plans), v0, boat.plans[-1].plan_version))

        print("== a recolour is a new plan; a checkpoint ask comes back as a label")
        panel.act_stage({"id": 1, "state": "flash_green"})
        check("SEND CHANGES", panel.act_send({})["ok"])
        check("plan_version bumps on the colour change", wait(lambda: boat.plans[-1].plan_version != v0), str(boat.plans[-1].plan_version))
        boat.gate_pub.publish(UInt8(data=1))
        check("the boat's ask (gate_reached 1) shows as a labelled checkpoint", wait(lambda: panel.state({})["checkpoint"] is not None) and
              panel.state({})["checkpoint"]["label"] == "ENTRY orbit done - confirm gate 1", str(panel.state({})["checkpoint"] and panel.state({})["checkpoint"]["label"]))
        r = panel.act_ack({})
        check("ACK", r["ok"], str(r))
        check("/crsd/next_gate carries seq 1", wait(lambda: any(g.gate_seq == 1 for g in boat.gates)), str([(g.gate_seq, g.red_id, g.green_id) for g in boat.gates]))
        boat.gate_pub.publish(UInt8(data=2))
        check("seq 2 (n_gates = 1 from the report): the EXIT label", wait(lambda: any(c["seq"] == 2 for c in panel.state({})["checkpoints"])) and
              [c for c in panel.state({})["checkpoints"] if c["seq"] == 2][0]["label"] == "EXIT gate - confirm exit",
              str([c["label"] for c in panel.state({})["checkpoints"]]))

        print("== START gating from the real FCU stream")
        boat.armed, boat.mode = True, "GUIDED"
        check("armed + GUIDED enables START", wait(lambda: panel.state({})["start_block"] is None), str(panel.state({})["start_block"]))
        boat.mode = "HOLD"
        check("the pilot switching to HOLD disables it", wait(lambda: panel.state({})["start_block"] is not None), str(panel.state({})["start_block"]))
    finally:
        for p in procs:
            try:
                os.killpg(p.pid, 2)
            except ProcessLookupError:
                pass
        panel.shutdown()
        rclpy.shutdown()
    print("RESULT %d/%d checks passed" % (sum(results), len(results)), flush=True)
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
