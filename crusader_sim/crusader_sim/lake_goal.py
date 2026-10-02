"""lake_goal — send the Task 1 goal to the REAL boat, and nothing else.

    python3 -m crusader_sim.lake_goal --approach 1.3000000,103.8500000

Runs in: the `asv` container on the Jetson (ROS sourced, PYTHONPATH on crusader_sim), started
by the Task 1 panel's START in lake mode (task1_panel --lake) or by hand. lake_rig_up.sh has
the environment.

THE ARMING-FREE COUNTERPART OF task1_goal. The sim's operator arms the autopilot over
MAVLink and requests GUIDED; on the water that is the PILOT's job, with the RC. This tool
does neither, and cannot:

  * it never arms, disarms or changes the mode, and imports nothing that could
    (operator_tools, the /crsd/set_mode publisher, /crsd/rc_override and the MAVLink
    tooling ports 14550/14551 are all absent; test/test_lake.py reads this file's code to
    keep it that way);
  * it REFUSES TO START unless /crsd/fcu_status, heard just now, says armed AND GUIDED;
    no status within --fcu-wait-s is a refusal too: a blank is not "probably armed";
  * SIGINT/SIGTERM CANCEL the goal (goal_client.run_goal) and wait for the tree to finish
    it. Cancelling the goal makes bt_runner stop commanding; it does not stop the boat. The
    pilot's stop is the RC (SC to HOLD, or the SB e-stop), and the panel says so.

The goal: tier 2 (Disruptive: the UAV's colours), the approach point you give, 600 s unless
--timeout-s says otherwise. --approach none (or 0,0) sends 0,0 = "already on station, skip
the drive". Output lines are task1_goal's ([operator], [tree], [result]); the panel parses
them.

Exit status: 0 the tree reported SUCCESS, 1 any other outcome (CANCELLED included), 2 no
bt_runner, 3 REFUSED (not armed+GUIDED, or no FCU status), 4 bad arguments.
"""
import argparse
import math
import sys
import time

from crusader_sim.goal_client import TIERS, install_stop, run_goal

FCU_TOPIC = "/crsd/fcu_status"
GOAL_TOPIC = "/crsd/safe_passage"
REQUIRED_MODE = "GUIDED"


def fcu_problem(mode, armed):
    """Why the goal must not be sent, or None. `mode`/`armed` are what /crsd/fcu_status
    carried last; None means it carried nothing."""
    if mode is None or armed is None:
        return "no /crsd/fcu_status heard: the autopilot's state is unknown (telemetry_bridge up?)"
    if not armed:
        return "the autopilot is NOT ARMED: the pilot arms it with the RC, this tool never does"
    if str(mode).upper() != REQUIRED_MODE:
        return "the autopilot is in %s, not %s: the pilot selects GUIDED, this tool never changes the mode" % (
            mode, REQUIRED_MODE)
    return None


def parse_approach(text):
    """'lat,lon' -> (lat, lon); 'none' -> (0.0, 0.0) (the goal's own 'skip the drive').
    ValueError for anything else, including a point that is not on the globe."""
    if text.strip().lower() in ("none", "skip", ""):
        return 0.0, 0.0
    parts = text.split(",")
    if len(parts) != 2:
        raise ValueError("--approach is 'lat,lon' or 'none', got %r" % text)
    lat, lon = float(parts[0]), float(parts[1])
    if not (math.isfinite(lat) and math.isfinite(lon) and abs(lat) <= 90 and abs(lon) <= 180):
        raise ValueError("--approach %r is not a latitude/longitude" % text)
    return lat, lon


def wait_fcu(spin, latest, wait_s, clock=time.monotonic):
    """Spin until the FCU status has arrived (latest() -> (mode, armed) or (None, None)) or
    wait_s ran out; returns the last (mode, armed)."""
    t_end = clock() + wait_s
    while latest()[0] is None and clock() < t_end:
        spin(0.1)
    return latest()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--approach", default="none",
                    help="'lat,lon' to drive to before looking, or 'none' (default)")
    ap.add_argument("--tier", choices=sorted(TIERS), default="disruptive")
    ap.add_argument("--timeout-s", type=float, default=600.0)
    ap.add_argument("--orbit-radius-m", type=float, default=0.0, help="0 = the tree's default")
    ap.add_argument("--fcu-wait-s", type=float, default=5.0)
    a = ap.parse_args(argv)
    try:
        lat, lon = parse_approach(a.approach)
    except ValueError as e:
        print("[operator] %s" % e, flush=True)
        return 4

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from crusader_msgs.action import SafePassage
    from crusader_msgs.msg import FcuStatus

    rclpy.init()
    node = Node("lake_operator")
    seen = [None, None]

    def on_fcu(m):
        seen[0], seen[1] = m.mode, bool(m.armed)
    node.create_subscription(FcuStatus, FCU_TOPIC, on_fcu, 10)
    client = ActionClient(node, SafePassage, GOAL_TOPIC)
    stop = install_stop()                   # after rclpy.init(): see goal_client.install_stop

    def spin(t):
        rclpy.spin_once(node, timeout_sec=t)

    mode, armed = wait_fcu(spin, lambda: (seen[0], seen[1]), a.fcu_wait_s)
    why = fcu_problem(mode, armed)
    if why:
        print("[operator] REFUSED: %s" % why, flush=True)
        return 3
    print("[operator] autopilot reports armed, %s (the pilot's doing)" % mode, flush=True)
    if not client.wait_for_server(timeout_sec=20.0):
        print("[operator] no %s server (is bt_runner up?)" % GOAL_TOPIC, flush=True)
        return 2
    goal = SafePassage.Goal(tier=TIERS[a.tier], approach_latitude=lat, approach_longitude=lon,
                            orbit_radius_m=float(a.orbit_radius_m), timeout_s=float(a.timeout_s))
    print("[operator] goal: tier %d, approach %.7f %.7f, timeout %.0f s" % (
        TIERS[a.tier], lat, lon, a.timeout_s), flush=True)
    run = run_goal(client, goal, spin, stop)
    node.destroy_node()
    rclpy.shutdown()
    ok = run.status == "result" and run.result.outcome == SafePassage.Result.OUTCOME_SUCCESS
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
