"""weather — wind and a water current on the simulated boat.

    python3 -m crusader_sim.weather run                     # sim.sh / gz_sim_up.sh start it: calm
    python3 -m crusader_sim.weather wind 6 90 [--gust 0.3] [--swing 30]
                                    # 6 m/s, pushing TOWARD 90 (east); gusts +-30%, the
                                    # direction wandering +-30 deg
    python3 -m crusader_sim.weather current 0.2 0 [--vary 0.5] [--swing 40]
                                    # 0.2 m/s, flowing TOWARD 0 (north), the speed wandering
                                    # +-50% and the direction +-40 deg
    python3 -m crusader_sim.weather rough | light           # both, varying (PRESETS)
    python3 -m crusader_sim.weather calm                    # both off
    python3 -m crusader_sim.weather show                    # the settings, and what it is now

Runs in: WSL, next to Gazebo (gz-transport's Python bindings, no ROS), like
task3_gz_agent. `run` applies whatever the settings file (SETTINGS) says, and the
other commands only edit that file, so they take effect within a fifth of a second
and the sim never restarts for the weather.

DIRECTIONS are compass degrees the air or the water moves TOWARD (0 north, 90 east).
On the Task 3 courses the dock is NORTH of the start with its slips opening south:
0 pushes the boat INTO the slip, 180 out of it, 90 and 270 across it.

THE CURRENT goes to the hull's own Hydrodynamics system (/ocean_current, world
frame): its drag works on the boat's velocity THROUGH THE WATER, so the hull drifts
with the current the way a real one does, and holding still costs thrust.

THE WIND is a force on the hull (ApplyLinkWrench's persistent wrench, world frame,
at the hull's origin): F = 0.5 * rho_air * CdA * v^2 along the wind, CdA = WIND_CDA
(0.5 m^2: a GUESS for the hull, mast, camera and cannon side-on; ~8 N at 5 m/s,
~31 N at 10 m/s). Gusts: the speed times (1 + gust * n(t)), n an Ornstein-Uhlenbeck
noise (unit variance, GUST_TAU_S), never below zero.

VARYING (2026-10-18): the wind's direction wanders by +-swing deg (one sigma, an OU
noise, SWING_TAU_S), and the current's speed by +-vary and its direction by +-swing
(CURRENT_TAU_S): neither holds still the way a constant setting does, so a hold that
only works against a steady push shows it. Each noise is clipped at 2 sigma. What
it actually is goes to the log (/tmp/weather.log) every LOG_PERIOD_S, and `show`
prints the last of it. Not modelled: the turning moment a real wind puts on the
bow, and waves.
"""
import argparse
import json
import math
import os
import random
import sys
import time

WORLD = "crusader_sim"
MODEL = "crusader"
SETTINGS = os.path.expanduser("~/.cache/crusader_sim/weather.json")
RHO_AIR = 1.225
WIND_CDA = 0.5         # m^2, a guess (see above)
GUST_TAU_S = 4.0
SWING_TAU_S = 15.0     # the wind's direction wanders this slowly
CURRENT_TAU_S = 20.0   # the current's speed and direction
WIND_PERIOD_S = 0.5    # how often the (gusting) force is re-set
CURRENT_PERIOD_S = 0.5 # the current is re-sent this often (a late subscriber still gets it)
LOG_PERIOD_S = 5.0
NOW_FILE = os.path.expanduser("~/.cache/crusader_sim/weather_now.txt")
CALM = {"wind": {"mps": 0.0, "toward_deg": 0.0, "gust": 0.3, "swing": 0.0},
        "current": {"mps": 0.0, "toward_deg": 0.0, "vary": 0.0, "swing": 0.0}}
# 0 = toward the dock (into the slips) on the Task 3 courses, 90 = across them
PRESETS = {
    "rough": {"wind": {"mps": 6.0, "toward_deg": 90.0, "gust": 0.4, "swing": 30.0},
              "current": {"mps": 0.15, "toward_deg": 0.0, "vary": 0.5, "swing": 40.0}},
    "light": {"wind": {"mps": 3.0, "toward_deg": 90.0, "gust": 0.3, "swing": 20.0},
              "current": {"mps": 0.05, "toward_deg": 0.0, "vary": 0.5, "swing": 30.0}},
}


def toward(deg):
    """(east, north) unit vector for a compass direction."""
    r = math.radians(deg)
    return math.sin(r), math.cos(r)


def wind_force(mps):
    return 0.5 * RHO_AIR * WIND_CDA * mps * mps


def load():
    try:
        with open(SETTINGS, encoding="utf-8") as f:
            d = json.load(f)
        out = json.loads(json.dumps(CALM))
        for k in out:
            out[k].update(d.get(k, {}))
        return out
    except (OSError, ValueError):
        return json.loads(json.dumps(CALM))


def save(d):
    os.makedirs(os.path.dirname(SETTINGS), exist_ok=True)
    tmp = SETTINGS + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(d, f)
    os.replace(tmp, SETTINGS)


def describe(d):
    w, c = d["wind"], d["current"]
    s = []
    if w["mps"] > 0:
        s.append(f"wind {w['mps']:.1f} m/s toward {w['toward_deg']:.0f} deg "
                 f"(~{wind_force(w['mps']):.0f} N on the hull, gusts +-{100 * w['gust']:.0f}%"
                 + (f", direction +-{w['swing']:.0f} deg" if w.get("swing", 0) > 0 else "") + ")")
    if c["mps"] > 0:
        extra = []
        if c.get("vary", 0) > 0:
            extra.append(f"speed +-{100 * c['vary']:.0f}%")
        if c.get("swing", 0) > 0:
            extra.append(f"direction +-{c['swing']:.0f} deg")
        s.append(f"current {c['mps']:.2f} m/s toward {c['toward_deg']:.0f} deg"
                 + (f" ({', '.join(extra)})" if extra else ""))
    return "; ".join(s) or "calm"


class OU:
    """Ornstein-Uhlenbeck noise, unit variance, time constant tau; clipped at 2 sigma."""

    def __init__(self, tau, rng=random):
        self.tau, self.n, self.rng = tau, 0.0, rng

    def step(self, dt):
        self.n += (-self.n / self.tau) * dt + math.sqrt(2.0 * dt / self.tau) * self.rng.gauss(0, 1)
        self.n = max(-2.0, min(2.0, self.n))
        return self.n


def actual(d, gust, wdir, cspd, cdir):
    """What the weather is NOW: (wind m/s, wind toward deg, current m/s, current toward
    deg), from the settings and the four noises' current values."""
    w, c = d["wind"], d["current"]
    ws = max(0.0, w["mps"] * (1.0 + w["gust"] * gust))
    wd = (w["toward_deg"] + w.get("swing", 0.0) * wdir) % 360.0
    cs = max(0.0, c["mps"] * (1.0 + c.get("vary", 0.0) * cspd))
    cd = (c["toward_deg"] + c.get("swing", 0.0) * cdir) % 360.0
    return ws, wd, cs, cd


def run():
    from gz.transport13 import Node
    from gz.msgs10.entity_pb2 import Entity
    from gz.msgs10.entity_wrench_pb2 import EntityWrench
    from gz.msgs10.vector3d_pb2 import Vector3d

    node = Node()
    cur_pub = node.advertise("/ocean_current", Vector3d)
    set_pub = node.advertise(f"/world/{WORLD}/wrench/persistent", EntityWrench)
    clr_pub = node.advertise(f"/world/{WORLD}/wrench/clear", Entity)
    save(json.loads(json.dumps(CALM)))       # every bring-up starts calm
    time.sleep(1.0)                          # discovery
    print(f"[weather] running: {SETTINGS} (calm)", flush=True)

    def clear():
        e = Entity()
        e.name, e.type = MODEL, Entity.MODEL
        clr_pub.publish(e)

    mtime, d = None, load()
    gust, wdir, cspd, cdir = OU(GUST_TAU_S), OU(SWING_TAU_S), OU(CURRENT_TAU_S), OU(CURRENT_TAU_S)
    t_wind, t_cur, t_log, wind_on = 0.0, 0.0, 0.0, False
    last_t = time.monotonic()
    while True:
        now = time.monotonic()
        dt, last_t = now - last_t, now
        for o in (gust, wdir, cspd, cdir):
            o.step(dt)
        ws, wd, cs, cd = actual(d, gust.n, wdir.n, cspd.n, cdir.n)
        try:
            m = os.stat(SETTINGS).st_mtime
        except OSError:
            m = None
        if m != mtime:
            mtime, d = m, load()
            t_wind = t_cur = 0.0                 # apply at once
            print(f"[weather] {describe(d)}", flush=True)
        # the current (its speed and direction wandering)
        if now - t_cur >= CURRENT_PERIOD_S:
            t_cur = now
            e, n = toward(cd)
            v = Vector3d()
            v.x, v.y, v.z = cs * e, cs * n, 0.0
            cur_pub.publish(v)
        if now - t_log >= LOG_PERIOD_S:
            t_log = now
            line = (f"now: wind {ws:.1f} m/s toward {wd:.0f} deg (~{wind_force(ws):.0f} N), "
                    f"current {cs:.2f} m/s toward {cd:.0f} deg")
            if d["wind"]["mps"] > 0 or d["current"]["mps"] > 0:
                print(f"[weather] {line}", flush=True)
            try:
                with open(NOW_FILE, "w", encoding="utf-8") as f:
                    f.write(line + "\n")
            except OSError:
                pass
        # the wind, with gusts and a wandering direction
        w = d["wind"]
        if w["mps"] <= 0.0:
            if wind_on:
                clear()
                wind_on = False
        elif now - t_wind >= WIND_PERIOD_S:
            t_wind = now
            f = wind_force(ws)
            e, n = toward(wd)
            clear()                              # persistent wrenches ADD up: replace it
            time.sleep(0.02)
            msg = EntityWrench()
            msg.entity.name, msg.entity.type = MODEL, Entity.MODEL
            msg.wrench.force.x, msg.wrench.force.y = f * e, f * n
            set_pub.publish(msg)
            wind_on = True
        time.sleep(0.2)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("run")
    p = sp.add_parser("wind")
    p.add_argument("mps", type=float, help="wind speed, m/s (0 = none)")
    p.add_argument("toward_deg", type=float, nargs="?", default=0.0,
                   help="compass deg it blows TOWARD (0 = into the slip on the Task 3 courses)")
    p.add_argument("--gust", type=float, default=None, help="gust size, fraction (default 0.3)")
    p.add_argument("--swing", type=float, default=None,
                   help="the direction wanders this much, deg (one sigma; default 0)")
    p = sp.add_parser("current")
    p.add_argument("mps", type=float, help="current speed, m/s (0 = none)")
    p.add_argument("toward_deg", type=float, nargs="?", default=0.0,
                   help="compass deg it flows TOWARD")
    p.add_argument("--vary", type=float, default=None,
                   help="the speed wanders this much, fraction (one sigma; default 0)")
    p.add_argument("--swing", type=float, default=None,
                   help="the direction wanders this much, deg (one sigma; default 0)")
    for name in PRESETS:
        sp.add_parser(name)
    sp.add_parser("calm")
    sp.add_parser("show")
    a = ap.parse_args(argv)
    if a.cmd == "run":
        run()
        return 0
    d = load()
    if a.cmd == "wind":
        d["wind"].update(mps=max(0.0, a.mps), toward_deg=a.toward_deg % 360.0)
        if a.gust is not None:
            d["wind"]["gust"] = max(0.0, a.gust)
        if a.swing is not None:
            d["wind"]["swing"] = max(0.0, a.swing)
    elif a.cmd == "current":
        d["current"].update(mps=max(0.0, a.mps), toward_deg=a.toward_deg % 360.0)
        if a.vary is not None:
            d["current"]["vary"] = max(0.0, a.vary)
        if a.swing is not None:
            d["current"]["swing"] = max(0.0, a.swing)
    elif a.cmd in PRESETS:
        d = json.loads(json.dumps(PRESETS[a.cmd]))
    elif a.cmd == "calm":
        d = json.loads(json.dumps(CALM))
    if a.cmd != "show":
        save(d)
    print(describe(d))
    if a.cmd == "show":
        try:
            with open(NOW_FILE, encoding="utf-8") as f:
                print(f.read().strip())
        except OSError:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
