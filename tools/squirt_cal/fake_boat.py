"""fake_boat — a boat, a pilot, a pump and an honest operator, for --fake.

The whole tool runs against this on a laptop with no ROS: the page, the state
machine, the estimator. Its nozzle is NOT the one in the config: the truth is
drawn a little off (range, elevation, height, a sideways bias), the way the
real nozzle will be, so the tool has to find the answer rather than read it
back out of its own model.

What it models:
  * rocking: a few sines under a slow envelope, so there are calm spells and
    rough ones; driving kicks the pitch
  * the pilot: holds a spot, drifts, and moves when nudged (or, with
    follow_advice, drives to where the tool says), moving the sticks while it
    does
  * the LiDAR: the range to the wall with centimetre noise, and SPRAY returns
    (short ranges) while the water is in the air
  * the pump: a little latency before water
  * the operator: oracle_verdict() says where the boat should go, from where
    the stream really hit, wrong now and then if you ask it to be

Stdlib only.
"""
import math
import random

from nozzle_model import NozzleModel, target_height


class FakeBoat:

    def __init__(self, cfg, seed=0):
        self.cfg = cfg
        rng = random.Random(seed)
        self.rng = rng
        # the hidden truth, deliberately not the config's numbers
        self.truth = NozzleModel.from_range(
            cfg["nozzle_range_m"] * rng.uniform(0.88, 1.03),
            cfg["nozzle_elev_deg"] + rng.uniform(-3.0, 3.0),
            cfg["nozzle_height_m"] + rng.uniform(-0.03, 0.03))
        self.deck = cfg["deck_height_m"] + rng.uniform(-0.02, 0.04)
        self.yaw_bias_deg = rng.uniform(-2.0, 2.0)      # nozzle not quite straight
        self.nozzle_x = cfg["nozzle_x_m"]
        self.setback = cfg["face_setback_m"]
        self.phase = [rng.uniform(0, 2 * math.pi) for _ in range(6)]
        # state
        self.t = None
        self.range = 1.6
        self.lat = rng.uniform(-0.08, 0.08)
        self.yaw_deg = rng.uniform(-2.0, 2.0)           # wall angle as ranged
        self.goal = (self.range, self.lat)              # where the pilot is holding
        self.moving_until = -1.0
        self.sea = 1.0                                   # 0 flat .. 3 rough
        self.drift = 0.01                                # m/s random walk scale
        self.follow_advice = False
        self.fingers = True
        self.pump_path = True                            # the bridge can fire
        self.p_wrong = 0.0                               # operator error rate
        self.pump_on_until = -1.0
        self.pump_on_from = -1.0
        self.latency = 0.12
        self.last_hit = None

    # ---- motion ----

    def attitude(self, t):
        """(roll, pitch, roll_rate, pitch_rate), degrees and deg/s."""
        ph = self.phase
        env = self.sea * (0.35 + 0.65 * (0.5 + 0.5 * math.sin(2 * math.pi * t / 23.0 + ph[5])))
        comps_r = ((1.4, 2.4, ph[0]), (0.5, 1.1, ph[1]))
        comps_p = ((1.0, 3.2, ph[2]), (0.4, 1.3, ph[3]))
        kick = 1.5 if t < self.moving_until else 0.0
        r = sum(a * math.sin(2 * math.pi * t / T + p) for a, T, p in comps_r)
        rr = sum(a * 2 * math.pi / T * math.cos(2 * math.pi * t / T + p)
                 for a, T, p in comps_r)
        pt = sum(a * math.sin(2 * math.pi * t / T + p) for a, T, p in comps_p)
        pr = sum(a * 2 * math.pi / T * math.cos(2 * math.pi * t / T + p)
                 for a, T, p in comps_p)
        pk = kick * math.sin(2 * math.pi * t / 0.9)
        pkr = kick * 2 * math.pi / 0.9 * math.cos(2 * math.pi * t / 0.9)
        return env * r, env * pt + pk, env * rr, env * pr + pkr

    def sticks(self, t):
        if t < self.moving_until:
            return [1500 + int(80 * math.sin(3 * t)), 1500, 1540, 1500]
        return [1500, 1500, 1500, 1500]

    def nudge(self, t, d_range=0.0, d_lat=0.0):
        g = self.goal
        self.set_goal(t, g[0] + d_range, g[1] + d_lat)

    def set_goal(self, t, rng=None, lat=None):
        g = self.goal
        new = (g[0] if rng is None else rng, g[1] if lat is None else lat)
        dist = math.hypot(new[0] - self.range, new[1] - self.lat)
        self.goal = new
        if dist > 0.005:
            self.moving_until = t + 0.8 + dist / 0.15

    def step(self, t):
        if self.t is None:
            self.t = t
            return
        while self.t < t:
            dt = min(0.02, t - self.t)
            self.t += dt
            # the pilot closes on the goal at ~0.15 m/s; the water pushes it around
            for i, attr in enumerate(("range", "lat")):
                cur = getattr(self, attr)
                err = self.goal[i] - cur
                v = max(-0.15, min(0.15, 1.5 * err))
                cur += v * dt + self.rng.gauss(0.0, self.drift) * math.sqrt(dt)
                setattr(self, attr, cur)

    # ---- sensors ----

    def measured(self, t):
        """(valid, range, angle, lat or None) as the wall fit would report."""
        rng = self.range + self.rng.gauss(0.0, 0.006)
        if self.pump_on_from <= t <= self.pump_on_until + 0.3 and self.rng.random() < 0.4:
            rng = self.range - self.rng.uniform(0.1, 0.6)        # spray returns
        lat = self.lat + self.rng.gauss(0.0, 0.008) if self.fingers else None
        return True, rng, self.yaw_deg + self.rng.gauss(0.0, 0.3), lat

    # ---- the pump and the stream ----

    def fire(self, t, burst_s):
        self.pump_on_from = t + self.latency
        self.pump_on_until = t + self.latency + burst_s
        mid = t + self.latency + burst_s / 2.0
        roll, pitch, _, _ = self.attitude(mid)
        x = self.range - self.nozzle_x - self.setback
        z = self.truth.height_at(x, pitch)
        y = (self.lat + x * math.tan(math.radians(self.yaw_bias_deg + self.yaw_deg))
             - (z - self.truth.h) * math.sin(math.radians(roll)))
        self.last_hit = dict(t=t, x=x, z=z, y=y, pitch=pitch, roll=roll)
        return self.last_hit

    def target(self, edge_mm, lat_m):
        return target_height(self.deck, edge_mm), lat_m

    def truth_range(self, edge_mm, branch="near"):
        """The range at which a level boat's stream hits this edge, or None."""
        x = self.truth.solve_x(target_height(self.deck, edge_mm), branch)
        return None if x is None else x + self.nozzle_x + self.setback

    def oracle_verdict(self, edge_mm, lat_m, tol_z=0.03, tol_y=0.04):
        """(fa, lat) for the last hit: where the BOAT should have been."""
        h = self.last_hit
        if h is None:
            return None
        zt, yt = self.target(edge_mm, lat_m)
        dz, dy = h["z"] - zt, h["y"] - yt
        if abs(dz) <= tol_z:
            fa = "ok"
        else:
            slope = self.truth.slope_at(h["x"], h["pitch"])
            apex_x, apex_z = self.truth.apex(h["pitch"])
            if zt > apex_z:                        # out of reach: head for the apex
                want_dx = apex_x - h["x"]
            else:
                want_dx = (-dz) * slope
            fa = "fwd" if want_dx > 0 else "back"   # needs more range = too far fwd
        lat = "ok" if abs(dy) <= tol_y else ("left" if dy > 0 else "right")
        if self.p_wrong and self.rng.random() < self.p_wrong:
            fa = {"fwd": "back", "back": "fwd", "ok": "fwd"}[fa]
        if self.p_wrong and self.rng.random() < self.p_wrong:
            lat = {"left": "right", "right": "left", "ok": "left"}[lat]
        return fa, lat


class FakeAdapter:
    """Feeds App from a FakeBoat, and lets the page poke the boat."""

    RATE_HZ = 30.0      # attitude, as SR0_EXTRA1=30 would give

    def __init__(self, boat):
        self.boat = boat
        self.app = None
        self._next_att = None
        self._next_range = None
        self._next_rc = None

    def attach(self, app):
        self.app = app

    def spin(self, stop):
        while not stop.wait(0.2):
            pass

    def stop(self):
        pass

    def info(self):
        return dict(kind="fake", pump_path=self.boat.pump_path,
                    status_line="fake bridge: " + ("pump path up" if self.boat.pump_path
                                                   else "no pump path (log only)"))

    def can_fire(self):
        if not self.boat.pump_path:
            return False, "fake bridge: pump path disabled (log only; use Pilot squirt)"
        return True, "ok"

    def fire(self, burst_s, seq):
        self.boat.fire(self.boat.t, burst_s)
        return True, f"fake burst {burst_s:.2f} s (seq {seq})"

    def snapshot(self, path):
        return False

    def poll(self, now):
        b, app = self.boat, self.app
        b.step(now)
        if self._next_att is None:
            self._next_att = self._next_range = self._next_rc = now
        while self._next_att <= now:
            r, p, rr, pr = b.attitude(self._next_att)
            app.on_att(self._next_att, r, p, rr, pr)
            self._next_att += 1.0 / self.RATE_HZ
        while self._next_range <= now:
            app.on_range(self._next_range, *b.measured(self._next_range))
            self._next_range += 0.1
        while self._next_rc <= now:
            app.on_sticks(self._next_rc, b.sticks(self._next_rc))
            self._next_rc += 0.1
        if b.follow_advice and app.state == "idle":
            adv = app.advice(now)
            g_rng, g_lat = adv["goal_range_m"], adv["goal_lat_m"]
            new_rng = g_rng is not None and abs(b.goal[0] - g_rng) > 0.005
            new_lat = g_lat is not None and abs(b.goal[1] - g_lat) > 0.005
            if new_rng or new_lat:
                # the pilot gets there to within a couple of centimetres
                b.set_goal(now,
                           rng=g_rng + b.rng.gauss(0.0, 0.015) if new_rng else None,
                           lat=g_lat + b.rng.gauss(0.0, 0.015) if new_lat else None)

    # ---- the page's fake panel ----

    def fake_state(self, target_id):
        b = self.boat
        tgt = next(t for t in self.app.targets if t[0] == target_id)
        tr = b.truth_range(tgt[2], self.app.cfg["branch"])
        return dict(sea=b.sea, follow_advice=b.follow_advice, fingers=b.fingers,
                    pump_path=b.pump_path, p_wrong=b.p_wrong,
                    true_range_m=round(b.range, 3), true_lat_m=round(b.lat, 3),
                    truth_range_m=None if tr is None else round(tr, 3),
                    apex_z_m=round(b.truth.apex()[1], 3),
                    last_hit=b.last_hit and {k: round(v, 3) for k, v in b.last_hit.items()})

    def fake_action(self, path, payload, app):
        b, now = self.boat, app.clock()
        if path == "/fake/nudge":
            try:
                b.nudge(now, float(payload.get("d_range", 0)), float(payload.get("d_lat", 0)))
            except (TypeError, ValueError):
                return dict(ok=False, message="bad nudge")
            return dict(ok=True, message="pilot moving")
        if path == "/fake/set":
            for key, kind in (("sea", float), ("p_wrong", float),
                              ("follow_advice", bool), ("fingers", bool),
                              ("pump_path", bool)):
                if key in payload:
                    setattr(b, key, kind(payload[key]))
            return dict(ok=True, message="fake updated")
        if path == "/fake/pilot_squirt":
            burst = float(payload.get("burst_s", 0.4))
            b.fire(now, burst)
            app.on_pump_edge(now + b.latency, True)
            app.on_pump_edge(now + b.latency + burst, False)
            return dict(ok=True, message="pilot squirted")
        if path == "/fake/oracle":
            tgt = next(t for t in app.targets if t[0] == app.target)
            v = b.oracle_verdict(tgt[2], tgt[3])
            if v is None:
                return dict(ok=False, message="no shot yet")
            ok, msg = app.verdict(*v)
            return dict(ok=ok, message=f"oracle: {v[0]}/{v[1]} ({msg})")
        return dict(ok=False, message=f"unknown fake action {path}")
