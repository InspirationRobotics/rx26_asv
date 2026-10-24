// test_fire_math — the fixed-nozzle shot's arithmetic, with no ROS and no boat.
//
//     g++ -std=c++17 -O2 -I include -o /tmp/t test/test_fire_math.cpp && /tmp/t
//
// The mistakes worth pinning down here all look plausible from the outside:
//
//   * TURNING THE WRONG WAY. The upper-left window is to the LEFT, and "turn
//     left" makes a compass heading SMALLER. Get one sign wrong and the boat
//     aims 4 degrees right, misses by half a metre, and the verdicts say "left"
//     forever.
//   * DRIVING INTO ITS OWN SPRAY. The stream returns LiDAR points short of the
//     wall. A filter that keeps them reads "too far out" and drives forward.
//   * CREEPING. A smooth law below ArduRover's speed floor asks for 3 cm/s,
//     gets nothing, and the boat never arrives.
#include <cmath>
#include <cstdio>
#include <string>

#include "crusader_bt/fire_math.hpp"

using namespace crusader_bt::fire;   // NOLINT(build/namespaces) — a test

static int g_fails = 0;
static int g_checks = 0;

static void chk(const std::string & name, bool pass)
{
  ++g_checks;
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s\n", pass ? "ok" : "FAIL", name.c_str());
}

static void chk_near(const std::string & name, double got, double want, double tol)
{
  const bool pass = std::fabs(got - want) <= tol;
  ++g_checks;
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s%s", pass ? "ok" : "FAIL", name.c_str(), pass ? "\n" : "");
  if (!pass) {std::printf("   (got %.6f, want %.6f)\n", got, want);}
}

static WallSample ws(double t, double range, double angle, double heading, double lat = kNaN,
                     bool valid = true)
{
  WallSample s;
  s.t = t; s.valid = valid; s.range_m = range; s.angle_deg = angle;
  s.heading_deg = heading; s.lat_m = lat;
  return s;
}

int main()
{
  std::printf("wrap\n");
  {
    chk_near("wrap180(190) = -170", wrap180(190), -170, 1e-9);
    chk_near("wrap180(-190) = 170", wrap180(-190), 170, 1e-9);
    chk_near("wrap360(-10) = 350", wrap360(-10), 350, 1e-9);
    chk_near("median even", median({4, 1, 3, 2}), 2.5, 1e-12);
    chk("median of nothing is NaN", std::isnan(median({})));
  }

  std::printf("the wall filter\n");
  {
    WallFilter f;
    for (int i = 0; i < 10; ++i) {f.add(ws(i * 0.1, 3.20 + (i % 2 ? 0.01 : -0.01), 2.0, 90.0));}
    chk_near("median range", f.range(0.9), 3.20, 0.011);
    // the boat heads 090 and the wall's nearest point is 2 deg LEFT: the wall
    // faces the boat from 088 (turn left = smaller heading)
    chk_near("inward normal = heading - angle", f.inward_deg(0.9), 88.0, 1e-9);
    chk_near("valid age", f.valid_age(1.0), 0.1, 1e-9);
    // spray: short returns while the water is in the air are ignored
    f.blank_until(1.6);
    f.add(ws(1.0, 2.60, 2.0, 90.0));
    f.add(ws(1.1, 2.70, 2.0, 90.0));
    f.add(ws(1.2, 2.55, 2.0, 90.0));
    chk("blanked", f.blanked(1.3));
    chk("blanked range is NaN, not the spray", std::isnan(f.range(1.3)));
    for (int i = 0; i < 6; ++i) {f.add(ws(1.7 + i * 0.1, 3.21, 2.0, 90.0));}
    chk_near("after blanking, the wall again", f.range(2.2), 3.21, 1e-9);
    // invalid samples do not count, and do not refresh the age
    WallFilter g;
    g.add(ws(0.0, 3.0, 0.0, 0.0, kNaN, false));
    chk("invalid only -> no range", std::isnan(g.range(0.0)));
    chk("invalid only -> age infinite", std::isinf(g.valid_age(0.0)));
    // the inward normal across north: headings 358/002, angle 0
    WallFilter h;
    h.add(ws(0.0, 3.0, 0.0, 358.0));
    h.add(ws(0.1, 3.0, 0.0, 2.0));
    h.add(ws(0.2, 3.0, 0.0, 0.0));
    const double in = h.inward_deg(0.2);
    chk("circular median across north (not 120)", std::fabs(wrap180(in - 0.0)) < 2.1);
    // lateral only from samples that saw both fingers
    WallFilter k;
    k.add(ws(0.0, 3.0, 0.0, 0.0, 0.05));
    k.add(ws(0.1, 3.0, 0.0, 0.0, kNaN));
    k.add(ws(0.2, 3.0, 0.0, 0.0, 0.07));
    chk_near("lat median of finite samples", k.lat(0.2), 0.06, 1e-9);
  }

  std::printf("steady\n");
  {
    SteadyMonitor m;
    auto feed = [&](double t0, double t1, double pitch_amp, double hz) {
      for (double t = t0; t < t1; t += 1.0 / hz) {
        const double w = 2 * kPi / 2.0;
        m.feed_att(t, 0.0, pitch_amp * std::sin(w * t), 0.0, pitch_amp * w * std::cos(w * t) / kDeg * kDeg);
      }
    };
    feed(0.0, 3.0, 0.0, 30.0);
    m.feed_cmd(0.0, 0.0);
    SteadyStatus s = m.status(2.99);
    chk("flat, no thrust -> steady", s.steady);
    m.feed_cmd(2.5, 0.2);            // we drove at 2.5 s
    s = m.status(2.99);
    chk("thrust 0.5 s ago -> not steady", !s.steady && s.why == "thrust too recent");
    m.feed_cmd(2.6, 0.0);
    feed(3.0, 5.0, 0.0, 30.0);
    chk("1.5 s after the last thrust -> steady", m.status(4.99).steady);
    // rocking: 3 deg at 0.5 Hz -> ~9.4 deg/s peak rate
    SteadyMonitor r;
    r.feed_cmd(0.0, 0.0);
    for (double t = 0; t < 6; t += 1.0 / 30.0) {
      const double w = 2 * kPi * 0.5;
      r.feed_att(t, 0.0, 3.0 * std::sin(w * t), 0.0, 3.0 * w * std::cos(w * t));
    }
    s = r.status(5.99);
    chk("rocking -> not steady", !s.steady);
    chk("rocking reason", s.why == "rocking" || s.why == "pitch swinging");
    // a steady LIST is fine
    SteadyMonitor l;
    l.feed_cmd(0.0, 0.0);
    for (double t = 0; t < 6; t += 1.0 / 30.0) {l.feed_att(t, 4.0, -2.0, 0.0, 0.0);}
    chk("a steady list is steady", l.status(5.99).steady);
    // too slow to judge
    SteadyMonitor q;
    q.feed_cmd(0.0, 0.0);
    for (double t = 0; t < 6; t += 0.25) {q.feed_att(t, 0.0, 0.0, 0.0, 0.0);}
    s = q.status(5.9);
    chk("4 Hz attitude -> refused", !s.steady && s.why.find("Hz") != std::string::npos);
    chk("stale attitude -> refused", l.status(7.0).why == "no attitude");
  }

  std::printf("the gate and the book\n");
  {
    SolutionGate g; g.hold_s = 1.0;
    chk("not yet", !g.update(0.0, true));
    chk("0.9 s", !g.update(0.9, true));
    chk("held 1 s", g.update(1.0, true));
    chk("a break resets", !g.update(1.1, false));
    chk("and it starts over", !g.update(1.2, true) && !g.update(2.1, true) && g.update(2.2, true));
    chk("in band", inBand(3.25, 3.22, 0.05) && !inBand(3.30, 3.22, 0.05) && !inBand(kNaN, 3.22, 1));
  }

  std::printf("the plane distance\n");
  {
    // a face 20 m north facing south; the boat 3.2 m in front of it
    chk_near("in front", planeDistance(0.3, 16.8, 0.0, 20.0, 0.0, -1.0), 3.2, 1e-9);
    chk_near("behind is negative", planeDistance(0.0, 21.0, 0.0, 20.0, 0.0, -1.0), -1.0, 1e-9);
  }

  std::printf("the wall filter's rate\n");
  {
    WallFilter w;
    for (int i = 0; i <= 10; ++i) {w.add(WallSample{i * 0.1, true, 3.5 - 0.1 * i * 0.1, 0, kNaN, 0});}
    chk_near("range rate", w.rate(1.0), -0.1, 1e-9);
  }

  std::printf("the strafe keep\n");
  {
    StrafeParams p;
    chk("in the band: nothing", axisLaw(0.04, 0.0, 90, 60, 0.05, 30, 120) == 0.0);
    chk_near("just out: the deadband offset plus P", axisLaw(0.06, 0.0, 90, 60, 0.05, 30, 120),
      30.0 + 90 * 0.06, 1e-9);
    chk_near("half a metre: offset + 45 us, not 45", axisLaw(0.5, 0.0, 90, 60, 0.05, 30, 120),
      75.0, 1e-9);
    chk_near("far out: capped", axisLaw(3.0, 0.0, 90, 60, 0.05, 30, 120), 120.0, 1e-9);
    chk("arriving within 1.5 s: coast", axisLaw(0.2, -0.29, 90, 60, 0.05, 30, 120) == 0.0);
    chk("creeping in (2 cm/s, 16 cm out): keep pushing",
      axisLaw(0.16, -0.02, 90, 60, 0.05, 30, 120) > 30.0);
    chk("overshooting: brake", axisLaw(0.1, -0.5, 90, 60, 0.05, 30, 120) < 0.0);

    StrafeInputs in;
    in.range_m = 4.0; in.range_rate = 0.0; in.lat_err_m = 0.2; in.lat_rate = 0.0;
    in.yaw_err_deg = 0.0; in.yaw_rate_dps = 0.0;
    StrafeState big; big.prev.fwd_us = 200; big.prev.lat_us = -200;
    StrafeCmd c = strafeKeep(p, in, big, 1.0);
    // stalled 0.78 m out for 1 s: offset + P, and the integrator has started
    chk_near("too far and stalled: ahead, offset + P + I", c.sticks.fwd_us,
      p.min_us + 90.0 * 0.78 + p.ki_fwd * 0.78 * 1.0, 1e-6);
    chk("window LEFT: slide left (lat -)", c.sticks.lat_us < -15.0);
    chk("square: no yaw", c.sticks.yaw_us == 0.0);
    chk("not on the spot", !c.range_ok && !c.lat_ok && c.yaw_ok);

    in.yaw_err_deg = 20.0;
    big.prev.fwd_us = 200; big.prev.lat_us = -200;
    c = strafeKeep(p, in, big, 1.0);
    chk("far off square: turn only", c.sticks.fwd_us == 0.0 && c.sticks.lat_us == 0.0 &&
      c.sticks.yaw_us > 0.0 && c.why == "squaring up first");

    in.yaw_err_deg = 0.0; in.range_m = 1.4;
    big.prev.fwd_us = 200; big.i_fwd = 50;
    c = strafeKeep(p, in, big, 1.0);
    chk("inside the minimum range: never ahead", c.sticks.fwd_us <= 0.0);
    {
      // the target itself inside the floor, and an integrator pushing ahead:
      // the floor wins, and the integrator is emptied
      StrafeParams q = p; q.fire_range_m = 1.45;
      StrafeState s2; s2.i_fwd = 50.0; s2.prev.fwd_us = 50.0;
      StrafeInputs i2 = in; i2.range_m = 1.40; i2.range_rate = 0.0;
      const StrafeCmd c2 = strafeKeep(q, i2, s2, 0.1);
      chk("the floor beats the integrator", c2.sticks.fwd_us <= 0.0 && s2.i_fwd <= 0.0);
    }

    StrafeState z;
    in.range_m = 3.22; in.lat_err_m = 0.0;
    c = strafeKeep(p, in, z, 1.0);
    chk("on the spot: all zero", c.sticks.max_abs() == 0.0 && c.range_ok && c.lat_ok && c.yaw_ok &&
      c.why == "on the spot" && !c.correcting);

    StrafeState z2;
    in.range_m = 5.0;
    c = strafeKeep(p, in, z2, 0.1);
    chk_near("slew-limited", c.sticks.fwd_us, 20.0, 1e-9);
    chk("far out: correcting, no windup", c.correcting && z2.i_fwd == 0.0);
    {
      // arriving (10 cm out, closing at 8 cm/s): no windup either
      StrafeState a;
      StrafeInputs ai = in; ai.range_m = 3.32; ai.range_rate = -0.08;
      for (int i = 0; i < 20; ++i) {strafeKeep(p, ai, a, 0.1);}
      chk("arriving: no windup", a.i_fwd == 0.0);
    }

    StrafeState z3;
    in.range_m = 3.22; in.lat_err_m = kNaN;
    c = strafeKeep(p, in, z3, 1.0);
    chk("no window: no sideways thrust", c.sticks.lat_us == 0.0);

    // a current pushing the boat right: the window sits 2 cm LEFT (inside the
    // band) for 10 s. P gives nothing there; I builds a steady push LEFT.
    StrafeState cur;
    in.lat_err_m = 0.02; in.lat_rate = 0.0;
    for (int i = 0; i < 100; ++i) {c = strafeKeep(p, in, cur, 0.1);}
    chk_near("I: a steady push left against the current", c.sticks.lat_us, -0.02 * 30.0 * 10.0, 0.5);
    chk("... and holding is not correcting", !c.correcting && c.lat_ok);
    for (int i = 0; i < 2000; ++i) {c = strafeKeep(p, in, cur, 0.1);}
    chk_near("I is capped", c.sticks.lat_us, -80.0, 1e-9);
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
