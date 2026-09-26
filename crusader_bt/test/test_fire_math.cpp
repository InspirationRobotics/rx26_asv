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

  std::printf("the aim: upper-left window, 0.22 m left, from 3.2 m\n");
  {
    AimParams p;
    p.fire_range_m = 3.22;
    p.window_lat_m = 0.22;
    // boat on the centreline, wall normal due north (000)
    Aim a = solveAim(p, Lateral::Centreline, 3.2, 0.0, 0.0, kNaN, kNaN);
    chk("centreline aim ok", a.ok);
    const double psi = std::atan2(0.22, 3.2) / kDeg;
    chk_near("aim is ~3.9 deg left", a.aim_left_deg, 3.93, 0.02);
    // LEFT of north is 356: the heading gets SMALLER
    chk_near("heading target is 356.1 (left of 000)", a.heading_deg, wrap360(-psi), 1e-6);
    chk_near("range target = r_cal * cos(psi)", a.range_target_m, 3.22 * std::cos(psi * kDeg), 1e-9);
    chk("range correction is millimetres", std::fabs(a.range_target_m - 3.22) < 0.01);

    // fingers: the boat already 0.10 m LEFT of centre -> only 0.12 m to go
    Aim f = solveAim(p, Lateral::Fingers, 3.2, 90.0, 90.0, 0.10, kNaN);
    chk("fingers aim ok", f.ok);
    chk_near("aim from fingers", f.aim_left_deg, std::atan2(0.12, 3.2) / kDeg, 1e-9);
    chk_near("heading about an east-facing wall", f.heading_deg, 90.0 - std::atan2(0.12, 3.2) / kDeg, 1e-9);
    // the boat 0.30 m LEFT: the window is now to its RIGHT -> heading larger
    Aim r = solveAim(p, Lateral::Fingers, 3.2, 90.0, 90.0, 0.30, kNaN);
    chk("past the window: aim right (heading > normal)", r.heading_deg > 90.0);
    chk("no fingers -> refused, says why",
        !solveAim(p, Lateral::Fingers, 3.2, 90.0, 90.0, kNaN, kNaN).ok);

    // camera: the window seen 5 deg LEFT while heading 010, wall normal 000
    Aim c = solveAim(p, Lateral::Camera, 3.2, 0.0, 10.0, kNaN, 5.0);
    chk("camera aim ok", c.ok);
    const double y = (3.2 - p.cam_x_m) * std::tan(5.0 * kDeg);
    const double turn = std::atan2(y, 3.2) / kDeg;
    chk_near("camera: turn left from the CURRENT heading", c.heading_deg, 10.0 - turn, 1e-9);
    chk_near("camera: aim reported about the normal", c.aim_left_deg, -(10.0 - turn), 1e-9);
    chk("camera: window not in view -> refused",
        !solveAim(p, Lateral::Camera, 3.2, 0.0, 10.0, kNaN, kNaN).ok);

    // yaw bias adds straight on
    AimParams pb = p; pb.yaw_bias_deg = 1.5;
    chk_near("yaw bias + aims further left",
             solveAim(pb, Lateral::Centreline, 3.2, 0.0, 0.0, kNaN, kNaN).aim_left_deg,
             a.aim_left_deg + 1.5, 1e-9);
    // guard rails
    chk("no range -> refused", !solveAim(p, Lateral::Centreline, kNaN, 0.0, 0.0, kNaN, kNaN).ok);
    chk("no wall bearing -> refused", !solveAim(p, Lateral::Centreline, 3.2, kNaN, 0.0, kNaN, kNaN).ok);
    AimParams pw = p; pw.window_lat_m = 2.0;
    Aim wide = solveAim(pw, Lateral::Centreline, 3.2, 0.0, 0.0, kNaN, kNaN);
    chk("an aim wider than 20 deg is refused", !wide.ok && !wide.why.empty());
  }

  std::printf("the keep: banded, gentle, never forward past the floor\n");
  {
    KeepParams k;
    // at the target: nothing
    KeepCmd c = stationKeep(k, 3.22, 3.22, 0.0, 0.0, 0.0, 0.1);
    chk_near("in the band: zero speed", c.speed_mps, 0.0, 1e-12);
    chk("heading passes through", c.heading_deg == 0.0);
    // just outside the band: at least v_min, not 0.6*0.06 = 3.6 cm/s
    KeepParams fast = k; fast.accel = 100.0;
    c = stationKeep(fast, 3.28, 3.22, 0.0, 0.0, 0.0, 0.1);
    chk_near("just too far: v_min forward (no creeping)", c.speed_mps, k.v_min, 1e-9);
    c = stationKeep(fast, 3.16, 3.22, 0.0, 0.0, 0.0, 0.1);
    chk_near("just too close: v_min astern", c.speed_mps, -k.v_min, 1e-9);
    c = stationKeep(fast, 5.0, 3.22, 0.0, 0.0, 0.0, 0.1);
    chk_near("far: capped forward", c.speed_mps, k.v_max_fwd, 1e-9);
    c = stationKeep(fast, 2.0, 3.22, 0.0, 0.0, 0.0, 0.1);
    chk_near("close: capped astern", c.speed_mps, -k.v_max_rev, 1e-9);
    // slew: from rest, one 0.1 s tick allows accel*dt
    c = stationKeep(k, 5.0, 3.22, 0.0, 0.0, 0.0, 0.1);
    chk_near("slew-limited from rest", c.speed_mps, k.accel * 0.1, 1e-9);
    c = stationKeep(k, 3.22, 3.22, 0.0, 0.0, 0.2, 0.1);
    chk_near("slew-limited to a stop", c.speed_mps, 0.2 - k.accel * 0.1, 1e-9);
    // turn before driving
    c = stationKeep(fast, 5.0, 3.22, 30.0, 0.0, 0.0, 0.1);
    chk_near("heading error 30 deg: turn first, zero speed", c.speed_mps, 0.0, 1e-12);
    chk("says so", c.why == "turning first");
    // the floor: never forward closer than min_range
    KeepParams fl = fast; fl.min_range_m = 3.3;
    c = stationKeep(fl, 3.25, 3.0, 0.0, 0.0, 0.0, 0.1);
    chk_near("at the floor, no forward even if asked", c.speed_mps, 0.0, 1e-12);
    c = stationKeep(fl, 3.25, 3.6, 0.0, 0.0, 0.0, 0.1);
    chk("astern still allowed at the floor", c.speed_mps < 0);
    // no range: stop
    c = stationKeep(fast, kNaN, 3.22, 0.0, 0.0, 0.0, 0.1);
    chk_near("no range: zero", c.speed_mps, 0.0, 1e-12);
    // sign: too far out is FORWARD (closing in)
    c = stationKeep(fast, 3.6, 3.22, 0.0, 0.0, 0.0, 0.1);
    chk("too far out -> forward", c.speed_mps > 0 && c.why == "closing in");
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
    chk("aimed across north", aimed(359.0, 1.0, 3.0) && !aimed(355.0, 1.0, 3.0));

    BurstBook b;
    chk("ready at first", b.ready(0.0, 2.0));
    const double blank = b.record(10.0, 0.5, 0.8);
    chk_near("blank until burst + flight", blank, 11.3, 1e-9);
    chk("counted", b.fired == 1);
    chk("gap from the burst's END", !b.ready(12.0, 2.0) && b.ready(12.5, 2.0));
  }

  std::printf("square first, then aim\n");
  {
    Aim a; a.ok = true; a.heading_deg = 356.0;
    chk_near("far out: square", keepHeading(a, 0.0, 4.5, 3.22, 0.5), 0.0, 1e-9);
    chk_near("too close: square astern", keepHeading(a, 0.0, 2.0, 3.22, 0.5), 0.0, 1e-9);
    chk_near("near the range: aim", keepHeading(a, 0.0, 3.5, 3.22, 0.5), 356.0, 1e-9);
    Aim none;
    chk_near("far with no aim: still square", keepHeading(none, 90.0, 6.0, 3.22, 0.5), 90.0, 1e-9);
    chk("close with no aim: nothing", std::isnan(keepHeading(none, 90.0, 3.3, 3.22, 0.5)));
    chk("no wall bearing, no aim: nothing", std::isnan(keepHeading(none, kNaN, 6.0, 3.22, 0.5)));
  }

  std::printf("the LiDAR / camera cross-check\n");
  {
    // a face 20 m north facing south; the boat 3.2 m in front of it
    chk_near("in front", planeDistance(0.3, 16.8, 0.0, 20.0, 0.0, -1.0), 3.2, 1e-9);
    chk_near("behind is negative", planeDistance(0.0, 21.0, 0.0, 20.0, 0.0, -1.0), -1.0, 1e-9);
    chk("agree within tol", rangesAgree(3.22, 3.40, 0.5));
    chk("finger tips: 2 m short", !rangesAgree(1.22, 3.22, 0.5));
    chk("no camera: no check", rangesAgree(1.22, kNaN, 0.5));
    chk("tol 0: off", rangesAgree(1.22, 3.22, 0.0));
    chk("no LiDAR with a camera: disagree", !rangesAgree(kNaN, 3.22, 0.5));
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
