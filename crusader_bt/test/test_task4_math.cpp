// test_task4_math — Task 4 Disruptive: RoboCommand's moving object (task4_math.hpp).
// The alert as the OCS relays it, dead reckoning, the keep-out the planners avoid, the threat
// verdict and the escape point.
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include -o /tmp/t test/test_task4_math.cpp && /tmp/t
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "crusader_bt/task4_math.hpp"

using namespace crusader_bt;            // NOLINT(build/namespaces) -- a test
using nav::Vec2;

static int g_fails = 0;

static void chk(const std::string & name, bool pass)
{
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s\n", pass ? "ok" : "FAIL", name.c_str());
}

static bool near(double a, double b, double tol = 1e-6) {return std::fabs(a - b) < tol;}

/// One object heading `hdg` at `v` m/s from p0, reported at t0 = 0, keep-out 10 m.
static t4::Invader obj(Vec2 p0, double hdg, double v, double r = 10.0)
{
  t4::Invader o;
  o.id = 1;
  o.p0 = p0;
  o.heading_deg = hdg;
  o.speed_mps = v;
  o.radius_m = r;
  return o;
}

int main()
{
  t4::Settings s;              // margin 3, horizon 40 s, sweep 20 s, boat 0.5 m/s

  std::printf("1. the alert as the OCS relays it (protobuf JSON, proto field names)\n");
  {
    t4::Alert a;
    const std::string full =
      "{\"seq\": 12, \"moving_object_alert\": {\"position\": {\"latitude\": 1.2805, "
      "\"longitude\": 103.8557}, \"heading_deg\": 270.5, \"speed_mps\": 1.25, "
      "\"affected_vehicle_types\": [\"TYPE_USV\", \"TYPE_UAV\"], \"keep_out_radius_m\": 8.0}}";
    chk("parses", t4::parseAlert(full, &a));
    chk("lat/lon", near(a.lat, 1.2805) && near(a.lon, 103.8557));
    chk("heading 270.5, speed 1.25", near(a.heading_deg, 270.5) && near(a.speed_mps, 1.25));
    chk("radius 8", near(a.radius_m, 8.0));
    chk("domains USV UAV", a.domains.size() == 2 && a.domains[0] == "USV" && a.domains[1] == "UAV");
    // protobuf omits zeros: heading 0 (north), speed 0, no radius, no domains
    const std::string bare =
      "{\"moving_object_alert\":{\"position\":{\"latitude\":1.28,\"longitude\":103.85}}}";
    chk("a bare alert parses", t4::parseAlert(bare, &a));
    chk("absent fields are zero", a.heading_deg == 0.0 && a.speed_mps == 0.0 && a.radius_m == 0.0);
    chk("no domains named", a.domains.empty());
    chk("numeric enums: [2] is UUV only",
      t4::parseAlert("{\"moving_object_alert\":{\"position\":{\"latitude\":1.28,\"longitude\":"
      "103.85},\"affected_vehicle_types\":[2]}}", &a) && a.domains.size() == 1 && a.domains[0] == "UUV");
    chk("camelCase JSON too", t4::parseAlert("{\"movingObjectAlert\":{\"position\":{\"latitude\":1.28,"
      "\"longitude\":103.85},\"speedMps\":0.5,\"keepOutRadiusM\":12}}", &a) &&
      near(a.speed_mps, 0.5) && near(a.radius_m, 12.0));
    chk("not an alert", !t4::parseAlert("{\"run_start\":{\"run_id\":3}}", &a));
    chk("no position is not an alert",
      !t4::parseAlert("{\"moving_object_alert\":{\"heading_deg\":90}}", &a));
    chk("All Clear recognised", t4::isAllClear("{\"all_clear\":{}}") && !t4::isAllClear(full));
    chk("the command's seq is read (12)", t4::parseAlert(full, &a) && near(a.seq, 12.0));
    chk("no seq -> 0", t4::parseAlert(bare, &a) && a.seq == 0.0);
    chk("declaration_seq is not the seq", t4::parseAlert(
      "{\"declaration_seq\": 9, \"moving_object_alert\":{\"position\":{\"latitude\":1.28,"
      "\"longitude\":103.85}}}", &a) && a.seq == 0.0);
  }

  std::printf("\n2. folding alerts in: same object or a new one\n");
  {
    std::vector<t4::Invader> objs;
    int next = 1;
    t4::Alert a;
    a.lat = 1.0;
    a.lon = 2.0;
    a.heading_deg = 90.0;
    a.speed_mps = 1.0;
    a.domains = {"USV"};
    chk("first -> id 1", t4::ingest(objs, next, a, {0.0, 0.0}, 0.0, s) == 1);
    chk("radius none given -> the fallback 10", near(objs[0].radius_m, 10.0) && !objs[0].radius_given);
    // 10 s later it is predicted at (10, 0); a report at (11, 0) is the same object
    a.radius_m = 6.0;
    chk("re-report near its prediction -> still id 1",
      t4::ingest(objs, next, a, {11.0, 0.0}, 10.0, s) == 1 && objs.size() == 1);
    chk("... updated: reports 2, radius 6, t0 10", objs[0].reports == 2 &&
      near(objs[0].radius_m, 6.0) && near(objs[0].t0, 10.0));
    a.domains = {"UAV"};
    chk("far away -> a new object, id 2", t4::ingest(objs, next, a, {200.0, 0.0}, 10.0, s) == 2);
    chk("UAV-only does not count for the boat", !objs[1].affects_usv && !t4::counts(objs[1], s));
    chk("expire drops none at 100 s", t4::expire(objs, 100.0, s) == 0);
    chk("expire drops both after expire_s", t4::expire(objs, 10.0 + s.expire_s + 1.0, s) == 2);
  }

  std::printf("\n2b. a late or re-delivered alert does not move the object back\n");
  {
    double last = 0.0;
    chk("seq 5 applies", t4::acceptSeq(5.0, last) && near(last, 5.0));
    chk("seq 5 again (re-delivered) is dropped", !t4::acceptSeq(5.0, last));
    chk("seq 4 (late) is dropped", !t4::acceptSeq(4.0, last) && near(last, 5.0));
    chk("no seq always applies", t4::acceptSeq(0.0, last) && near(last, 5.0));
    chk("seq 6 applies", t4::acceptSeq(6.0, last) && near(last, 6.0));
    // the node's path: accept, then ingest. An east-bound object, reported at 0 m (seq 5, t 0)
    // and at 20 m (seq 6, t 20); seq 5 arriving again at t 40 must leave it at 20 m, t0 20.
    std::vector<t4::Invader> objs;
    int next = 1;
    double seq_seen = 0.0;
    t4::Alert a;
    a.lat = 1.0;
    a.lon = 2.0;
    a.heading_deg = 90.0;
    a.speed_mps = 1.0;
    auto deliver = [&](double seq, Vec2 p, double t) {
        a.seq = seq;
        if (t4::acceptSeq(a.seq, seq_seen)) {t4::ingest(objs, next, a, p, t, s);}
      };
    deliver(5.0, {0.0, 0.0}, 0.0);
    deliver(6.0, {20.0, 0.0}, 20.0);
    deliver(5.0, {0.0, 0.0}, 40.0);
    chk("one object, not moved back", objs.size() == 1 && near(objs[0].p0.x, 20.0) &&
      near(objs[0].t0, 20.0) && objs[0].reports == 2);
  }

  std::printf("\n3. dead reckoning and closest approach\n");
  {
    const t4::Invader o = obj({0.0, 0.0}, 90.0, 1.0);     // east at 1 m/s
    chk("at t=10: (10, 0)", near(t4::at(o, 10.0).x, 10.0) && near(t4::at(o, 10.0).y, 0.0));
    chk("before t0: clamped to p0", near(t4::at(o, -5.0).x, 0.0));
    const t4::Cpa c = t4::cpaFixed(o, {20.0, 5.0}, 0.0, 40.0);
    chk("passes (20, 5) at 5 m, 20 s from now", near(c.d, 5.0) && near(c.dt, 20.0));
    const t4::Cpa c2 = t4::cpaFixed(o, {60.0, 5.0}, 0.0, 40.0);
    chk("beyond the horizon: nearest at the horizon end", near(c2.dt, 40.0) &&
      near(c2.d, std::hypot(20.0, 5.0)));
    const t4::Cpa c3 = t4::cpaFixed(o, {-10.0, 0.0}, 0.0, 40.0);
    chk("behind it: nearest now", near(c3.dt, 0.0) && near(c3.d, 10.0));
    const t4::Invader still = obj({5.0, 5.0}, 0.0, 0.0);
    chk("a still object: its distance", near(t4::cpaFixed(still, {5.0, 0.0}, 3.0, 40.0).d, 5.0));
  }

  std::printf("\n4. the swept keep-out the planners avoid\n");
  {
    t4::Invader o = obj({0.0, 0.0}, 0.0, 1.0);             // north at 1 m/s, keep-out 10 + 3
    const path::Hazard h = t4::sweptHazard(o, s, 0.0);
    chk("moving: a polygon, SRC_KEEPOUT", h.kind == path::HazardKind::Polygon &&
      h.source == path::HazardSource::Keepout);
    chk("its start is inside (clearance -13)", near(path::clearance(h, {0.0, 0.0}), -13.0, 0.05));
    chk("20 s along its track is inside", path::clearance(h, {0.0, 20.0}) < -12.9);
    chk("13.5 m abeam is outside", path::clearance(h, {13.5, 10.0}) > 0.0);
    chk("12.5 m abeam is inside", path::clearance(h, {12.5, 10.0}) < 0.0);
    chk("35 m ahead (past the sweep) is outside", path::clearance(h, {0.0, 35.0}) > 0.0);
    o.speed_mps = 0.0;
    const path::Hazard c = t4::sweptHazard(o, s, 0.0);
    chk("still: a circle of 13", c.kind == path::HazardKind::Circle && near(c.r, 13.0));
    s.enabled = false;
    chk("Task 4 off: no hazards", t4::hazards({o}, s, 0.0).empty());
    s.enabled = true;
    o.affects_usv = false;
    chk("UAV-only: no hazards", t4::hazards({o}, s, 0.0).empty());
  }

  std::printf("\n5. the verdict: is it coming at the boat?\n");
  {
    // east at 1 m/s from (-30, 0); the boat at the origin: it passes over the boat in 30 s
    const std::vector<t4::Invader> objs{obj({-30.0, 0.0}, 90.0, 1.0)};
    t4::Assessment a = t4::assess(objs, s, {0.0, 0.0}, 0.0, false);
    chk("dead ahead of it, 30 s out: THREAT", a.threat && a.id == 1 && near(a.cpa_m, 0.0) &&
      near(a.cpa_s, 30.0));
    a = t4::assess(objs, s, {0.0, 14.0}, 0.0, false);
    chk("14 m off its track (keep-out 13): no threat", !a.threat && near(a.cpa_m, 14.0));
    a = t4::assess(objs, s, {0.0, 14.0}, 0.0, true);
    chk("... but an evasion in hand needs 13 + 2 to end", a.threat);
    a = t4::assess(objs, s, {80.0, 0.0}, 0.0, false);
    chk("on its track but 80 s away (horizon 40): no threat", !a.threat);
    a = t4::assess(objs, s, {-45.0, 0.0}, 0.0, false);
    chk("15 m behind it: no threat", !a.threat);
    std::vector<t4::Invader> uav = objs;
    uav[0].affects_usv = false;
    chk("UAV-only object: never a threat", !t4::assess(uav, s, {0.0, 0.0}, 0.0, false).threat);
  }

  std::printf("\n6. the escape\n");
  {
    const std::vector<t4::Invader> objs{obj({-30.0, 0.0}, 90.0, 1.0)};
    const t4::Escape e = t4::pickEscape(objs, s, {0.0, 0.0}, 0.0, {}, 0.8);
    chk("found, and safe", e.ok && e.safe);
    chk("sideways off its track (|y| >= 13)", std::fabs(e.p.y) >= 13.0);
    chk("not toward it", e.p.x > -10.0);
    chk("its margin holds over the whole drive and wait",
      t4::legMargin(objs, s, {0.0, 0.0}, e.p, 0.0) >= 0.0 && near(e.margin_m,
      t4::legMargin(objs, s, {0.0, 0.0}, e.p, 0.0)));
    // a wall of buoys north of the boat: the escape goes south
    std::vector<path::Hazard> field;
    for (double x = -20.0; x <= 20.0; x += 1.0) {
      field.push_back(path::detail::circleHazard(path::HazardSource::PlanBuoy, 0, {x, 4.0}, 0.3));
    }
    const t4::Escape south = t4::pickEscape(objs, s, {0.0, 0.0}, 0.0, field, 0.8);
    chk("buoys to the north: escape south", south.ok && south.safe && south.p.y <= -13.0);
    chk("... and its leg is clear of them", t4::legClearOfField({0.0, 0.0}, south.p, field, 0.8));
    // the course: a box the boat is in, too small to leave the track sideways northward
    const std::vector<Vec2> course{{-50.0, -50.0}, {50.0, -50.0}, {50.0, 8.0}, {-50.0, 8.0}};
    const t4::Escape boxed = t4::pickEscape(objs, s, {0.0, 0.0}, 0.0, {}, 0.8, course);
    chk("course edge 8 m north: escape south", boxed.ok && boxed.p.y < 0.0);
    // a boat docked: 1.5 m off a wall (closer than hard_m allows) may still back away
    std::vector<path::Hazard> dock{path::detail::polygonHazard(path::HazardSource::Dock, 0,
      {{-5.0, 1.5}, {5.0, 1.5}, {5.0, 3.0}, {-5.0, 3.0}})};
    chk("docked: backing away from the dock is allowed",
      t4::legClearOfField({0.0, 0.9}, {0.0, -10.0}, dock, 0.8));
    chk("docked: driving into the dock is not",
      !t4::legClearOfField({0.0, 0.9}, {0.0, 10.0}, dock, 0.8));
    // nothing safe: it still offers the best there is
    std::vector<t4::Invader> swarm;
    for (int k = 0; k < 12; ++k) {
      swarm.push_back(obj(nav::headingVec(k * 30.0) * 5.0, 0.0, 0.0, 40.0));
    }
    const t4::Escape best = t4::pickEscape(swarm, s, {0.0, 0.0}, 0.0, {}, 0.8);
    chk("surrounded: not safe, but a point all the same", best.ok && !best.safe);
  }

  std::printf("\n6b. inside a keep-out, or beside a parked or slow object\n");
  {
    // a parked object 8 m north, keep-out 13: the boat is inside it. Straight away (south), not
    // 3 m sideways; the point ends outside keep-out + hysteresis (15 m)
    const std::vector<t4::Invader> parked{obj({0.0, 8.0}, 0.0, 0.0)};
    const t4::Escape e = t4::pickEscape(parked, s, {0.0, 0.0}, 0.0, {}, 0.8);
    chk("inside the keep-out: a point is found", e.ok);
    chk("straight away from it (south), not 3 m sideways", std::fabs(e.p.x) < 1.0 && e.p.y <= -7.0);
    chk("ends outside keep-out + hysteresis", !t4::assess(parked, s, e.p, 0.0, true).threat &&
      nav::norm(e.p - Vec2{0.0, 8.0}) >= 15.0);
    const std::vector<t4::Invader> east{obj({8.0, 0.0}, 0.0, 0.0)};
    const t4::Escape ee = t4::pickEscape(east, s, {0.0, 0.0}, 0.0, {}, 0.8);
    chk("object east: away to the west", ee.ok && ee.p.x <= -7.0 && std::fabs(ee.p.y) < 1.0);
    const std::vector<t4::Invader> ese{obj(nav::headingVec(100.0) * 8.0, 0.0, 0.0)};
    const t4::Escape es = t4::pickEscape(ese, s, {0.0, 0.0}, 0.0, {}, 0.8);
    chk("object at 100 deg: the end is clear of it", es.ok &&
      !t4::assess(ese, s, es.p, 0.0, true).threat);

    // an object parked / creeping 14 m east: 1 m outside its keep-out, inside the hysteresis, so
    // a 3 m step that keeps m >= 0 but ends within 15 m would wait for ever. It must clear.
    for (double v : {0.0, 0.1}) {
      const std::vector<t4::Invader> beside{obj({14.0, 0.0}, 0.0, v)};
      const t4::Escape b = t4::pickEscape(beside, s, {0.0, 0.0}, 0.0, {}, 0.8);
      chk("beside it: found and safe", b.ok && b.safe);
      chk("... ends clear: not a threat at the point, with hysteresis",
        !t4::assess(beside, s, b.p, 0.0, true).threat && b.end_m >= s.clear_hyst_m);
      chk("... a spot 3 m north of the boat is not safe to wait at",
        !t4::evalEscape(beside, s, {0.0, 0.0}, {0.0, 3.0}, 0.0).safe);
    }
  }

  std::printf("\n7. the capsule is convex and counter-clockwise\n");
  {
    const std::vector<Vec2> poly = t4::capsule({0.0, 0.0}, {10.0, 0.0}, 3.0);
    bool ccw = true;
    for (std::size_t i = 0; i < poly.size(); ++i) {
      const Vec2 a = poly[i], b = poly[(i + 1) % poly.size()], c = poly[(i + 2) % poly.size()];
      if (nav::cross(b - a, c - b) < -1e-9) {ccw = false;}
    }
    chk("every turn is a left turn", ccw);
  }

  std::printf("\n8. the detour round the Task 1 field\n");
  {
    // two columns 3 m apart, 5 buoys each, like the 2026-11-10 Singapore field
    std::vector<path::Hazard> field;
    for (double y = 0.0; y <= 12.0; y += 3.0) {
      field.push_back(path::detail::circleHazard(path::HazardSource::Track, 0, {-1.5, y}, 0.3));
      field.push_back(path::detail::circleHazard(path::HazardSource::Track, 0, {1.5, y}, 0.3));
    }
    const std::vector<double> clears{3.0, 2.0, 0.8};
    const Vec2 a{0.0, 15.0}, b{0.0, -15.0};
    const t4::Detour d = t4::detour(a, b, field, clears);
    chk("through the field: a via point", d.route == t4::Route::Via);
    chk("... keeping 3 m", near(d.clear_m, 3.0));
    chk("... beside the field (|x| >= 4.8)", std::fabs(d.via.x) >= 4.8 - 1e-9);
    chk("... both legs keep it", t4::legKeeps(a, d.via, field, 3.0, true, false) &&
      t4::legKeeps(d.via, b, field, 3.0, false, true));
    chk("... the via point itself is open water",
      path::minClearance(field, d.via) >= 3.0 - 1e-9);
    const t4::Detour s = t4::detour({10.0, 15.0}, {10.0, -15.0}, field, clears);
    chk("well clear of the field: straight", s.route == t4::Route::Straight && near(s.clear_m, 3.0));
    // the course ends 4 m east of the axis: the only way round is west
    const std::vector<Vec2> course{{-60.0, -60.0}, {4.0, -60.0}, {4.0, 60.0}, {-60.0, 60.0}};
    const t4::Detour w = t4::detour(a, b, field, clears, course);
    chk("course edge to the east: round the west side", w.route == t4::Route::Via && w.via.x < 0.0);
    // starting 0.9 m off a buoy (the end of an EXIT orbit) still finds a way out
    const Vec2 tight{1.5, 13.2};
    const t4::Detour t = t4::detour(tight, b, field, clears);
    chk("starting 0.9 m off a buoy: a via point all the same", t.route == t4::Route::Via);
    chk("... and the first leg gets no closer to that buoy",
      t4::legKeeps(tight, t.via, {field[9]}, 0.85, false, false));
    // the goal inside a closed ring of buoys: no way in
    std::vector<path::Hazard> ring;
    for (int k = 0; k < 36; ++k) {
      ring.push_back(path::detail::circleHazard(path::HazardSource::Track, 0,
        b + nav::headingVec(k * 10.0) * 5.0, 0.3));
    }
    chk("goal walled in: none", t4::detour(a, b, ring, clears).route == t4::Route::None);
  }

  std::printf("acknowledging an alert (IncidentAck, once per object)\n");
  {
    t4::Alert usv;
    usv.seq = 4217.0;
    usv.domains = {"USV", "UAV"};
    chk("a new object for the USV: acknowledged", t4::shouldAck(true, usv));
    chk("the same object again: not", !t4::shouldAck(false, usv));
    t4::Alert air = usv;
    air.domains = {"UAV"};
    chk("an object for the UAV only: not the boat's to acknowledge", !t4::shouldAck(true, air));
    t4::Alert any = usv;
    any.domains.clear();
    chk("no domains named: every vehicle's, acknowledged", t4::shouldAck(true, any));
    t4::Alert noseq = usv;
    noseq.seq = 0.0;
    chk("no seq to quote: not", !t4::shouldAck(true, noseq));
    chk("the body quotes the command's seq", t4::incidentAckJson(4217.0) == "{\"command_seq\":4217}");
    std::vector<t4::Invader> objs;
    int next = 1;
    t4::ingest(objs, next, air, {0.0, 0.0}, 0.0, s);
    chk("ingest uses the same domain rule", !objs.empty() && !objs.front().affects_usv);
  }

  std::printf("\n%s\n", g_fails == 0 ? "ALL PASS" : "FAILED");
  return g_fails == 0 ? 0 : 1;
}
