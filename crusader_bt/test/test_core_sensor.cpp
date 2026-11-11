// test_core_sensor.cpp — Core tier's sensor-triggered replan check (gp::sensorChangeMatters,
// gp::wrongSideNow) and the evidence-based ENTRY/EXIT pick (nav::bestBeacon), off the boat.
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include test/test_core_sensor.cpp -o /tmp/t && /tmp/t
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "crusader_bt/global_passage.hpp"

using crusader_bt::gp::Phase;
using crusader_bt::nav::Beacon;
using crusader_bt::nav::Buoy;
using crusader_bt::nav::Vec2;
namespace gp = crusader_bt::gp;
namespace nav = crusader_bt::nav;
namespace path = crusader_bt::path;

static int g_fail = 0, g_pass = 0;

#define CHECK(cond, ...) do { \
    if (cond) {++g_pass;} else { \
      ++g_fail; std::printf("FAIL %s:%d: %s  ", __FILE__, __LINE__, #cond); \
      std::printf(__VA_ARGS__); std::printf("\n"); \
    } \
} while (0)

static Buoy buoy(int id, double x, double y, Beacon s, int hits = 0, double conf = 0.0)
{
  Buoy b;
  b.id = id;
  b.p = {x, y};
  b.state = s;
  b.hits = hits;
  b.conf = conf;
  return b;
}

/// ENTRY (0,0) -> EXIT (60,0), gates at x = 20 and 40, red to starboard (-y) of the axis.
static std::vector<Buoy> straightField()
{
  return {buoy(0, 0, 0, Beacon::FlashingBlue), buoy(1, 60, 0, Beacon::SteadyBlue),
    buoy(2, 20, -2.5, Beacon::FlashingRed), buoy(3, 20, 2.5, Beacon::FlashingGreen),
    buoy(4, 40, -2.5, Beacon::FlashingRed), buoy(5, 40, 2.5, Beacon::FlashingGreen)};
}

static gp::Plan planOf(const std::vector<Buoy> & bs, Vec2 boat)
{
  path::NavParams np;
  gp::Request rq;
  rq.from = Phase::Approach;
  rq.boat = boat;
  rq.buoys = bs;
  rq.hazards = path::buildHazards(bs, {}, crusader_bt::dock::DockBook{}, 1, np);
  rq.entry = bs[0].p;
  rq.exitp = bs[1].p;
  gp::Params p;
  p.hard_m = np.hard_m;
  p.orbit_r_pref_m = np.orbit_radius_m;
  p.orbit_clear_m = np.orbit_clear_m;
  return gp::plan(rq, p);
}

static std::string matters(const gp::Plan & pl, Phase ph, const std::vector<Buoy> & now,
  Vec2 entry, Vec2 exitp, const std::vector<Vec2> & judged)
{
  return gp::sensorChangeMatters(pl, ph, now, entry, exitp, pl.entry_ring, judged, 2.0, 1.5);
}

static void testSensorCheck()
{
  const std::vector<Buoy> bs = straightField();
  const gp::Plan pl = planOf(bs, {-8.0, -8.0});
  CHECK(pl.ok, "%s", pl.why.c_str());
  const std::vector<Vec2> & tr = pl.legs[static_cast<int>(Phase::Transit)];
  CHECK(tr.size() > 10, "transit has %zu points", tr.size());

  // nothing changed: nothing matters
  CHECK(matters(pl, Phase::Approach, bs, bs[0].p, bs[1].p, tr).empty(), "unchanged");
  // jitter of 0.3-0.4 m changes nothing
  std::vector<Buoy> jit = bs;
  jit[2].p.y += 0.4;
  jit[4].p.x += 0.3;
  CHECK(matters(pl, Phase::Approach, jit, bs[0].p, bs[1].p, tr).empty(), "jitter");
  // the field mirrored (every red on the port hand of the transit): flagged, with its name
  std::vector<Buoy> mir = bs;
  for (Buoy & b : mir) {b.p.y = -b.p.y;}
  const std::string why = matters(pl, Phase::Approach, mir, bs[0].p, bs[1].p, tr);
  CHECK(why.find("wrong side") != std::string::npos, "mirrored: '%s'", why.c_str());
  // a colour swap of one buoy of a gate (red <-> green) is flagged too
  std::vector<Buoy> col = bs;
  col[2].state = Beacon::FlashingGreen;
  CHECK(!matters(pl, Phase::Approach, col, bs[0].p, bs[1].p, tr).empty(), "recolour");
  CHECK(!matters(pl, Phase::EntryOrbit, col, bs[0].p, bs[1].p, tr).empty(), "recolour in orbit");

  // a red BESIDE the entry orbit is excluded by the planner: wherever the transit passes it,
  // it must not be flagged
  std::vector<Buoy> beside = bs;
  beside.push_back(buoy(6, 0.0, 5.5, Beacon::FlashingRed));
  const gp::Plan pb = planOf(beside, {-8.0, -8.0});
  CHECK(pb.ok, "%s", pb.why.c_str());
  CHECK(std::find(pb.excluded.begin(), pb.excluded.end(), 6) != pb.excluded.end(),
    "planner should have excluded buoy 6");
  const std::vector<Vec2> & tb = pb.legs[static_cast<int>(Phase::Transit)];
  const std::string wb = matters(pb, Phase::Approach, beside, beside[0].p, beside[1].p, tb);
  CHECK(wb.empty(), "buoy beside the orbit flagged: '%s'", wb.c_str());

  // a buoy the plan dropped because it was recoloured after being passed stays dropped
  gp::Plan pn = pl;
  pn.excluded.push_back(2);
  CHECK(matters(pn, Phase::Approach, col, bs[0].p, bs[1].p, tr).find("RED 2") == std::string::npos &&
    matters(pn, Phase::Approach, col, bs[0].p, bs[1].p, tr).find("GREEN 2") == std::string::npos,
    "not-chased buoy re-flagged");

  // ENTRY / EXIT moved: only before their orbit has started
  CHECK(matters(pl, Phase::Approach, bs, bs[0].p + Vec2{3.0, 0.0}, bs[1].p, tr).find("ENTRY") !=
    std::string::npos, "entry moved in approach");
  CHECK(matters(pl, Phase::EntryOrbit, bs, bs[0].p + Vec2{3.0, 0.0}, bs[1].p, tr).empty(),
    "entry moved once orbiting");
  CHECK(matters(pl, Phase::Transit, bs, bs[0].p, bs[1].p + Vec2{0.0, 3.0}, tr).find("EXIT") !=
    std::string::npos, "exit moved in transit");
  CHECK(matters(pl, Phase::ExitOrbit, bs, bs[0].p, bs[1].p + Vec2{0.0, 3.0}, {}).empty(),
    "exit moved once orbiting");
  CHECK(matters(pl, Phase::Approach, bs, bs[0].p + Vec2{1.0, 0.0}, bs[1].p, tr).empty(),
    "entry scatter under the threshold");

  // the transit judged as driven: a track south of red 2 passes it on its port side -> flagged
  std::vector<Vec2> driven;
  for (double x = -5.0; x <= 30.0; x += 0.5) {driven.push_back({x, -4.5});}
  CHECK(!matters(pl, Phase::Transit, bs, bs[0].p, bs[1].p, driven).empty(), "driven wrong side");
}

/// The 2026-11-10 Singapore field (crusader_sim/courses/task1_sg.yaml) as dealt that day:
/// ENTRY and EXIT first (planOf's convention), reds on the east column, greens on the west.
static std::vector<Buoy> singaporeField()
{
  const Beacon R = Beacon::FlashingRed, G = Beacon::FlashingGreen;
  return {buoy(1, -4.52, 9.02, Beacon::FlashingBlue), buoy(11, -5.22, 21.91, Beacon::SteadyBlue),
    buoy(2, -8.62, 9.69, G), buoy(3, -4.35, 12.40, R), buoy(4, -7.30, 12.46, G),
    buoy(5, -7.36, 14.80, G), buoy(6, -5.10, 14.90, R), buoy(7, -0.53, 15.26, Beacon::Off),
    buoy(8, -3.20, 16.55, R), buoy(9, -5.78, 17.91, G), buoy(10, -4.06, 18.62, R)};
}

/// Run 2 with the colours wiped at mission start: the first plan is made with every buoy
/// Unknown (ENTRY/EXIT from memory) and goes up the west side of the field. When the camera reads
/// the deal again, the greens that transit would leave to starboard must ask for a replan, and a
/// plan made with the colours must fit (at the 0.5 m relax floor) and ask for nothing more.
static void testSingaporeRun2()
{
  const std::vector<Buoy> dealt = singaporeField();
  std::vector<Buoy> wiped = dealt;
  for (Buoy & b : wiped) {b.state = Beacon::Unknown;}
  const Vec2 boat{-1.0, -2.0};
  const int transit = static_cast<int>(Phase::Transit);
  const gp::Plan blind = planOf(wiped, boat);
  CHECK(blind.ok, "a plan with every colour wiped");
  const std::string why = matters(blind, Phase::Approach, dealt, dealt[0].p, dealt[1].p,
      blind.legs[transit]);
  CHECK(why.find("wrong side") != std::string::npos, "the colours read again: '%s'", why.c_str());
  const gp::Plan seen = planOf(dealt, boat);
  CHECK(seen.ok, "a plan with the colours fits the field");
  const std::string again = matters(seen, Phase::Approach, dealt, dealt[0].p, dealt[1].p,
      seen.legs[transit]);
  CHECK(again.empty(), "and asks for nothing more: '%s'", again.c_str());
}

static void testBestBeacon()
{
  using nav::bestBeacon;
  const Beacon E = Beacon::FlashingBlue;
  std::vector<Buoy> v{buoy(1, 0, 0, E, 5), buoy(2, 3, 0, E, 12),
    buoy(3, 6, 0, Beacon::FlashingRed, 99)};
  CHECK(bestBeacon(v, E)->id == 2, "most hits wins");
  CHECK(bestBeacon(v, Beacon::SteadyBlue) == nullptr, "none -> nullptr");
  CHECK(bestBeacon({}, E) == nullptr, "empty -> nullptr");
  CHECK(bestBeacon(v, E, 1)->id == 1, "keeps current vs a modest rival (12 < 5 + 10)");
  v[1].hits = 15;      // >= 2 x 5 and >= 5 + 10
  CHECK(bestBeacon(v, E, 1)->id == 2, "clear rival takes over");
  v[1].hits = 14;      // ratio met, margin not
  CHECK(bestBeacon(v, E, 1)->id == 1, "needs both ratio and margin");
  v[0].hits = 40;
  v[1].hits = 60;      // margin met, ratio not
  CHECK(bestBeacon(v, E, 1)->id == 1, "ratio not met");
  CHECK(bestBeacon(v, E, 99)->id == 2, "stale current ignored");
  v[0].state = Beacon::Off;
  CHECK(bestBeacon(v, E, 1)->id == 2, "current lost its label");
  std::vector<Buoy> t{buoy(1, 0, 0, E, 5, 0.4), buoy(2, 1, 0, E, 5, 0.9)};
  CHECK(bestBeacon(t, E)->id == 2, "tie broken by confidence");
  CHECK(bestBeacon(t, E, -1, 2)->id == 1, "excluded id skipped");
  t[1].p.x = std::nan("");
  CHECK(bestBeacon(t, E)->id == 1, "nan position skipped");
}

int main()
{
  testSensorCheck();
  testSingaporeRun2();
  testBestBeacon();
  std::printf("%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
