// test_global_passage.cpp — the whole-field Task 1 planner and its follower, off the boat.
//
//     g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I include test/test_global_passage.cpp -o /tmp/t && /tmp/t
//
// Three layers:
//   1. the ray/parity primitives, by hand;
//   2. plan() on every Task 1 field we have (the sim courses and the panel layouts the
//      user built, the 3-5 m ones included): a plan exists, every red ends up to starboard
//      and every green to port, nothing is closer than hard_m, both orbits sweep a full turn
//      the right way round;
//   3. CLOSED LOOP: the plan driven by path_follower against a kinematic boat that takes
//      setpoints the way ArduRover 4.6 GUIDED does (input-shaped target at <= 1 m/s, 1 m/s^2,
//      position P 0.2 with velocity feed-forward), and the same checks made on the TRACK, not
//      on the plan.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "crusader_bt/global_passage.hpp"
#include "crusader_bt/path_follower.hpp"

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

struct Field
{
  std::string name;
  Vec2 boat;
  std::vector<Buoy> buoys;
  Vec2 entry, exitp;
};

static Beacon beacon(const std::string & s)
{
  if (s == "flash_red") {return Beacon::FlashingRed;}
  if (s == "flash_green") {return Beacon::FlashingGreen;}
  if (s == "flash_blue") {return Beacon::FlashingBlue;}
  if (s == "steady_blue") {return Beacon::SteadyBlue;}
  return Beacon::Off;
}

struct B {double x, y; const char * c;};

static Field field(const std::string & name, std::vector<B> bs, Vec2 boat = {0.0, 0.0})
{
  Field f;
  f.name = name;
  f.boat = boat;
  int id = 0;
  for (const B & b : bs) {
    Buoy u;
    u.id = id++;
    u.p = {b.x, b.y};
    u.state = beacon(b.c);
    if (u.state == Beacon::FlashingBlue) {f.entry = u.p;}
    if (u.state == Beacon::SteadyBlue) {f.exitp = u.p;}
    f.buoys.push_back(u);
  }
  return f;
}

static std::vector<Field> fields()
{
  std::vector<Field> fs;
  // The user's crowded field (panel layouts user_crowded_1002 / packed_field, 2026-10-02):
  // 4-5 m between buoys, the passage zig-zags west then back east.
  fs.push_back(field("user_crowded", {
    {6.8, 7.2, "flash_blue"}, {13.3, 14.6, "flash_red"}, {8.1, 14.8, "flash_green"},
    {8.8, 24.1, "flash_red"}, {3.2, 17.7, "flash_green"}, {7.4, 29.8, "steady_blue"},
    {10.3, 10.3, "off"}, {3.4, 24.5, "off"}, {6.2, 20.8, "flash_red"}, {9.8, 18.1, "off"}}));
  fs.push_back(field("layout_10021708", {
    {11.0, 5.8, "flash_blue"}, {27.9, 17.9, "flash_red"}, {11.0, 13.6, "flash_green"},
    {18.5, 32.4, "flash_red"}, {18.3, 20.0, "flash_green"}, {8.4, 30.9, "steady_blue"},
    {18.3, 10.4, "off"}, {31.9, 24.8, "off"}, {23.7, 27.3, "flash_red"}, {13.3, 27.3, "off"}}));
  fs.push_back(field("layout_09301555", {
    {12.0, 3.0, "flash_blue"}, {22.0, -3.0, "flash_red"}, {25.9, 3.1, "flash_green"},
    {30.5, -1.9, "flash_red"}, {30.7, 6.5, "flash_green"}, {42.0, -4.0, "flash_red"},
    {42.0, 2.0, "flash_green"}, {52.0, 0.0, "steady_blue"}, {26.8, -0.2, "off"},
    {34.1, 2.1, "off"}}));
  // courses/task1_tight.yaml: 3-5 m of water everywhere
  fs.push_back(field("task1_tight", {
    {6.0, 2.0, "flash_blue"}, {11.0, -1.8, "flash_red"}, {11.0, 1.8, "flash_green"},
    {15.5, -1.5, "flash_red"}, {15.5, 2.0, "flash_green"}, {20.0, 0.0, "flash_red"},
    {20.0, 3.8, "flash_green"}, {25.0, 1.0, "steady_blue"}, {13.5, 3.5, "off"},
    {18.0, -2.5, "off"}}));
  fs.push_back(field("task1_avoid", {
    {9.0, 3.0, "flash_blue"}, {30.0, -5.5, "flash_red"}, {30.0, 3.5, "flash_green"},
    {55.0, -2.5, "flash_red"}, {55.0, 6.5, "flash_green"}, {77.0, -1.0, "steady_blue"},
    {17.9, 0.3, "off"}, {39.5, 0.1, "off"}, {43.5, 1.1, "off"}, {66.0, 1.1, "off"}}));
  // singles on the "wrong" side of the axis: the passage must weave round them
  fs.push_back(field("task1_unpaired", {
    {10.0, 2.0, "flash_blue"}, {88.0, 0.0, "steady_blue"}, {30.0, -4.5, "flash_red"},
    {30.0, 4.5, "flash_green"}, {76.0, -4.0, "flash_green"}, {56.0, 1.2, "flash_red"},
    {66.0, -1.2, "flash_green"}, {19.0, 1.0, "off"}, {45.0, 0.2, "off"}, {79.0, 3.0, "off"}}));
  fs.push_back(field("task1_core", {
    {12.0, 3.0, "flash_blue"}, {22.0, -3.0, "flash_red"}, {22.0, 3.0, "flash_green"},
    {32.0, -1.0, "flash_red"}, {32.0, 5.0, "flash_green"}, {42.0, -4.0, "flash_red"},
    {42.0, 2.0, "flash_green"}, {52.0, 0.0, "steady_blue"}, {27.0, 8.0, "off"},
    {37.0, -9.0, "off"}}));
  fs.push_back(field("task1_blocked_exit", {
    {12.0, 3.0, "flash_blue"}, {22.0, -3.0, "flash_red"}, {22.0, 3.0, "flash_green"},
    {32.0, -1.0, "flash_red"}, {32.0, 5.0, "flash_green"}, {42.0, -4.0, "flash_red"},
    {42.0, 2.0, "flash_green"}, {62.0, 0.0, "steady_blue"}, {27.0, 8.0, "off"},
    {53.0, -0.6, "off"}}));
  fs.push_back(field("task1_entry_black", {
    {12.0, 3.0, "flash_blue"}, {22.0, -3.0, "flash_red"}, {22.0, 3.0, "flash_green"},
    {32.0, -1.0, "flash_red"}, {32.0, 5.0, "flash_green"}, {42.0, -4.0, "flash_red"},
    {42.0, 2.0, "flash_green"}, {52.0, 0.0, "steady_blue"}, {37.0, -9.0, "off"},
    {3.5, 0.9, "off"}}));
  // A HAIRPIN: out east below a wall of black buoys, round its end, back west above it.
  // No straight-line notion of "along the course" survives this; the parity does.
  {
    std::vector<B> bs{{0.0, 0.0, "flash_blue"}, {0.0, 10.0, "steady_blue"},
      {14.0, -3.0, "flash_red"}, {14.0, 1.5, "flash_green"},      // southern leg, heading east
      {14.0, 12.5, "flash_red"}, {14.0, 8.0, "flash_green"}};     // northern leg, heading west
    // the wall, y = 5, x -12 .. 22, 1.6 m apart (no gap the boat fits)
    Field f = field("hairpin", bs, {-4.0, -4.0});
    int id = static_cast<int>(f.buoys.size());
    for (double x = -12.0; x <= 22.0 + 1e-9; x += 1.6) {
      Buoy w;
      w.id = id++;
      w.p = {x, 5.0};
      w.state = Beacon::Off;
      f.buoys.push_back(w);
    }
    fs.push_back(f);
  }
  return fs;
}

static std::vector<path::Hazard> hazardsOf(const Field & f, const path::NavParams & np)
{
  return path::buildHazards(f.buoys, {}, crusader_bt::dock::DockBook{}, 1, np);
}

static gp::Params params(const path::NavParams & np)
{
  gp::Params p;
  p.hard_m = np.hard_m;
  p.orbit_r_pref_m = np.orbit_radius_m;
  p.orbit_clear_m = np.orbit_clear_m;
  return p;
}

static gp::Request request(const Field & f, const std::vector<path::Hazard> & hz)
{
  gp::Request rq;
  rq.from = Phase::Approach;
  rq.boat = f.boat;
  rq.buoys = f.buoys;
  rq.hazards = hz;
  rq.entry = f.entry;
  rq.exitp = f.exitp;
  return rq;
}

// ----------------------------------------------------------------- 1. primitives

static void testPrimitives()
{
  gp::Ray r;
  r.o = {0.0, 0.0};
  r.d = {1.0, 0.0};                 // the ray along +x from the origin
  CHECK(gp::segmentCrossesRay(r, {1.0, -1.0}, {1.0, 1.0}), "crosses ahead of the origin");
  CHECK(!gp::segmentCrossesRay(r, {-1.0, -1.0}, {-1.0, 1.0}), "behind the origin is not the ray");
  CHECK(!gp::segmentCrossesRay(r, {1.0, 1.0}, {2.0, 2.0}), "same side");
  // a polyline that touches the line and turns back crosses an even number of times
  const std::vector<gp::Ray> rays{r};
  CHECK(gp::polylineMask(rays, {{1.0, 1.0}, {1.0, 0.0}, {2.0, 1.0}}) == 0u, "touch and back");
  CHECK(gp::polylineMask(rays, {{1.0, 1.0}, {1.0, 0.0}, {2.0, -1.0}}) == 1u, "touch and through");
  CHECK(gp::polylineMask(rays, {{1.0, 1.0}, {1.0, -1.0}, {3.0, -1.0}, {3.0, 1.0}}) == 0u,
    "down and back up");

  // a red buoy on the axis: the straight line through it would be "both sides"; the ray to
  // starboard of an eastbound axis points south
  Buoy red;
  red.id = 7;
  red.p = {10.0, 0.0};
  red.state = Beacon::FlashingRed;
  const gp::Ray rr = gp::rayOf(red, {1.0, 0.0});
  CHECK(std::fabs(rr.d.x) < 1e-12 && rr.d.y < 0.0, "red ray points to starboard (south)");
  // passing NORTH of it (red to starboard) crosses nothing; passing south crosses once
  CHECK(gp::polylineMask({rr}, {{0.0, 0.0}, {10.0, 2.0}, {20.0, 0.0}}) == 0u, "north of red: even");
  CHECK(gp::polylineMask({rr}, {{0.0, 0.0}, {10.0, -2.0}, {20.0, 0.0}}) == 1u, "south of red: odd");

  // orbitPath: one full turn + extra, starting and ending where asked
  gp::Ring ring;
  ring.r = 2.0;
  ring.pts = gp::ringPoints({0.0, 0.0}, 2.0, true, 0.1);
  const int n = static_cast<int>(ring.pts.size());
  const auto op = gp::orbitPath(ring, 5, 10);
  CHECK(static_cast<int>(op.size()) == n + 11, "orbit size %zu vs %d", op.size(), n + 11);
  CHECK(nav::norm(op.front() - ring.pts[5]) < 1e-12 && nav::norm(op.back() - ring.pts[15]) < 1e-12,
    "orbit ends");
  const double sw = gp::sweptDeg({0.0, 0.0}, op);
  const double want = -(360.0 + 10.0 * 360.0 / n);
  CHECK(std::fabs(sw - want) < 1e-6, "cw sweep %.3f vs %.3f", sw, want);

  // re-tasking: the aircraft's scatter is not a change; a colour or a real move is
  std::vector<nav::PlanBuoy> f0{{0, {0.0, 0.0}, Beacon::FlashingBlue}, {1, {10.0, 2.0}, Beacon::FlashingRed},
    {2, {10.0, -2.0}, Beacon::FlashingGreen}, {3, {20.0, 0.0}, Beacon::SteadyBlue}};
  const gp::FieldSig s0 = gp::fieldSig(f0, {0.0, 0.0}, {20.0, 0.0});
  std::vector<nav::PlanBuoy> f1 = f0;
  f1[1].p = f1[1].p + Vec2{1.1, -0.9};                 // 1.4 m of scatter
  std::swap(f1[0], f1[3]);                              // and a different order
  CHECK(gp::fieldChange(s0, gp::fieldSig(f1, {0.5, 0.3}, {20.4, -0.6}), 2.0).empty(), "scatter");
  std::vector<nav::PlanBuoy> f2 = f0;
  f2[2].state = Beacon::Off;
  CHECK(gp::fieldChange(s0, gp::fieldSig(f2, {0.0, 0.0}, {20.0, 0.0}), 2.0) == "buoy 2 GREEN -> OFF",
    "recolour: '%s'", gp::fieldChange(s0, gp::fieldSig(f2, {0.0, 0.0}, {20.0, 0.0}), 2.0).c_str());
  std::vector<nav::PlanBuoy> f3 = f0;
  f3[1].p = f3[1].p + Vec2{2.5, 0.0};
  CHECK(!gp::fieldChange(s0, gp::fieldSig(f3, {0.0, 0.0}, {20.0, 0.0}), 2.0).empty(), "a real move");
  CHECK(!gp::fieldChange(s0, gp::fieldSig(f0, {0.0, 0.0}, {23.0, 0.0}), 2.0).empty(), "exit moved");

  // slice and pointAt
  const std::vector<Vec2> pl{{0.0, 0.0}, {10.0, 0.0}};
  const auto sl = gp::slice(pl, 2.0, 5.0);
  CHECK(sl.size() == 2 && std::fabs(sl[0].x - 2.0) < 1e-12 && std::fabs(sl[1].x - 5.0) < 1e-12,
    "slice");
}

// ---------------------------------------------------------------- 2. plans

static double sideOfTrack(const std::vector<Vec2> & tr, Vec2 b)
{
  double bd = 1e18, side = 0.0;
  for (std::size_t i = 1; i < tr.size(); ++i) {
    const Vec2 a = tr[i - 1], ab = tr[i] - a;
    const double l2 = nav::dot(ab, ab);
    if (l2 < 1e-12) {continue;}
    const double t = std::clamp(nav::dot(b - a, ab) / l2, 0.0, 1.0);
    const double d = nav::norm(b - (a + ab * t));
    if (d < bd) {bd = d; side = nav::cross(ab, b - a);}
  }
  return side;
}

static void checkPlan(const Field & f, const gp::Plan & pl, const std::vector<path::Hazard> & hz,
  double hard)
{
  const char * n = f.name.c_str();
  CHECK(pl.ok, "%s: %s", n, pl.why.c_str());
  if (!pl.ok) {return;}
  CHECK(!pl.relaxed, "%s: relaxed to %.2f", n, pl.hard_used);
  CHECK(pl.min_clear >= hard - 0.06, "%s: min clearance %.2f", n, pl.min_clear);
  CHECK(gp::wrongSide(pl.legs[2], pl.rays).empty(), "%s: a held buoy on the wrong side", n);
  // every red and green in the field is either held to its side or named as excluded
  for (const Buoy & b : f.buoys) {
    if (!nav::isSideConstrained(b.state)) {continue;}
    const bool held = std::any_of(pl.rays.begin(), pl.rays.end(),
        [&](const gp::Ray & r) {return r.id == b.id;});
    const bool excl = std::find(pl.excluded.begin(), pl.excluded.end(), b.id) != pl.excluded.end();
    CHECK(held != excl, "%s: buoy %d held %d excluded %d", n, b.id, held, excl);
  }
  CHECK(pl.entry_sweep_deg <= -380.0, "%s: entry sweep %.0f (must be CW, > 360)", n, pl.entry_sweep_deg);
  CHECK(pl.exit_sweep_deg >= 380.0, "%s: exit sweep %.0f (must be CCW, > 360)", n, pl.exit_sweep_deg);
  // legs join up
  CHECK(nav::norm(pl.legs[0].back() - pl.legs[1].front()) < 1e-6, "%s: approach -> orbit", n);
  CHECK(nav::norm(pl.legs[1].back() - pl.legs[2].front()) < 1e-6, "%s: orbit -> transit", n);
  CHECK(nav::norm(pl.legs[2].back() - pl.legs[3].front()) < 1e-6, "%s: transit -> exit orbit", n);
  CHECK(nav::norm(pl.legs[0].front() - f.boat) < 1e-6, "%s: starts at the boat", n);
  // nothing in the legs is closer than hard (densified, so vertex checks are enough)
  double worst = 1e9;
  for (int k = 0; k < 4; ++k) {worst = std::min(worst, gp::polylineClearance(pl.legs[k], hz));}
  CHECK(worst >= hard - 0.06, "%s: leg clearance %.2f", n, worst);
}

static void testPlans()
{
  path::NavParams np;                     // the YAML defaults: hard 0.8, orbit 6 m, clear 1.4
  for (const Field & f : fields()) {
    const auto hz = hazardsOf(f, np);
    const gp::Params p = params(np);
    const gp::Plan pl = gp::plan(request(f, hz), p);
    std::printf("  %-20s %s\n", f.name.c_str(), gp::summary(pl).c_str());
    for (const std::string & s : pl.notes) {std::printf("  %-20s   note: %s\n", "", s.c_str());}
    checkPlan(f, pl, hz, p.hard_m);
  }
  // the same fields with the panel's tight tuning (orbit 3 m, orbit_clear 1.0)
  np.orbit_radius_m = 3.0;
  np.orbit_clear_m = 1.0;
  for (const Field & f : fields()) {
    const auto hz = hazardsOf(f, np);
    const gp::Params p = params(np);
    const gp::Plan pl = gp::plan(request(f, hz), p);
    checkPlan(f, pl, hz, p.hard_m);
  }
}

static void testHairpinSides()
{
  // the hairpin's four gate buoys must all be HELD (not excluded): the parity, not the axis,
  // decides which side is which
  path::NavParams np;
  for (const Field & f : fields()) {
    if (f.name != "hairpin") {continue;}
    const auto hz = hazardsOf(f, np);
    const gp::Plan pl = gp::plan(request(f, hz), params(np));
    CHECK(pl.ok, "hairpin: %s", pl.why.c_str());
    CHECK(pl.rays.size() == 4, "hairpin: %zu buoys held", pl.rays.size());
    CHECK(pl.checkpoints.size() == 2, "hairpin: %zu checkpoints", pl.checkpoints.size());
  }
}

static void testReplanFromTransit()
{
  // Half way along the transit, the boat replans from where it is with the parity of what it
  // has driven: the new transit must still end with every side kept.
  path::NavParams np;
  const Field f = fields()[0];
  const auto hz = hazardsOf(f, np);
  const gp::Params p = params(np);
  const gp::Plan pl = gp::plan(request(f, hz), p);
  CHECK(pl.ok, "replan base: %s", pl.why.c_str());
  if (!pl.ok) {return;}
  const auto & tr = pl.legs[2];
  const std::vector<double> cum = gp::arcLengths(tr);
  for (const double frac : {0.2, 0.5, 0.8}) {
    const double s = frac * cum.back();
    gp::Request rq = request(f, hz);
    rq.from = Phase::Transit;
    rq.traj = gp::slice(tr, 0.0, s);
    rq.boat = gp::pointAt(tr, cum, s);
    rq.entry_ring_done = pl.entry_ring;
    for (const gp::Ray & r : pl.rays) {rq.prev_rays.push_back({r.id, r.red});}
    const gp::Plan p2 = gp::plan(rq, p);
    CHECK(p2.ok, "replan at %.0f%%: %s", frac * 100, p2.why.c_str());
    if (!p2.ok) {continue;}
    std::vector<Vec2> whole = rq.traj;
    whole.insert(whole.end(), p2.legs[2].begin(), p2.legs[2].end());
    CHECK(gp::wrongSide(whole, pl.rays).empty(), "replan at %.0f%%: wrong side", frac * 100);
    CHECK(p2.rays.size() == pl.rays.size(), "replan at %.0f%%: %zu vs %zu held", frac * 100,
      p2.rays.size(), pl.rays.size());
  }
}

static void testRecolourAtCheckpoint()
{
  // Disruptive: at the first gate's checkpoint the aircraft turns b3 (red, ahead) GREEN. The
  // replan from the boat, counting what it has driven, keeps the new side; nothing passed
  // already is chased.
  path::NavParams np;
  const Field f = fields()[0];                      // user_crowded
  const auto hz = hazardsOf(f, np);
  const gp::Params p = params(np);
  const gp::Plan pl = gp::plan(request(f, hz), p);
  CHECK(pl.ok && !pl.checkpoints.empty(), "recolour base: %s", pl.why.c_str());
  if (!pl.ok || pl.checkpoints.empty()) {return;}
  const auto & tr = pl.legs[2];
  const double s = pl.checkpoints[0].s;
  Field g = f;
  g.buoys[3].state = Beacon::FlashingGreen;
  gp::Request rq = request(g, hz);
  rq.from = Phase::Transit;
  rq.traj = gp::slice(tr, 0.0, s);
  rq.boat = rq.traj.back();
  rq.entry_ring_done = pl.entry_ring;
  for (const gp::Ray & r : pl.rays) {rq.prev_rays.push_back({r.id, r.red});}
  rq.cleared = {{pl.checkpoints[0].red_id, pl.checkpoints[0].green_id}};
  const gp::Plan p2 = gp::plan(rq, p);
  std::printf("  %-20s %s\n", "recolour b3 green", gp::summary(p2).c_str());
  CHECK(p2.ok, "recolour: %s", p2.why.c_str());
  if (!p2.ok) {return;}
  std::vector<Vec2> whole = rq.traj;
  whole.insert(whole.end(), p2.legs[2].begin(), p2.legs[2].end());
  CHECK(gp::wrongSide(whole, p2.rays).empty(), "recolour: a buoy on the wrong side");
  const bool b3_green_held = std::any_of(p2.rays.begin(), p2.rays.end(),
      [](const gp::Ray & r) {return r.id == 3 && !r.red;});
  CHECK(b3_green_held, "recolour: b3 must be held as GREEN");
  CHECK(sideOfTrack(whole, g.buoys[3].p) > 0.0, "recolour: b3 must end up to port");
}

static void testRelaxAndBoxedIn()
{
  // courses/task1_boxed_in: four blacks 1.3 m round the boat. The only ways out are 4 cm
  // wide even at the lowest relaxed clearance: no plan, and the reason names the boat.
  path::NavParams np;
  Field f = field("boxed_in", {
    {12.0, 3.0, "flash_blue"}, {22.0, -3.0, "flash_red"}, {22.0, 3.0, "flash_green"},
    {32.0, -1.0, "flash_red"}, {32.0, 5.0, "flash_green"}, {42.0, 0.0, "steady_blue"},
    {1.3, 0.0, "off"}, {-1.3, 0.0, "off"}, {0.0, 1.3, "off"}, {0.0, -1.3, "off"}});
  const auto hz = hazardsOf(f, np);
  const gp::Plan pl = gp::plan(request(f, hz), params(np));
  CHECK(!pl.ok && pl.why.find("boat") != std::string::npos, "boxed_in: %s", pl.why.c_str());

  // A ring of blacks round the boat, 1.6 m apart, with one 2.2 m gap toward the entry: at
  // 0.8 m nothing fits (the gap leaves 0.8 m at its very middle), relaxed it does.
  Field g = field("ring_gap", {
    {12.0, 3.0, "flash_blue"}, {22.0, -3.0, "flash_red"}, {22.0, 3.0, "flash_green"},
    {32.0, 0.0, "steady_blue"}});
  int id = static_cast<int>(g.buoys.size());
  const double R = 3.0;
  const double gap_half = std::asin(1.1 / R);            // chord 2.2 m
  const double step = 2.0 * std::asin(0.8 / R);          // chord 1.6 m
  for (double a = gap_half; a <= 2.0 * nav::kPi - gap_half + 1e-9; a += step) {
    Buoy w;
    w.id = id++;
    w.p = {R * std::cos(a), R * std::sin(a)};
    w.state = Beacon::Off;
    g.buoys.push_back(w);
  }
  {
    // close the ring exactly at the far side of the gap
    Buoy w;
    w.id = id++;
    w.p = {R * std::cos(-gap_half), R * std::sin(-gap_half)};
    w.state = Beacon::Off;
    g.buoys.push_back(w);
  }
  const auto hz2 = hazardsOf(g, np);
  const gp::Plan p2 = gp::plan(request(g, hz2), params(np));
  CHECK(p2.ok, "ring_gap: %s", p2.why.c_str());
  CHECK(p2.relaxed && p2.hard_used < 0.8, "ring_gap should need the relaxed clearance (%.2f)",
    p2.hard_used);
  std::printf("  %-20s %s\n", "ring_gap", gp::summary(p2).c_str());
  gp::Params strict = params(np);
  strict.relax1_m = strict.relax2_m = strict.hard_m;
  const gp::Plan no = gp::plan(request(g, hz2), strict);
  CHECK(!no.ok, "ring_gap strict should fail");
}

// ------------------------------------------------------------ 3. closed loop

/// ArduRover 4.6 GUIDED with GUID_OPTIONS 0, roughly: an input-shaped target runs at the
/// setpoint (<= vmax, accel <= a, decelerating to stop on it); the boat follows the target's
/// velocity plus P 0.2 on the position error, through a 0.5 s velocity lag.
struct SimBoat
{
  Vec2 p, v, t, tv;
  double vmax = 1.0, a = 1.0;
  void step(double dt, Vec2 sp)
  {
    const Vec2 e = sp - t;
    const double d = nav::norm(e);
    const double vd = std::min(vmax, std::sqrt(2.0 * a * d));
    const Vec2 want = d > 1e-6 ? e * (vd / d) : Vec2{0.0, 0.0};
    Vec2 dv = want - tv;
    const double ndv = nav::norm(dv);
    if (ndv > a * dt) {dv = dv * (a * dt / ndv);}
    tv = tv + dv;
    t = t + tv * dt;
    Vec2 vc = tv + (t - p) * 0.2;
    const double nvc = nav::norm(vc);
    if (nvc > 1.5) {vc = vc * (1.5 / nvc);}
    v = v + (vc - v) * (dt / 0.5);
    p = p + v * dt;
  }
};

struct Run
{
  bool ok = false;
  std::string why;
  double t = 0.0;
  double min_clear = 1e9;
  double entry_sweep = 0.0, exit_sweep = 0.0;
  std::vector<Vec2> track[4];
  int replans = 0;
};

/// The leaves' logic in miniature: plan once, drive each phase; at a phase start the plan is
/// re-made from the boat if the boat is off the leg's start; a blocked leg replans.
/// `truth` (optional): the field as it really is. The boat plans and follows on `f` (the
/// aircraft's report), and the clearance is measured against `truth`.
static Run fly(const Field & f, const path::NavParams & np, gp::Params p, gp::FollowParams fp,
  const Field * truth = nullptr)
{
  Run run;
  const auto hz = hazardsOf(f, np);
  const auto hz_true = truth != nullptr ? hazardsOf(*truth, np) : hz;
  SimBoat boat;
  boat.p = boat.t = f.boat;
  gp::Plan pl = gp::plan(request(f, hz), p);
  if (!pl.ok) {run.why = "no plan: " + pl.why; return run;}
  const double dt = 0.02;
  std::vector<Vec2> traj;
  std::vector<Vec2> entry_ring = pl.entry_ring;
  for (int ph = 0; ph < 4; ++ph) {
    const Phase phase = static_cast<Phase>(ph);
    if (pl.legs[ph].empty() ||
      gp::polylineClearance(gp::slice(pl.legs[ph], 0.0, 3.0), {path::detail::circleHazard(
        path::HazardSource::Track, -1, boat.p, 0.0)}) > 2.5)
    {
      gp::Request rq = request(f, hz);
      rq.from = phase;
      rq.boat = boat.p;
      rq.traj = traj;
      rq.entry_ring_done = entry_ring;
      pl = gp::plan(rq, p);
      ++run.replans;
      if (!pl.ok) {run.why = std::string("replan at ") + gp::phaseName(phase) + ": " + pl.why; return run;}
    }
    gp::FollowParams lp = fp;
    lp.pass_early_m = ph <= 1 || ph == 2 ? fp.lookahead_m + 0.5 : 0.0;
    lp.check_clear_m = std::min(fp.check_clear_m, pl.hard_used - 0.1);
    gp::Follower fol(lp);
    fol.reset(pl.legs[ph]);
    if (phase == Phase::Transit && traj.empty()) {traj.push_back(pl.legs[ph].front());}
    Vec2 sp = boat.p;
    double t_phase = 0.0;
    bool done = false;
    while (!done) {
      const gp::FollowOut o = fol.step(boat.p, hz);
      if (o.arrived) {done = true; break;}
      if (o.blocked) {run.why = std::string("blocked in ") + gp::phaseName(phase); return run;}
      if (o.send) {sp = o.setpoint;}
      for (int k = 0; k < 5; ++k) {
        boat.step(dt, sp);
        run.min_clear = std::min(run.min_clear, path::minClearance(hz_true, boat.p));
      }
      run.track[ph].push_back(boat.p);
      if (phase == Phase::Transit && nav::norm(boat.p - traj.back()) > 0.2) {traj.push_back(boat.p);}
      t_phase += 0.1;
      run.t += 0.1;
      if (t_phase > 400.0) {run.why = std::string("timeout in ") + gp::phaseName(phase); return run;}
    }
  }
  run.entry_sweep = gp::sweptDeg(pl.entry_c, run.track[1]);
  run.exit_sweep = gp::sweptDeg(pl.exit_c, run.track[3]);
  // the sides, judged on the driven transit (plus the first/last few metres of the orbits
  // either side, so a buoy passed right at the hand-over is still judged on real motion)
  std::vector<Vec2> tr = run.track[2];
  for (const gp::Ray & r : pl.rays) {
    const double side = sideOfTrack(tr, r.o);
    if ((r.red && side >= 0.0) || (!r.red && side <= 0.0)) {
      run.why = std::string(r.red ? "RED " : "GREEN ") + std::to_string(r.id) + " on the wrong side";
      return run;
    }
  }
  run.ok = true;
  return run;
}

static void testClosedLoop()
{
  for (const bool tight : {false, true}) {
    path::NavParams np;
    if (tight) {
      np.orbit_radius_m = 3.0;
      np.orbit_clear_m = 1.0;
    }
    for (const Field & f : fields()) {
      gp::FollowParams fp;
      const Run r = fly(f, np, params(np), fp);
      std::printf("  %-20s %s  %s  t %.0f s, hull clearance %.2f m, entry %.0f deg, exit %.0f deg, "
        "replans %d%s%s\n", f.name.c_str(), tight ? "orbit 3" : "orbit 6", r.ok ? "PASS" : "FAIL",
        r.t, r.min_clear, r.entry_sweep, r.exit_sweep, r.replans, r.ok ? "" : "  ", r.why.c_str());
      CHECK(r.ok, "%s closed loop: %s", f.name.c_str(), r.why.c_str());
      if (!r.ok) {continue;}
      CHECK(r.min_clear >= 0.6, "%s: the boat came within %.2f m of a buoy", f.name.c_str(), r.min_clear);
      CHECK(r.entry_sweep <= -360.0, "%s: entry orbit swept %.0f deg", f.name.c_str(), r.entry_sweep);
      CHECK(r.exit_sweep >= 360.0, "%s: exit orbit swept %.0f deg", f.name.c_str(), r.exit_sweep);
    }
  }
}

/// The aircraft's fix error (< 1 m, R 0.8 in the panel): every buoy displaced up to 0.8 m in
/// a random direction. The boat plans on that; the truth decides how close it really came.
/// No fusion here (the camera would pull the positions back toward the truth as the boat
/// closes in), so this is the WORST case the plan alone must survive.
static void testNoisyClosedLoop()
{
  path::NavParams np;
  np.orbit_radius_m = 3.0;
  np.orbit_clear_m = 1.0;
  std::uint32_t seed = 12345u;
  const auto rnd = [&seed]() {
      seed = seed * 1664525u + 1013904223u;
      return (seed >> 8) / 16777216.0;
    };
  double worst = 1e9;
  std::string worst_name;
  for (const Field & truth : fields()) {
    if (truth.name == "hairpin") {continue;}
    for (int k = 0; k < 4; ++k) {
      Field seen = truth;
      for (Buoy & b : seen.buoys) {
        const double a = 2.0 * nav::kPi * rnd(), r = 0.8 * std::sqrt(rnd());
        b.p = b.p + Vec2{std::cos(a), std::sin(a)} * r;
        if (b.state == Beacon::FlashingBlue) {seen.entry = b.p;}
        if (b.state == Beacon::SteadyBlue) {seen.exitp = b.p;}
      }
      const Run r = fly(seen, np, params(np), gp::FollowParams{}, &truth);
      CHECK(r.ok, "%s noisy %d: %s", truth.name.c_str(), k, r.why.c_str());
      if (r.ok && r.min_clear < worst) {worst = r.min_clear; worst_name = truth.name;}
    }
  }
  std::printf("  noisy fields (0.8 m UAV error, no fusion): worst TRUE clearance %.2f m (%s)\n",
    worst, worst_name.c_str());
  CHECK(worst >= 0.3, "noisy closed loop came within %.2f m of a buoy (%s)", worst, worst_name.c_str());
}

int main()
{
  std::printf("primitives\n");
  testPrimitives();
  std::printf("plans (orbit 6 m, then 3 m)\n");
  testPlans();
  testHairpinSides();
  testReplanFromTransit();
  testRecolourAtCheckpoint();
  testRelaxAndBoxedIn();
  std::printf("closed loop\n");
  testClosedLoop();
  testNoisyClosedLoop();
  std::printf("%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
