// test_dock_math — prove the Task 3 arithmetic with no ROS, no boat, no colcon.
//
//     g++ -std=c++17 -O2 -I include -o /tmp/t test/test_dock_math.cpp && /tmp/t
//
// Two mistakes in this task look completely plausible from the outside, and
// both are pinned down here against hand-worked cases:
//
//   * MIRRORED BAY NUMBERS. "Bay numbers referenced left-to-right when facing
//     the bays." Get the facing direction backwards and the boat still docks
//     in the right bay - it is the REPORT that says 3 instead of 1.
//   * RENUMBERED COLOURS. The CV numbers RED 2 and GREEN 3 (OFF is 1);
//     RoboCommand numbers RED 1 and GREEN 2. Cast one into the other and a RED
//     light is reported as GREEN.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "crusader_bt/dock_math.hpp"

using namespace crusader_bt;         // NOLINT(build/namespaces) — a test
using namespace crusader_bt::dock;   // NOLINT(build/namespaces)
using nav::Vec2;

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

// A bay sighting built from WORLD truth: what a perfect camera at `boat` would
// report for a face centred at `face` whose outward normal is `out`.
//
// It inverts the transform under test, so on its own it would let a shared sign
// error cancel. That is why the "placing" section below checks the forward
// transform against numbers worked out by hand first; everything after that
// may lean on it.
static BaySighting see(
  Vec2 boat, double heading_deg, const Mount & m, Vec2 face, Vec2 out,
  Colour ind = Colour::Unknown)
{
  const Vec2 cam = bodyToWorld(boat, heading_deg, m.x, m.y);
  const Vec2 f = nav::headingVec(heading_deg - m.yaw_deg);   // boresight
  const Vec2 l = nav::portOf(f);
  const Vec2 d = face - cam;
  BaySighting s;
  s.bearing_deg = std::atan2(nav::dot(d, l), nav::dot(d, f)) / nav::kDeg;
  // Plane through `face` with normal `out`, in the camera frame.
  const double nx = nav::dot(out, f), ny = nav::dot(out, l);
  const double off = -(nx * nav::dot(d, f) + ny * nav::dot(d, l));
  s.has_normal = true;
  s.nx = nx;
  s.ny = ny;
  s.d = off;
  s.range_m = sightingRange(true, nx, ny, off, s.bearing_deg, kNaN);
  s.indicator_present = ind != Colour::Unknown;
  s.indicator = ind;
  s.indicator_conf = 0.9;
  return s;
}

int main()
{
  // --------------------------------------------------------------- colours
  std::printf("colours\n");
  chk("CV 2 is RED", colourFromCv(2) == Colour::Red);
  chk("CV 3 is GREEN", colourFromCv(3) == Colour::Green);
  chk("CV 1 is OFF", colourFromCv(1) == Colour::Off);
  chk("an unknown CV value is Unknown, not a cast", colourFromCv(9) == Colour::Unknown);
  chk("CV RED goes on the wire as RoboCommand RED (1)",
    wireColour(colourFromCv(2)) == kWireRed && kWireRed == 1);
  chk("... and NOT as 2, which is RoboCommand GREEN", wireColour(colourFromCv(2)) != 2);
  chk("CV GREEN goes on the wire as 2", wireColour(colourFromCv(3)) == 2);
  chk("CV BLUE goes on the wire as 3", wireColour(colourFromCv(4)) == 3);
  chk("OFF never goes on the wire", wireColour(Colour::Off) == kWireUnknown);
  chk("names are case-blind", colourFromName("RED") == Colour::Red &&
    colourFromName("Blue") == Colour::Blue);
  chk("an unknown name is Unknown", colourFromName("purple") == Colour::Unknown &&
    colourFromName("") == Colour::Unknown);
  chk("proto names", std::string(wireColourName(kWireAny)) == "COLOR_ANY" &&
    std::string(wireColourName(kWireBlue)) == "COLOR_BLUE");

  // --------------------------------------------------------------- placing
  std::printf("placing (hand-worked)\n");
  {
    const Mount m;   // 0.37 forward, centreline, square
    // Facing NORTH, the camera is 0.37 m north of the origin.
    const Vec2 cam = bodyToWorld({0, 0}, 0.0, m.x, m.y);
    chk_near("camera is 0.37 m north when facing north", cam.y, 0.37, 1e-9);
    // Dead ahead at 8 m: north.
    Vec2 d = camDirToWorld(0.0, 0.0, 1.0, 0.0);
    chk_near("bearing 0 facing north points north (x)", d.x, 0.0, 1e-9);
    chk_near("bearing 0 facing north points north (y)", d.y, 1.0, 1e-9);
    // 30 deg to PORT while facing north is WEST of north: x < 0.
    const double b = 30.0 * nav::kDeg;
    d = camDirToWorld(0.0, 0.0, std::cos(b), std::sin(b));
    chk_near("+30 (left) facing north is 30 deg west of north (x)", d.x, -0.5, 1e-9);
    chk_near("+30 (left) facing north is 30 deg west of north (y)", d.y, std::sqrt(3) / 2, 1e-9);
    // Facing EAST, 90 deg to port is NORTH.
    d = camDirToWorld(90.0, 0.0, 0.0, 1.0);
    chk_near("left of an east-facing boat is north (y)", d.y, 1.0, 1e-9);
    // A camera yawed 90 deg to PORT on a north-facing boat looks WEST.
    d = camDirToWorld(0.0, 90.0, 1.0, 0.0);
    chk_near("camera yawed to port looks west", d.x, -1.0, 1e-9);
  }
  std::printf("a pitched camera\n");
  {
    // The easy ray: camera forward pitched DOWN 10 deg levels to cos(10) forward.
    const Vec2 h = levelled(1.0, 0.0, 0.0, 10.0);
    chk_near("forward, pitched down 10: cos10 of it is horizontal", h.x,
      std::cos(10 * nav::kDeg), 1e-12);
    // Camera-frame UP on a camera pitched down 10 deg leans FORWARD in the body.
    chk("camera-frame up leans forward when pitched down", levelled(0.0, 0.0, 1.0, 10.0).x > 0.0);

    // A face 5 m north, 2 m to port, 0.4 m above the camera; the camera
    // pitched UP 20 deg (-20). Build the sighting as the CV would - bearing and
    // plane in the tilted camera frame - and it must still land on the face.
    Mount pm;
    pm.pitch_deg = -20.0;
    const Vec2 boat{0, 0};
    const Vec2 cam = bodyToWorld(boat, 0.0, pm.x, pm.y);
    const double bx = 5.0 - cam.y, by = 2.0, bz = 0.4;       // body frame, face centre
    const double p = pm.pitch_deg * nav::kDeg;
    const double cx = bx * std::cos(p) - bz * std::sin(p);    // body -> camera (tilted)
    const double cz = bx * std::sin(p) + bz * std::cos(p);
    const double cy = by;
    // Plane x_body = bx, normal toward the camera (-1, 0, 0) in the body.
    const double nx = -std::cos(p), nz = -std::sin(p), ny = 0.0;
    const double d = -(nx * cx + ny * cy + nz * cz);
    BaySighting s;
    s.bearing_deg = std::atan2(cy, cx) / nav::kDeg;
    s.range_m = sightingRange(true, nx, ny, d, s.bearing_deg, kNaN);
    s.has_normal = true;
    s.nx = nx;
    s.ny = ny;
    s.nz = nz;
    s.d = d;
    DockBook b;
    b.prm.face_dz = 0.4;                // the course says where the face centre is
    Frame f;
    f.bays = {s};
    b.ingest(f, boat, 0.0, pm);
    chk("a pitched sighting places a track", b.tracks.size() == 1);
    chk_near("... on the face plane, 5 m north", b.tracks[0].p.y, 5.0, 0.02);
    chk_near("... and at the face, 2 m WEST (port)", b.tracks[0].p.x, -2.0, 0.08);
    chk_near("its normal levels to due south", b.tracks[0].outward().y, -1.0, 1e-6);
    // Ignore the pitch and the range comes out long by 1/cos(20): 6 %.
    DockBook wrong;
    wrong.ingest(f, boat, 0.0, Mount{});
    chk("ignoring the pitch puts it >0.2 m too far", wrong.tracks[0].p.y > 5.2);
    // Know the pitch but get the face's height 0.4 m wrong, and the point
    // slides along the plane: 6 cm sideways here (tan(azimuth) * sin(pitch)
    // per metre of height). Forgiving - the prior need not be precise - but
    // not zero, which is why it is a parameter and not a constant.
    DockBook flat;
    flat.prm.face_dz = 0.0;
    flat.ingest(f, boat, 0.0, pm);
    const double slide = std::abs(flat.tracks[0].p.x + 2.0);
    chk("a 0.4 m error in face height slides it 4-10 cm sideways", slide > 0.04 && slide < 0.10);
    // And with a LEVEL camera the face height does not matter at all.
    const Mount m0;
    DockBook lv1, lv2;
    lv2.prm.face_dz = 3.0;
    Frame lf;
    lf.bays = {see({0, 8}, 0.0, m0, {-2, 20}, {0, -1})};
    lv1.ingest(lf, {0, 8}, 0.0, m0);
    lv2.ingest(lf, {0, 8}, 0.0, m0);
    chk_near("level camera: face height irrelevant", lv1.tracks[0].p.x, lv2.tracks[0].p.x, 1e-9);
  }

  std::printf("range to the face plane\n");
  {
    // Face 8 m straight ahead, normal toward the camera (-x): -x + 8 = 0.
    chk_near("straight ahead", rangeToPlane(-1.0, 0.0, 8.0, 0.0), 8.0, 1e-9);
    chk_near("20 deg off the normal is 8/cos20",
      rangeToPlane(-1.0, 0.0, 8.0, 20.0), 8.0 / std::cos(20.0 * nav::kDeg), 1e-9);
    chk("a plane behind the camera is no range",
      std::isnan(rangeToPlane(-1.0, 0.0, -8.0, 0.0)));
    chk("edge-on is no range", std::isnan(rangeToPlane(-1.0, 0.0, 8.0, 90.0)));
    chk("NaN plane is no range", std::isnan(rangeToPlane(kNaN, 0.0, 8.0, 0.0)));
    chk_near("no plane: the window-size range",
      sightingRange(false, 0, 0, 0, 0.0, 5.0), 5.0, 1e-9);
    chk_near("a degenerate plane falls back to size",
      sightingRange(true, -1.0, 0.0, 8.0, 90.0, 5.0), 5.0, 1e-9);
    chk("neither: NaN", std::isnan(sightingRange(false, 0, 0, 0, 0.0, kNaN)));
  }

  // --------------------------------------------------------------- the book
  //
  // THE COURSE USED FROM HERE ON: three faces on the line y = 20, facing SOUTH
  // (out = (0, -1)), 2 m apart - the real dock's pitch: 1.5 m slips between
  // 0.5 m fingers (RobotX 2026 Docking Bay Structure build guide). A boat
  // south of them looking north sees bay 1 on its LEFT, which is WEST: x = -2.
  const Vec2 W{-2, 20}, C{0, 20}, E{2, 20};
  const Vec2 south{0, -1};
  const Mount m;
  std::printf("the book\n");
  {
    DockBook b;
    Frame f;
    f.bays = {see({0, 8}, 0.0, m, W, south, Colour::Red),
      see({0, 8}, 0.0, m, C, south, Colour::Green),
      see({0, 8}, 0.0, m, E, south, Colour::Red)};
    const auto ids = b.ingest(f, {0, 8}, 0.0, m);
    chk("three sightings, three tracks", b.tracks.size() == 3);
    chk_near("the west face lands at x=-2", b.find(ids[0])->p.x, -2.0, 1e-6);
    chk_near("... and y=20", b.find(ids[0])->p.y, 20.0, 1e-6);
    chk_near("outward normal is south", b.find(ids[1])->outward().y, -1.0, 1e-6);

    // Later, from further east and turned: the camera sees only C and E, and a
    // CV bay_index would now call them 0 and 1. They must still land on the
    // tracks that are C and E.
    Frame g;
    g.bays = {see({1, 14}, -15.0, m, C, south, Colour::Green),
      see({1, 14}, -15.0, m, E, south, Colour::Red)};
    const auto ids2 = b.ingest(g, {1, 14}, -15.0, m);
    chk("a partial view adds no tracks", b.tracks.size() == 3);
    chk("C from a new place joins C's track", ids2[0] == ids[1]);
    chk("E from a new place joins E's track", ids2[1] == ids[2]);
    chk("so C has two sightings", b.find(ids[1])->n == 2);
    chk("and W still has one", b.find(ids[0])->n == 1);

    // A gate wider than the spacing must still not merge two bays seen
    // together: one sighting per track per frame.
    DockBook wide;
    wide.prm.gate_m = 5.0;
    wide.ingest(f, {0, 8}, 0.0, m);
    wide.ingest(f, {0, 8}, 0.0, m);
    chk("a wide gate still keeps three bays three", wide.tracks.size() == 3);
    bool each_two = true;
    for (const auto & t : wide.tracks) {each_two = each_two && t.n == 2;}
    chk("and each got exactly its own second sighting", each_two);

    // No heading, no placing: a bearing off an unknown datum is a guess.
    DockBook nh;
    nh.ingest(f, {0, 8}, kNaN, m);
    chk("NaN heading places nothing", nh.tracks.empty());

    // A truncated face cannot START a track (its centre is biased) ...
    DockBook tr;
    Frame t1;
    t1.bays = {see({0, 8}, 0.0, m, W, south, Colour::Red)};
    t1.bays[0].truncated = true;
    tr.ingest(t1, {0, 8}, 0.0, m);
    chk("a truncated face starts no track", tr.tracks.empty());
    // ... but it votes, and does not move, a track that exists.
    Frame t2 = t1;
    t2.bays[0].truncated = false;
    tr.ingest(t2, {0, 8}, 0.0, m);
    Frame t3 = t1;
    t3.bays[0].range_m += 0.5;          // biased, inside the 0.7 m gate
    tr.ingest(t3, {0, 8}, 0.0, m);
    chk("a truncated face votes", tr.tracks.size() == 1 && tr.tracks[0].red == 2);
    chk_near("but does not move the track", tr.tracks[0].p.y, 20.0, 1e-6);

    // Too far: nothing.
    DockBook far;
    Frame ff;
    ff.bays = {see({0, -20}, 0.0, m, C, south, Colour::Green)};   // 40 m away
    far.ingest(ff, {0, -20}, 0.0, m);
    chk("beyond max_range_m places nothing", far.tracks.empty());
  }

  // --------------------------------------------------------------- layout
  std::printf("layout: numbering left to right FACING the bays\n");
  {
    DockBook b;
    for (int i = 0; i < 6; ++i) {
      Frame f;
      f.bays = {see({0, 8}, 0.0, m, W, south), see({0, 8}, 0.0, m, C, south),
        see({0, 8}, 0.0, m, E, south)};
      b.ingest(f, {0, 8}, 0.0, m);
    }
    const DockLayout L = layout(b, 5);
    chk("three confirmed bays make a layout", L.ok);
    chk_near("the dock faces south", L.out.y, -1.0, 1e-6);
    chk_near("left-to-right facing north is +east", L.right.x, 1.0, 1e-6);
    int w = -1, e = -1;
    for (const auto & t : b.tracks) {
      if (t.p.x < -1) {w = t.id;}
      if (t.p.x > 1) {e = t.id;}
    }
    chk("facing NORTH, the WEST bay is bay 1", L.numberOf(w) == 1);
    chk("facing NORTH, the EAST bay is bay 3", L.numberOf(e) == 3);
    chk_near("the bay pitch is MEASURED: 2 m", bayPitch(b, L), 2.0, 1e-6);
    chk("no layout, no pitch", std::isnan(bayPitch(b, DockLayout{})));

    // The mirror: the same three faces turned to face NORTH, seen by a boat to
    // the north looking south. Now bay 1 - on its left - is in the EAST.
    DockBook n;
    const Vec2 north{0, 1};
    for (int i = 0; i < 6; ++i) {
      Frame f;
      f.bays = {see({0, 32}, 180.0, m, W, north), see({0, 32}, 180.0, m, C, north),
        see({0, 32}, 180.0, m, E, north)};
      n.ingest(f, {0, 32}, 180.0, m);
    }
    const DockLayout N = layout(n, 5);
    int nw = -1, ne = -1;
    for (const auto & t : n.tracks) {
      if (t.p.x < -1) {nw = t.id;}
      if (t.p.x > 1) {ne = t.id;}
    }
    chk("facing SOUTH, the EAST bay is bay 1", N.ok && N.numberOf(ne) == 1);
    chk("facing SOUTH, the WEST bay is bay 3", N.ok && N.numberOf(nw) == 3);

    // Two of three: refuse to number. Bay 2 of three could be either one.
    DockBook two;
    for (int i = 0; i < 6; ++i) {
      Frame f;
      f.bays = {see({0, 8}, 0.0, m, W, south), see({0, 8}, 0.0, m, C, south)};
      two.ingest(f, {0, 8}, 0.0, m);
    }
    const DockLayout T = layout(two, 5);
    chk("two bays seen: no numbering", !T.ok);
    chk("and it says why", T.why.find("2 of 3") != std::string::npos);

    // A false positive seen twice must not displace a real bay seen six times.
    DockBook fp = b;
    Frame junk;
    junk.bays = {see({0, 8}, 0.0, m, {10, 18}, south)};
    fp.ingest(junk, {0, 8}, 0.0, m);
    fp.ingest(junk, {0, 8}, 0.0, m);
    const DockLayout F = layout(fp, 5);
    chk("a weak fourth track is left out", F.ok && F.ids.size() == 3 &&
      F.numberOf(fp.tracks.back().id) == 0);

    // Two tracks 1 m apart are not two bays: layout refuses them (built by
    // hand here, because ingest would merge them first - see below).
    DockBook dup = b;
    dup.tracks[0].n = 10;                 // the pair is the best-seen two, so the
    BayTrack twin = dup.tracks[0];        // layout's top three must include both
    twin.id = 99;
    twin.p = twin.p + Vec2{1.0, 0.0};
    dup.tracks.push_back(twin);
    chk("two tracks 1 m apart are refused", !layout(dup, 5).ok);
  }

  std::printf("merging one bay seen twice\n");
  {
    // A biased early sighting lands 1.0 m from the bay: outside the 0.7 m
    // gate, so it starts its own track - and inside merge_m (1.2 m), so the
    // book folds it straight back in. Without this the layout refused to number the dock
    // for 70 s in the sim.
    DockBook b;
    Frame good;
    good.bays = {see({0, 8}, 0.0, m, W, south, Colour::Red)};
    for (int i = 0; i < 3; ++i) {b.ingest(good, {0, 8}, 0.0, m);}
    const int first = b.tracks[0].id;
    Frame biased;
    biased.bays = {see({0, 8}, 0.0, m, {-2, 21.0}, south, Colour::Red)};
    const auto ids = b.ingest(biased, {0, 8}, 0.0, m);
    chk("the biased sighting did start a track of its own", ids[0] != first);
    chk("and it was merged: one track", b.tracks.size() == 1);
    chk("its id still resolves - to the merged track", b.find(ids[0]) == b.find(first) &&
      b.find(first) != nullptr);
    chk("the votes were summed", b.tracks[0].red == 4 && b.tracks[0].n == 4);
    chk("the position is the weighted mean, nearer the good ones",
      b.tracks[0].p.y > 20.0 && b.tracks[0].p.y < 20.5);
    // Two real bays 2 m apart are never merged, whatever the gate.
    DockBook two;
    two.prm.gate_m = 5.0;
    Frame f;
    f.bays = {see({0, 8}, 0.0, m, W, south), see({0, 8}, 0.0, m, C, south)};
    two.ingest(f, {0, 8}, 0.0, m);
    chk("bays 2 m apart stay two", two.tracks.size() == 2);
  }

  // --------------------------------------------------------------- choice
  std::printf("choosing the safe bay\n");
  {
    auto dockWith = [&](int wr, int wg, int cr, int cg, int er, int eg) {
        DockBook b;
        Frame f;
        f.bays = {see({0, 8}, 0.0, m, W, south), see({0, 8}, 0.0, m, C, south),
          see({0, 8}, 0.0, m, E, south)};
        for (int i = 0; i < 6; ++i) {b.ingest(f, {0, 8}, 0.0, m);}
        auto vote = [&](Vec2 where, int r, int g) {
            for (auto & t : b.tracks) {
              if (nav::norm(t.p - where) < 1.0) {
                for (int i = 0; i < r; ++i) {
                  ++t.red; t.green_ema *= 0.85;
                }
                for (int i = 0; i < g; ++i) {
                  ++t.green; t.green_ema = 0.85 * t.green_ema + 0.15;
                }
              }
            }
          };
        vote(W, wr, wg);
        vote(C, cr, cg);
        vote(E, er, eg);
        return b;
      };
    const VoteParams v;
    {
      DockBook b = dockWith(12, 0, 0, 12, 12, 0);
      const Choice c = chooseSafeBay(b, layout(b, 5), v, true);
      chk("RED GREEN RED: bay 2", c.ok && c.bay_number == 2);
    }
    {
      DockBook b = dockWith(12, 0, 0, 12, 0, 12);
      const Choice c = chooseSafeBay(b, layout(b, 5), v, false);
      chk("two GREENs: refuse, even when lenient", !c.ok);
      chk("and say so", c.why.find("more than one") != std::string::npos);
    }
    {
      DockBook b = dockWith(0, 0, 0, 12, 2, 0);
      chk("GREEN + two unresolved: strict refuses",
        !chooseSafeBay(b, layout(b, 5), v, true).ok);
      const Choice c = chooseSafeBay(b, layout(b, 5), v, false);
      chk("GREEN + two unresolved: lenient takes it", c.ok && c.bay_number == 2);
    }
    {
      DockBook b = dockWith(12, 0, 0, 3, 12, 0);
      chk("too few votes is not GREEN", !chooseSafeBay(b, layout(b, 5), v, false).ok);
    }
    {
      // 40 green far out, then 7 red close in: 85% green all-time, but the
      // recent readings say otherwise. Not GREEN.
      DockBook b = dockWith(12, 0, 0, 40, 12, 0);
      BayTrack * t = nullptr;
      for (auto & k : b.tracks) {if (nav::norm(k.p - C) < 1.0) {t = &k;}}
      for (int i = 0; i < 7; ++i) {++t->red; t->green_ema *= 0.85;}
      chk_near("share is still 85%", t->greenShare(), 40.0 / 47.0, 1e-9);
      chk("but recent reds veto GREEN", verdict(*t, v) != Verdict::Green);
      chk("still safe at ema 0.32", stillSafe(*t, 0.3));
      ++t->red;
      t->green_ema *= 0.85;
      chk("one more red: no longer safe", !stillSafe(*t, 0.3));
    }
  }

  // --------------------------------------------------------------- vantage
  std::printf("where to look from\n");
  {
    DockBook none;
    const Vantage a = vantage(none, true, {5, 5}, 0, 0, 10.0, 5);
    chk_near("nothing seen: the approach point", a.p.x, 5.0, 1e-9);
    chk("which is not a look FROM the bays", a.ok && !a.from_bays);
    chk_near("no bay seen: the approach point is its own lead", a.lead.x, 5.0, 1e-9);
    chk("nothing seen and no approach: refuse", !vantage(none, false, {}, 0, 0, 10, 5).ok);
    // The ring search: 4 m out from the approach point, arriving facing out.
    const Vantage r1 = vantage(none, true, {5, 5}, 1, 0, 10.0, 5);
    chk_near("ring 1 looks NORTH: 4 m north of the approach point", r1.p.y, 9.0, 1e-9);
    chk_near("and is driven to FROM the approach point", r1.lead.y, 5.0, 1e-9);
    const Vantage r2 = vantage(none, true, {5, 5}, 2, 0, 10.0, 5);
    chk("ring 2 looks 72 deg: north-east", r2.p.x > 5.0 && r2.p.y > 5.0);
    chk("after the ring: refuse, the dock is not here",
      !vantage(none, true, {5, 5}, 6, 0, 10, 5).ok);

    DockBook b;
    Frame f;
    f.bays = {see({0, 8}, 0.0, m, W, south), see({0, 8}, 0.0, m, C, south),
      see({0, 8}, 0.0, m, E, south)};
    for (int i = 0; i < 6; ++i) {b.ingest(f, {0, 8}, 0.0, m);}
    const Vantage v0 = vantage(b, false, {}, 0, 0, 5.0, 5);
    chk("with bays seen, a look FROM the bays", v0.from_bays);
    chk_near("attempt 0: 5 m in front of the centre (x)", v0.p.x, 0.0, 1e-6);
    chk_near("attempt 0: on the WATER side, south (y)", v0.p.y, 15.0, 1e-6);
    chk_near("the lead is 3 m further out, so the last leg is driven NORTH at the dock",
      v0.lead.y, 12.0, 1e-6);
    b.tracks[0].red = 20;
    b.tracks[2].red = 20;
    const Vantage v1 = vantage(b, false, {}, 0, 1, 5.0, 5);
    chk_near("attempt 1: head on to the least-read bay (x)", v1.p.x, 0.0, 1e-6);
    chk_near("attempt 1: still 5 m out - never inside a slip (y)", v1.p.y, 15.0, 1e-6);
    b.tracks[1].red = 30;
    const Vantage v2 = vantage(b, false, {}, 0, 1, 5.0, 5);
    chk_near("the least-read bay changes: head on to the west one", v2.p.x, -2.0, 1e-6);
  }

  // --------------------------------------------------------------- berthing
  std::printf("berthing\n");
  {
    BayTrack t;
    t.p = C;
    t.normal_sum = south;
    t.n_normal = 1;
    // The real numbers: line up 3 m out (fingers 2 m + half a 1 m hull +
    // margin), berth with the body origin 1.3 m out (stern inside the fingers),
    // lead-in 2 m beyond the line-up point.
    const Berth b = berthFor(t, DockLayout{}, 3.0, 1.3, 2.0);
    chk("a berth", b.ok);
    chk_near("line up 3 m south of the face", b.predock.y, 17.0, 1e-9);
    chk_near("berth 1.3 m south of the face", b.berth.y, 18.7, 1e-9);
    chk_near("lead-in 2 m beyond the line-up point", b.lead.y, 15.0, 1e-9);
    chk_near("all three on the centreline", b.lead.x + b.predock.x + b.berth.x, 0.0, 1e-9);

    DockedCheck d = dockedIn(b, {0, 18.7}, 0.0, 1.3, 0.25, 0.3, 15.0);
    chk("at the berth, bow north (into the bay): docked", d.docked);
    d = dockedIn(b, {0, 18.7}, 180.0, 1.3, 0.25, 0.3, 15.0);
    chk("at the berth but stern-first: NOT docked", !d.docked);
    d = dockedIn(b, {0.4, 18.7}, 0.0, 1.3, 0.25, 0.3, 15.0);
    chk("0.4 m off the centreline: not docked", !d.docked);
    chk_near("... and it is to the RIGHT facing the bay (east)", d.lateral, 0.4, 1e-9);
    d = dockedIn(b, {0, 17.7}, 0.0, 1.3, 0.25, 0.3, 15.0);
    chk("1 m short: not docked", !d.docked);
    d = dockedIn(b, {0, 18.7}, 10.0, 1.3, 0.25, 0.3, 15.0);
    chk_near("bow 10 deg east of straight in reads +10 (bow right)",
      d.heading_err_deg, 10.0, 1e-6);
    chk("and is inside a 15 deg tolerance", d.docked);

    // The hull, not its middle: straight in, the corners reach half the beam
    // (the boat: ~1.0 x 0.6 m).
    chk_near("straight in: the corners reach half the beam",
      hullHalfWidthUsed(b, {0, 18.7}, 0.0, 1.0, 0.6), 0.3, 1e-9);
    // 0.3 m off and 15 deg skewed: the worst corner is 0.72 m out. The slip
    // edge is 0.75 m out (pitch 2 m, fingers 0.5 m) - it fits by 3 cm, which
    // is not a margin; DockedInBay keeps half a finger plus 0.1 m clear.
    const double reach = hullHalfWidthUsed(b, {0.3, 18.7}, 15.0, 1.0, 0.6);
    chk_near("0.3 m off and 15 deg skewed reaches 0.72 m", reach, 0.719, 0.002);
    chk("which is over DockedInBay's line (2/2 - 0.35)", reach > 2.0 / 2.0 - 0.35);

    // Lined up: the survey vantage in front of this bay, facing it.
    chk("5 m out on the centreline facing in: lined up",
      linedUp(b, {0, 15}, 0.0, 2.7, 0.4, 20.0));
    chk("... but facing AWAY: not", !linedUp(b, {0, 15}, 180.0, 2.7, 0.4, 20.0));
    chk("2 m to the side: not", !linedUp(b, {2, 15}, 0.0, 2.7, 0.4, 20.0));
    chk("already inside the line-up point: not", !linedUp(b, {0, 17.5}, 0.0, 2.7, 0.4, 20.0));
  }

  // --------------------------------------------------------------- the code
  std::printf("the resource request\n");
  {
    Request r = requestFrom("code", {Colour::Red, Colour::Blue});
    chk("code red,blue: RED tin", r.ok && r.resource == kWireRed);
    chk("code red,blue: to the BLUE circle", r.delivery == kWireBlue);
    r = requestFrom("code", {Colour::Blue, Colour::Blue});
    chk("c1 may equal c2", r.ok && r.resource == kWireBlue && r.delivery == kWireBlue);
    r = requestFrom("flash", {Colour::Green});
    chk("Advanced flash green: ANY tin to GREEN",
      r.ok && r.resource == kWireAny && r.delivery == kWireGreen);
    chk("code with one colour is refused", !requestFrom("code", {Colour::Red}).ok);
    chk("code with an unlit colour is refused",
      !requestFrom("code", {Colour::Red, Colour::Off}).ok);
    chk("steady is not a request", !requestFrom("steady", {Colour::Red}).ok);
    chk("unresolved is not a request", !requestFrom("unresolved", {}).ok);

    RequestHold h;
    const Request rb = requestFrom("code", {Colour::Red, Colour::Blue});
    const Request gg = requestFrom("code", {Colour::Green, Colour::Green});
    bool done = false;
    double when = -1.0;
    for (int i = 0; i <= 150 && !done; ++i) {       // 15 fps for 10 s
      const double t = i / 15.0;
      done = h.update(rb, t, 5.0, 30);
      if (done) {when = t;}
    }
    chk_near("held for 5 s: confirmed", when, 5.0, 0.07);
    h.reset();
    for (int i = 0; i < 60; ++i) {h.update(rb, i / 15.0, 5.0, 30);}    // 4 s of red,blue
    const bool flipped = h.update(gg, 4.0, 5.0, 30);
    chk("a different answer restarts the clock", !flipped && h.frames == 1);
    h.reset();
    for (int i = 0; i < 45; ++i) {h.update(rb, i / 15.0, 5.0, 30);}
    for (int i = 45; i < 60; ++i) {h.update(Request{}, i / 15.0, 5.0, 30);}  // pending
    chk("a non-answer does not reset", h.cand.ok && h.frames == 45);
    chk("and the clock kept running", h.update(rb, 5.0, 5.0, 30));

    HitWatch w;
    for (int i = 0; i < 20; ++i) {w.update(Colour::Green, i / 15.0, 0.4);}
    chk("GREEN with no RED before it is not a hit", !w.hit);
    w.reset();
    for (int i = 0; i < 15; ++i) {w.update(Colour::Red, i / 15.0, 0.4);}
    w.update(Colour::Green, 1.0, 0.4);
    w.update(Colour::Unknown, 1.2, 0.4);           // the rule abstained
    w.update(Colour::Green, 1.3, 0.4);
    chk("0.3 s of GREEN: not yet", !w.hit);
    w.update(Colour::Green, 1.45, 0.4);
    chk("0.45 s of GREEN after RED, across an abstention: hit", w.hit);
  }

  // --------------------------------------------------------------- reports
  std::printf("reports\n");
  {
    chk("docking report", dockingReportJson(2) == "{\"bay_id\":2}");
    chk("firefighting report", firefightingReportJson(1) == "{\"window_id\":1}");
    const Request r = requestFrom("code", {Colour::Red, Colour::Blue});
    chk("RoboCommand request carries proto NAMES",
      resourceRequestJson(r) ==
      "{\"task\":\"TASK_COORDINATED_LOGISTICS\",\"resource_color\":\"COLOR_RED\","
      "\"delivery_circle_color\":\"COLOR_BLUE\"}");
    chk("UAV request carries RXL numbers",
      uavRequestJson(r, 3) == "{\"seq\":3,\"resource_color\":1,\"delivery_color\":3}");
    chk("Advanced: COLOR_ANY", resourceRequestJson(requestFrom("flash", {Colour::Green}))
      .find("\"resource_color\":\"COLOR_ANY\"") != std::string::npos);
    chk("cannon off", cannonJson(false, 0, 0, 0).find("\"fire\":false") != std::string::npos);
  }

  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  std::printf("%s\n", g_fails == 0 ? "PASS" : "FAIL");
  return g_fails == 0 ? 0 : 1;
}
