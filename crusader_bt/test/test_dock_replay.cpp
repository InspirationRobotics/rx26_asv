// test_dock_replay — the 2026-11-12 water run's camera, frame by frame, through the boat's
// own DockBook: does it find the dock's three bays and nothing else, and can our bay be
// followed once committed?
//
// test/data/run2_20261112-032016.txt is task3_first_dock on course ALPHA: the boat's
// camera frames at 1 Hz through its dock model (best.pt), with the pose and heading of
// each. It committed to bay 3, the right end, at frame 24 and never docked: camera_node
// published the bearings +right and the runner read them +left. The fixture holds them as
// the message now carries them (+left, flipped in dock_obs_core).
//
// No ROS:  g++ -std=c++17 -O2 -I include test/test_dock_replay.cpp -o test_dock_replay
//          ./test_dock_replay [-v] [window_size_m]
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "crusader_bt/dock_math.hpp"
#include "crusader_bt/fire_math.hpp"

namespace dk = crusader_bt::dock;
namespace nav = crusader_bt::nav;

#ifndef CRSD_BT_TEST_DATA
#define CRSD_BT_TEST_DATA "test/data"
#endif

static int g_fails = 0;
static int g_checks = 0;

static void chk(const std::string & name, bool pass)
{
  ++g_checks;
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s\n", pass ? "ok" : "FAIL", name.c_str());
}

/// ffcv.yaml's window_size_m when the fixture's ranges were made, and now: the box the
/// model draws holds the window's hole AND its black border. 0.48 is this recording's own
/// answer: bay 3 seen from either side lands in one place (0.1 m apart; 1.4 m at 0.40).
constexpr double kWindowWas = 0.25;
constexpr double kWindowNow = 0.48;
/// The bow's heading facing the bays on that run (the GCS's dock_heading_deg).
constexpr double kDockHeading = 86.0;
/// The water run's commit: task3_first_dock took the first bay it saw, bay 3.
constexpr int kCommitFrame = 24;
/// RoboCommand's course ALPHA, as the OCS relayed it on 2026-11-12 (RxCourse corners).
static const std::vector<nav::LatLon> kAlpha = {
  {1.2809887596341554, 103.85481480992878}, {1.2808658372160693, 103.85500721025717},
  {1.2810900427153769, 103.85515265929305}, {1.2812172480539488, 103.8549604814802}};

struct Row
{
  int frame = 0;
  double lat = 0, lon = 0, hdg = 0;
  dk::Frame f;
};

static std::vector<Row> load(const std::string & path, double window_m)
{
  std::vector<Row> rows;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::vector<std::string> parts;
    std::stringstream ss(line);
    for (std::string p; std::getline(ss, p, '|'); ) {parts.push_back(p);}
    Row r;
    int n = 0;
    std::stringstream h(parts[0]);
    h >> r.frame >> r.f.t >> r.lat >> r.lon >> r.hdg >> n;
    for (int i = 1; i <= n && i < static_cast<int>(parts.size()); ++i) {
      std::stringstream b(parts[static_cast<std::size_t>(i)]);
      double brg = 0, conf = 0;
      std::string rng, col;
      int tr = 0;
      b >> brg >> rng >> tr >> col >> conf;
      dk::BaySighting s;
      s.bearing_deg = brg;
      s.range_m = rng == "nan" ? NAN : std::stod(rng) * window_m / kWindowWas;
      s.truncated = tr != 0;
      if (col != "-") {
        s.indicator_present = true;
        s.indicator = col == "R" ? dk::Colour::Red : col == "G" ? dk::Colour::Green :
          col == "O" ? dk::Colour::Off : dk::Colour::Unknown;
        s.indicator_conf = conf;
      }
      r.f.bays.push_back(s);
    }
    rows.push_back(r);
  }
  return rows;
}

/// Across (+ right, facing the bays) and along the dock heading, m.
static double across(nav::Vec2 p) {return nav::dot(p, nav::starboardOf(nav::headingVec(kDockHeading)));}
static double along(nav::Vec2 p) {return nav::dot(p, nav::headingVec(kDockHeading));}

int main(int argc, char ** argv)
{
  bool verbose = false;
  double window_m = kWindowNow;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "-v") {verbose = true;} else {window_m = std::atof(argv[i]);}
  }
  const std::string path = std::string(CRSD_BT_TEST_DATA) + "/run2_20261112-032016.txt";
  const std::vector<Row> rows = load(path, window_m);
  std::printf("== %s: %zu frames, window_size_m %.2f\n", path.c_str(), rows.size(), window_m);
  if (rows.empty()) {chk("the fixture loads", false); return 1;}

  const nav::LatLon origin{rows[0].lat, rows[0].lon};
  dk::DockBook book;
  for (const auto & c : kAlpha) {book.keep_in.push_back(nav::toLocal(c, origin));}
  const dk::Mount m;
  bool have = false;
  int ours_track = -1;
  dk::FrameBay ours;
  int seen = 0, lost = 0, max_tracks = 0, wrong_side = 0;
  for (const Row & r : rows) {
    const nav::Vec2 boat = nav::toLocal({r.lat, r.lon}, origin);
    book.ingest(r.f, boat, r.hdg, m, kDockHeading);
    max_tracks = std::max(max_tracks, static_cast<int>(book.tracks.size()));
    if (r.frame == kCommitFrame && !book.frame_bays.empty()) {
      ours = book.frame_bays[0];
      ours_track = ours.track;
      have = true;
      if (verbose) {
        std::printf("  frame %2d: committed, %zu faces placed, ours across %.2f\n", r.frame,
          book.frame_bays.size(), across(ours.p));
      }
    } else if (have && r.frame > kCommitFrame) {
      // as pickGreen follows a committed bay: the face the book put on its track
      const dk::FrameBay * best = book.onTrack(ours_track);
      if (best) {
        ours = *best;
        ++seen;
        // bay 3 is the right end: no whole face placed in this frame lies right of it
        for (const auto & b : book.frame_bays) {
          if (!b.truncated && across(b.p) > across(ours.p) + 1.0) {++wrong_side; break;}
        }
      } else if (!book.frame_bays.empty()) {
        ++lost;
      }
      if (verbose) {
        std::string all;
        for (const auto & b : book.frame_bays) {
          all += " " + std::to_string(across(b.p)).substr(0, 6);
        }
        std::printf("  frame %2d hdg %5.1f: faces across%s | ours %s\n", r.frame, r.hdg,
          all.c_str(), best ? std::to_string(across(best->p)).substr(0, 6).c_str() : "-");
      }
    }
  }

  std::vector<double> xs;
  for (const auto & t : book.tracks) {
    std::printf("   track %d: n %3d  across %+6.2f m  along %+6.2f m  R%d G%d OFF%d\n", t.id, t.n,
      across(t.p), along(t.p), t.red, t.green, t.off);
    xs.push_back(across(t.p));
  }
  std::sort(xs.begin(), xs.end());
  double worst_pitch = 0.0, worst_line = 0.0;
  for (std::size_t i = 1; i < xs.size(); ++i) {worst_pitch = std::max(worst_pitch, std::fabs(xs[i] - xs[i - 1] - 2.0));}
  for (const auto & t : book.tracks) {
    worst_line = std::max(worst_line, std::fabs(along(t.p) - along(book.tracks[0].p)));
  }
  std::printf("   %d faces dropped beyond course ALPHA; ours followed in %d frames after the "
    "commit, lost in %d\n", book.fenced, seen, lost);

  chk("the book ends with the dock's 3 bays (" + std::to_string(book.tracks.size()) + ")",
    book.tracks.size() == 3);
  chk("never more than 3 tracks (" + std::to_string(max_tracks) + ")", max_tracks <= 3);
  chk("the bays 2 m apart across the dock, +-0.6 m (worst " +
    std::to_string(worst_pitch).substr(0, 4) + ")", xs.size() == 3 && worst_pitch <= 0.6);
  chk("... and in a line, within 1.5 m along it (worst " +
    std::to_string(worst_line).substr(0, 4) + ")", worst_line <= 1.5);
  chk("our bay followed in 90 % of the frames that placed a face after the commit (" +
    std::to_string(seen) + " of " + std::to_string(seen + lost) + ")",
    have && seen >= 9 * (seen + lost) / 10);
  chk("... and it stays the right end (" + std::to_string(wrong_side) + " frames with a "
    "face right of it)", wrong_side == 0);
  chk("no face of the real dock is dropped by the course fence (" + std::to_string(book.fenced) +
    "): its bays sit 0.9-1.8 m inside ALPHA's edge", book.fenced == 0);
  const dk::BayTrack * t = book.find(ours_track);
  chk("the bay followed is the book's right end, bay 3",
    t != nullptr && !xs.empty() && across(t->p) == xs.back());

  // The course fence on its own: a face placed 30 m out across the water is dropped.
  {
    dk::DockBook b;
    for (const auto & c : kAlpha) {b.keep_in.push_back(nav::toLocal(c, origin));}
    dk::Frame f;
    f.t = 1.0;
    dk::BaySighting s;
    s.bearing_deg = 0.0;
    s.range_m = 12.0;
    f.bays.push_back(s);
    const nav::Vec2 boat = nav::toLocal({rows[0].lat, rows[0].lon}, origin);
    b.ingest(f, boat, 180.0, m, kDockHeading);        // due south: out of the course in 12 m
    const bool out = b.tracks.empty() && b.frame_bays.empty() && b.fenced == 1;
    b.ingest(f, boat, 30.0, m, kDockHeading);         // 12 m on 030: still inside ALPHA
    chk("a face placed outside the course places nothing; one inside is a bay",
      out && b.tracks.size() == 1 && b.fenced == 1);
  }

  std::printf("\n%d checks, %d failed\n%s\n", g_checks, g_fails, g_fails == 0 ? "PASS" : "FAIL");
  return g_fails == 0 ? 0 : 1;
}
