// test_dock_sim — Task 3's dock trees closed loop, without Gazebo: the real XML and leaves on a
// Context, a holonomic hull answering the sticks, and a camera that sees three bay faces the
// way bt_runner_node gets them from camera_node (bearing + LEFT, the plane's range or a biased
// window-size range, faces cut by the picture's edge, missed boxes, junk faces, a GREEN
// indicator read OFF or not at all). Every scenario must commit the right bay and dock in it:
// the camera 1.7 m off its face, centred, square, and stay there.
//
// Links the leaves, so it needs ROS: build with colcon, run from the build directory
//
//     ./build/crusader_bt/test_dock_sim [trace_dir]
//
// trace_dir: one CSV per scenario (t, x, y, heading, sticks, phase) for a picture.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "crusader_bt/context.hpp"

namespace cb = crusader_bt;
namespace nav = crusader_bt::nav;
namespace dk = crusader_bt::dock;
using nav::Vec2;

#ifndef CRSD_BT_DIR
#define CRSD_BT_DIR "behavior_trees"
#endif

static int g_fails = 0;
static int g_checks = 0;

static void chk(const std::string & name, bool pass)
{
  ++g_checks;
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s\n", pass ? "ok" : "FAIL", name.c_str());
}

struct Scenario
{
  std::string name;
  std::string tree;                     ///< file in behavior_trees/
  std::vector<dk::Colour> colours;      ///< bay 1 (left, facing the bays) first
  double right_m = 6.0;                 ///< start: m right of bay 2's centre line (facing them)
  double out_m = 6.0;                   ///< ... and this far out from the faces
  double bow_off_deg = 40.0;            ///< the bow this far off the dock heading at the start
  int expect_bay = 3;
  double p_face = 0.9;                  ///< a whole face in view is boxed
  double p_ind = 0.6;                   ///< ... its indicator boxed too
  double p_off = 0.3;                   ///< a GREEN indicator read OFF
  double p_junk = 0.05;                 ///< a cut-off junk face at the picture's edge, per frame
  double p_far = 0.15;                  ///< a whole face 9 m beyond the dock (a building), per
                                        ///< frame: 6 m outside the course, so the fence stops it
  double p_plane = 0.8;                 ///< the face plane from depth (else the size range)
  double size_bias = 0.8;               ///< the window-size range reads short
  double current_mps = 0.02;            ///< a set across the dock
  unsigned seed = 1;
};

/// The truth: the hull, the dock, and what the camera reports.
struct World
{
  static constexpr double kPitch = 2.0;        ///< bay to bay, m
  static constexpr double kFaceW = 1.0;        ///< face width, m
  static constexpr double kHalfFov = 39.3;     ///< deg: fx 1173 at 1920 wide
  static constexpr double kCamX = 0.37;        ///< camera ahead of the body origin, m

  const Scenario & sc;
  double dock_h = 86.0;
  Vec2 fwd, right;                             ///< toward the faces; left->right facing them
  std::vector<Vec2> faces;
  Vec2 p;
  double h = 0.0, u = 0.0, v = 0.0, w = 0.0;   ///< heading; surge, sway (+stbd) m/s; deg/s
  std::mt19937 rng;

  explicit World(const Scenario & s)
  : sc(s), rng(s.seed)
  {
    fwd = nav::headingVec(dock_h);
    right = nav::starboardOf(fwd);
    for (int i = 0; i < 3; ++i) {faces.push_back(right * ((i - 1) * kPitch));}
    p = right * s.right_m - fwd * s.out_m;
    h = dock_h + s.bow_off_deg;
  }

  double gauss(double s) {return std::normal_distribution<double>(0.0, s)(rng);}
  bool chance(double pr) {return std::uniform_real_distribution<double>(0.0, 1.0)(rng) < pr;}

  /// Sticks in us about neutral (fwd + ahead, lat + starboard, yaw + right) for dt s. The
  /// T200 ESCs do nothing within +-25 us of neutral; 100 us out is 0.45 / 0.35 m/s, 30 deg/s.
  void step(double fwd_us, double lat_us, double yaw_us, double dt)
  {
    auto lag = [dt](double & x, double want, double tau) {x += (want - x) * std::min(1.0, dt / tau);};
    auto esc = [](double us) {
        const double a = std::max(0.0, std::min(std::fabs(us), 150.0) - 25.0) / 75.0;
        return std::copysign(a, us);
      };
    lag(u, esc(fwd_us) * 0.45, 0.8);
    lag(v, esc(lat_us) * 0.35, 0.8);
    lag(w, esc(yaw_us) * 30.0, 0.5);
    const Vec2 bf = nav::headingVec(h);
    p = p + (bf * u + nav::starboardOf(bf) * v + right * sc.current_mps) * dt;
    h = std::fmod(h + w * dt + 360.0, 360.0);
  }

  Vec2 cam() const {return p + nav::headingVec(h) * kCamX;}

  /// Bay `i`'s face in the camera frame: ahead, left (m).
  Vec2 inCam(int i) const
  {
    const Vec2 bf = nav::headingVec(h);
    const Vec2 d = faces[static_cast<std::size_t>(i)] - cam();
    return {nav::dot(d, bf), nav::dot(d, nav::portOf(bf))};
  }

  /// Across the dock from bay `i`'s face to the camera (+ = right of it, facing the bays),
  /// and the camera's distance out from the faces.
  void offBay(int i, double & across, double & out) const
  {
    const Vec2 d = cam() - faces[static_cast<std::size_t>(i)];
    across = nav::dot(d, right);
    out = -nav::dot(d, fwd);
  }

  /// One DockObservation, as bt_runner_node's onDock makes the dock::Frame of it.
  dk::Frame frame(double t)
  {
    dk::Frame f;
    f.t = t;
    const Vec2 bf = nav::headingVec(h);
    const Vec2 nw = fwd * -1.0;                       // the faces' normal, toward the water
    const double nx = nav::dot(nw, bf), ny = nav::dot(nw, nav::portOf(bf));
    for (int i = 0; i < 3; ++i) {
      const Vec2 c = inCam(i);
      if (c.x < 0.5) {continue;}
      const double dist = nav::norm(c);
      const double b = std::atan2(c.y, c.x) / nav::kDeg;
      const double half = std::atan2(kFaceW / 2.0, dist) / nav::kDeg;
      const double lo = std::max(b - half, -kHalfFov), hi = std::min(b + half, kHalfFov);
      if (hi - lo < 0.3 * 2.0 * half) {continue;}    // under a third of it in the picture
      const bool cut = lo > b - half || hi < b + half;
      if (!chance(cut ? 0.7 : sc.p_face)) {continue;}
      dk::BaySighting s;
      s.bearing_deg = (lo + hi) / 2.0 + gauss(0.3);   // the box's centre: biased when cut
      s.truncated = cut;
      if (dist < 9.0 && chance(sc.p_plane)) {
        s.has_normal = true;
        s.nx = nx + gauss(0.02);
        s.ny = ny + gauss(0.02);
        s.d = -(nx * c.x + ny * c.y) * (1.0 + gauss(0.02));
        s.range_m = dk::sightingRange(true, s.nx, s.ny, s.d, s.bearing_deg, kNaN);
      } else {
        s.range_m = dist * sc.size_bias * (1.0 + gauss(0.08));
      }
      const dk::Colour truth = sc.colours[static_cast<std::size_t>(i)];
      if (!cut && chance(sc.p_ind)) {
        s.indicator_present = true;
        s.indicator = truth == dk::Colour::Green && chance(sc.p_off) ? dk::Colour::Off : truth;
        s.indicator_conf = 0.8;
      }
      f.bays.push_back(s);
    }
    if (chance(sc.p_far)) {                           // a whole box beyond the dock
      const Vec2 bf2 = nav::headingVec(h);
      const Vec2 d = faces[1] + fwd * 9.0 + right * (chance(0.5) ? 5.0 : -5.0) - cam();
      const double ahead = nav::dot(d, bf2), left = nav::dot(d, nav::portOf(bf2));
      const double b = std::atan2(left, ahead) / nav::kDeg;
      if (ahead > 0.5 && std::fabs(b) < kHalfFov - 5.0) {    // the book drops it past 14 m
        dk::BaySighting s;
        s.bearing_deg = b;
        s.range_m = nav::norm(d);
        f.bays.push_back(s);
      }
    }
    if (chance(sc.p_junk)) {                          // a cut box at the edge: sky, a tower
      dk::BaySighting s;
      s.bearing_deg = (chance(0.5) ? 1.0 : -1.0) * (kHalfFov - 2.0);
      s.truncated = true;
      s.range_m = 3.0 + 3.0 * chance(0.5);
      f.bays.push_back(s);
    }
    std::sort(f.bays.begin(), f.bays.end(), [](const dk::BaySighting & a,
      const dk::BaySighting & b) {return a.bearing_deg > b.bearing_deg;});   // left to right
    return f;
  }

  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
};

/// One scenario: the tree from start to the docking report and 15 s past it.
static void run(const Scenario & sc, const std::string & trace_dir)
{
  std::printf("\n== %s (%s; expect bay %d)\n", sc.name.c_str(), sc.tree.c_str(), sc.expect_bay);
  World W(sc);
  auto ctx = std::make_shared<cb::Context>();
  double fwd = 0.0, lat = 0.0, yaw = 0.0, report_t = -1.0;
  std::string reports;
  ctx->sticks = [&](double f, double l, double y) {fwd = f; lat = l; yaw = y;};
  ctx->release_sticks = [&]() {fwd = lat = yaw = 0.0;};
  ctx->request_mode = [&](const std::string & m) {ctx->mode = m;};
  ctx->set_camera_model = [&](const std::string &, const std::string &, std::string &) {
      ctx->camera_model_ack = 1;
      return true;
    };
  ctx->report = [&](const std::string & kind, const std::string & body) {
      reports += kind + " " + body + "\n";
      if (kind == "docking" && report_t < 0.0) {report_t = ctx->now_s;}
    };
  ctx->set_task = [](const std::string &) {};
  ctx->send_setpoint = [](nav::LatLon) {};
  ctx->cannon = [](bool, double, double, double) {};
  ctx->pump = [](double, std::uint32_t) {};
  ctx->heading_speed = [](double, double) {};
  ctx->set_avoidance = [](bool) {};
  ctx->origin = nav::LatLon{1.2810, 103.8550};
  ctx->origin_set = true;
  ctx->mode = "MANUAL";
  ctx->autonomous = false;
  ctx->drop_tripped = false;
  ctx->publish_setpoints = true;
  ctx->dock_heading = W.dock_h;
  ctx->search_first = "left";
  ctx->has_approach = false;
  // RoboCommand's course: from 25 m off the faces to 3 m behind them, 15 m either side
  for (const Vec2 c : {W.fwd * -25.0 + W.right * -15.0, W.fwd * 3.0 + W.right * -15.0,
      W.fwd * 3.0 + W.right * 15.0, W.fwd * -25.0 + W.right * 15.0})
  {
    ctx->course_corners.push_back(nav::toLatLon(c, ctx->origin));
  }
  ctx->course_id = "SIM";
  ++ctx->course_seq;
  cb::resetTask3(*ctx);

  BT::BehaviorTreeFactory factory;
  cb::registerCrusaderNodes(factory);
  auto bb = BT::Blackboard::create();
  bb->set("ctx", ctx);
  BT::Tree tree = factory.createTreeFromFile(std::string(CRSD_BT_DIR) + "/" + sc.tree, bb);

  std::ofstream trace;
  if (!trace_dir.empty()) {
    trace.open(trace_dir + "/" + sc.name + ".csv");
    trace << "t,x,y,h,fwd,lat,yaw,phase,chosen_bay,tracks\n";
  }
  const double dt = 0.05, frame_dt = 0.1, latency = 0.1;
  std::deque<dk::Frame> pipe;
  double next_frame = 0.0, last_obs = -1e9, worst_lat = 0.0, worst_out = 0.0, best_out = 1e9;
  int commit_tracks = -1, most_tracks = 0;
  BT::NodeStatus st = BT::NodeStatus::RUNNING;
  double t = 0.0;
  for (; t < 300.0 && st == BT::NodeStatus::RUNNING; t += dt) {
    W.step(fwd, lat, yaw, dt);
    if (t >= next_frame) {pipe.push_back(W.frame(t)); next_frame += frame_dt;}
    {
      std::lock_guard<std::mutex> lk(ctx->mu);
      ctx->now_s = t;
      ctx->boat = W.p + Vec2{W.gauss(0.01), W.gauss(0.01)};      // RTK fixed
      ctx->heading_deg = std::fmod(W.h + W.gauss(0.3) + 360.0, 360.0);
      ctx->pose_fresh = true;
      cb::ingestAttitude(*ctx, t, 0.01 * std::sin(t), 0.01 * std::cos(t), 0.0, 0.0,
        W.w * nav::kDeg);
      ctx->att_age_s = 0.0;
      while (!pipe.empty() && t - pipe.front().t >= latency) {
        cb::ingestDockObservation(*ctx, pipe.front());
        last_obs = t;
        pipe.pop_front();
      }
      ctx->dock_obs_age_s = t - last_obs;
      most_tracks = std::max(most_tracks, static_cast<int>(ctx->dock.tracks.size()));
      if (commit_tracks < 0 && ctx->chosen_track >= 0) {
        commit_tracks = static_cast<int>(ctx->dock.tracks.size());
      }
    }
    st = tree.tickOnce();
    if (trace.is_open() && std::fmod(t, 0.2) < dt) {
      char buf[256];
      std::snprintf(buf, sizeof buf, "%.2f,%.3f,%.3f,%.1f,%.0f,%.0f,%.0f,\"%s\",%d,%zu\n", t,
        W.p.x, W.p.y, W.h, fwd, lat, yaw, ctx->task3_phase.c_str(), ctx->chosen_bay,
        ctx->dock.tracks.size());
      trace << buf;
    }
    if (report_t >= 0.0) {
      double across, out;
      W.offBay(sc.expect_bay - 1, across, out);
      worst_lat = std::max(worst_lat, std::fabs(across));
      worst_out = std::max(worst_out, out);
      best_out = std::min(best_out, out);
      if (t - report_t >= 15.0) {break;}
    }
  }
  tree.haltTree();

  double across = 0.0, out = 0.0;
  W.offBay(sc.expect_bay - 1, across, out);
  std::printf("   t %.0f s, tree %s, committed bay %d (%d tracks then, %zu now), docked %s\n",
    t, BT::toStr(st).c_str(), ctx->chosen_bay, commit_tracks, ctx->dock.tracks.size(),
    report_t >= 0.0 ? ("at " + std::to_string(static_cast<int>(report_t)) + " s").c_str() : "never");
  std::printf("   camera now %+.2f m across bay %d's face, %.2f m out, bow %+.1f deg off square\n",
    across, sc.expect_bay, out, cb::fire::wrap180(W.h - W.dock_h));
  chk(sc.name + ": committed bay " + std::to_string(sc.expect_bay),
    ctx->chosen_bay == sc.expect_bay);
  chk(sc.name + ": docking reported", report_t >= 0.0);
  chk(sc.name + ": in the bay at the report and 15 s on (|across| " +
    std::to_string(worst_lat).substr(0, 4) + " <= 0.30 m, out " +
    std::to_string(best_out).substr(0, 4) + "-" + std::to_string(worst_out).substr(0, 4) +
    " in 1.3-2.1 m)",
    report_t >= 0.0 && worst_lat <= 0.30 && best_out >= 1.3 && worst_out <= 2.1);
  chk(sc.name + ": never more than the dock's 3 bays in the book (" +
    std::to_string(most_tracks) + "), " + std::to_string(ctx->dock.fenced) +
    " faces beyond the course dropped", most_tracks >= 1 && most_tracks <= 3);
}

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  const std::string trace_dir = argc > 1 ? argv[1] : "";
  using C = dk::Colour;
  std::vector<Scenario> all;
  {
    Scenario s;
    s.name = "first_dock_right_end";
    s.tree = "task3_first_dock.xml";
    s.colours = {C::Green, C::Red, C::Red};
    s.expect_bay = 3;
    all.push_back(s);
  }
  {
    Scenario s = all.back();                     // the water run: dock at the picture's left
    s.name = "first_dock_water_start";           // edge, a worse camera, a stronger set
    s.right_m = 5.0;
    s.out_m = 4.5;
    s.bow_off_deg = 57.0;
    s.p_face = 0.75;
    s.p_junk = 0.15;
    s.size_bias = 0.7;
    s.p_plane = 0.5;
    s.current_mps = 0.04;
    s.seed = 7;
    all.push_back(s);
  }
  {
    Scenario s;
    s.name = "scan_fire_green_bay1";
    s.tree = "task3_scan_fire.xml";
    s.colours = {C::Green, C::Red, C::Red};
    s.expect_bay = 1;
    s.p_off = 0.5;
    s.seed = 3;
    all.push_back(s);
  }
  {
    Scenario s = all.back();
    s.name = "scan_fire_green_bay2";
    s.colours = {C::Red, C::Green, C::Red};
    s.expect_bay = 2;
    s.seed = 5;
    all.push_back(s);
  }
  for (const auto & s : all) {run(s, trace_dir);}
  rclcpp::shutdown();
  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  std::printf("%s\n", g_fails == 0 ? "PASS" : "FAIL");
  return g_fails == 0 ? 0 : 1;
}
