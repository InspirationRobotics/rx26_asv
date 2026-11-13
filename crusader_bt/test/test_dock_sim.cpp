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
namespace t4 = crusader_bt::t4;
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
  // The fire (the cannon branch): our bay's window lights RED light_s after the docking report,
  // as at the competition, turns GREEN after hit_s of spray from within 2.6 m of the face, and
  // the hold must then back off to backoff_m (t3_backoff_m, which bt_runner_node always puts on
  // the blackboard at Start). Windows are only drawn when fire is on, so the scenarios without
  // it keep their random streams.
  bool fire = false;
  int lit_slot = 0;                     ///< DockWindow.index of our bay's lit window
  double light_s = 3.0;
  double hit_s = 2.0;
  double backoff_m = 4.0;
  double p_win = 0.85;                  ///< a window of a whole face in view is boxed
  // A mission tree with Task 3 first (mission_t3_t4_t1): run until Task 3 hands the boat over
  // (asks for GUIDED) and check it ENDS after the one fire, backed off, with the reports sent.
  bool until_handover = false;
  // The resource request the window flashes after the hit (3.3.4: GREEN 5 s, off 1 s, then c1 1 s,
  // off 1 s, c2 1 s, off 2 s for 60 s), as the timing layer reads it: "code" [c1, c2] once two 5 s
  // cycles have run. Empty: no code, as the scenarios before it had (the decode's timeout is wall
  // clock, so without one a tree that waits for the code never ends here).
  std::vector<dk::Colour> code;
  double code_s = 50.0;                 ///< ... read that long (the 60 s of code less two cycles)
  // Task 4 (Disruptive): a virtual boat crossing the dock front at t4_mps, t4_off_m off the faces,
  // alerted (RoboCommand's 10 m keep-out) at t4_alert_s with it t4_lead_m left of the dock's centre,
  // then as RoboCommand might: the same message re-delivered 1 s on, an update 10 s on. < 0: none.
  double t4_alert_s = -1.0;
  double t4_lead_m = 60.0, t4_off_m = 5.0, t4_mps = 1.0;
  bool guided_moves = false;            ///< the boat follows GUIDED setpoints (Task 4's escape)
  bool t4_after_hit = false;            ///< t4_alert_s counts from the hit, not the start
  // The stick path (2026-11-13 water runs: the search slid left at 100 us, ~0.3 m/s, and the
  // approach's 30-60 us corrections barely moved the hull back: the boat hit the bay's left finger)
  double fc_dz_us = 0.0;                ///< the FC's RC dead zone (RCx_DZ), taken off every stick
  double lat_mps = 0.35;                ///< sway 75 us past the ESC deadband
  double sway_tau_s = 0.8;              ///< how long a sideways slide takes to die away
  cb::t3tune::Values t3;                ///< the Run panel's "Task 3 control"; a key not here: default
  bool baseline = false;                ///< a before-picture: its checks are printed, not counted
};

/// How many reports of `kind` the run sent.
static int countReports(const std::string & reports, const std::string & kind)
{
  int n = 0;
  for (std::size_t i = reports.find(kind + " "); i != std::string::npos;
    i = reports.find(kind + " ", i + 1)) {++n;}
  return n;
}

/// The truth: the hull, the dock, and what the camera reports.
struct World
{
  static constexpr double kPitch = 2.0;        ///< bay to bay, m
  static constexpr double kFaceW = 1.0;        ///< face width, m
  static constexpr double kHalfFov = 39.3;     ///< deg: fx 1173 at 1920 wide
  static constexpr double kCamX = 0.37;        ///< camera ahead of the body origin, m
  static constexpr double kWinDx = 0.25;       ///< windows 0 / 1 this far left / right of centre
  static constexpr double kWinZ = 0.6;         ///< ... and this far above the camera, m
  static constexpr double kFx = 1173.0, kCx = 960.0;   ///< the RGB pinhole, 1920 px wide
  static constexpr double kSlipW = 1.5;        ///< between a bay's fingers (nav_dock_slip_w_m)
  static constexpr double kFingerLen = 2.0;    ///< fingers out from the faces (nav_dock_finger_len_m)
  /// The hull about the body origin, ASSUMED: Nav2's robot_radius 0.82 m circumscribes it.
  static constexpr double kHullL = 1.4, kHullB = 0.8;

  const Scenario & sc;
  int lit_bay = -1;                            ///< 0-based bay with the lit window; -1 = no fire
  dk::Colour lit = dk::Colour::Off;            ///< its state: OFF, RED, then GREEN once hit
  double code_from = -1.0;                     ///< the timing layer reads sc.code from here; < 0 not
  double t4_t0 = -1.0;                         ///< the virtual boat: first alerted at; < 0 not yet
  Vec2 t4_p0, t4_v;                            ///< ... where it was then, and its velocity
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

  /// Sticks in us about neutral (fwd + ahead, lat + starboard, yaw + right) for dt s. The FC
  /// takes its RC dead zone (sc.fc_dz_us) off each; the T200 ESCs then do nothing within +-25 us
  /// of neutral; 100 us out is 0.45 m/s, sc.lat_mps sideways, 30 deg/s.
  void step(double fwd_us, double lat_us, double yaw_us, double dt)
  {
    auto esc = [this](double us) {
        const double past_dz = std::max(0.0, std::fabs(us) - sc.fc_dz_us);
        const double a = std::max(0.0, std::min(past_dz, 150.0) - 25.0) / 75.0;
        return std::copysign(a, us);
      };
    lag(u, esc(fwd_us) * 0.45, 0.8, dt);
    lag(v, esc(lat_us) * sc.lat_mps, sc.sway_tau_s, dt);
    lag(w, esc(yaw_us) * 30.0, 0.5, dt);
    integrate(dt);
  }

  /// How far a hull corner is into bay i's fingers, m (0 = clear): a corner between the faces
  /// and the finger tips, past the slip's side and short of the next slip.
  double fingerDepth(int i) const
  {
    const Vec2 bf = nav::headingVec(h), sb = nav::starboardOf(bf);
    double deepest = 0.0;
    for (const double a : {-0.5, 0.5}) {
      for (const double b : {-0.5, 0.5}) {
        const Vec2 c = p + bf * (a * kHullL) + sb * (b * kHullB) -
          faces[static_cast<std::size_t>(i)];
        const double out = -nav::dot(c, fwd), across = std::fabs(nav::dot(c, right));
        if (out < 0.0 || out > kFingerLen) {continue;}
        const double into = std::min(across - kSlipW / 2.0, kPitch - kSlipW / 2.0 - across);
        deepest = std::max(deepest, into);
      }
    }
    return deepest;
  }

  /// GUIDED: ArduRover's position controller as a velocity toward the setpoint (WP_SPEED 0.5,
  /// easing in over the last 1.25 m), the heading held: the omni frame strafes.
  void stepGuided(Vec2 goal, double dt)
  {
    const Vec2 d = goal - p;
    const double dist = nav::norm(d);
    const Vec2 want = dist > 1e-6 ? d * (std::min(0.5, 0.4 * dist) / dist) : Vec2{0.0, 0.0};
    const Vec2 bf = nav::headingVec(h);
    lag(u, nav::dot(want, bf), 0.8, dt);
    lag(v, nav::dot(want, nav::starboardOf(bf)), 0.8, dt);
    lag(w, 0.0, 0.5, dt);
    integrate(dt);
  }

  static void lag(double & x, double want, double tau, double dt)
  {
    x += (want - x) * std::min(1.0, dt / tau);
  }

  /// The body rates over dt, with the set across the dock.
  void integrate(double dt)
  {
    const Vec2 bf = nav::headingVec(h);
    p = p + (bf * u + nav::starboardOf(bf) * v + right * sc.current_mps) * dt;
    h = std::fmod(h + w * dt + 360.0, 360.0);
  }

  /// Where the virtual boat truly is at t (straight on, as RoboCommand projects it).
  Vec2 invader(double t) const {return t4_p0 + t4_v * (t - t4_t0);}

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
      if (!cut && lit_bay >= 0) {                     // the windows, as DockWindow gives them
        for (int k = 0; k < 2; ++k) {
          if (!chance(sc.p_win)) {continue;}
          const Vec2 d = faces[static_cast<std::size_t>(i)] + right * (k == 0 ? -kWinDx : kWinDx) -
            cam();
          const double ax = nav::dot(d, bf), ay = nav::dot(d, nav::portOf(bf));
          if (ax < 0.5 || std::fabs(std::atan2(ay, ax) / nav::kDeg) > kHalfFov) {continue;}
          dk::WindowSighting ws;
          ws.index = k;
          ws.state = i == lit_bay && k == sc.lit_slot ? lit : dk::Colour::Off;
          ws.conf = 0.9;
          ws.has_position = true;
          ws.x = ax;
          ws.y = ay;
          ws.z = kWinZ;
          ws.u_px = kCx - kFx * ay / ax;
          s.windows.push_back(ws);
        }
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
    if (code_from >= 0.0 && t >= code_from && t < code_from + sc.code_s) {   // to the code's end
      f.target_pattern = "code";
      f.target_colours = sc.code;
    }
    return f;
  }

  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
};

/// One scenario: the tree from start to the docking report and 15 s past it.
static void run(const Scenario & sc, const std::string & trace_dir)
{
  std::printf("\n== %s (%s; expect bay %d)\n", sc.name.c_str(), sc.tree.c_str(), sc.expect_bay);
  World W(sc);
  if (sc.fire) {W.lit_bay = sc.expect_bay - 1;}
  auto ctx = std::make_shared<cb::Context>();
  double fwd = 0.0, lat = 0.0, yaw = 0.0, report_t = -1.0;
  bool cannon_on = false, fire_reported = false;
  double fire_report_t = -1.0, guided_t = -1.0, handover_out = -1.0, resource_t = -1.0;
  std::vector<std::string> tasks;                 // the heartbeat's current_task, as it changed
  std::string reports, resource_body;
  ctx->sticks = [&](double f, double l, double y) {fwd = f; lat = l; yaw = y;};
  ctx->release_sticks = [&]() {fwd = lat = yaw = 0.0;};
  ctx->request_mode = [&](const std::string & m) {
      ctx->mode = m;
      ctx->autonomous = m == "GUIDED";            // as the bridge reports an armed GUIDED boat
      // Task 1's: asked for once Task 3 has ended (a Task 4 escape asks with Task 3 still on)
      if (m == "GUIDED" && fire_reported && guided_t < 0.0 && !tasks.empty() &&
        tasks.back() == "TASK_NONE")
      {
        guided_t = ctx->now_s;
      }
    };
  ctx->set_camera_model = [&](const std::string &, const std::string &, std::string &) {
      ctx->camera_model_ack = 1;
      return true;
    };
  ctx->report = [&](const std::string & kind, const std::string & body) {
      reports += kind + " " + body + "\n";
      if (kind == "docking" && report_t < 0.0) {report_t = ctx->now_s;}
      if (kind == "firefighting" && !fire_reported) {
        fire_reported = true;
        fire_report_t = ctx->now_s;
      }
      if (kind == "resource_delivery" && resource_t < 0.0) {
        resource_t = ctx->now_s;
        resource_body = body;
      }
    };
  ctx->set_task = [&](const std::string & k) {
      if (tasks.empty() || tasks.back() != k) {tasks.push_back(k);}
    };
  Vec2 guided_goal;
  bool have_goal = false;
  ctx->send_setpoint = [&](nav::LatLon ll) {
      guided_goal = nav::toLocal(ll, ctx->origin);
      have_goal = true;
    };
  // Task 4: a MovingObjectAlert as bt_runner_node's onMovingObject takes it. CALL UNDER ctx->mu.
  double t4_min = 1e9, t4_min_t = -1.0;           // the closest the virtual boat truly came
  int t4_sent = 0;
  auto alert = [&](double t_now, double seq) {
      t4::Alert a;
      const nav::LatLon ll = nav::toLatLon(W.invader(t_now), ctx->origin);
      a.lat = ll.lat;
      a.lon = ll.lon;
      a.heading_deg = nav::bearingDeg(Vec2{0.0, 0.0}, W.t4_v);
      a.speed_mps = sc.t4_mps;
      a.radius_m = 10.0;
      a.domains = {"USV"};
      a.seq = seq;
      ++ctx->task4.alerts;
      if (!t4::acceptSeq(a.seq, ctx->task4.last_seq)) {return;}
      const std::size_t before = ctx->task4.objects.size();
      t4::ingest(ctx->task4.objects, ctx->task4.next_id, a, nav::toLocal(ll, ctx->origin), t_now,
        ctx->task4.settings);
      ctx->task4.last_alert_s = t_now;
      if (t4::shouldAck(ctx->task4.objects.size() > before, a)) {
        cb::sendReport(*ctx, "incident_ack", t4::incidentAckJson(a.seq));
      }
    };
  ctx->cannon = [&](bool on, double, double, double) {cannon_on = on;};
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
  bb->set("t3_backoff_m", sc.backoff_m);          // as bt_runner_node does at Start
  bb->set("t1_scan_dwell_s", 5.0);                // ... and the rest of its Start inputs,
  bb->set("t1_pause_every_m", 5.0);               // which the mission trees read
  bb->set("t1_unknown_range_m", 12.0);
  for (const char * k : {"t1_approach_lat", "t1_approach_lon", "t3_via_lat", "t3_via_lon"}) {
    bb->set(k, 0.0);                              // unset, as the Run panel leaves them
  }
  for (const cb::t3tune::Knob & k : cb::t3tune::kKnobs) {   // Task 3 control, as at Start
    bb->set(k.key, cb::t3tune::get(sc.t3, k.key));
  }
  const double hold_m = cb::t3tune::get(sc.t3, "t3_hold_m");
  const double offset_m = cb::t3tune::get(sc.t3, "t3_lat_offset_m");
  // a baseline's checks are a picture of the old behaviour: printed, not counted
  auto ck = [&sc](const std::string & what, bool pass) {
      if (!sc.baseline) {chk(what, pass); return;}
      std::printf("   [baseline, not counted] %s: %s\n", what.c_str(), pass ? "yes" : "NO");
    };
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
  // from the commit: time with a hull corner in the bay's fingers and the deepest; the camera's
  // furthest left / right of where it should sit
  double commit_t = -1.0, finger_s = 0.0, finger_depth = 0.0, most_left = 0.0, most_right = 0.0;
  // the fire: when the light came on, the camera's distance out when the spray first counted,
  // the hit, and the hold 15-30 s after it (out, |across|, |bow off square|)
  double light_t = -1.0, spray_out = -1.0, pumped = 0.0, hit_t = -1.0;
  double bo_min = 1e9, bo_max = 0.0, bo_lat = 0.0, bo_yaw = 0.0;
  BT::NodeStatus st = BT::NodeStatus::RUNNING;
  double t = 0.0;
  const double t_max = sc.until_handover ? 600.0 : sc.fire ? 420.0 : 300.0;
  for (; t < t_max && st == BT::NodeStatus::RUNNING; t += dt) {
    if (sc.guided_moves && ctx->mode == "GUIDED" && have_goal) {
      W.stepGuided(guided_goal, dt);
    } else {
      W.step(fwd, lat, yaw, dt);
    }
    if (W.t4_t0 >= 0.0 && nav::norm(W.p - W.invader(t)) < t4_min) {
      t4_min = nav::norm(W.p - W.invader(t));
      t4_min_t = t;
    }
    if (sc.fire) {
      double f_across, f_out;
      W.offBay(sc.expect_bay - 1, f_across, f_out);
      if (report_t >= 0.0 && light_t < 0.0 && t >= report_t + sc.light_s) {
        W.lit = dk::Colour::Red;
        light_t = t;
      }
      if (W.lit == dk::Colour::Red && cannon_on && f_out < 2.6) {
        if (spray_out < 0.0) {spray_out = f_out;}
        pumped += dt;
        if (pumped >= sc.hit_s) {
          W.lit = dk::Colour::Green;
          hit_t = t;
          if (!sc.code.empty()) {W.code_from = t + 16.0;}   // GREEN 5 + off 1 + two 5 s cycles
        }
      }
      if (hit_t >= 0.0 && t - hit_t >= 15.0 && guided_t < 0.0) {   // ... till Task 1 has it
        bo_min = std::min(bo_min, f_out);
        bo_max = std::max(bo_max, f_out);
        bo_lat = std::max(bo_lat, std::fabs(f_across));
        bo_yaw = std::max(bo_yaw, std::fabs(cb::fire::wrap180(W.h - W.dock_h)));
      }
    }
    if (commit_t >= 0.0) {                        // from the commit on: the slip and the overshoot
      const double depth = W.fingerDepth(sc.expect_bay - 1);
      if (depth > 0.0) {finger_s += dt;}
      finger_depth = std::max(finger_depth, depth);
      double c_across, c_out;
      W.offBay(sc.expect_bay - 1, c_across, c_out);
      most_left = std::max(most_left, -c_across - offset_m);
      most_right = std::max(most_right, c_across + offset_m);
    }
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
      const double t4_at = !sc.t4_after_hit ? sc.t4_alert_s :
        hit_t >= 0.0 ? hit_t + sc.t4_alert_s : 1e18;
      if (sc.t4_alert_s >= 0.0 && t >= t4_at) {
        if (W.t4_t0 < 0.0) {
          W.t4_t0 = t;
          W.t4_p0 = W.faces[1] - W.fwd * sc.t4_off_m - W.right * sc.t4_lead_m;
          W.t4_v = W.right * sc.t4_mps;
        }
        const double since = t - W.t4_t0;
        if (t4_sent == 0) {alert(t, 4217.0); t4_sent = 1;}
        if (t4_sent == 1 && since >= 1.0) {alert(t, 4217.0); t4_sent = 2;}   // re-delivered
        if (t4_sent == 2 && since >= 10.0) {alert(t, 4218.0); t4_sent = 3;}  // an update
      }
      most_tracks = std::max(most_tracks, static_cast<int>(ctx->dock.tracks.size()));
      if (commit_tracks < 0 && ctx->chosen_track >= 0) {
        commit_tracks = static_cast<int>(ctx->dock.tracks.size());
        commit_t = t;
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
      if (t - report_t <= 15.0 && hit_t < 0.0) {     // the docked hold, before any back-off
        worst_lat = std::max(worst_lat, std::fabs(across + offset_m));   // from where it should sit
        worst_out = std::max(worst_out, out);
        best_out = std::min(best_out, out);
      }
      if (sc.until_handover) {
        if (guided_t >= 0.0 && handover_out < 0.0) {handover_out = out;}
        if (guided_t >= 0.0 && t - guided_t >= 3.0) {break;}
        continue;
      }
      if (!sc.fire && t - report_t >= 15.0) {break;}
      if (sc.fire && ((hit_t >= 0.0 && t - hit_t >= 30.0) || t - report_t >= 150.0)) {break;}
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
  ck(sc.name + ": committed bay " + std::to_string(sc.expect_bay),
    ctx->chosen_bay == sc.expect_bay);
  std::printf("   from the commit (%.1f s): camera up to %.2f m left / %.2f m right of its spot; "
    "a hull corner in the fingers %.1f s, %.2f m deep at most\n",
    commit_t, most_left, most_right, finger_s, finger_depth);
  ck(sc.name + ": docking reported", report_t >= 0.0);
  char want[96];
  std::snprintf(want, sizeof want, "|across - spot| <= 0.30 m, out in %.1f-%.1f m",
    hold_m - 0.4, hold_m + 0.4);
  ck(sc.name + ": in the bay at the report and 15 s on (" + want + "; |across - spot| " +
    std::to_string(worst_lat).substr(0, 4) + ", out " +
    std::to_string(best_out).substr(0, 4) + "-" + std::to_string(worst_out).substr(0, 4) + ")",
    report_t >= 0.0 && worst_lat <= 0.30 && best_out >= hold_m - 0.4 &&
    worst_out <= hold_m + 0.4);
  ck(sc.name + ": never into the bay's fingers (" + std::to_string(finger_s).substr(0, 4) +
    " s, " + std::to_string(finger_depth).substr(0, 4) + " m)", finger_s <= 0.0);
  ck(sc.name + ": never more than the dock's 3 bays in the book (" +
    std::to_string(most_tracks) + "), " + std::to_string(ctx->dock.fenced) +
    " faces beyond the course dropped", most_tracks >= 1 && most_tracks <= 3);
  if (sc.fire) {
    std::printf("   fire: RED at %.1f s, spray counted from %.2f m out, GREEN at %.1f s; 15-30 s "
      "after: %.2f-%.2f m out (want %.1f), |across| <= %.2f m, |off square| <= %.1f deg\n",
      light_t, spray_out, hit_t, bo_min, bo_max, sc.backoff_m, bo_lat, bo_yaw);
    ck(sc.name + ": sprayed from the hold (t3_hold_m +-0.4 m out)",
      spray_out >= hold_m - 0.4 && spray_out <= hold_m + 0.4);
    ck(sc.name + ": the window went GREEN", hit_t >= 0.0);
    ck(sc.name + ": firefighting reported", fire_reported);
    if (!sc.t4_after_hit) {                      // a break-off then takes the boat away
      ck(sc.name + ": backed off to t3_backoff_m " + std::to_string(sc.backoff_m).substr(0, 3) +
        " +-0.4 m, centred (<= 0.35 m) and square (<= 10 deg), 15-30 s after the hit",
        hit_t >= 0.0 && bo_min >= sc.backoff_m - 0.4 && bo_max <= sc.backoff_m + 0.4 &&
        bo_lat <= 0.35 && bo_yaw <= 10.0);
    }
  }
  if (sc.until_handover) {
    std::string seq;
    for (const std::string & k : tasks) {seq += (seq.empty() ? "" : " > ") + k;}
    std::printf("   Task 3 handed over at %.1f s (firefighting reported at %.1f s, the resource "
      "request at %.1f s: %s), %.2f m off the face; current_task %s\n", guided_t, fire_report_t,
      resource_t, resource_body.c_str(), handover_out, seq.c_str());
    if (!sc.code.empty()) {
      ck(sc.name + ": the resource request reported, right colours, before the handover",
        resource_t >= 0.0 && resource_t <= guided_t &&      // the same tick: report, then GUIDED
        resource_body == dk::resourceRequestJson(dk::requestFrom("code", sc.code)));
    }
    ck(sc.name + ": Task 3 ENDED after the fire and asked for GUIDED for Task 1",
      guided_t >= 0.0 && fire_report_t >= 0.0 && guided_t > fire_report_t);
    if (!sc.t4_after_hit) {
      ck(sc.name + ": left the dock from the back-off (" + std::to_string(handover_out).substr(0, 4) +
        " m, want " + std::to_string(sc.backoff_m).substr(0, 3) + " +-0.5)",
        handover_out >= sc.backoff_m - 0.5 && handover_out <= sc.backoff_m + 0.5);
    }
    ck(sc.name + ": firefighting reported once", countReports(reports, "firefighting") == 1);
    ck(sc.name + ": current_task Task 3, then NONE, then Task 1",
      tasks.size() >= 3 && tasks[0] == "TASK_COORDINATED_LOGISTICS" && tasks[1] == "TASK_NONE" &&
      tasks[2] == "TASK_SAFE_PASSAGE");
    ck(sc.name + ": the docking report names bay " + std::to_string(sc.expect_bay),
      reports.find("docking " + dk::dockingReportJson(sc.expect_bay)) != std::string::npos);
  }
  if (sc.t4_alert_s >= 0.0) {
    const int acks = countReports(reports, "incident_ack");
    std::printf("   Task 4: %d alerts, %d acknowledged, %d break-offs; the virtual boat came no "
      "closer than %.1f m (at %.0f s)\n", ctx->task4.alerts, acks, ctx->task4.evasions, t4_min,
      t4_min_t);
    ck(sc.name + ": Task 4 acknowledged once, command_seq 4217 (not the re-delivery or the update)",
      acks == 1 && reports.find("incident_ack " + t4::incidentAckJson(4217.0)) != std::string::npos);
    ck(sc.name + ": broke off for the virtual boat", ctx->task4.evasions >= 1);
    ck(sc.name + ": never within the virtual boat's 10 m (closest " +
      std::to_string(t4_min).substr(0, 4) + " m)", t4_min >= 10.0);
  }
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
  {
    Scenario s = all.back();                     // the fire, then t3_backoff_m (4 m default)
    s.name = "scan_fire_backoff_4m_bay2";
    s.fire = true;
    s.seed = 11;
    all.push_back(s);
  }
  {
    Scenario s = all.back();                     // the GCS's "back off" at 3 m, the right window
    s.name = "scan_fire_backoff_3m_bay1";
    s.colours = {C::Green, C::Red, C::Red};
    s.expect_bay = 1;
    s.backoff_m = 3.0;
    s.lit_slot = 1;
    s.seed = 13;
    all.push_back(s);
  }
  {
    Scenario s = all.back();                     // Task 3 first: it must END after the fire
    s.name = "mission_t3_t4_t1_handover_bay1";   // and hand the boat to Task 1 (GUIDED)
    s.tree = "mission_t3_t4_t1.xml";
    s.backoff_m = 4.0;
    s.until_handover = true;
    s.code = {C::Blue, C::Green};                // a BLUE tin to the GREEN circle
    s.seed = 17;
    all.push_back(s);
  }
  {
    Scenario s = all.back();                     // ... and a Task 4 virtual boat crossing the dock
    s.name = "mission_t3_t4_t1_virtual_boat";    // front while it docks: break off, come back,
    s.t4_alert_s = 25.0;                         // then dock, fire and hand over all the same
    s.guided_moves = true;
    all.push_back(s);
  }
  {
    Scenario s = all.back();                     // ... and the virtual boat just after the fire:
    s.name = "mission_t3_t4_t1_virtual_boat_after_fire";   // back, no second RED waited for,
    s.t4_alert_s = 1.0;                          // the code read, one firefighting report
    s.t4_after_hit = true;
    s.code_s = 400.0;    // the decode's 80 s timeout is wall clock, never reached here: the code
                         // stays on for the restart to read (at the competition it ends 60 s after
                         // the hit, and a restart after that gives up on it after 80 s)
    all.push_back(s);
  }
  // 2026-11-13 on the water: the search slid LEFT at 100 us (~0.3 m/s), and the approach's
  // corrections (30 us + 100 us/m) mostly fell inside the FC's 30 us RC dead zone, so the slide
  // carried the boat left of the GREEN bay and it went in against the left finger. This world
  // has that dead zone, a sway that makes 100 us ~0.3 m/s as on the water, and a slide that glides.
  {
    Scenario s;
    s.name = "left_slide_old_gains";
    s.tree = "task3_scan_fire.xml";
    s.colours = {C::Red, C::Green, C::Red};
    s.expect_bay = 2;
    s.right_m = 6.0;
    s.bow_off_deg = 10.0;
    s.fc_dz_us = 30.0;
    s.lat_mps = 0.5;
    s.sway_tau_s = 2.5;                          // the hull glides on once the stick is off
    s.seed = 21;
    s.baseline = true;                           // the gains the boat ran that morning
    s.t3 = {{"t3_search_lat_us", 100.0}, {"t3_app_kp_lat", 100.0}, {"t3_app_kd_lat", 40.0},
      {"t3_app_min_us", 30.0}, {"t3_app_centre_tol_m", 0.15}, {"t3_app_centred_s", 1.0},
      {"t3_app_recentre_m", 0.40}, {"t3_hold_kp_lat", 100.0}, {"t3_hold_ki_lat", 10.0},
      {"t3_hold_kd_yaw", 0.5}, {"t3_hold_min_us", 30.0}};
    all.push_back(s);
  }
  {
    Scenario s = all.back();                     // the same water with the retuned defaults
    s.name = "left_slide_retuned";
    s.baseline = false;
    s.t3.clear();
    all.push_back(s);
  }
  {
    Scenario s = all.back();                     // the Run panel's docking position: 2.5 m off
    s.name = "left_slide_hold_2p5m_0p2m_left";   // the face, 0.2 m left of the bay's centre
    s.t3 = {{"t3_hold_m", 2.5}, {"t3_lat_offset_m", 0.2}, {"t3_handover_m", 2.8}};
    s.seed = 23;
    all.push_back(s);
  }
  {
    Scenario s = all.back();                     // a harder glide and a stronger set: the slide
    s.name = "left_slide_retuned_long_glide";    // carries well past the bay before it commits
    s.t3.clear();
    s.sway_tau_s = 4.0;
    s.current_mps = 0.04;
    s.seed = 29;
    all.push_back(s);
  }
  for (const auto & s : all) {run(s, trace_dir);}
  rclcpp::shutdown();
  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  std::printf("%s\n", g_fails == 0 ? "PASS" : "FAIL");
  return g_fails == 0 ? 0 : 1;
}
