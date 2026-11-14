// test_mission_nodes — WaitForRunStart, TaskAttempt and PassagePlanFresh, ticked against a Context;
// FailureTrace on the Task 3 guards; SetCameraModel; the cannon beside the docking and the
// centring on the GREEN indicator; the bay number the docking report gives RoboCommand.
//
// Links the leaves, so it needs ROS: build with colcon, run from the build directory
//
//     ./build/crusader_bt/test_mission_nodes
//
// A scripted child stands in for a task; the Context's clock (now_s) is moved by hand.
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "crusader_bt/context.hpp"
#include "crusader_bt/tree_view.hpp"

using crusader_bt::Context;
using crusader_bt::ContextPtr;

static int g_fails = 0;
static int g_checks = 0;

static void chk(const std::string & name, bool pass)
{
  ++g_checks;
  if (!pass) {++g_fails;}
  std::printf("  [%s] %s\n", pass ? "ok" : "FAIL", name.c_str());
}

/// A leaf whose result the test chooses; RUNNING until told otherwise.
static BT::NodeStatus g_child = BT::NodeStatus::RUNNING;
static int g_child_starts = 0;

class Scripted : public BT::StatefulActionNode
{
public:
  Scripted(const std::string & n, const BT::NodeConfig & c)
  : BT::StatefulActionNode(n, c) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override {++g_child_starts; return g_child;}
  BT::NodeStatus onRunning() override {return g_child;}
  void onHalted() override {}
};

/// A Context with the outputs recorded: every current_task, and every setpoint sent.
struct Rig
{
  ContextPtr ctx = std::make_shared<Context>();
  std::vector<std::string> tasks;
  int setpoints = 0;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr bb = BT::Blackboard::create();
  BT::Tree tree;

  Rig()
  {
    ctx->set_task = [this](const std::string & t) {tasks.push_back(t);};
    ctx->send_setpoint = [this](crusader_bt::nav::LatLon) {++setpoints;};
    ctx->publish_setpoints = true;
    crusader_bt::registerCrusaderNodes(factory);
    factory.registerNodeType<Scripted>("Scripted");
    bb->set("ctx", ctx);
  }

  void load(const std::string & body)
  {
    tree = factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\">" + body + "</BehaviorTree></root>", bb);
  }

  BT::NodeStatus tick() {return tree.tickOnce();}

  /// One tick with Ekko's field `age_s` old (Context::kNoPlanAgeS = none, or cleared).
  BT::NodeStatus tickWithPlanAge(double age_s)
  {
    ctx->plan_age_s = age_s;
    return tick();
  }

  /// The boat has a pose, so a hold can be sent.
  void withPose()
  {
    ctx->pose_fresh = true;
    ctx->origin_set = true;
  }

  std::string taskList() const
  {
    std::string s;
    for (const auto & t : tasks) {s += (s.empty() ? "" : ",") + t;}
    return s;
  }
};

static const BT::NodeStatus RUN = BT::NodeStatus::RUNNING;
static const BT::NodeStatus OK = BT::NodeStatus::SUCCESS;
static const BT::NodeStatus BAD = BT::NodeStatus::FAILURE;

static void taskAttemptTests()
{
  std::printf("1. TaskAttempt\n");
  {
    Rig r;
    r.load("<TaskAttempt token=\"TASK_SAFE_PASSAGE\"><Scripted/></TaskAttempt>");
    g_child = RUN;
    chk("RUNNING while the child runs", r.tick() == RUN);
    chk("the token goes out on the first tick", r.taskList() == "TASK_SAFE_PASSAGE");
    r.tick();
    r.tick();
    chk("and only once", r.taskList() == "TASK_SAFE_PASSAGE");
    g_child = OK;
    chk("SUCCESS passes through", r.tick() == OK);
    chk("TASK_NONE when the child succeeds", r.taskList() == "TASK_SAFE_PASSAGE,TASK_NONE");
    g_child = RUN;
    r.ctx->now_s += 2.0;
    r.tick();
    chk("a second run, after the dwell, announces the token again",
      r.taskList() == "TASK_SAFE_PASSAGE,TASK_NONE,TASK_SAFE_PASSAGE");
  }
  {
    Rig r;
    r.load("<TaskAttempt token=\"TASK_COORDINATED_LOGISTICS\"><Scripted/></TaskAttempt>");
    g_child = BAD;
    chk("FAILURE passes through", r.tick() == BAD);
    chk("TASK_NONE when the child fails, after the token",
      r.taskList() == "TASK_COORDINATED_LOGISTICS,TASK_NONE");
  }
  {
    Rig r;
    r.load("<TaskAttempt token=\"TASK_SAFE_PASSAGE\"><Scripted/></TaskAttempt>");
    g_child = RUN;
    r.tick();
    r.tree.haltTree();
    chk("TASK_NONE when halted mid-attempt", r.taskList() == "TASK_SAFE_PASSAGE,TASK_NONE");
    r.tree.haltTree();
    chk("a second halt says nothing more", r.taskList() == "TASK_SAFE_PASSAGE,TASK_NONE");
  }
  {
    Rig r;
    r.load("<TaskAttempt token=\"TASK_SAFE_PASSAGE\"><Scripted/></TaskAttempt>");
    r.tree.haltTree();
    chk("halting an attempt that never started announces nothing", r.tasks.empty());
  }
  {
    Rig r;
    r.load("<ReactiveSequence><IsAutonomous/>"
      "<TaskAttempt token=\"TASK_SAFE_PASSAGE\"><Scripted/></TaskAttempt></ReactiveSequence>");
    g_child = RUN;
    r.ctx->autonomous = true;
    r.tick();
    r.ctx->autonomous = false;
    chk("the pilot taking over fails the guard", r.tick() == BAD);
    chk("... and ends the attempt with TASK_NONE", r.taskList() == "TASK_SAFE_PASSAGE,TASK_NONE");
  }
  {
    Rig r;
    r.load("<Sequence><TaskAttempt token=\"TASK_SAFE_PASSAGE\" none_dwell_s=\"0.0\"><Scripted/></TaskAttempt>"
      "<TaskAttempt token=\"TASK_COORDINATED_LOGISTICS\" none_dwell_s=\"0.0\"><Scripted/></TaskAttempt>"
      "</Sequence>");
    g_child = OK;
    chk("none_dwell_s 0: two attempts finish in one tick", r.tick() == OK);
    chk("... each NONE -> task -> NONE", r.taskList() ==
      "TASK_SAFE_PASSAGE,TASK_NONE,TASK_COORDINATED_LOGISTICS,TASK_NONE");
  }
}

static void dwellTests()
{
  std::printf("\n1b. TaskAttempt holds TASK_NONE between attempts (the 2 Hz heartbeat must carry it)\n");
  const std::string two =
    "<Sequence><TaskAttempt token=\"TASK_SAFE_PASSAGE\"><Scripted/></TaskAttempt>"
    "<TaskAttempt token=\"TASK_COORDINATED_LOGISTICS\"><Scripted/></TaskAttempt></Sequence>";
  {
    Rig r;
    r.ctx->now_s = 100.0;
    r.load(two);
    g_child = OK;
    g_child_starts = 0;
    chk("the second attempt waits: RUNNING, not SUCCESS", r.tick() == RUN);
    chk("TASK_NONE is current, the next task is not claimed",
      r.taskList() == "TASK_SAFE_PASSAGE,TASK_NONE");
    chk("the held attempt's child was not ticked", g_child_starts == 1);
    r.ctx->now_s = 100.5;
    chk("still held at 0.5 s", r.tick() == RUN && r.taskList() == "TASK_SAFE_PASSAGE,TASK_NONE");
    r.ctx->now_s = 101.4;
    chk("still held at 1.4 s", r.tick() == RUN && r.taskList() == "TASK_SAFE_PASSAGE,TASK_NONE");
    r.ctx->now_s = 101.5;
    chk("claimed once 1.5 s have passed, and finishes", r.tick() == OK);
    chk("NONE, then the second task, then NONE", r.taskList() ==
      "TASK_SAFE_PASSAGE,TASK_NONE,TASK_COORDINATED_LOGISTICS,TASK_NONE");
    chk("the second child ran", g_child_starts == 2);
  }
  {
    Rig r;
    r.ctx->now_s = 100.0;
    r.load("<Sequence><TaskAttempt token=\"TASK_SAFE_PASSAGE\"><Scripted/></TaskAttempt>"
      "<TaskAttempt token=\"TASK_COORDINATED_LOGISTICS\" none_dwell_s=\"3.0\"><Scripted/></TaskAttempt>"
      "</Sequence>");
    g_child = OK;
    r.tick();
    r.ctx->now_s = 102.9;
    r.tick();
    chk("none_dwell_s is a port: 3 s holds past 1.5", r.taskList() == "TASK_SAFE_PASSAGE,TASK_NONE");
    r.ctx->now_s = 103.0;
    r.tick();
    chk("... and claims at 3 s", r.taskList().find("TASK_COORDINATED_LOGISTICS") != std::string::npos);
  }
  {
    Rig r;
    r.ctx->now_s = 100.0;
    r.load(two);
    g_child = OK;
    r.tick();
    r.tree.haltTree();
    chk("halting during the hold adds no second TASK_NONE", r.taskList() == "TASK_SAFE_PASSAGE,TASK_NONE");
  }
  {
    Rig r;
    r.ctx->now_s = 5.0;
    r.load("<TaskAttempt token=\"TASK_SAFE_PASSAGE\"><Scripted/></TaskAttempt>");
    g_child = RUN;
    r.tick();
    chk("the first attempt of a run claims at once (NONE was long ago)", r.taskList() == "TASK_SAFE_PASSAGE");
  }
}

static void planFreshTests()
{
  std::printf("\n2. PassagePlanFresh (Ekko's field keeps arriving)\n");
  {
    Rig r;
    r.load("<PassagePlanFresh max_age_s=\"15.0\"/>");
    chk("no plan yet (age blank) is not fresh", r.tick() == BAD);
    chk("a plan 0.4 s old is fresh", r.tickWithPlanAge(0.4) == OK);
    chk("... still, exactly at max_age_s", r.tickWithPlanAge(15.0) == OK);
    chk("older than max_age_s is stale", r.tickWithPlanAge(15.5) == BAD);
    chk("a cleared plan (age blank again) is not fresh",
      r.tickWithPlanAge(Context::kNoPlanAgeS) == BAD);
  }
  {
    Rig r;
    r.load("<PassagePlanFresh/>");
    chk("max_age_s defaults to 15 s: 14 s is fresh", r.tickWithPlanAge(14.0) == OK);
    chk("... and 16 s is not", r.tickWithPlanAge(16.0) == BAD);
  }
  {
    Rig r;
    r.load("<PassagePlanFresh max_age_s=\"3.0\"/>");
    chk("the port moves the limit", r.tickWithPlanAge(4.0) == BAD);
  }
  {
    // First in a ReactiveSequence guard band, it fails the attempt mid-run, not just at the start.
    Rig r;
    r.load("<ReactiveSequence><PassagePlanFresh max_age_s=\"15.0\"/><Scripted/></ReactiveSequence>");
    g_child = RUN;
    chk("the guarded child runs while the plan is fresh", r.tickWithPlanAge(1.0) == RUN);
    chk("the plan going stale fails the attempt on the next tick", r.tickWithPlanAge(20.0) == BAD);
  }
}

static void waitForRunStartTests()
{
  std::printf("\n3. WaitForRunStart\n");
  {
    Rig r;
    r.withPose();
    r.ctx->now_s = 100.0;
    r.load("<WaitForRunStart run_id=\"{rid}\"/>");
    chk("RUNNING with no RunStart", r.tick() == RUN);
    r.ctx->now_s = 250.0;
    chk("... however long it waits", r.tick() == RUN);
    chk("it held station once", r.setpoints == 1);
    r.ctx->run_start.set(251.0, 7);
    chk("SUCCESS once the RunStart arrives", r.tick() == OK);
    chk("run_id is out", r.bb->get<int>("rid") == 7);
    chk("RunStart latches across a restart of the node", [&] {
        r.tree.haltTree();
        return r.tick() == OK;
      }());
  }
  {
    Rig r;
    r.ctx->now_s = 1000.0;
    r.ctx->run_start.set(980.0, 3);
    r.load("<WaitForRunStart run_id=\"{rid}\"/>");
    chk("a RunStart 20 s before the start still counts (max_age_s 30)", r.tick() == OK);
    chk("with its id", r.bb->get<int>("rid") == 3);
  }
  {
    Rig r;
    r.ctx->now_s = 1000.0;
    r.ctx->run_start.set(50.0, 3);
    r.load("<WaitForRunStart/>");
    chk("a RunStart 950 s before the start is too old", r.tick() == RUN);
    r.load("<WaitForRunStart max_age_s=\"1000.0\"/>");
    chk("... unless max_age_s allows it, and an unbound run_id is no error", r.tick() == OK);
  }
  {
    // The runner's RunStart gate already took this run's RunStart (or the bench release): an old
    // saved mission that still has the leaf must not wait for it again.
    Rig r;
    r.withPose();
    r.ctx->now_s = 500.0;
    r.ctx->run_released = true;
    r.load("<WaitForRunStart run_id=\"{rid}\"/>");
    chk("run_released: SUCCESS at once with no RunStart latched", r.tick() == OK);
    chk("... without holding station", r.setpoints == 0);
    r.ctx->run_released = false;
    r.tree.haltTree();
    chk("run_released false again (next goal): waits as before", r.tick() == RUN);
  }
  {
    Rig r;
    r.ctx->now_s = 10.0;
    r.load("<WaitForRunStart timeout_s=\"5.0\"/>");
    chk("RUNNING inside the timeout", r.tick() == RUN);
    r.ctx->now_s = 14.0;
    chk("... still, at 4 s", r.tick() == RUN);
    r.ctx->now_s = 15.0;
    chk("FAILURE once timeout_s has passed", r.tick() == BAD);
  }
  {
    Rig r;
    r.ctx->now_s = 10.0;
    r.load("<WaitForRunStart hold=\"false\"/>");
    r.withPose();
    r.tick();
    chk("hold=false sends no setpoint", r.setpoints == 0);
  }
  {
    Rig r;
    r.load("<WaitForRunStart/>");
    r.tick();
    chk("no pose: nothing to hold yet", r.setpoints == 0);
    r.withPose();
    r.tick();
    r.tick();
    chk("the hold goes out once the pose is fresh, and only once", r.setpoints == 1);
  }
  {
    Rig r;
    r.ctx->publish_setpoints = false;
    r.withPose();
    r.load("<WaitForRunStart/>");
    r.tick();
    chk("publish_setpoints false: it cannot move the boat", r.setpoints == 0);
  }
}

/// The first orbit hop's distance from the anchor and the ring size, after one tick of `main`
/// (a Main tree that calls the subtree `Orbit`, which holds one CircleBuoy; nested: Orbit holds
/// a call to `OrbitBody` instead, with _autoremap alone, like the lib trees' common bodies).
static void orbitOf(Rig & r, const std::string & call_attrs, double * radius, int * hops,
  bool nested = false)
{
  r.ctx->nav.orbit_radius_m = 9.0;
  r.ctx->nav.orbit_points = 12;
  r.ctx->boat = crusader_bt::nav::Vec2{30.0, 0.0};
  r.ctx->origin = crusader_bt::nav::LatLon{0.0, 0.0};
  r.ctx->origin_set = true;
  r.ctx->pose_fresh = true;
  r.tree = r.factory.createTreeFromText(
    "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\">"
    "<BehaviorTree ID=\"Main\"><SubTree ID=\"Orbit\" _autoremap=\"true\" " + call_attrs + "/></BehaviorTree>"
    + std::string(nested ?
      "<BehaviorTree ID=\"Orbit\"><SubTree ID=\"OrbitBody\" _autoremap=\"true\"/></BehaviorTree>"
      "<BehaviorTree ID=\"OrbitBody\">" :
      "<BehaviorTree ID=\"Orbit\">") +
    "<CircleBuoy anchor=\"fix\" lat=\"0.0\" lon=\"0.0\" "
    "radius=\"{orbit_radius_m}\" points=\"{orbit_points}\"/></BehaviorTree></root>", r.bb);
  r.tick();
  *radius = crusader_bt::nav::norm(r.ctx->leg.goal);
  *hops = r.ctx->leg.hops;
}

static void tunableTests()
{
  std::printf("\n4. SubTree tunables reach the leaf, and omitting them keeps the nav_* fallback\n");
  double rad = 0.0;
  int hops = 0;
  Rig r;
  orbitOf(r, "orbit_radius_m=\"7.5\" orbit_points=\"5\"", &rad, &hops);
  const int hops5 = hops;
  chk("written values drive the leaf: radius 7.5", std::abs(rad - 7.5) < 0.05);
  Rig r2;
  orbitOf(r2, "orbit_radius_m=\"\" orbit_points=\"\"", &rad, &hops);
  chk("empty values fall back to nav_orbit_radius_m (9.0 here)", std::abs(rad - 9.0) < 0.05);
  chk("... and nav_orbit_points (12 here, not 5)", hops != hops5 && hops > hops5);
  Rig r3;
  orbitOf(r3, "", &rad, &hops);
  chk("absent values fall back the same way", std::abs(rad - 9.0) < 0.05 && hops > hops5);
  // The per-tier lib trees hand their common body only _autoremap: a value written on the
  // outer call must reach a leaf two SubTrees down, empty ones included.
  Rig r4;
  orbitOf(r4, "orbit_radius_m=\"7.5\" orbit_points=\"5\"", &rad, &hops, true);
  chk("two SubTrees down: written values still drive the leaf", std::abs(rad - 7.5) < 0.05 && hops == hops5);
  Rig r5;
  orbitOf(r5, "orbit_radius_m=\"\" orbit_points=\"\"", &rad, &hops, true);
  chk("... and empty ones still fall back", std::abs(rad - 9.0) < 0.05 && hops > hops5);
}

/// Task 3's scan_fire guards, cut down: the waypoint leg, then the MANUAL body. Reactive, as
/// in the trees: MANUAL is looked at on every tick, not only before the leg starts.
static const char * kScanFireGuards =
  "<Sequence name=\"scan_fire\">"
  "<ReactiveFallback name=\"to_the_waypoint\"><ModeIs mode=\"MANUAL\"/>"
  "<ReactiveSequence name=\"guided_leg\"><IsAutonomous/><NotDropped/><Scripted/></ReactiveSequence>"
  "</ReactiveFallback>"
  "<ReactiveSequence name=\"manual\"><ModeIs mode=\"MANUAL\"/><NotDropped/><Scripted/></ReactiveSequence>"
  "</Sequence>";

/// Ticks once through a FailureTrace and returns where it says the tree failed.
static std::string failedAt(Rig & r, BT::NodeStatus * st = nullptr)
{
  crusader_bt::FailureTrace trace(r.tree);
  trace.beginTick();
  const BT::NodeStatus s = r.tick();
  if (st) {*st = s;}
  return trace.text();
}

static void failureTraceTests()
{
  std::printf("\n5. FailureTrace names the node that failed the tree\n");
  BT::NodeStatus st = RUN;
  {
    Rig r;
    r.load(kScanFireGuards);
    r.ctx->mode = "MANUAL";
    r.ctx->drop_tripped = true;
    g_child = RUN;
    const std::string at = failedAt(r, &st);
    chk("MANUAL with SD latched: FAILURE on the first tick", st == BAD);
    chk("... at NotDropped in scan_fire > manual (got '" + at + "')",
      at == "NotDropped in scan_fire > manual");
  }
  {
    Rig r;
    r.load(kScanFireGuards);
    r.ctx->mode = "HOLD";
    r.ctx->autonomous = false;
    const std::string at = failedAt(r, &st);
    chk("HOLD: ModeIs fails first, but IsAutonomous is what failed the tree (got '" + at + "')",
      st == BAD && at == "IsAutonomous in scan_fire > to_the_waypoint > guided_leg");
  }
  {
    Rig r;
    r.load(std::string("<ReactiveSequence><IsAutonomous/><Sequence name=\"mission\">") +
      kScanFireGuards + "</Sequence></ReactiveSequence>");
    r.ctx->mode = "MANUAL";
    r.ctx->autonomous = false;
    const std::string at = failedAt(r, &st);
    chk("a Mission-tab copy in MANUAL fails at its own outer guard (got '" + at + "')",
      st == BAD && at == "IsAutonomous");
  }
  {
    Rig r;
    r.load(kScanFireGuards);
    r.ctx->mode = "MANUAL";
    r.ctx->drop_tripped = false;     // Context starts latched, as the bridge does
    g_child = RUN;
    crusader_bt::FailureTrace trace(r.tree);
    trace.beginTick();
    chk("SD safe: the body runs", r.tick() == RUN && trace.text().empty());
    r.ctx->drop_tripped = true;
    trace.beginTick();
    chk("SD flipped mid-run: the next tick fails at NotDropped",
      r.tick() == BAD && trace.text() == "NotDropped in scan_fire > manual");
  }
  {
    // 2026-11-12 water: the pilot took MANUAL during the GUIDED leg. A plain Fallback went on
    // ticking the running leg, its IsAutonomous failed the tree, and with no mission ticking
    // the boat reported STATE_MANUAL to RoboCommand.
    auto leg = [](Rig & r) {
        r.load(kScanFireGuards);
        r.ctx->mode = "GUIDED";
        r.ctx->autonomous = true;
        r.ctx->drop_tripped = false;
        g_child = RUN;
        g_child_starts = 0;
        return r.tick();
      };
    Rig r;
    chk("GUIDED: the waypoint leg runs", leg(r) == RUN && g_child_starts == 1);
    r.ctx->mode = "MANUAL";
    r.ctx->autonomous = false;
    crusader_bt::FailureTrace trace(r.tree);
    trace.beginTick();
    const BT::NodeStatus s = r.tick();
    chk("the pilot takes MANUAL mid-leg: still RUNNING, nothing failed (got '" + trace.text() +
      "')", s == RUN && trace.text().empty());
    chk("... the leg was dropped and the MANUAL body started", g_child_starts == 2);
    Rig q;
    leg(q);
    q.ctx->mode = "HOLD";
    q.ctx->autonomous = false;
    chk("HOLD mid-leg still ends the run", q.tick() == BAD);
  }
  {
    Rig r;
    r.load("<Sequence name=\"s\"><Inverter><AlwaysSuccess/></Inverter></Sequence>");
    const std::string at = failedAt(r, &st);
    chk("no leaf failed: the first node that did, the Inverter (got '" + at + "')",
      st == BAD && at == "Inverter in s");
  }
  {
    Rig r;
    r.load("<Sequence name=\"s\"><Fallback><AlwaysFailure name=\"first\"/><Scripted/></Fallback>"
      "</Sequence>");
    g_child = BAD;
    const std::string at = failedAt(r, &st);
    chk("a Fallback whose children all fail: the last one (got '" + at + "')",
      st == BAD && at == "Scripted in s");
  }

  std::printf("\n6. searchFirst: the leaf's port, else the GCS, else left\n");
  Context c;
  chk("nothing set: left", crusader_bt::searchFirst(c, "") == "left");
  c.search_first = "right";
  chk("the GCS's right", crusader_bt::searchFirst(c, "") == "right");
  chk("a port wins over the GCS", crusader_bt::searchFirst(c, "committed") == "committed");
}

/// A Rig whose camera_node answers through `up`: false = no parameter service.
struct CameraRig : Rig
{
  /// "model|indicator|weights" per ask (indicator and weights "" off the dock model; weights
  /// "" = the new detector, dock_ffcv_dir's best.pt)
  std::vector<std::string> asks;
  bool up = true;

  CameraRig()
  {
    ctx->set_camera_model = [this](const std::string & m, const std::string & i,
        const std::string & w, std::string & why) {
        if (!up) {why = "/camera_node has no parameter service"; return false;}
        asks.push_back(m + "|" + i + "|" + w);
        return true;
      };
  }

  /// A whole file's trees, main_tree_to_execute="Main": for SubTree ports.
  void loadFile(const std::string & trees)
  {
    tree = factory.createTreeFromText(
      "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\">" + trees + "</root>", bb);
  }

  /// A DockObservation arrives: what the dock model, running, publishes every frame.
  void observation()
  {
    ++ctx->dock_seq;
    ctx->dock_obs_age_s = 0.1;
  }
};

static void setCameraModelTests()
{
  std::printf("\n7. SetCameraModel\n");
  const std::string dock = "<SetCameraModel model=\"dock\" timeout_s=\"15\"/>";
  {
    CameraRig r;
    r.observation();
    r.load(dock);
    chk("dock frames already arriving: asks anyway (they may be the other indicator's)",
      r.tick() == RUN && r.asks.size() == 1 && r.asks[0] == "dock|yolo|");
    r.ctx->camera_model_ack = 1;
    chk("accepted: RUNNING until two frames since the answer", r.tick() == RUN);
    r.observation();
    chk("... one: still RUNNING (it may have been in flight at the switch)", r.tick() == RUN);
    r.observation();
    chk("... two: SUCCESS, asked once", r.tick() == OK && r.asks.size() == 1);
  }
  {
    CameraRig r;                                        // dock_obs_age_s: never (1e9)
    r.load(dock);
    chk("on buoy: asks camera_node for dock, indicator yolo (the default), RUNNING",
      r.tick() == RUN && r.asks.size() == 1 && r.asks[0] == "dock|yolo|");
    r.ctx->camera_model_ack = 1;
    r.ctx->now_s = 2.0;
    chk("accepted, no observation yet: still RUNNING", r.tick() == RUN);
    r.observation();
    r.observation();
    chk("two observations since: SUCCESS, asked once", r.tick() == OK && r.asks.size() == 1);
  }
  {
    CameraRig r;
    r.load("<SetCameraModel model=\"dock\" indicator=\"cv\" timeout_s=\"15\"/>");
    chk("indicator cv: asks for the dock model reading the indicator by cv",
      r.tick() == RUN && r.asks.size() == 1 && r.asks[0] == "dock|cv|");
    r.ctx->camera_model_ack = 1;
    r.tick();
    r.observation();
    r.observation();
    chk("... two frames after the answer: SUCCESS", r.tick() == OK);
  }
  {
    CameraRig r;
    r.load("<SetCameraModel model=\"dock\" indicator=\"cv\"/>");
    r.tick();
    r.ctx->camera_model_ack = -1;
    r.ctx->camera_model_why = "camera_node has no dock_indicator parameter (an older build)";
    chk("a camera_node that cannot read by cv: FAILURE, saying so", r.tick() == BAD &&
      r.ctx->camera_model_why.find("camera_node refused the dock model (indicator by cv, new weights): "
      "camera_node has no dock_indicator") == 0);
  }
  {
    CameraRig r;
    r.load("<SetCameraModel model=\"dock\" indicator=\"hsv\"/>");
    chk("indicator neither yolo nor cv: FAILURE, camera_node not asked", r.tick() == BAD &&
      r.asks.empty() && r.ctx->camera_model_why.find("indicator must be yolo or cv") == 0);
  }
  {
    CameraRig r;
    r.ctx->dock_obs_age_s = 3.0;                        // stale: older than the ask
    r.load(dock);
    r.tick();
    r.ctx->camera_model_ack = 1;
    chk("an observation from before the ask does not count", r.tick() == RUN);
  }
  {
    CameraRig r;
    r.load(dock);
    r.tick();
    r.ctx->camera_model_ack = 1;
    r.ctx->now_s = 15.5;
    chk("no observation 15.5 s after asking: FAILURE", r.tick() == BAD);
  }
  {
    CameraRig r;
    r.load(dock);
    r.tick();
    r.ctx->camera_model_ack = -1;
    r.ctx->camera_model_why = "model must be one of none, buoy, dock";
    chk("camera_node refuses: FAILURE, its reason kept for the GCS", r.tick() == BAD &&
      r.ctx->camera_model_why.find("camera_node refused the dock model (indicator by yolo, new weights): "
      "model must be") == 0);
  }
  {
    CameraRig r;
    r.up = false;
    r.load(dock);
    chk("camera_node not running: FAILURE on the first tick", r.tick() == BAD && r.asks.empty());
  }
  {
    Rig r;                                              // bt_runner never wired the hook
    r.load(dock);
    chk("no hook: FAILURE, not a crash", r.tick() == BAD);
  }
  {
    CameraRig r;
    r.observation();                                    // dock frames do not mean buoy
    r.load("<SetCameraModel model=\"buoy\" indicator=\"cv\"/>");
    chk("buoy: asks, even with dock frames arriving, and no indicator", r.tick() == RUN &&
      r.asks.size() == 1 && r.asks[0] == "buoy||");
    r.ctx->camera_model_ack = 1;
    chk("buoy accepted: SUCCESS", r.tick() == OK);
  }

  std::printf("\n7b. SetCameraModel: which dock detector (dock_weights)\n");
  const std::string old_pt = "/root/robotx_ws/models/ffcv_c1002_2026-10-03/best.pt";
  {
    CameraRig r;
    r.load("<SetCameraModel model=\"dock\" dock_weights=\"" + old_pt + "\"/>");
    chk("dock_weights named: asked for, with the indicator's default",
      r.tick() == RUN && r.asks.size() == 1 && r.asks[0] == "dock|yolo|" + old_pt);
    r.ctx->camera_model_ack = 1;
    r.tick();
    r.observation();
    r.observation();
    chk("... two frames after the answer (the reload done): SUCCESS", r.tick() == OK);
  }
  {
    CameraRig r;
    r.load("<SetCameraModel model=\"dock\" dock_weights=\"{dock_weights}\"/>");
    chk("dock_weights from a blackboard entry nobody set: the new detector (\"\")",
      r.tick() == RUN && r.asks.size() == 1 && r.asks[0] == "dock|yolo|");
  }
  {
    CameraRig r;
    r.load("<SetCameraModel model=\"buoy\" dock_weights=\"" + old_pt + "\"/>");
    chk("buoy: dock_weights not asked about", r.tick() == RUN && r.asks.size() == 1 &&
      r.asks[0] == "buoy||");
  }
  {
    CameraRig r;
    r.load("<SetCameraModel model=\"dock\" dock_weights=\"/nowhere/x.pt\"/>");
    r.tick();
    r.ctx->camera_model_ack = -1;
    r.ctx->camera_model_why = "dock_weights '/nowhere/x.pt' does not exist";
    chk("camera_node refuses the weights: FAILURE naming them", r.tick() == BAD &&
      r.ctx->camera_model_why.find("camera_node refused the dock model (indicator by yolo, "
      "weights /nowhere/x.pt): dock_weights '/nowhere/x.pt' does not exist") == 0);
  }
  // The trees: lib/task3_scan_fire.xml's leaf reads {dock_weights}; task3_scan_fire.xml passes
  // "" and task3_scan_fire_old_dock.xml the Oct 3 path, as SubTree attributes (_autoremap)
  const std::string body =
    "<BehaviorTree ID=\"Body\"><SetCameraModel model=\"dock\" indicator=\"yolo\" "
    "dock_weights=\"{dock_weights}\" timeout_s=\"15.0\"/></BehaviorTree>";
  {
    CameraRig r;
    r.loadFile(body + "<BehaviorTree ID=\"Main\"><SubTree ID=\"Body\" _autoremap=\"true\" "
      "dock_weights=\"" + old_pt + "\"/></BehaviorTree>");
    chk("old-dock wrapper: the SubTree's literal reaches the leaf",
      r.tick() == RUN && r.asks.size() == 1 && r.asks[0] == "dock|yolo|" + old_pt);
  }
  {
    CameraRig r;
    r.loadFile(body + "<BehaviorTree ID=\"Main\"><SubTree ID=\"Body\" _autoremap=\"true\" "
      "dock_weights=\"\"/></BehaviorTree>");
    chk("new wrapper (dock_weights=\"\"): the new detector",
      r.tick() == RUN && r.asks.size() == 1 && r.asks[0] == "dock|yolo|");
  }
  {
    CameraRig r;
    r.loadFile(body + "<BehaviorTree ID=\"Main\"><SubTree ID=\"Body\" _autoremap=\"true\"/>"
      "</BehaviorTree>");
    chk("a wrapper that does not set it: the new detector",
      r.tick() == RUN && r.asks.size() == 1 && r.asks[0] == "dock|yolo|");
  }
}

/// Task 3 on the sticks with the cannon: a boat at the origin heading north, and a camera
/// that sees one bay face straight ahead. Records the reports, the cannon and the sticks.
struct FireRig : Rig
{
  std::vector<std::string> reports;
  std::vector<std::string> bodies;               ///< each report's body_json, in step
  int cannon_calls = 0;
  bool cannon_on = false;
  double cannon_x = 0.0, cannon_y = 0.0;
  double fwd = 0.0, lat = 0.0, yaw = 0.0;

  FireRig()
  {
    ctx->report = [this](const std::string & kind, const std::string & body) {
        reports.push_back(kind);
        bodies.push_back(body);
      };
    ctx->cannon = [this](bool on, double x, double y, double) {
        ++cannon_calls; cannon_on = on; cannon_x = x; cannon_y = y;
      };
    ctx->sticks = [this](double f, double l, double y) {fwd = f; lat = l; yaw = y;};
    withPose();
    ctx->heading_deg = 0.0;
    ctx->dock_heading = 0.0;
  }

  static crusader_bt::dock::WindowSighting win(int i, crusader_bt::dock::Colour c,
    double x = 2.0)
  {
    crusader_bt::dock::WindowSighting w;
    w.index = i;
    w.state = c;
    w.has_position = true;
    w.x = x;
    w.y = 0.3 - 0.6 * i;
    w.z = 0.2;
    return w;
  }

  /// One DockObservation stamped t: the face `ahead` m in front of the camera, square to
  /// it, with these windows. The bay it makes reads GREEN, and is track 0, committed.
  void frame(double t, std::vector<crusader_bt::dock::WindowSighting> ws, double ahead = 2.0)
  {
    crusader_bt::dock::BaySighting s;
    s.bearing_deg = 0.0;
    s.has_normal = true;
    s.nx = -1.0;
    s.d = ahead;
    s.range_m = ahead;
    s.indicator_present = true;
    s.indicator = crusader_bt::dock::Colour::Green;
    s.indicator_conf = 1.0;
    s.windows = ws;
    crusader_bt::dock::Frame f;
    f.t = t;
    f.bays.push_back(s);
    std::lock_guard<std::mutex> lk(ctx->mu);
    crusader_bt::ingestDockObservation(*ctx, f);
    ctx->chosen_track = 0;
  }

  /// A frame placing one bay `ahead` m in front of the camera and `left` m left of it, its
  /// indicator reading `ind` (Unknown = none read).
  void place(double t, double ahead, double left = 0.0,
    crusader_bt::dock::Colour ind = crusader_bt::dock::Colour::Unknown)
  {
    crusader_bt::dock::BaySighting s;
    s.bearing_deg = std::atan2(left, ahead) / crusader_bt::nav::kDeg;
    s.has_normal = true;
    s.nx = -1.0;
    s.d = ahead;
    s.range_m = std::hypot(ahead, left);
    s.indicator_present = ind != crusader_bt::dock::Colour::Unknown;
    s.indicator = ind;
    s.indicator_conf = 1.0;
    crusader_bt::dock::Frame f;
    f.t = t;
    f.bays.push_back(s);
    std::lock_guard<std::mutex> lk(ctx->mu);
    crusader_bt::ingestDockObservation(*ctx, f);
  }

  /// The centring spot, seen at now_s: `left` m left of the bow's line, `ahead` m ahead.
  void spot(double left, double ahead)
  {
    ctx->chosen_track = 0;
    ctx->centre = crusader_bt::fire::CentreTarget{};
    ctx->centre.sight({-left, ahead}, ahead, ctx->now_s, ctx->now_s);
  }
};

/// task3_first_dock: no window of ours reads RED after docking -> spray one anyway.
static void firstDockNoRedTests()
{
  using crusader_bt::dock::Colour;
  const std::string await = "<AwaitFireTarget target=\"{target}\" min_red_s=\"0.2\" "
    "timeout_s=\"1000\" no_red_s=\"30\"/>";
  auto docked = [](FireRig & r, double report_s, double now_s) {
      r.ctx->docking_report_s = report_s;
      r.ctx->readiness_confirmed = true;                 // no re-sends in these
      r.ctx->now_s = now_s;
    };
  auto target = [](FireRig & r) {int t = -1; return r.bb->get("target", t) ? t : -1;};
  {
    FireRig r;
    r.frame(10.0, {FireRig::win(0, Colour::Off), FireRig::win(1, Colour::Off)});
    r.load(await);
    r.ctx->now_s = 100.0;
    chk("no_red_s, not docked yet: waits however long", r.tick() == RUN);
    r.ctx->now_s = 200.0;
    chk("... 100 s on, still", r.tick() == RUN);
    docked(r, 200.0, 200.0);
    r.tick();
    r.ctx->now_s = 229.0;
    chk("docked 29 s, no RED: still waiting", r.tick() == RUN);
    r.ctx->now_s = 230.5;
    chk("docked 30 s, no RED: FAILURE (the tree sprays one anyway)", r.tick() == BAD);
  }
  {
    FireRig r;
    docked(r, 50.0, 100.0);                              // docked before this wait began
    r.frame(10.0, {FireRig::win(0, Colour::Off)});
    r.load(await);
    chk("a retry, docked long ago: waits no_red_s from its own start", r.tick() == RUN);
    r.ctx->now_s = 129.0;
    chk("... 29 s in, still", r.tick() == RUN);
    r.ctx->now_s = 130.0;
    chk("... 30 s in, FAILURE", r.tick() == BAD);
  }
  {
    FireRig r;
    r.ctx->fires_out = 1;                                // a fire is out already
    docked(r, 50.0, 100.0);
    r.frame(10.0, {FireRig::win(0, Colour::Off)});
    r.load(await);
    r.tick();
    r.ctx->now_s = 500.0;
    chk("a fire already out: no_red_s never gives up", r.tick() == RUN);
  }
  {
    FireRig r;
    docked(r, 50.0, 100.0);
    r.frame(10.0, {FireRig::win(0, Colour::Off)});
    r.load(await);
    r.tick();
    r.ctx->now_s = 129.0;
    r.frame(10.1, {FireRig::win(0, Colour::Red)});
    r.tick();
    r.ctx->now_s = 129.5;
    r.frame(10.35, {FireRig::win(0, Colour::Red)});
    chk("RED just inside no_red_s: the fire, as before", r.tick() == OK && target(r) == 0);
  }
  {
    FireRig r;
    r.frame(10.0, {FireRig::win(0, Colour::Off), FireRig::win(1, Colour::Off)});
    r.load("<PickAnyWindow target=\"{target}\"/>");
    chk("PickAnyWindow: a window of our bay", r.tick() == OK && target(r) == 0);
    chk("... the other one on the next try", r.tick() == OK && target(r) == 1);
    chk("... then back", r.tick() == OK && target(r) == 0);
    FireRig q;
    q.frame(10.0, {});
    q.load("<PickAnyWindow target=\"{target}\"/>");
    chk("PickAnyWindow, no window seen: FAILURE", q.tick() == BAD);
  }
  {
    FireRig r;
    r.load("<SprayUntilHit target=\"0\" window=\"{window}\" timeout_s=\"100\" "
      "green_is_hit=\"true\"/>");
    r.frame(10.0, {FireRig::win(0, Colour::Off), FireRig::win(1, Colour::Off)});
    chk("green_is_hit: a window that never read RED is sprayed", r.tick() == RUN && r.cannon_on);
    r.frame(10.1, {FireRig::win(0, Colour::Green), FireRig::win(1, Colour::Off)});
    r.tick();
    r.frame(10.6, {FireRig::win(0, Colour::Green), FireRig::win(1, Colour::Off)});
    int window = -1;
    chk("... GREEN 0.5 s with no RED before it: the hit, cannon off", r.tick() == OK &&
      !r.cannon_on && r.bb->get("window", window) && window == 0);
    FireRig q;
    q.load("<SprayUntilHit target=\"0\" window=\"{window}\" timeout_s=\"100\"/>");
    q.frame(10.0, {FireRig::win(0, Colour::Off)});
    q.tick();
    q.frame(10.1, {FireRig::win(0, Colour::Green)});
    q.tick();
    q.frame(10.6, {FireRig::win(0, Colour::Green)});
    chk("without it, GREEN with no RED first is still not a hit", q.tick() == RUN);
  }
  {
    // the no-RED shot backs off by setting hold_m: StrafeKeep reads it every tick
    FireRig r;
    r.load("<Sequence><Script code=\"hold_m := 2.0\"/>"
      "<StrafeKeep fire_range_m=\"{hold_m}\" use_lidar=\"false\"/></Sequence>");
    r.spot(0.0, 1.7 + r.ctx->cam_mount.x);                 // docked: the camera 1.7 m off
    r.tick();
    chk("hold_m 2.0 with the camera 1.7 m off the face: backing off", r.fwd < 0.0);
    FireRig q;
    q.load("<Sequence><Script code=\"hold_m := 1.7\"/>"
      "<StrafeKeep fire_range_m=\"{hold_m}\" use_lidar=\"false\"/></Sequence>");
    q.spot(0.0, 1.7 + q.ctx->cam_mount.x);
    q.tick();
    chk("hold_m 1.7 there: on the spot, no push", q.fwd == 0.0);
  }
  {
    // the fire branch of task3_first_dock.xml: its red-else-any-window core
    FireRig r;
    docked(r, 0.0, 1.0);
    r.frame(10.0, {FireRig::win(0, Colour::Off), FireRig::win(1, Colour::Off)});
    r.load("<Fallback><Sequence>" + await +
      "<SprayUntilHit target=\"{target}\" window=\"{window}\" timeout_s=\"1000\"/></Sequence>"
      "<Sequence><PickAnyWindow target=\"{target}\"/>"
      "<SprayUntilHit target=\"{target}\" window=\"{window}\" timeout_s=\"15\" "
      "keep_after_docking=\"true\" green_is_hit=\"true\"/></Sequence></Fallback>");
    chk("fire branch: docked, no RED yet: waiting, water off", r.tick() == RUN && !r.cannon_on);
    r.ctx->now_s = 31.5;
    chk("... docked 30 s: spraying window 0 anyway", r.tick() == RUN && r.cannon_on &&
      target(r) == 0);
    r.frame(10.1, {FireRig::win(0, Colour::Green), FireRig::win(1, Colour::Off)});
    r.tick();
    r.frame(10.6, {FireRig::win(0, Colour::Green), FireRig::win(1, Colour::Off)});
    int window = -1;
    chk("... it reads GREEN: put out (the reports follow), water off", r.tick() == OK &&
      !r.cannon_on && r.bb->get("window", window) && window == 0 && r.ctx->fires_out == 1);
  }
}

/// The bay number the docking report names: RoboCommand lights a window in THAT bay
/// (rx_reports.proto), bays 1-3 left to right facing them. The boat at the origin heading
/// north, the dock 4 m ahead facing it.
static void bayNumberTests()
{
  using crusader_bt::dock::Colour;
  std::printf("\n9. Task 3: the bay number RoboCommand lights a window in\n");
  // six sightings of a bay 4 m ahead, `left` m left of the bow's line, from t on
  auto seen = [](FireRig & r, double t, double left, Colour ind) {
      for (int i = 0; i < 6; ++i) {r.place(t + 0.1 * i, 4.0, left, ind);}
    };
  // CommitFirstGreen as task3_scan_fire(_cv) has it: the number committed to, -1 for none
  auto commit = [](FireRig & r) {
      r.load("<CommitFirstGreen bay=\"{bay}\" min_reads=\"4\" min_conf=\"0.5\" "
        "min_obs=\"4\" max_turn_deg=\"15\"/>");
      int b = -1;
      return r.tick() == OK && r.bb->get("bay", b) ? b : -1;
    };
  {
    FireRig r;
    r.ctx->search_dir = -1;
    seen(r, 1.0, 0.0, Colour::Green);
    chk("a lone GREEN bay sliding left (from the dock's right end): bay 3", commit(r) == 3);
    FireRig q;
    q.ctx->search_dir = 1;
    seen(q, 1.0, 0.0, Colour::Green);
    chk("... sliding right (from the left end): bay 1", commit(q) == 1);
  }
  {
    FireRig r;                                              // StrafeSearch not run yet
    seen(r, 1.0, 0.0, Colour::Green);
    chk("no slide yet, the GCS's slide-first unset (left): bay 3", commit(r) == 3);
    FireRig q;
    q.ctx->search_first = "right";
    seen(q, 1.0, 0.0, Colour::Green);
    chk("... set to right: bay 1", commit(q) == 1);
  }
  {
    FireRig r;
    r.ctx->search_dir = -1;
    seen(r, 1.0, -2.0, Colour::Red);                        // slid past a RED bay on its right
    seen(r, 2.0, 0.0, Colour::Green);
    chk("sliding left, one bay seen right of ours: bay 2", commit(r) == 2);
    FireRig q;
    q.ctx->search_dir = 1;
    seen(q, 1.0, 2.0, Colour::Red);                         // sliding right: on its left
    seen(q, 2.0, 0.0, Colour::Green);
    chk("sliding right, one bay seen left of ours: bay 2", commit(q) == 2);
  }
  {
    FireRig r;
    r.ctx->search_dir = 1;
    seen(r, 1.0, -2.0, Colour::Red);
    seen(r, 2.0, -4.0, Colour::Red);
    seen(r, 3.0, 0.0, Colour::Green);
    chk("all three bays known: the layout's number (ours leftmost: 1)", commit(r) == 1);
  }
  {
    // committed on a guess; every bay known by the docking report: the report, and its
    // re-sends, name the layout's bay, not the tree's {bay}
    FireRig r;
    r.ctx->search_dir = -1;
    seen(r, 1.0, 0.0, Colour::Green);
    chk("committed alone sliding left: bay 3, a guess", commit(r) == 3);
    seen(r, 2.0, -2.0, Colour::Red);                        // closing in: two bays on its RIGHT
    seen(r, 3.0, -4.0, Colour::Red);
    r.load("<ReportDocking bay=\"{bay}\"/>");
    chk("the docking report names bay 1, the layout's, and keeps it", r.tick() == OK &&
      r.bodies.size() == 1 && r.bodies[0] == "{\"bay_id\":1}" && r.ctx->chosen_bay == 1);
    r.load("<AwaitFireTarget bay=\"{bay}\" target=\"{target}\" resend_s=\"10\" "
      "timeout_s=\"1000\"/>");
    r.ctx->now_s += 10.5;                                   // no confirmation from RoboCommand
    chk("... its re-send names bay 1 too ({bay} still says 3)", r.tick() == RUN &&
      r.bodies.size() == 2 && r.bodies[1] == "{\"bay_id\":1}");
    FireRig q;
    q.ctx->search_dir = -1;
    seen(q, 1.0, 0.0, Colour::Green);
    commit(q);
    q.load("<ReportDocking bay=\"{bay}\"/>");
    chk("still alone at the docking report: the guess goes out", q.tick() == OK &&
      q.bodies.size() == 1 && q.bodies[0] == "{\"bay_id\":3}");
  }
}

static void fireBesideDockingTests()
{
  using crusader_bt::dock::Colour;
  std::printf("\n8. Task 3: the cannon beside the docking, and the centring\n");
  {
    FireRig r;
    r.frame(10.0, {});
    r.load("<AwaitFireTarget bay=\"2\" target=\"{target}\" min_red_s=\"0.2\" timeout_s=\"100\"/>");
    chk("no light: RUNNING", r.tick() == RUN);
    r.frame(10.0, {FireRig::win(0, Colour::Off), FireRig::win(1, Colour::Red)});
    chk("RED in one frame: not yet (0.2 s)", r.tick() == RUN);
    r.frame(10.1, {FireRig::win(1, Colour::Red)});
    r.tick();
    r.frame(10.25, {FireRig::win(0, Colour::Off), FireRig::win(1, Colour::Red)});
    chk("our bay's window RED for 0.25 s: SUCCESS, docked or not", r.tick() == OK);
    int target = -1;
    chk("... and the target is that window", r.bb->get("target", target) && target == 1);
  }
  {
    FireRig r;
    r.frame(10.0, {});
    r.load("<AwaitFireTarget bay=\"2\" min_red_s=\"0.2\" timeout_s=\"1000\" resend_s=\"10\"/>");
    r.tick();
    r.ctx->now_s = 30.0;
    r.tick();
    chk("beside a docking still under way: no docking report", r.reports.empty());
    r.ctx->docking_report_s = 30.0;                       // ReportDocking sent it
    r.ctx->now_s = 35.0;
    r.tick();
    chk("5 s after ReportDocking: not yet re-sent", r.reports.empty());
    r.ctx->now_s = 40.5;
    r.tick();
    chk("10 s after it, unconfirmed: re-sent once", r.reports.size() == 1 &&
      r.reports[0] == "docking");
    r.ctx->now_s = 45.0;
    r.tick();
    chk("... and not again 4.5 s later", r.reports.size() == 1);
    r.ctx->readiness_confirmed = true;
    r.ctx->now_s = 60.0;
    r.tick();
    chk("confirmed: no more", r.reports.size() == 1);
  }
  {
    FireRig r;
    r.ctx->fires_out = 1;                                  // the code flashes after a hit
    r.frame(10.0, {});
    r.load("<AwaitFireTarget target=\"{target}\" min_red_s=\"0.2\" refire_red_s=\"1.5\" "
      "timeout_s=\"100\"/>");
    double t = 10.0;
    for (; t < 10.95; t += 0.1) {r.frame(t, {FireRig::win(0, Colour::Red)}); r.tick();}
    r.frame(t, {FireRig::win(0, Colour::Off)});
    chk("after a fire: a 1 s RED flash is not another fire", r.tick() == RUN);
    bool early = false;
    for (t = 12.0; t < 13.45; t += 0.1) {
      r.frame(t, {FireRig::win(0, Colour::Red)});
      early = early || r.tick() != RUN;
    }
    chk("... nor is 1.4 s of it", !early);
    r.frame(13.6, {FireRig::win(0, Colour::Red)});
    chk("... a RED held 1.6 s is", r.tick() == OK);
  }
  {
    FireRig r;
    r.frame(10.0, {});
    r.ctx->dock_pattern = "steady";
    r.ctx->dock_colours = {Colour::Red};
    r.ctx->dock_target_window = 0;
    r.load("<AwaitFireTarget target=\"{target}\" timeout_s=\"100\"/>");
    int target = -1;
    chk("min_red_s unset: the timing layer's steady RED, as before",
      r.tick() == OK && r.bb->get("target", target) && target == 0);
  }
  {
    FireRig r;
    r.load("<SprayUntilHit target=\"1\" window=\"{window}\" timeout_s=\"100\" "
      "max_aim_age_s=\"5.0\"/>");
    r.ctx->now_s = 1.0;
    r.frame(10.0, {FireRig::win(0, Colour::Off), FireRig::win(1, Colour::Red, 2.0)});
    chk("RED, aim point seen: spraying it on the first tick", r.tick() == RUN && r.cannon_on &&
      r.cannon_x == 2.0);
    r.ctx->now_s = 3.0;
    r.frame(10.1, {FireRig::win(0, Colour::Off)});        // window 1 lost
    chk("window lost for 2 s: still spraying its last aim point", r.tick() == RUN &&
      r.cannon_on && r.cannon_x == 2.0);
    r.ctx->now_s = 5.9;
    chk("... for 4.9 s too", r.tick() == RUN && r.cannon_on);
    r.ctx->now_s = 6.2;
    chk("past max_aim_age_s (5 s): holding fire", r.tick() == RUN && !r.cannon_on);
    r.ctx->now_s = 6.3;
    r.frame(10.2, {FireRig::win(1, Colour::Red, 2.1)});
    chk("seen again: spraying again, at the new aim point", r.tick() == RUN && r.cannon_on &&
      r.cannon_x == 2.1);
    r.frame(10.3, {FireRig::win(1, Colour::Green, 2.1)});
    r.tick();
    r.frame(10.8, {FireRig::win(1, Colour::Green, 2.1)});
    int window = -1;
    chk("GREEN 0.5 s after RED: SUCCESS, cannon off", r.tick() == OK && !r.cannon_on &&
      r.bb->get("window", window) && window == 1);
    chk("... a fire out, counted", r.ctx->fires_out == 1);
  }
  {
    FireRig r;
    r.load("<SprayUntilHit target=\"1\" window=\"{window}\" timeout_s=\"100\" "
      "max_aim_age_s=\"5.0\" keep_after_docking=\"true\"/>");
    r.ctx->now_s = 1.0;
    r.frame(10.0, {FireRig::win(1, Colour::Red, 2.0)});
    r.tick();
    r.ctx->now_s = 6.2;
    r.frame(10.1, {FireRig::win(0, Colour::Off)});        // window 1 lost
    chk("keep_after_docking, not docked yet: holds fire past max_aim_age_s as before",
      r.tick() == RUN && !r.cannon_on);
    r.ctx->docking_report_s = 6.5;                         // ReportDocking sent it
    r.ctx->now_s = 7.0;
    chk("docked: sprays the lost window's last aim point again", r.tick() == RUN &&
      r.cannon_on && r.cannon_x == 2.0);
    r.ctx->now_s = 60.0;
    chk("... and 59 s after it was last seen, still", r.tick() == RUN && r.cannon_on &&
      r.cannon_x == 2.0);
    r.frame(10.2, {FireRig::win(1, Colour::Red, 2.4)});
    chk("seen again: re-aimed at the new point, the water never off", r.tick() == RUN &&
      r.cannon_on && r.cannon_x == 2.4);
  }
  {
    FireRig r;
    r.load("<SprayUntilHit target=\"0\" window=\"{window}\" timeout_s=\"100\"/>");
    double t = 20.0;
    for (; t < 21.95; t += 0.1) {
      r.frame(t, {FireRig::win(0, Colour::Off), FireRig::win(1, Colour::Red)});
      r.tick();
    }
    chk("window 1 RED, ours (0) dark for 1.9 s since the trigger: still spraying 0",
      r.cannon_on && r.cannon_y > 0.0);
    for (; t < 22.55; t += 0.1) {
      r.frame(t, {FireRig::win(0, Colour::Off), FireRig::win(1, Colour::Red)});
      r.tick();
    }
    r.frame(t, {FireRig::win(1, Colour::Green)});
    r.tick();
    r.frame(t + 0.5, {FireRig::win(1, Colour::Green)});
    int window = -1;
    chk("triggered on window 0 by a misread, window 1 RED for 2.5 s: it switches and puts "
      "1 out", r.tick() == OK && r.bb->get("window", window) && window == 1);
  }
  {
    FireRig r;
    r.load("<ApproachBay use_lidar=\"false\" handover_m=\"1.8\" recentre_m=\"0.40\"/>");
    r.spot(1.0, 1.5);                                      // inside 1.8 m, 1 m to the left
    r.tick();                                              // onStart
    r.ctx->now_s += 0.1;
    chk("inside the hand-over distance but 1 m off: RUNNING, not handed over",
      r.tick() == RUN);
    chk("... strafing LEFT to it (lat + = starboard)", r.lat < 0.0);
    r.ctx->now_s += 0.1;
    r.spot(-0.1, 1.5);
    chk("10 cm off: handed over", r.tick() == OK);
  }
  {
    FireRig r;
    r.load("<StrafeKeep fire_range_m=\"1.2\" use_lidar=\"false\"/>");
    r.spot(-0.8, 1.2);                                     // 0.8 m RIGHT, not in the picture
    r.tick();
    chk("bay out of the picture, its spot kept: strafing RIGHT to it (the 20:03 stall)",
      r.lat > 0.0);
    r.ctx->now_s += 9.0;
    r.tick();
    chk("9 s on, nothing seen: the spot has lapsed, no sideways push", r.lat == 0.0);
  }
  {
    using crusader_bt::dock::Colour;
    FireRig r;
    r.place(9.9, 2.0);
    chk("no GREEN read yet: an unread bay is not the GREEN one", r.ctx->green.t < 0.0);
    r.frame(10.0, {}, 2.0);
    chk("read GREEN: the GREEN bay", r.ctx->green.t == 10.0);
    r.place(10.1, 4.0);                                    // the same bay, ranged 2 m long
    chk("ranged 2 m long but in line: the same bay, one track (the gate is across the dock)",
      r.ctx->dock.tracks.size() == 1);
    chk("... and committed to it, it is still our bay",
      r.ctx->green.t == 10.1 && std::fabs(r.ctx->green.bay.ahead_m - 2.0) > 1.0);
    r.place(10.2, 4.0, -2.0);                              // 2 m to the right: the next bay
    chk("the next bay over, unread, is another bay: not taken",
      r.ctx->green.t == 10.1 && r.ctx->dock.tracks.size() == 2);
    r.place(10.3, 4.0, -2.0, Colour::Green);
    chk("... nor when it reads GREEN: committed, our track is followed, not a light",
      r.ctx->green.t == 10.1);
    r.place(10.4, 4.0, 0.9);                               // no two bays are 0.9 m apart
    chk("a face 0.9 m across is our bay again: merged into its track, and followed",
      r.ctx->green.t == 10.4 && r.ctx->dock.tracks.size() == 2);
    FireRig q;
    q.frame(10.0, {}, 2.0);
    q.place(10.1, 4.0, 0.0, Colour::Red);
    chk("in line but reading RED: still our bay (its colour is the book's votes)",
      q.ctx->green.t == 10.1);
  }
  {
    FireRig r;
    r.load("<ApproachBay use_lidar=\"false\" recentre_m=\"0.40\"/>");
    r.spot(0.05, 4.0);                                     // last seen on the line
    r.tick();
    r.ctx->now_s += 0.1;
    r.tick();
    r.ctx->now_s += 9.0;
    r.tick();
    chk("lost 5 cm off the line: holding, not a full slide on a coin toss", r.lat == 0.0);
    FireRig q;
    q.load("<ApproachBay use_lidar=\"false\" recentre_m=\"0.40\"/>");
    q.spot(1.0, 4.0);
    q.tick();
    q.ctx->now_s += 0.1;
    q.tick();
    q.ctx->now_s += 9.0;
    q.tick();
    chk("lost 1 m left of the line: still slides left to it", q.lat < 0.0);
  }
  {
    FireRig r;
    r.load("<StrafeKeep fire_range_m=\"1.5\" use_lidar=\"false\"/>");
    r.spot(0.0, 1.5 + r.ctx->cam_mount.x);
    r.tick();
    chk("range from the camera's lens, not the body origin",
      std::fabs(r.ctx->strafe_in.range_m - 1.5) < 1e-9);
  }
  firstDockNoRedTests();
}

/// RoboCommand's course as the OCS relays it (ocs_info.course, latched on /crsd/ocs_course):
/// the dock book places no face outside it.
static void courseFenceTests()
{
  std::printf("\n-- the course fence\n");
  namespace nav = crusader_bt::nav;
  // The OCS's own JSON for course ALPHA on 2026-11-12: a closed ring of five corners.
  const std::string alpha = "{\"course_id\": \"ALPHA\", \"pinger_freq_hz\": 0, \"corners\": ["
    "{\"latitude\": 1.2809887596341554, \"longitude\": 103.85481480992878}, "
    "{\"latitude\": 1.2808658372160693, \"longitude\": 103.85500721025717}, "
    "{\"latitude\": 1.2810900427153769, \"longitude\": 103.85515265929305}, "
    "{\"latitude\": 1.2812172480539488, \"longitude\": 103.8549604814802}, "
    "{\"latitude\": 1.2809887596341554, \"longitude\": 103.85481480992878}]}";
  std::string id;
  std::vector<nav::LatLon> corners;
  chk("the OCS's course parses: ALPHA, its 5 corners", crusader_bt::parseCourse(alpha, id,
    corners) && id == "ALPHA" && corners.size() == 5 &&
    std::fabs(corners[1].lat - 1.2808658372160693) < 1e-12 &&
    std::fabs(corners[1].lon - 103.85500721025717) < 1e-12);
  chk("no corners: no course", !crusader_bt::parseCourse("{\"course_id\": \"X\"}", id,
    corners) && corners.empty());
  chk("two corners: no course", !crusader_bt::parseCourse("{\"corners\": [{\"latitude\": 1, "
    "\"longitude\": 2}, {\"latitude\": 1, \"longitude\": 3}]}", id, corners) && corners.empty());

  // A 16 m square course around a boat heading north: 5 m ahead is in it, 9 m is 1.4 m
  // out (the camera is 0.37 m ahead of the body), inside the 3 m margin, and 13 m is not.
  auto fenced = [](FireRig & r) {
      for (const nav::Vec2 c : {nav::Vec2{-8, -8}, nav::Vec2{8, -8}, nav::Vec2{8, 8},
          nav::Vec2{-8, 8}})
      {
        r.ctx->course_corners.push_back(nav::toLatLon(c, r.ctx->origin));
      }
      ++r.ctx->course_seq;
    };
  {
    FireRig r;
    fenced(r);
    r.place(1.0, 5.0);
    r.place(2.0, 13.0);
    chk("a face inside the course is a bay; one 5 m beyond it is dropped",
      r.ctx->dock.tracks.size() == 1 && r.ctx->dock.fenced == 1);
    r.place(3.0, 9.0);
    chk("... one 1.4 m outside is kept: the dock may sit at the edge, its range long",
      r.ctx->dock.tracks.size() == 2 && r.ctx->dock.fenced == 1);
    crusader_bt::resetTask3(*r.ctx);                    // a new goal: a new book
    r.place(4.0, 13.0);
    chk("... and the new book is fenced too", r.ctx->dock.tracks.empty() &&
      r.ctx->dock.fenced == 1);
  }
  {
    FireRig r;                                          // the OCS sent no course
    r.place(1.0, 13.0);
    chk("no course: nothing is fenced", r.ctx->dock.tracks.size() == 1 &&
      r.ctx->dock.fenced == 0);
  }
}

int main()
{
  rclcpp::init(0, nullptr);
  taskAttemptTests();
  dwellTests();
  planFreshTests();
  waitForRunStartTests();
  tunableTests();
  failureTraceTests();
  setCameraModelTests();
  fireBesideDockingTests();
  bayNumberTests();
  courseFenceTests();
  rclcpp::shutdown();
  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  std::printf("%s\n", g_fails == 0 ? "PASS" : "FAIL");
  return g_fails == 0 ? 0 : 1;
}
