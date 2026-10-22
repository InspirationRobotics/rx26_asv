// test_mission_nodes — WaitForRunStart, TaskAttempt and PassagePlanFresh, ticked against a Context.
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

int main()
{
  rclcpp::init(0, nullptr);
  taskAttemptTests();
  dwellTests();
  planFreshTests();
  waitForRunStartTests();
  tunableTests();
  rclcpp::shutdown();
  std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
  std::printf("%s\n", g_fails == 0 ? "PASS" : "FAIL");
  return g_fails == 0 ? 0 : 1;
}
