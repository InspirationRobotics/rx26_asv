// task4_leaves.cpp — Task 4 Disruptive (handbook 3.3.5): the AvoidInvaders guard.
//
// The geometry is task4_math.hpp (tested off the boat by test/test_task4_math.cpp). What runs at
// ALL times, whatever the mission, lives in bt_runner_node and context.hpp: every
// MovingObjectAlert is folded into ctx.task4 the moment it arrives, the objects' keep-outs swept
// along their projected travel are in knownHazards() (so the Task 1 planner re-plans round them
// and a guarded leg holds rather than cross one, waiting for it to pass without timing out), and
// /crsd/bt_status carries them to the GCS. This file is the part a TREE adds: getting OUT of the
// way when an object's projected travel is coming at the boat itself.
#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorator_node.h"

#include "crusader_bt/context.hpp"
#include "crusader_bt/task4_math.hpp"

namespace crusader_bt
{
namespace
{

using nav::Vec2;

/// AvoidInvaders: wraps a task. While no moving object threatens the boat it is invisible: it
/// ticks its child and returns what the child returns. When one does (t4::assess: the boat staying
/// where it is, the object's projected travel over t4_horizon_s comes inside its keep-out):
///   EVADING    the task is HALTED; the boat goes GUIDED (asked for from MANUAL when the task was
///              driving the sticks) and drives to t4::pickEscape's point: the nearest one the
///              object's projected travel keeps clear of, reached without crossing a buoy or the
///              dock or leaving the course. Re-picked when that point stops being safe.
///   WAITING    there, until the threat has been gone clear_s (outside the keep-out by
///              t4::Settings::clear_hyst_m: it ends clearly, not at the line).
///   RETURNING  (return_first) back to where the boat broke off, when that spot is clear. A
///              docking (MANUAL) waits for it to clear, up to return_wait_s: it cannot start
///              again from the escape point, where the bays are beyond the camera's reach.
///   RESTORING  the mode the task was in (MANUAL for a Task 3 docking), asked for again.
/// Then the task starts AGAIN from its first node, and every task tree here resumes from where the
/// mission is: Task 1 skips its search once a plan exists, re-plans from the boat and skips
/// finished phases; Task 3 keeps its bay and fires again only on a new RED.
/// Never FAILURE of its own. Sends nothing when publish_setpoints is false (it still says what it
/// would do); t4_enabled false = invisible.
class AvoidInvaders : public BT::DecoratorNode
{
public:
  AvoidInvaders(const std::string & n, const BT::NodeConfig & c)
  : BT::DecoratorNode(n, c), ctx_(contextFrom(c, n)) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("clear_s", 3.0, "the threat gone this long before the task resumes"),
      BT::InputPort<double>("arrive_m", 2.0, "at the escape or return point within this"),
      BT::InputPort<bool>("return_first", true,
        "drive back to where the task broke off before resuming it"),
      BT::InputPort<double>("return_wait_s", 120.0,
        "a docking (MANUAL) waits this long for where it broke off to clear, then resumes "
        "from where it is")};
  }

  BT::NodeStatus tick() override
  {
    const double clear_s = getInput<double>("clear_s").value_or(3.0);
    const double arrive_m = getInput<double>("arrive_m").value_or(2.0);
    const bool return_first = getInput<bool>("return_first").value_or(true);
    const double return_wait_s = getInput<double>("return_wait_s").value_or(120.0);

    t4::Settings s;
    t4::Assessment a;
    std::vector<t4::Invader> objs;
    Vec2 boat;
    double now = 0.0;
    bool fresh = false, publish = false;
    std::string mode;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      s = ctx_->task4.settings;
      objs = ctx_->task4.objects;
      boat = ctx_->boat;
      now = ctx_->now_s;
      fresh = ctx_->pose_fresh;
      mode = ctx_->mode;
      publish = ctx_->publish_setpoints;
      if (fresh) {a = t4::assess(objs, s, boat, now, phase_ != Phase::Clear);}
    }

    if (phase_ == Phase::Clear) {
      if (!a.threat) {
        report("CLEAR", objs.empty() ? "no moving object reported" : "no threat", a, now);
        return passThrough();
      }
      phase_ = Phase::Evading;
      resume_p_ = boat;
      resume_mode_ = mode;
      have_escape_ = false;
      asked_guided_ = false;
      waiting_back_ = false;
      clear_since_ = -1.0;
      picked_s_ = -1e9;
      {
        std::lock_guard<std::mutex> lk(ctx_->mu);
        ++ctx_->task4.evasions;
      }
      RCLCPP_WARN(log(), "Task 4: object %d is %.0f m off and will pass %.1f m from the boat in "
        "%.0f s (keep-out %.0f m): breaking off the task (%s) to get out of its way", a.id,
        a.dist_m, a.cpa_m, a.cpa_s, a.keepout_m, mode.c_str());
      haltChild();
      sent_ = false;
    }

    // how long the threat has been gone
    if (a.threat) {
      clear_since_ = -1.0;
    } else if (clear_since_ < 0.0) {
      clear_since_ = now;
    }
    const bool gone = !a.threat && now - clear_since_ >= clear_s;

    if (!fresh) {
      report(phaseName(), "pose stale: not commanding", a, now);
      return BT::NodeStatus::RUNNING;
    }
    if (a.threat && (phase_ == Phase::Returning || phase_ == Phase::Restoring)) {
      RCLCPP_WARN(log(), "Task 4: object %d threatens again: back to evading", a.id);
      phase_ = Phase::Evading;
      have_escape_ = false;
    }

    if ((phase_ == Phase::Evading || phase_ == Phase::Waiting) && gone) {
      t4::Assessment back;          // where the task broke off
      {
        std::lock_guard<std::mutex> lk(ctx_->mu);
        back = t4::assess(ctx_->task4.objects, s, resume_p_, now, true);
      }
      const bool back_clear = !back.threat;
      const bool away = return_first && nav::norm(resume_p_ - boat) > arrive_m;
      if (away && back_clear) {
        phase_ = Phase::Returning;
        sent_ = false;
        RCLCPP_INFO(log(), "Task 4: clear for %.0f s: back to where the task broke off (%.0f m)",
          clear_s, nav::norm(resume_p_ - boat));
      } else if (away && resume_mode_ == "MANUAL" && now - clear_since_ < clear_s + return_wait_s) {
        if (!waiting_back_) {       // stays WAITING at the escape point (below)
          waiting_back_ = true;
          RCLCPP_WARN(log(), "Task 4: clear here, but where the docking broke off is still in "
            "object %d's way: waiting up to %.0f s for it to clear, then back there", back.id,
            return_wait_s);
        }
      } else {
        phase_ = Phase::Restoring;
        restore_t0_ = now;
        RCLCPP_INFO(log(), "Task 4: clear for %.0f s%s", clear_s,
          return_first && !back_clear ? " (where it broke off is not clear yet: resuming from here)" : "");
      }
    }

    if (phase_ == Phase::Evading || phase_ == Phase::Waiting) {
      if (!ensureGuided(mode, publish, now)) {
        report(phaseName(), mode == "MANUAL" ? "asking for GUIDED to get out of the way" :
          "mode " + mode + ": cannot drive out of the way", a, now);
        return BT::NodeStatus::RUNNING;
      }
      pick(objs, s, boat, now);
      phase_ = nav::norm(escape_ - boat) <= arrive_m ? Phase::Waiting : Phase::Evading;
      drive(escape_, phase_ == Phase::Waiting ? "Task 4 wait at" : "Task 4 escape ->");
      report(phaseName(), phase_ == Phase::Waiting ?
        "out of the way; waiting for object " + std::to_string(a.id) + " to pass" :
        "getting out of object " + std::to_string(a.id) + "'s way" +
        (escape_safe_ ? "" : " (no point clear of it: the best there is)"), a, now, escape_);
      return BT::NodeStatus::RUNNING;
    }

    if (phase_ == Phase::Returning) {
      if (!ensureGuided(mode, publish, now)) {
        report("RETURNING", "mode " + mode + ": waiting for GUIDED", a, now);
        return BT::NodeStatus::RUNNING;
      }
      if (nav::norm(resume_p_ - boat) > arrive_m) {
        drive(resume_p_, "Task 4 return ->");
        report("RETURNING", "back to where the task broke off", a, now, resume_p_);
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::Restoring;
      restore_t0_ = now;
    }

    // RESTORING: the mode the task was in. A docking (MANUAL) asks for it back; 10 s at most.
    if (resume_mode_ == "MANUAL" && mode != "MANUAL" && now - restore_t0_ < 10.0) {
      if (publish && ctx_->request_mode && now - last_ask_ >= 0.5) {
        last_ask_ = now;
        ctx_->request_mode("MANUAL");
      }
      report("RESTORING", "asking for MANUAL again (the task was docking)", a, now);
      return BT::NodeStatus::RUNNING;
    }
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (mode == resume_mode_ || mode == "GUIDED") {ctx_->handed_mode = mode;}
      clearLeg(*ctx_);
    }
    RCLCPP_WARN(log(), "Task 4: resuming the task in %s (it starts again from its first step)",
      mode.c_str());
    phase_ = Phase::Clear;
    report("CLEAR", "resumed", a, now);
    return passThrough();
  }

  void halt() override
  {
    phase_ = Phase::Clear;
    DecoratorNode::halt();
  }

private:
  enum class Phase {Clear, Evading, Waiting, Returning, Restoring};

  rclcpp::Logger log() const {return ctx_->logger();}

  const char * phaseName() const
  {
    switch (phase_) {
      case Phase::Evading: return "EVADING";
      case Phase::Waiting: return "WAITING";
      case Phase::Returning: return "RETURNING";
      case Phase::Restoring: return "RESTORING";
      case Phase::Clear: break;
    }
    return "CLEAR";
  }

  BT::NodeStatus passThrough()
  {
    const BT::NodeStatus st = child_node_->executeTick();
    if (BT::isStatusCompleted(st)) {resetChild();}
    return st;
  }

  /// GUIDED: true. MANUAL (the task was driving the sticks): ask for GUIDED, false until it is.
  /// Any other mode, or MANUAL with the autonomy-drop latch set, is the pilot's: false, nothing
  /// asked (the tree's IsAutonomous manual="true" guard then ends the run).
  bool ensureGuided(const std::string & mode, bool publish, double now)
  {
    if (mode == "GUIDED") {
      if (asked_guided_) {
        std::lock_guard<std::mutex> lk(ctx_->mu);
        ctx_->handed_mode = "GUIDED";     // planned, not the pilot taking over
      }
      return true;
    }
    if (mode != "MANUAL") {return false;}
    {
      // the pilot dropped autonomy (SC): MANUAL is theirs now, never asked back from them
      std::lock_guard<std::mutex> lk(ctx_->mu);
      if (ctx_->drop_tripped) {return false;}
    }
    if (!asked_guided_) {
      asked_guided_ = true;
      RCLCPP_WARN(log(), "Task 4: MANUAL -> GUIDED to get out of the way%s",
        publish ? "" : " [shadow: not asking, publish_setpoints is false]");
    }
    if (publish && ctx_->request_mode && now - last_ask_ >= 0.5) {
      last_ask_ = now;
      ctx_->request_mode("GUIDED");
    }
    return false;
  }

  /// Where to go: keep the point while it stays safe (clear of every object's projected travel
  /// by the hysteresis, from arrival on), else re-pick (at most every 2 s). A point just outside
  /// a parked or slow object's line is not kept: the boat would wait there for ever.
  void pick(const std::vector<t4::Invader> & objs, const t4::Settings & s, Vec2 boat, double now)
  {
    if (have_escape_ && now - picked_s_ < 2.0) {return;}
    if (have_escape_ && escape_safe_ && t4::evalEscape(objs, s, boat, escape_, now).safe) {
      picked_s_ = now;
      return;
    }
    std::vector<path::Hazard> field;
    std::vector<Vec2> course;
    double hard = 0.8;
    {
      std::lock_guard<std::mutex> lk(ctx_->mu);
      for (const path::Hazard & h : knownHazards(*ctx_)) {
        if (h.source != path::HazardSource::Keepout) {field.push_back(h);}
      }
      hard = ctx_->nav.hard_m;
      if (ctx_->origin_set) {
        for (const nav::LatLon & ll : ctx_->course_corners) {
          course.push_back(nav::toLocal(ll, ctx_->origin));
        }
      }
    }
    picked_s_ = now;
    const t4::Escape e = t4::pickEscape(objs, s, boat, now, field, hard, course);
    if (!e.ok) {
      if (!have_escape_) {
        escape_ = boat;
        have_escape_ = true;
        escape_safe_ = false;
        RCLCPP_WARN(log(), "Task 4: no escape that crosses no buoy or dock and stays on the "
          "course: holding here");
      }
      return;
    }
    if (!have_escape_ || nav::norm(e.p - escape_) > 1.0 || e.safe != escape_safe_) {
      RCLCPP_WARN(log(), "Task 4: escape point %.1f m away, %s (%.1f m beyond the keep-out)",
        e.dist_m, e.safe ? "clear of its projected travel" : "the least bad there is", e.margin_m);
    }
    escape_ = e.p;
    escape_safe_ = e.safe;
    have_escape_ = true;
  }

  /// Send `p` as the GUIDED setpoint when it moved (a stream of setpoints is another mode to
  /// ArduRover), and show it as the running leg on the GCS map.
  void drive(Vec2 p, const char * what)
  {
    if (!sent_ || nav::norm(p - sent_p_) > 0.5) {
      sendSetpoint(*ctx_, log(), p, what);
      sent_ = true;
      sent_p_ = p;
    }
    std::lock_guard<std::mutex> lk(ctx_->mu);
    Context::LegStatus & l = ctx_->leg;
    l = Context::LegStatus{};
    l.leaf = "AvoidInvaders";
    l.name = phaseName();
    l.state = "FOLLOWING";
    l.why = "Task 4";
    l.have_goal = true;
    l.goal = p;
    l.have_target = true;
    l.target = p;
    l.path = {ctx_->boat, p};
    ++ctx_->leg_seq;
  }

  /// What the guard is doing, for /crsd/bt_status (the GCS); `target` = where it drives.
  void report(const std::string & state, const std::string & why, const t4::Assessment & a,
    double now)
  {
    setState(state, why, a, now, false, Vec2{});
  }

  void report(const std::string & state, const std::string & why, const t4::Assessment & a,
    double now, Vec2 target)
  {
    setState(state, why, a, now, true, target);
  }

  void setState(const std::string & state, const std::string & why, const t4::Assessment & a,
    double now, bool have_target, Vec2 target)
  {
    std::lock_guard<std::mutex> lk(ctx_->mu);
    Context::Task4 & k = ctx_->task4;
    k.state = state;
    k.why = why;
    k.state_s = now;
    k.last = a;
    k.have_target = have_target;
    k.target = target;
  }

  ContextPtr ctx_;
  Phase phase_ = Phase::Clear;
  Vec2 resume_p_, escape_, sent_p_;
  std::string resume_mode_;
  bool have_escape_ = false, escape_safe_ = false, asked_guided_ = false, sent_ = false;
  bool waiting_back_ = false;       ///< a docking's escape: said once that it waits to go back
  double clear_since_ = -1.0, picked_s_ = -1e9, restore_t0_ = 0.0, last_ask_ = -1e9;
};

}  // namespace

void registerTask4Nodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<AvoidInvaders>("AvoidInvaders");
}

}  // namespace crusader_bt
