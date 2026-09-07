// tree_view.hpp — the whole tree, with every node's status, whenever it changes.
//
// BehaviorTree.CPP ships StdCoutLogger, which prints one line per TRANSITION:
//
//     NavigateTo   IDLE -> RUNNING
//     NavigateTo   RUNNING -> SUCCESS
//
// That is enough to debug a single leaf and useless for answering the question
// people actually ask, which is "where is it now, and what has it already
// done?". Reconstructing that from a scrolling transition log means holding the
// tree's shape in your head and replaying it. So this renders the SHAPE with the
// STATE on it, and reprints only when something changed:
//
//     [ ok ] guard_band            ReactiveSequence
//     [ ok ]   IsAutonomous
//     [ >> ]   tour                Sequence
//     [ ok ]     SetTask
//     [ ok ]     NavigateTo        target=fix
//     [ >> ]     NavigateTo        target=fix      <- here
//     [    ]     CircleBuoy        anchor=fix
//     [    ]     NavigateTo        target=home
//
// Groot2 draws exactly this, live and prettier, but only in the PRO build. This
// needs nothing, works over ssh, and goes in the log so a run can be read back
// afterwards.
#ifndef CRUSADER_BT__TREE_VIEW_HPP_
#define CRUSADER_BT__TREE_VIEW_HPP_

#include <cstdio>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/control_node.h"
#include "behaviortree_cpp/decorator_node.h"

namespace crusader_bt
{

/// Renders a tree's live status, and can tell you when it changed.
class TreeView
{
public:
  explicit TreeView(BT::Tree & tree)
  : root_(tree.rootNode()) {}

  /// The rendered tree, or "" if nothing has changed since the last call.
  ///
  /// Change is measured on the STATUS STRING, not on a tick counter: a tree
  /// ticking at 10 Hz with one leaf running is not news, and printing it 600
  /// times a minute would bury the transitions that matter.
  std::string renderIfChanged(bool colour = true)
  {
    std::string out;
    walk(root_, 0, colour, out);
    if (out == last_) {return {};}
    last_ = out;
    return out;
  }

  /// Same picture with no ANSI escapes, for a log file.
  std::string plain() {return render(false);}

  /// The tree as JSON, for a GUI to draw.
  ///
  /// Structured rather than the rendered text on purpose: a viewer that parses
  /// indentation back into a tree breaks the first time a node is renamed, and
  /// cannot style a node by its KIND because that information was thrown away
  /// in the rendering. `kind` is the BT.CPP family — control, decorator or
  /// leaf — which is what a drawing needs to pick a shape.
  std::string json(double elapsed_s)
  {
    std::string out = "{\"elapsed_s\":" + fmt(elapsed_s) + ",\"nodes\":[";
    bool first = true;
    jsonWalk(root_, 0, out, first);
    out += "]}";
    return out;
  }

  std::string render(bool colour = true)
  {
    std::string out;
    walk(root_, 0, colour, out);
    return out;
  }

private:
  static const char * glyph(BT::NodeStatus s, bool colour)
  {
    if (!colour) {
      switch (s) {
        case BT::NodeStatus::SUCCESS: return "[ ok ]";
        case BT::NodeStatus::FAILURE: return "[FAIL]";
        case BT::NodeStatus::RUNNING: return "[ >> ]";
        case BT::NodeStatus::SKIPPED: return "[skip]";
        default: return "[    ]";
      }
    }
    switch (s) {
      case BT::NodeStatus::SUCCESS: return "\033[32m[ ok ]\033[0m";
      case BT::NodeStatus::FAILURE: return "\033[31m[FAIL]\033[0m";
      case BT::NodeStatus::RUNNING: return "\033[33m[ >> ]\033[0m";
      case BT::NodeStatus::SKIPPED: return "\033[90m[skip]\033[0m";
      default: return "\033[90m[    ]\033[0m";
    }
  }

  static std::string fmt(double v)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f", v);
    return buf;
  }

  static const char * statusName(BT::NodeStatus s)
  {
    switch (s) {
      case BT::NodeStatus::SUCCESS: return "SUCCESS";
      case BT::NodeStatus::FAILURE: return "FAILURE";
      case BT::NodeStatus::RUNNING: return "RUNNING";
      case BT::NodeStatus::SKIPPED: return "SKIPPED";
      default: return "IDLE";
    }
  }

  static std::string esc(const std::string & s)
  {
    std::string o;
    for (char c : s) {
      if (c == '"' || c == '\\') {o += '\\';}
      o += c;
    }
    return o;
  }

  static void jsonWalk(BT::TreeNode * node, int depth, std::string & out, bool & first)
  {
    if (node == nullptr) {return;}
    const char * kind = "leaf";
    auto * ctrl = dynamic_cast<BT::ControlNode *>(node);
    auto * dec = dynamic_cast<BT::DecoratorNode *>(node);
    if (ctrl) {kind = "control";} else if (dec) {kind = "decorator";}

    if (!first) {out += ',';}
    first = false;
    out += "{\"d\":" + std::to_string(depth) +
      ",\"name\":\"" + esc(node->name()) +
      "\",\"type\":\"" + esc(node->registrationName()) +
      "\",\"kind\":\"" + kind +
      "\",\"status\":\"" + statusName(node->status()) + "\"}";

    if (ctrl) {
      for (auto * c : ctrl->children()) {jsonWalk(c, depth + 1, out, first);}
    } else if (dec) {
      jsonWalk(dec->child(), depth + 1, out, first);
    }
  }

  static void walk(BT::TreeNode * node, int depth, bool colour, std::string & out)
  {
    if (node == nullptr) {return;}

    out += glyph(node->status(), colour);
    out += ' ';
    out.append(static_cast<std::size_t>(depth) * 2, ' ');
    out += node->name();

    // The registration name only when it differs from the instance name, so a
    // leaf called "NavigateTo" is not printed as "NavigateTo NavigateTo" while
    // a composite named "tour" still shows that it is a Sequence.
    if (node->registrationName() != node->name()) {
      out += "  ";
      out += node->registrationName();
    }
    out += '\n';

    // Children, by node family. BT.CPP has no common children() on TreeNode,
    // so the cast is how the hierarchy is walked at all.
    if (auto * ctrl = dynamic_cast<BT::ControlNode *>(node)) {
      for (auto * c : ctrl->children()) {
        walk(c, depth + 1, colour, out);
      }
    } else if (auto * dec = dynamic_cast<BT::DecoratorNode *>(node)) {
      walk(dec->child(), depth + 1, colour, out);
    }
  }

  BT::TreeNode * root_ = nullptr;
  std::string last_;
};

}  // namespace crusader_bt

#endif  // CRUSADER_BT__TREE_VIEW_HPP_
