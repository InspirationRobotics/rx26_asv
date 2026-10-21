// tree_view.hpp — the tree's shape with every node's status: a log rendering reprinted only on
// change (BT.CPP's own logger prints one line per transition), and JSON for /crsd/bt_status.
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

  /// The tree as JSON, for a GUI to draw. Structured rather than rendered text: a
  /// viewer that parses indentation breaks when a node is renamed and cannot style
  /// a node by its KIND. `kind` is the BT.CPP family (control, decorator or leaf).
  /// Every node as a JSON array of {d, name, type, kind, status}, depth-first.
  std::string nodesJson()
  {
    std::string out = "[";
    bool first = true;
    jsonWalk(root_, 0, out, first);
    return out + "]";
  }

  /// The deepest RUNNING node's name ("" when nothing runs): where the mission is now.
  std::string runningNode()
  {
    std::string best;
    int best_depth = -1;
    deepest(root_, 0, best, best_depth);
    return best;
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
    // BT.CPP reports CONDITION separately from ACTION: a condition only answers a
    // question, an action changes something, and a viewer should show which.
    const char * kind = "action";
    auto * ctrl = dynamic_cast<BT::ControlNode *>(node);
    auto * dec = dynamic_cast<BT::DecoratorNode *>(node);
    if (ctrl) {
      kind = "control";
    } else if (dec) {
      kind = "decorator";
    } else if (node->type() == BT::NodeType::CONDITION) {
      kind = "condition";
    }

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

  static void deepest(BT::TreeNode * node, int depth, std::string & best, int & best_depth)
  {
    if (node == nullptr || node->status() != BT::NodeStatus::RUNNING) {return;}
    if (depth > best_depth) {
      best_depth = depth;
      best = node->name();
    }
    if (auto * ctrl = dynamic_cast<BT::ControlNode *>(node)) {
      for (auto * c : ctrl->children()) {deepest(c, depth + 1, best, best_depth);}
    } else if (auto * dec = dynamic_cast<BT::DecoratorNode *>(node)) {
      deepest(dec->child(), depth + 1, best, best_depth);
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
