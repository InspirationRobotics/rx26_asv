// bt_tree_check — does BehaviorTree.CPP accept these mission files?
//
//     ros2 run crusader_bt bt_tree_check <file.xml>...          built-in trees: their <include>s resolve
//     ros2 run crusader_bt bt_tree_check --user <file.xml>...   user missions: the task library is preloaded
//     ros2 run crusader_bt bt_tree_check --models               the TreeNodesModel of every node
//
// Loads each file as the runner does (every Crusader node registered, a Context on the
// blackboard as `ctx`, the one loader of crusader_bt/mission_loader.hpp) and prints `OK <file>`
// or `FAIL <file>: <error>`. Nothing is ticked. Exits non-zero if any file failed. It catches
// an unknown node, a bad port, a missing <include>, a SubTree without _autoremap, an unknown
// SubTree ID, a malformed TreeNodesModel.
//
// --models prints BT.CPP's TreeNodesModel XML, built-ins included, and nothing else: the ground
// station reads it to learn which blocks and ports a mission may use.
#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "rclcpp/rclcpp.hpp"

#include "crusader_bt/context.hpp"
#include "crusader_bt/mission_loader.hpp"

namespace
{

/// Empty when `path` builds into a tree, else why not.
std::string checkOne(const std::string & path, bool user)
{
  try {
    BT::BehaviorTreeFactory factory;
    crusader_bt::registerCrusaderNodes(factory);
    auto bb = BT::Blackboard::create();
    crusader_bt::ContextPtr ctx = std::make_shared<crusader_bt::Context>();
    bb->set("ctx", ctx);
    // A user file needs the installed task library; a built-in names its own includes.
    const crusader_bt::MissionDirs dirs =
      user ? crusader_bt::missionDirs("") : crusader_bt::MissionDirs{};
    BT::Tree tree = crusader_bt::loadMission(factory, dirs, {path, user, ""}, bb);
    return "";
  } catch (const std::exception & e) {
    return e.what();
  }
}

std::string modelsXml()
{
  BT::BehaviorTreeFactory factory;
  crusader_bt::registerCrusaderNodes(factory);
  return BT::writeTreeNodesModelXML(factory, true);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  bool user = false, models = false;
  std::vector<std::string> files;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--user") {
      user = true;
    } else if (arg == "--models") {
      models = true;
    } else {
      files.push_back(arg);
    }
  }
  int failed = 0;
  if (models) {
    std::printf("%s\n", modelsXml().c_str());
  } else {
    for (const std::string & file : files) {
      const std::string why = checkOne(file, user);
      if (why.empty()) {
        std::printf("OK %s\n", file.c_str());
      } else {
        std::printf("FAIL %s: %s\n", file.c_str(), why.c_str());
        ++failed;
      }
    }
    if (files.empty()) {
      std::printf("usage: bt_tree_check [--user] <file.xml>...   |   bt_tree_check --models\n");
      failed = 1;
    }
  }
  rclcpp::shutdown();
  return failed == 0 ? 0 : 1;
}
