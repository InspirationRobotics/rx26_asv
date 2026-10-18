// bt_tree_check — does BehaviorTree.CPP accept these mission files?
//
//     ros2 run crusader_bt bt_tree_check <file.xml>...
//
// Loads each file as the runner does (every Crusader node registered, a Context on the
// blackboard as `ctx`) and prints `OK <file>` or `FAIL <file>: <error>`. Nothing is ticked.
// Exits non-zero if any file failed. It catches an unknown node, a bad port, a missing
// <include>, a SubTree without _autoremap, a malformed TreeNodesModel.
#include <cstdio>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "crusader_bt/context.hpp"

namespace
{

/// Empty when `path` builds into a tree, else why not.
std::string checkOne(const std::string & path)
{
  try {
    BT::BehaviorTreeFactory factory;
    crusader_bt::registerCrusaderNodes(factory);
    auto bb = BT::Blackboard::create();
    crusader_bt::ContextPtr ctx = std::make_shared<crusader_bt::Context>();
    bb->set("ctx", ctx);
    // Absolute: BT.CPP throws "cannot make absolute path" on a bare filename, and resolves
    // <include path="lib/x.xml"/> against the file's own directory.
    BT::Tree tree = factory.createTreeFromFile(std::filesystem::absolute(path).string(), bb);
    return "";
  } catch (const std::exception & e) {
    return e.what();
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  int failed = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string why = checkOne(argv[i]);
    if (why.empty()) {
      std::printf("OK %s\n", argv[i]);
    } else {
      std::printf("FAIL %s: %s\n", argv[i], why.c_str());
      ++failed;
    }
  }
  if (argc < 2) {
    std::printf("usage: bt_tree_check <file.xml>...\n");
    failed = 1;
  }
  rclcpp::shutdown();
  return failed == 0 ? 0 : 1;
}
