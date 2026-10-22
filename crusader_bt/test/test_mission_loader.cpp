// test_mission_loader — how a mission NAME becomes a tree (crusader_bt/mission_loader.hpp).
//
// Links the leaves, so it needs ROS: build with colcon and run from the build directory
//
//     ./build/crusader_bt/test_mission_loader
//
// The built-in trees are the source tree's behavior_trees/ (CRSD_TREES_DIR), the user's
// directory a temporary one, so no package has to be installed.
#include <gtest/gtest.h>

#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "crusader_bt/context.hpp"
#include "crusader_bt/mission_loader.hpp"

namespace fs = std::filesystem;
using crusader_bt::MissionDirs;
using crusader_bt::MissionError;
using crusader_bt::MissionFile;

namespace
{

// A mission as the ground station uploads it: the guard, then BODY. No <include>.
std::string mission(const std::string & body)
{
  return "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
         "<ReactiveSequence><IsAutonomous/><Sequence name=\"mission\">" + body +
         "</Sequence></ReactiveSequence></BehaviorTree></root>";
}

// The tunables of Task1Advanced / Task1Disruptive, as their built-in wrappers write them.
const char * kTask1Ports =
  "orbit_radius_m=\"\" orbit_min_m=\"1.5\" overshoot_deg=\"45.0\" lookahead_m=\"2.5\" "
  "blocked_timeout_s=\"15.0\" plan_attempts=\"4\"";
const char * kTask3Ports =
  "vantage_standoff_m=\"5.0\" predock_m=\"3.0\" berth_m=\"1.25\" lead_m=\"2.0\" "
  "berth_tolerance_m=\"0.25\" spray_timeout_s=\"60.0\" camera_max_age_s=\"3.0\"";

class MissionLoader : public ::testing::Test
{
protected:
  void SetUp() override
  {
    user_ = fs::temp_directory_path() / ("crsd_missions_" + std::to_string(::getpid()));
    fs::remove_all(user_);
    fs::create_directories(user_);
    dirs_ = {CRSD_TREES_DIR, user_.string()};
  }
  void TearDown() override {fs::remove_all(user_);}

  /// Writes <user dir>/<name>.xml and returns its resolved file.
  MissionFile put(const std::string & name, const std::string & xml)
  {
    std::ofstream(user_ / (name + ".xml")) << xml;
    return crusader_bt::resolveMission(dirs_, name);
  }

  /// Loads on a fresh factory, as the runner does for every goal.
  BT::Tree load(const MissionFile & file)
  {
    BT::BehaviorTreeFactory factory;
    crusader_bt::registerCrusaderNodes(factory);
    auto bb = BT::Blackboard::create();
    bb->set("ctx", std::make_shared<crusader_bt::Context>());
    return crusader_bt::loadMission(factory, dirs_, file, bb);
  }

  fs::path user_;
  MissionDirs dirs_;
};

TEST_F(MissionLoader, UserMissionResolvesTaskSubtreesFromTheLibrary)
{
  const MissionFile file = put(
    "one_tier",
    mission(std::string("<SubTree ID=\"Task1Advanced\" _autoremap=\"true\" ") + kTask1Ports + "/>"));
  EXPECT_TRUE(file.user);
  EXPECT_NO_THROW(load(file));
}

TEST_F(MissionLoader, TwoSubtreesSharingACommonBodyDoNotCollide)
{
  // Task1Advanced and Task1Disruptive both call Task1PlanDrive; Task3Core and Task3Advanced both
  // call Task3DockFire. The loader registers each body once, so no tree ID is a duplicate.
  const MissionFile file = put(
    "shared_body",
    mission(
      std::string("<SubTree ID=\"Task1Advanced\" _autoremap=\"true\" ") + kTask1Ports + "/>" +
      "<SubTree ID=\"Task1Disruptive\" _autoremap=\"true\" " + kTask1Ports + " plan_max_age_s=\"15.0\"/>" +
      "<SubTree ID=\"Task3Core\" _autoremap=\"true\" " + kTask3Ports + "/>" +
      "<SubTree ID=\"Task3Advanced\" _autoremap=\"true\" " + kTask3Ports + " decode_timeout_s=\"80.0\"/>"));
  EXPECT_NO_THROW(load(file));
}

TEST_F(MissionLoader, UnknownSubtreeIdIsAnError)
{
  const MissionFile file = put(
    "no_such_task", mission("<SubTree ID=\"Task9Core\" _autoremap=\"true\"/>"));
  EXPECT_ANY_THROW(load(file));
}

TEST_F(MissionLoader, BuiltInTreesLoadAsBefore)
{
  for (const char * name : {"task1_advanced", "task1_disruptive", "task3_core", "task3_advanced"}) {
    const MissionFile file = crusader_bt::resolveMission(dirs_, name);
    EXPECT_FALSE(file.user) << name;
    EXPECT_NO_THROW(load(file)) << name;
  }
}

TEST_F(MissionLoader, UnknownNameIsRefusedAndSaysWhatExists)
{
  put("mine", mission("<Sleep msec=\"1\"/>"));
  try {
    crusader_bt::resolveMission(dirs_, "nope");
    FAIL() << "resolved a name that is no file";
  } catch (const MissionError & e) {
    const std::string why = e.what();
    EXPECT_NE(why.find("no mission 'nope'"), std::string::npos) << why;
    EXPECT_NE(why.find("task1_advanced"), std::string::npos) << why;
    EXPECT_NE(why.find("yours: mine"), std::string::npos) << why;
  }
}

TEST_F(MissionLoader, ANameIsNeverAPath)
{
  put("mine", mission("<Sleep msec=\"1\"/>"));
  for (const char * name : {"", "../mine", "sub/mine", "mine.xml", "task1_advanced.xml", ".."}) {
    EXPECT_THROW(crusader_bt::resolveMission(dirs_, name), MissionError) << name;
  }
}

TEST_F(MissionLoader, BuiltInWinsANameBothHaveAndSaysSo)
{
  put("task1_advanced", mission("<Sleep msec=\"1\"/>"));
  const MissionFile file = crusader_bt::resolveMission(dirs_, "task1_advanced");
  EXPECT_FALSE(file.user);
  EXPECT_EQ(file.path, std::string(CRSD_TREES_DIR) + "/task1_advanced.xml");
  EXPECT_FALSE(file.warning.empty());
  EXPECT_TRUE(crusader_bt::resolveMission(dirs_, "task3_core").warning.empty());
}

TEST_F(MissionLoader, UserFileIsReadAgainOnEveryLoad)
{
  MissionFile file = put("again", mission("<Sleep msec=\"1\"/>"));
  EXPECT_NO_THROW(load(file));
  put("again", "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\"><NoSuchLeaf/></BehaviorTree></root>");
  EXPECT_ANY_THROW(load(file));
  fs::remove(user_ / "again.xml");
  EXPECT_THROW(crusader_bt::resolveMission(dirs_, "again"), MissionError);
}

TEST_F(MissionLoader, MissionNamesAreTheXmlStemsSorted)
{
  put("b_two", mission("<Sleep msec=\"1\"/>"));
  put("a_one", mission("<Sleep msec=\"1\"/>"));
  std::ofstream(user_ / "notes.txt") << "not a mission";
  fs::create_directories(user_ / ".archive");
  std::ofstream(user_ / ".archive" / "a_one.20260101-000000.xml") << "<root/>";
  EXPECT_EQ(crusader_bt::missionNames(user_.string()), (std::vector<std::string>{"a_one", "b_two"}));
  EXPECT_TRUE(crusader_bt::missionNames((user_ / "absent").string()).empty());
  EXPECT_EQ(crusader_bt::joinNames({}), "none");
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
