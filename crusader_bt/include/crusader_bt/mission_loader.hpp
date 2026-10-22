// mission_loader — where a mission NAME is a file, and how that file becomes a tree.
//
// The one loader bt_runner_node and bt_tree_check share, so a mission the checker accepts is
// loaded by the runner exactly as the checker loaded it.
//
//   built-in   <share>/crusader_bt/behavior_trees/<name>.xml. Its own <include>s pull in the
//              lib/ files it needs, relative to itself.
//   user       <user_trees_dir>/<name>.xml, uploaded from the ground station. It has NO
//              <include>: it names the task subtrees by ID (Task1Advanced, Task3Core ...), and
//              this loader registers all of lib/common/ and lib/ first, in that order, so a
//              body two tiers share (Task1PlanDrive) is registered once, never twice.
//
// A built-in wins a name both have. The files are read on every call: nothing is cached, so an
// upload is the next goal's mission.
#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

namespace crusader_bt
{

/// A mission that cannot be found or loaded; the message is for the operator.
struct MissionError : std::runtime_error
{
  using std::runtime_error::runtime_error;
};

struct MissionDirs
{
  std::string builtin;   ///< <share>/crusader_bt/behavior_trees
  std::string user;      ///< user_trees_dir with a leading ~ expanded; may not exist yet
};

/// The installed built-in directory and `user_trees_dir` (a leading ~ is the HOME directory).
/// MissionError when crusader_bt is not installed.
MissionDirs missionDirs(const std::string & user_trees_dir);

struct MissionFile
{
  std::string path;
  bool user = false;
  std::string warning;   ///< not empty when a user mission of this name exists and the built-in won
};

/// The file a goal's `name` means: the built-in first, else the user's. A bare name only
/// (letters, digits, _), never a path. MissionError names what exists when there is no such file.
MissionFile resolveMission(const MissionDirs & dirs, const std::string & name);

/// THE loader. A user file gets lib/common/*.xml, then lib/*.xml registered on `factory` before
/// it is read, so `factory` must be fresh (a second user mission on it would register twice).
/// BT.CPP's own exceptions pass through.
BT::Tree loadMission(
  BT::BehaviorTreeFactory & factory, const MissionDirs & dirs, const MissionFile & file,
  BT::Blackboard::Ptr blackboard);

/// The stems of the .xml files directly in `dir`, sorted; empty when there is no such directory.
std::vector<std::string> missionNames(const std::string & dir);

/// "a, b, c", or "none" for no names.
std::string joinNames(const std::vector<std::string> & names);

}  // namespace crusader_bt
