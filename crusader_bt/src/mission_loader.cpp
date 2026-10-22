// mission_loader — see crusader_bt/mission_loader.hpp.
#include "crusader_bt/mission_loader.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <system_error>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace crusader_bt
{
namespace
{

namespace fs = std::filesystem;

/// A leading ~ is the HOME directory, as gcs_bridge expands its own user_trees_dir.
std::string expandHome(const std::string & path)
{
  const char * home = std::getenv("HOME");
  if (home == nullptr || path.empty() || path[0] != '~' || (path.size() > 1 && path[1] != '/')) {
    return path;
  }
  return std::string(home) + path.substr(1);
}

bool isFile(const std::string & path)
{
  std::error_code ec;
  return fs::is_regular_file(path, ec);
}

}  // namespace

MissionDirs missionDirs(const std::string & user_trees_dir)
{
  try {
    return {ament_index_cpp::get_package_share_directory("crusader_bt") + "/behavior_trees",
      expandHome(user_trees_dir)};
  } catch (const std::exception & e) {
    throw MissionError(std::string("crusader_bt is not installed: ") + e.what());
  }
}

std::vector<std::string> missionNames(const std::string & dir)
{
  std::vector<std::string> names;
  std::error_code ec;
  for (const auto & e : fs::directory_iterator(dir, ec)) {
    if (e.is_regular_file(ec) && e.path().extension() == ".xml") {
      names.push_back(e.path().stem().string());
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

std::string joinNames(const std::vector<std::string> & names)
{
  std::string out;
  for (const auto & n : names) {out += (out.empty() ? "" : ", ") + n;}
  return out.empty() ? "none" : out;
}

MissionFile resolveMission(const MissionDirs & dirs, const std::string & name)
{
  const bool bare = !name.empty() && std::all_of(name.begin(), name.end(), [](char c) {
      return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
    });
  const std::string builtin = dirs.builtin + "/" + name + ".xml";
  const std::string user = dirs.user + "/" + name + ".xml";
  if (bare && isFile(builtin)) {
    MissionFile file{builtin, false, ""};
    if (isFile(user)) {
      file.warning = "mission '" + name + "' exists as a user file too: the built-in wins; "
        "rename the user file to run it";
    }
    return file;
  }
  if (bare && isFile(user)) {
    return {user, true, ""};
  }
  throw MissionError(
    "no mission '" + name + "' (built-in: " + joinNames(missionNames(dirs.builtin)) +
    "; yours: " + joinNames(missionNames(dirs.user)) + ")");
}

BT::Tree loadMission(
  BT::BehaviorTreeFactory & factory, const MissionDirs & dirs, const MissionFile & file,
  BT::Blackboard::Ptr blackboard)
{
  if (file.user) {
    // The shared bodies first: the per-tier files call them by ID and do not include them.
    for (const char * sub : {"/lib/common", "/lib"}) {
      for (const auto & stem : missionNames(dirs.builtin + sub)) {
        factory.registerBehaviorTreeFromFile(dirs.builtin + sub + "/" + stem + ".xml");
      }
    }
  }
  // Absolute: BT.CPP throws "cannot make absolute path" on a bare filename, and resolves
  // <include path="lib/x.xml"/> against the file's own directory.
  return factory.createTreeFromFile(fs::absolute(file.path).string(), blackboard);
}

}  // namespace crusader_bt
