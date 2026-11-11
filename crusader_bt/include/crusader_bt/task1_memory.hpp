// task1_memory.hpp — the Core tier's ENTRY/EXIT between runs. Pure: no ROS, no files.
//
// The competition re-deals the reds and greens between runs but not the blues, so the boat
// keeps what it learnt about the ENTRY (flashing blue) and EXIT (solid blue) and forgets the
// colours (target_tracker, at every mission start). bt_runner_node owns the file and the log;
// this header holds the rules, so they are tested off-ROS (test/test_task1_memory.cpp):
//
//   * WHICH POINT IS USED, per beacon, first that exists:
//       PLAN        the passage plan's point: Ekko's, or the ground station's MANUAL field;
//       SEEN        the tracked buoy of that colour with the most evidence (nav::bestBeacon);
//       REMEMBERED  the last SEEN one with enough evidence, kept in a file across runs.
//     A hand-placed point only has to be roughly right: the passage planner centres the orbit
//     on a tracked buoy within anchor_snap_m of it (gp::anchorOf).
//   * WHEN A SIGHTING IS REMEMBERED: at least min_hits sightings behind the track, and either
//     nothing is remembered yet or it is more than move_m from what is.
//   * A plan with NO red or green (a MANUAL field holding only ENTRY/EXIT) is Core tier: the
//     reds and greens are the boat's own, with sensor-triggered replans.
#pragma once

#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include "crusader_bt/nav_math.hpp"

namespace crusader_bt
{
namespace t1mem
{

/// One remembered beacon. `saved` is a local time stamp for the operator, never parsed.
struct Remembered
{
  bool have = false;
  nav::LatLon ll{};
  int hits = 0;
  std::string saved;
};

enum class Source {None, Plan, Seen, Remembered};

inline const char * sourceName(Source s)
{
  switch (s) {
    case Source::Plan: return "plan";
    case Source::Seen: return "seen";
    case Source::Remembered: return "remembered";
    case Source::None: break;
  }
  return "none";
}

/// A plan position that means something: finite and not the (0, 0) a cleared or partial
/// PassagePlan carries for "no point".
inline bool validLatLon(double lat, double lon)
{
  return std::isfinite(lat) && std::isfinite(lon) && std::fabs(lat) <= 90.0 &&
         std::fabs(lon) <= 180.0 && !(lat == 0.0 && lon == 0.0);
}

/// Does the plan hold any side rule? None = Core tier (the plan gives only the anchors).
inline bool hasSides(const std::vector<nav::PlanBuoy> & plan)
{
  for (const nav::PlanBuoy & b : plan) {
    if (nav::isSideConstrained(b.state)) {return true;}
  }
  return false;
}

struct Anchor
{
  bool have = false;
  nav::Vec2 p{};
  Source src = Source::None;
};

/// The point used for one beacon: the plan's, else the one seen now, else the remembered one.
/// `plan_ok` says whether `plan_p` is a real point (validLatLon of the plan's lat/lon).
inline Anchor pick(bool plan_ok, nav::Vec2 plan_p, const nav::Buoy * seen,
  const Remembered & mem, nav::LatLon origin)
{
  if (plan_ok) {return {true, plan_p, Source::Plan};}
  if (seen != nullptr) {return {true, seen->p, Source::Seen};}
  if (mem.have) {return {true, nav::toLocal(mem.ll, origin), Source::Remembered};}
  return {};
}

/// Should this sighting replace what is remembered?
inline bool shouldRemember(const nav::Buoy * seen, const Remembered & mem, nav::LatLon origin,
  int min_hits, double move_m)
{
  if (seen == nullptr || seen->hits < min_hits || !std::isfinite(seen->p.x) ||
    !std::isfinite(seen->p.y))
  {
    return false;
  }
  if (!mem.have) {return true;}
  return nav::norm(seen->p - nav::toLocal(mem.ll, origin)) > move_m;
}

// ---------------------------------------------------------------- the file

/// "entry <lat> <lon> <hits> <saved>" per remembered beacon, under a comment line.
inline std::string serialize(const Remembered & entry, const Remembered & exitb)
{
  std::string out =
    "# Task 1 ENTRY/EXIT remembered between runs by bt_runner_node. Delete it to forget.\n";
  const auto line = [&out](const char * name, const Remembered & r) {
      if (!r.have) {return;}
      char b[160];
      std::snprintf(b, sizeof(b), "%s %.8f %.8f %d %s\n", name, r.ll.lat, r.ll.lon, r.hits,
        r.saved.empty() ? "-" : r.saved.c_str());
      out += b;
    };
  line("entry", entry);
  line("exit", exitb);
  return out;
}

/// The inverse of serialize. A malformed or out-of-range line is skipped, never half-read.
inline void parse(const std::string & text, Remembered & entry, Remembered & exitb)
{
  entry = Remembered{};
  exitb = Remembered{};
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ls(line);
    std::string name, saved;
    double lat = NAN, lon = NAN;
    int hits = 0;
    if (!(ls >> name >> lat >> lon >> hits)) {continue;}
    ls >> saved;
    if (!validLatLon(lat, lon)) {continue;}
    Remembered r{true, {lat, lon}, hits, saved == "-" ? std::string() : saved};
    if (name == "entry") {entry = r;} else if (name == "exit") {exitb = r;}
  }
}

/// {"lat":..,"lon":..,"hits":..,"saved":".."} or null, for /crsd/bt_status.
inline std::string json(const Remembered & r)
{
  if (!r.have) {return "null";}
  char b[200];
  std::snprintf(b, sizeof(b), "{\"lat\":%.8f,\"lon\":%.8f,\"hits\":%d,\"saved\":\"%s\"}",
    r.ll.lat, r.ll.lon, r.hits, r.saved.c_str());
  return b;
}

}  // namespace t1mem
}  // namespace crusader_bt
