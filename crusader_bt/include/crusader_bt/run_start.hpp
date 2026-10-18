// run_start.hpp — RoboCommand's RunStart, as the runner latches it. stdlib only.
//
// ocs_client relays RoboCommand's commands on /crsd/ocs_command as JSON; the run
// start is `{"run_start": {"run_id": 3, ...}}` (`runStart` / `runId` also accepted).
// The runner calls parse() on every message and latch.set() on a hit, with the
// receive time on the clock the leaves read (Context::now_s). WaitForRunStart asks
// counts(). The latch is NOT reset per goal: a RunStart that beat the START button
// must still count.
#ifndef CRUSADER_BT__RUN_START_HPP_
#define CRUSADER_BT__RUN_START_HPP_

#include <cstddef>
#include <cstdlib>
#include <string>

namespace crusader_bt
{
namespace runstart
{

namespace detail
{

/// Position of the quoted token `"name"` in `s`, or npos (`"run_start_x"` does not match).
inline std::size_t findQuoted(const std::string & s, const std::string & name)
{
  return s.find("\"" + name + "\"");
}

/// The integer after the key at `at` (past `"name"`, a colon, optional quote), or -1.
inline int intAfter(const std::string & s, std::size_t at, std::size_t key_len)
{
  std::size_t i = at + key_len + 2;
  while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) {++i;}
  if (i >= s.size() || s[i] != ':') {return -1;}
  ++i;
  while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '"')) {++i;}
  const char * begin = s.c_str() + i;
  char * end = nullptr;
  const long v = std::strtol(begin, &end, 10);
  if (end == begin || v < 0 || v > 2147483647L) {return -1;}
  return static_cast<int>(v);
}

}  // namespace detail

/// True when `json` carries a run start. `*id` is its run id, -1 when it has none.
inline bool parse(const std::string & json, int * id)
{
  const bool hit = detail::findQuoted(json, "run_start") != std::string::npos ||
    detail::findQuoted(json, "runStart") != std::string::npos;
  if (!hit) {return false;}
  *id = -1;
  std::size_t best = std::string::npos;
  for (const char * key : {"run_id", "runId"}) {
    const std::size_t at = detail::findQuoted(json, key);
    if (at != std::string::npos && at < best) {
      best = at;
      *id = detail::intAfter(json, at, std::string(key).size());
    }
  }
  return true;
}

/// The newest RunStart seen. `t_s` is on the clock of Context::now_s.
struct Latch
{
  bool have = false;
  double t_s = 0.0;
  int id = -1;

  void set(double now_s, int run_id)
  {
    have = true;
    t_s = now_s;
    id = run_id;
  }

  /// Received no earlier than `max_age_s` before `since_s` (when the waiting began), so a
  /// RunStart that beat the START button still counts. A stale one does not.
  bool counts(double since_s, double max_age_s) const
  {
    return have && t_s >= since_s - max_age_s;
  }
};

}  // namespace runstart
}  // namespace crusader_bt

#endif  // CRUSADER_BT__RUN_START_HPP_
