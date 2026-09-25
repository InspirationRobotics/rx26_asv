// rclcpp.hpp — NOT rclcpp. The four things crusader_bt's leaves use from it.
//
// Only the off-ROS runner puts this directory on the include path
// (tools/task3_sim/build.py: `-I crusader_bt/offros/shim` FIRST), so that
// context.hpp and leaves.cpp compile UNCHANGED against it. colcon never sees
// it: CMakeLists.txt does not mention offros/.
//
// What the leaves actually use, and so all this provides:
//
//   rclcpp::Node          only as `ctx->node->get_logger()` and `get_clock()`
//   rclcpp::Logger        a name
//   rclcpp::get_logger    by name
//   RCLCPP_INFO / WARN / ERROR / DEBUG, and the _THROTTLE forms
//
// If a leaf starts using anything else from rclcpp, the off-ROS build fails
// here, at compile time, naming the symbol - which is the right place to find
// out that a leaf has stopped being plumbing.
//
// Logs go to STDERR, one line each. STDOUT belongs to the JSON-lines protocol
// the sim speaks, and a log line there would be a malformed message.
#ifndef CRUSADER_BT_OFFROS_SHIM_RCLCPP_HPP_
#define CRUSADER_BT_OFFROS_SHIM_RCLCPP_HPP_

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>

namespace rclcpp
{

class Logger
{
public:
  explicit Logger(std::string name = "crusader_bt")
  : name_(std::move(name)) {}
  const char * get_name() const {return name_.c_str();}

private:
  std::string name_;
};

inline Logger get_logger(const std::string & name) {return Logger(name);}

/// Only ever dereferenced to hand to a _THROTTLE macro, which ignores it.
class Clock
{
public:
  using SharedPtr = std::shared_ptr<Clock>;
};

class Node
{
public:
  explicit Node(std::string name)
  : logger_(std::move(name)), clock_(std::make_shared<Clock>()) {}
  Logger get_logger() const {return logger_;}
  Clock::SharedPtr get_clock() const {return clock_;}

private:
  Logger logger_;
  Clock::SharedPtr clock_;
};

namespace shim
{

inline std::mutex & ioMutex()
{
  static std::mutex m;
  return m;
}

inline void log(const char * level, const Logger & lg, const char * fmt, ...)
{
  char buf[2048];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  std::lock_guard<std::mutex> lk(ioMutex());
  std::fprintf(stderr, "[%s] [%s] %s\n", level, lg.get_name(), buf);
  std::fflush(stderr);
}

/// One throttle clock per call site: the macro gives each its own static.
inline bool throttleOpen(std::chrono::steady_clock::time_point & last, long ms)
{
  const auto now = std::chrono::steady_clock::now();
  if (last.time_since_epoch().count() != 0 &&
    std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count() < ms)
  {
    return false;
  }
  last = now;
  return true;
}

}  // namespace shim
}  // namespace rclcpp

#define RCLCPP_DEBUG(logger, ...) do {(void)(logger);} while (0)
#define RCLCPP_INFO(logger, ...) ::rclcpp::shim::log("INFO", (logger), __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) ::rclcpp::shim::log("WARN", (logger), __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) ::rclcpp::shim::log("ERROR", (logger), __VA_ARGS__)

#define CRSD_SHIM_THROTTLE_(level, logger, ms, ...) \
  do { \
    static std::chrono::steady_clock::time_point crsd_last_{}; \
    if (::rclcpp::shim::throttleOpen(crsd_last_, static_cast<long>(ms))) { \
      ::rclcpp::shim::log(level, (logger), __VA_ARGS__); \
    } \
  } while (0)
#define RCLCPP_INFO_THROTTLE(logger, clock, ms, ...) \
  do {(void)(clock); CRSD_SHIM_THROTTLE_("INFO", logger, ms, __VA_ARGS__);} while (0)
#define RCLCPP_WARN_THROTTLE(logger, clock, ms, ...) \
  do {(void)(clock); CRSD_SHIM_THROTTLE_("WARN", logger, ms, __VA_ARGS__);} while (0)
#define RCLCPP_ERROR_THROTTLE(logger, clock, ms, ...) \
  do {(void)(clock); CRSD_SHIM_THROTTLE_("ERROR", logger, ms, __VA_ARGS__);} while (0)

#endif  // CRUSADER_BT_OFFROS_SHIM_RCLCPP_HPP_
