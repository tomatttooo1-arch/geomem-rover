// DPUPIPE1 (2026-11-03): minimal compile-only stand-in for <glog/logging.h>, used ONLY when the board has the
// glog runtime (libglog.so.0, which libunilog links) but not the -dev headers. VART 3.0's UniLog.hpp includes
// glog for its logging macros; dpu_rx never calls them, so LOG()/CHECK() here just swallow the stream (CHECK
// failures abort). The real libglog is still what libunilog/libvart use at run time.
#pragma once
#include <cstdlib>
#include <ostream>
#include <sstream>
namespace google {
enum LogSeverity { GLOG_INFO = 0, GLOG_WARNING = 1, GLOG_ERROR = 2, GLOG_FATAL = 3 };
class NullStream : public std::ostringstream {};
class LogMessageVoidify {
 public:
  void operator&(std::ostream&) {}
};
struct FatalStream : public std::ostringstream {
  ~FatalStream() { std::abort(); }
};
}  // namespace google
#define LOG(severity) google::NullStream()
#define VLOG(level) google::NullStream()
#define LOG_IF(severity, cond) google::NullStream()
#define VLOG_IS_ON(level) false
#define CHECK(cond) if (!(cond)) google::FatalStream()
#define CHECK_EQ(a, b) CHECK((a) == (b))
#define CHECK_NE(a, b) CHECK((a) != (b))
#define CHECK_LT(a, b) CHECK((a) < (b))
#define CHECK_LE(a, b) CHECK((a) <= (b))
#define CHECK_GT(a, b) CHECK((a) > (b))
#define CHECK_GE(a, b) CHECK((a) >= (b))
#define DCHECK(cond) CHECK(cond)
