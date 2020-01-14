#ifndef __VCP_UTILS_LOGGING_H__
#define __VCP_UTILS_LOGGING_H__

#include <iostream>
#include <chrono>
#include <ctime>

//FIXME TODO make LOG_DEBUG and enable/disable with cmake option!
//FIXME implement _NSEC macros
// * Default macros use ANSI color codes to colorize the log severity.
// TODO DEBUG
// * DEBUG and INFO messages are logged to std::cout
// * WARNING and FAILURE messages are logged to std::cerr
// * TODO doc NSEC, etc.

//#define PVT_LOG_INFO(msg) (vcp::utils::logging::Log(std::cout, "\033[36;1mINFO\033[0m", __FILE__, __LINE__, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg))
//#define PVT_LOG_INFO_NOFILE(msg) (vcp::utils::logging::Log(std::cout, "\033[36;1mINFO\033[0m", nullptr, -1, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg))
#define PVT_LOG_NOFILE(msg) (vcp::utils::logging::Log(std::cout, nullptr, nullptr, -1, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg))

// Log current time + message
#define PVT_LOG_TIMED(msg) (vcp::utils::logging::LogTimed(std::cout, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg))

// Log message every nsec
#define PVT_LOG_NSEC(nsec, msg) (vcp::utils::logging::LogNSec(std::cout, "\033[36;1mINFO\033[0m", __FILE__, __LINE__, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg, nsec));
#define PVT_LOG_NOFILE_NSEC(nsec, msg) (vcp::utils::logging::LogNSec(std::cout, nullptr, nullptr, -1, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg, nsec));

// Logs a warning to stderr.
//#define PVT_LOG_WARNING(msg) (vcp::utils::logging::Log(std::cerr, "\033[35;1mWARNING\033[0m", __FILE__, __LINE__, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg))
//#define PVT_LOG_WARNING_NOFILE(msg) (vcp::utils::logging::Log(std::cerr, "\033[35;1mWARNING\033[0m", nullptr, -1, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg))
#define PVT_LOG_WARNING_NSEC(nsec, msg) (vcp::utils::logging::LogNSec(std::cerr, "\033[35;1mWARNING\033[0m", __FILE__, __LINE__, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg, nsec));

// Use only if you know what you are doing - you're probably looking
// for VCP_LOG_INFO, VCP_LOG_WARNING, or VCP_LOG_FAILURE!
#define VCP_LOG_STREAM(stream, prefix, filename, linenr, msg)   (vcp::utils::logging::Log(stream, prefix, filename, linenr, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg))

//TODO add debug macro

// Logs a simple information/notification to stdout.
#define VCP_LOG_INFO_LOCATION(msg)       VCP_LOG_STREAM(std::cout, "\033[36;1mINFO\033[0m", __FILE__, __LINE__, msg)
#define VCP_LOG_INFO_DEFAULT(msg)        VCP_LOG_STREAM(std::cout, "\033[36;1mINFO\033[0m", nullptr, -1, msg)
#define VCP_LOG_INFO(msg)  VCP_LOG_INFO_DEFAULT(msg)

#define PVT_LOG_INFO(msg) (vcp::utils::logging::Log(std::cout, "\033[36;1mINFO\033[0m", __FILE__, __LINE__, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg))
// Logs a WARNING to stderr.
#define VCP_LOG_WARNING_LOCATION(msg)       VCP_LOG_STREAM(std::cerr, "\033[35;1mWARNING\033[0m", __FILE__, __LINE__, msg)
#define VCP_LOG_WARNING_DEFAULT(msg)        VCP_LOG_STREAM(std::cerr, "\033[35;1mWARNING\033[0m", nullptr, -1, msg)
// TODO should we add a cmake option to enable something like: #ifdef VCP_LOG_WITH_SRC_LOCATION ??
#define VCP_LOG_WARNING(msg)  VCP_LOG_WARNING_DEFAULT(msg)

// Logs a FAILURE to stderr.
#define VCP_LOG_FAILURE_LOCATION(msg)       VCP_LOG_STREAM(std::cerr, "\033[31;1mFAILURE\033[0m", __FILE__, __LINE__, msg)
#define VCP_LOG_FAILURE_DEFAULT(msg)        VCP_LOG_STREAM(std::cerr, "\033[31;1mFAILURE\033[0m", nullptr, -1, msg)
// TODO should we add a cmake option to enable something like: #ifdef VCP_LOG_WITH_SRC_LOCATION ??
#define VCP_LOG_FAILURE(msg)  VCP_LOG_FAILURE_DEFAULT(msg)





#ifndef NOINLINE_ATTRIBUTE
// Adapted from http://stackoverflow.com/questions/19415845/a-better-log-macro-using-template-metaprogramming
// This workaround is needed because GCC 4.7.2 doesn't know the noinline attribute
    #ifdef __ICC
        #define NOINLINE_ATTRIBUTE __attribute__(( noinline ))
    #else
        #define NOINLINE_ATTRIBUTE
    #endif // __ICC
#endif // NOINLINE_ATTRIBUTE

/** @brief A collection of utilities - gathered and re-implemented over and over again throughout my studies. */
namespace vcp
{
namespace utils
{
namespace logging
{
struct None { };

template<typename List>
struct LogData
{
  List list;
};


template<typename List>
void LogNSec(std::ostream &stream, const char* severity, const char* file, int line, LogData<List>&& data, double nsec)
NOINLINE_ATTRIBUTE {
  static bool initialized = false;
  static std::chrono::system_clock::time_point prev_invocation;

  std::chrono::system_clock::time_point now = std::chrono::high_resolution_clock::now();
  if (initialized)
  {
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - prev_invocation).count() / 1000. < nsec)
      return;
  }
  else
  {
    initialized = true;
  }
  prev_invocation = now;
  if (severity)
    stream << "[" << severity << "]: ";
  output(stream, std::move(data.list));
  stream << std::endl;
  if (file)
    stream << "  In " << file << ":" << line << std::endl;
}

template<typename List>
void LogTimed(std::ostream &stream, LogData<List>&& data)
  NOINLINE_ATTRIBUTE {
  const time_t raw_time = std::time(nullptr);
  const struct tm *time_info = std::localtime(&raw_time);

  char buffer[80];
  std::strftime(buffer, 80, "%Y-%m-%d %I:%M:%S", time_info);

  stream << "[" << buffer << "] ";
  output(stream, std::move(data.list));
  stream << std::endl;
}

template<typename List>
void Log(std::ostream &stream, const char *severity, const char* file, int line, LogData<List>&& data)
  NOINLINE_ATTRIBUTE {
  if (severity)
    stream << "[" << severity << "]: ";
  output(stream, std::move(data.list));
  stream << std::endl;
  if (file)
    stream << "  In " << file << ":" << line << std::endl;
}

template<typename Begin, typename Value>
constexpr LogData<std::pair<Begin&&, Value&&>> operator<<(LogData<Begin>&& begin,
                                                          Value&& value) noexcept {
  return {{ std::forward<Begin>(begin.list), std::forward<Value>(value) }};
}

template<typename Begin, size_t n>
constexpr LogData<std::pair<Begin&&, const char*>> operator<<(LogData<Begin>&& begin,
                                                              const char (&value)[n]) noexcept {
  return {{ std::forward<Begin>(begin.list), value }};
}

typedef std::ostream& (*PfnManipulator)(std::ostream&);

template<typename Begin>
constexpr LogData<std::pair<Begin&&, PfnManipulator>> operator<<(LogData<Begin>&& begin,
                                                                 PfnManipulator value) noexcept {
  return {{ std::forward<Begin>(begin.list), value }};
}

template <typename Begin, typename Last>
void output(std::ostream& os, std::pair<Begin, Last>&& data) {
  output(os, std::move(data.first));
  os << data.second;
}

inline void output(std::ostream& /*os*/, None) { }
} // namespace logging
} // namespace utils
} // namespace vcp

#endif // __VCP_UTILS_LOGGING_H__
