#ifndef __VCP_UTILS_LOGGING_H__
#define __VCP_UTILS_LOGGING_H__

#include <iostream>
#include <chrono>
#include <ctime>

// TODO doc
// VCP_LOG_LOCATION
// * Default macros use ANSI color codes to colorize the log severity.
// * DEBUG and INFO messages are logged to std::cout
// * WARNING and FAILURE messages are logged to std::cerr

//TODO skip if empty, else do undef, def
#define VCP_LOGGING_COMPONENT nullptr

#define VCP_UNUSED_VAR(var) (void)(var)

//----------------------------------------------------------------
// Generic/base macros, use only if you know what you are doing.
// You're probably looking for VCP_LOG_INFO, VCP_LOG_WARNING,
// or VCP_LOG_FAILURE!

// Default logging macro - write 'msg' to given 'stream'.
// If prefix/filename/line-nr are given, they will be printed too.
#define VCP_LOG_STREAM(stream, prefix, filename, linenr, msg)   (vcp::utils::logging::Log(stream, prefix, VCP_LOGGING_COMPONENT, filename, linenr, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg))

// Similar to VCP_LOG_STREAM, but there will be at most 1 message
// within nsec seconds.
// So make sure, not to send failure/error messages to NSEC...
#define VCP_LOG_STREAM_NSEC(stream, prefix, filename, linenr, msg, nsec)  (vcp::utils::logging::LogNSec(stream, prefix, VCP_LOGGING_COMPONENT, filename, linenr, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg, nsec));


// In case you want a time-stamped log message to a specific stream, use this
//TODO maybe deprecate this (?)
#define VCP_LOG_TIMED(stream, msg) (vcp::utils::logging::LogTimed(stream, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg))


//----------------------------------------------------------------
// Macro abstractions which prefix the proper log severity/level
//
// Logs a DEBUG notification to stdout.
#define VCP_LOG_DEBUG_LOCATION(msg)      VCP_LOG_STREAM(std::cout, "\033[34;1mDEBUG  \033[0m", __FILE__, __LINE__, msg)
#define VCP_LOG_DEBUG_DEFAULT(msg)       VCP_LOG_STREAM(std::cout, "\033[34;1mDEBUG  \033[0m", nullptr, -1, msg)
// Logs a INFO/status notification to stdout.
#define VCP_LOG_INFO_LOCATION(msg)       VCP_LOG_STREAM(std::cout, "\033[36;1mINFO   \033[0m", __FILE__, __LINE__, msg)
#define VCP_LOG_INFO_DEFAULT(msg)        VCP_LOG_STREAM(std::cout, "\033[36;1mINFO   \033[0m", nullptr, -1, msg)
// Logs a WARNING to stderr.
#define VCP_LOG_WARNING_LOCATION(msg)    VCP_LOG_STREAM(std::cerr, "\033[35;1mWARNING\033[0m", __FILE__, __LINE__, msg)
#define VCP_LOG_WARNING_DEFAULT(msg)     VCP_LOG_STREAM(std::cerr, "\033[35;1mWARNING\033[0m", nullptr, -1, msg)
// Logs a FAILURE to stderr.
#define VCP_LOG_FAILURE_LOCATION(msg)    VCP_LOG_STREAM(std::cerr, "\033[31;1mFAILURE\033[0m", __FILE__, __LINE__, msg)
#define VCP_LOG_FAILURE_DEFAULT(msg)     VCP_LOG_STREAM(std::cerr, "\033[31;1mFAILURE\033[0m", nullptr, -1, msg)


//----------------------------------------------------------------
// Macros intended to be used by library users/module devs.
//
// ==> If you want to use these macros outside of the VCP CMake
//     environment, you have to #define the log level:
//     VCP_LOG_LEVEL_DEBUG, VCP_LOG_LEVEL_INFO or VCP_LOG_LEVEL_WARNING
//
//
// Log DEBUG messages
#ifdef VCP_LOG_LEVEL_DEBUG
    #ifdef VCP_LOG_LOCATION
        #define VCP_LOG_DEBUG(msg) VCP_LOG_DEBUG_LOCATION(msg)
    #else // VCP_LOG_LOCATION
        #define VCP_LOG_DEBUG(msg) VCP_LOG_DEBUG_DEFAULT(msg)
    #endif // VCP_LOG_LOCATION
#else // VCP_LOG_ENABLE_DEBUG
    #define VCP_LOG_DEBUG(msg) do {} while(0)
#endif // VCP_LOG_ENABLE_DEBUG
//
//
// Log INFO messages
#if defined(VCP_LOG_LEVEL_DEBUG) || defined(VCP_LOG_LEVEL_INFO)
    #ifdef VCP_LOG_LOCATION
        #define VCP_LOG_INFO(msg)  VCP_LOG_INFO_LOCATION(msg)
    #else // VCP_LOG_LOCATION
        #define VCP_LOG_INFO(msg)  VCP_LOG_INFO_DEFAULT(msg)
    #endif // VCP_LOG_LOCATION
#else
    #define VCP_LOG_INFO(msg)  do {} while(0)
#endif
//
//
// Log WARNING messages
#if defined(VCP_LOG_LEVEL_DEBUG) || defined(VCP_LOG_LEVEL_INFO) || defined(VCP_LOG_LEVEL_WARNING)
    #ifdef VCP_LOG_LOCATION
        #define VCP_LOG_WARNING(msg)  VCP_LOG_WARNING_LOCATION(msg)
    #else // VCP_LOG_LOCATION
        #define VCP_LOG_WARNING(msg)  VCP_LOG_WARNING_DEFAULT(msg)
    #endif // VCP_LOG_LOCATION
#else
    #define VCP_LOG_WARNING(msg)  do {} while(0)
#endif
//
//
// Always log FAILURE messages
#ifdef VCP_LOG_LOCATION
    #define VCP_LOG_FAILURE(msg)    VCP_LOG_FAILURE_LOCATION(msg)
#else // VCP_LOG_LOCATION
    #define VCP_LOG_FAILURE(msg)    VCP_LOG_FAILURE_DEFAULT(msg)
#endif // VCP_LOG_LOCATION


// FIXME: clean up
//// Log message every nsec
//#define PVT_LOG_NSEC(nsec, msg) (vcp::utils::logging::LogNSec(std::cout, "\033[36;1mINFO\033[0m", __FILE__, __LINE__, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg, nsec));
//#define PVT_LOG_NOFILE_NSEC(nsec, msg) (vcp::utils::logging::LogNSec(std::cout, nullptr, nullptr, -1, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg, nsec));
//// Logs a warning to stderr.
////#define PVT_LOG_WARNING(msg) (vcp::utils::logging::Log(std::cerr, "\033[35;1mWARNING\033[0m", __FILE__, __LINE__, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg))
////#define PVT_LOG_WARNING_NOFILE(msg) (vcp::utils::logging::Log(std::cerr, "\033[35;1mWARNING\033[0m", nullptr, -1, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg))
//#define PVT_LOG_WARNING_NSEC(nsec, msg) (vcp::utils::logging::LogNSec(std::cerr, "\033[35;1mWARNING\033[0m", __FILE__, __LINE__, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg, nsec));
//#define PVT_LOG_INFO(msg) (vcp::utils::logging::Log(std::cout, "\033[36;1mINFO\033[0m", __FILE__, __LINE__, vcp::utils::logging::LogData<vcp::utils::logging::None>() << msg))



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
/** @brief Plain C++ utilities (extensions you commonly use, but don't want to add heavier dependencies, like boost). */
namespace utils
{
/** @brief Stream manipulation utilities to enable flexible logging to stdout/stderr via the VCP_LOG_{DEBUG|INFO|WARNING|FAILURE} macros. */
namespace logging
{
struct None { };

template<typename List>
struct LogData
{
  List list;
};


template<typename List>
void LogNSec(std::ostream &stream, const char* severity, const char* component, const char* file, int line, LogData<List>&& data, double nsec)
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
    stream << "[" << severity << "] ";

  if (component)
    stream << "(" << component << ") ";

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
void Log(std::ostream &stream, const char* severity, const char* component, const char* file, int line, LogData<List>&& data)
  NOINLINE_ATTRIBUTE {
  if (severity)
    stream << "[" << severity << "] ";

  if (component)
    stream << "(" << component << ") ";

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
