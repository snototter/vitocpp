#ifndef __VCP_UTILS_ERROR_H__
#define __VCP_UTILS_ERROR_H__

#include <cstdlib>
#include <assert.h>
#include <exception>

#if defined(__APPLE__) || defined(__linux__) || defined(__unix__)
  #include <execinfo.h>
  #include <unistd.h>
#endif

#include "vcp_logging.h"

//FIXME
// define VCP_RAISE_ERROR()
// add cmake option PRINT_STACK_TRACE
#if defined(__APPLE__) || defined(__linux__) || defined(__unix__)
// Print stack trace when aborting: https://stackoverflow.com/a/77336
#define PVT_ABORT(msg) {\
  PVT_LOG_FAILURE(msg);\
  throw std::runtime_error("Aborting PVT library call due to error!");\
  }
/**#define PVT_ABORT(msg) {\
//  PVT_LOG_FAILURE(msg);\
//  void *array[10];\
//  size_t size = backtrace(array, 10);\
//  backtrace_symbols_fd(array, size, STDERR_FILENO);\
//  abort();\
  }*/
#else
#define PVT_ABORT(msg) {\
  PVT_LOG_FAILURE(msg);\
  abort();\
  }
#endif

#define PVT_EXIT(msg) {\
  PVT_LOG_FAILURE(msg);\
  exit(1);\
  }

#define PVT_CHECK(condition) {assert(condition);}

#define PVT_CHECK_NOTNULL(ptr) {assert(ptr != nullptr);}

#endif // __VCP_UTILS_ERROR_H__
