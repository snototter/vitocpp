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

/**
  * Nice summary of the problems of abort() and exit() and how these
  * can be overcome by catching exceptions within main():
  * https://stackoverflow.com/a/397081
  */
#ifdef VCP_LOG_STACK_TRACE_ON_ERROR
    #if defined(__APPLE__) || defined(__linux__) || defined(__unix__)
        // On these systems, we can use glibc to print the
        // current stack trace, see https://stackoverflow.com/a/77336

        /**@brief Show a [FAILURE] message on stderr, then throw a std::runtime_error. */
        #define VCP_ERROR(msg) {\
            VCP_LOG_FAILURE_LOCATION(msg);\
            void *array[10];\
            size_t size = backtrace(array, 10);\
            backtrace_symbols_fd(array, size, STDERR_FILENO);\
            throw std::runtime_error("Aborting VCP library call due to error!");\
        }

        /**@brief Show a [FAILURE] message on stderr, then abort. */
        #define VCP_ABORT(msg) {\
            VCP_LOG_FAILURE_LOCATION(msg);\
            void *array[10];\
            size_t size = backtrace(array, 10);\
            backtrace_symbols_fd(array, size, STDERR_FILENO);\
            abort();\
        }

        /**@brief Show a [FAILURE] message on stderr, then exit. */
        #define VCP_EXIT(msg, errcode) {\
            VCP_LOG_FAILURE_LOCATION(msg);\
            void *array[10];\
            size_t size = backtrace(array, 10);\
            backtrace_symbols_fd(array, size, STDERR_FILENO);\
            exit(errcode);\
        }
    #else //  defined(__APPLE__) || defined(__linux__) || defined(__unix__)
        #error "Printing stack trace is currently only supported under *nix-based systems"
    #endif //  defined(__APPLE__) || defined(__linux__) || defined(__unix__)
#else  // VCP_LOG_STACK_TRACE_ON_ERROR
    /**@brief Show a [FAILURE] message on stderr, then throw a std::runtime_error. */
    #define VCP_ERROR(msg) {\
        VCP_LOG_FAILURE_LOCATION(msg);\
        throw std::runtime_error("Aborting VCP library call due to error!");\
    }

    /**@brief Show a [FAILURE] message on stderr, then abort. */
    #define VCP_ABORT(msg) {\
        VCP_LOG_FAILURE_LOCATION(msg);\
        abort();\
    }

    /**@brief Show a [FAILURE] message on stderr, then exit. */
    #define VCP_EXIT(msg, errcode) {\
        VCP_LOG_FAILURE_LOCATION(msg);\
        exit(errcode);\
    }
#endif // VCP_LOG_STACK_TRACE_ON_ERROR


#define VCP_CHECK(condition)   { assert(condition); }


#define VCP_CHECK_NOTNULL(ptr) { assert(ptr != nullptr); }

#endif // __VCP_UTILS_ERROR_H__
