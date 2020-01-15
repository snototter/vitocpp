#ifndef __VCP_UTILS_TIMING_UTILS_H__
#define __VCP_UTILS_TIMING_UTILS_H__

#include <chrono>
#include <iostream>
#include <iomanip>

// For VCP, there's a CMake option to enable (verbose) timing.
// If you want to use TIC/TOC in your code, define VCP_VERBOSE_TIMING before including this header!
//#define VCP_VERBOSE_TIMING

/**
 * @brief To copy MATLAB's tic/toc functionality within C++, we have
 * to initialize some variables.
 */
#ifdef VCP_VERBOSE_TIMING
#define VCP_INIT_TIC_TOC \
  std::chrono::steady_clock::time_point _tt_chrono_begin; \
  std::chrono::steady_clock::time_point _tt_chrono_end;
#else
#define VCP_INIT_TIC_TOC
#endif


/**
 * @brief Starts the timer (like MATLAB's tic).
 */
#ifdef VCP_VERBOSE_TIMING
#define VCP_TIC \
  _tt_chrono_begin = std::chrono::steady_clock::now();
#else
#define VCP_TIC
#endif


/**
 * @brief Stops the timer and displays the elapsed time (like MATLAB's toc).
 */
#ifdef VCP_VERBOSE_TIMING
#define VCP_TOC_ \
  _tt_chrono_end = std::chrono::steady_clock::now(); \
  std::cout << "[Elapsed time]: "  << std::fixed << std::setprecision(3) << (std::chrono::duration_cast<std::chrono::microseconds>(_tt_chrono_end-_tt_chrono_begin).count() / 1000.) << " ms" << std::endl;
#else
#define VCP_TOC_
#endif


/**
 * @brief Same as TOC, but displaying a custom label.
 */
#ifdef VCP_VERBOSE_TIMING
#define VCP_TOC(label) \
  _tt_chrono_end = std::chrono::steady_clock::now(); \
  std::cout << "[" << (label) << "] Elapsed time: "  << std::fixed << std::setprecision(3) << (std::chrono::duration_cast<std::chrono::microseconds>(_tt_chrono_end-_tt_chrono_begin).count() / 1000.) << " ms" << std::endl;
#else
#define VCP_TOC(label)
#endif


/**
 * @brief Stops the timer and stores elapsed milliseconds in a variable of type double with the given name.
 */
#ifdef VCP_VERBOSE_TIMING
#define VCP_TOCVAL(varname) \
  _tt_chrono_end = std::chrono::steady_clock::now(); \
  double varname = (std::chrono::duration_cast<std::chrono::microseconds>(_tt_chrono_end-_tt_chrono_begin).count() / 1000.);
#else
#define VCP_TOCVAL(varname)
#endif

#endif // __VCP_UTILS_TIMING_UTILS_H__
