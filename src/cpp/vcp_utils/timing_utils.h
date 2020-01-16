#ifndef __VCP_UTILS_TIMING_UTILS_H__
#define __VCP_UTILS_TIMING_UTILS_H__

#include <chrono>
#include <iostream>
#include <iomanip>

// For VCP, there's a CMake option to enable (verbose) timing (i.e. TOC outputs to stdout).
// If you want to use TIC/TOC in your code, define VCP_VERBOSE_TIMING before including this header!
//
// Otherwise, you'll only be able to use "VCP_TOC_DEFVAR" and "VCP_TOC_ASSIGN" to store the elapsed time
// into a variable.

/**
 * @brief To copy MATLAB's tic/toc functionality within C++, we have
 * to initialize some variables.
 */
#define VCP_INIT_TIC_TOC \
  std::chrono::steady_clock::time_point _tt_chrono_begin; \
  std::chrono::steady_clock::time_point _tt_chrono_end;


/**
 * @brief Starts the timer (like MATLAB's tic).
 */
#define VCP_TIC \
  _tt_chrono_begin = std::chrono::steady_clock::now();


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
 * If you already have defined the variable, use VCP_TOC_ASSIGN(varname) instead.
 */
#define VCP_TOC_DEFVAR(varname) \
  _tt_chrono_end = std::chrono::steady_clock::now(); \
  double varname = (std::chrono::duration_cast<std::chrono::microseconds>(_tt_chrono_end-_tt_chrono_begin).count() / 1000.);


/**
 * @brief Stops the timer and stores elapsed milliseconds in a (double) variable of type double with the given name.
 * If this variable hasn't been defined yet, use VCP_TOC_DEFVAR(varname) instead!
 */
#define VCP_TOC_ASSIGN(varname) \
  _tt_chrono_end = std::chrono::steady_clock::now(); \
  varname = (std::chrono::duration_cast<std::chrono::microseconds>(_tt_chrono_end-_tt_chrono_begin).count() / 1000.);

#endif // __VCP_UTILS_TIMING_UTILS_H__
