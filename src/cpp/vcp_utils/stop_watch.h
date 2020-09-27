#ifndef __VCP_UTILS_STOP_WATCH_H__
#define __VCP_UTILS_STOP_WATCH_H__

#include <iostream>
#include <chrono>

#include "vcp_logging.h"

namespace vcp
{
namespace utils
{
/** @brief A stop watch with configurable clock and precision. */
template <typename C=std::chrono::high_resolution_clock, typename P=std::chrono::milliseconds>
class StopWatch
{
public:
  using clock_type = C; /**< @brief Clock type used by this stop watch. */
  using precision = P; /**< @brief Precision used by this stop watch. */

  /** @brief C'tor starts the stop watch. */
  StopWatch() { Start(); }

  /** @brief (Re-)starts the stop watch. */
  void Start()
  {
    t_start_ = clock_type::now();
  }

  /** @brief Returns the elapsed time (in precision ticks) since start (or construction if you didn't @see Start() yourself afterwards). */
  double Elapsed() const
  {
    const auto t_end = clock_type::now();
    const auto duration = std::chrono::duration_cast<precision>(t_end - t_start_);
    return duration.count();
  }

  /** @brief Returns the number of years before this stop watch will overflow. */
  double YearsToOverflow() const
  {
    const auto duration_hrs = std::chrono::duration_cast<std::chrono::hours>(clock_type::time_point::max() - clock_type::now());
    return duration_hrs.count() / (24.0 * 365.25);
  }

private:
  typename clock_type::time_point t_start_;
};

/** @brief A stop watch which logs elapsed time via vcp_logging macros. */
template <typename C=std::chrono::high_resolution_clock, typename P=std::chrono::milliseconds>
class LogWatch
{
public:
  LogWatch(const std::string &label):
    label_(label)
  {}

  void Start()
  {
    watch_.Start();
  }

  void Log() const
  {
    VCP_LOG_INFO_DEFAULT("[" << label_ << "] Elapsed: " << watch_.Elapsed());
  }
private:
  std::string label_;
  StopWatch<C, P> watch_;
};
} // namespace utils
} // namespace vcp

#endif // __VCP_UTILS_STOP_WATCH_H__
