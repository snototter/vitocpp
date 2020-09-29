#ifndef __VCP_UTILS_STOP_WATCH_H__
#define __VCP_UTILS_STOP_WATCH_H__

#include <iostream>
#include <chrono>
#include <sstream>

#include "vcp_logging.h"

namespace vcp
{
namespace utils
{

template <typename P>
std::string DurationAbbreviation()
{
  if (std::is_same<P, std::chrono::nanoseconds>::value)
    return "ns";
  else if (std::is_same<P, std::chrono::microseconds>::value)
    return "us";
  else if (std::is_same<P, std::chrono::milliseconds>::value)
    return "ms";
  else if (std::is_same<P, std::chrono::seconds>::value)
    return "sec";
  else if (std::is_same<P, std::chrono::minutes>::value)
    return "min";
  else if (std::is_same<P, std::chrono::hours>::value)
    return "h";
  return std::string("");
}

/** @brief A stop watch with configurable clock and precision. */
template <typename C=std::chrono::high_resolution_clock, typename P=std::chrono::milliseconds>
class StopWatch
{
public:
  using clock_type = C; /**< @brief Clock type used by this stop watch. */
  using precision = P; /**< @brief Precision used by this stop watch. */

  /** @brief C'tor starts the stop watch. */
  StopWatch() { Start(); }

  StopWatch(const StopWatch &other) :
    t_start_(other.t_start_)
  {}

  virtual ~StopWatch() {}

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

  /** @brief Returns whether the underyling clock is steady or not. */
  bool IsSteady() const
  {
    return clock_type::is_steady;
  }

  /** @brief Returns the abbreviation for the clock's precision ('ns', 'us', 'ms', etc.). */
  std::string PrecisionAbbreviation() const
  {
    return DurationAbbreviation<P>();
  }

private:
  typename clock_type::time_point t_start_;
};


/** @brief A stop watch which logs elapsed time via vcp_logging macros. */
template <typename C=std::chrono::high_resolution_clock, typename P=std::chrono::milliseconds>
class LogWatch : public StopWatch<C, P>
{
public:
  LogWatch(const std::string &label):
    StopWatch<C,P>(),
    label_(label)
  {}

  std::string LogMessage() const
  {
    std::stringstream msg;
    msg << "StopWatch [" << label_ << "] elapsed: " << this->Elapsed() << " " << this->PrecisionAbbreviation();
    return msg.str();
  }

  void PrintLog() const
  {
    VCP_LOG_INFO_DEFAULT(LogMessage());
  }

private:
  std::string label_;
};
} // namespace utils
} // namespace vcp

#endif // __VCP_UTILS_STOP_WATCH_H__
