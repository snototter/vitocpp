#ifndef __VCP_IMVIS_TRAJECTORIES_H__
#define __VCP_IMVIS_TRAJECTORIES_H__

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/core/core.hpp>
#else
    #include <opencv2/core.hpp>
#endif

namespace vcp
{
namespace imvis
{
/** @brief TODO doc. */
namespace trajectories
{
/** @brief Smoothes the given data points such that each point is the average over a
 * window of "span" values (centered on the processed point). First and last
 * point of the data won't be smoothed. Similar behavior as MATLAB's smooth().
 *
 * Requires that the span is odd and >= 3. Example, span = 5:
 * output[0] = trajectory[0]
 * output[1] = (trajectory[0] + trajectory[1] + trajectory[2]) / 3
 * output[2] = (t[0] + ... + t[4]) / 5
 * output[3] = (t[1] + ... + t[5]) / 5
 */
template <class Container>
Container SmoothTrajectoryMovingAverage(const Container &trajectory, int span)
{
  Container smoothed_trajectory;
  const int neighbors = (span - 1)/2;
  for (size_t ti = 0; ti < trajectory.size(); ++ti)
  {
    const int idx_int = static_cast<int>(ti);
    int from = std::max(0, idx_int - neighbors);
    int to = std::min(static_cast<int>(trajectory.size()-1), idx_int + neighbors);

    // Reduce span at the beginning/end (where there are less neighbors).
    const int n = std::min(idx_int - from, to - idx_int);
    from = idx_int - n;
    to = idx_int + n;

    // Average all values within the span.
    typename Container::value_type average = trajectory[from];
    for (int win_idx = from + 1; win_idx <= to; ++win_idx)
    {
      average += trajectory[win_idx];
    }
    average /= static_cast<double>(2*n + 1);
    smoothed_trajectory.push_back(average);
  }
  return smoothed_trajectory;
}


// Note that we couldn't use typename Container::value_type as this lead to "mismatch errors" when
// using "auto" type derivation in lambdas (with GCC 4.8).
// Thus, we specify the typename explicitly.
template <class Container, typename T>
std::vector<T> SmoothConvertedTrajectoryMovingAverage(const Container &trajectory,
                                                  T (*converter)(const typename Container::value_type &),
                                                  int span)
{
  std::vector<T> converted;
  converted.reserve(trajectory.size());
  for (size_t i = 0; i < trajectory.size(); ++i)
    converted.push_back(converter(trajectory[i]));
  return SmoothTrajectoryMovingAverage(converted, span);
}


/** @brief Visualize a trajectory with fading line thickness/color.
 * @param[in,out] image
 * @param[in] positions         Target positions (from [0] oldest to [size-1] newest; otherwise set newest_position_first).
 * @param[in] newest_position_first Set to true if positions[0] is the newest.
 * @param[in] trajectory_length How many positions should be included in the trace (must be >= 2), set to -1 to draw all given positions.
 * @param[in] obj_color
 * @param[in] fade_color
 * @param[in] max_trace_width   Maximum line width in pixels (>= 2, trace width decreases based on age of the position).
 * @param[in] dash_length       If negative, line will be solid; else, defines the length of each dash in pixels.
 * @param[in] smoothing_window  If negative, uses raw positions; else, calls @see SmoothTrajectoryMovingAverage() first. Must be odd.
 */
void DrawFadingTrajectory2d(cv::Mat &image, const std::vector<cv::Vec2d> &positions, bool newest_position_first=false,
              int smoothing_window=-1, int trajectory_length=-1, const cv::Scalar &obj_color=cv::Scalar(255, 0, 0), const cv::Scalar &fade_color=cv::Scalar(180, 180, 180),
              int max_line_width=3, int dash_length=-1);



/** @brief Visualize a trajectory with fading line thickness/color. @see DrawFadingTrajectory for other parameters
 * @param[in] converter         Function to convert the container element to a cv::Vec2d
 */
template <typename Container, typename PositionType>
void DrawFadingTrajectory(cv::Mat &image, const Container &positions, cv::Vec2d (*converter)(const PositionType &), bool newest_position_first=false,
              int smoothing_window=-1, int trajectory_length=-1, const cv::Scalar &obj_color=cv::Scalar(255, 0, 0), const cv::Scalar &fade_color=cv::Scalar(180, 180, 180),
              int max_line_width=3, int dash_length=-1)
{
  std::vector<cv::Vec2d> trajectory;
  trajectory.reserve(positions.size());
  for (size_t i = 0; i < positions.size(); ++i)
    trajectory.push_back(converter(positions[i]));
  DrawFadingTrajectory2d(image, trajectory, newest_position_first, smoothing_window, trajectory_length,
                         obj_color, fade_color, max_line_width, dash_length);
}


/** @brief Similar to @see DrawFadingTrajectory() except that it just draws a polyline :-). */
void DrawTrajectory2d(cv::Mat &image, const std::vector<cv::Vec2d> &positions, bool newest_position_first=false,
              int smoothing_window=-1, int trajectory_length=-1, const cv::Scalar &obj_color=cv::Scalar(255, 0, 0),
              int line_width=1, int dash_length=-1);


/** @brief Visualize a trajectory (without fading the line width or color).
 * @param[in] converter         Function to convert the container element to a cv::Vec2d
 */
template <typename Container, typename PositionType>
void DrawTrajectory2d(cv::Mat &image, const Container &positions, cv::Vec2d (*converter)(const PositionType &), bool newest_position_first=false,
              int smoothing_window=-1, int trajectory_length=-1, const cv::Scalar &obj_color=cv::Scalar(255, 0, 0),
              int line_width=1, int dash_length=-1)
{
  std::vector<cv::Vec2d> trajectory;
  trajectory.reserve(positions.size());
  for (size_t i = 0; i < positions.size(); ++i)
    trajectory.push_back(converter(positions[i]));
  DrawTrajectory2d(image, trajectory, newest_position_first, smoothing_window, trajectory_length, obj_color, line_width, dash_length);
}


/** @brief Error elipse for covariance matrix. */
cv::RotatedRect GetErrorEllipse(double chisquare_val, const cv::Vec2d &mean, const cv::Mat &cov_mat);

/** @brief Error elipse at 90 % confidence. */
cv::RotatedRect GetErrorEllipse90(const cv::Vec2d &mean, const cv::Mat &cov_mat);

/** @brief Error elipse at 95 % confidence. */
cv::RotatedRect GetErrorEllipse95(const cv::Vec2d &mean, const cv::Mat &cov_mat);

/** @brief Error elipse at 99 % confidence. */
cv::RotatedRect GetErrorEllipse99(const cv::Vec2d &mean, const cv::Mat &cov_mat);

/** @brief Visualize an error ellipse for 2D data. */
void DrawErrorEllipse(cv::Mat &image, const cv::Vec2d &mean, const cv::Mat &cov_mat, const cv::Scalar &color, double chisquare_val, int line_width=1, double fill_opacity=0.0);

/** @brief Visualize an error ellipse at 90 % confidence for 2D data. */
void DrawErrorEllipse90(cv::Mat &image, const cv::Vec2d &mean, const cv::Mat &cov_mat, const cv::Scalar &color, int line_width=1, double fill_opacity=0.0);

/** @brief Visualize an error ellipse at 95 % confidence for 2D data. */
void DrawErrorEllipse95(cv::Mat &image, const cv::Vec2d &mean, const cv::Mat &cov_mat, const cv::Scalar &color, int line_width=1, double fill_opacity=0.0);

/** @brief Visualize an error ellipse at 99 % confidence for 2D data. */
void DrawErrorEllipse99(cv::Mat &image, const cv::Vec2d &mean, const cv::Mat &cov_mat, const cv::Scalar &color, int line_width=1, double fill_opacity=0.0);

} // namespace trajectories
} // namespace imvis
} // namespace vcp

#endif // __VCP_IMVIS_TRAJECTORIES_H__
