#include "trajectories.h"

#include <vcp_utils/vcp_logging.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "drawing.h"
#include <vcp_math/conversions.h>
#include <vcp_math/common.h>

namespace vcp
{
namespace imvis
{
namespace trajectories
{

void DrawFadingTrajectory2d(cv::Mat &image, const std::vector<cv::Vec2d> &positions, bool newest_position_first, int smoothing_window, int trajectory_length, const cv::Scalar &obj_color, const cv::Scalar &fade_color, int max_line_width, int dash_length)
{
  //FIXME TODO dashed trajectories doesn't work yet! we need to interpolate the trajectory points at the dash transitions :-/
  const std::vector<cv::Vec2d> trajectory = smoothing_window > 0 ? SmoothTrajectoryMovingAverage<std::vector<cv::Vec2d>>(positions, smoothing_window) : positions;

  if (trajectory_length == 0)
    return;

  if (trajectory_length < 0)
    trajectory_length = trajectory.size();

  int trace_from = newest_position_first ? 0
        : std::max(0, static_cast<int>(trajectory.size()) - trajectory_length);
  int trace_to = newest_position_first ? std::min(static_cast<int>(trajectory.size()) - 1, trajectory_length)
        : static_cast<int>(trajectory.size()) - 1;

  if (trace_to == trace_from)
    return;

  if (max_line_width < 2)
    max_line_width = 2;

  cv::Point pt1 = vcp::convert::ToPoint(trajectory[trace_from]);
  double progress = 0.0;
  const double progress_inc = 1.0 / (trace_to - trace_from);

  for (int i = trace_from + 1; i <= trace_to; ++i)
  {
    const cv::Point pt2 = vcp::convert::ToPoint(trajectory[i]);
    const int thickness = newest_position_first ? static_cast<int>((1.0-progress) * (max_line_width - 1)) + 1
          : static_cast<int>(progress * (max_line_width - 1)) + 1;
    const cv::Scalar color = newest_position_first ? ((1.0-progress) * obj_color + progress * fade_color)
          : (progress * obj_color + (1.0-progress) * fade_color);

    if (dash_length > 0)
      vcp::imvis::drawing::DrawDashedLine(pt1, pt2, color, dash_length, thickness, image);
    else
      cv::line(image, pt1, pt2, color, thickness);

    pt1 = pt2;
    progress += progress_inc;
  }
}


void DrawTrajectory2d(cv::Mat &image, const std::vector<cv::Vec2d> &positions, bool newest_position_first, int smoothing_window, int trajectory_length, const cv::Scalar &obj_color, int line_width, int dash_length)
{
  const std::vector<cv::Vec2d> trajectory = smoothing_window > 0 ? SmoothTrajectoryMovingAverage<std::vector<cv::Vec2d>>(positions, smoothing_window) : positions;

  if (trajectory_length == 0)
    return;

  if (trajectory_length < 0)
    trajectory_length = trajectory.size();

  int trace_from = newest_position_first ? 0
        : std::max(0, static_cast<int>(trajectory.size()) - trajectory_length);
  int trace_to = newest_position_first ? std::min(static_cast<int>(trajectory.size()) - 1, trajectory_length)
        : static_cast<int>(trajectory.size()) - 1;

  if (trace_to == trace_from)
    return;

  cv::Point pt1 = vcp::convert::ToPoint(trajectory[trace_from]);

  for (int i = trace_from + 1; i <= trace_to; ++i)
  {
    const cv::Point pt2 = vcp::convert::ToPoint(trajectory[i]);

    if (dash_length > 0)
      vcp::imvis::drawing::DrawDashedLine(pt1, pt2, obj_color, dash_length, line_width, image);
    else
      cv::line(image, pt1, pt2, obj_color, line_width);

    pt1 = pt2;
  }
}


cv::RotatedRect GetErrorEllipse(double chisquare_val, const cv::Vec2d &mean, const cv::Mat &cov_mat)
{
  // See http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/
  // Get the eigenvalues and eigenvectors.
  cv::Mat eigenvalues, eigenvectors;
  cv::eigen(cov_mat, eigenvalues, eigenvectors);

  // Calculate the angle between the largest eigenvector and the x-axis
  double angle = atan2(eigenvectors.at<double>(0,1), eigenvectors.at<double>(0,0));

  //Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
  if (angle < 0)
  {
    angle += 6.28318530718;
  }

  // Calculate the size of the minor and major axes.
  double major_axis_half = chisquare_val * std::sqrt(eigenvalues.at<double>(0));
  double minor_axis_half = chisquare_val * std::sqrt(eigenvalues.at<double>(1));

  // Return the oriented ellipse (OpenCV defines the angle clockwise instead of anti-clockwise, thus use the negative angle).
  return cv::RotatedRect(cv::Point2f(mean[0], mean[1]), cv::Size2f(major_axis_half, minor_axis_half), -vcp::math::Rad2Deg(angle));
}

// Error elipse at 90 % confidence
cv::RotatedRect GetErrorEllipse90(const cv::Vec2d &mean, const cv::Mat &cov_mat)
{
  return GetErrorEllipse(std::sqrt(4.605), mean, cov_mat);
}

// Error elipse at 95 % confidence
cv::RotatedRect GetErrorEllipse95(const cv::Vec2d &mean, const cv::Mat &cov_mat)
{
  return GetErrorEllipse(std::sqrt(5.991), mean, cov_mat);
}

// Error elipse at 99 % confidence
cv::RotatedRect GetErrorEllipse99(const cv::Vec2d &mean, const cv::Mat &cov_mat)
{
  return GetErrorEllipse(std::sqrt(9.210), mean, cov_mat);
}


void DrawErrorEllipse(cv::Mat &image, const cv::Vec2d &mean, const cv::Mat &cov_mat, const cv::Scalar &color, double chisquare_val, int line_width, double fill_opacity)
{
  const cv::RotatedRect r = GetErrorEllipse(chisquare_val, mean, cov_mat);
  vcp::imvis::drawing::DrawEllipse(image, r, color, line_width, fill_opacity);
}


void DrawErrorEllipse90(cv::Mat &image, const cv::Vec2d &mean, const cv::Mat &cov_mat, const cv::Scalar &color, int line_width, double fill_opacity)
{
  const cv::RotatedRect r = GetErrorEllipse90(mean, cov_mat);
  vcp::imvis::drawing::DrawEllipse(image, r, color, line_width, fill_opacity);
}


void DrawErrorEllipse95(cv::Mat &image, const cv::Vec2d &mean, const cv::Mat &cov_mat, const cv::Scalar &color, int line_width, double fill_opacity)
{
  const cv::RotatedRect r = GetErrorEllipse95(mean, cov_mat);
  vcp::imvis::drawing::DrawEllipse(image, r, color, line_width, fill_opacity);
}


void DrawErrorEllipse99(cv::Mat &image, const cv::Vec2d &mean, const cv::Mat &cov_mat, const cv::Scalar &color, int line_width, double fill_opacity)
{
  const cv::RotatedRect r = GetErrorEllipse99(mean, cov_mat);
  vcp::imvis::drawing::DrawEllipse(image, r, color, line_width, fill_opacity);
}

} // namespace trajectories
} // namespace imvis
} // namespace vcp
