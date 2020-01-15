#ifndef __VCP_OPENCV_COMPATIBILITY_H__
#define __VCP_OPENCV_COMPATIBILITY_H__

// TODO: should we move this file to a more common module??
//   => vcp_utils doesn't depend on OpenCV and I like to keep it that way! there is no "common vcp/opencv module" and just creating it for a simple header is overkill

/** We're currently developing with Versions 2.4.x-3.4.x, so some APIs are not available on all installations. */

#include <opencv2/core/core.hpp>

#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >= 3)
    #define CV_SIZE_HAS_EMPTY
#endif

#if CV_MAJOR_VERSION < 3
namespace cv
{
typedef Rect_<double> Rect2d;
}
#endif

/** @brief Checks, if the given size has valid dimensions, i.e. W,H > 0. */
inline bool IsValidSize(const cv::Size &sz)
{
#ifdef CV_SIZE_HAS_EMPTY
  return !sz.empty() && sz.width > 0 && sz.height > 0;
#else
  return sz.width > 0 && sz.height > 0;
#endif
}

#endif // __VCP_OPENCV_COMPATIBILITY_H__
