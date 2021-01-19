#ifndef __VCP_OPENCV_COMPATIBILITY_H__
#define __VCP_OPENCV_COMPATIBILITY_H__

// should we move this file to a "more common" module (vcp_utils)? => vcp_utils doesn't depend on OpenCV and I like to keep it that way! there is no "common vcp/opencv module" and just creating it for a simple header is overkill

/** We're currently developing with Versions 2.4.x-3.4.x, so some APIs are not available on all installations. */

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR == 1
  #error "OpenCV version 1.x is not supported"
#elif CV_VERSION_MAJOR == 2
  #include <opencv2/core/core.hpp>
#elif CV_VERSION_MAJOR == 3 or CV_VERSION_MAJOR == 4
  #include <opencv2/core.hpp>
#else
  #error "OpenCV version > 4 is not YET supported"
#endif

#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION >= 3)
    #define CV_SIZE_HAS_EMPTY
#endif

#if CV_MAJOR_VERSION < 3
/** @brief "Future definitions" to support OpenCV 2.x. */
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

// OpenCV 4 changed the constants for cvtColor and mouse events
//// Wrap cvtColor constants
#if CV_VERSION_MAJOR < 3
//FIXME
#define CVTCOLOR_BGR2GRAY CV_BGR2GRAY
#define CVTCOLOR_BGRA2BGR CV_BGRA2BGR
#define CVTCOLOR_BGRA2RGB CV_BGRA2RGB
#define CVTCOLOR_RGB2HSV CV_RGB2HSV

#else
#define CVTCOLOR_BGR2GRAY cv::COLOR_BGR2GRAY
#define CVTCOLOR_RGB2GRAY cv::COLOR_RGB2GRAY
#define CVTCOLOR_BGRA2GRAY cv::COLOR_BGRA2GRAY
#define CVTCOLOR_RGBA2GRAY cv::COLOR_RGBA2GRAY

#define CVTCOLOR_BGRA2BGR cv::COLOR_BGRA2BGR
#define CVTCOLOR_BGRA2RGB cv::COLOR_BGRA2RGB
#define CVTCOLOR_BGRA2RGBA cv::COLOR_BGRA2RGBA

#define CVTCOLOR_RGBA2BGR cv::COLOR_RGBA2BGR
#define CVTCOLOR_RGBA2RGB cv::COLOR_RGBA2RGB
#define CVTCOLOR_RGBA2BGRA cv::COLOR_RGBA2BGRA

#define CVTCOLOR_BGR2RGB cv::COLOR_BGR2RGB
#define CVTCOLOR_BGR2BGRA cv::COLOR_BGR2BGRA
#define CVTCOLOR_RGB2BGR cv::COLOR_RGB2BGR
#define CVTCOLOR_RGB2BGRA cv::COLOR_RGB2BGRA

#define CVTCOLOR_RGB2HSV cv::COLOR_RGB2HSV
#define CVTCOLOR_BGR2HSV cv::COLOR_BGR2HSV
#define CVTCOLOR_GRAY2RGB cv::COLOR_GRAY2RGB
#define CVTCOLOR_GRAY2BGR cv::COLOR_GRAY2BGR

#define CVTCOLOR_RGB2YCrCb cv::COLOR_RGB2YCrCb
#define CVTCOLOR_BGR2YCrCb cv::COLOR_BGR2YCrCb
#define CVTCOLOR_YCrCb2RGB cv::COLOR_YCrCb2RGB
#define CVTCOLOR_YCrCb2BGR cv::COLOR_YCrCb2BGR

#define CVTCOLOR_RGB2Lab cv::COLOR_RGB2Lab
#define CVTCOLOR_BGR2Lab cv::COLOR_BGR2Lab
#endif

#endif // __VCP_OPENCV_COMPATIBILITY_H__
