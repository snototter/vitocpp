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

// OpenCV 4 dropped some deprecated constants
//// Wrap cvtColor constants
#if CV_VERSION_MAJOR < 3
  #define CVTCOLOR_BGR2GRAY CV_BGR2GRAY
  #define CVTCOLOR_RGB2GRAY CV_RGB2GRAY
  #define CVTCOLOR_BGRA2GRAY CV_BGRA2GRAY
  #define CVTCOLOR_RGBA2GRAY CV_RGBA2GRAY

  #define CVTCOLOR_BGRA2BGR CV_BGRA2BGR
  #define CVTCOLOR_BGRA2RGB CV_BGRA2RGB
  #define CVTCOLOR_BGRA2RGBA CV_BGRA2RGBA

  #define CVTCOLOR_RGBA2BGR CV_RGBA2BGR
  #define CVTCOLOR_RGBA2RGB CV_RGBA2RGB
  #define CVTCOLOR_RGBA2BGRA CV_RGBA2BGRA

  #define CVTCOLOR_BGR2RGB CV_BGR2RGB
  #define CVTCOLOR_BGR2BGRA CV_BGR2BGRA
  #define CVTCOLOR_RGB2BGR CV_RGB2BGR
  #define CVTCOLOR_RGB2BGRA CV_RGB2BGRA

  #define CVTCOLOR_RGB2HSV CV_RGB2HSV
  #define CVTCOLOR_BGR2HSV CV_BGR2HSV
  #define CVTCOLOR_GRAY2RGB CV_GRAY2RGB
  #define CVTCOLOR_GRAY2BGR CV_GRAY2BGR

  #define CVTCOLOR_RGB2YCrCb CV_RGB2YCrCb
  #define CVTCOLOR_BGR2YCrCb CV_BGR2YCrCb
  #define CVTCOLOR_YCrCb2RGB CV_YCrCb2RGB
  #define CVTCOLOR_YCrCb2BGR CV_YCrCb2BGR

  #define CVTCOLOR_RGB2Lab CV_RGB2Lab
  #define CVTCOLOR_BGR2Lab CV_BGR2Lab

#else
  // Color conversion codes for OpenCV 3 & 4
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

//// Wrap CAPTURE PROP constants
#if CV_VERSION_MAJOR < 3
  #define COMPAT_CV_CAP_PROP_POS_FRAMES CV_CAP_PROP_POS_FRAMES
#else
  #define COMPAT_CV_CAP_PROP_POS_FRAMES cv::CAP_PROP_POS_FRAMES
#endif

//// Wrap image loading/decoding flags
#if CV_VERSION_MAJOR < 3
  #define COMPAT_CV_LOAD_IMAGE_UNCHANGED CV_LOAD_IMAGE_UNCHANGED
  #define COMPAT_CV_LOAD_IMAGE_GRAYSCALE CV_LOAD_IMAGE_GRAYSCALE
  #define COMPAT_CV_LOAD_IMAGE_COLOR CV_LOAD_IMAGE_COLOR
  #define COMPAT_CV_LOAD_IMAGE_ANYDEPTH CV_LOAD_IMAGE_ANYDEPTH
  #define COMPAT_CV_LOAD_IMAGE_ANYCOLOR CV_LOAD_IMAGE_ANYCOLOR
#else
  #define COMPAT_CV_LOAD_IMAGE_UNCHANGED cv::IMREAD_UNCHANGED
  #define COMPAT_CV_LOAD_IMAGE_GRAYSCALE cv::IMREAD_GRAYSCALE
  #define COMPAT_CV_LOAD_IMAGE_COLOR cv::IMREAD_COLOR
  #define COMPAT_CV_LOAD_IMAGE_ANYDEPTH cv::IMREAD_ANYDEPTH
  #define COMPAT_CV_LOAD_IMAGE_ANYCOLOR cv::IMREAD_ANYCOLOR
#endif


//// Wrap UI events
#if CV_VERSION_MAJOR < 3
  #define COMPAT_CV_EVENT_LBUTTONDOWN CV_EVENT_LBUTTONDOWN
  #define COMPAT_CV_EVENT_LBUTTONUP CV_EVENT_LBUTTONUP
  #define COMPAT_CV_EVENT_MBUTTONUP CV_EVENT_MBUTTONUP
  #define COMPAT_CV_EVENT_RBUTTONUP CV_EVENT_RBUTTONUP
  #define COMPAT_CV_EVENT_MOUSEMOVE CV_EVENT_MOUSEMOVE
#else
  #define COMPAT_CV_EVENT_LBUTTONDOWN cv::EVENT_LBUTTONDOWN
  #define COMPAT_CV_EVENT_LBUTTONUP cv::EVENT_LBUTTONUP
  #define COMPAT_CV_EVENT_MBUTTONUP cv::EVENT_MBUTTONUP
  #define COMPAT_CV_EVENT_RBUTTONUP cv::EVENT_RBUTTONUP
  #define COMPAT_CV_EVENT_MOUSEMOVE cv::EVENT_MOUSEMOVE
#endif

#endif // __VCP_OPENCV_COMPATIBILITY_H__
