#ifndef __VCP_BEST_RGBD_H__
#define __VCP_BEST_RGBD_H__

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <memory>

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/core/core.hpp>
#else
    #include <opencv2/core.hpp>
#endif

namespace vcp
{
namespace best
{

/** @brief Functionality to align depth to color images. */
namespace rgbd
{

/** @brief Manually aligns depth to RGB in a calibrated RGBD stereo pair. */
class RgbdAlignment
{
public:
  RgbdAlignment(const cv::Mat& K_c, const cv::Mat& K_d,
                const cv::Mat& R_d2c, const cv::Mat& t_d2c,
                const cv::Size& size_c, const cv::Size& size_d,
                const cv::Mat& D_c, const cv::Mat& D_d);

  virtual ~RgbdAlignment() {}

  virtual cv::Mat AlignDepth2Color(const cv::Mat& depth) = 0;

  virtual void AlignDepthIR2Color(const cv::Mat& depth, const cv::Mat& ir,
                                  cv::Mat& aligned_depth, cv::Mat& aligned_ir) = 0;

  /** @brief Returns true if this object is properly configured, all external libraries/drivers are usable, etc. */
  virtual bool IsAlignmentValid() const = 0;

protected:
  cv::Mat K_c_;
  cv::Mat K_d_;
  cv::Mat R_d2c_;
  cv::Mat t_d2c_;
  cv::Size size_c_;
  cv::Size size_d_;
  cv::Mat D_c_;
  cv::Mat D_d_;
};

/** @brief Initialize an RgbdAlignment instance to align depth to color imagery. */
std::unique_ptr<RgbdAlignment> CreateRgbdAlignment(const cv::Mat& K_c, const cv::Mat& K_d,
                                                   const cv::Mat& R_d2c, const cv::Mat& t_d2c,
                                                   const cv::Size& size_c, const cv::Size& size_d,
                                                   const cv::Mat& D_c=cv::Mat(), const cv::Mat& D_d=cv::Mat());
} // namespace rgbd
} // namespace best
} // namespace vcp
#endif // __VCP_BEST_RGBD_H__
