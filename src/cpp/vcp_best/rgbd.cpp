#include "rgbd.h"

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>

#ifdef VCP_BEST_WITH_K4A
  #include "k4a_sink.h"
#endif

namespace vcp
{
namespace best
{
namespace rgbd
{
RgbdAlignment::RgbdAlignment(const cv::Mat& K_c, const cv::Mat& K_d,
                             const cv::Mat& R_d2c, const cv::Mat& t_d2c,
                             const cv::Size& size_c, const cv::Size& size_d,
                             const cv::Mat& D_c=cv::Mat(), const cv::Mat& D_d=cv::Mat())
{
  K_c_ = K_c.clone();
  K_d_ = K_d.clone();
  R_d2c_ = R_d2c.clone();
  t_d2c_ = t_d2c.clone();
  size_c_ = size_c;
  size_d_ = size_d;
  D_c_ = D_c.clone();
  D_d_ = D_d.clone();
}

//TODO add function (to calibration.*!) to create a standardized calibration struct from distortion coeffs of different length (8 k4a, 5 opencv)

std::unique_ptr<RgbdAlignment> CreateRgbdAlignment(const cv::Mat& K_c, const cv::Mat& K_d,
                                                   const cv::Mat& R_d2c, const cv::Mat& t_d2c,
                                                   const cv::Size& size_c, const cv::Size& size_d,
                                                   const cv::Mat& D_c, const cv::Mat& D_d)
{
#ifdef VCP_BEST_WITH_K4A
  return k4a::CreateK4ARgbdAlignment(K_c, K_d, R_d2c, t_d2c, size_c, size_d, D_c, D_d);
#else
  VCP_LOG_FAILURE("No RgbdAlignment implementation is available, try rebuilding vcp with e.g. K4A SDK.");
  return nullptr;
#endif
}

} // namespace rgbd
} // namespace best
} // namespace vcp
