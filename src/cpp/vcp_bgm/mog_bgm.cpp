#include "mog_bgm.h"
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/timing_utils.h>

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/imgproc/imgproc.hpp>
    #include <opencv2/video/video.hpp>
#else
    #include <opencv2/imgproc.hpp>
    #include <opencv2/video.hpp>
#endif

#include <vcp_imutils/matutils.h>
#include <cmath>


namespace vcp
{
namespace bgm
{

// Specify number of channels (1 or 3)
class GMMBackgroundModel : public BackgroundModel
{
public:
  GMMBackgroundModel(const MixtureOfGaussiansBgmParams &params)
    : BackgroundModel(), params_(params),
      cvmog_(cv::createBackgroundSubtractorMOG2())
  {
    UpdateParameters(params);
  }

  virtual ~GMMBackgroundModel()
  {}

  bool Init(const cv::Mat &initial_bg_img) override
  {
    cv::Mat tmp;
    cvmog_->apply(initial_bg_img, tmp, 1.0); // Reset the model
    return true;
  }

  cv::Mat ReportChanges(const cv::Mat &current_image, bool update_model, const cv::Mat &update_mask=cv::Mat()) override
  {
    if (!update_mask.empty())
      VCP_LOG_FAILURE("GMMBackgroundModel doesn't support masking update regions.");
    cv::Mat mask;
    cvmog_->apply(current_image, mask, update_model ? -1.0 : 0.0);
    return mask;
  }

  cv::Mat BackgroundImage() override
  {
    VCP_LOG_FAILURE("GMMBackgroundModel doesn't support querying the background image.");
    return cv::Mat();
  }

  bool UpdateParameters(const BgmParams &params) override
  {
    try
    {
      const MixtureOfGaussiansBgmParams &p = dynamic_cast<const MixtureOfGaussiansBgmParams &>(params);
      params_ = p;
      cvmog_->setHistory(p.history);
      cvmog_->setDetectShadows(p.detect_shadows);
      cvmog_->setVarThreshold(p.variance_threshold);
      cvmog_->setComplexityReductionThreshold(p.complexity_reduction_threshold);
      return true;
    }
    catch (std::bad_cast)
    {
      VCP_ERROR("Parameters are not of type ApproxMedianBgmParams.");
    }
    return false;
  }

private:
  MixtureOfGaussiansBgmParams params_;
  cv::Ptr<cv::BackgroundSubtractorMOG2> cvmog_;
};


std::unique_ptr<BackgroundModel> CreateMixtureOfGaussiansBgm(const MixtureOfGaussiansBgmParams &params)
{
  return std::unique_ptr<GMMBackgroundModel>(new GMMBackgroundModel(params));
}

} // namespace bgm
} // namespace vcp
