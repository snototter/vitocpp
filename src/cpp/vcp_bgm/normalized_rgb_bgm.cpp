#include "normalized_rgb_bgm.h"
#include <vcp_utils/timing_utils.h>
#include <vcp_utils/vcp_error.h>

namespace vcp
{
namespace bgm
{
class NormalizedRGBBackgroundModel : public BackgroundModel
{
public:
  NormalizedRGBBackgroundModel(const NormalizedRgbBgmParams &params)
    : BackgroundModel(), params_(params), background_image_(cv::Mat())
  {}


  bool Init(const cv::Mat &initial_bg_img) override
  {
    background_image_ = cv::Mat();
    return UpdateModel(initial_bg_img, cv::Mat());
  }


  cv::Mat ReportChanges(const cv::Mat &current_image, bool update_model, const cv::Mat &update_mask=cv::Mat()) override
  {
    VCP_INIT_TIC_TOC;
    cv::Mat IF, nF, distance;

    VCP_TIC;
    if (current_image.depth() == CV_8U)
      current_image.convertTo(IF, CV_64F, 1.0/255.0);
    //TODO else if 64F & Co?
    else
      current_image.copyTo(IF);

    std::vector<cv::Mat> IFc(3);
    std::vector<cv::Mat> nFc(3);

    cv::split(IF, IFc);

    cv::pow(IFc[0], 2.0, nFc[0]);
    cv::pow(IFc[1], 2.0, nFc[1]);
    cv::pow(IFc[2], 2.0, nFc[2]);

    cv::sqrt(nFc[0] + nFc[1] + nFc[2], nF);

  //      TOC("NormRGB - Intensity normalizers");
    // eqs 17/18 done

  //      TIC;
    cv::Mat term2, nDiffSquared;
    cv::pow(nF - nB_, 2.0, nDiffSquared);
    term2 = params_.beta*nDiffSquared;


    cv::Mat term1, nF_alpha;
    nF_alpha = nF + cv::Scalar(params_.alpha);
    std::vector<cv::Mat> squared_diffs(3);
    for (int c = 0; c < 3; ++c)
    {
      const cv::Mat tmp = (IFc[c] / nF_alpha) - IBdivNB_[c];
      cv::pow(tmp, 2.0, squared_diffs[c]);
    }
    cv::sqrt(squared_diffs[0] + squared_diffs[1] + squared_diffs[2], term1);

    distance = (term1 + term2) / (1.0f + params_.beta);

    VCP_TOC("NormRGB - Report");

    cv::Mat mask;
    if (params_.apply_threshold)
      mask = distance > params_.threshold;
    else
      mask = distance;

    if (update_model)
      UpdateModel(IF, update_mask);

    return mask;
  }


  cv::Mat BackgroundImage() override
  {
    return background_image_.clone();
  }


  bool UpdateParameters(const BgmParams &params) override
  {
    try
    {
      const NormalizedRgbBgmParams &p = dynamic_cast<const NormalizedRgbBgmParams &>(params);
      params_ = p;
      return true;
    }
    catch (std::bad_cast)
    {
      VCP_ERROR("Parameters are not of type NormalizedRgbBgmParams.");
    }
    return false;
  }

  BgmParams Parameters() const override
  {
    return params_;
  }

private:
  NormalizedRgbBgmParams params_;
  cv::Mat background_image_; /**< The background image/model. */
  cv::Mat nB_; /**< Intensity normalizer of the background image. */
  std::vector<cv::Mat> IBdivNB_; /**< Holds the 3 separate channels of the term  \f$ \frac{I_{B,c}(\vec{x})}{n_B(\vec{x})+\alpha} \f$, used to improve the performance of the <code>report</code> functionality. */

  bool UpdateModel(const cv::Mat &current_image, const cv::Mat &mask)
  {
    cv::Mat input;
    VCP_INIT_TIC_TOC;
    VCP_TIC;

    assert(current_image.channels() == 3);
    if (current_image.depth() == CV_8U)
      current_image.convertTo(input, CV_64F, 1.0/255.0);
    else
      current_image.copyTo(input);

    if (background_image_.empty())
    {
      input.copyTo(background_image_);
    }
    else
    {
      cv::Mat weighted_sum;
      cv::addWeighted(background_image_, 1.0-params_.learning_rate, input, params_.learning_rate, 0.0, weighted_sum);
      weighted_sum.copyTo(background_image_, mask);
    }

    // Eq 18
    std::vector<cv::Mat> IBc(3);
    std::vector<cv::Mat> nBc(3);
    cv::split(background_image_, IBc);

    cv::pow(IBc[0], 2.0, nBc[0]);
    cv::pow(IBc[1], 2.0, nBc[1]);
    cv::pow(IBc[2], 2.0, nBc[2]);

    cv::sqrt(nBc[0] + nBc[1] + nBc[2], nB_);
    // Prepare background related variables for eq 16
    cv::Mat nB_alpha = nB_ + cv::Scalar(params_.alpha);

    IBdivNB_.clear();
    for (int c = 0; c < 3; ++c)
      IBdivNB_.push_back(IBc[c] / nB_alpha);
    VCP_TOC("NormRGB - BG update");
    return true;
  }
};


std::unique_ptr<BackgroundModel> CreateNormalizedRgbBgm(const NormalizedRgbBgmParams &params)
{
  return std::unique_ptr<NormalizedRGBBackgroundModel>(new NormalizedRGBBackgroundModel(params));
}

} // end namespace bgm
} // end namespace vcp
