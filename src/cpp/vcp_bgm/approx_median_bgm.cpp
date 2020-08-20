#include "approx_median_bgm.h"
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/timing_utils.h>

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/imgproc/imgproc.hpp>
#else
    #include <opencv2/imgproc.hpp>
#endif

#include <vcp_imutils/matutils.h>
#include <cmath>

namespace vcp
{
namespace bgm
{
namespace
{
void OrChannels(const cv::Mat& src, cv::Mat& dst)
{
  if (src.type() != CV_8UC3)
    VCP_ERROR("Input image to OrChannels() must be CV_8UC3, not "
              << vcp::imutils::CVMatDepthToString(src.depth(), src.channels()));
  int rows = src.rows;
  int cols = src.cols;

  dst.create(src.size(), CV_8UC1);

  if (src.isContinuous() && dst.isContinuous())
  {
    cols = rows * cols;
    rows = 1;
  }

  for (int row = 0; row < rows; row++)
  {
    const uchar* src_ptr = src.ptr<uchar>(row);
    uchar* dst_ptr = dst.ptr<uchar>(row);

    for (int col = 0; col < cols; col++)
    {
      dst_ptr[col] = src_ptr[0] | src_ptr[1] | src_ptr[2];
      src_ptr += 3;
    }
  }
}

/** @brief Generic template (quite slow) to convert a single pixel to grayscale. */
template<typename T, int Channels>
inline float ConvertToGray(const T *ptr, int col)
{
  switch (Channels)
  {
  case 1:
    return static_cast<float>(ptr[col]);

  case 3:
    return 0.114 * static_cast<float>(ptr[col]) + 0.587 * static_cast<float>(ptr[col + 1]) + 0.299 * static_cast<float>(ptr[col + 2]);

  default:
    VCP_ERROR("Number of channels ('" << Channels << "') is not supported");
  }
}

/** @brief Template specialization for uchar's, based on cv::cvtColor(). */
template<>
inline float ConvertToGray<uchar,3>(const uchar *ptr, int col)
{
  const int yuv_shift = 14;
  const int r2y = 4899;
  const int g2y = 9617;
  const int b2y = 1868;
  const int yb = static_cast<int>(ptr[col]) * b2y;
  const int yg = static_cast<int>(ptr[col+1]) * g2y;
  const int yr = static_cast<int>(ptr[col+2]) * r2y;
  const uchar gray = static_cast<uchar>((yb + yg + yr) >> yuv_shift);
  return static_cast<float>(gray);
}

template<>
inline float ConvertToGray<uchar,1>(const uchar *ptr, int col)
{
  return static_cast<float>(ptr[col]);
}

template<>
inline float ConvertToGray<float,1>(const float *ptr, int col)
{
  return ptr[col];
}

template<typename T, int Channels>
void ReportGrayscale(const cv::Mat &current_image, float threshold, float adaption_step, bool update, cv::Mat &bg, cv::Mat &mask)
{
  // Allocate output mask.
  mask.create(current_image.rows, current_image.cols, CV_8UC1);

  // Optimize loop indices for traversal if all matrices are continuous.
  int rows = current_image.rows;
  int cols = current_image.cols;
  if (current_image.isContinuous() && mask.isContinuous() && bg.isContinuous())
  {
    cols *= rows;
    rows = 1;
  }

  for (int row = 0; row < rows; ++row)
  {
    const T *ptr_image = current_image.ptr<T>(row);
    uchar *ptr_mask = mask.ptr<uchar>(row);
    float *ptr_bg = bg.ptr<float>(row);

    for (int col = 0, col_img = 0; col < cols; ++col, col_img += Channels)
    {
      const float bg_val = ptr_bg[col];
      const float img_val = ConvertToGray<T, Channels>(ptr_image, col_img);

      // Report foreground if absolute difference is above threshold.
      const bool is_fg = std::fabs(img_val - bg_val) > threshold;
      ptr_mask[col] = is_fg ? 255 : 0;

      if (update)
      {
        // Increase/decrease background representation by adaption step.
        if (bg_val < img_val)
        {
          ptr_bg[col] += adaption_step;
        }
        else if (bg_val > img_val)
        {
          ptr_bg[col] -= adaption_step;
        }

        // Clip to [0, 255].
        if (ptr_bg[col] < 0.0f)
        {
          ptr_bg[col] = 0.0f;
        }
        else if (ptr_bg[col] > 255.0f)
        {
          ptr_bg[col] = 255.0f;
        }
      }
    }
  }
}


template<typename T>
void ReportRgb(const cv::Mat &current_image, float threshold, float adaption_step, bool update, cv::Mat &bg, cv::Mat &mask)
{
  // Allocate output mask.
  cv::Mat mask_rgb;
  mask_rgb.create(current_image.rows, current_image.cols, CV_8UC3);
  // Optimize loop indices for traversal if all matrices are continuous.
  int rows = current_image.rows;
  int cols = current_image.cols;
  if (current_image.isContinuous() && mask_rgb.isContinuous() && bg.isContinuous())
  {
    cols *= rows;
    rows = 1;
  }
  for (int row = 0; row < rows; ++row)
  {
    const T *ptr_image = current_image.ptr<T>(row);
    uchar *ptr_mask = mask_rgb.ptr<uchar>(row);
    float *ptr_bg = bg.ptr<float>(row);

    for (int col = 0; col < cols * 3; ++col)
    {
      const float bg_val = ptr_bg[col];
      const float img_val = ptr_image[col];

      // Report foreground if absolute difference is above threshold.
      const bool is_fg = std::fabs(img_val - bg_val) > threshold;
      ptr_mask[col] = is_fg ? 255 : 0;

      if (update)
      {
        // Increase/decrease background representation by adaption step.
        if (bg_val < img_val)
        {
          ptr_bg[col] += adaption_step;
        }
        else if (bg_val > img_val)
        {
          ptr_bg[col] -= adaption_step;
        }

        // Clip to [0, 255].
        if (ptr_bg[col] < 0.0f)
        {
          ptr_bg[col] = 0.0f;
        }
        else if (ptr_bg[col] > 255.0f)
        {
          ptr_bg[col] = 255.0f;
        }
      }
    }
  }

  // Reduce 3-channel mask to single channel:
  OrChannels(mask_rgb, mask);
}
} // namespace

// Specify number of channels (1 or 3)
template<int N>
class ApproxMedianBackgroundModel : public BackgroundModel
{
public:
  ApproxMedianBackgroundModel(const ApproxMedianBgmParams &params)
    : BackgroundModel(), params_(params), approx_median_(cv::Mat())
  {}

  virtual ~ApproxMedianBackgroundModel()
  {}

  bool Init(const cv::Mat &initial_bg_img) override
  {
    if (N == 1)
    {
      if (initial_bg_img.depth() == CV_32F)
      {
        // TODO should we support RGBA? --> Nope.
        if (initial_bg_img.channels() == 1)
        {
          initial_bg_img.copyTo(approx_median_);
        }
        else
        {
          cv::cvtColor(initial_bg_img, approx_median_, CV_BGR2GRAY);
        }
      }
      else
      {
        if (initial_bg_img.channels() == 1)
        {
          initial_bg_img.convertTo(approx_median_, CV_32F);
        }
        else
        {
          cv::Mat single_channel;
          cv::cvtColor(initial_bg_img, single_channel, CV_BGR2GRAY);
          single_channel.convertTo(approx_median_, CV_32F);
        }
      }
    }
    else if (N == 3)
    {
      if (initial_bg_img.depth() == CV_32F)
      {
        initial_bg_img.copyTo(approx_median_);
      }
      else
      {
        initial_bg_img.convertTo(approx_median_, CV_32F);
      }
    }
    else
      VCP_ERROR("Invalid template parameter N=" << N);

    return true;
  }

  cv::Mat ReportChanges(const cv::Mat &current_image, bool update_model, const cv::Mat &update_mask=cv::Mat()) override
  {
    // Naive vs templated single-pass ("Optimized").
    // Average times to process 1024x768:
    // * 8UC3
    //   o Without model update:
    //     Naive     18 ms
    //     Optimized  2 ms
    //   o With model update:
    //     Naive     25 ms
    //     Optimized  3 ms
    // * 8UC1
    //   o w/o update 15-25ms vs 0.4ms
    //   o with update 20-22ms vs 1.8ms

    VCP_INIT_TIC_TOC;
    VCP_TIC;
    if (!update_mask.empty())
      VCP_LOG_FAILURE("ApproxMedianBgm doesn't support masking update regions.");

    cv::Mat mask;
    if (N == 1)
    {
      switch (current_image.type())
      {
      case CV_8UC1:
        ReportGrayscale<uchar,1>(current_image, params_.threshold, params_.adaption_step, update_model, approx_median_, mask);
        break;

      case CV_8UC3:
        ReportGrayscale<uchar,3>(current_image, params_.threshold, params_.adaption_step, update_model, approx_median_, mask);
        break;

      case CV_32FC1:
        ReportGrayscale<float,1>(current_image, params_.threshold, params_.adaption_step, update_model, approx_median_, mask);
        break;

      case CV_32FC3:
        ReportGrayscale<float,3>(current_image, params_.threshold, params_.adaption_step, update_model, approx_median_, mask);
        break;

      default:
        VCP_ERROR("Input image type '"
                  << vcp::imutils::CVMatDepthToString(current_image.depth(), current_image.channels())
                  << "' not supported for 1-channel background model!");
        break;
      }
    }
    else if (N == 3)
    {
      switch (current_image.type())
      {
      case CV_8UC3:
        ReportRgb<uchar>(current_image, params_.threshold, params_.adaption_step, update_model, approx_median_, mask);
        break;

      case CV_32FC3:
        ReportRgb<float>(current_image, params_.threshold, params_.adaption_step, update_model, approx_median_, mask);
        break;

      default:
          VCP_ERROR("Input image type '"
                    << vcp::imutils::CVMatDepthToString(current_image.depth(), current_image.channels())
                    << "' not supported for 3-channel background model!");
        break;
      }
    }
    else
      VCP_ERROR("Invalid template parameter N=" << N);
    VCP_TOC("ApproxMedianBgm::ReportChanges");

  /*#if false
    // Naive version.
    VCP_TIC;

    cv::Mat diff;
    cv::Mat curr32;
    if (current_image.depth() == CV_32F) {
      if (current_image.channels() == 1) {
        current_image.copyTo(curr32);
      } else {
        cv::cvtColor(current_image, curr32, CV_BGR2GRAY);
      }
    } else {
      if (current_image.channels() == 1) {
        current_image.convertTo(curr32, CV_32F);
      } else {
        cv::Mat single_channel;
        cv::cvtColor(current_image, single_channel, CV_BGR2GRAY);
        single_channel.convertTo(curr32, CV_32F);
      }
    }

    cv::absdiff(curr32, approx_median_, diff);

    mask = diff > threshold_;
    if (update_model)
    {
      for (int row = 0; row < approx_median_.rows; ++row)
      {
        for (int col = 0; col < approx_median_.cols; ++col)
        {
          if (approx_median_.at<float>(row, col) < curr32.at<float>(row, col))
            approx_median_.at<float>(row, col) += adaption_step_;
          else if (approx_median_.at<float>(row, col) > curr32.at<float>(row, col))
            approx_median_.at<float>(row, col) -= adaption_step_;

          if (approx_median_.at<float>(row, col) < 0.0f)
            approx_median_.at<float>(row, col) = 0.0f;
          else if (approx_median_.at<float>(row, col) > 255.0f)
            approx_median_.at<float>(row, col) = 255.0f;
        }
      }
    }
    VP_TOC("ApproxMedian::naive");
  #endif*/
    return mask;
  }

  cv::Mat BackgroundImage() override
  {
    cv::Mat background;
    approx_median_.convertTo(background, CV_8U);
    return background;
  }

  bool UpdateParameters(const BgmParams &params) override
  {
    try
    {
      const ApproxMedianBgmParams &p = dynamic_cast<const ApproxMedianBgmParams &>(params);
      params_ = p;
      return true;
    }
    catch (std::bad_cast)
    {
      VCP_ERROR("Parameters are not of type ApproxMedianBgmParams.");
    }
    return false;
  }

  BgmParams Parameters() const override
  {
    return params_;
  }

private:
  ApproxMedianBgmParams params_;
  cv::Mat approx_median_;
};


std::unique_ptr<BackgroundModel> CreateApproxMedianBgmGrayscale(const ApproxMedianBgmParams &params)
{
  return std::unique_ptr<ApproxMedianBackgroundModel<1>>(new ApproxMedianBackgroundModel<1>(params));
}

std::unique_ptr<BackgroundModel> CreateApproxMedianBgmColor(const ApproxMedianBgmParams &params)
{
  return std::unique_ptr<ApproxMedianBackgroundModel<3>>(new ApproxMedianBackgroundModel<3>(params));
}

} // namespace bgm
} // namespace vcp
