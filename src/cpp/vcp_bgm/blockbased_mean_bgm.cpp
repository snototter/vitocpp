#include "blockbased_mean_bgm.h"
#include <vcp_utils/timing_utils.h>
#include <vcp_utils/vcp_error.h>

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/imgproc/imgproc.hpp>
#else
    #include <opencv2/imgproc.hpp>
#endif

namespace vcp
{
namespace bgm
{
class BlockBasedMeanBackgroundModel : public BackgroundModel
{
public:
  BlockBasedMeanBackgroundModel(const BlockBasedMeanBgmParams &params)
    : BackgroundModel(), params_(params), mean_(cv::Mat())
  {}

  virtual ~BlockBasedMeanBackgroundModel()
  {}

  bool Init(const cv::Mat &initial_bg_img) override
  {
    CreateModel(initial_bg_img, mean_);
    return true;
  }

  bool UpdateParameters(const BgmParams &params) override
  {
    try
    {
      const BlockBasedMeanBgmParams &p = dynamic_cast<const BlockBasedMeanBgmParams &>(params);
      params_ = p;
      return true;
    }
    catch (std::bad_cast)
    {
      VCP_ERROR("Parameters are not of type BlockBasedMeanBgmParams.");
    }
    return false;
  }


  cv::Mat ReportChanges(const cv::Mat &current_image, bool update_model, const cv::Mat &update_mask=cv::Mat()) override
  {
    cv::Mat mask;
    ReportChanges(current_image, mask, params_.report_threshold, update_model, update_mask);
    //FIXME we should return the mask as CV_8U (but float looks better)
        //FIXME TIC/TOC is muted
    double mi, ma;
    cv::minMaxLoc(mask, &mi, &ma);
    VCP_LOG_FAILURE("Min/max of blockbased mean foreground mask: " << mi << "..." << ma);
    VCP_LOG_FAILURE("FIXME: normalizing 32f mask!");
    //mask = mask / ma;
//    return mask;
    cv::Mat foo;
    mask.convertTo(foo, CV_8U);
    return foo;
  }


  cv::Mat BackgroundImage() override
  {
    cv::Mat bg;
    cv::normalize(mean_, bg, 1, 0, cv::NORM_MINMAX);
    return bg;
  }

  BgmParams Parameters() const override
  {
    return params_;
  }

private:
  BlockBasedMeanBgmParams params_;
  cv::Mat mean_;  /**< @brief The mean background model. */

  void CreateModel(const cv::Mat &frame, cv::Mat &mean)
  {
    // this function computes the mean values for each block in a given input image ("frame")
    // and writes these values to "mean". Therefore, you can think of traversing the whole image
    // with a block sized window, moving on for windowHeight (or Width respectively) * shift_factor
    // and computing the mean value of the pixels lying beyond the corresponging window.

    cv::Mat to_model;
    if (frame.channels() == 1)
      to_model = frame;
    else
    {
      if (params_.channel == BlockBasedMeanBgmChannel::GRAYSCALE)
      {
        cv::cvtColor(frame, to_model, CV_BGR2GRAY);
      }
      else if (params_.channel == BlockBasedMeanBgmChannel::SATURATION)
      {
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, CV_RGB2HSV);
        std::vector<cv::Mat> layers;
        cv::split(hsv, layers);

        to_model = layers[1]; // take saturation layer (should remove penumbra (soft shadows))
      }
      else
        VCP_ERROR("Invalid/unsupported channel type for BlockBasedMeanBgm (" << static_cast<int>(params_.channel) << ")");
    }

    const int rows = (to_model.rows - params_.block_size.height) / (params_.block_size.height * params_.shift) + 1;
    const int cols = (to_model.cols - params_.block_size.width) / (params_.block_size.width * params_.shift) + 1;

    mean.create(rows, cols, CV_32FC1);

    cv::Mat sum;
    cv::integral(to_model, sum, CV_32F);

    const float N = static_cast<float>(params_.block_size.width)
        * static_cast<float>(params_.block_size.height);

    for(int process = 0; process < rows*cols; ++process)
    {
      const int row = process / cols;
      const int col = process % cols;
      const cv::Rect r = GetRect(row, col);

      float sum_xi = sum.at<float>(r.y, r.x)
                      + sum.at<float>(r.y + r.height, r.x + r.width)
                      - sum.at<float>(r.y + r.height, r.x)
                      - sum.at<float>(r.y, r.x + r.width);

      mean.at<float>(row, col) = sum_xi / N;
    }
  }

  void UpdateModel(cv::Mat &mean_frame, const cv::Mat &update_mask)
  {
    if (update_mask.empty())
    {
      cv::addWeighted(mean_, 1.0 - params_.learning_rate, mean_frame, params_.learning_rate, 0.0, mean_);
    }
    else
    {
      cv::Mat weighted;
      cv::addWeighted(mean_, 1.0 - params_.learning_rate, mean_frame, params_.learning_rate, 0.0, weighted);
      weighted.copyTo(mean_, update_mask);
    }
  }


  // Returns the number of changed/foreground blocks
  int ReportChanges(const cv::Mat &frame, cv::Mat &threshold_img,
                    float threshold, bool do_update, const cv::Mat &update_mask)
  {
    VCP_INIT_TIC_TOC;
    VCP_TIC;
    // get the mean values of the current frame and write them to mean_frame
    cv::Mat mean_frame;
    CreateModel(frame, mean_frame);

    threshold_img.create(frame.size(), CV_32FC1);
    threshold_img.setTo(cv::Scalar::all(0));

    // now traverse the current mean_frame image
    int count = 0;
    for (int process = 0; process < mean_frame.rows * mean_frame.cols; ++process)
    {
      const int row = process / mean_frame.cols;
      const int col = process % mean_frame.cols;
      // current mean
      const float *ptr_frame = mean_frame.ptr<float>(row);
      float *ptr_mean = mean_.ptr<float>(row);

      const float val_mean_frame = ptr_frame[col];
      const float val_mean = ptr_mean[col];

      // now check, whether there have been changes in the selected area,
      // i.e if we have a foreground blob here
      const float change = std::fabs(val_mean_frame - val_mean);
      if(change > threshold)
      {
        // get the right values for the original sized image,
        // i.e make a coordinate transformation
        const cv::Rect r = GetRect(row, col);
        cv::Mat thresh_roi = threshold_img(r);

        // add the change to the background value
        thresh_roi = thresh_roi + cv::Scalar::all(change);
        ++count;
      }
    }

    // if update is wanted, update the background model
    // by a weighted sum of origin and new source
    if(do_update)
      UpdateModel(mean_frame, update_mask);

    VCP_TOC("BlockBased BGM done!");//FIXME remove
    return count;
  }

  inline cv::Rect GetRect(int row, int col) const {
    return cv::Rect(col * params_.block_size.width * params_.shift,
                    row * params_.block_size.height * params_.shift,
                    params_.block_size.width, params_.block_size.height);
  }


  inline cv::Rect GetModelRect(int img_x, int img_y, int model_width, int model_height) const
  {
    const int x = std::max(0, std::min(model_width - 1,
          (int)((float)(img_x - params_.block_size.width) / (params_.block_size.width * params_.shift) + 1)));
    const int y = std::max(0, std::min(model_height - 1,
          (int)((float)(img_y - params_.block_size.height) / (params_.block_size.height * params_.shift) + 1)));
    const int end_x = std::min(model_width - 1,
          (int)((float)img_x / (params_.block_size.width * params_.shift)));
    const int end_y = std::min(model_height - 1,
          (int)((float)img_y / (params_.block_size.height * params_.shift)));
    return cv::Rect(x, y, end_x - x + 1, end_y - y + 1);
  }
};


std::unique_ptr<BackgroundModel> CreateBlockBasedMeanBgm(const BlockBasedMeanBgmParams &params)
{
  return std::unique_ptr<BlockBasedMeanBackgroundModel>(new BlockBasedMeanBackgroundModel(params));
}
} // namespace bgm
} // namespace vcp
