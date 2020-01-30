//#ifndef __VCP_BLOCKBASED_MEAN_BGM_H__
//#define __VCP_BLOCKBASED_MEAN_BGM_H__

//#include "background_model.h"
//#include <memory>

//namespace vcp
//{
//namespace bgm
//{
///** @brief Implements the block-based mean background model on the CPU. <b>Note:</b> the report() method may cause race conditions when using OpenMP (i.e. for large input images, say 1600x1200 pixels, the report call may last for up to 900 ms). */
//class BlockBasedMeanBackgroundModel : public BackgroundModel
//{
//public:
//  // mode 0 => gray, 1 => hsv (S)
//  BlockBasedMeanBackgroundModel(const cv::Mat & initial_bg, cv::Size block_size = cv::Size(32, 32), double shift = 0.25, double update_rate = 0.05, double report_threshold = 5.0, int mode = 0);
//  virtual ~BlockBasedMeanBackgroundModel();

//  void ReportChanges(const cv::Mat &current_image, cv::Mat &mask, bool update_model) override;
//  void BackgroundImage(cv::Mat &background) override;

//private:
//  cv::Size block_size_;     /**< @brief Block/patch size. */
//  double shift_;            /**< @brief Shift between patches, i.e. this defines the patch overlap. */
//  double update_rate_;      /**< @brief Learning rate \f$\lambda\f$, how fast the bgm should adapt to the update image, \f$ \lambda \in [0,1]\f$. */
//  double report_threshold_; /**< @brief Threshold s.t. patches will only be reported if their (absolute) difference to the background (mean) model is above this threshold. */
//  int mode_;                /**< @brief Indicates whether this model operates on grayscale (mode_ = 0) or HSV (ie saturation channel, mode_ = 1) images. */
//  cv::Mat mean_;            /**< @brief The mean background model. */

//  void CreateModel(const cv::Mat &frame, cv::Mat &mean);
//  void UpdateModel(cv::Mat &mean_frame);
//  void Update(const cv::Mat &frame);

//  int ReportChanges(const cv::Mat &frame, cv::Mat &threshold_img, double threshold, bool do_update);

//  inline cv::Rect GetRect(int row, int col) const {
//    return cv::Rect(col * block_size_.width * shift_, row * block_size_.height * shift_, block_size_.width, block_size_.height);
//  }

//  inline cv::Rect GetModelRect(int img_x, int img_y, int model_width, int model_height) const {

//    int x = std::max(0, std::min(model_width - 1, (int)((double)(img_x - block_size_.width) / (block_size_.width * shift_) + 1)));
//    int y = std::max(0, std::min(model_height - 1, (int)((double)(img_y - block_size_.height) / (block_size_.height * shift_) + 1)));
//    int end_x = std::min(model_width - 1, (int)((double)img_x / (block_size_.width * shift_)));
//    int end_y = std::min(model_height - 1, (int)((double)img_y / (block_size_.height * shift_)));
//    return cv::Rect(x, y, end_x - x + 1, end_y - y + 1);
//  }
//};

//std::unique_ptr<BackgroundModel> CreateBlockBasedMeanBgm(const cv::Mat & initial_bg, cv::Size block_size = cv::Size(32, 32), double shift = 0.25, double update_rate = 0.05, double report_threshold = 5.0, int mode = 0);
//} // namespace bgm
//} // namespace vcp

//#endif // __VCP_BLOCKBASED_MEAN_BGM_H__
