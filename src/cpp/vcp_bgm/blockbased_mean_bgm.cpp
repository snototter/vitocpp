//#include "blockbased_mean_bgm.h"
//#include <vcp_utils/timing_utils.h>
//#include <vcp_utils/vcp_error.h>

//#include <opencv2/core/version.hpp>
//#if CV_VERSION_MAJOR < 3
//    #include <opencv2/imgproc/imgproc.hpp>
//#else
//    #include <opencv2/imgproc.hpp>
//#endif

//namespace vcp
//{
//namespace bgm
//{
//BlockBasedMeanBackgroundModel::BlockBasedMeanBackgroundModel(const cv::Mat &initial_bg, cv::Size block_size, double shift, double update_rate, double report_threshold, int mode)
//  : BackgroundModel(), block_size_(block_size), shift_(shift), update_rate_(update_rate), report_threshold_(report_threshold), mode_(mode)
//{
//  CreateModel(initial_bg, mean_);
//}

//BlockBasedMeanBackgroundModel::~BlockBasedMeanBackgroundModel()
//{
//}

//void BlockBasedMeanBackgroundModel::Update(const cv::Mat &frame)
//{
//  cv::Mat mean_frame;
//  CreateModel(frame, mean_frame);
//  UpdateModel(mean_frame);
//}

//void BlockBasedMeanBackgroundModel::ReportChanges(const cv::Mat &current_image, cv::Mat &mask, bool update_model)
//{
//  ReportChanges(current_image, mask, report_threshold_, update_model);
//}

//void BlockBasedMeanBackgroundModel::BackgroundImage(cv::Mat &background)
//{
//  cv::normalize(mean_, background,1, 0, cv::NORM_MINMAX);
//}

//int BlockBasedMeanBackgroundModel::ReportChanges(const cv::Mat &frame, cv::Mat &threshold_img, double threshold, bool do_update)
//{
////  INIT_TIC_TOC;
////  TIC;

//  // get the mean values of the current frame and write them to mean_frame
//  cv::Mat mean_frame;
//  CreateModel(frame, mean_frame);

//  threshold_img.create(frame.size(), CV_64FC1);
//  threshold_img.setTo(cv::Scalar::all(0));

//  // now traverse the current mean_frame image
//  int count = 0;
//  for (int process = 0; process < mean_frame.rows*mean_frame.cols; ++process)
//  {
//    int row = process / mean_frame.cols;
//    int col = process % mean_frame.cols;
//    // current mean
//    double *ptr_frame = mean_frame.ptr<double>(row);
//    double *ptr_mean = mean_.ptr<double>(row);

//    double val_mean_frame = ptr_frame[col];
//    double val_mean = ptr_mean[col];

//    // here we have the mean of the background image

//    // now check, whether there have been changes in the selected area, i.e we have a blob here
//    if( fabs(val_mean_frame - val_mean) > threshold )
//    {
//      // get the right values for the original sized image
//      // i.e a coordinate transformation respectively
//      const cv::Rect r = GetRect(row, col);

//      // crop the thresholdImg to that specific area
//      cv::Mat thresh_roi = threshold_img(r);

//      // add the difference to background value
//      //cvAddS(thresholdImg, cvScalar(abs(mean_val - mean)), thresholdImg);
//      thresh_roi = thresh_roi + cv::Scalar(fabs(val_mean_frame - val_mean));
//      ++count;
//    }
//  }

//  // if update is wanted, update the background model
//  // by a weighted sum of origin and new source
//  if(do_update)
//    UpdateModel(mean_frame);

////  TOC("BlockBasedMeanBackgroundModel::report");

//  return count;
//}


//void BlockBasedMeanBackgroundModel::CreateModel(const cv::Mat &frame, cv::Mat &mean)
//{
//  // this function computes the mean values for each block in a given input image ("frame")
//  // and writes these values to a "mean". Therefore you can think of traversing the whole image
//  // with a block sized window, moving on for windowHeight (or Width respectively) * shift_factor
//  // and computing the mean value of the pixels lying beyond the corresponging window.

////  INIT_TIC_TOC;
////  TIC;
//  cv::Mat to_model;
//  if (frame.channels() == 1)
//    to_model = frame;
//  else
//  {
//    if (mode_ == 0)
//    {
////      PVT_LOG_INFO("[BlockBasedMeanBackgroundModel] Multi-channel image given - converting to grayscale");
//      cv::cvtColor(frame, to_model, CV_BGR2GRAY);
//    }
//    else
//    {
////      PVT_LOG_INFO("[BlockBasedMeanBackgroundModel] Multi-channel image given - converting to HSV");
//      cv::Mat hsv;
//      cv::cvtColor(frame, hsv, CV_RGB2HSV);
//      std::vector<cv::Mat> layers;
//      cv::split(hsv, layers);

//      to_model = layers[1]; // take saturation layer (should remove penumbra (soft shadows))
//    }
//  }

//  int rows = (to_model.rows - block_size_.height) / (block_size_.height * shift_) +1; //FIXME +1? original formulation not, but while implementing the GPU counterpart, +1 seemed just SO correct!
//  int cols = (to_model.cols - block_size_.width) / (block_size_.width * shift_) +1;

//  mean.create(rows, cols, CV_64FC1);

//  cv::Mat sum;
//  cv::integral(to_model, sum, CV_64F);

//  double N = (double)block_size_.width * (double)block_size_.height;

////#pragma omp parallel for
//  for(int process = 0; process < rows*cols; ++process)
//  {
//    int row = process / cols;
//    int col = process % cols;
//    const cv::Rect r = GetRect(row, col);

//    double sum_xi = sum.at<double>(r.y, r.x)
//                    + sum.at<double>(r.y + r.height, r.x + r.width)
//                    - sum.at<double>(r.y + r.height, r.x)
//                    - sum.at<double>(r.y, r.x + r.width);

//    mean.at<double>(row, col) = sum_xi / N;
//  }
////  TOC("BlockBasedMeanBackgroundModel::createModel");
//}

//void BlockBasedMeanBackgroundModel::UpdateModel(cv::Mat &mean_frame)
//{
//  // calculate weighted sum of two input images and write to destination image
//  // mean_ * (1.0 - update_rate_) + mean_current_frame * update_rate_ + 0.0 = mean_
//  cv::addWeighted(mean_, 1.0 - update_rate_, mean_frame, update_rate_, 0.0, mean_);
//}

//std::unique_ptr<BackgroundModel> CreateBlockBasedMeanBgm(const cv::Mat & initial_bg, cv::Size block_size, double shift, double update_rate, double report_threshold, int mode)
//{
//  return std::unique_ptr<BlockBasedMeanBackgroundModel>(new BlockBasedMeanBackgroundModel(initial_bg, block_size, shift, update_rate, report_threshold, mode));
//}
//} // namespace bgm
//} // namespace vcp
