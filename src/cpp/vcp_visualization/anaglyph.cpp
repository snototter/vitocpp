#include "anaglyph.h"

#include <vector>
#include <opencv2/imgproc/imgproc.hpp>

namespace vcp
{
namespace visualization
{
namespace anaglyph
{
void GenerateAnaglyph(const cv::Mat &left, const cv::Mat &right, cv::Mat &anaglyph, bool flip_color_channels, cv::Mat *invalid)
{
  std::vector<cv::Mat> r, l;

  cv::split(left, l);
  cv::split(right, r);

  // OpenCV uses BGR
  if (flip_color_channels)
    r[0] = l[0];
  else
    r[2] = l[2];

  cv::merge(r, anaglyph);

  if (invalid)
    anaglyph.setTo(cv::Scalar::all(0.0), *invalid);

//  for (int row = 0; row < left.rows; ++row)
//  {
//    for (int col = 0; col < left.cols; ++col)
//    {
//      // check, if the color of the pixel in the left OR right image is
//      // black (all three channels are equal to 0)
//      // -> if yes: set all channels of the right image to 0
//      // -> if no: set red channel of the right image to the red channel
//      //           of the left image
//      if ((l[0].at<uchar>(row, col) > 0 || l[1].at<uchar>(row, col) > 0 || l[2].at<uchar>(row, col) > 0)
//          && (r[0].at<uchar>(row, col) > 0 || r[1].at<uchar>(row, col) > 0 || r[2].at<uchar>(row, col) > 0))
//      {
//        r[2].at<uchar>(row, col) = l[2].at<uchar>(row, col);
//      }
//      else
//      {
//        r[0].at<uchar>(row, col) = 0;
//        r[1].at<uchar>(row, col) = 0;
//        r[2].at<uchar>(row, col) = 0;
//      }
//    }
//  }
//  cv::merge(r, anaglyph);
}

cv::Mat ShiftImage(const cv::Mat &img, int offset_x, int offset_y, cv::Mat *padded)
{
  cv::Mat shifted = cv::Mat::zeros(img.size(), img.type());
  if (padded)
    *padded = cv::Mat(img.rows, img.cols, CV_8UC1, cv::Scalar::all(255.0));

  cv::Rect src_view, dst_view;
  if (offset_x < 0)
  {
    src_view.x = -offset_x;
    src_view.width = img.cols + offset_x;
    dst_view.x = 0;
  }
  else
  {
    src_view.x = 0;
    src_view.width = img.cols - offset_x;
    dst_view.x = offset_x;
  }
  dst_view.width = src_view.width;

  if (offset_y < 0)
  {
    src_view.y = -offset_y;
    src_view.height = img.rows + offset_y;
    dst_view.y = 0;
  }
  else
  {
    src_view.y = 0;
    src_view.height = img.rows - offset_y;
    dst_view.y = offset_y;
  }
  dst_view.height = src_view.height;


  cv::Mat dst = shifted(dst_view);
  img(src_view).copyTo(dst);

  if (padded)
  {
    dst = (*padded)(dst_view);
    dst.setTo(cv::Scalar::all(0.0));
  }

  return shifted;
}

//DEPRECATED
cv::Mat AdjustBaseline(const cv::Mat &img, int offset)
{
  return ShiftImage(img, -offset, 0, nullptr);
  // * adjust the baseline of 'img' and store it in 'shifted_img'
  //
  //   - get the parameter config.baseline
    //   - check if config.baseline > 0
  //      -> if yes: move every pixel of 'img' by config.baseline pixels to
  //                 the left side. set the rest to black.
  //      -> if no:  calculate absolute value of config.baseline
  //                 move every pixel of 'img' by absolute value of
  //                 config.baseline pixels to the right side.
  //                 set the rest to black.
//  cv::Mat shifted_img = cv::Mat::zeros(img.size(), img.type());
//  if (offset < 0)
//  {
//    offset = -offset;
//    cv::Mat roi = shifted_img(cv::Rect(offset, 0, shifted_img.cols - offset, shifted_img.rows));
//    img(cv::Rect(0, 0, img.cols - offset, img.rows)).copyTo(roi);
//  }
//  else
//  {
//    cv::Mat roi = shifted_img(cv::Rect(0, 0, shifted_img.cols - offset, shifted_img.rows));
//    img(cv::Rect(offset, 0, img.cols - offset, img.rows)).copyTo(roi);
//  }
//  return shifted_img;
}

} // namespace anaglyph
} // namespace visualization
} // namespace vcp
