#include "imabstraction.h"
#include <vcp_math/common.h>

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/imgproc/imgproc.hpp>
#else
    #include <opencv2/imgproc.hpp>
#endif

namespace vcp
{
namespace imutils
{
// minimum variance quantization
//https://stackoverflow.com/questions/49710006/fast-color-quantization-in-opencv

namespace
{
//struct ImagePartition
//{
//  size_t num_pixels; // number of pixels within the partition
//  std::vector<size_t> left_edges; // top-left corner (the left edge of the partition)
//  std::vector<size_t> right_edges; // bottom-right corner (the right edge of the partition)
//  std::vector<size_t> mean; // location of the partition's mean
//  size_t optimal_split_dimension; // dimension along which to split (if needed)
//  size_t threshold;
//  float variance; // variance along optimal_split_dimension
//  float split_variances; // sum of variances along optimal_split_dimension (if split)
//  const cv::Mat &image;
//  // TODO sum projection function

//  explicit ImagePartition(const cv::Mat &img) : image(img) {}

//  void SetRootPartition()
//  {
//    num_pixels = image.cols * image.rows;
//    const int num_dimensions = 2;
//    left_edges.resize(num_dimensions, 0);
//    right_edges.resize(num_dimensions);
//    right_edges[0] = image.cols-1;
//    right_edges[1] = image.rows-1;
////    DIP_OVL_ASSIGN_NONCOMPLEX( computeSumProjections, ComputeSumProjections, image.DataType() );
////                FindOptimalSplit( computeSumProjections( image, leftEdges, rightEdges ));
//  }


//}
//class KDTree
//{
//public:
//private:

//};
} // (private) namespace

void Cartoonify(cv::Mat &image, int num_pyramid_levels, int num_bilateral_filters, int diameter_pixel_neighborhood,
                double sigma_color, double sigma_space, int kernel_size_median, int edge_block_size, bool is_rgb)
{
  // TODO doc block size defines the edge thickness, set to ~9 for a more cartoonish look

  // Based on mbeyeler's opencv-python-blueprints
  // added some fixes (e.g. incorrect resize), ported to c++.

  const bool is_color = image.channels() == 3 || image.channels() == 4;
  cv::Mat img_gray, img_color;
  if (is_color)
  {
    if (image.channels() == 4)
      cv::cvtColor(image, img_color, is_rgb ? cv::COLOR_RGBA2RGB : cv::COLOR_BGRA2BGR);
    else
      img_color = image;
    cv::cvtColor(img_color, img_gray, is_rgb ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
  }
  else
  {
    cv::cvtColor(image, img_color, cv::COLOR_GRAY2BGR);
    img_gray = image;
  }

  cv::Mat filter_helper;
  for (int i = 0; i < num_pyramid_levels-1; ++i)
  {
    cv::pyrDown(img_color, filter_helper);
    filter_helper.copyTo(img_color);
  }

  for (int i = 0; i < num_bilateral_filters; ++i)
  {
    cv::bilateralFilter(img_color, filter_helper, diameter_pixel_neighborhood, sigma_color, sigma_space);
    filter_helper.copyTo(img_color);
  }

  // Upsample
  for (int i = 0; i < num_pyramid_levels-1; ++i)
  {
    cv::pyrUp(img_color, filter_helper);
    filter_helper.copyTo(img_color);
  }
  // ... and ensure that the size fits
  if (img_color.rows != image.rows || img_color.cols != image.cols)
  {
    cv::resize(img_color, filter_helper, image.size());
    filter_helper.copyTo(img_color);
  }

  // Enhance edges
  cv::Mat output;
  if (edge_block_size > 1)
  {
    cv::medianBlur(img_gray, filter_helper, kernel_size_median);
    cv::Mat img_edge;
    cv::adaptiveThreshold(filter_helper, img_edge, 255.0, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, edge_block_size, 2.0);

    cv::cvtColor(img_edge, filter_helper, is_rgb ? cv::COLOR_GRAY2RGB : cv::COLOR_GRAY2BGR);
    filter_helper.copyTo(img_edge);

    cv::bitwise_and(img_color, img_edge, output);
  }
  else
    output = img_color;

  if (is_color)
    image = output;
  else
    cv::cvtColor(output, image, cv::COLOR_BGR2GRAY);
}


cv::Mat Pixelate(const cv::Mat &image, int block_width, int block_height)
{
  const int w = std::max(1, image.cols / block_width);
  const int h = std::max(1, image.rows / (block_height > 0 ? block_height : block_width));
  cv::Mat small_image, output;
  cv::resize(image, small_image, cv::Size(w, h), 0, 0, cv::INTER_NEAREST);
  cv::resize(small_image, output, image.size(), 0, 0, cv::INTER_NEAREST);
  return output;
}

} // namespace imutils
} // namespace vcp
