#ifndef __VCP_IMUTILS_IMABSTRACTION_H__
#define __VCP_IMUTILS_IMABSTRACTION_H__

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/core/core.hpp>
#else
    #include <opencv2/core.hpp>
#endif
#include <vector>

namespace vcp
{
namespace imutils
{

//TODO num_blocks vs block size vs max num pixels (to ensure proper pixelation for privacy reasons) void pixelate(cv::Mat &image, int num_pixels)

/** @brief Uses a bilateral filter and edge enhancement to create a cartoon-like effect. Repeatedly (num_bilateral_filter times) applies a small bilateral filter instead of a single large filter. */
void Cartoonify(cv::Mat &image, int num_pyramid_levels=3, int num_bilateral_filters=5, int diameter_pixel_neighborhood=7, double sigma_color=9.0, double sigma_space=7.0, int kernel_size_median=7, int edge_block_size=9, bool is_rgb=false);

} // namespace imutils
} // namespace vcp

#endif // __VCP_IMUTILS_IMABSTRACTION_H__
