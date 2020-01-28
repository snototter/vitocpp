#ifndef __VCP_IMVIS_COLLAGE_H__
#define __VCP_IMVIS_COLLAGE_H__

#include <vector>
#include <opencv2/core/core.hpp>

namespace vcp
{
namespace imvis
{
namespace collage
{
/**
 * @brief Make a collage of the given images.
 *
 * * If images are different types, all are converted to CV_8U.
 * * Images may have different number of channels (1 and 3 are supported).
 * * If fixed_size is given, all images are resized s.t. they optimally fit the given size (respecting their aspect ratio).
 * * Supports CV_32F, CV_64F and CV_8U images. If you mix the depths, single/double precision values will be scaled by 255,
 *   i.e. we silently assume that they are in the range [0,1] (and mixed depth always cause a conversion to CV_8U.
 * * You may add a padding between the images (col padding = row padding). There is no margin at the borders of the collage.
 *
 * @param[in] images
 * @param[out] collage
 * @param[in] images_per_row
 * @param[in] padding
 * @param[in] fixed_size
 * @param[in] convert_8U
 * @param[in] bg_color
 */
void Collage(const std::vector<cv::Mat> &images, cv::Mat &collage, size_t images_per_row=3, size_t padding=0, const cv::Size &fixed_size=cv::Size(), bool convert_8U=false, const cv::Scalar &bg_color=cv::Scalar::all(0.0));


/** @brief Resizes an image, s.t. it fits "optimally" into the given size
  * @param[in] image Image to resize
  * @param[out] resized Resized image (the input image will keep its aspect ratio, and it will be centered in the output image - boundaries will be black)
  * @param[in] new_size Size of the resized image
  */
void Resize(const cv::Mat &image, cv::Mat &resized, const cv::Size &new_size);


/** @brief Applies a perspective warp such that it looks like the image plane would be viewed at the given extrinsics.
 * Rotation angles should be given in radians.
 * border_color == (-1,-1,-1) will yield a RGB/BGR+A output image with invalid regions masked out.
 */
cv::Mat RenderPerspective(const cv::Mat &image, float rx, float ry, float rz, float tx, float ty, float tz, const cv::Scalar &border_color=cv::Scalar::all(-1));
} // namespace collage
} // namespace imvis
} // namespace vcp
#endif // __VCP_IMVIS_COLLAGE_H__
