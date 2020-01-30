#ifndef __VCP_IMVIS_COLLAGE_H__
#define __VCP_IMVIS_COLLAGE_H__

#include <vector>
#include <opencv2/core/core.hpp>

namespace vcp
{
namespace imvis
{
/** @brief TODO doc. */
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
 *
 * rx, ry, rz: Rotation angles should be given in radians, unless you set "angles_in_deg".
 * tx, ty, tz: Translation of the camera.
 * border_color: (-1,-1,-1) will yield a RGB/BGR+A output image with invalid regions masked out.
 *              Otherwise, invalid regions (warping border) will be set to border_color.
 * inter_linear_alpha: If border_color is (-1,-1,-1), then the alpha channel will be either linearly
 *              interpolated (true) or via nearest neighbor lookup (false).
 * img_plane_z: Position of the image plane along the optical axis. Changing this has the same effect
 *              as changing tz - nevertheless, it is required to simplify @see RenderImageSequence.
 * angles_in_deg: Set to true if rx, ry, rz are given in degrees instead of radians.
 * projection_roi: If it is a valid pointer, the actual projection region will be returned - this
 *              is necessary, because a) the warped image will be shifted to be visible and b) the
 *              output size will be clipped to at most 3*image.size().
 */
cv::Mat RenderPerspective(const cv::Mat &image,
                          float rx, float ry, float rz, bool angles_in_deg=false,
                          float tx=0.0f, float ty=0.0f, float tz=0.0f,
                          const cv::Scalar &border_color=cv::Scalar::all(-1),
                          bool inter_linear_alpha=false,
                          float img_plane_z=1.0f,
                          cv::Rect2d *projection_roi=nullptr,
                          cv::Mat *projection_mask=nullptr);

/** @brief Stacks the given images along the optical axis and renders them to look like they were viewed with the given extrinsics.
 *
 * rx, ry, rz: Virtual camera rotation angles
 * angles_in_deg: Set to true if rotation angles are given in degrees.
 * tx, ty, tz: Virtual camera translation
 * delta_z: Distance between the images along the optical axis. Hint on scaling: an image plane spans from [-1, +1] (in both x and y).
 */
cv::Mat RenderImageSequence(const std::vector<cv::Mat> &images,
                            float rx, float ry, float rz, bool angles_in_deg=false,
                            float tx=0.0f, float ty=0.0f, float tz=0.0f, float delta_z = 0.1f,
                            const cv::Scalar &border_color=cv::Scalar::all(-1), bool inter_linear_alpha=false);

} // namespace collage
} // namespace imvis
} // namespace vcp
#endif // __VCP_IMVIS_COLLAGE_H__
