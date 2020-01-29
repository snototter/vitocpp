#include "collage.h"

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_imutils/matutils.h>
#include <vcp_imutils/imutils.h>
#include <vcp_imutils/opencv_compatibility.h>
#include <vcp_math/conversions.h>
#include <vcp_math/common.h>
#include <vcp_math/geometry3d.h>
#include <opencv2/imgproc/imgproc.hpp>

//FIXME remove
#include <opencv2/highgui.hpp>
#include <vcp_utils/string_utils.h>

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::imvis::collage"

namespace vcp
{
namespace imvis
{
namespace collage
{
namespace utils
{
/** @brief Check if all images have the same depth. Must have at least 1 image. */
bool HaveSameDepth(const std::vector<cv::Mat> &images)
{
//  VCP_LOG_DEBUG("HaveSameDepth()");
  const int first_depth = images[0].depth();
  for (size_t i = 1; i < images.size(); ++i)
  {
    if (images[i].depth() != first_depth)
      return false;
  }
  return true;
}

/** @brief Check if images have the supported 1, 3 or 4 layers. Optionally (if not NULL) gets
 * the minimum/maximum number of layers. Must have at least 1 image. */
bool CheckLayers(const std::vector<cv::Mat> &images, int *max_layers, int *min_layers)
{
//  VCP_LOG_DEBUG("CheckLayers()");
  bool supported_layers = true;
  if (min_layers)
    *min_layers = images[0].channels();
  if (max_layers)
    *max_layers = images[0].channels();

  for (size_t i = 0; i < images.size(); ++i)
  {
    if (images[i].channels() != 1
        && images[i].channels() != 3
        && images[i].channels() != 4)
    {
      supported_layers = false;
      VCP_LOG_WARNING("CheckLayers(): Image #" << i << " has invalid shape: " << images[i].rows << "x" << images[i].cols << "x" << images[i].channels());
    }

    if (min_layers && images[i].channels() < *min_layers)
      *min_layers = images[i].channels();
    if (max_layers && images[i].channels() > *max_layers)
      *max_layers = images[i].channels();
  }
  return supported_layers;
}

/** @brief Compute size if the image needs to be rescaled (keeping the aspect ratio intact).
 * Optionally sets the ROI s.t. the image is centered within the output rectangle.
 * Image must be valid (i.e. not empty).
 */
cv::Size ComputeRescaledSize(const cv::Mat &image, const cv::Size &fixed_size, cv::Rect *centered_roi)
{
//  VCP_LOG_DEBUG("ComputeRescaledSize()");
  if (!IsValidSize(fixed_size))
  {
    if (centered_roi)
    {
      centered_roi->x = 0;
      centered_roi->y = 0;
      centered_roi->width = image.cols;
      centered_roi->height = image.rows;
    }
    return image.size();
  }

  // Try to scale to new width:
  cv::Size scaled;
  scaled.width = fixed_size.width;
  scaled.height = (fixed_size.width * image.rows) / image.cols;

  // If new image would be too large, scale to new height:
  if (scaled.height > fixed_size.height)
  {
    scaled.height = fixed_size.height;
    scaled.width = (fixed_size.height * image.cols) / image.rows;

    if (centered_roi)
    {
      centered_roi->x = (fixed_size.width - scaled.width) / 2;
      centered_roi->y = 0;
    }
  }
  else
  {
    if (centered_roi)
    {
      centered_roi->x = 0;
      centered_roi->y = (fixed_size.height - scaled.height) / 2;
    }
  }
  if (centered_roi)
  {
    centered_roi->width = scaled.width;
    centered_roi->height = scaled.height;
  }
  return scaled;
}

/** @brief Compute size of the final collage. */
void ComputeCollageSize(const std::vector<cv::Mat> &images, const cv::Size &fixed_size, size_t padding, size_t &num_images_per_row, size_t &num_rows, cv::Size &collage_size, std::vector<int> &max_row_heights)
{
//  VCP_LOG_DEBUG("ComputeCollageSize()");
  const size_t num_images = images.size();
  num_images_per_row = std::min(num_images, num_images_per_row);
  num_rows = static_cast<size_t>(std::ceil(static_cast<double>(num_images) / static_cast<double>(num_images_per_row)));

  for (size_t row = 0, img_idx = 0; row < num_rows && img_idx < num_images; ++row)
  {
    cv::Size row_size;
    for (size_t col = 0; col < num_images_per_row && img_idx < num_images; ++col, ++img_idx)
    {
      cv::Size img_size;
      if (IsValidSize(fixed_size))
        img_size = fixed_size;
      else
        img_size = ComputeRescaledSize(images[img_idx], fixed_size, nullptr);

      if (col == 0)
      {
        row_size.width = img_size.width;
        row_size.height = img_size.height;
      }
      else
      {
        // Accumulate width:
        row_size.width += img_size.width + padding;
        // Store maximum height per row:
        if (row_size.height < img_size.height)
          row_size.height = img_size.height;
      }
    }
    // Store maximum width over all rows:
    if (row == 0 || collage_size.width < row_size.width)
      collage_size.width = row_size.width;

    // Accumulate height:
    if (row == 0)
      collage_size.height = row_size.height;
    else
      collage_size.height += row_size.height + padding;
    max_row_heights.push_back(row_size.height);
  }
}

/** Optionally resize (exactly to the given target_size!) and convert (depth and number of layers) the given image. */
cv::Mat PrepareImageForCollage(const cv::Mat &image, const cv::Size &target_size, bool convert_8U, int num_layers)
{
//  VCP_LOG_DEBUG("PrepareImageForCollage()");
  // Resize if needed.
  cv::Mat resized;
  if (!IsValidSize(target_size) || (target_size.width == image.cols && target_size.height == image.rows))
    resized = image.clone();
  else
    cv::resize(image, resized, target_size);

  // Convert depth if needed.
  cv::Mat converted;
  if (convert_8U && resized.depth() != CV_8U)
    vcp::imutils::ConvertTo8U(resized, converted);
  else
    converted = resized;

  // Stack more layers ;-)
  if (converted.channels() != num_layers)
    return vcp::imutils::StackLayers(converted, num_layers);
  return converted;
}
} // namespace utils


void Collage(const std::vector<cv::Mat> &images, cv::Mat &collage, size_t num_images_per_row, size_t padding, const cv::Size &fixed_size, bool convert_8U, const cv::Scalar &bg_color)
{
  VCP_LOG_DEBUG("Collage()");
  if (images.empty())
  {
    VCP_LOG_FAILURE("Collage() called without images!");
    return;
  }

  for (const auto &m : images)
  {
    if (m.empty())
    {
      VCP_LOG_FAILURE("Collage() called with at least one invalid image!");
      return;
    }
  }

  int max_layers = -1;
  if (!utils::CheckLayers(images, &max_layers, nullptr))
    VCP_ERROR("Images for collage must have either 1, 3 or 4 layers!");


  // Compute collage size
  cv::Size collage_size;
  std::vector<int> max_row_heights;
  size_t num_rows;
  utils::ComputeCollageSize(images, fixed_size, padding, num_images_per_row, num_rows, collage_size, max_row_heights);


  const size_t num_images = images.size();

  // If different depths, convert.
  const bool same_depth = utils::HaveSameDepth(images);
  convert_8U = convert_8U || !same_depth;

  // Build collage
  collage = cv::Mat(collage_size, convert_8U ? CV_MAKETYPE(CV_8U, max_layers) : CV_MAKETYPE(images[0].depth(), max_layers), bg_color);

  int insert_top = 0;
  for (size_t row = 0, img_idx = 0; row < num_rows && img_idx < num_images; ++row)
  {
    int insert_left = 0;
    for (size_t col = 0; col < num_images_per_row && img_idx < num_images; ++col, ++img_idx)
    {
      // Prepare image.
      cv::Rect centered_roi;
      const cv::Size max_target_size = !IsValidSize(fixed_size)
          ? cv::Size(images[img_idx].cols, max_row_heights[row])
          : cv::Size(fixed_size.width, std::min(fixed_size.height, max_row_heights[row]));
      const cv::Size img_size = utils::ComputeRescaledSize(images[img_idx], max_target_size, &centered_roi);
      cv::Mat img = utils::PrepareImageForCollage(images[img_idx], img_size, convert_8U, max_layers);

      // Copy onto collage.
      cv::Rect insertion(insert_left + centered_roi.x, insert_top + centered_roi.y, centered_roi.width, centered_roi.height);
      cv::Mat roi = collage(insertion);
      img.copyTo(roi);

      if (!IsValidSize(fixed_size))
        insert_left += img.cols + padding;
      else
        insert_left += fixed_size.width + padding;
    }

    insert_top += max_row_heights[row] + padding;
  }
}


void Resize(const cv::Mat &image, cv::Mat &resized, const cv::Size &new_size)
{
//  VCP_LOG_DEBUG("Resize()");
  resized = cv::Mat::zeros(new_size, image.type());

  cv::Size image_new_size;
  cv::Rect rect;

  image_new_size.width = new_size.width;
  image_new_size.height = (new_size.width * image.rows) / image.cols;

  if (image_new_size.height > new_size.height)
  {
    image_new_size.height = new_size.height;
    image_new_size.width = (new_size.height * image.cols) / image.rows;

    rect.x = (new_size.width - image_new_size.width) / 2;
    rect.y = 0;
  }
  else
  {
    rect.x = 0;
    rect.y = (new_size.height - image_new_size.height) / 2;
  }
  rect.width = image_new_size.width;
  rect.height = image_new_size.height;

  cv::Mat roi = resized(rect);
  cv::resize(image, roi, image_new_size);
}



cv::Mat RenderPerspective(const cv::Mat &image,
                          float rx, float ry, float rz,
                          bool angles_in_deg,
                          float tx, float ty, float tz,
                          const cv::Scalar &border_color,
                          bool inter_linear_alpha, float img_plane_z,
                          cv::Rect2d *projection_roi)
{
  const cv::Mat R = math::geo3d::RotationMatrix(rx, ry, rz, angles_in_deg);
  const cv::Mat t = (cv::Mat_<double>(3, 1) << tx, ty, tz);

  cv::Mat K = (cv::Mat_<double>(3, 3)
               << image.cols/2.0, 0.0, image.cols/2.0,
               0.0, image.rows/2.0, image.rows/2.0,
               0.0, 0.0, 1.0);

  const auto corners2d_src = imutils::ImageCorners<float>(image);
  const std::vector<cv::Vec3d> corners3d_src = {
    cv::Vec3d(-1, -1, img_plane_z),
    cv::Vec3d(1, -1, img_plane_z),
    cv::Vec3d(1, 1, img_plane_z),
    cv::Vec3d(-1, 1, img_plane_z)
  };

  // Project the "3D image plane" with the new camera extrinsics.
  cv::Mat P = math::geo3d::ProjectionMatrixFromKRt(K, R, t);
  auto projected = math::geo3d::ProjectVecs(P, corners3d_src);
  // Check position of projected corner points
  cv::Vec2d prj_min, prj_max;
  math::MinMaxVec(projected, prj_min, prj_max);
  const cv::Vec2d prj_span = prj_max - prj_min;
  const cv::Size span = cv::Size(
        static_cast<int>(std::ceil(prj_span[0])),
        static_cast<int>(std::ceil(prj_span[1])));
  // Store the projected region (might contain negative coordinates which we'll shift next)
  if (projection_roi)
    *projection_roi = cv::Rect2d(prj_min[0], prj_min[1], prj_span[0], prj_span[1]);

  // Adjust principal point offset so that the (top-left of the) warped image is visible
  K.at<double>(0, 2) -= prj_min[0];
  K.at<double>(1, 2) -= prj_min[1];
  // Reproject corners once again with the adjusted intriniscs
  P = math::geo3d::ProjectionMatrixFromKRt(K, R, t);
  projected = math::geo3d::ProjectVecs(P, corners3d_src);

  VCP_LOG_DEBUG("Image plane initially projected to " << prj_min << " <--> " << prj_max << ", " << span
                  << std::endl << "          With adjusted camera matrix: " << projected);
  const cv::Mat M = cv::getPerspectiveTransform(corners2d_src, vcp::convert::ToPoint2f(projected));
  cv::Mat warped;
  // Clip output size
  const cv::Size output_size = cv::Size(
        std::max(1, std::min(3 * image.cols, span.width)),
        std::max(1, std::min(3 * image.rows, span.height)));

  if (border_color[0] < 0 || border_color[1] < 0 || border_color[2] < 0 )
  {
    // Warp the image
    cv::warpPerspective(image, warped, M, output_size);
    if (image.channels() != 4)
    {
      // If the input image is already RGB/BGR+A, we don't need to
      // consider/add the alpha channel.

      // Warp the alpha channel
      const cv::Mat mask = cv::Mat(image.size(), CV_8U, cv::Scalar::all(vcp::math::MaxPixelValue<unsigned char>(image.depth())));
      cv::Mat wm8u, warped_mask;
      cv::warpPerspective(mask, wm8u, M, output_size,
                          inter_linear_alpha ? cv::INTER_LINEAR : cv::INTER_NEAREST,
                          cv::BORDER_CONSTANT, cv::Scalar::all(0));
      wm8u.convertTo(warped_mask, image.type());

      // Stack the layers
      if (warped.channels() == 1)
      {
        const cv::Mat layers[4] = {warped, warped, warped, warped_mask};
        cv::Mat tmp;
        cv::merge(layers, 4, tmp);
        warped = tmp;
      }
      else if (warped.channels() == 3)
      {
        std::vector<cv::Mat> layers_img;
        cv::split(warped, layers_img);
        const cv::Mat layers_out[4] = {layers_img[0], layers_img[1], layers_img[2], warped_mask};
        cv::Mat tmp;
        cv::merge(layers_out, 4, tmp);
        warped = tmp;
      }
      else
      {
        VCP_ERROR("Invalid number of input channels, only 1, 3 or 4 are supported.");
      }
    }
  }
  else
  {
    cv::warpPerspective(image, warped, M, output_size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, border_color);
  }
  return warped;
}


cv::Mat RenderImageSequence(const std::vector<cv::Mat> &images, float rx, float ry, float rz, bool angles_in_deg, float tx, float ty, float tz, float delta_z, const cv::Scalar &border_color, bool inter_linear_alpha)
{
  std::vector<cv::Mat> warped;
  warped.reserve(images.size());

  std::vector<cv::Rect2d> prj_rois;
  std::vector<cv::Vec2d> corners;
  prj_rois.resize(images.size()); // Resize on purpose

  // FIXME we must know the projection (mask), cannot rely on alpha channel: padded border may be transparent)
  float img_plane_z = 1.0f;
  for (size_t i = 0; i < images.size(); ++i)
  {
    warped.push_back(RenderPerspective(images[i], rx, ry, rz, angles_in_deg,
                                       tx, ty, tz, border_color, inter_linear_alpha,
                                       img_plane_z, &prj_rois[i]));
    corners.push_back(convert::ToVec2d(prj_rois[i].tl()));
    corners.push_back(convert::ToVec2d(prj_rois[i].br()));
    img_plane_z += delta_z;

    VCP_LOG_FAILURE("Image #" << i << ", z=" << img_plane_z << " warped to " << prj_rois[i]);
  }
  cv::Vec2d prj_min, prj_max;
  math::MinMaxVec(corners, prj_min, prj_max);
  const cv::Vec2d prj_span = prj_max - prj_min;
  const cv::Size span = cv::Size(
        static_cast<int>(std::ceil(prj_span[0])),
        static_cast<int>(std::ceil(prj_span[1])));


  // Clip output size
  const cv::Size output_size = cv::Size(
        std::max(1, std::min(3*images[0].cols, 2*span.width)),
        std::max(1, std::min(3*images[0].rows, 2*span.height))); //FIXME FUCKER 3* size?, 4x size...
  VCP_LOG_FAILURE("FIXME!");

  const bool force_alpha_channel = border_color[0] < 0 || border_color[1] < 0 || border_color[2] < 0;
  cv::Mat out = cv::Mat::zeros(output_size, CV_MAKETYPE(images[0].type(), force_alpha_channel ? 4 : images[0].channels()));

  if (!force_alpha_channel)
    out.setTo(border_color);

  const auto offset = convert::ToPoint(prj_min);
  for (int i = static_cast<int>(warped.size())-1; i >= 0; --i)
//  fuckit
  //for (int i = 0; i < warped.size(); ++i)
  {
    const auto prj_roi = convert::ToRect(prj_rois[i]);
    cv::Rect roi = cv::Rect(prj_roi.tl() - offset, warped[i].size());
    cv::Point tl = roi.tl();
    cv::Point br = roi.br();

    if (!imutils::IsPointInsideImage(tl, out.size()))
    {
      tl.x = std::max(0, std::min(out.cols, tl.x));
      tl.y = std::max(0, std::min(out.rows, tl.y));
    }

    if (!imutils::IsPointInsideImage(br, out.size()))
    {
      br.x = std::max(0, std::min(out.cols, br.x));
      br.y = std::max(0, std::min(out.rows, br.y));
    }

    roi = cv::Rect(tl, br - tl);
    if (roi.width > 0 && roi.height > 0)
    {
      VCP_LOG_FAILURE("ROI TO DRAW TOOOO: #" << i << " " << roi
                      << std::endl << prj_roi << ":::" << warped[i].size());
      cv::Mat in = warped[i];
      if (in.size() == roi.size())
      {
        cv::Mat out_roi = out(roi);
        in.copyTo(out_roi); //TODO MASK!
        cv::imshow("normalin" + vcp::utils::string::ToStr(i), in);
        cv::imshow("normalout" + vcp::utils::string::ToStr(i), out_roi);
        cv::waitKey(100);
      }
      else
      {
        VCP_LOG_FAILURE("clipping stuff:" << std::endl
                        << prj_roi << " changed to " << roi << std::endl
                        << "Warped: " << warped[i].size() << " vs roi " << roi.size() << std::endl
                        << "VS out: " << out.size());
        roi.width = std::min(roi.width, in.cols);
        roi.height = std::min(roi.height, in.rows);
        cv::Mat out_roi = out(roi);
        const cv::Rect in_roi = cv::Rect(0, 0, roi.width, roi.height);
        in(in_roi).copyTo(out_roi); //TODO MASK!
        cv::imshow("fuckin" + vcp::utils::string::ToStr(i), in);
        cv::imshow("fuckout" + vcp::utils::string::ToStr(i), out_roi);
        cv::waitKey(100);
      }
    }
    else
    {
      VCP_LOG_WARNING("Image #" << i << " is outside projection region.");
    }
  }
  return out;
}
} // namespace collage
} // namespace imvis
} // namespace vcp
