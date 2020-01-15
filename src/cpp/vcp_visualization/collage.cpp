#include "collage.h"

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_imutils/matutils.h>
#include <vcp_imutils/imutils.h>
#include <vcp_imutils/opencv_compatibility.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace vcp
{
namespace visualization
{
namespace collage
{
namespace utils
{
/** @brief Check if all images have the same depth. Must have at least 1 image. */
bool HaveSameDepth(const std::vector<cv::Mat> &images)
{
  const int first_depth = images[0].depth();
  for (size_t i = 1; i < images.size(); ++i)
  {
    if (images[i].depth() != first_depth)
      return false;
  }
  return true;
}

/** @brief Check if images have the supported 1 or 3 layers. Optionally (if not NULL) gets
 * the minimum/maximum number of layers. Must have at least 1 image. */
bool CheckLayers(const std::vector<cv::Mat> &images, int *max_layers, int *min_layers)
{
  bool supported_layers = true;
  if (min_layers)
    *min_layers = images[0].channels();
  if (max_layers)
    *max_layers = images[0].channels();

  for (size_t i = 0; i < images.size(); ++i)
  {
    if (images[i].channels() != 1 && images[i].channels() != 3)
    {
      supported_layers = false;
      VCP_LOG_WARNING("CheckLayers(): Image " << i << " has invalid shape: " << images[i].rows << "x" << images[i].cols << "x" << images[i].channels());
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
  const size_t num_images = images.size();
  num_images_per_row = std::min(num_images, num_images_per_row);
  num_rows = static_cast<size_t>(std::ceil(static_cast<double>(num_images) / static_cast<double>(num_images_per_row)));

  for (size_t row = 0, img_idx = 0; row < num_rows && img_idx < num_images; ++row)
  {
    cv::Size row_size;
    for (size_t col = 0; col < num_images_per_row && img_idx < num_images; ++col, ++img_idx)
    {
      cv::Size img_size;
      if (!IsValidSize(fixed_size))
        img_size = ComputeRescaledSize(images[img_idx], fixed_size, nullptr);
      else
        img_size = fixed_size;

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
  if (images.empty())
    return;

  int max_layers = -1;
  if (!utils::CheckLayers(images, &max_layers, nullptr))
    VCP_ERROR("Images for collage must have either 1 or 3 layers!");


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

} // namespace collage
} // namespace visualization
} // namespace vcp
