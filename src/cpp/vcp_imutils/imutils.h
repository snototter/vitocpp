#ifndef __VCP_IMUTILS_IMUTILS_H__
#define __VCP_IMUTILS_IMUTILS_H__

#include <opencv2/core/core.hpp>
#include <vector>

namespace vcp
{
namespace imutils
{

/** @brief Return rectangular ROI which contains the full image. */
inline cv::Rect ImageRect(const cv::Mat &image)
{
  return cv::Rect(0, 0, image.cols, image.rows);
}


/** @brief Return the image center. */
inline cv::Vec2d ImageCenter(const cv::Mat &image)
{
  return cv::Vec2d(static_cast<double>(image.cols)/2.0, static_cast<double>(image.rows)/2.0);
}

/** @brief Return the image corners. */
template<typename T>
inline std::vector<cv::Point_<T>> ImageCorners(const cv::Size &size)
{
  const int r = size.width - 1;
  const int b = size.height - 1;
  std::vector<cv::Point_<T>> corners = {
    cv::Point_<T>(static_cast<T>(0), static_cast<T>(0)),
    cv::Point_<T>(static_cast<T>(r), static_cast<T>(0)),
    cv::Point_<T>(static_cast<T>(r), static_cast<T>(b)),
    cv::Point_<T>(static_cast<T>(0), static_cast<T>(b))
  };
  return corners;
}

template<typename T>
inline std::vector<cv::Point_<T>> ImageCorners(const cv::Mat &image)
{
  return ImageCorners<T>(image.size());
}


/** @brief Clip a rectangle to stay within the image boundaries. */
inline void ClipRectangleToImageBoundaries(cv::Rect &rect, const cv::Size &img_size)
{
  int l = rect.x;
  const int r = std::min(std::max(0, l+rect.width-1), img_size.width-1);
  l = std::max(l, 0);
  int t = rect.y;
  const int b = std::min(std::max(0, t+rect.height-1), img_size.height-1);
  t = std::max(t, 0);
  rect.x = l;
  rect.y = t;
  rect.width = r - l + 1;
  rect.height = b - t + 1;
}


/** @brief Apply the given function to a region of interest. Takes care of clipping the ROI. */
//TODO make variadic: https://stackoverflow.com/questions/25392935/wrap-a-function-pointer-in-c-with-variadic-template
inline void ApplyFunctionToImageROI(cv::Mat &image, const cv::Rect &roi, void (*function)(cv::Mat &))
{
  cv::Rect clipped(roi);
  ClipRectangleToImageBoundaries(clipped, image.size());
  if (clipped.width > 0 && clipped.height > 0)
  {
    cv::Mat view = image(clipped);
    function(view);
  }
}


/** @brief Check if point lies within the image. */
bool IsPointInsideImage(const cv::Point &pt, const cv::Size &image_size);
/** @brief Check if point lies within the image. */
bool IsPointInsideImage(const cv::Vec2d &pt, const cv::Size &image_size);


/** @brief Crop a rectangular WxH ROI, centered at the image center. */
cv::Mat CenterCrop(const cv::Mat &image, const cv::Size &crop_size);


/** @brief Returns the size of the centered ROI with the maximum area to crop the rotated image, leaving no invalid pixels. Theta in radians. */
cv::Size MaxAxisAlignedCropSize(const cv::Size &image_size, double theta);


/** @brief Rotates the image by theta radians, optionally crops it (to contain only valid pixels). */
void RotateImage(cv::Mat &image, double theta, bool crop);


/** @brief Rotates the image by theta radians around the given anchor point. */
void RotateImage(cv::Mat &image, const cv::Point &anchor, double theta, int border_mode=cv::BORDER_CONSTANT);


/** @brief Rotates the image and its corresponding bounding boxes by theta radians. */
std::vector<cv::RotatedRect> RotateAnnotatedImage(cv::Mat &image, const std::vector<cv::Rect> &bounding_boxes, double theta, bool crop);


/** @brief Rotates the given point (image frame coordinates, top-left is (0,0)) around the image center assuming a left-handed (image!) coordinate system.
 * Positive radians will rotate counter-clockwise!
 */
cv::Vec2d RotateImageVector(const cv::Vec2d &vec, const cv::Size &image_size, double theta);


/** @brief Rotates the given rectangle by the given radians, assuming a left-handed (image!) coordinate system. */
cv::RotatedRect RotateRect(const cv::Rect &rect, const cv::Vec2d &rotation_center, double theta);


/** @brief Resize the image, rounding scaling factor to the closest 1/10th. */
cv::Mat FuzzyResize(const cv::Mat &image, double &scaling_factor);


/** @brief Aspect ratio-aware resize, border pixels will be filled by "padding_value". You can force the image to the top-left by setting center_output=false.
 * Yields the location of the actual image content within the output image (if location is not null).
 */
cv::Mat ResizeKeepAspectRatio(const cv::Mat &image, const cv::Size &new_size, const cv::Scalar &padding_value=cv::Scalar::all(0.0), bool center_output=true, cv::Rect *roi=nullptr);


/** @brief Convert to (single channel or 3-channel) grayscale (latter for drawing on it). */
cv::Mat Grayscale(const cv::Mat &image, bool is_rgb=false, bool output_single_channel=true);


/** @brief Mirror an image point (left<->right if flip_horizontally; top<->down if flip_vertically). */
cv::Point FlipPoint(const cv::Point &pt, const cv::Size &image_size, bool flip_horizontally, bool flip_vertically);


/** @brief Wrapper to @see FlipPoint() processing multiple points. */
std::vector<cv::Point> FlipPoints(const std::vector<cv::Point> &pts, const cv::Size &image_size, bool flip_horizontally, bool flip_vertically);


/** @brief Mirror an image point (left<->right if flip_horizontally; top<->down if flip_vertically). */
cv::Vec2d FlipVec(const cv::Vec2d &pt, const cv::Size &image_size, bool flip_horizontally, bool flip_vertically);


/** @brief Wrapper to @see FlipVec() processing multiple points. */
std::vector<cv::Vec2d> FlipVecs(const std::vector<cv::Vec2d> &pts, const cv::Size &image_size, bool flip_horizontally, bool flip_vertically);


/** @brief Replicates layers or removes alpha channel to ensure image.channels() == 3 after this call.
  * Inputs can be 1-layer, 3-layer, 4-layer images.
  */
void Ensure3Channels(cv::Mat &image);


/** @brief Similar to repmat([1,1,num_layers]); useful to generate a 3-channel image out of a single channel image. */
cv::Mat StackLayers(const cv::Mat &image, int num_layers);


/** @brief Convert from 32F/64F (assuming they are in [0,1]), 8S to 8U for visualization. */
void ConvertTo8U(const cv::Mat &image, cv::Mat &converted);
} // namespace imutils
} // namespace vcp

#endif // __VCP_IMUTILS_IMUTILS_H__
