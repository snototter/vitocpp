#include "imutils.h"
#include "matutils.h"
#include <vcp_math/common.h>
#include <vcp_math/geometry2d.h>
#include <vcp_math/conversions.h>
#include <vcp_utils/string_utils.h>

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/imgproc/imgproc.hpp>
#else
    #include <opencv2/imgproc.hpp>
#endif
#include <opencv2/imgproc/imgproc.hpp>

namespace vcp
{
namespace imutils
{

ImgTransform operator &(ImgTransform lhs, ImgTransform rhs)
{
    return static_cast<ImgTransform> (
        static_cast<std::underlying_type<ImgTransform>::type>(lhs) &
        static_cast<std::underlying_type<ImgTransform>::type>(rhs));
}

ImgTransform operator ^(ImgTransform lhs, ImgTransform rhs)
{
    return static_cast<ImgTransform> (
        static_cast<std::underlying_type<ImgTransform>::type>(lhs) ^
        static_cast<std::underlying_type<ImgTransform>::type>(rhs));
}

ImgTransform operator ~(ImgTransform rhs)
{
    return static_cast<ImgTransform> (~static_cast<std::underlying_type<ImgTransform>::type>(rhs));
}

ImgTransform& operator |=(ImgTransform &lhs, ImgTransform rhs)
{
    lhs = static_cast<ImgTransform> (
        static_cast<std::underlying_type<ImgTransform>::type>(lhs) |
        static_cast<std::underlying_type<ImgTransform>::type>(rhs));
    return lhs;
}

ImgTransform& operator &=(ImgTransform &lhs, ImgTransform rhs)
{
    lhs = static_cast<ImgTransform> (
        static_cast<std::underlying_type<ImgTransform>::type>(lhs) &
        static_cast<std::underlying_type<ImgTransform>::type>(rhs));
    return lhs;
}

ImgTransform& operator ^=(ImgTransform &lhs, ImgTransform rhs)
{
    lhs = static_cast<ImgTransform> (
        static_cast<std::underlying_type<ImgTransform>::type>(lhs) ^
        static_cast<std::underlying_type<ImgTransform>::type>(rhs));
    return lhs;
}

#define MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(st) case ImgTransform::st: rep = std::string(#st); break
std::string ImgTransformToString(const ImgTransform &t)
{
  std::string rep;
  switch (t)
  {
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(NONE);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(MIRROR_HORZ);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(MIRROR_VERT);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(ROTATE_90);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(ROTATE_180);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(ROTATE_270);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(HISTOGRAM_EQUALIZATION);

  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(COLOR_HSV);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(COLOR_LAB);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(GRAYSCALE);
  default:
    std::stringstream str;
    str << "(" << static_cast<int>(t) << ")";
    rep = str.str();
    break;
  }

  vcp::utils::string::ToLower(rep);
  return vcp::utils::string::Replace(rep, "_", "-");
}


ImgTransform ImgTransformFromToken(const std::string &s)
{
  // Convert to lowercase, remove dash and underscore.
  const std::string lower = vcp::utils::string::Trim(
        vcp::utils::string::Replace(
          vcp::utils::string::Replace(
              vcp::utils::string::Lower(s)
              , "-", ""),
          "_", ""));

  if (lower.empty() || lower.compare("none") == 0)
    return ImgTransform::NONE;

  if (lower.compare("mirrorhorz") == 0
      || lower.compare("fliplr") == 0)
    return ImgTransform::MIRROR_HORZ;

  if (lower.compare("mirrorvert") == 0
      || lower.compare("flipud") == 0)
    return ImgTransform::MIRROR_VERT;

  if (lower.compare("rotate90") == 0
      || lower.compare("rot90") == 0)
    return ImgTransform::ROTATE_90;

  if (lower.compare("rotate180") == 0
      || lower.compare("rot180") == 0)
    return ImgTransform::ROTATE_180;

  if (lower.compare("rotate270") == 0
      || lower.compare("rot270") == 0)
    return ImgTransform::ROTATE_270;

  if (lower.compare("histeq") == 0
      || lower.compare("equalizehist") == 0
      || lower.compare("equalizehistogram") == 0
      || lower.compare("histogramequalization") == 0)
    return ImgTransform::HISTOGRAM_EQUALIZATION;

  if (lower.compare("grayscale") == 0
      || lower.compare("gray") == 0
      || lower.compare("grey") == 0
      || lower.compare("greyscale") == 0)
    return ImgTransform::GRAYSCALE;

  if (lower.compare("hsv") == 0
      || lower.compare("colorhsv") == 0)
    return ImgTransform::COLOR_HSV;

  if (lower.compare("lab") == 0
      || lower.compare("colorlab") == 0)
    return ImgTransform::COLOR_LAB;

  VCP_ERROR("ImgTransformFromString(): Cannot convert '" << s << "' to ImgTransform.");
}

ImgTransform ImgTransformFromString(const std::string &s)
{
  const auto tokens = vcp::utils::string::Split(vcp::utils::string::Replace(s, ";", ","), ',');
  ImgTransform t = ImgTransform::NONE;
  for (const auto &token : tokens)
    t |= ImgTransformFromToken(token);
  return t;
}

cv::Mat MirrorHorizontally(const cv::Mat &img)
{
  if (img.empty())
    return cv::Mat();
  cv::Mat res;
  cv::flip(img, res, 1);
  return res;
}

cv::Mat MirrorVertically(const cv::Mat &img)
{
  if (img.empty())
    return cv::Mat();
  cv::Mat res;
  cv::flip(img, res, 0);
  return res;
}

cv::Mat Rotate90(const cv::Mat &img)
{
  if (img.empty())
    return cv::Mat();
  // Transpose, then flip horizontally
  cv::Mat res;
  cv::flip(img.t(), res, 1);
  return res;
}

cv::Mat Rotate180(const cv::Mat &img)
{
  if (img.empty())
    return cv::Mat();
  // Flip both vertically and horizontally
  cv::Mat res;
  cv::flip(img, res, -1);
  return res;
}

cv::Mat Rotate270(const cv::Mat &img)
{
  if (img.empty())
    return cv::Mat();
  // Transpose, then flip vertically
  cv::Mat res;
  cv::flip(img.t(), res, 0);
  return res;
}

cv::Mat HistogramEqualization(const cv::Mat &img, bool is_rgb)
{
  cv::Mat res;
  if (img.channels() == 3 || img.channels() == 4)
  {
    // Convert to luminance - chrominance
    cv::Mat ycrcb;
    if (img.channels() == 3)
    {
      cv::cvtColor(img, ycrcb, is_rgb ? CV_RGB2YCrCb : CV_BGR2YCrCb);
    }
    else
    {
      cv::Mat c3;
      cv::cvtColor(img, c3, CV_BGRA2BGR);
      cv::cvtColor(c3, ycrcb, is_rgb ? CV_RGB2YCrCb : CV_BGR2YCrCb);
    }

    // Split channels
    std::vector<cv::Mat> channels;
    cv::split(ycrcb, channels);

    // Equalize luminance channel and restore image
    cv::equalizeHist(channels[0], channels[0]);
    cv::merge(channels, ycrcb);
    if (img.channels() == 3)
    {
      cv::cvtColor(ycrcb, res, is_rgb ? CV_YCrCb2RGB : CV_YCrCb2BGR);
    }
    else
    {
      cv::Mat c3;
      cv::cvtColor(ycrcb, c3, is_rgb ? CV_YCrCb2RGB : CV_YCrCb2BGR);
      cv::cvtColor(c3, res, CV_BGR2BGRA);
    }
  }
  else if (img.channels() == 1)
  {
    cv::equalizeHist(img, res);
  }
  else
    VCP_ERROR("Only single-channel or RGB/BGR (+alpha) input images are supported for histogram equalization.");
  return res;
}

cv::Mat ConvertToHsv(const cv::Mat &img, bool is_rgb)
{
  cv::Mat res;
  if (img.channels() == 3)
    cv::cvtColor(img, res, is_rgb ? CV_RGB2HSV : CV_BGR2HSV);
  else if (img.channels() == 4)
  {
    cv::Mat c3;
    cv::cvtColor(img, c3, CV_BGRA2BGR);
    cv::cvtColor(c3, res, is_rgb ? CV_RGB2HSV : CV_BGR2HSV);
  }
  else
    VCP_ERROR("Only RGB/BGR (+alpha) input images are supported for HSV conversion.");
}

cv::Mat ConvertToLab(const cv::Mat &img, bool is_rgb)
{
  cv::Mat res;
  if (img.channels() == 3)
    cv::cvtColor(img, res, is_rgb ? CV_RGB2Lab : CV_BGR2Lab);
  else if (img.channels() == 4)
  {
    cv::Mat c3;
    cv::cvtColor(img, c3, CV_BGRA2BGR);
    cv::cvtColor(c3, res, is_rgb ? CV_RGB2Lab : CV_BGR2Lab);
  }
  else
    VCP_ERROR("Only RGB/BGR (+alpha) input images are supported for L*a*b* conversion.");
}

cv::Mat ApplyImageTransformation(const cv::Mat &img, const ImgTransform &transform)
{
  cv::Mat res;
  if (img.empty())
    return res;

  if (transform == ImgTransform::NONE)
      return img.clone();

  res = img.clone();
  if ((transform & ImgTransform::MIRROR_HORZ) != ImgTransform::NONE)
    res = MirrorHorizontally(res);

  if ((transform & ImgTransform::MIRROR_VERT) != ImgTransform::NONE)
    res = MirrorVertically(res);

  if ((transform & ImgTransform::ROTATE_90) != ImgTransform::NONE)
    res = Rotate90(res);

  if ((transform & ImgTransform::ROTATE_180) != ImgTransform::NONE)
    res = Rotate180(res);

  if ((transform & ImgTransform::ROTATE_270) != ImgTransform::NONE)
    res = Rotate270(res);

  if ((transform & ImgTransform::HISTOGRAM_EQUALIZATION) != ImgTransform::NONE)
    res = HistogramEqualization(res);

  if ((transform & ImgTransform::COLOR_HSV) != ImgTransform::NONE)
    res = ConvertToHsv(res);

  if ((transform & ImgTransform::COLOR_LAB) != ImgTransform::NONE)
    res = ConvertToLab(res);

  // Grayscale conversion should happen last
  if ((transform & ImgTransform::GRAYSCALE) != ImgTransform::NONE)
    res = Grayscale(res);

  return res;
}

bool IsPointInsideImage(const cv::Point &pt, const cv::Size &image_size)
{
  const cv::Rect img_rect(0, 0, image_size.width, image_size.height);
  return img_rect.contains(pt);
}


bool IsPointInsideImage(const cv::Vec2d &pt, const cv::Size &image_size)
{
  // Because of using double, check for inclusive right/bottom bounds!
  return (pt[0] >= 0.0) && (pt[0] <= image_size.width-1.0)
      && (pt[1] >= 0.0) && (pt[1] <= image_size.height-1.0);
}


cv::Mat CenterCrop(const cv::Mat &image, const cv::Size &crop_size)
{
  const int left = (image.cols - crop_size.width)/2;
  const int top = (image.rows - crop_size.height)/2;
  cv::Rect rect(left, top, crop_size.width, crop_size.height);

  // Clip to image boundaries
  int l = rect.x;
  const int r = std::min(l+rect.width-1, image.cols-1);
  l = std::max(l, 0);
  int t = rect.y;
  const int b = std::min(t+rect.height-1, image.rows-1);
  t = std::max(t, 0);
  rect.x = l;
  rect.y = t;
  rect.width = r - l + 1;
  rect.height = b - t + 1;

  return image(rect).clone();
}


cv::Size MaxAxisAlignedCropSize(const cv::Size &image_size, double theta)
{
  // Based on https://stackoverflow.com/a/16778797/400948
  cv::Size crop_size;
  if (image_size.width > 0 && image_size.height > 0)
  {
    const bool width_is_longer = image_size.width >= image_size.height;
    int side_long = image_size.height;
    int side_short = image_size.width;
    if (width_is_longer)
    {
      side_long = image_size.width;
      side_short = image_size.height;
    }

    // Solutions for angle, -angle and 180-angle are all the same, so just
    // look at the first quadrant and the absolute values of sin/cos:
    const double sin_a = std::abs(std::sin(theta));
    const double cos_a = std::abs(std::cos(theta));

    double wr, hr;
    if (side_short <= 2.0*sin_a*cos_a*side_long || std::abs(sin_a-cos_a) < 1e-10)
    {
      // Half-constrained: two crop corners touch the longer side, the other
      // two are on the mid-line parallel to the longer line
      const double x = 0.5*side_short;
      if (width_is_longer)
      {
        wr = x / sin_a;
        hr = x / cos_a;
      }
      else
      {
        wr = x / cos_a;
        hr = x / sin_a;
      }
    }
    else
    {
      // Fully constrained case: crop touches all 4 sides
      const double cos_2a = cos_a*cos_a - sin_a*sin_a;
      wr = (image_size.width*cos_a - image_size.height*sin_a) / cos_2a;
      hr = (image_size.height*cos_a - image_size.width*sin_a) / cos_2a;
    }
    crop_size.width = static_cast<int>(std::ceil(wr));
    crop_size.height = static_cast<int>(std::ceil(hr));
  }
  return crop_size;
}


void RotateImage(cv::Mat &image, double theta, bool crop)
{
  const cv::Point2f image_center((float)image.cols/2.0f, (float)image.rows/2.0f);
  cv::Mat M = cv::getRotationMatrix2D(image_center, vcp::math::Rad2Deg(theta), 1.0);
  if (crop)
  {
    // Check, how big the rotated image will be
    const cv::RotatedRect rotated = RotateRect(ImageRect(image), ImageCenter(image), theta);
    const cv::Size rotated_size = rotated.boundingRect().size();

    // Adjust transformation matrix (shift, s.t. all image pixels are inside the image)
    double xoffset = (rotated_size.width - image.cols)/2.0;
    double yoffset = (rotated_size.height - image.rows)/2.0;
    M.at<double>(0,2) += xoffset;
    M.at<double>(1,2) += yoffset;

    // Rotate image
    cv::Mat dst;
    cv::warpAffine(image, dst, M, rotated_size);

    const cv::Size crop_size = MaxAxisAlignedCropSize(image.size(), theta);
    image = CenterCrop(dst, crop_size);
  }
  else
  {
    cv::warpAffine(image, image, M, image.size());
  }
}


void RotateImage(cv::Mat &image, const cv::Point &anchor, double theta, int border_mode)
{
  const cv::Mat M = cv::getRotationMatrix2D(anchor, vcp::math::Rad2Deg(theta), 1.0);
  cv::warpAffine(image, image, M, image.size(), cv::INTER_LINEAR, border_mode);
}


std::vector<cv::RotatedRect> RotateAnnotatedImage(cv::Mat &image, const std::vector<cv::Rect> &bounding_boxes, double theta, bool crop)
{
  std::vector<cv::RotatedRect> rotated_boxes;
  rotated_boxes.reserve(bounding_boxes.size());
  const cv::Vec2d orig_image_center = ImageCenter(image);

  RotateImage(image, theta, crop);
  const cv::Vec2d rot_image_center = ImageCenter(image);

  for(const cv::Rect &rect : bounding_boxes)
  {
    cv::RotatedRect rrect = RotateRect(rect, orig_image_center, theta);
    if (crop)
    {
      // Both image and rects were rotated around the (original) image center.
      // Since the user requested a center crop, we have to adjust the position of the bounding boxes:
      rrect.center.x = rrect.center.x - orig_image_center.val[0] + rot_image_center.val[0];
      rrect.center.y = rrect.center.y - orig_image_center.val[1] + rot_image_center.val[1];
    }
    rotated_boxes.push_back(rrect);
  }

  return rotated_boxes;
}


cv::Vec2d RotateImageVector(const cv::Vec2d &vec, const cv::Size &image_size, double theta)
{
  const cv::Vec2d rotation_center(image_size.width/2.0, image_size.height/2.0);
  const cv::Vec2d rotated = vcp::math::geo2d::RotateVector(vec - rotation_center, -theta) + rotation_center; // RotateVector uses right-handed, but we require left-handed ==> negative angle!
  return rotated;
}


cv::RotatedRect RotateRect(const cv::Rect &rect, const cv::Vec2d &rotation_center, double theta)
{
  // https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html?#getrotationmatrix2d
  // https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_geometric_transformations/py_geometric_transformations.html
  const cv::Vec2d center(rect.x + rect.width/2.0, rect.y + rect.height/2.0);
  const cv::Vec2d rotated_center = vcp::math::geo2d::RotateVector(center - rotation_center, -theta) + rotation_center;
  return cv::RotatedRect(vcp::convert::ToPoint2f(rotated_center), cv::Size2f(rect.width, rect.height), -vcp::math::Rad2Deg(theta));
}


cv::Mat FuzzyResize(const cv::Mat &image, double &scaling_factor)
{
  cv::Mat dst;
  scaling_factor = std::round(10.0 * scaling_factor)/10.0;
  cv::resize(image, dst, cv::Size(), scaling_factor, scaling_factor);
  // Store actual scaling factor
  scaling_factor = std::min(static_cast<double>(dst.cols) / static_cast<double>(image.cols),
                            static_cast<double>(dst.rows) / static_cast<double>(image.rows));
  return dst;
}


cv::Mat ResizeKeepAspectRatio(const cv::Mat &image, const cv::Size &new_size, const cv::Scalar &padding_value, bool center_output, cv::Rect *roi)
{
  const double sx = static_cast<double>(new_size.width) / static_cast<double>(image.cols);
  const double sy = static_cast<double>(new_size.height) / static_cast<double>(image.rows);
  const double scaling_factor = std::min(sx, sy);
  cv::Mat res;
  cv::resize(image, res, cv::Size(), scaling_factor, scaling_factor);

  cv::Mat dst(new_size.height, new_size.width, image.type(), padding_value);
  const int offset_x = center_output ? (dst.cols - res.cols) / 2 : 0;
  const int offset_y = center_output ? (dst.rows - res.rows) / 2 : 0;
  // To be safe (in case there were rounding issues):
  const int width = std::min(res.cols, new_size.width);
  const int height = std::min(res.rows, new_size.height);
  cv::Mat dst_roi = dst(cv::Rect(offset_x, offset_y, width, height));

  res(cv::Rect(0,0,width,height)).copyTo(dst_roi);
  if (roi)
    *roi = cv::Rect(offset_x, offset_y, width, height);

  return dst;
}


cv::Mat Grayscale(const cv::Mat &image, bool is_rgb, bool output_single_channel)
{
  // Check layers
  cv::Mat gray;
  if (image.channels() == 3)
    cv::cvtColor(image, gray, is_rgb ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
  else if (image.channels() == 4)
    cv::cvtColor(image, gray, is_rgb ? cv::COLOR_RGBA2GRAY : cv::COLOR_BGRA2GRAY);
  else if (image.channels() == 1)
    gray = image;
  else
    VCP_ERROR("Grayscale(): Cannot convert an image with " << image.channels() << " layers to grayscale!");

  if (output_single_channel)
    return gray;

  return StackLayers(gray, 3);
}


cv::Point FlipPoint(const cv::Point &pt, const cv::Size &image_size, bool flip_horizontally, bool flip_vertically)
{
  cv::Point flipped(pt);
  if (flip_horizontally)
    flipped.x = image_size.width - pt.x - 1;
  if (flip_vertically)
    flipped.y = image_size.height - pt.y - 1;
  return flipped;
}


std::vector<cv::Point> FlipPoints(const std::vector<cv::Point> &pts, const cv::Size &image_size, bool flip_horizontally, bool flip_vertically)
{
  std::vector<cv::Point> flipped;
  flipped.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); ++i)
    flipped.push_back(FlipPoint(pts[i], image_size, flip_horizontally, flip_vertically));
  return flipped;
}


cv::Vec2d FlipVec(const cv::Vec2d &pt, const cv::Size &image_size, bool flip_horizontally, bool flip_vertically)
{
  cv::Vec2d flipped(pt);
  if (flip_horizontally)
    flipped.val[0] = image_size.width - pt.val[0] - 1;
  if (flip_vertically)
    flipped.val[1] = image_size.height - pt.val[1] - 1;
  return flipped;
}

std::vector<cv::Vec2d> FlipVecs(const std::vector<cv::Vec2d> &pts, const cv::Size &image_size, bool flip_horizontally, bool flip_vertically)
{
  std::vector<cv::Vec2d> flipped;
  flipped.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); ++i)
    flipped.push_back(FlipVec(pts[i], image_size, flip_horizontally, flip_vertically));
  return flipped;
}


void Ensure3Channels(cv::Mat &image)
{
  if (image.channels() == 3)
    return;

  if (image.channels() == 1)
    image = StackLayers(image, 3);
  else if (image.channels() > 3)
  {
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    while (channels.size() > 3)
      channels.pop_back();
    cv::merge(channels, image);
  }
  else
  {
    VCP_ERROR("Ensure3Channels(): Cannot convert a " << image.channels() << " channel image to 3 channels!");
  }
}


void ConvertTo8U(const cv::Mat &image, cv::Mat &converted)
{
  switch (image.depth())
  {
  case CV_8U:
    converted = image.clone();
    break;

  case CV_8S:
    converted = image + cv::Scalar::all(127.0);
    break;

  case CV_32F:
  case CV_64F:
    image.convertTo(converted, CV_8U, 255.0);
    break;

  default:
    VCP_ERROR("Converting " << vcp::imutils::CVMatDepthToString(image.depth(), image.channels()) << " is not yet supported!");
  }
}


cv::Mat StackLayers(const cv::Mat &image, int num_layers)
{
  if (image.channels() != 1)
    VCP_ERROR("Currently, only stacking single-channel images is supported.");

  cv::Mat stacked;
  std::vector<cv::Mat> layers;
  layers.reserve(num_layers);
  for (int i = 0; i < num_layers; ++i)
    layers.push_back(image);
  cv::merge(layers, stacked);
  return stacked;
}
} // namespace imutils
} // namespace vcp
