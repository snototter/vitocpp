#include "imutils.h"
#include "matutils.h"
#include "constants.h"
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

#define VCP_VERBOSE_TIMING
#include <vcp_utils/timing_utils.h>

namespace vcp
{
namespace imutils
{
std::ostream& operator<<(std::ostream & os, const ImgTransform &t)
{
  os << ImgTransformToString(t);
  return os;
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

  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(DEPTH2SURFACENORMALS);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(COLOR_SURFACENORMALS_RGB);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(COLOR_SURFACENORMALS_BGR);

#ifdef VCP_IMUTILS_WITH_COLORNAMES
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(COLOR_RGB2COLORNAME);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(COLOR_BGR2COLORNAME);
#endif

  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(COLOR_GRAY2RGB);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(COLOR_RGB2HSV);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(COLOR_BGR2HSV);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(COLOR_RGB2LAB);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(COLOR_BGR2LAB);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(COLOR_RGB2GRAY);
  MAKE_IMAGETRANSFORMATION_TO_STRING_CASE(COLOR_BGR2GRAY);
//  default:
//    std::stringstream str;
//    str << "(" << static_cast<int>(t) << ")";
//    rep = str.str();
//    break;
  }

  return vcp::utils::string::Canonic(rep, false);
}


ImgTransform ImgTransformFromString(const std::string &s)
{
  const std::string lower = vcp::utils::string::Canonic(s, true);

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

  if (lower.compare("depth2sn") == 0
      || lower.compare("depth2surfnorm") == 0
      || lower.compare("depth2surfacenormals") == 0)
    return ImgTransform::DEPTH2SURFACENORMALS;

  if (lower.compare("sn2rgb") == 0
      || lower.compare("surfnorm2rgb") == 0
      || lower.compare("surfacenormals2rgb") == 0)
    return ImgTransform::COLOR_SURFACENORMALS_RGB;

  if (lower.compare("sn2bgr") == 0
      || lower.compare("surfnorm2bgr") == 0
      || lower.compare("surfacenormals2bgr") == 0)
    return ImgTransform::COLOR_SURFACENORMALS_BGR;

#ifdef VCP_IMUTILS_WITH_COLORNAMES
  if (lower.compare("rgb2cn") == 0
      || lower.compare("rgb2colorname") == 0)
    return ImgTransform::COLOR_RGB2COLORNAME;

  if (lower.compare("bgr2cn") == 0
      || lower.compare("bgr2colorname") == 0)
    return ImgTransform::COLOR_BGR2COLORNAME;

  if (lower.compare("cn") == 0
      || lower.compare("colorname") == 0)
  {
    VCP_LOG_WARNING("Ambiguous ImgTransform '" << s << "' will be converted to RGB2COLORNAME. Consider replacing it by explicit 'bgr2cn' or 'rgb2cn'.");
    return ImgTransform::COLOR_RGB2COLORNAME;
  }
  //TODO add cnprobs (11-channel output!)
#endif

  if (lower.compare("rgb2hsv") == 0)
    return ImgTransform::COLOR_RGB2HSV;
  if (lower.compare("bgr2hsv") == 0)
    return ImgTransform::COLOR_BGR2HSV;
  if (lower.compare("hsv") == 0
      || lower.compare("colorhsv") == 0)
  {
    VCP_LOG_WARNING("Ambiguous ImgTransform '" << s << "' will be converted to RGB2HSV. Consider replacing it by explicit 'bgr2hsv' or 'rgb2hsv'.");
    return ImgTransform::COLOR_RGB2HSV;
  }

  if (lower.compare("rgb2lab") == 0)
    return ImgTransform::COLOR_RGB2LAB;
  if (lower.compare("bgr2lab") == 0)
    return ImgTransform::COLOR_BGR2LAB;
  if (lower.compare("lab") == 0
      || lower.compare("colorlab") == 0)
  {
    VCP_LOG_WARNING("Ambiguous ImgTransform '" << s << "' will be converted to RGB2LAB. Consider replacing it by explicit 'bgr2lab' or 'rgb2lab'.");
    return ImgTransform::COLOR_RGB2LAB;
  }

  if (lower.compare("gray2rgb") == 0
      || lower.compare("gray2bgr") == 0
      || lower.compare("grey2rgb") == 0
      || lower.compare("grey2bgr") == 0)
    return ImgTransform::COLOR_GRAY2RGB;

  if (lower.compare("rgb2gray") == 0
      || lower.compare("rgb2grey") == 0)
    return ImgTransform::COLOR_RGB2GRAY;
  if (lower.compare("bgr2gray") == 0
      || lower.compare("bgr2grey") == 0)
    return ImgTransform::COLOR_BGR2GRAY;
  if (lower.compare("gray") == 0
      || lower.compare("grayscale") == 0
      || lower.compare("grey") == 0
      || lower.compare("greyscale") == 0)
  {
    VCP_LOG_WARNING("Ambiguous ImgTransform '" << s << "' will be converted to RGB2GRAY. Consider replacing it by explicit 'bgr2gray' or 'rgb2gray'.");
    return ImgTransform::COLOR_RGB2GRAY;
  }

  VCP_ERROR("ImgTransformFromString(): Cannot convert '" << s << "' to ImgTransform.");
}


std::vector<ImgTransform> ImgTransformsFromString(const std::string &s)
{
  std::vector<ImgTransform> transforms;
  const auto tokens = vcp::utils::string::Split(vcp::utils::string::Replace(s, ";", ","), ',');
  for (const auto &token : tokens)
    transforms.push_back(ImgTransformFromString(token));

  return transforms;
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

template <typename _T>
cv::Mat SurfaceNormalsC1Helper(const cv::Mat &depth)
{
//  // Barron & Malik
//  cv::Mat d32;
//  depth.convertTo(d32, CV_32F);

//  // filters
//  cv::Mat f1 = (cv::Mat_<float>(3, 3) << 1,  2,  1,
//                                     0,  0,  0,
//                                    -1, -2, -1) / 8;

//  cv::Mat f2 = (cv::Mat_<float>(3, 3) << 1, 0, -1,
//                                     2, 0, -2,
//                                     1, 0, -1) / 8;

//  cv::Mat f1m, f2m;
//  cv::flip(f1, f1m, 0);
//  cv::flip(f2, f2m, 1);
//  cv::Mat n1, n2;
//  cv::filter2D(d32, n1, -1, f1m, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
//  cv::filter2D(d32, n2, -1, f2m, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);

//  n1 *= -1;
//  n2 *= -1;

//  cv::Mat temp = n1.mul(n1) + n2.mul(n2) + 1;
//  cv::sqrt(temp, temp);

//  cv::Mat N3 = 1 / temp;
//  cv::Mat N1 = n1.mul(N3);
//  cv::Mat N2 = n2.mul(N3);

//  std::vector<cv::Mat> N;
//  N.push_back(N1);
//  N.push_back(N2);
//  N.push_back(N3);

//  cv::Mat normals;
//  cv::merge(N, normals);
//  return normals;
  cv::Mat normals = cv::Mat::zeros(depth.size(), CV_64FC3);

  cv::Vec3d dir;
  double dzdx, dzdy;
  for (int row = 1; row < depth.rows-1; ++row)
  {
    for (int col = 1; col < depth.cols-1; ++col)
    {
      if (depth.at<_T>(row, col) > static_cast<_T>(0))
      {
        // Save some computation (we can replace the cross product by the
        // following derivation), based on https://stackoverflow.com/a/34644939/400948
        dzdx = (depth.at<_T>(row, col+1) - depth.at<_T>(row, col-1)) / 2.0;
        dzdy = (depth.at<_T>(row+1, col) - depth.at<_T>(row-1, col)) / 2.0;
        dir = cv::Vec3d(-dzdx, -dzdy, 1.0);
        normals.at<cv::Vec3d>(row, col) = cv::normalize(dir); // Already takes care of 0-length vectors.
      }
    }
  }
  return normals;
}


template <typename _T>
cv::Mat SurfaceNormalsC3Helper(const cv::Mat &xyz)
{
  //TODO compute normals from 3d point coordinates
  VCP_ERROR("Not yet implemented!");
}

cv::Mat ComputeSurfaceNormals(const cv::Mat &depth)
{
  // For single-channel inputs, we assume the depth has already been
  // properly scaled, such that we can take image coordinates for x/y
  // Otherwise, inputs must have 3 channels, x/y/z.
  if (depth.channels() != 1 && depth.channels() != 3)
    VCP_ERROR("Input depth must be a single-channel depth image or 3-channel XYZ.");

  switch (depth.type())
  {
    case CV_16UC1:
      return SurfaceNormalsC1Helper<unsigned short>(depth);
    case CV_16UC3:
      return SurfaceNormalsC3Helper<unsigned short>(depth);
    case CV_16SC1:
      return SurfaceNormalsC1Helper<short>(depth);
    case CV_16SC3:
      return SurfaceNormalsC3Helper<short>(depth);
    case CV_32FC1:
      return SurfaceNormalsC1Helper<float>(depth);
    case CV_32FC3:
      return SurfaceNormalsC3Helper<float>(depth);
    case CV_32SC1:
      return SurfaceNormalsC1Helper<int>(depth);
    case CV_32SC3:
      return SurfaceNormalsC3Helper<int>(depth);
    case CV_64FC1:
      return SurfaceNormalsC1Helper<double>(depth);
    case CV_64FC3:
      return SurfaceNormalsC3Helper<double>(depth);
    default:
      VCP_ERROR("ComputeSurfaceNormals() does not support depth image type '" << vcp::imutils::CVMatDepthToString(depth.depth(), depth.channels()));
  }
}

template<typename _T>
cv::Mat ColorizeSurfaceNormalsHelper(const cv::Mat &normals, bool output_bgr)
{
  cv::Mat colorized(normals.size(), CV_8UC3);

  // Efficiently iterate the normal image
  int cols = normals.cols;
  int rows = normals.rows;
  if (normals.isContinuous() && colorized.isContinuous())
  {
    cols = rows * cols;
    rows = 1;
  }

  cv::Vec3b color;
  // Shading direction - naive solution: red light from the left, green
  // light from the top, and blue light from the front/camera center
  const cv::Vec<_T, 3> shading_dir_red = cv::normalize(cv::Vec<_T, 3>(1, 0, 0));
  const cv::Vec<_T, 3> shading_dir_green = cv::normalize(cv::Vec<_T, 3>(0, 1, 0));
  const cv::Vec<_T, 3> shading_dir_blue = cv::normalize(cv::Vec<_T, 3>(0, 0, -1));
  const double shading_factor = 0.2;

  for (int row = 0; row < rows; ++row)
  {
    const _T* nptr = normals.ptr<_T>(row);
    uchar* cptr = colorized.ptr<uchar>(row);

    for (int col = 0; col < cols; ++col)
    {
      const _T nx = nptr[0];
      const _T ny = nptr[1];
      const _T nz = nptr[2];
      if (nx*nx + ny*ny + nz*nz > static_cast<_T>(0))
      {
        // Normal mapping:
        // X: -1 to +1  ==> Red:     0 to 255
        // Y: -1 to +1  ==> Green:   0 to 255
        // Z:  0 to -1  ==> Blue:  128 to 255
        color[output_bgr ? 2 : 0] = static_cast<uchar>((nx + 1) / 2.0 * 255.0);
        color[1] = static_cast<uchar>((ny + 1) / 2.0 * 255);
        color[output_bgr ? 0 : 2] = static_cast<uchar>(std::fabs(nz)*127 + 128);

        // Shading, adapted from https://stackoverflow.com/a/34644939/400948
        const _T shading_red = std::min(static_cast<_T>(1), std::max(static_cast<_T>(0),
            static_cast<_T>((1.0 - shading_factor) +
                            shading_factor * (shading_dir_red[0] * nx + shading_dir_red[1] * ny + shading_dir_red[2] * nz))));
        const _T shading_green = std::min(static_cast<_T>(1), std::max(static_cast<_T>(0),
            static_cast<_T>((1.0 - shading_factor) +
                            shading_factor * (shading_dir_green[0] * nx + shading_dir_green[1] * ny + shading_dir_green[2] * nz))));
        const _T shading_blue = std::min(static_cast<_T>(1), std::max(static_cast<_T>(0),
            static_cast<_T>((1.0 - shading_factor) +
                            shading_factor * (shading_dir_blue[0] * nx + shading_dir_blue[1] * ny + shading_dir_blue[2] * nz))));

        cptr[0] = static_cast<uchar>((output_bgr ? shading_blue : shading_red) * color[0]);
        cptr[1] = static_cast<uchar>(shading_green * color[1]);
        cptr[2] = static_cast<uchar>((output_bgr ? shading_red : shading_blue) * color[2]);
      }
      else
      {
        cptr[0] = 0;
        cptr[1] = 0;
        cptr[2] = 0;
      }
      cptr += 3;
      nptr += 3;
    }
  }
  return colorized;
}


cv::Mat ColorizeSurfaceNormals(const cv::Mat &normals, bool output_bgr)
{
  if (normals.type() == CV_32FC3)
    return ColorizeSurfaceNormalsHelper<float>(normals, output_bgr);

  if (normals.type() == CV_64FC3)
    return ColorizeSurfaceNormalsHelper<double>(normals, output_bgr);

  VCP_ERROR("ColorizeSurfaceNormals() input must be 32FC3 or 64FC3, you provided "
            << vcp::imutils::CVMatDepthToString(normals.depth(), normals.channels()) << ".");
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
  return res;
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
  return res;
}

#ifdef VCP_IMUTILS_WITH_COLORNAMES
template <bool PROBS>
cv::Mat ColorNameHelper(const cv::Mat &rgb, bool output_rgb)
{
  VCP_INIT_TIC_TOC;
  VCP_TIC;
  if (rgb.type() != CV_8UC3)
    VCP_ERROR("CN (color name) conversion util expects RGB uint8 input, not " << CVMatDepthToString(rgb.depth(), rgb.channels()));

  cv::Mat out;
  if (PROBS)
    out = cv::Mat(rgb.rows, rgb.cols, CV_MAKETYPE(CV_32F, 11));
  else
    out = cv::Mat(rgb.rows, rgb.cols, CV_8UC3);

  // Optimize the per-pixel loop if possible:
  int rows = rgb.rows;
  int cols = rgb.cols;
  if (rgb.isContinuous() && out.isContinuous())
  {
    cols = rows * cols;
    rows = 1;
  }

  unsigned index;
  for (int row = 0; row < rows; ++row)
  {
    const uchar* rgb_ptr = rgb.ptr<uchar>(row);
    float* probs_ptr = PROBS ? out.ptr<float>(row) : nullptr;
    uchar* color_ptr = PROBS ? nullptr : out.ptr<uchar>(row);

    for (int col = 0; col < cols; col++)
    {
      index =
          (rgb_ptr[0] >> 3)
          + (static_cast<unsigned>(rgb_ptr[1] >> 3) << 5)
          + (static_cast<unsigned>(rgb_ptr[2] >> 3) << 10);

      if (PROBS)
      {
        probs_ptr[0] = kColorNameProbabilitiesRgb[index][0];
        probs_ptr[1] = kColorNameProbabilitiesRgb[index][1];
        probs_ptr[2] = kColorNameProbabilitiesRgb[index][2];
        probs_ptr[3] = kColorNameProbabilitiesRgb[index][3];
        probs_ptr[4] = kColorNameProbabilitiesRgb[index][4];
        probs_ptr[5] = kColorNameProbabilitiesRgb[index][5];
        probs_ptr[6] = kColorNameProbabilitiesRgb[index][6];
        probs_ptr[7] = kColorNameProbabilitiesRgb[index][7];
        probs_ptr[8] = kColorNameProbabilitiesRgb[index][8];
        probs_ptr[9] = kColorNameProbabilitiesRgb[index][9];
        probs_ptr[10] = kColorNameProbabilitiesRgb[index][10];
        probs_ptr += 11;
      }
      else
      {
        if (output_rgb)
        {
          color_ptr[0] = kColorNameColorsRgb[kColorNameColorLookup[index]][0];
          color_ptr[1] = kColorNameColorsRgb[kColorNameColorLookup[index]][1];
          color_ptr[2] = kColorNameColorsRgb[kColorNameColorLookup[index]][2];
        }
        else
        {
          color_ptr[0] = kColorNameColorsRgb[kColorNameColorLookup[index]][2];
          color_ptr[1] = kColorNameColorsRgb[kColorNameColorLookup[index]][1];
          color_ptr[2] = kColorNameColorsRgb[kColorNameColorLookup[index]][0];
        }
        color_ptr += 3;
      }
      rgb_ptr += 3;
    }
  }
  VCP_TOC("COLORNAMES");
  return out;
}

cv::Mat ConvertToColorName(const cv::Mat &img, bool is_rgb)
{
  cv::Mat res, cvt;
  if (img.channels() == 3)
  {
    if (is_rgb)
      res = ColorNameHelper<false>(img, true);
    else
    {
      cv::cvtColor(img, cvt, CV_BGR2RGB);
      res = ColorNameHelper<false>(cvt, false);
    }
  }
  else if (img.channels() == 4)
  {
    if (is_rgb)
    {
      cv::cvtColor(img, cvt, CV_RGBA2RGB);
      res = ColorNameHelper<false>(cvt, true);
    }
    else
    {
      cv::cvtColor(img, cvt, CV_BGRA2RGB);
      res = ColorNameHelper<false>(cvt, false);
    }
  }
  else
    VCP_ERROR("Only RGB/BGR (+alpha) input images are supported for CN (color name) conversion.");
  return res;
}

cv::Mat ConvertToColorNameFeature(const cv::Mat &img, bool is_rgb)
{
  cv::Mat res, cvt;
  if (img.channels() == 3)
  {
    if (is_rgb)
      res = ColorNameHelper<true>(img, true);
    else
    {
      cv::cvtColor(img, cvt, CV_BGR2RGB);
      res = ColorNameHelper<true>(cvt, false);
    }
  }
  else if (img.channels() == 4)
  {
    if (is_rgb)
    {
      cv::cvtColor(img, cvt, CV_RGBA2RGB);
      res = ColorNameHelper<true>(cvt, true);
    }
    else
    {
      cv::cvtColor(img, cvt, CV_BGRA2RGB);
      res = ColorNameHelper<true>(cvt, false);
    }
  }
  else
    VCP_ERROR("Only RGB/BGR (+alpha) input images are supported for CN (color name) conversion.");
  return res;
}
#endif

cv::Mat ApplyImageTransformation(const cv::Mat &img, const ImgTransform &transform)
{
  if (img.empty())
    return cv::Mat();

  switch (transform)
  {
    case ImgTransform::NONE:
      return img.clone();

    case ImgTransform::MIRROR_HORZ:
      return MirrorHorizontally(img);
    case ImgTransform::MIRROR_VERT:
      return MirrorVertically(img);

    case ImgTransform::ROTATE_90:
      return Rotate90(img);
    case ImgTransform::ROTATE_180:
      return Rotate180(img);
    case ImgTransform::ROTATE_270:
      return Rotate270(img);

    case ImgTransform::HISTOGRAM_EQUALIZATION:
      return HistogramEqualization(img);

    case ImgTransform::DEPTH2SURFACENORMALS:
      return ComputeSurfaceNormals(img);

    case ImgTransform::COLOR_SURFACENORMALS_RGB:
      return ColorizeSurfaceNormals(img, false);
    case ImgTransform::COLOR_SURFACENORMALS_BGR:
      return ColorizeSurfaceNormals(img, true);


#ifdef VCP_IMUTILS_WITH_COLORNAMES
    case ImgTransform::COLOR_RGB2COLORNAME:
      return ConvertToColorName(img, true);
    case ImgTransform::COLOR_BGR2COLORNAME:
      return ConvertToColorName(img, false);
#endif

    case ImgTransform::COLOR_GRAY2RGB:
      return Grayscale(img, false, false);
    case ImgTransform::COLOR_RGB2HSV:
      return ConvertToHsv(img, true);
    case ImgTransform::COLOR_BGR2HSV:
      return ConvertToHsv(img, false);
    case ImgTransform::COLOR_RGB2LAB:
      return ConvertToLab(img, true);
    case ImgTransform::COLOR_BGR2LAB:
      return ConvertToLab(img, false);
    case ImgTransform::COLOR_RGB2GRAY:
      return Grayscale(img, true, true);
    case ImgTransform::COLOR_BGR2GRAY:
      return Grayscale(img, false, true);
  }
  return cv::Mat();
}

cv::Mat ApplyImageTransformations(const cv::Mat &img, const std::vector<ImgTransform> &transforms)
{
  if (img.empty())
    return cv::Mat();
  if (transforms.empty())
    return img.clone();

  cv::Mat res;
  for (const auto &t : transforms)
  {
    const cv::Mat &input = res.empty() ? img : res;
    res = ApplyImageTransformation(input, t);
  }
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
