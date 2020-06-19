#include "drawing.h"
#include "pseudocolor.h"
#include <vcp_imutils/imutils.h>
#include <vcp_imutils/matutils.h>
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_math/geometry3d.h>
#include <vcp_math/geometry2d.h>
#include <vcp_math/conversions.h>
#include <vcp_math/common.h>
#include <vcp_utils/sort_utils.h>
#include <limits>

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/imgproc/imgproc.hpp>
#else
    #include <opencv2/imgproc.hpp>
#endif

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::imvis::drawing"
namespace vcp
{
namespace imvis
{
namespace drawing
{
namespace utils
{

/** @brief return V/homogeneous coordinate, where V = scale * H * [pt, 1]^T. */
cv::Vec2d ProjectVecWithScale(const cv::Mat &P, const cv::Vec3d &pt, double pixel_scale)
{
  const cv::Vec2d prj = vcp::math::geo3d::ProjectVec(P, pt);
  return pixel_scale * prj;
}


/** @brief Same as @see ProjectVecWithScale, but casts to a (int) point. */
cv::Point ProjectPointWithScale(const cv::Mat &P, const cv::Vec3d &pt, double pixel_scale)
{
  return vcp::convert::ToPoint(ProjectVecWithScale(P, pt, pixel_scale));
}


/** @brief return scale * H * [pt, 1]^T. */
cv::Vec2d TransformVecWithScale(const cv::Mat &H, const cv::Vec2d &pt, double pixel_scale=1.0)
{
  const cv::Vec2d prj = vcp::math::geo3d::ProjectVec(H, cv::Vec3d(pt[0], pt[1], 1.0));
  return pixel_scale * prj;
}


/** @brief Utility to project a line into the image. */
vcp::math::geo2d::Line2d GetEdgeProjections(const vcp::math::geo3d::Line3d &edge, const cv::Mat &P, const cv::Size &image_size, double scale_image_points)
{
  const cv::Vec2d from = utils::ProjectVecWithScale(P, edge.From(), scale_image_points);
  const cv::Vec2d to = utils::ProjectVecWithScale(P, edge.To(), scale_image_points);
  const vcp::math::geo2d::Line2d prj(from, to);
  return vcp::math::geo2d::ClipLineSegmentByRectangle(prj, cv::Rect(0, 0, image_size.width, image_size.height));
}


/** @brief Returns the top point of the line segment. */
cv::Point GetEdgeTop(const vcp::math::geo2d::Line2d &edge)
{
  if (edge.From().val[1] < edge.To().val[1])
    return vcp::convert::ToPoint(edge.From());
  return vcp::convert::ToPoint(edge.To());
}


/** @brief Returns the bottom point of the line segment. */
cv::Point GetEdgeBottom(const vcp::math::geo2d::Line2d &edge)
{
  if (edge.From().val[1] > edge.To().val[1])
    return vcp::convert::ToPoint(edge.From());
  return vcp::convert::ToPoint(edge.To());
}


/** @brief Apply the 3x4 transformation on all corners of the given box. */
inline Box3d NormalizeBox3dCoordinates(const Box3d &box, const cv::Mat &Rt)
{
  Box3d normalized;
  for (const cv::Vec3d &v : box.TopCorners())
    normalized.AddTopCorner(vcp::math::geo3d::Apply3x4(Rt, v));
  for (const cv::Vec3d &v : box.BottomCorners())
    normalized.AddBottomCorner(vcp::math::geo3d::Apply3x4(Rt, v));
  return normalized;
}


/** @brief Ensures that val is odd (by adding 1 if it's not). */
template <typename T>
inline T MakeOdd(T val)
{
  return (val % 2 == 0) ? val + static_cast<T>(1) : val;
}


/** @brief Check if color is white. */
inline bool IsWhite(const cv::Scalar &v)
{
  return !(v[0] < 255.0 || v[1] < 255.0 || v[2] < 255.0);
}


/** @brief Check if color is black. */
inline bool IsBlack(const cv::Scalar &v)
{
  return !(v[0] > 0.0 || v[1] > 0.0 || v[2] > 0.0);
}

/** @brief Check if color is shade of gray. */
inline bool IsShadeOfGray(const cv::Scalar &v)
{
  const double d1 = std::abs(v[0]-v[1]);
  const double d2 = std::abs(v[0]-v[2]);
  return d1 < 5.0 && d2 < 5.0; // TODO adjust threshold!
}


inline double MaxColorValue(const cv::Scalar &v)
{
  return (v.val[0] > v.val[1]) ? ((v.val[0] > v.val[2]) ? v.val[0] : v.val[2]) : ((v.val[1] > v.val[2]) ? v.val[1] : v.val[2]);
}


inline double MinColorValue(const cv::Scalar &v)
{
  return (v.val[0] < v.val[1]) ? ((v.val[0] < v.val[2]) ? v.val[0] : v.val[2]) : ((v.val[1] < v.val[2]) ? v.val[1] : v.val[2]);
}


/** @brief Converts a RGB vector to HSV. */
cv::Scalar RGB2HSV(const cv::Scalar &rgb)
{
  // Color conversion based on https://stackoverflow.com/a/1664186/400948
  double hue, saturation, value;
  double max = MaxColorValue(rgb);
  double diff = max - MinColorValue(rgb);

  saturation = (max > 0.0) ? (100.0 * diff / max) : 0.0;
  const double r = rgb.val[0];
  const double g = rgb.val[1];
  const double b = rgb.val[2];
  if (saturation > 0.0)
  {
    if (r == max)
      hue = 60.0 * (g-b) / diff;
    else if (g == max)
      hue = 120.0 + 60.0 * (b-r) / diff;
    else
      hue = 240.0 + 60.0 * (r-g) / diff;
  }
  else
    hue = 0.0;

  if (hue < 0.0)
    hue += 360.0;

  value = std::round(max * 100.0/255.0);
  hue = std::round(hue);
  saturation = std::round(saturation);

  return cv::Scalar(hue, saturation, value);
}


/** @brief Converts a HSV vector to RGB). */
cv::Scalar HSV2RGB(const cv::Scalar &hsv)
{
  double hue = hsv.val[0];
  double saturation = hsv.val[1];
  double value = hsv.val[2];
  double r,g,b;

  if (saturation > 0.0)
  {
    hue /= 60.0;
    saturation /= 100.0;
    value /= 100.0;

    const int i = static_cast<int>(std::floor(hue));
    const double f = hue - i;
    const double p = value * (1.0 - saturation);
    const double q = value * (1.0 - saturation*f);
    const double t = value * (1.0 - saturation*(1.0-f));
    switch(i)
    {
    case 0:
      r = value;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = value;
      b = p;
      break;
    case 2:
      r = p;
      g = value;
      b = t;
      break;
    case 3:
      r = p;
      g = q;
      b = value;
      break;
    case 4:
      r = t;
      g = p;
      b = value;
      break;
    default:
      r = value;
      g = p;
      b = q;
    }
    r = std::round(r * 255.0);
    g = std::round(g * 255.0);
    b = std::round(b * 255.0);
  }
  else
  {
    r = std::round(value*2.55);
    g = std::round(value*2.55);
    b = std::round(value*2.55);
  }

  return cv::Scalar(r, g, b, 255);
}


double ShiftHue(double hue, double shift)
{
  hue += shift;
  while (hue >= 360.0)
    hue -= 360.0;
  while (hue < 0.0)
    hue += 360.0;
  return hue;
}
} // namespace utils


const cv::Scalar kExemplaryColors[] = {
  cv::Scalar(255,    0,    0), // Red
  cv::Scalar(  0,  200,    0), // light green(ish)
  cv::Scalar(  0,    0,  255), // deep blue
  cv::Scalar(230,  230,    0), // yellow(ish)
  cv::Scalar(230,    0,  230), // magenta(ish)
  cv::Scalar(  0,  230,  230), // cyan(ish)
  cv::Scalar(255,  128,    0), // orange
  cv::Scalar(255,  128,  128), // skin(ish)
  cv::Scalar(128,   64,    0), // brown(ish)
  cv::Scalar(160,  160,  160), // gray(ish)
  cv::Scalar(  0,  128,  255), // light blue
  cv::Scalar(153,   77,  204) // "flieder"
};
const size_t kExemplaryColorsSize = sizeof(kExemplaryColors) / sizeof(kExemplaryColors[0]);


cv::Scalar GetExemplaryColor(size_t idx)
{
  return kExemplaryColors[idx % kExemplaryColorsSize];
}


const cv::Scalar kAxisColorsBGR[3] = { cv::Scalar(0, 80, 255), cv::Scalar(0, 200, 0), cv::Scalar(255, 0, 0) };
const cv::Scalar kAxisColorsRGB[3] = { cv::Scalar(255, 80, 0), cv::Scalar(0, 200, 0), cv::Scalar(0, 0, 255) };


void DrawRoundedRect(cv::Mat &image, const cv::Rect &rect, const cv::Scalar &color, double corner_percentage, double fill_opacity, int line_width)
{
  VCP_LOG_DEBUG("DrawRoundedRect()");
  // Adapted from https://stackoverflow.com/a/18975399/400948
  if (corner_percentage < 0.0 || corner_percentage > 0.5)
  {
    VCP_ERROR("DrawRoundedRect(): Corner percentage for rounded rectangle must be within [0, 0.5], not " << corner_percentage << "!");
  }

  const double radius_frac = std::round(corner_percentage * static_cast<double>(std::min(rect.width, rect.height)));
  const int corner_radius = radius_frac > 2 ? static_cast<int>(radius_frac) : 0;

  if (corner_radius > 0)
  {
    const cv::Point tl(rect.x, rect.y);
    const cv::Point tr(rect.x + rect.width - 1, tl.y);
    const cv::Point br(tr.x, rect.y + rect.height - 1);
    const cv::Point bl(tl.x, br.y);

    const cv::Point xoffset(corner_radius, 0);
    const cv::Point yoffset(0, corner_radius);
    const cv::Size arc_size(corner_radius, corner_radius);
    // Ellipse centers:
    // tl---------tr
    // |  e1   e2  |
    //      ...
    // |  e4   e3  |
    // bl---------br
    const cv::Point e1 = tl+xoffset+yoffset;
    const cv::Point e2 = tr-xoffset+yoffset;
    const cv::Point e3 = br-xoffset-yoffset;
    const cv::Point e4 = bl+xoffset-yoffset;


    if (fill_opacity > 0.0)
    {
      cv::Rect roi_rect(rect);
      vcp::imutils::ClipRectangleToImageBoundaries(roi_rect, image.size());
      if (roi_rect.width > 0 && roi_rect.height > 0)
      {
        cv::Mat roi = image(roi_rect);
        cv::Mat filled;
        roi.copyTo(filled);
        const cv::Point roi_tl(rect.x != roi_rect.x ? rect.x : 0, rect.y != roi_rect.y? rect.y : 0);
        const cv::Point roi_tr(roi_tl.x + rect.width - 1, roi_tl.y);
        const cv::Point roi_bl(roi_tl.x, roi_tl.y + rect.height - 1);
        const cv::Point roi_br(roi_tr.x, roi_bl.y);
        //const cv::Point roi_br(filled.cols-1, filled.rows-1);
        //const cv::Point roi_bl(0, filled.rows-1);
        //const cv::Point roi_tr(filled.cols-1, 0);

        // Fill inner rect
        cv::rectangle(filled, roi_tl+yoffset, roi_br-yoffset, color, -1);
        cv::rectangle(filled, roi_tl+xoffset, roi_tr-xoffset+yoffset, color, -1);
        cv::rectangle(filled, roi_bl+xoffset-yoffset, roi_br-xoffset, color, -1);

        // Fill arcs
        cv::ellipse(filled, roi_tl+xoffset+yoffset, arc_size, 180.0, 0.0, 90.0, color, -1);
        cv::ellipse(filled, roi_tr-xoffset+yoffset, arc_size, 270.0, 0.0, 90.0, color, -1);
        cv::ellipse(filled, roi_br-xoffset-yoffset, arc_size,   0.0, 0.0, 90.0, color, -1);
        cv::ellipse(filled, roi_bl+xoffset-yoffset, arc_size,  90.0, 0.0, 90.0, color, -1);

        cv::addWeighted(filled, fill_opacity, roi, 1.0-fill_opacity, 0.0, roi);
      }
    }

    if (line_width > 0)
    {
      // Draw lines
      cv::line(image, tl+xoffset, tr-xoffset, color, line_width);
      cv::line(image, tr+yoffset, br-yoffset, color, line_width);
      cv::line(image, br-xoffset, bl+xoffset, color, line_width);
      cv::line(image, bl-yoffset, tl+yoffset, color, line_width);

      // Draw arcs
      cv::ellipse(image, e1, arc_size, 180.0, 0.0, 90.0, color, line_width);
      cv::ellipse(image, e2, arc_size, 270.0, 0.0, 90.0, color, line_width);
      cv::ellipse(image, e3, arc_size,   0.0, 0.0, 90.0, color, line_width);
      cv::ellipse(image, e4, arc_size,  90.0, 0.0, 90.0, color, line_width);
    }
  }
  else
    DrawRect(image, rect, color, fill_opacity, line_width);
}


void DrawRoundedRects(cv::Mat &image, const std::vector<cv::Rect> &rects, const cv::Scalar &color, double corner_percentage, double fill_opacity, int line_width, bool non_overlapping)
{
  VCP_LOG_DEBUG("DrawRoundedRects()");
  if (non_overlapping)
  {
    cv::Mat mask = cv::Mat::ones(image.size(), CV_8UC1);
    for (const auto &rect : rects)
    {
      cv::Mat clone = image.clone();

      DrawRoundedRect(clone, rect, color, corner_percentage, fill_opacity, line_width);

      // Copy only non-overlapping parts back to the original image:
      cv::Rect roi(rect);
      vcp::imutils::ClipRectangleToImageBoundaries(roi, image.size());

      cv::Mat clone_roi = clone(roi);
      cv::Mat image_roi = image(roi);
      cv::Mat mask_roi = mask(roi);
      clone_roi.copyTo(image_roi, mask_roi);

      // (Un)set mask
      DrawRoundedRect(mask, rect, cv::Scalar::all(0.0), corner_percentage, 1.0, line_width);
    }
  }
  else
  {
    for (const auto &rect : rects)
      DrawRoundedRect(image, rect, color, corner_percentage, fill_opacity, line_width);
  }
}


void DrawRect(cv::Mat &image, const cv::Rect &rect, const cv::Scalar &color, double fill_opacity, int line_width, int dash_length)
{
  VCP_LOG_DEBUG("DrawRect()");
  // Fill, if requested:
  if (fill_opacity > 0.0)
  {
    cv::Rect rect_roi(rect);
    vcp::imutils::ClipRectangleToImageBoundaries(rect_roi, image.size());
    if (rect_roi.width > 0 && rect_roi.height > 0)
    {
      cv::Mat filled = cv::Mat(rect_roi.size(), image.type(), color);
      cv::Mat roi = image(rect_roi);
      cv::addWeighted(filled, fill_opacity, roi, 1.0-fill_opacity, 0.0, roi);
    }
  }

  // Draw edges, if requested:
  if (line_width > 0)
  {
    if (dash_length > 0)
      DrawDashedRect(image, rect, color, dash_length, line_width);
    else
      cv::rectangle(image, rect, color, line_width);
  }
}


void DrawRects(cv::Mat &image, const std::vector<cv::Rect> &rects, const cv::Scalar &color, double fill_opacity, int line_width, int dash_length, bool non_overlapping)
{
  VCP_LOG_DEBUG("DrawRects()");
  if (non_overlapping)
  {
    cv::Mat mask = cv::Mat::ones(image.size(), CV_8UC1);
    for (const auto &rect : rects)
    {
      cv::Mat clone = image.clone();

      DrawRect(image, rect, color, fill_opacity, line_width, dash_length);

      // Copy only non-overlapping parts back to the original image:
      cv::Rect roi(rect);
      vcp::imutils::ClipRectangleToImageBoundaries(roi, image.size());

      cv::Mat clone_roi = clone(roi);
      cv::Mat image_roi = image(roi);
      cv::Mat mask_roi = mask(roi);
      clone_roi.copyTo(image_roi, mask_roi);

      // (Un)set mask
      DrawRect(mask, rect, cv::Scalar::all(0.0), 1.0, line_width, dash_length);
    }
  }
  else
  {
    for (const auto &rect : rects)
      DrawRect(image, rect, color, fill_opacity, line_width, dash_length);
  }
}


void DrawBoundingBoxes2d(cv::Mat &image, const std::vector<cv::Rect> &rects, const std::vector<cv::Scalar> &rect_colors, const std::vector<std::string> &captions,
                         bool non_overlapping, int line_width, double fill_opacity, const std::vector<int> &dash_lengths, int text_anchor,
                         const std::vector<cv::Scalar> &font_colors, const std::vector<cv::Scalar> &text_box_colors, int text_box_padding,
                         double text_box_opacity, int font_face, double font_scale, int font_thickness)
{
  VCP_LOG_DEBUG("DrawBoundingBoxes2d()");
  const bool dashes_provided = !dash_lengths.empty();
  const bool text_bg_provided = !text_box_colors.empty();
  const bool text_fg_provided = !font_colors.empty();

  if (non_overlapping)
  {
    cv::Mat mask = cv::Mat::ones(image.size(), CV_8UC1);
    for (size_t i = 0; i < rects.size(); ++i)
    {
      cv::Rect roi = rects[i];
      const int dash = dashes_provided ? dash_lengths[i] : -1;
      const cv::Point text_pos = GetTextPos(roi, text_anchor);
      const cv::Scalar text_bg_color = text_bg_provided ? text_box_colors[i] : rect_colors[i];
      const cv::Scalar text_fg_color = text_fg_provided ? font_colors[i] : ComplementaryColor(text_bg_color, true);

      // Draw onto clone
      cv::Mat clone = image.clone();
      DrawRect(clone, rects[i], rect_colors[i], fill_opacity, line_width, dash);

      if (!captions[i].empty())
      {
        cv::Rect text_roi;
        DrawTextBox(clone, captions[i], text_pos, text_anchor,
                    text_box_padding, text_box_opacity,
                    text_fg_color, text_bg_color,
                    font_face, font_scale, font_thickness, &text_roi);
        roi = roi | text_roi; // Union
      }

      // Copy valid area onto image
      vcp::imutils::ClipRectangleToImageBoundaries(roi, image.size());
      if (roi.width > 0 && roi.height > 0)
      {
        cv::Mat clone_roi = clone(roi);
        cv::Mat image_roi = image(roi);
        cv::Mat mask_roi = mask(roi);
        clone_roi.copyTo(image_roi, mask_roi);

        // Unset mask
        DrawRect(mask, rects[i], cv::Scalar::all(0.0), 1.0, line_width, dash);
        if (!captions[i].empty())
        {
          DrawTextBox(mask, captions[i], text_pos, text_anchor,
                      text_box_padding,  1.0, cv::Scalar::all(0.0), cv::Scalar::all(0.0),
                      font_face, font_scale, font_thickness);
        }
      }
    }
  }
  else
  {
    for (size_t i = 0; i < rects.size(); ++i)
    {
      const int dash = dashes_provided ? dash_lengths[i] : -1;
      const cv::Point text_pos = GetTextPos(rects[i], text_anchor);
      const cv::Scalar text_bg_color = text_bg_provided ? text_box_colors[i] : rect_colors[i];
      const cv::Scalar text_fg_color = text_fg_provided ? font_colors[i] : ComplementaryColor(text_bg_color, true);

      DrawRect(image, rects[i], rect_colors[i], fill_opacity, line_width, dash);

      if (!captions[i].empty())
      {
        DrawTextBox(image, captions[i], text_pos, text_anchor,
                    text_box_padding, text_box_opacity,
                    text_fg_color, text_bg_color,
                    font_face, font_scale, font_thickness);
      }
    }
  }
}



void DrawRotatedBoundingBoxes2d(cv::Mat &image, const std::vector<cv::RotatedRect> &rects, const std::vector<cv::Scalar> &rect_colors, const std::vector<std::string> &captions,
                         bool non_overlapping, int line_width, double fill_opacity, const std::vector<int> &dash_lengths, int text_anchor,
                         const std::vector<cv::Scalar> &font_colors, const std::vector<cv::Scalar> &text_box_colors, int text_box_padding,
                         double text_box_opacity, int font_face, double font_scale, int font_thickness)
{
  VCP_LOG_DEBUG("DrawRotatedBoundingBoxes2d()");
  const bool dashes_provided = !dash_lengths.empty();
  const bool text_bg_provided = !text_box_colors.empty();
  const bool text_fg_provided = !font_colors.empty();

  if (non_overlapping)
  {
    cv::Mat mask = cv::Mat::ones(image.size(), CV_8UC1);
    for (size_t i = 0; i < rects.size(); ++i)
    {
      cv::Rect roi = rects[i].boundingRect();
      const int dash = dashes_provided ? dash_lengths[i] : -1;
      const cv::Point text_pos = GetTextPos(rects[i], text_anchor);
      const cv::Scalar text_bg_color = text_bg_provided ? text_box_colors[i] : rect_colors[i];
      const cv::Scalar text_fg_color = text_fg_provided ? font_colors[i] : ComplementaryColor(text_bg_color, true);

      // Draw onto clone
      cv::Mat clone = image.clone();
      DrawRotatedRect(clone, rects[i], rect_colors[i], fill_opacity, line_width, dash);

      if (!captions[i].empty())
      {
        cv::Rect text_roi;
        DrawTextBox(clone, captions[i], text_pos, text_anchor,
                    text_box_padding, text_box_opacity,
                    text_fg_color, text_bg_color,
                    font_face, font_scale, font_thickness, &text_roi);
        roi = roi | text_roi; // Union
      }

      // Copy valid area onto image
      vcp::imutils::ClipRectangleToImageBoundaries(roi, image.size());
      cv::Mat clone_roi = clone(roi);
      cv::Mat image_roi = image(roi);
      cv::Mat mask_roi = mask(roi);
      clone_roi.copyTo(image_roi, mask_roi);

      // Unset mask
      DrawRotatedRect(mask, rects[i], cv::Scalar::all(0.0), 1.0, line_width, dash);
      if (!captions[i].empty())
      {
        DrawTextBox(mask, captions[i], text_pos, text_anchor,
                    text_box_padding,  1.0, cv::Scalar::all(0.0), cv::Scalar::all(0.0),
                    font_face, font_scale, font_thickness);
      }
    }
  }
  else
  {
    for (size_t i = 0; i < rects.size(); ++i)
    {
      const int dash = dashes_provided ? dash_lengths[i] : -1;
      const cv::Point text_pos = GetTextPos(rects[i], text_anchor);

      DrawRotatedRect(image, rects[i], rect_colors[i], fill_opacity, line_width, dash);

      if (!captions[i].empty())
      {
        const cv::Scalar text_bg_color = text_bg_provided ? text_box_colors[i] : rect_colors[i];
        const cv::Scalar text_fg_color = text_fg_provided ? font_colors[i] : ComplementaryColor(text_bg_color, true);
        DrawTextBox(image, captions[i], text_pos, text_anchor,
                    text_box_padding, text_box_opacity,
                    text_fg_color, text_bg_color,
                    font_face, font_scale, font_thickness);
      }
    }
  }
}




void DrawRotatedRects(cv::Mat &image, const std::vector<cv::RotatedRect> &rects, const cv::Scalar &color, double fill_opacity, int line_width, int dash_length, bool non_overlapping)
{
  VCP_LOG_DEBUG("DrawRotatedRects()");
  if (non_overlapping)
  {
    cv::Mat mask = cv::Mat::ones(image.size(), CV_8UC1);
    for (const auto &rect : rects)
    {
      cv::Mat clone = image.clone();

      DrawRotatedRect(clone, rect, color, fill_opacity, line_width, dash_length);

      // Copy only non-overlapping parts back to the original image:
      cv::Rect roi = rect.boundingRect();
      vcp::imutils::ClipRectangleToImageBoundaries(roi, image.size());

      cv::Mat clone_roi = clone(roi);
      cv::Mat image_roi = image(roi);
      cv::Mat mask_roi = mask(roi);
      clone_roi.copyTo(image_roi, mask_roi);

      // (Un)set mask
      DrawRotatedRect(mask, rect, cv::Scalar::all(0.0), 1.0, line_width, dash_length);
    }
  }
  else
  {
    for (const auto &rect : rects)
      DrawRotatedRect(image, rect, color, fill_opacity, line_width, dash_length);
  }
}


void DrawDashedRect(cv::Mat &image, const cv::Rect &rect, const cv::Scalar &color, int dash_length, int line_width)
{
  VCP_LOG_DEBUG("DrawDashedRect()");
  const cv::Point tl = rect.tl();
  const cv::Point br(rect.x + rect.width - 1, rect.y + rect.height - 1); // OpenCV calculates rect.br() as x+w, y+h (which would be (W+1) x (H+1)!;
  const cv::Point tr(br.x, tl.y);
  const cv::Point bl(tl.x, br.y);

  DrawDashedLine(tl, tr, color, dash_length, line_width, image);
  DrawDashedLine(tr, br, color, dash_length, line_width, image);
  DrawDashedLine(br, bl, color, dash_length, line_width, image);
  DrawDashedLine(bl, tl, color, dash_length, line_width, image);
}


void DrawRotatedRect(cv::Mat &image, const cv::RotatedRect &rect, const cv::Scalar &color, double fill_opacity, int line_width, int dash_length)
{
  VCP_LOG_DEBUG("DrawRotatedRect()");
  // Fill, if requested:
  if (fill_opacity > 0.0)
  {
    // Extract image ROI (axis-aligned rectangular region, respecting boundaries)
    const cv::Rect bounding_rect = rect.boundingRect();
    cv::Rect bounding_rect_roi(bounding_rect);
    vcp::imutils::ClipRectangleToImageBoundaries(bounding_rect_roi, image.size());
    if (bounding_rect_roi.width > 0 && bounding_rect_roi.height > 0)
    {
      cv::Mat roi = image(bounding_rect_roi);
      cv::Mat filled;
      roi.copyTo(filled);

      // Shift rotated rect to coordinate frame of ROI
      const float cx = bounding_rect.width/2.0f + (bounding_rect.x < 0 ? bounding_rect.x : 0.0f);
      const float cy = bounding_rect.height/2.0f + (bounding_rect.y < 0 ? bounding_rect.y : 0.0f);
      const cv::Size2f size(rect.size.width-1.0f, rect.size.height-1.0f); // Required, otherwise the bounding rect of the following shifted_rect will be (w+1)x(h+1) :-/
      const cv::RotatedRect shifted_rect(cv::Point2f(cx, cy), size, rect.angle);

      // Convert rotated rect to polygon
      cv::Point2f vertices2f[4];
      cv::Point vertices[4];
      shifted_rect.points(vertices2f);
      for (size_t i = 0; i < 4; ++i)
        vertices[i] = vertices2f[i];
      cv::fillConvexPoly(filled, vertices, 4, color);

      cv::addWeighted(filled, fill_opacity, roi, 1.0-fill_opacity, 0.0, roi);

      // Draw edges using the same points as used for filling:
      if (line_width > 0)
      {
        if (dash_length > 0)
        {
          for (size_t i = 0; i < 4; ++i)
            DrawDashedLine(vertices[i], vertices[(i+1)%4], color, dash_length, line_width, roi);
        }
        else
        {
          for (size_t i = 0; i < 4; ++i)
            cv::line(roi, vertices[i], vertices[(i+1)%4], color, line_width);
        }
      }
    }
  }
  else
  {
    // Only draw outlines:
    // Draw edges, if requested:
    if (line_width > 0)
    {
      if (dash_length > 0)
      {
        DrawDashedRotatedRect(image, rect, color, dash_length, line_width);
      }
      else
      {
        cv::Point2f vertices[4];
        rect.points(vertices);

        for (size_t i = 0; i < 4; ++i)
          cv::line(image, vertices[i], vertices[(i+1)%4], color, line_width);
      }
    }
  }
}


void DrawDashedRotatedRect(cv::Mat &image, const cv::RotatedRect &rect, const cv::Scalar &color, int dash_length, int line_width)
{
  VCP_LOG_DEBUG("DrawDashedRotatedRect()");
  cv::Point2f vertices[4];
  rect.points(vertices);

  for (size_t i = 0; i < 4; ++i)
    DrawDashedLine(vertices[i], vertices[(i+1)%4], color, dash_length, line_width, image);
}


void DrawDashedLine(const cv::Point &pt1, const cv::Point &pt2, const cv::Scalar &color, int dash_length, int thickness, cv::Mat &image)
{
  VCP_LOG_DEBUG("DrawDashedLine()");
  if (pt1 == pt2)
    return;

  /**
   * We have multiple options:
   * 1 Iterate over neighboring lines using Bresenham - doesn't look so nice, as
   *   we didn't implement anti aliasing or rounded/fattened corners.
   * 2 Draw each dash using cv::lines - is slightly slower. It's 0.004-0.010 ms
   *   with Bresenham vs. 0.005-0.019 with cv::lines (uses fillPoly internally).
   * 3 Use cv::polylines - this caused too much overhead when extracting and
   *   preparing the line segments for the polylines invocation.
   * We currently use 2) as it yields aesthetic dashes at a negligible
   * computational overhead (typically takes ~twice as long as our ugly
   * Bresenham approach).
   */
   VCP_CHECK(image.type() == CV_8UC3);
   if (thickness < 1)
     thickness = 1;

  // Unit direction vector:
  const cv::Vec2d vfrom(pt1.x, pt1.y);
  const cv::Vec2d diff = cv::Vec2d(pt2.x, pt2.y) - vfrom;
  const double distance = std::sqrt(diff[0]*diff[0] + diff[1]*diff[1]);
  const cv::Vec2d direction(diff[0]/distance, diff[1]/distance);

  // Ensure an odd number of steps (so we can start and end with a dash)
  const int num_segments = utils::MakeOdd<int>(static_cast<int>(std::ceil(distance / dash_length)));
  const int num_dashes = (num_segments+1)/2;

  const double ddash_length = distance / static_cast<double>(num_segments);
  const cv::Vec2d segment_offset = 2.0 * ddash_length * direction;

  // OpenCV will fatten the start/end points, so our line to draw should be a
  // little shorter
  const cv::Vec2d thickness_half = thickness/2.0 * direction;
  const cv::Vec2d dash = ddash_length * direction - 2.0*thickness_half;

  for (int i = 0; i < num_dashes; ++i)
  {
    // Computing the start/end point in each iteration is faster, e.g. 0.004 ms
    // vs 0.006 ms on average.
    const cv::Vec2d vf = vfrom + i*segment_offset + thickness_half;
    const cv::Vec2d vt = vf + dash;
    cv::line(image, vcp::convert::ToPoint(vf), vcp::convert::ToPoint(vt), color, thickness);
  }
}


void DrawLines(cv::Mat &image, const std::vector<cv::Point> &start, const std::vector<cv::Point> &end, const cv::Scalar &color, int line_width, const std::vector<int> &dash_lengths)
{
  VCP_LOG_DEBUG("DrawLines() with single color.");
  VCP_CHECK(start.size() == end.size());
  VCP_CHECK(dash_lengths.empty() || dash_lengths.size() == start.size());

  const bool maybe_dashed = !dash_lengths.empty();
  for (size_t i = 0; i < start.size(); ++i)
  {
    if (maybe_dashed && dash_lengths[i] > 0)
      DrawDashedLine(start[i], end[i], color, dash_lengths[i], line_width, image);
    else
      cv::line(image, start[i], end[i], color, line_width);
  }
}


void DrawLines(cv::Mat &image, const std::vector<cv::Point> &start, const std::vector<cv::Point> &end, const std::vector<cv::Scalar> &colors, int line_width, const std::vector<int> &dash_lengths)
{
  VCP_LOG_DEBUG("DrawLines() with different colors.");
  VCP_CHECK(start.size() == end.size());
  VCP_CHECK(colors.size() == start.size());
  VCP_CHECK(dash_lengths.empty() || dash_lengths.size() == start.size());

  const bool maybe_dashed = !dash_lengths.empty();
  for (size_t i = 0; i < start.size(); ++i)
  {
    if (maybe_dashed && dash_lengths[i] > 0)
      DrawDashedLine(start[i], end[i], colors[i], dash_lengths[i], line_width, image);
    else
      cv::line(image, start[i], end[i], colors[i], line_width);
  }
}

void DrawInfiniteLine(cv::Mat &image, const cv::Point &pt1, const cv::Point &pt2, const cv::Scalar &color, int line_width, int dash_length)
{
  VCP_LOG_DEBUG("DrawInfiniteLine()");
  const vcp::math::geo2d::Line2d line = vcp::math::geo2d::ClipLineByRectangle(vcp::math::geo2d::Line2d(pt1, pt2), cv::Rect(0, 0, image.cols, image.rows));

  if (!line.empty())
  {
    const cv::Point from = vcp::convert::ToPoint(line.From());
    const cv::Point to = vcp::convert::ToPoint(line.To());
    if (dash_length > 0)
      DrawDashedLine(from, to, color, dash_length, line_width, image);
    else
      cv::line(image, from, to, color, line_width);
  }
}


void DrawPolygon2d(cv::Mat &image, const std::vector<cv::Point> &polygon, const cv::Scalar &color, int line_width, int dash_length, double fill_opacity)
{
  VCP_LOG_DEBUG("DrawPolygon2d()");
  if (polygon.size() < 3)
    return;

  // How to pass a vector of points to OpenCV's poly stuff:
  // https://stackoverflow.com/questions/8281239/drawing-polygons-in-opencv
  std::vector<std::vector<cv::Point>> polys;
  polys.push_back(polygon);

  if (fill_opacity > 0.0)
  {
    // Subsampling the region of interest yields messy code (instead of the 3 lines below), but a significant speed-up.
    // Drawing a single box takes 0.1-1.0 ms instead of 3-5ms.
//    cv::Mat filled = image.clone();
//    cv::fillConvexPoly(filled, poly, color);
//    cv::addWeighted(filled, fill_opacity, image, (1.0 - fill_opacity), 0.0, image);

    // Get bounding ROI
    int x_min = polygon[0].x;
    int x_max = x_min;
    int y_min = polygon[0].y;
    int y_max = y_min;
    for (size_t i = 1; i < polygon.size(); ++i)
    {
      if (polygon[i].x < x_min)
        x_min = polygon[i].x;
      if (polygon[i].x > x_max)
        x_max = polygon[i].x;
      if (polygon[i].y < y_min)
        y_min = polygon[i].y;
      if (polygon[i].y > y_max)
        y_max = polygon[i].y;
    }

    // Clip to visible image
    cv::Rect roi_rect(x_min, y_min, x_max - x_min + 1, y_max - y_min + 1);
    vcp::imutils::ClipRectangleToImageBoundaries(roi_rect, image.size());
    if (roi_rect.width > 0 && roi_rect.height > 0)
    {
      cv::Mat roi = image(roi_rect);
      cv::Mat filled;
      roi.copyTo(filled);

      // Shift polygon to coordinate frame of ROI
      const cv::Point offset(-roi_rect.x, -roi_rect.y);

      if (offset.x != 0 || offset.y != 0)
      {
        std::vector<cv::Point> poly_offset;
        poly_offset.reserve(polygon.size());

        for (const cv::Point &pt : polygon)
          poly_offset.push_back(pt + offset);

        std::vector<std::vector<cv::Point>> polys_offset;
        polys_offset.push_back(poly_offset);
        cv::fillPoly(filled, polys_offset, color);

//        cv::fillConvexPoly(filled, poly_offset, color); -- also gives a speed up (but only a few tenths of a ms)
        cv::addWeighted(filled, fill_opacity, roi, (1.0 - fill_opacity), 0.0, roi);
      }
      else
      {
        cv::fillPoly(filled, polys, color);
//        cv::fillConvexPoly(filled, poly, color);
        cv::addWeighted(filled, fill_opacity, roi, (1.0 - fill_opacity), 0.0, roi);
      }
    }
  }

  if (line_width > 0)
  {
    if (dash_length > 0)
    {
      std::vector<cv::Point> poly_starts, poly_ends;
      poly_starts.reserve(polygon.size()-1);
      poly_ends.reserve(polygon.size()-1);

      std::vector<int> dash_lengths;
      dash_lengths.reserve(polygon.size()-1);

      for (size_t i = 1; i < polygon.size(); ++i)
      {
        poly_starts.push_back(polygon[i-1]);
        poly_ends.push_back(polygon[i]);
        dash_lengths.push_back(dash_length);
      }
      DrawLines(image, poly_starts, poly_ends, color, line_width, dash_lengths);
    }
    else
    {
      cv::polylines(image, polys, polygon[0] == polygon[polygon.size()-1], color, line_width);
    }
  }
}


void DrawLineSegment3d(cv::Mat &image, const cv::Vec3d &pt1, const cv::Vec3d &pt2, const cv::Mat &P, const cv::Scalar &color, int line_width, int dash_length, double scale_image_points, cv::Point *prj1, cv::Point *prj2)
{
  VCP_LOG_DEBUG("DrawLineSegment3d()");
  const cv::Point from = utils::ProjectPointWithScale(P, pt1, scale_image_points);
  const cv::Point to = utils::ProjectPointWithScale(P, pt2, scale_image_points);

  if (prj1)
    *prj1 = from;
  if (prj2)
    *prj2 = to;

  if (from == to)
    return;

  const vcp::math::geo2d::Line2d clipped = vcp::math::geo2d::ClipLineSegmentByRectangle(
        vcp::math::geo2d::Line2d(from, to), vcp::imutils::ImageRect(image));

  if (clipped.empty())
    return;

  if (dash_length > 0)
    DrawDashedLine(from, to, color, dash_length, line_width, image);
  else
    cv::line(image, from, to, color, line_width);
}


void DrawPolygon3d(cv::Mat &image, const std::vector<cv::Vec3d> &pts, const cv::Mat &P, const cv::Scalar &color, double fill_opacity, int line_width, int dash_length, double scale_image_points)
{
  VCP_LOG_DEBUG("DrawPolygon3d()");
  if (pts.size() < 2)
    return;

  std::vector<cv::Point> poly;
  for (const auto &pt : pts)
  {
    const cv::Point prj = utils::ProjectPointWithScale(P, pt, scale_image_points);
    if (poly.empty() || (prj.x != poly[poly.size()-1].x || prj.y != poly[poly.size()-1].y))
      poly.push_back(prj);
  }

  DrawPolygon2d(image, poly, color, line_width, dash_length, fill_opacity);
}


cv::Point GetTextPos(const cv::Rect &rect, int text_anchor)
{
  cv::Point pos;
  // Horizontally:
  if (text_anchor & textanchor::LEFT)
    pos = cv::Point(rect.x, 0);
  else if (text_anchor & textanchor::HCENTER)
    pos = cv::Point(rect.x + rect.width/2, 0);
  else
    pos = cv::Point(rect.x + rect.width - 1, 0);

  // Vertically:
  if (text_anchor & textanchor::TOP)
    pos.y = rect.y;
  else if (text_anchor & textanchor::VCENTER)
    pos.y = rect.y + rect.height/2;
  else
    pos.y = rect.y + rect.height - 1;
  return pos;
}


cv::Point GetTextPos(const cv::RotatedRect &rect, int text_anchor)
{
  cv::Vec2d tpvec;
  // Horizontally:
  if (text_anchor & vcp::imvis::drawing::textanchor::LEFT)
    tpvec = cv::Vec2d(rect.center.x - rect.size.width/2.0f, 0);
  else if (text_anchor & vcp::imvis::drawing::textanchor::HCENTER)
    tpvec = cv::Vec2d(rect.center.x, 0);
  else
    tpvec = cv::Vec2d(rect.center.x + rect.size.width/2.0f, 0);

  // Vertically:
  if (text_anchor & vcp::imvis::drawing::textanchor::TOP)
    tpvec.val[1] = rect.center.y - rect.size.height/2.0f;
  else if (text_anchor & vcp::imvis::drawing::textanchor::VCENTER)
    tpvec.val[1] = rect.center.y;
  else
    tpvec.val[1] = rect.center.y + rect.size.height/2.0f;

  // OpenCV's RotatedRect angle is negative (seems to assume a right-handed coordinate system).
  cv::Vec2d tpvec_rot = vcp::math::geo2d::RotateVector(tpvec, vcp::convert::ToVec2d(rect.center), vcp::math::Deg2Rad(rect.angle));

  return vcp::convert::ToPoint(tpvec_rot);
}


std::string TextAnchorToString(int text_anchor)
{
  if (text_anchor & textanchor::LEFT)
  {
    if (text_anchor & textanchor::TOP)
      return "northwest";
    if (text_anchor & textanchor::VCENTER)
      return "west";
    else
      return "southwest";
  }
  if (text_anchor & textanchor::RIGHT)
  {
    if (text_anchor & textanchor::TOP)
      return "northeast";
    if (text_anchor & textanchor::VCENTER)
      return "east";
    else
      return "southeast";
  }

  if (text_anchor & textanchor::TOP)
    return "north";
  if (text_anchor & textanchor::VCENTER)
    return "center";
  else
    return "south";
}


void DrawTextBox(cv::Mat &image, const std::string &text, const cv::Point &pos, int anchor, int padding, double fill_opacity, const cv::Scalar &font_color, const cv::Scalar &bg_color, int font_face, double font_scale, int thickness, cv::Rect *image_roi)
{
  VCP_LOG_DEBUG("DrawTextBox()");
  const cv::Size txt_size = cv::getTextSize(text, font_face, font_scale, thickness, nullptr);
  const cv::Size padded_txt_size(txt_size.width + 2*padding, txt_size.height + 2*padding);

  // Check anchor points
  cv::Point text_pos_bl;
  cv::Rect bg_box(0, 0, padded_txt_size.width, padded_txt_size.height);

  // Horizontal
  if (anchor & textanchor::LEFT)
  {
    bg_box.x = pos.x;
  }
  else if (anchor & textanchor::HCENTER)
  {
    bg_box.x = pos.x - padded_txt_size.width/2;
  }
  else if (anchor & textanchor::RIGHT)
  {
    bg_box.x = pos.x - padded_txt_size.width;
  }
  else
  {
    VCP_ERROR("DrawTextBox(): The text anchor doesn't specify the horizontal position!");
  }
  text_pos_bl.x = bg_box.x + padding;


  // Vertical
  if (anchor & textanchor::TOP)
  {
    bg_box.y = pos.y;
  }
  else if (anchor & textanchor::VCENTER)
  {
    bg_box.y = pos.y - padded_txt_size.height/2;
  }
  else if (anchor & textanchor::BOTTOM)
  {
    bg_box.y = pos.y - padded_txt_size.height;
  }
  else
  {
    VCP_ERROR("DrawTextBox(): The text anchor doesn't specify the horizontal position!");
  }
  text_pos_bl.y = bg_box.y + padding + txt_size.height;

  // Clip box with image boundaries:
  vcp::imutils::ClipRectangleToImageBoundaries(bg_box, image.size());

  // Fill the background
  if (bg_color[0] < 0 || bg_color[1] < 0 || bg_color[2] < 0)
    DrawRect(image, bg_box, ComplementaryColor(font_color, false), fill_opacity, -1);
  else
    DrawRect(image, bg_box, bg_color, fill_opacity, -1);

  // Draw the text
  cv::putText(image, text, text_pos_bl, font_face, font_scale, font_color, thickness);

  // Set image ROI if necessary
  if (image_roi)
    *image_roi = bg_box; // This is already clipped.
}


void DrawArrow(const cv::Point &pt1, const cv::Point &pt2, const cv::Scalar &color, cv::Mat &image,
               int thickness, double tip_length, int dash_length)
{
  VCP_LOG_DEBUG("DrawArrow()");
  // Adapted from OpenCV 3.X for two reasons: OpenCV 2.x has no such functionality and we want dashed arrows sometimes.
  // Factor to normalize the size of the tip depending on the length of the arrow
  const double tip_size = cv::norm(pt1 - pt2) * tip_length;

  if (dash_length > 0)
    DrawDashedLine(pt1, pt2, color, dash_length, thickness, image);
  else
    cv::line(image, pt1, pt2, color, thickness);

  const double angle = atan2((double)pt1.y - pt2.y, (double)pt1.x - pt2.x);

  cv::Point p(cvRound(pt2.x + tip_size * cos(angle + CV_PI / 4)),
      cvRound(pt2.y + tip_size * sin(angle + CV_PI / 4)));

  if (dash_length > 0)
    DrawDashedLine(p, pt2, color, dash_length, thickness, image);
  else
    cv::line(image, p, pt2, color, thickness);

  p.x = cvRound(pt2.x + tip_size * cos(angle - CV_PI / 4));
  p.y = cvRound(pt2.y + tip_size * sin(angle - CV_PI / 4));
  if (dash_length > 0)
    DrawDashedLine(p, pt2, color, dash_length, thickness, image);
  else
    cv::line(image, p, pt2, color, thickness);
}


cv::Scalar ComplementaryColor(const cv::Scalar &color, bool is_rgb)
{
  // Opposites of white and black are ambiguous within the color wheel!
  if (utils::IsWhite(color))
    return cv::Scalar(0.0, 0.0, 0.0, 255.0);
  if (utils::IsBlack(color))
    return cv::Scalar(255.0, 255.0, 255.0, 255.0);
  if (utils::IsShadeOfGray(color))
  {
    if (color[0] <= 127)
      return cv::Scalar(255.0, 255.0, 255.0, 255.0);
    return cv::Scalar(0.0, 0.0, 0.0, 255.0);
  }

  cv::Scalar hsv = utils::RGB2HSV(is_rgb ? color : cv::Scalar(color[2], color[1], color[0]));
  hsv[0] = utils::ShiftHue(hsv[0], 180.0);
  const cv::Scalar rgb = utils::HSV2RGB(hsv);

  if (is_rgb)
    return rgb;
  else
    return cv::Scalar(rgb[2], rgb[1], rgb[0], rgb[3]);
}


cv::Scalar InvertColorRGB(const cv::Scalar &color)
{
  return cv::Scalar(255.0-color[0], 255.0-color[1], 255.0-color[2]);
}


void DrawPoints(cv::Mat &image, const std::vector<cv::Point> &points, const std::vector<cv::Scalar> &colors, int radius, int line_width, double alpha)
{
  VCP_LOG_DEBUG("DrawPoints() with different colors.");
  VCP_CHECK(points.size() == colors.size());
  VCP_CHECK(alpha >= 0.0 && alpha <= 1.0);

  cv::Mat draw_on = image.clone();
  for (size_t i = 0; i < points.size(); ++i)
    cv::circle(draw_on, points[i], radius, colors[i], line_width);

  if (alpha < 1.0)
    cv::addWeighted(draw_on, alpha, image, 1.0-alpha, 0.0, image);
  else
    image = draw_on;
}


void DrawPoints(cv::Mat &image, const std::vector<cv::Point> &points, const cv::Scalar &color, int radius, int line_width, double alpha, bool flip_color_channels)
{
  VCP_LOG_DEBUG("DrawPoints() with single color.");
  std::vector<cv::Scalar> colors;
  colors.reserve(points.size());
  const bool valid_color = !((color[0] < 0.0) || (color[1] < 0.0) || (color[2] < 0.0));
  for (size_t i = 0; i < points.size(); ++i)
  {
    const cv::Scalar ptcolor = valid_color ? color : kExemplaryColors[i % kExemplaryColorsSize];
    if (flip_color_channels)
      colors.push_back(cv::Scalar(ptcolor.val[2], ptcolor.val[1], ptcolor.val[0]));
    else
      colors.push_back(ptcolor);
  }

  DrawPoints(image, points, colors, radius, line_width, alpha);
}


void DrawCrosses(cv::Mat &image, const std::vector<cv::Point> &points, const std::vector<cv::Scalar> &colors,
                 int diagonal, int line_width, int dash_length, double alpha, bool vertical)
{
  VCP_LOG_DEBUG("DrawCrosses() with different colors.");
  VCP_CHECK(points.size() == colors.size());
  VCP_CHECK(alpha >= 0.0 && alpha <= 1.0);

  cv::Mat draw_on = image.clone();
  const double offset = vertical ? diagonal / 2.0 : diagonal / (2.0 * std::sqrt(2.0));
  for (size_t i = 0; i < points.size(); ++i)
  {
    cv::Point p1, p2, p3, p4;
    if (vertical)
    {
      // Draw a plus sign, vertical line:
      p1.x = points[i].x;
      p1.y = static_cast<int>(points[i].y - offset);
      p2.x = p1.x;
      p2.y = static_cast<int>(points[i].y + offset);
      // Horizontal line.
      p3.x = static_cast<int>(points[i].x - offset);
      p3.y = points[i].y;
      p4.x = static_cast<int>(points[i].x + offset);
      p4.y = p3.y;
    }
    else
    {
      // Draw a cross, top-left to bottom-right:
      p1.x = static_cast<int>(points[i].x - offset);
      p1.y = static_cast<int>(points[i].y - offset);
      p2.x = static_cast<int>(points[i].x + offset);
      p2.y = static_cast<int>(points[i].y + offset);
      // Bottom-left to top-right.
      p3.x = p1.x;
      p3.y = p2.y;
      p4.x = p2.x;
      p4.y = p1.y;
    }

    if (dash_length > 0)
    {
      DrawDashedLine(p1, p2, colors[i], dash_length, line_width, draw_on);
      DrawDashedLine(p3, p4, colors[i], dash_length, line_width, draw_on);
    }
    else
    {
      cv::line(draw_on, p1, p2, colors[i], line_width);
      cv::line(draw_on, p3, p4, colors[i], line_width);
    }
  }

  if (alpha < 1.0)
    cv::addWeighted(draw_on, alpha, image, 1.0-alpha, 0.0, image);
  else
    image = draw_on;
}


void DrawCrosses(cv::Mat &image, const std::vector<cv::Point> &points,
                 const cv::Scalar &color, int diagonal, int line_width,
                 int dash_length, double alpha, bool vertical, bool flip_color_channels)
{
  VCP_LOG_DEBUG("DrawCrosses() with single color.");
  std::vector<cv::Scalar> colors;
  colors.reserve(points.size());
  const bool valid_color = !((color[0] < 0.0) || (color[1] < 0.0) || (color[2] < 0.0));
  for (size_t i = 0; i < points.size(); ++i)
  {
    const cv::Scalar ptcolor = valid_color ? color : kExemplaryColors[i % kExemplaryColorsSize];
    if (flip_color_channels)
      colors.push_back(cv::Scalar(ptcolor.val[2], ptcolor.val[1], ptcolor.val[0]));
    else
      colors.push_back(ptcolor);
  }

  DrawCrosses(image, points, colors, diagonal, line_width, dash_length, alpha, vertical);
}

void DrawEllipse(cv::Mat &image, const cv::RotatedRect &rect, const cv::Scalar &color, int line_width, double fill_opacity)
{
  VCP_LOG_DEBUG("DrawEllipse()");
  // Fill, if requested:
  if (fill_opacity > 0.0)
  {
    // Extract image ROI (axis-aligned rectangular region, respecting boundaries)
    const cv::Rect bounding_rect = rect.boundingRect();
    cv::Rect bounding_rect_roi(bounding_rect);
    vcp::imutils::ClipRectangleToImageBoundaries(bounding_rect_roi, image.size());
    if (bounding_rect_roi.width > 0 && bounding_rect_roi.height > 0)
    {
      cv::Mat roi = image(bounding_rect_roi);
      cv::Mat filled;
      roi.copyTo(filled);

      // Shift rotated rect to coordinate frame of ROI
      const float cx = bounding_rect.width/2.0f + (bounding_rect.x < 0 ? bounding_rect.x : 0.0f);
      const float cy = bounding_rect.height/2.0f + (bounding_rect.y < 0 ? bounding_rect.y : 0.0f);
      const cv::Size2f size(rect.size.width-1.0f, rect.size.height-1.0f); // Required, otherwise the bounding rect of the following shifted_rect will be (w+1)x(h+1) :-/
      const cv::RotatedRect shifted_rect(cv::Point2f(cx, cy), size, rect.angle);

      cv::ellipse(filled, shifted_rect, color, -1);
      cv::addWeighted(filled, fill_opacity, roi, 1.0-fill_opacity, 0.0, roi);
    }
  }

  // Draw edges, if requested:
  if (line_width > 0)
    cv::ellipse(image, rect, color, line_width);
}


void DrawCircles(cv::Mat &image, const std::vector<cv::Point> &centers,
                 const std::vector<int> &radii, const std::vector<cv::Scalar> &colors, const cv::Scalar &default_color,
                 int thickness, int line_type)
{
  if (centers.size() != radii.size())
    VCP_ERROR("Number of centers and radii to draw differ.");
  const bool use_defcol = colors.empty() || colors.size() != centers.size();
  for (size_t i = 0; i < centers.size(); ++i)
    cv::circle(image, centers[i], radii[i],
               use_defcol ? default_color : colors[i], thickness, line_type);
}


void DrawXYZAxes(cv::Mat &image, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, const cv::Vec3d &origin,
              double scale_axes, double scale_image_points, int line_width, int dash_length, bool image_is_rgb,
              double tip_length)
{
  VCP_LOG_DEBUG("DrawXYZAxes()");
  // Check if origin is visible.
  // ... it must be in front of the image plane:
  const cv::Vec4d image_plane = vcp::math::geo3d::ImagePlaneInWorldCoordinateSystem(R, t);

  // Project origin and the axis end points:
  const cv::Mat projection_matrix = vcp::math::geo3d::ProjectionMatrixFromKRt(K, R, t);
  cv::Vec3d x = origin + cv::Vec3d(scale_axes, 0.0, 0.0);
  cv::Vec3d y = origin + cv::Vec3d(0.0, scale_axes, 0.0);
  cv::Vec3d z = origin + cv::Vec3d(0.0, 0.0, scale_axes);

  const bool is_origin_in_front = vcp::math::geo3d::IsPointInFrontOfPlane(origin, image_plane);
  if (!is_origin_in_front)
  {
    VCP_LOG_WARNING("DrawXYZAxes(): origin is behind the image plane.");
  }
  else
  {
    // Check if axis end points lie in front of the image plane:
    for (const auto &p : {std::make_pair('x', &x), std::make_pair('y', &y), std::make_pair('z', &z)})
    {
      if (!vcp::math::geo3d::IsPointInFrontOfPlane(*p.second, image_plane))
      {
        VCP_LOG_WARNING("End point of " << p.first << "-axis would project behind the image plane. Adjusting it automatically...");
        const auto axis = vcp::math::geo3d::Line3d(origin, *p.second);
        cv::Vec3d intersection;
        if (vcp::math::geo3d::IntersectionLineSegmentPlane(axis, image_plane, &intersection))
        {
          // Replace the invalid end point by the midpoint from origin to the image plane
          const auto visible_axis = vcp::math::geo3d::Line3d(origin, intersection);
          *p.second = visible_axis.MidPoint();
        }
        else
          VCP_LOG_FAILURE("Couldn't compute a valid intersection point between " << p.second << "-axis and image plane (maybe numerical issue?)");
      }
    }
  }

  std::vector<cv::Point> pts;
  pts.reserve(4);
  pts.push_back(utils::ProjectPointWithScale(projection_matrix, origin, scale_image_points));
  pts.push_back(utils::ProjectPointWithScale(projection_matrix, x, scale_image_points));
  pts.push_back(utils::ProjectPointWithScale(projection_matrix, y, scale_image_points));
  pts.push_back(utils::ProjectPointWithScale(projection_matrix, z, scale_image_points));

  // Check if any point is visible
  bool is_any_visible = false;
  for (size_t i = 0; i < 4; ++i)
    is_any_visible |= vcp::imutils::IsPointInsideImage(pts[i], image.size());
  if (!is_any_visible)
    VCP_LOG_WARNING("DrawXYZAxes(): Neither origin nor the axis end points are visible in the image.");

  // Continue, even if none is visible as an axis' line may still be partially visible - it's too much
  // effort to clip each line prior to drawing (but these checks would be the fool-proof version)
  const cv::Scalar *colors = image_is_rgb ? kAxisColorsRGB : kAxisColorsBGR;

  for (size_t i = 1; i < 4; ++i)
    DrawArrow(pts[0], pts[i], colors[i-1], image, line_width, tip_length, dash_length);
}


void DrawHorizon(cv::Mat &image, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, const cv::Scalar &color, double scale_image_points, int line_width, int dash_length, bool warn_if_not_visible, double text_opacity)
{
  VCP_LOG_DEBUG("DrawHorizon()");
  const double original_width = image.cols / scale_image_points;
  const double original_height = image.rows / scale_image_points;
  vcp::math::geo2d::Line2d horizon = vcp::math::geo2d::GetProjectionOfHorizon(K, R, t, cv::Size(original_width, original_height));
  if (!horizon.empty())
  {
    // Scale the points
    const cv::Vec2d from = scale_image_points * horizon.From();
    const cv::Vec2d to = scale_image_points * horizon.To();
    DrawInfiniteLine(image, vcp::convert::ToPoint(from), vcp::convert::ToPoint(to), color, line_width, dash_length);

    // Draw text
    if (text_opacity > 0.0)
    {
      const cv::Vec2d line_center = 0.5 * (from + to); //from + 0.5 * (to - from);
      DrawTextBox(image, "Horizon", vcp::convert::ToPoint(line_center),
                  textanchor::HCENTER | textanchor::VCENTER, 5, text_opacity, cv::Scalar(0, 0, 0), cv::Scalar(255,255,255),
                 cv::FONT_HERSHEY_PLAIN, 1.0, 1);
    }
  }
  else
  {
    if (warn_if_not_visible)
      VCP_LOG_WARNING("DrawHorizon(): Horizon is not visible in camera view!");
  }
}


void DrawGroundplaneGrid(cv::Mat &image, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, double grid_spacing, const cv::Rect2d &grid_limits,
                         const cv::Vec2d &grid_origin, double scale_image_points,
                         int point_radius, int point_thickness, int line_thickness, double opacity, bool flip_color_channels)
{
  VCP_LOG_DEBUG("DrawGroundplaneGrid()");
  // If our FOV is smaller than the given grid extent, we don't want to project all the points - so first check what's actually visible, then continue to draw.

  // We need to avoid projecting points which are above the horizon!
  const double original_width = image.cols / scale_image_points;
  const double original_height = image.rows / scale_image_points;
  vcp::math::geo2d::Line2d horizon = vcp::math::geo2d::GetProjectionOfHorizon(K, R, t, cv::Size(original_width, original_height));

  std::vector<cv::Vec2d> image_corners_on_gp;
  const cv::Mat P = vcp::math::geo3d::ProjectionMatrixFromKRt(K, R, t);
  const cv::Mat H_img2gp = vcp::math::geo3d::ImageToGroundplaneHomographyFromP(P);

  if (!horizon.empty())
  {
    // Get a vector perpendicular to the horizon and pointing to the bottom edge of the image
    const cv::Vec2d horizon_dir = horizon.Direction();
    cv::Vec2d towards_bottom_edge(horizon_dir[1], -horizon_dir[0]);

    // Flip perpendicular vector, such that it always points towards the bottom edge (work it out on paper, else it might be a bit confusing):
    if (vcp::math::eps_equal(horizon_dir[0], 0.0)) // If the horizon is purely vertical:
    {
      // Flip, if it points up
      if (horizon_dir[1] < 0.0)
        towards_bottom_edge *= -1.0;
    }
    else
    {
      // Flip if horizon goes from left to right:
      if (horizon_dir[0] > 0.0)
        towards_bottom_edge *= -1.0;
    }
    towards_bottom_edge = vcp::math::geo2d::Normalize(towards_bottom_edge);
//FIXME - constrains too much - intersect image border with horizon
    // Horizon maps to the plane at infinity, so take a reference point somewhere below
    image_corners_on_gp.push_back(utils::TransformVecWithScale(H_img2gp, horizon.From() + 2.0 * towards_bottom_edge, 1.0));
    image_corners_on_gp.push_back(utils::TransformVecWithScale(H_img2gp, horizon.To() + 2.0 * towards_bottom_edge, 1.0));
  }
  else
  {
    // If the horizon is not visible, take the top left/right image corners
    image_corners_on_gp.push_back(utils::TransformVecWithScale(H_img2gp, cv::Vec2d(0.0, 0.0)));
    image_corners_on_gp.push_back(utils::TransformVecWithScale(H_img2gp, cv::Vec2d(original_width-1.0, 0.0)));
  }
  // Always take the bottom corners (yes, we deliberately do not support a 180 degree rotated camera) //FIXME maybe change that now in 2020...
  image_corners_on_gp.push_back(utils::TransformVecWithScale(H_img2gp, cv::Vec2d(original_width-1.0, original_height-1.0)));
  image_corners_on_gp.push_back(utils::TransformVecWithScale(H_img2gp, cv::Vec2d(0.0, original_height-1.0)));

  // Find extent
  double x_min = image_corners_on_gp[0].val[0];
  double x_max = x_min;
  double y_min = image_corners_on_gp[0].val[1];
  double y_max = y_min;
  for (size_t i = 1; i < image_corners_on_gp.size(); ++i)
  {
    const double x = image_corners_on_gp[i].val[0];
    const double y = image_corners_on_gp[i].val[1];
    if (x < x_min)
      x_min = x;
    if (x > x_max)
      x_max = x;
    if (y < y_min)
      y_min = y;
    if (y > y_max)
      y_max = y;
  }


  VCP_LOG_DEBUG("DrawGroundplaneGrid():" << std::endl
               << "  Image corners project to ground plane: " << image_corners_on_gp[0] << ", " <<image_corners_on_gp[1] << ", " <<image_corners_on_gp[2] << ", " <<image_corners_on_gp[3] << std::endl
               << "  Requested grid limits: " << grid_limits.tl() << ", " << grid_limits.br() << std::endl
               << "  Drawing grid from x min to max: " << x_min << ", " << x_max << std::endl
               << "                    y min to max: " << y_min << ", " << y_max);

  x_min = std::max(x_min, grid_limits.tl().x);
  x_max = std::min(x_max, grid_limits.br().x);
  y_min = std::max(y_min, grid_limits.tl().y);
  y_max = std::min(y_max, grid_limits.br().y);

  // Compute grid points which are to be drawn
  const int cnt_x_from = static_cast<int>(std::ceil(x_min / grid_spacing));
  const int cnt_x_to = static_cast<int>(std::floor(x_max / grid_spacing));
  const int cnt_y_from = static_cast<int>(std::ceil(y_min / grid_spacing));
  const int cnt_y_to = static_cast<int>(std::floor(y_max / grid_spacing));
  std::vector<cv::Point> grid_points;
  const cv::Mat H_gp2img = vcp::math::geo3d::GroundplaneToImageHomographyFromP(P);
  std::vector<double> distances;
  double min_distance = std::numeric_limits<double>::max();
  double max_distance = 0.0;

  double x = cnt_x_from * grid_spacing;
  for (int cnt_x = cnt_x_from; cnt_x <= cnt_x_to; ++cnt_x)
  {
    double y = cnt_y_from * grid_spacing;
    for (int cnt_y = cnt_y_from; cnt_y <= cnt_y_to; ++cnt_y)
    {
      const cv::Vec2d gp_pos(x,y);
      const cv::Point grid_pt = vcp::convert::ToPoint(utils::TransformVecWithScale(H_gp2img, gp_pos, scale_image_points));

      // The limits above only consider an axis aligned rectangular region, so check again:
      if (grid_pt.x >= 0 && grid_pt.x < image.cols && grid_pt.y >= 0 && grid_pt.y < image.rows)
      {
        grid_points.push_back(grid_pt);

        const double dist = vcp::math::geo2d::Length(gp_pos - grid_origin);
        distances.push_back(dist);
        if (dist < min_distance)
          min_distance = dist;
        if (dist > max_distance)
          max_distance = dist;
      }
      y += grid_spacing;
    }
    x += grid_spacing;
  }

  // Draw grid lines
  const cv::Vec4d clip_plane = vcp::math::geo3d::ImagePlaneInWorldCoordinateSystem(R, t);
  const cv::Scalar &color_x_axis = flip_color_channels ? kAxisColorsRGB[0] : kAxisColorsBGR[0];
  const cv::Scalar &color_y_axis = flip_color_channels ? kAxisColorsRGB[1] : kAxisColorsBGR[1];
  x = cnt_x_from * grid_spacing;
  for (int cnt_x = cnt_x_from; cnt_x <= cnt_x_to; ++cnt_x)
  {
    const cv::Vec2d from(x, y_min), to(x, y_max);
    const vcp::math::geo3d::Line3d clipped = vcp::math::geo3d::ClipLineSegmentByPlane(
          vcp::math::geo3d::Line3d(cv::Vec3d(from[0], from[1], 0.0), cv::Vec3d(to[0], to[1], 0.0)), clip_plane);

    if (!clipped.empty())
    {
      const cv::Vec3d from3 = clipped.From();
      const cv::Vec3d to3 = clipped.To();
      const cv::Vec2d axis_from = utils::TransformVecWithScale(H_gp2img, cv::Vec2d(from3[0], from3[1]), scale_image_points);
      const cv::Vec2d axis_to = utils::TransformVecWithScale(H_gp2img, cv::Vec2d(to3[0], to3[1]), scale_image_points);
      // Finally, clip the 2d line to the image region
      const auto line2d = vcp::math::geo2d::ClipLineSegmentByRectangle(vcp::math::geo2d::Line2d(axis_from, axis_to), vcp::imutils::ImageRect(image));
      cv::line(image, vcp::convert::ToPoint(line2d.From()), vcp::convert::ToPoint(line2d.To()), color_y_axis, line_thickness);
    }
    x += grid_spacing;
  }
  double y = cnt_y_from * grid_spacing;
  for (int cnt_y = cnt_y_from; cnt_y <= cnt_y_to; ++cnt_y)
  {
    const cv::Vec2d from(x_min, y), to(x_max, y);
    const vcp::math::geo3d::Line3d clipped = vcp::math::geo3d::ClipLineSegmentByPlane(
          vcp::math::geo3d::Line3d(cv::Vec3d(from[0], from[1], 0.0), cv::Vec3d(to[0], to[1], 0.0)), clip_plane);

    if (!clipped.empty())
    {
      const cv::Vec3d from3 = clipped.From();
      const cv::Vec3d to3 = clipped.To();
      const cv::Vec2d axis_from = utils::TransformVecWithScale(H_gp2img, cv::Vec2d(from3[0], from3[1]), scale_image_points);
      const cv::Vec2d axis_to = utils::TransformVecWithScale(H_gp2img, cv::Vec2d(to3[0], to3[1]), scale_image_points);
      const auto line2d = vcp::math::geo2d::ClipLineSegmentByRectangle(vcp::math::geo2d::Line2d(axis_from, axis_to), vcp::imutils::ImageRect(image));
      cv::line(image, vcp::convert::ToPoint(line2d.From()), vcp::convert::ToPoint(line2d.To()), color_x_axis, line_thickness);
    }
    y += grid_spacing;
  }


  // Draw points at grid intersection, colored by their distance
  if (grid_points.empty())
  {
    VCP_LOG_WARNING("DrawGroundplaneGrid(): No grid points within the field-of-view - adjust the grid limits!");
  }
  else
  {
    // Assign colors
    std::vector<cv::Scalar> colors;
    colors.reserve(grid_points.size());

    for (size_t i = 0; i < grid_points.size(); ++i)
    {
      const cv::Scalar color = pseudocolor::GetPseudocolor(distances[i], pseudocolor::ColorMap::Parula, min_distance, max_distance);
      if (flip_color_channels)
        colors.push_back(cv::Scalar(color.val[2], color.val[1], color.val[0]));
      else
        colors.push_back(color);
    }

    // Finally: draw!
    DrawPoints(image, grid_points, colors, point_radius, point_thickness, opacity);
  }
}


Box3d::Box3d(const Box3d &other)
{
  if (!other.top_corners_.empty())
    top_corners_.insert(top_corners_.end(), other.top_corners_.begin(), other.top_corners_.end());
  if (!other.bottom_corners_.empty())
    bottom_corners_.insert(bottom_corners_.end(), other.bottom_corners_.begin(), other.bottom_corners_.end());
}

void Box3d::AddTopCorner(const cv::Vec3d &v)
{
  top_corners_.push_back(v);
}


void Box3d::AddBottomCorner(const cv::Vec3d &v)
{
  bottom_corners_.push_back(v);
}

cv::Vec3d Box3d::Centroid() const
{
  cv::Vec3d sum(0.0, 0.0, 0.0);

  for (const auto &v : top_corners_)
    sum += v;
  for (const auto &v : bottom_corners_)
    sum += v;

  return sum / static_cast<double>(top_corners_.size() + bottom_corners_.size());
}

inline std::vector<cv::Vec3d> SortClippingPoints(const std::vector<cv::Vec3d> &clipping_points)
{
  if (clipping_points.empty())
    return std::vector<cv::Vec3d>();

  // All clipping points lie on the image plane, so sort them in 2D (by their rotation about the center)
  const cv::Vec3d centroid = vcp::math::geo3d::Centroid(clipping_points);
  std::vector<float> angles;
  angles.reserve(clipping_points.size());
  for (const cv::Vec3d &v : clipping_points)
  {
    const cv::Vec3d shifted = v - centroid;
    float angle = std::atan2(shifted[1], shifted[0]);
    if (angle < 0)
      angle += MATH_2PI;
    angles.push_back(angle);
  }

  return vcp::utils::SortByExternalVector(clipping_points, angles);
}

inline std::vector<cv::Vec4d> Box3dPlanes(const Box3d &box)
{
  std::vector<cv::Vec4d> planes;
  if (!box.Valid())
    return planes;

  const std::vector<cv::Vec3d> &top = box.TopCorners();
  const std::vector<cv::Vec3d> &bottom = box.BottomCorners();
  // Top
  planes.push_back(vcp::math::geo3d::PlaneFrom3Points(top[0], top[1], top[2]));
  // Bottom
  planes.push_back(vcp::math::geo3d::PlaneFrom3Points(bottom[0], bottom[1], bottom[2]));
  // Walls
  for (size_t i = 0; i < 4; ++i)
    planes.push_back(vcp::math::geo3d::PlaneFrom3Points(top[i], top[(i+1) % top.size()], bottom[i]));
  return planes;
}


inline vcp::math::geo3d::Plane3d PrepareBoxWall(const cv::Vec3d &a, const cv::Vec3d &b, const cv::Vec3d &c, const cv::Vec3d &d)
{
  const cv::Vec4d hnf = vcp::math::geo3d::PlaneFrom3Points(a, b, c);
  const cv::Vec3d origin = (a + b + c + d) / 4.0;

  const cv::Vec3d ref = (a + b) / 2.0;
  const cv::Vec3d e1 = vcp::math::geo3d::Normalize(ref - origin);
  const cv::Vec3d e2 = e1.cross(vcp::math::geo3d::Normalize(vcp::math::geo3d::PlaneNormal(hnf)));

  vcp::math::geo3d::Plane3d plane(hnf, origin, e1, e2);
  // Add all points to the plane's ROI
  plane.AddPolygonPoint(a);
  plane.AddPolygonPoint(b);
  plane.AddPolygonPoint(c);
  plane.AddPolygonPoint(d);

  return plane;
}

inline bool IsRayOccluded(const vcp::math::geo3d::Line3d &ray, const std::vector<vcp::math::geo3d::Plane3d> &box_walls)
{
  for (const vcp::math::geo3d::Plane3d &wall : box_walls)
  {
    cv::Vec3d intersection;
    if (!vcp::math::eps_zero(wall.DistanceToPoint(ray.From())) && wall.IntersectionLineSegment(ray, &intersection))
        return true;
  }
  return false;
}

/** @brief Works with normalized camera coordinates! */
std::vector<bool> GetEdgeVisibility(const std::vector<vcp::math::geo3d::Line3d> &edges, const Box3d &box, const std::vector<cv::Vec3d> &clipping_points, int *text_wall_idx)
{
  std::vector<vcp::math::geo3d::Plane3d> walls;
  // If the box is clipped, check against the clipping polygon on the image plane.
  if (!clipping_points.empty())
  {
    vcp::math::geo3d::Plane3d image_plane(cv::Vec4d(0.0, 0.0, 1.0, -1.0), cv::Vec3d(0.0, 0.0, 1.0), cv::Vec3d(1.0, 0.0, 0.0), cv::Vec3d(0.0, 1.0, 0.0));
    image_plane.AddPolygonPoints(clipping_points);
    walls.push_back(image_plane);
  }

  // Always check against all walls of the box.
  const std::vector<cv::Vec3d> &top = box.TopCorners();
  const std::vector<cv::Vec3d> &bottom = box.BottomCorners();
  // Top
  walls.push_back(PrepareBoxWall(top[0], top[1], top[2], top[3]));
  // Bottom
  walls.push_back(PrepareBoxWall(bottom[0], bottom[1], bottom[2], bottom[3]));
  // Side/body walls
  for (size_t i = 0; i < 4; ++i)
    walls.push_back(PrepareBoxWall(top[i], top[(i+1) % 4], bottom[(i+1) % 4], bottom[i]));

  // Find a "good" box wall to place text upon.
  if (text_wall_idx)
  {
    // Find those walls, which are facing towards
    std::vector<size_t> candidates;
    const size_t offset = clipping_points.empty() ? 0 : 1;
    for (size_t i = offset; i < walls.size(); ++i)
    {
      // Angle is in [0, PI] = [0, 180] deg. We take |angle-90], thus values close to zero
      // belong to walls which are orthogonal to the image plane.
      const double angle = std::fabs(walls[i].Angle(cv::Vec3d(0.0, 0.0, 1.0)) - MATH_PI/2.0);
      if (angle > 0.7) // Aim for at least ~40 deg
        candidates.push_back(i);
    }

    // Sort the candidate walls by their distance to the image plane and take the closest.
    std::vector<double> distances;
    std::vector<size_t> visible;
    for (size_t i = 0; i < candidates.size(); ++i)
    {
      const double origin_dist = vcp::math::geo3d::DistancePointPlane(walls[candidates[i]].Origin(), cv::Vec4d(0.0, 0.0, 1.0, -1.0));
      if (origin_dist >= 0.0)
      {
        visible.push_back(candidates[i]);
        distances.push_back(origin_dist);
      }
    }

    std::vector<size_t> sorted = vcp::utils::SortByExternalVector(visible, distances, vcp::utils::CmpAsc<double>);
    if (!sorted.empty())
      *text_wall_idx = static_cast<int>(sorted[0]-offset);
    else
      *text_wall_idx = -1;
  }

  // Check visibility for each edge.
  std::vector<bool> visible;
  visible.reserve(edges.size());
  for (const auto &edge : edges)
  {
    bool is_visible = false;
    if (!edge.empty())
    {
      cv::Vec3d midpoint = edge.MidPoint();
      midpoint -= 0.1 * vcp::math::geo3d::Normalize(midpoint);
      is_visible = !IsRayOccluded(vcp::math::geo3d::Line3d(midpoint, cv::Vec3d(0.0, 0.0, 0.0)), walls);
    }
    visible.push_back(is_visible);
  }
  return visible;
}


/** @brief Returns the text position corresponding to the text_anchor - if not possible, (-1,-1) is returned. */
cv::Point GetTextPos(const std::vector<vcp::math::geo3d::Line3d> &input_edges,
                         const cv::Mat &P, const cv::Size &image_size, double scale_image_points, int text_anchor)
{
  namespace geo2 = vcp::math::geo2d;
  // Project input edges to image space.
  std::vector<geo2::Line2d> prj_edges_horz, prj_edges_vert;
  // Store the projected midpoints (to simplify computing the centroid later on).
  std::vector<cv::Vec2d> midpoints;
  for (const auto &edge : input_edges)
  {
    const auto prj = utils::GetEdgeProjections(edge, P, image_size, scale_image_points);
    if (!prj.empty())
    {
      // Angle to the vertical image axis (0,1) is in [0, 180].
      // By taking |angle-90|, values >= 45 belong to vertical edges.
      const double angle = std::fabs(prj.Angle(cv::Vec2d(0.0, 1.0)) - MATH_PI/2.0);

      // Normally, we would check for values >= 45 deg (MATH_PI/4) and add those to the vertical edges
      // and others to horizontal edges. However, due to rounding issues and perspective projection,
      // we use a fuzzy threshold and allow adding an edge to both vertical and horizontal candidates.
      if (angle > 0.5) // ~30 deg
        prj_edges_vert.push_back(prj);

      if (angle < 1.0) // ~60
        prj_edges_horz.push_back(prj);

      midpoints.push_back(prj.MidPoint());
    }
  }

  if (prj_edges_horz.empty() && prj_edges_vert.empty())
    return cv::Point(-1, -1);

  // Sort vertical lines by their midpoints to find left/right edge
  std::vector<geo2::Line2d> sorted_lr = vcp::utils::SortVector<geo2::Line2d>(prj_edges_vert,
      [](const geo2::Line2d &a, const geo2::Line2d &b) -> bool
      {
        const cv::Vec2d ma = a.MidPoint();
        const cv::Vec2d mb = b.MidPoint();
        return ma[0] < mb[0];
      });

// Visualization for debugging:
//  if (!sorted_lr.empty())
//  {
//    // Left: cyan
//    // Right: magenta
//    // Top: white
//    // Bottom: gray
//    const geo2::Line2d &left_edge = sorted_lr[0];
//    const geo2::Line2d &right_edge = sorted_lr[sorted_lr.size()-1];
//    cv::line(image, vcp::convert::ToPoint(left_edge.From()), vcp::convert::ToPoint(left_edge.To()), cv::Scalar(0, 255, 255), 3);
//    DrawDashedLine(vcp::convert::ToPoint(right_edge.From()), vcp::convert::ToPoint(right_edge.To()), cv::Scalar(255, 0, 255), 10, 3, image);
//  }
//  if (!sorted_tb.empty())
//  {
//    const geo2::Line2d &top_edge = sorted_tb[0];
//    const geo2::Line2d &bottom_edge = sorted_tb[sorted_tb.size()-1];
//    cv::line(image, vcp::convert::ToPoint(top_edge.From()), vcp::convert::ToPoint(top_edge.To()), cv::Scalar(255, 255, 255), 3);
//    DrawDashedLine(vcp::convert::ToPoint(bottom_edge.From()), vcp::convert::ToPoint(bottom_edge.To()), cv::Scalar(150, 150, 150), 10, 3, image);
//  }


  if (text_anchor & textanchor::LEFT)
  {
    if (sorted_lr.empty())
      return cv::Point(-1, -1);

    const geo2::Line2d &left_edge = sorted_lr[0];
    if (text_anchor & textanchor::TOP)
      return utils::GetEdgeTop(left_edge);
    else if (text_anchor & textanchor::BOTTOM)
      return utils::GetEdgeBottom(left_edge);
    else
      return vcp::convert::ToPoint(left_edge.MidPoint());
  }
  else if (text_anchor & textanchor::RIGHT)
  {
    if (sorted_lr.empty())
      return cv::Point(-1, -1);

    const geo2::Line2d &right_edge = sorted_lr[sorted_lr.size()-1];
    if (text_anchor & textanchor::TOP)
      return utils::GetEdgeTop(right_edge);
    else if (text_anchor & textanchor::BOTTOM)
      return utils::GetEdgeBottom(right_edge);
    else
      return vcp::convert::ToPoint(right_edge.MidPoint());
  }
  else // north, center, south
  {
    // Sort horizontal lines by their midpoints to get the top/bottom edge
    std::vector<geo2::Line2d> sorted_tb = vcp::utils::SortVector<geo2::Line2d>(prj_edges_horz,
        [](const geo2::Line2d &a, const geo2::Line2d &b) -> bool
        {
          const cv::Vec2d ma = a.MidPoint();
          const cv::Vec2d mb = b.MidPoint();
          return ma[1] < mb[1];
        });

    if (text_anchor & textanchor::TOP)
    {
      if (sorted_tb.empty())
        return cv::Point(-1, -1);

      const geo2::Line2d &top_edge = sorted_tb[0];
      return vcp::convert::ToPoint(top_edge.MidPoint());
    }
    else if (text_anchor & textanchor::BOTTOM)
    {
      if (sorted_tb.empty())
        return cv::Point(-1, -1);

      const geo2::Line2d &bottom_edge = sorted_tb[sorted_tb.size()-1];
      return vcp::convert::ToPoint(bottom_edge.MidPoint());
    }
    else
    {
      // We need to find the centroid
      cv::Vec2d centroid;
      for (const auto &mp : midpoints)
        centroid += mp;
      centroid /= static_cast<double>(midpoints.size());
      return vcp::convert::ToPoint(centroid);
    }

  }
  return cv::Point(-1, -1);
}


/** @brief Convenience wrapper. */
cv::Point GetTextPos(const vcp::math::geo3d::Line3d &a, const vcp::math::geo3d::Line3d &b, const vcp::math::geo3d::Line3d &c, const vcp::math::geo3d::Line3d &d,
                         const cv::Mat &P, const cv::Size &image_size, double scale_image_points, int text_anchor)
{
  const std::vector<vcp::math::geo3d::Line3d> input_edges = { a, b, c, d };
  return GetTextPos(input_edges, P, image_size, scale_image_points, text_anchor);
}


bool DrawBox3d(cv::Mat &image, const Box3d &box, const cv::Mat &K, const cv::Mat &Rt, const cv::Scalar &color, cv::Rect *bounding_rect_box, cv::Point *text_pos, std::vector<cv::Point> *convex_hull, int text_anchor, int line_width, double scale_image_points, int dash_length, double top_fill_opacity, double bottom_fill_opacity)
{
  VCP_LOG_DEBUG("DrawBox3d()");
  namespace geo3 = vcp::math::geo3d;

  if (!box.Valid())
    return false;

  // We compute everything in normalized camera coordinates
  const Box3d box_normed = utils::NormalizeBox3dCoordinates(box, Rt);

  // Clip all edges against the image plane
  const cv::Vec4d image_plane(0.0, 0.0, 1.0, -1.0);

  std::vector<geo3::Line3d> edges_top, edges_body, edges_bottom;
  const std::vector<cv::Vec3d> &top_corners = box_normed.TopCorners();
  const std::vector<cv::Vec3d> &bottom_corners = box_normed.BottomCorners();

  std::vector<cv::Vec3d> clipping_points; // < the points which are exactly on the image plane (used to draw dashed lines, indicating the clipping)
  bool all_behind_plane = true;
  for (size_t i = 0; i < 4; ++i)
  {
    // Clip top edge.
    const cv::Vec3d &tfrom = top_corners[i];
    const cv::Vec3d &tto = top_corners[(i+1) % 4];
    const geo3::Line3d clipped_top = geo3::ClipLineSegmentByPlane(geo3::Line3d(tfrom, tto), image_plane);
    edges_top.push_back(clipped_top);
    all_behind_plane &= clipped_top.empty();

    if (!clipped_top.empty())
    {
      if (!vcp::math::IsVecEqual(tfrom, clipped_top.From()))
        clipping_points.push_back(clipped_top.From());
      if (!vcp::math::IsVecEqual(tto, clipped_top.To()))
        clipping_points.push_back(clipped_top.To());
    }

    // Clip bottom edge.
    const cv::Vec3d bfrom = bottom_corners[i];
    const cv::Vec3d bto = bottom_corners[(i+1) % 4];
    const geo3::Line3d clipped_bottom = geo3::ClipLineSegmentByPlane(geo3::Line3d(bfrom, bto), image_plane);
    edges_bottom.push_back(clipped_bottom);
    all_behind_plane &= clipped_bottom.empty();

    if (!clipped_bottom.empty())
    {
      if (!vcp::math::IsVecEqual(bfrom, clipped_bottom.From()))
        clipping_points.push_back(clipped_bottom.From());
      if (!vcp::math::IsVecEqual(bto, clipped_bottom.To()))
        clipping_points.push_back(clipped_bottom.To());
    }

    // Clip side/body edge.
    const geo3::Line3d clipped_body = geo3::ClipLineSegmentByPlane(geo3::Line3d(tfrom, bfrom), image_plane);
    edges_body.push_back(clipped_body);
    all_behind_plane &= clipped_body.empty();

    if (!clipped_body.empty())
    {
      if (!vcp::math::IsVecEqual(tfrom, clipped_body.From()))
        clipping_points.push_back(clipped_body.From());
      if (!vcp::math::IsVecEqual(bfrom, clipped_body.To()))
        clipping_points.push_back(clipped_body.To());
    }
  }

  // Early stopping if box is not visible at all.
  if (all_behind_plane)
    return false;


  // Make clipping points a closed convex polygon (via sorting by the angle on the image plane).
  if (!clipping_points.empty())
  {
    clipping_points = SortClippingPoints(clipping_points);
    clipping_points.push_back(clipping_points[0]);
  }


  // Extract visible parts of the box's top and bottom plane.
  std::vector<cv::Vec3d> polygon_top;
  std::vector<cv::Vec3d> polygon_bottom;
  for (int i = 0; i < 4; ++i)
  {
    // Check for visible top corners.
    const geo3::Line3d &curr_top_edge = edges_top[i];
    const geo3::Line3d &prev_top_edge = edges_top[vcp::math::Mod(i-1, edges_top.size())];
    if (curr_top_edge.empty())
    {
      // Current is behind image plane, but the previous one might be visible - so take its end point.
      if (!prev_top_edge.empty())
        polygon_top.push_back(prev_top_edge.To());
    }
    else
    {
      // Current edge exists, take its start point.
      // Additionally, we may need to add the previous edge's end point - if it exists and the points differ
      if (!prev_top_edge.empty() && !vcp::math::IsVecEqual(prev_top_edge.To(), curr_top_edge.From()))
        polygon_top.push_back(prev_top_edge.To());
      polygon_top.push_back(curr_top_edge.From());
    }

    // Similar for bottom corners. See inline doc above.
    const geo3::Line3d &curr_bottom_edge = edges_bottom[i];
    const geo3::Line3d &prev_bottom_edge = edges_bottom[vcp::math::Mod(i-1, edges_top.size())];
    if (curr_bottom_edge.empty())
    {
      if (!prev_bottom_edge.empty())
        polygon_bottom.push_back(prev_bottom_edge.To());
    }
    else
    {
      if (!prev_bottom_edge.empty() && !vcp::math::IsVecEqual(prev_bottom_edge.To(), curr_bottom_edge.From()))
        polygon_bottom.push_back(prev_bottom_edge.To());
      polygon_bottom.push_back(curr_bottom_edge.From());
    }
  }


  // If the top or bottom plane is visible, fill the corresponding polygon.
  if (!polygon_top.empty())
  {
    polygon_top.push_back(polygon_top[0]);
    DrawPolygon3d(image, polygon_top, K, color, top_fill_opacity, -1, -1, scale_image_points);
  }
  if (!polygon_bottom.empty())
  {
    polygon_bottom.push_back(polygon_bottom[0]);
    DrawPolygon3d(image, polygon_bottom, K, color, bottom_fill_opacity, -1, -1, scale_image_points);
  }


  // Combine all edges into a large vector for visibility check (and drawing):
  std::vector<geo3::Line3d> edges;
  edges.insert(edges.end(), edges_top.begin(), edges_top.end());
  edges.insert(edges.end(), edges_body.begin(), edges_body.end());
  edges.insert(edges.end(), edges_bottom.begin(), edges_bottom.end());
  int text_wall_idx;
  const std::vector<bool> visible = GetEdgeVisibility(edges, box_normed, clipping_points, text_pos ? &text_wall_idx : nullptr);
  // While we're projecting the lines, also compute their min/max pixel location, so we can compute a bounding rectangle
  // (which is useful to mask an image, position text, check if anything has been drawn, etc.)
  cv::Point min_prj(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()),
      max_prj(std::numeric_limits<int>::min(), std::numeric_limits<int>::min());
  // Keep also track of the corresponding pixel locations, so we can compute the convex hull of the rectangle (if requested).
  std::vector<cv::Point> projections;
  for (size_t i = 0; i < edges.size(); ++i)
  {
    const geo3::Line3d &edge = edges[i];
    if (!edge.empty())
    {
      cv::Point prj1, prj2;
      const bool dashed = !visible[i];
      DrawLineSegment3d(image, edge.From(), edge.To(), K, color, line_width, dashed ? dash_length : -1, scale_image_points, &prj1, &prj2);

      if (prj1.x < min_prj.x)
        min_prj.x = prj1.x;
      if (prj1.x > max_prj.x)
        max_prj.x = prj1.x;
      if (prj1.y < min_prj.y)
        min_prj.y = prj1.y;
      if (prj1.y > max_prj.y)
        max_prj.y = prj1.y;

      if (prj2.x < min_prj.x)
        min_prj.x = prj2.x;
      if (prj2.x > max_prj.x)
        max_prj.x = prj2.x;
      if (prj2.y < min_prj.y)
        min_prj.y = prj2.y;
      if (prj2.y > max_prj.y)
        max_prj.y = prj2.y;

      if (convex_hull)
      {
        projections.push_back(prj1);
        projections.push_back(prj2);
      }
    }
  }

  // Check, if we need to report the bounding rectangle of the full projected box.
  cv::Rect bounding_rect_box_(min_prj.x, min_prj.y, max_prj.x - min_prj.x +1 , max_prj.y - min_prj.y + 1);
  vcp::imutils::ClipRectangleToImageBoundaries(bounding_rect_box_, image.size());
  if (bounding_rect_box)
  {
    *bounding_rect_box = bounding_rect_box_;
  }

  // Check, if we need to compute the bounding rectangle of the nearest plane.
  if (text_pos)
  {
    if (text_wall_idx < 0)
    {
      *text_pos = GetTextPos(bounding_rect_box_, text_anchor);
    }
    else
    {
      if (text_wall_idx == 0)
        *text_pos = GetTextPos(edges_top[0], edges_top[1], edges_top[2], edges_top[3], K, image.size(), scale_image_points, text_anchor);
      else if (text_wall_idx == 1)
        *text_pos = GetTextPos(edges_bottom[0], edges_bottom[1], edges_bottom[2], edges_bottom[3], K, image.size(), scale_image_points, text_anchor);
      else
        *text_pos = GetTextPos(edges_top[text_wall_idx-2], edges_body[text_wall_idx-2],
          edges_bottom[text_wall_idx-2], edges_body[(text_wall_idx-1) % 4], K, image.size(), scale_image_points, text_anchor);
      if (text_pos->x < 0 || text_pos->y < 0)
        *text_pos = GetTextPos(bounding_rect_box_, text_anchor);
    }
  }

  // Check if we need to compute the convex hull
  if (convex_hull)
  {
    // Add points of the clipping plane, then compute the convex hull.
    for (const cv::Vec3d &cp : clipping_points)
      projections.push_back(utils::ProjectPointWithScale(K, cp, scale_image_points));

    const std::vector<cv::Point> hull = vcp::math::geo2d::ConvexHullGrahamScan(projections, true);
    convex_hull->clear();
    convex_hull->insert(convex_hull->end(), hull.begin(), hull.end());
  }

  // Visualize the clipping plane
  // The complementary color draws too much attention ;-) ComplementaryColor(color,true)
  DrawPolygon3d(image, clipping_points, K, cv::Scalar::all(180), 0.0, line_width, dash_length, scale_image_points);
  return bounding_rect_box_.area() > 0;
}


void DrawBoundingBoxes3d(cv::Mat &image, const cv::Mat &K, const cv::Mat &Rt, const std::vector<Box3d> &boxes, const std::vector<cv::Scalar> &box_colors,
                         const std::vector<std::string> &captions, bool non_overlapping, int line_width, int dash_length, double scale_projected_image_points,
                         double fill_top_opacity, double fill_bottom_opacity, int text_anchor, int text_box_padding, double text_box_opacity,
                         const std::vector<cv::Scalar> &font_colors, const std::vector<cv::Scalar> &text_box_colors, int font_face, double font_scale, int font_thickness)
{
  VCP_LOG_DEBUG("DrawBoundingBoxes3d()");
  const bool text_bg_provided = !text_box_colors.empty();
  const bool text_fg_provided = !font_colors.empty();

  if (non_overlapping)
  {
    cv::Mat mask = cv::Mat::ones(image.size(), CV_8UC1);

    // Sort boxes by their distance to the optical center (so that masking them for the non-overlapping check works)
    std::vector<double> distances;
    distances.reserve(boxes.size());

    const cv::Vec3d optical_center = vcp::math::geo3d::CameraCenterFromRt(Rt);
    for (size_t i = 0; i < boxes.size(); ++i)
      distances.push_back(vcp::math::geo3d::Distance(optical_center, boxes[i].Centroid()));

    std::vector<size_t> lookup = vcp::utils::GetSortedIndices(distances, vcp::utils::CmpAsc<double>);

    for (size_t i = 0; i < boxes.size(); ++i)
    {
      const size_t idx = lookup[i];

      const cv::Scalar text_bg_color = text_bg_provided ? text_box_colors[idx] : box_colors[idx];
      const cv::Scalar text_fg_color = text_fg_provided ? font_colors[idx] : ComplementaryColor(text_bg_color, true);

      // Draw onto clone
      cv::Mat clone = image.clone();

      cv::Point text_pos;
      cv::Rect bounding_rect;
      std::vector<cv::Point> convex_hull;
      const bool has_caption = !captions[idx].empty();
      const bool drawn = DrawBox3d(clone, boxes[idx], K, Rt, box_colors[idx],
        &bounding_rect, has_caption ? &text_pos : nullptr, &convex_hull, text_anchor,
        line_width, scale_projected_image_points, dash_length, fill_top_opacity, fill_bottom_opacity);

      if (drawn)
      {
        if (has_caption)
        {
          cv::Rect text_roi;
          DrawTextBox(clone, captions[idx], text_pos, text_anchor,
                      text_box_padding, text_box_opacity, text_fg_color, text_bg_color,
                      font_face, font_scale, font_thickness, &text_roi);
          bounding_rect = bounding_rect | text_roi; // Union
        }

        // Copy valid area onto image
        vcp::imutils::ClipRectangleToImageBoundaries(bounding_rect, image.size());
        cv::Mat clone_roi = clone(bounding_rect);
        cv::Mat image_roi = image(bounding_rect);
        cv::Mat mask_roi = mask(bounding_rect);
        clone_roi.copyTo(image_roi, mask_roi);

        // Unset mask:
        DrawPolygon2d(mask, convex_hull, cv::Scalar::all(0.0), line_width, -1, 1.0);
        if (has_caption)
        {
          DrawTextBox(mask, captions[idx], text_pos, text_anchor,
                      text_box_padding, 1.0, cv::Scalar::all(0.0), cv::Scalar::all(0.0),
                      font_face, font_scale, font_thickness);
        }
      }
    }
  }
  else
  {
    for (size_t i = 0; i < boxes.size(); ++i)
    {
      const cv::Scalar text_bg_color = text_bg_provided ? text_box_colors[i] : box_colors[i];
      const cv::Scalar text_fg_color = text_fg_provided ? font_colors[i] : ComplementaryColor(text_bg_color, true);

      cv::Point text_pos;
      const bool has_caption = !captions[i].empty();
      const bool drawn = DrawBox3d(image, boxes[i], K, Rt, box_colors[i],
        nullptr, has_caption ? &text_pos : nullptr, nullptr, text_anchor,
        line_width, scale_projected_image_points, dash_length, fill_top_opacity, fill_bottom_opacity);

      if (drawn && has_caption)
      {
        DrawTextBox(image, captions[i], text_pos, text_anchor,
                    text_box_padding, text_box_opacity, text_fg_color, text_bg_color,
                    font_face, font_scale, font_thickness);
      }
    }
  }
}

} // namespace drawing
} // namespace imvis
} // namespace vcp
