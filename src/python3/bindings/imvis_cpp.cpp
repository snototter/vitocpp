#include "conversion/cv_core_conversion.h"
#include "conversion/np_cv_conversion.h"
#include "conversion/vcp_conversion.h"

#include <vcp_imvis/anaglyph.h>
#include <vcp_imvis/collage.h>
#include <vcp_imvis/drawing.h>
#include <vcp_imvis/trajectories.h>
#include <vcp_imvis/pseudocolor.h>
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/sort_utils.h>
#include <vcp_math/geometry2d.h>
#include <vcp_math/geometry3d.h>
#include <vcp_math/common.h>
#include <vcp_imutils/imutils.h>
#include <vcp_imutils/matutils.h>

#ifdef WITH_OPENCV3
  #include <opencv2/imgproc.hpp>
#else
  #include <opencv2/imgproc/imgproc.hpp>
#endif


//-----------------------------------------------------------------------------
// Wrapper/Helper code

namespace vcp
{
namespace python
{
namespace imvis
{
py::tuple ColorById(size_t id)
{
  const cv::Scalar c = vcp::imvis::drawing::GetExemplaryColor(id);
  return py::make_tuple(c[0], c[1], c[2]);
}


cv::Mat MakeCollage(const std::vector<cv::Mat> &images, int num_images_per_row,
                    size_t padding, const cv::Size &fixed_size_per_image,
                    const cv::Scalar &bg_color, bool convert_to_uint8)
{
  cv::Mat collage;
  vcp::imvis::collage::Collage(images, collage, num_images_per_row, padding, fixed_size_per_image, convert_to_uint8, bg_color);
  return collage;
}


cv::Mat MakeAnaglyph(const cv::Mat &left, const cv::Mat &right, const py::tuple &shift, bool input_is_rgb)
{
  const int offset_x = py::cast<int>(shift[0]);
  const int offset_y = py::cast<int>(shift[1]);

  cv::Mat shifted, anaglyph, invalid;
  shifted = vcp::imvis::anaglyph::ShiftImage(right, offset_x, offset_y, &invalid);
  vcp::imvis::anaglyph::GenerateAnaglyph(left, shifted, anaglyph, input_is_rgb, &invalid);
  return anaglyph;
}


cv::Mat DrawBoundingBoxes2D(const cv::Mat &image,
            const std::vector<vcp::python::conversion::BoundingBox2d> &bounding_boxes,
            const cv::Scalar &default_box_color,
            int line_width, double fill_opacity,
            const cv::Scalar &font_color, int text_anchor,
            double font_scale, int font_thickness, int text_box_padding,
            double text_box_opacity, bool non_overlapping)
{
  const bool flip_colors = false;
  const bool use_complementary_font_color = font_color[0] < 0.0 || font_color[1] < 0.0 || font_color[2] < 0.0;
  const int dash_length = 10;

  std::vector<cv::Rect> rects;
  std::vector<cv::Scalar> colors;
  std::vector<int> dash_lengths;
  std::vector<std::string> captions;
  std::vector<cv::Scalar> text_fg_colors, text_bg_colors;

  for (const auto &bbox : bounding_boxes)
  {
    if (bbox.empty())
      continue;

    const cv::Scalar color = bbox.Color(default_box_color);
    rects.push_back(bbox.box);
    colors.push_back(color);
    dash_lengths.push_back(bbox.is_dashed ? dash_length : -1);

    if (!bbox.caption.empty())
    {
      // Check if we should compute the complementary color (and if yes, we may have to flip the color channels).
      cv::Scalar text_color = font_color;
      if (use_complementary_font_color)
      {
        text_color = vcp::imvis::drawing::ComplementaryColor(color, true);
        if (flip_colors)
        {
          double r = text_color[0];
          text_color[0] = text_color[2];
          text_color[2] = r;
        }
      }

      captions.push_back(bbox.caption);
      text_fg_colors.push_back(text_color);
      text_bg_colors.push_back(color);
    }
    else
    {
      captions.push_back("");
      text_fg_colors.push_back(cv::Scalar());
      text_bg_colors.push_back(cv::Scalar());
    }
  }

  cv::Mat vis = image.clone();
  vcp::imvis::drawing::DrawBoundingBoxes2d(vis, rects, colors, captions, non_overlapping,
                                 line_width, fill_opacity, dash_lengths, text_anchor, text_fg_colors, text_bg_colors,
                                 text_box_padding, text_box_opacity, cv::FONT_HERSHEY_PLAIN, font_scale, font_thickness);

  return vis;
}


cv::Mat DrawRoundedRects(const cv::Mat &image, const std::vector<cv::Rect> &rects, const cv::Scalar &color,
                         double corner_percentage, double fill_opacity, int line_width, bool non_overlapping)
{
  cv::Mat img = image.clone();
  vcp::imvis::drawing::DrawRoundedRects(img, rects, color, corner_percentage, fill_opacity, line_width, non_overlapping);
  return img;
}


cv::Mat DrawRects(const cv::Mat &image, const std::vector<cv::Rect> &rects, const cv::Scalar &color,
                  double fill_opacity, int line_width, int dash_length, bool non_overlapping)
{
  cv::Mat img = image.clone();
  vcp::imvis::drawing::DrawRects(img, rects, color, fill_opacity, line_width, dash_length, non_overlapping);
  return img;
}


cv::Mat DrawRotatedRects(const cv::Mat &image, const std::vector<cv::RotatedRect> &rects, const cv::Scalar &color,
                         double fill_opacity, int line_width, int dash_length, bool non_overlapping)
{
  cv::Mat img = image.clone();
  vcp::imvis::drawing::DrawRotatedRects(img, rects, color, fill_opacity, line_width, dash_length, non_overlapping);
  return img;
}



cv::Mat DrawTextBox(const cv::Mat &image, const std::string &text, const cv::Point &pos, int textbox_anchor,
                    const cv::Scalar &bg_color, const cv::Scalar &font_color, double font_scale, int font_thickness,
                    int padding, double fill_opacity)
{
  cv::Scalar fg_color;
  if (font_color[0] < 0.0 || font_color[1] < 0.0 || font_color[2] < 0.0)
    fg_color = vcp::imvis::drawing::ComplementaryColor(bg_color, true);
  else
    fg_color = font_color;

  cv::Mat img = image.clone();
  vcp::imvis::drawing::DrawTextBox(img, text, pos, textbox_anchor, padding, fill_opacity,
                                           fg_color, bg_color, cv::FONT_HERSHEY_PLAIN, font_scale,
                                           font_thickness);
  return img;
}





cv::Mat DrawLines(const cv::Mat &image, const std::vector<vcp::python::conversion::VisLine2d> &lines,
                  const cv::Scalar &default_color, int line_width, int dash_length)
{
  std::vector<cv::Point> start_points, end_points;
  std::vector<cv::Scalar> colors;
  std::vector<int> dash_lengths;

  for (const auto &line : lines)
  {
    if (line.empty())
      continue;

    start_points.push_back(line.FromPt());
    end_points.push_back(line.ToPt());
    colors.push_back(line.Color(default_color));

    if (line.is_dashed)
      dash_lengths.push_back(dash_length);
    else
      dash_lengths.push_back(-1);
  }

  cv::Mat img = image.clone();
  vcp::imvis::drawing::DrawLines(img, start_points, end_points, colors, line_width, dash_lengths);
  return img;
}

cv::Mat DrawArrows(const cv::Mat &image, const std::vector<vcp::python::conversion::VisLine2d> &arrows,
                   const cv::Scalar &default_color, int line_width, double arrow_head_factor, int dash_length)
{
  cv::Mat img = image.clone();

  for (const auto &arrow : arrows)
  {
    if (arrow.empty())
      continue;

    vcp::imvis::drawing::DrawArrow(arrow.FromPt(), arrow.ToPt(), arrow.Color(default_color),
                                img, line_width, arrow_head_factor, arrow.is_dashed ? dash_length : -1);
  }
  return img;
}


cv::Mat DrawRotatedBoundingBoxes2D(const cv::Mat &image, const std::vector<vcp::python::conversion::RotatedBoundingBox2d> &bounding_boxes,
                                   const cv::Scalar &default_box_color, int line_width, double fill_opacity,
                                   const cv::Scalar &font_color, int text_anchor, double font_scale,
                                   int font_thickness, int text_box_padding, double text_box_opacity, bool non_overlapping,
                                   int dash_length)
{
  const bool use_complementary_font_color = font_color[0] < 0.0 || font_color[1] < 0.0 || font_color[2] < 0.0;

  cv::Mat img = image.clone();

  std::vector<cv::RotatedRect> rects;
  std::vector<cv::Scalar> colors;
  std::vector<int> dash_lengths;
  std::vector<std::string> captions;
  std::vector<cv::Scalar> text_fg_colors, text_bg_colors;

  for (const auto &rbox : bounding_boxes)
  {
    if (rbox.empty())
      continue;

    rects.push_back(rbox.box);
    const cv::Scalar color = rbox.Color(default_box_color);
    colors.push_back(color);
    dash_lengths.push_back(rbox.is_dashed ? dash_length : -1);

    if (!rbox.caption.empty())
    {
      // Check if we should compute the complementary color.
      cv::Scalar text_color = font_color;
      if (use_complementary_font_color)
        text_color = vcp::imvis::drawing::ComplementaryColor(color, true);

      captions.push_back(rbox.caption);
      text_fg_colors.push_back(text_color);
      text_bg_colors.push_back(color);
    }
    else
    {
      captions.push_back("");
      text_fg_colors.push_back(cv::Scalar());
      text_bg_colors.push_back(cv::Scalar());
    }
  }

  vcp::imvis::drawing::DrawRotatedBoundingBoxes2d(img, rects, colors, captions,
          non_overlapping, line_width, fill_opacity, dash_lengths, text_anchor, text_fg_colors, text_bg_colors,
          text_box_padding, text_box_opacity, cv::FONT_HERSHEY_PLAIN, font_scale, font_thickness);

  return img;
}


cv::Mat DrawPoints(const cv::Mat &image, const std::vector<cv::Point> &points, const py::object &py_color,
                   int radius, int line_width, double alpha)
{
  cv::Mat img = image.clone();

  if (py::isinstance<py::list>(py_color))
  {
    const py::list list = py_color.cast<py::list>();
    if (py::len(list) != points.size())
      throw std::runtime_error("Lengths of points and corresponding colors doesn't match!");

    std::vector<cv::Scalar> colors;
    for (size_t i = 0; i < points.size(); ++i)
      colors.push_back(vcp::python::conversion::PyObjectToScalar(list[i], nullptr));

    vcp::imvis::drawing::DrawPoints(img, points, colors, radius, line_width, alpha);
  }
  else
  {
    const cv::Scalar single_color = vcp::python::conversion::PyObjectToScalar(py_color, nullptr);
    vcp::imvis::drawing::DrawPoints(img, points, single_color, radius, line_width, alpha, false);
  }

  return img;
}

cv::Mat DrawXYZAxes(const cv::Mat &image, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t,
                    const cv::Vec3d &origin, double scale_axes, double scale_image_points, int line_width,
                    int dash_length, bool image_is_rgb)
{
  cv::Mat img = image.clone();
  vcp::imvis::drawing::DrawXYZAxes(img, K, R, t, origin, scale_axes, scale_image_points, line_width, dash_length, image_is_rgb);
  return img;
}


cv::Mat DrawHorizon(const cv::Mat &image, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t,
                    const cv::Scalar &color, double scale_image_points, int line_width, int dash_length,
                    bool warn_if_not_visible)
{
  cv::Mat img = image.clone();
  vcp::imvis::drawing::DrawHorizon(img, K, R, t, color, scale_image_points, line_width, dash_length, warn_if_not_visible);
  return img;
}


cv::Mat DrawGroundplaneGrid(const cv::Mat &image, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t,
                            double grid_spacing, const cv::Rect2d &grid_limits, const cv::Vec2d &grid_origin,
                            double scale_image_points, int point_radius, int line_width, double opacity)
{
  cv::Mat img = image.clone();
  vcp::imvis::drawing::DrawGroundplaneGrid(img, K, R, t, grid_spacing, grid_limits, grid_origin,
                                                   scale_image_points, point_radius, line_width, opacity, false);
  return img;
}

cv::Mat DrawFadingTrajectory(const cv::Mat &image, const std::vector<cv::Vec2d> &positions, bool newest_position_first,
                             int smoothing_window, int trajectory_length, const cv::Scalar &obj_color, const cv::Scalar &fade_color,
                             int max_line_width, int dash_length)
{
  cv::Mat img = image.clone();
  vcp::imvis::trajectories::DrawFadingTrajectory2d(img, positions, newest_position_first, smoothing_window,
                                                           trajectory_length, obj_color, fade_color,
                                                           max_line_width, dash_length);
  return img;
}


cv::Mat DrawTrajectory(const cv::Mat &image, const std::vector<cv::Vec2d> &positions, bool newest_position_first,
                       int smoothing_window, int trajectory_length, const cv::Scalar &obj_color, int line_width,
                       int dash_length)
{
  cv::Mat img = image.clone();
  vcp::imvis::trajectories::DrawTrajectory2d(img, positions, newest_position_first, smoothing_window,
                                                     trajectory_length, obj_color, line_width, dash_length);
  return img;
}


cv::Mat DrawPolygon(const cv::Mat &image, const std::vector<cv::Point> &polygon, const cv::Scalar &color,
                    int line_width, int dash_length, double fill_opacity)
{
  cv::Mat img = image.clone();
  vcp::imvis::drawing::DrawPolygon2d(img, polygon, color, line_width, dash_length, fill_opacity);
  return img;
}


cv::Mat DrawBoundingBoxes3D(const cv::Mat &image, const std::vector<vcp::python::conversion::BoundingBox3d> &bounding_boxes,
                            const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, double scale_image_points,
                            const cv::Scalar &default_box_color, int line_width,
                            int dash_length, double fill_opacity_top, double fill_opacity_bottom,
                            const cv::Scalar &font_color, int text_anchor,
                            double font_scale, int font_thickness, int text_box_padding,
                            double text_box_opacity, bool non_overlapping)
{
  const bool use_complementary_font_color = font_color[0] < 0.0 || font_color[1] < 0.0 || font_color[2] < 0.0;

  cv::Mat img = image.clone();
  const cv::Mat Rt = vcp::imutils::ColumnStack(R, t);

  // Extract the bounding boxes:
  std::vector<vcp::imvis::drawing::Box3d> boxes;
  std::vector<cv::Scalar> colors;
  std::vector<std::string> captions;
  std::vector<cv::Scalar> text_fg_colors, text_bg_colors;

  for (const auto &box : bounding_boxes)
  {
    if (box.empty())
      continue;

    boxes.push_back(box.box);
    const cv::Scalar box_color = box.Color(default_box_color);
    colors.push_back(box_color);

    if (!box.caption.empty())
    {
      // Check if we should compute the complementary color (and if yes, we may have to flip the color channels).
      cv::Scalar text_color = font_color;
      if (use_complementary_font_color)
        text_color = vcp::imvis::drawing::ComplementaryColor(box_color, true);

      captions.push_back(box.caption);
      text_fg_colors.push_back(text_color);
      text_bg_colors.push_back(box_color);
    }
    else
    {
      captions.push_back("");
      text_fg_colors.push_back(cv::Scalar());
      text_bg_colors.push_back(cv::Scalar());
    }
  }

  vcp::imvis::drawing::DrawBoundingBoxes3d(img, K, Rt, boxes,
              colors, captions, non_overlapping, line_width, dash_length, scale_image_points,
              fill_opacity_top, fill_opacity_bottom, text_anchor, text_box_padding,
              text_box_opacity, text_fg_colors, text_bg_colors,
              cv::FONT_HERSHEY_PLAIN, font_scale, font_thickness);
  return img;
}

cv::Mat DrawErrorEllipse90(const cv::Mat &image, const cv::Vec2d &mean, const cv::Mat &cov_mat, const cv::Scalar &color, int line_width=1, double fill_opacity=0.0)
{
  cv::Mat vis = image.clone();
  vcp::imvis::trajectories::DrawErrorEllipse90(vis, mean, cov_mat, color, line_width, fill_opacity);
  return vis;
}

cv::Mat DrawErrorEllipse95(const cv::Mat &image, const cv::Vec2d &mean, const cv::Mat &cov_mat, const cv::Scalar &color, int line_width=1, double fill_opacity=0.0)
{
  cv::Mat vis = image.clone();
  vcp::imvis::trajectories::DrawErrorEllipse95(vis, mean, cov_mat, color, line_width, fill_opacity);
  return vis;
}

cv::Mat DrawErrorEllipse99(const cv::Mat &image, const cv::Vec2d &mean, const cv::Mat &cov_mat, const cv::Scalar &color, int line_width=1, double fill_opacity=0.0)
{
  cv::Mat vis = image.clone();
  vcp::imvis::trajectories::DrawErrorEllipse99(vis, mean, cov_mat, color, line_width, fill_opacity);
  return vis;
}


cv::Mat RenderPerspective(const cv::Mat &image, float rx, float ry, float rz, float tx, float ty, float tz, const py::tuple &border_color)
{
  const cv::Scalar bg_color = border_color.is_none() ? cv::Scalar::all(-1) : vcp::python::conversion::PyObjectToScalar(border_color, nullptr);
  return vcp::imvis::collage::RenderPerspective(image, rx, ry, rz, tx, ty, tz, bg_color);
}

} // namespace imvis
} // namespace python
} // namespace vcp


//-----------------------------------------------------------------------------
// Python module declarations

PYBIND11_MODULE(imvis_cpp, m)
{
  namespace vpi = vcp::python::imvis;
  m.doc() = "Python/C++ bindings for vcp::imvis";

  m.def("color_by_id__", &vpi::ColorById,
        "Color for a given object ID, label, etc.\n:return: RGB/BGR tuple",
        py::arg("id"));


  m.def("complementary_color", &vcp::imvis::drawing::ComplementaryColor,
        "Returns the complementary color (opposite on the HSV color wheel) of\n"
        "the RGB (is_rgb==true) or BGR (is_rgb==false) tuple.",
        py::arg("color"),
        py::arg("is_rgb")=true);


  m.def("make_collage", &vpi::MakeCollage,
        "Arrange the given list of images as a collage for display.\n\n"
        ":param images: list of numpy.array\n"
        ":param num_images_per_row: how many images in each row\n"
        ":param padding: padding in pixels between images\n"
        ":param fixed_size_per_image: set to (w,h) if inputs should be resized\n"
        ":param bg_color: RGB/BGR tuple for background\n"
        ":param convert_to_uint8: true if you want to explicitly cast all inputs to uint8\n"
        ":return: the collage",
        py::arg("images"), py::arg("num_images_per_row") = 3,
        py::arg("padding") = 0,
        py::arg("fixed_size_per_image") = cv::Size(0,0),
        py::arg("bg_color") = cv::Scalar::all(0.0),
        py::arg("convert_to_uint8") = false);


  m.def("render_perspective", &vpi::RenderPerspective,
        "Applies a perspective warp to the given image\n"
        "such that it looks like the image plane would\n"
        "be viewed from a camera with the given extrinsics.\n\n"
        ":param image: numpy ndarray.\n"
        ":params rx, ry, rz: Rotation angles (float) in radians.\n"
        ":params tx, ty, tz: Translation vector components (float).\n"
        ":param border_color: If None, output will be a RGBA/BGRA image\n"
        "         where invalid regions are masked out via the alpha\n"
        "         channel. Otherwise it must be a 3-element tuple,\n"
        "         specifying the background/replacement RGB/BGR color.\n"
        ":return: numpy ndarray, 3 or 4 channel image",
        py::arg("image"),
        py::arg("rx")=0, py::arg("ry")=0, py::arg("rz")=0,
        py::arg("tx")=0, py::arg("ty")=0, py::arg("tz")=0,
        py::arg("border_color")=py::none());


  m.def("make_anaglyph", &vpi::MakeAnaglyph,
        "Make a red-cyan anaglyph from the given stereo pair.\n"
        "You can adjust the stereoscopic effect by shifting the right image if\n"
        "shift != (0,0).\n"
        ":param left:\n"
        ":param right:\n"
        ":param shift: allows to control the stereoscopic effect\n"
        ":param input_is_rgb: true if input is RGB, false if BGR\n"
        ":return: the anaglyph",
        py::arg("left"), py::arg("right"),
        py::arg("shift") = py::make_tuple(0,0),
        py::arg("input_is_rgb") = false);


  m.def("highlight", &vcp::imvis::pseudocolor::Highlight,
        "Highlight the masked area.\n"
        "If color is negative, i.e. (-1,-1,-1), converts the unmasked region\n"
        "to grayscale (and dampens it even more); otherwise draws the masked\n"
        "region with the color at the given opacity.\n"
        "If image is floating point, ensure that it is in [0,1].\n\n"
        ":param image:\n"
        ":param mask:  single channel mask of regions to highlight\n"
        ":param color: (-1,-1,-1) or RGB/BGR tuple\n"
        ":param color_opacity: opacity\n"
        ":return: image after highlighting the region",
        py::arg("image"), py::arg("mask"),
        py::arg("color") = cv::Scalar::all(-1.0),
        py::arg("color_opacity") = 0.5);


  m.def("draw_bboxes2d__", &vpi::DrawBoundingBoxes2D,
        "C++ part of draw_bboxes2d().",
        py::arg("image"),
        py::arg("bounding_boxes"),
        py::arg("default_box_color"),
        py::arg("line_width"),
        py::arg("fill_opacity"),
        py::arg("font_color"),
        py::arg("text_anchor"),
        py::arg("font_scale"),
        py::arg("font_thickness"),
        py::arg("text_box_padding"),
        py::arg("text_box_opacity"),
        py::arg("non_overlapping"));


  m.def("draw_rotated_bboxes2d__", &vpi::DrawRotatedBoundingBoxes2D,
        "C++ part of draw_rotated_bboxes2d().",
        py::arg("image"), py::arg("bounding_boxes"),
        py::arg("default_box_color"), py::arg("line_width"), py::arg("fill_opacity"),
        py::arg("font_color"), py::arg("text_anchor"), py::arg("font_scale"),
        py::arg("font_thickness"), py::arg("text_box_padding"), py::arg("text_box_opacity"),
        py::arg("non_overlapping"), py::arg("dash_length"));


  m.def("draw_rounded_rects", &vpi::DrawRoundedRects,
        "Draw a list of rectangles with rounded corners.\n\n"
        ":param image:\n"
        ":param rects: a list of rectangles [r1, r2, ...], where each rectangle is\n"
        "      either a list, NumPy ndarray or tuple: [x,y,w,h].\n"
        ":param color: RGB or BGR tuple.\n"
        ":param corner_percentage: Corner radius will be percentage * min(W,H). Must\n"
        "      be within [0, 0.5].\n"
        ":param fill_opacity: Opacity in [0,1], fills the rectangle if > 0.\n"
        ":param line_width:   Edge width (set to 0 if you only want to fill the rect).\n"
        ":param non_overlapping: Don't draw overlapping parts (assume that rects[0]\n"
        "      is closest to the viewer, then rects[1], ...)\n"
        ":return: numpy.ndarray",
        py::arg("image"), py::arg("rects"),
        py::arg("color") = cv::Scalar(255, 0, 0),
        py::arg("corner_percentage") = 0.2,
        py::arg("fill_opacity") = 0.0,
        py::arg("line_width") = 1,
        py::arg("non_overlapping") = false);


  m.def("draw_rects", &vpi::DrawRects,
        "Draw a list of rectangles.\n\n"
        ":param image:\n"
        ":param rects: a list of rectangles [r1, r2, ...], where each rectangle is\n"
        "      either a list, NumPy ndarray or tuple: [x,y,w,h].\n"
        ":param color:        RGB or BGR tuple.\n"
        ":param fill_opacity: Opacity in [0,1], fills the rectangle if > 0.\n"
        ":param line_width:   Edge width (set to 0 if you only want to fill the rect).\n"
        ":param dash_length:  If > 0, the edges will be drawn dashed, such that\n"
        "      each dash has a length of dash_length pixels, followed by\n"
        "      dash_length unchanged pixels.\n"
        ":param non_overlapping: Don't draw overlapping parts (assume that rects[0]\n"
        "      is closest to the viewer, then rects[1], ...)\n"
        ":return: numpy.ndarray",
        py::arg("image"), py::arg("rects"),
        py::arg("color") = cv::Scalar(255, 0, 0),
        py::arg("fill_opacity") = 0.0,
        py::arg("line_width") = 1,
        py::arg("dash_length") = -1,
        py::arg("non_overlapping") = false);


  m.def("draw_rotated_rects", &vpi::DrawRotatedRects,
        "Draw a list of rotated rectangles.\n\n"
        ":param image:\n"
        ":param rects: a list of rectangles [r1, r2, ...], where each rectangle is\n"
        "        either a list, NumPy ndarray or tuple: [cx,cy,w,h,t].\n"
        "        Its center is (cx,cy), the angle t is in degrees (following\n"
        "        OpenCV's RotatedRect declaration).\n"
        ":param color:        RGB or BGR tuple.\n"
        ":param fill_opacity: Opacity in [0,1], fills the rectangle if > 0.\n"
        ":param line_width:   Edge width (set to 0 if you only want to fill the rect).\n"
        ":param dash_length:  If > 0, the edges will be drawn dashed, such that\n"
        "        each dash has a length of dash_length pixels, followed by\n"
        "        dash_length unchanged pixels.\n"
        ":param non_overlapping: Don't draw overlapping parts (assume that rects[0]\n"
        "        is closest to the viewer, then rects[1], ...)\n"
        ":return: numpy.ndarray",
        py::arg("image"), py::arg("rects"),
        py::arg("color") = cv::Scalar(255, 0, 0),
        py::arg("fill_opacity") = 0.0,
        py::arg("line_width") = 1,
        py::arg("dash_length") = -1,
        py::arg("non_overlapping") = false);


  m.def("draw_lines", &vpi::DrawLines,
        "Draws multiple lines.\n\n"
        ":param image:\n"
        ":param lines: A list of lines [l1, l2, ...], where each line is a list/tuple\n"
        "    (pt1, pt2) or (pt1, pt2, is_dashed) or (pt1, pt2, is_dashed, color).\n"
        "    pt1 and pt2 are the start and end points of the line, given as tuples\n"
        "    pt1 = (x,y).\n"
        "    is_dashed is a flag indicating whether the line should be drawn dashed\n"
        "    or solid.\n"
        "    If you provide the optional 'color', i.e. a RGB/BGR tuple, the line will\n"
        "    be drawn in this color. If not specified, the line will be drawn with the\n"
        "    'default_color'.\n"
        ":param default_color:  Default color to use if a line doesn't specify its own.\n"
        ":param line_width:     Thickness of line.\n"
        ":param dash_length:    If the list contains dashed lines, they will be drawn\n"
        "    with a dash length of dash_length pixels.\n"
        ":return: numpy.ndarray",
        py::arg("image"), py::arg("lines"),
        py::arg("default_color") = cv::Scalar(255,0,0),
        py::arg("line_width") = 1,
        py::arg("dash_length") = 10);


  m.def("draw_arrows", &vpi::DrawArrows,
        "Draws multiple arrows.\n\n"
        ":param image:\n"
        ":param arrows: A list of lines [l1, l2, ...], where each line is a list/tuple\n"
        "    (pt1, pt2) or (pt1, pt2, is_dashed) or (pt1, pt2, is_dashed, color).\n"
        "    pt1 and pt2 are the start and end points of the arrow, given as tuples\n"
        "    pt1 = (x,y).\n"
        "    is_dashed is a flag indicating whether the arrow should be drawn dashed\n"
        "    or solid.\n"
        "    If you provide the optional 'color', i.e. a RGB/BGR tuple, the arrow will\n"
        "    be drawn in this color. If not specified, the arrow will be drawn with the\n"
        "    'default_color'.\n"
        ":param default_color:  Default color to use if an arrow doesn't specify its own.\n"
        ":param line_width:     Thickness of line.\n"
        ":param arrow_head_factor: Size of the arrow head (tip, pointing to the line's end)\n"
        "                       as a fraction of the line length, must be in [0,1]."
        ":param dash_length: If the list contains dashed arrows, they will be drawn\n"
        "                    with a dash length of dash_length pixels.\n"
        ":return: numpy.ndarray",
        py::arg("image"), py::arg("arrows"),
        py::arg("default_color") = cv::Scalar(255,0,0),
        py::arg("line_width") = 1,
        py::arg("arrow_head_factor") = 0.1,
        py::arg("dash_length") = 10);


  m.def("draw_polygon", &vpi::DrawPolygon,
        "Draws a single polygon.\n\n"
        ":param image:        to draw upon\n"
        ":param polygon:      is a list of 2D points, [(x0, y0), (x1, y1), ...]\n"
        ":param color:        RGB (or BGR) tuple\n"
        ":param line_width:   draw the edges of the polygon by lines X pixels thick\n"
        ":param dash_length:  edges will be dashed if > 0\n"
        ":param fill_opacity: fills the polygon if > 0.0",
        py::arg("image"), py::arg("polygon"),
        py::arg("color") = cv::Scalar(255, 0, 255),
        py::arg("line_width") = 1,
        py::arg("dash_length") = -1,
        py::arg("fill_opacity") = 0.0);


  m.def("draw_trajectory", &vpi::DrawTrajectory,
        "Draw an object's trajectory.\n\n"
        ":param image:     to draw upon.\n\n"
        ":param positions: list of 2D points, i.e. a list, tuple or np.array: (x,y).\n"
        ":param newest_position_first: Set True if positions[0] is the most recent\n"
        "    object position.\n"
        ":param smoothing_window: -1 no smoothing, otherwise the trajectory will be\n"
        "    smoothed by a moving average. Similar to MATLAB's smooth. Must be odd.\n"
        ":param trajectory_length: -1 draws the full trajectory, shorten to the X\n"
        "    most recent points if > 0.\n"
        ":param color:       the trajectory's line color.\n"
        ":param line_width:  line thickness.\n"
        ":param dash_length: -1 solid, otherwise (approximate) length of a dash.\n",
        py::arg("image"), py::arg("positions"),
        py::arg("newest_position_first") = false,
        py::arg("smoothing_window") = -1,
        py::arg("trajectory_length") = -1,
        py::arg("color") = cv::Scalar(255, 0, 0),
        py::arg("line_width") = 3,
        py::arg("dash_length") = -1);


  m.def("draw_fading_trajectory", &vpi::DrawFadingTrajectory,
        "Nicely draw an object's trajectory with fading color.\n\n"
        ":param image:     to draw upon.\n\n"
        ":param positions: list of 2D points, i.e. a list, tuple or np.array: (x,y).\n"
        ":param newest_position_first: Set True if positions[0] is the most recent\n"
        "    object position.\n"
        ":param smoothing_window: -1 no smoothing, otherwise the trajectory will be\n"
        "    smoothed by a moving average. Similar to MATLAB's smooth. Must be odd.\n"
        ":param trajectory_length: -1 draws the full trajectory, shorten to the X\n"
        "    most recent points if > 0.\n"
        ":param obj_color:   color of the trajectory's head.\n"
        ":param fade_color:  color of the trajectory's tail.\n"
        ":param max_line_width:  line thickness at the head, must be >= 2.\n"
        ":param dash_length: -1 solid, otherwise (approximate) length of a dash.\n",
        py::arg("image"), py::arg("positions"),
        py::arg("newest_position_first") = false,
        py::arg("smoothing_window") = -1,
        py::arg("trajectory_length") = -1,
        py::arg("obj_color") = cv::Scalar(255, 0, 0),
        py::arg("fade_color") = cv::Scalar(180, 180, 180),
        py::arg("max_line_width") = 3,
        py::arg("dash_length") = -1);


  m.def("draw_text_box__", &vpi::DrawTextBox,
        "C++ part of draw_text_box().",
        py::arg("image"), py::arg("text"), py::arg("pos"), py::arg("textbox_anchor"),
        py::arg("bg_color"), py::arg("font_color"), py::arg("font_scale"),
        py::arg("font_thickness"), py::arg("padding"), py::arg("fill_opacity"));


  m.def("draw_bboxes3d__", &vpi::DrawBoundingBoxes3D,
        "C++ part of draw_bboxes3d().",
        py::arg("image"), py::arg("bounding_boxes"),
        py::arg("K"), py::arg("R"), py::arg("t"),
        py::arg("scale_image_points"), py::arg("default_box_color"),
        py::arg("line_width"), py::arg("dash_length"),
        py::arg("fill_opacity_top"), py::arg("fill_opacity_bottom"),
        py::arg("font_color"), py::arg("text_anchor"),
        py::arg("font_scale"), py::arg("font_thickness"),
        py::arg("text_box_padding"), py::arg("text_box_opacity"),
        py::arg("non_overlapping"));


  m.def("draw_points__", &vpi::DrawPoints,
        "C++ part of draw_points().",
        py::arg("image"), py::arg("points"), py::arg("color"),
        py::arg("radius"), py::arg("line_width"), py::arg("opacity"));


  m.def("draw_xyz_axes", &vpi::DrawXYZAxes,
        "Draw the coordinate system axes (+x: red-ish, +y: green, +z: blue). Check\n"
        "if one axis is actually red-ish (otherwise, you have forgotten to set\n"
        "image_is_rgb correctly)!\n\n"
        ":param image:  Input, np.array\n"
        ":param K:      3x3 camera intrinsics, np.array\n"
        ":param R:      3x3 camera rotation, np.array\n"
        ":param t:      3x1 translation (t = -RC), np.array\n"
        ":param origin: Center of the coordinate system\n"
        ":param scale_axes: How far to shift the end-points of the axis arrows, they\n"
        "    will be at origin+(scale_axes, 0, 0) etc.\n"
        ":param scale_image_points: If the image is resized (i.e. not the original\n"
        "    size which corresponds to the given intrinsics), you need to\n"
        "    provide the scaling factor as scale_image_points.\n"
        ":param line_width:   Line width in pixels.\n"
        ":param dash_length:  Arrows will be dashed if > 0.\n"
        ":param image_is_rgb: Set this to False if your image is BGR!"
        ":return: Visualization as np.array",
        py::arg("image"), py::arg("K"), py::arg("R"), py::arg("t"),
        py::arg("origin") = cv::Scalar::all(0.0),
        py::arg("scale_axes") = 1000.0,
        py::arg("scale_image_points") = 1.0,
        py::arg("line_width") = 1,
        py::arg("dash_length") = -1,
        py::arg("image_is_rgb") = true);


  m.def("draw_horizon", &vpi::DrawHorizon,
        "Draws the (slightly approximated) horizon line.\n\n"
        ":param image:  Input, np.array\n"
        ":param K:      3x3 camera intrinsics, np.array\n"
        ":param R:      3x3 camera rotation, np.array\n"
        ":param t:      3x1 translation (t = -RC), np.array\n"
        ":param color:  RGB/BGR tuple\n"
        ":param scale_image_points: If the image is resized (i.e. not the original\n"
        "    size which corresponds to the given intrinsics), you need to\n"
        "    provide the scaling factor as scale_image_points.\n"
        ":param line_width:   Line width in pixels.\n"
        ":param dash_length:  Horizon will be dashed if > 0.\n"
        ":param warn_if_not_visible: If True and the horizon is outside the\n"
        "    field-of-view, a warning will be written to std::err.\n"
        ":return: Visualization as np.array",
        py::arg("image"), py::arg("K"), py::arg("R"), py::arg("t"),
        py::arg("color") = cv::Scalar(255,0,255),
        py::arg("scale_image_points") = 1.0,
        py::arg("line_width") = 1,
        py::arg("dash_length") = -1,
        py::arg("warn_if_not_visible") = true);


  m.def("draw_groundplane_grid", &vpi::DrawGroundplaneGrid,
        "Draws the ground plane grid points. Default values assume that the camera\n"
        "calibration is provided in millimeters, thus drawing a 20x20m grid from\n"
        "(-10m,-10m) to (+10m,+10m), with a spacing of 0.5m between grid cells.\n\n"
        ":param image: Input image as np.array.\n"
        ":param K:      3x3 camera intrinsics, np.array.\n"
        ":param R:      3x3 camera rotation, np.array.\n"
        ":param t:      3x1 translation (t = -RC), np.array.\n"
        ":param grid_spacing: Size of a grid cell (length between the grid corners,\n"
        "    we use the same value for both x and y direction).\n"
        ":param grid_limits: 4x1 list/tuple/np.array constraining the ground plane\n"
        "    rectangle as (xmin, ymin, width, height).\n"
        ":param grid_origin: Center of the coordinate system as 2D tuple (x,y).\n"
        ":param scale_image_points: If the image is resized (i.e. not the original\n"
        "    size which corresponds to the given projection matrix), you need to\n"
        "    provide the scaling factor as scale_image_points.\n"
        ":param point_radius: Radius of each grid point in pixels.\n"
        ":param line_width: Line width in pixels, -1 for filled dots/circles.\n"
        ":param opacity: Opacity in [0,1].\n"
        ":return: Visualization as np.array.",
        py::arg("image"), py::arg("K"), py::arg("R"), py::arg("t"),
        py::arg("grid_spacing") = 500.0,
        py::arg("grid_limits") = cv::Rect2d(-10000.0, -10000.0, 20000.0, 20000.0),
        py::arg("grid_origin") = cv::Vec2d(0.0, 0.0),
        py::arg("scale_image_points") = 1.0,
        py::arg("point_radius") = 5,
        py::arg("line_width") = 1,
        py::arg("opacity") = 1.0);


  m.def("draw_error_ellipse_90", &vpi::DrawErrorEllipse90,
        "Draws an error ellipse at 90 % confidence interval for 2D data.\n"
        ":param image:\n"
        ":param mean: 2d center (x,y) of the data samples\n"
        ":param cov_mat: 2x2 covariance matrix (np.array)\n"
        ":param color: RGB/BGR tuple\n"
        ":param line_width: int\n"
        ":param fill_opacity: double in [0,1]\n"
        ":return: Image with visualized error ellipse.",
        py::arg("image"), py::arg("mean"), py::arg("cov_mat"),
        py::arg("color")=cv::Scalar(255, 0, 0),
        py::arg("line_width")=1,
        py::arg("fill_opacity")=0.0);
  m.def("draw_error_ellipse_95", &vpi::DrawErrorEllipse95,
        "Draws an error ellipse at 95 % confidence interval for 2D data.\n"
        ":param image:\n"
        ":param mean: 2d center (x,y) of the data samples\n"
        ":param cov_mat: 2x2 covariance matrix (np.array)\n"
        ":param color: RGB/BGR tuple\n"
        ":param line_width: int\n"
        ":param fill_opacity: double in [0,1]\n"
        ":return: Image with visualized error ellipse.",
        py::arg("image"), py::arg("mean"), py::arg("cov_mat"),
        py::arg("color")=cv::Scalar(255, 0, 0),
        py::arg("line_width")=1,
        py::arg("fill_opacity")=0.0);
  m.def("draw_error_ellipse_99", &vpi::DrawErrorEllipse99,
        "Draws an error ellipse at 99 % confidence interval for 2D data.\n"
        ":param image:\n"
        ":param mean: 2d center (x,y) of the data samples\n"
        ":param cov_mat: 2x2 covariance matrix (np.array)\n"
        ":param color: RGB/BGR tuple\n"
        ":param line_width: int\n"
        ":param fill_opacity: double in [0,1]\n"
        ":return: Image with visualized error ellipse.",
        py::arg("image"), py::arg("mean"), py::arg("cov_mat"),
        py::arg("color")=cv::Scalar(255, 0, 0),
        py::arg("line_width")=1,
        py::arg("fill_opacity")=0.0);
}
