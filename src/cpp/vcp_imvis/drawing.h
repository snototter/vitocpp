#ifndef __VCP_IMVIS_DRAWING_H__
#define __VCP_IMVIS_DRAWING_H__

#include <vector>
#include <string>
#include <vcp_imutils/opencv_compatibility.h>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/imgproc/imgproc.hpp>
#else
    #include <opencv2/imgproc.hpp>
#endif

namespace vcp
{
/** @brief Visualization utilities. */
namespace imvis
{
/** @brief Drawing primitives and commonly used shapes (e.g. 2D/3D bounding boxes). */
namespace drawing
{
/** @brief Exemplary colors. */
extern const cv::Scalar kExemplaryColors[];

/** @brief Size of the exemplary colors array. */
extern const size_t kExemplaryColorsSize;

/** @brief Return an exemplary color, avoids out-of-bounds errors. */
cv::Scalar GetExemplaryColor(size_t idx);

/** @brief Colors to visualize coordinate axes. */
extern const cv::Scalar kAxisColorsRGB[3];
extern const cv::Scalar kAxisColorsBGR[3];


namespace textanchor
{

/** @brief Supported anchor definitions for positioning text and text boxes. Force them to be int, so we can perform bit-wise operations. */
enum TextAnchor : int
{
  // Do NOT change these values, unless you know what you're doing.
  LEFT = 1,
  HCENTER = 2,
  RIGHT = 4,
  TOP = 8,
  VCENTER = 16,
  BOTTOM = 32
};
} // namespace textanchor


/** @brief Computes where to put a text (box) within the rectangle w.r.t. the text anchor. */
cv::Point GetTextPos(const cv::Rect &rect, int text_anchor=textanchor::TOP | textanchor::LEFT);
cv::Point GetTextPos(const cv::RotatedRect &rect, int text_anchor=textanchor::TOP | textanchor::LEFT);


/** @brief Get a readable representation of a text anchor value. */
std::string TextAnchorToString(int text_anchor);


/** @brief Draw text on top of a (potentially transparent) filled rectangle. */
void DrawTextBox(cv::Mat &image, const std::string &text, const cv::Point &pos, int anchor=textanchor::TOP | textanchor::LEFT, int padding=0,
                 double fill_opacity=0.7, const cv::Scalar &font_color=cv::Scalar(255.0,0.0,0.0), const cv::Scalar &bg_color=cv::Scalar::all(0.0),
                 int font_face=cv::FONT_HERSHEY_PLAIN, double font_scale=1.0, int thickness=1, cv::Rect *image_roi=nullptr);


/** @brief Computes the complementary color (opposite on the HSL/HSV color wheel). */
cv::Scalar ComplementaryColor(const cv::Scalar &color, bool is_rgb);

cv::Scalar InvertColorRGB(const cv::Scalar &color);


/** @brief Draw a rectangle with rounded corners. Radius is given by corner_percentage in [0,0.5] of the shorter dimension (width or height).
 * You'll get visually nice rects if you use radius 0.25. */
void DrawRoundedRect(cv::Mat &image, const cv::Rect &rect, const cv::Scalar &color, double corner_percentage, double fill_opacity=0.0, int line_width=1);


/** @brief Draw rectangles with rounded corners, @see DrawRoundedRect.
 * If non_overlapping is true, overlapping rects won't be drawn (rects[0] will be fully visible, rects[1] will only drawn outside rects[0]'s boundaries, ...).
 */
void DrawRoundedRects(cv::Mat &image, const std::vector<cv::Rect> &rects, const cv::Scalar &color, double corner_percentage, double fill_opacity=0.0, int line_width=1, bool non_overlapping=false);


/** @brief Draw a (potentially filled) rectangle.
 * Examples:
 * * Draw a transparent patch without edges: fill_opacity > 0, line_width = 0
 * * Draw a transparent patch with edges: fill_opacity > 0, line_width > 0
 * * Draw only the edges: fill_opacity = 0.0, line_width > 0
 * * Draw a transparent patch with dashed edges: fill_opacity > 0, line_width > 0, dash_length > 0
 * * Draw only dashed edges: fill_opacity = 0.0, line_width > 0, dash_length > 0
 */
void DrawRect(cv::Mat &image, const cv::Rect &rect, const cv::Scalar &color, double fill_opacity=0.0, int line_width=1, int dash_length=-1);


/** @brief Draws multiple rectangles - @see DrawRect() for parameters and usage.
 * If non_overlapping is true, overlapping rects won't be drawn (rects[0] will be fully visible, rects[1] will only drawn outside rects[0]'s boundaries, ...).
 */
void DrawRects(cv::Mat &image, const std::vector<cv::Rect> &rects, const cv::Scalar &color, double fill_opacity=0.0, int line_width=1, int dash_length=-1, bool non_overlapping=false);


/** @brief Draws 2D bounding boxes (i.e. rects with a caption).
 *
 * Caption will be placed inside a text box according to the text_anchor.
 * If text_box_color is empty, the bounding box color will be used.
 * If font_color is empty, the complementary color to the text_box_color will be used.
 * If a caption string is empty, no label will be placed.
 * If dash_lengths is empty, all boxes will be drawn solid.
 * If non_overlapping is true, occluded parts of boxes won't be drawn (assuming that rects[0] contains the box closest to the viewer). -- Watch out: this is way slower than the non-occluding version!
 */
void DrawBoundingBoxes2d(cv::Mat &image, const std::vector<cv::Rect> &rects, const std::vector<cv::Scalar> &rect_colors, const std::vector<std::string> &captions,
                         bool non_overlapping=false, int line_width=1, double fill_opacity=0.0, const std::vector<int> &dash_lengths=std::vector<int>(),
                         int text_anchor=textanchor::LEFT | textanchor::TOP, const std::vector<cv::Scalar> &font_colors = std::vector<cv::Scalar>(),
                         const std::vector<cv::Scalar> &text_box_colors = std::vector<cv::Scalar>(), int text_box_padding=5, double text_box_opacity=0.7,
                         int font_face=cv::FONT_HERSHEY_PLAIN, double font_scale=1.0, int font_thickness=1);


/** @brief Draws rotated 2D bounding boxes. @see DrawBoundingBoxes2d for documentation. */
void DrawRotatedBoundingBoxes2d(cv::Mat &image, const std::vector<cv::RotatedRect> &rects, const std::vector<cv::Scalar> &rect_colors, const std::vector<std::string> &captions,
                         bool non_overlapping=false, int line_width=1, double fill_opacity=0.0, const std::vector<int> &dash_lengths=std::vector<int>(),
                         int text_anchor=textanchor::LEFT | textanchor::TOP, const std::vector<cv::Scalar> &font_colors = std::vector<cv::Scalar>(),
                         const std::vector<cv::Scalar> &text_box_colors = std::vector<cv::Scalar>(), int text_box_padding=5, double text_box_opacity=0.7,
                         int font_face=cv::FONT_HERSHEY_PLAIN, double font_scale=1.0, int font_thickness=1);


/** @brief Draw a rectangle with dashed lines, @see DrawDashedLine for dash_length parameter. */
void DrawDashedRect(cv::Mat &image, const cv::Rect &rect, const cv::Scalar &color, int dash_length=10, int line_width=1);


/** @brief Draw a rotated rectangle with dashed lines, @see DrawDashedLine for dash_length parameter. */
void DrawRotatedRect(cv::Mat &image, const cv::RotatedRect &rect, const cv::Scalar &color, double fill_opacity, int line_width, int dash_length);


/** @brief Draw a rotated rectangle with dashed lines, @see DrawDashedLine for dash_length parameter. */
void DrawDashedRotatedRect(cv::Mat &image, const cv::RotatedRect &rect, const cv::Scalar &color, int dash_length, int line_width);


/** @brief Draw rotated rectangles @see DrawRotatedRect().
 * If non_overlapping is true, overlapping rects won't be drawn (rects[0] will be fully visible, rects[1] will only drawn outside rects[0]'s boundaries, ...).
 */
void DrawRotatedRects(cv::Mat &image, const std::vector<cv::RotatedRect> &rects, const cv::Scalar &color, double fill_opacity, int line_width, int dash_length, bool non_overlapping=false);


/** @brief Draws a dashed line between the given points, each dash will be dash_length pixels long and afterwards, dash_length pixels will be left blank.
 */
void DrawDashedLine(const cv::Point &pt1, const cv::Point &pt2, const cv::Scalar &color, int dash_length, int thickness, cv::Mat &image);


/** @brief Draw multiple lines - you can mix solid and dashed.
 *
 * Draws N lines where N = start.size() = end.size().
 *
 * Examples:
 * * All solid: dash_lengths is empty, or all N entries are < 0
 * * Some solid, some dashed: dash_lengths has N entries, where dash_lenghts[i] < 0 for solid
 *   lines and > 0 for dashed, @see DrawDashedLine for explanation of dash length.
 */
void DrawLines(cv::Mat &image, const std::vector<cv::Point> &start, const std::vector<cv::Point> &end,
               const cv::Scalar &color, int line_width=1, const std::vector<int> &dash_lengths=std::vector<int>());

/** @brief Same as @see DrawLines() but supports coloring each line differently! */
void DrawLines(cv::Mat &image, const std::vector<cv::Point> &start, const std::vector<cv::Point> &end,
               const std::vector<cv::Scalar> &colors, int line_width=1, const std::vector<int> &dash_lengths=std::vector<int>());


/** @brief Clips the (infinite) line going through the reference points by the image boundaries before drawing. */
void DrawInfiniteLine(cv::Mat &image, const cv::Point &pt1, const cv::Point &pt2, const cv::Scalar &color, int line_width=1, int dash_length=-1);


/** @brief Draws a polygon. */
void DrawPolygon2d(cv::Mat &image, const std::vector<cv::Point> &polygon, const cv::Scalar &color, int line_width=1, int dash_length=-1, double fill_opacity=0.0);


/** @brief Projects the line into the image using the projection matrix P.
 * If the image is resized (i.e. not the original size which corresponds to the given projection matrix),
 * you need to provide the scaling factor as scale_image_points.
 */
void DrawLineSegment3d(cv::Mat &image, const cv::Vec3d &pt1, const cv::Vec3d &pt2, const cv::Mat &P, const cv::Scalar &color, int line_width=1, int dash_length=-1, double scale_image_points=1.0, cv::Point *prj1=nullptr, cv::Point *prj2=nullptr);


/** @brief Projects the points into the image using the projection matrix P (either 3x3 or 3x4) and draws the polygon. */
void DrawPolygon3d(cv::Mat &image, const std::vector<cv::Vec3d> &pts, const cv::Mat &P, const cv::Scalar &color, double fill_opacity=0.0, int line_width=1, int dash_length=-1, double scale_image_points=1.0);


/** @brief Draws an arrow (can be dashed, too).
 * @param thickness: line width.
 * @param tip_length: length of the arrow tip as a fraction [0,1] of its length.
 * @param dash_length: If > 0, the line will be dashed with each dash (approximately) this length, if < 0 the arrow will be solid.
 */
void DrawArrow(const cv::Point &pt1, const cv::Point &pt2, const cv::Scalar &color, cv::Mat &image, int thickness, double tip_length=0.1, int dash_length=-1);


/** @brief Draw the given points onto an image, potentially transparent.
 *
 * Line width in pixels or -1 if you want it filled.
 * Alpha = 0 (not visible), alpha=1 (fully opaque).
 */
void DrawPoints(cv::Mat &image, const std::vector<cv::Point> &points, const std::vector<cv::Scalar> &colors, int radius=5, int line_width=-1, double alpha=1.0);


/** @brief Convenience function to DrawPoints(...) which assigns a single color (if color is a valid cv::Scalar) or alternating colors from kExemplaryColors (if color=(-1,-1,-1)).
 * flip_color_channels: if input image is RGB (instead of BGR), you can also adjust the exemplary colors by setting the flip flag to true (since they are defined as RGB).
 */
void DrawPoints(cv::Mat &image, const std::vector<cv::Point> &points, const cv::Scalar &color=cv::Scalar::all(-1.0), int radius=5, int line_width=-1, double alpha=1.0, bool flip_color_channels=false);


/** @brief Draw x/+ marks at the given points, potentially transparent.
 *
 * Use the convenience overload which assigns a single (or alternating) color(s).
 * Provide length of diagonal (in pixels).
 * Provide line width in pixels.
 * Dash length > 0 to draw crosses dashed.
 * Set vertical true to draw "+", otherwise "x".
 * Alpha = 0 (not visible), alpha=1 (fully opaque).
 */
void DrawCrosses(cv::Mat &image, const std::vector<cv::Point> &points, const std::vector<cv::Scalar> &colors, int diagonal=15, int line_width=1, int dash_length=0, double alpha=1.0, bool vertical=true);


/** @brief Convenience function to DrawCrosses(...) which assigns a single color (if color is a valid cv::Scalar) or alternating colors from kExemplaryColors (if color=(-1,-1,-1)).
 * flip_color_channels: if input image is RGB (instead of BGR), you can also adjust the exemplary colors by setting the flip flag to true (since they are defined as RGB).
 */
void DrawCrosses(cv::Mat &image, const std::vector<cv::Point> &points, const cv::Scalar &color=cv::Scalar::all(-1.0), int diagonal=15, int line_width=1, int dash_length=0, double alpha=1.0, bool vertical=true, bool flip_color_channels=false);


/** @brief Draws the given circles.
 * If thickness < 0, the circle will be filled. Otherwise the contour will be
 * drawn with this line width.
 * If colors is empty the circle(s) will be drawn in "default_color".
 */
void DrawCircles(cv::Mat &image, const std::vector<cv::Point> &centers, const std::vector<int> &radii,
                 const std::vector<cv::Scalar> &colors=std::vector<cv::Scalar>(),
                 const cv::Scalar &default_color=cv::Scalar(255, 0, 255),
                 int thickness=2, int line_type=cv::LINE_8);


/** @brief Draws an ellipse.
 *
 * * If line_width > 0: draws a fully opaque edge.
 * * If fill_opacity > 0: fills the ellipse (partially opaque; fill_opacity must be in [0,1]).
 */
void DrawEllipse(cv::Mat &image, const cv::RotatedRect &rect, const cv::Scalar &color, int line_width=1, double fill_opacity=0.0);


/** @brief Draws the coordinate system's axes (+x: red-ish, +y: green, +z: blue).
 *
 * * Draws arrows from the origin to (scale_axes,0,0), (0,scale_axes,0), ...
 * * If the image is resized (i.e. not the original size which corresponds to the given projection matrix),
 *   you need to provide the scaling factor as scale_image_points.
 * * Set image_is_rgb if your image is RGB (instead of BGR).
 * * If dash length > 0, the arrows will be drawn dashed.
 */
void DrawXYZAxes(cv::Mat &image, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, const cv::Vec3d &origin=cv::Vec3d(0.0, 0.0, 0.0),
              double scale_axes=1.0, double scale_image_points=1.0, int line_width=2, int dash_length=-1, bool image_is_rgb=false, double tip_length=0.1);


/** @brief Draws the horizon line onto the image.
 * If the image is scaled (i.e. not the original size which corresponds to the given K), adjust the scale_image_points parameter!
 */
void DrawHorizon(cv::Mat &image, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, const cv::Scalar &color,
                 double scale_image_points=1.0, int line_width=2, int dash_length=-1, bool warn_if_not_visible=false,
                 double text_opacity=0.7);


/** @brief Draws a ground plane grid (allows you to check/visualize your camera2world calibrations).
 * The grid will be drawn such that the it is aligned with the origin (0,0,0), even if you provide "funny" grid_limits.
 * 1D example:
 *   x_limit_min = 23, x_limit_max = 49
 *   grid_spacing = 5
 *   will draw the points at x = { 25, 30, 35, 40, 45 }
 * See our python projects (e.g. multicam-mayhem) for usage (of the python wrapper).
 */
void DrawGroundplaneGrid(cv::Mat &image, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, double grid_spacing=1000.0,
                         const cv::Rect2d &grid_limits=cv::Rect2d(-10000.0, -10000.0, 20000.0, 20000.0),
                         const cv::Vec2d &grid_origin=cv::Vec2d(0.0, 0.0), double scale_image_points=1.0,
                         int point_radius=5, int point_thickness=-1, int line_thickness=1, double opacity=1.0, bool flip_color_channels=false);


/** @brief Represents a 3D bounding box (8 vertices, 4 on top, 4 on the bottom plane). */
class Box3d
{
public:
  Box3d() {}
  Box3d(const Box3d &other);
  virtual ~Box3d() {}

  /** @brief Adds a point to the top corners. */
  void AddTopCorner(const cv::Vec3d &v);

  /** @brief Adds a point to the bottom corners. */
  void AddBottomCorner(const cv::Vec3d &v);

  /** @brief Returns the centroid of the box. */
  cv::Vec3d Centroid() const;

  /** @brief A valid box must have 4 top and 4 bottom corners. */
  bool Valid() const { return top_corners_.size() == 4 && bottom_corners_.size() == 4; }

  /** @brief Provides access to the top corners. */
  const std::vector<cv::Vec3d> &TopCorners() const { return top_corners_; }

  /** @brief Provides access to the bottom corners. */
  const std::vector<cv::Vec3d> &BottomCorners() const { return bottom_corners_; }

private:
  std::vector<cv::Vec3d> top_corners_;
  std::vector<cv::Vec3d> bottom_corners_;
};


/** @brief Clips the given 3d box by the image plane, projects it into the view and draws it.
 *
 * * Assumes that the (extrinsic) calibration is in [mm]! Otherwise, the image plane (i.e. clipping plane) will be too far into the field-of-view!
 * * Visible edges are solid, hidden edges dashed.
 * * If the box gets clipped by the image plane, the clipping polygon will be visualized with the complementary color.
 * * Use top/bottom fill opacity to get slightly transparent top/bottom planes (so you can easily spot the up/down orientation of the box).
 * * See examples/python/vis_demo.py for how to use it to render 3D bounding boxes (with box labels).
 * * If the "text_pos" parameter is set, we try to find the "best box wall" to put text on (in front of the image
 *   plane, similar plane normal as the image plane) w.r.t. the given "text_anchor" choice.
 *   However, there are some edge cases (fancy flipped 3D boxes which intersect the image plane) where the text position won't match
 *   the text anchor exactly (it will still be placed as close to the anchor as possible). After all, the VCP library is not
 *   meant to be a rendering engine :-p
 */
bool DrawBox3d(cv::Mat &image, const Box3d &box, const cv::Mat &K, const cv::Mat &Rt, const cv::Scalar &color,
               cv::Rect *bounding_rect_box=nullptr, cv::Point *text_pos=nullptr, std::vector<cv::Point> *convex_hull=nullptr,
               int text_anchor=textanchor::LEFT | textanchor::TOP,
               int line_width=1, double scale_image_points=1.0, int dash_length=10, double top_fill_opacity=0.2, double bottom_fill_opacity=0.5);


/** @brief Draw labelled 3D boxes, @see DrawBoundingBoxes2d for default behavior when providing empty font_colors, text_box_colors, etc. */
void DrawBoundingBoxes3d(cv::Mat &image, const cv::Mat &K, const cv::Mat &Rt, const std::vector<Box3d> &boxes,
                         const std::vector<cv::Scalar> &box_colors, const std::vector<std::string> &captions,
                         bool non_overlapping=false,
                         int line_width=1, int dash_length=10, double scale_projected_image_points=1.0,
                         double fill_top_opacity=0.2, double fill_bottom_opacity=0.5,
                         int text_anchor=textanchor::LEFT | textanchor::TOP, int text_box_padding=5,
                         double text_box_opacity=0.7, const std::vector<cv::Scalar> &font_colors=std::vector<cv::Scalar>(),
                         const std::vector<cv::Scalar> &text_box_colors=std::vector<cv::Scalar>(),
                         int font_face=cv::FONT_HERSHEY_PLAIN, double font_scale=1.0, int font_thickness=1);


} // namespace drawing
} // namespace imvis
} // namespace vcp
#endif // __VCP_IMVIS_DRAWING_H__
