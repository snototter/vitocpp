#ifndef __VCP_MATH_GEOMETRY2D_H__
#define __VCP_MATH_GEOMETRY2D_H__

#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <vcp_imutils/opencv_compatibility.h>

// TODO line-line distance, math.stackexchange.com/questions/2213165/find-shortest-distance-between-lines-in-3d
namespace vcp
{
namespace math
{
/** @brief 2D geometry-related math. */
namespace geo2d
{
/** @brief Capsulates lines and line segments. */
class Line2d
{
public:
  Line2d() : empty_(true), pt_from_(), pt_to_() {}

  /** @brief Construct a line from 2 real valued points. In case of a segment, these are the start and end points. */
  Line2d(const cv::Vec2d &from, const cv::Vec2d &to) : empty_(false), pt_from_(from), pt_to_(to) {}

  /** @brief Construct a line from 2 integer points. In case of a segment, these are the start and end points. */
  Line2d(const cv::Point &from, const cv::Point &to) : empty_(false), pt_from_(from.x, from.y), pt_to_(to.x, to.y) {}

  /** @brief Returns a line with flipped direction vector. */
  Line2d Flipped() const;

  /** @brief For a segment, this returns the start point. For a line its one of the given points to construct the line in the first place. */
  const cv::Vec2d &From() const;

  /** @brief For a segment, this returns the end point. For a line its one of the given points to construct the line in the first place. */
  const cv::Vec2d &To() const;

  /** @brief Returns the direction vector (not normalized!) from the start point to the end point. */
  cv::Vec2d Direction() const;

  /** @brief Returns the unit direction vector from the start point to the end point. */
  cv::Vec2d UnitDirection() const;

  /** @brief Returns the point halfway between from and to. */
  cv::Vec2d MidPoint() const;

  /** @brief Returns true, if the line object is empty, i.e. not set, or a single point (to == from). */
  bool empty() const;

  /** @brief Returns the angle between the line and v. */
  double Angle(const cv::Vec2d &v) const;

  /** @brief Return the line in homogeneous coordinates (3-element vector in P^2). */
  cv::Vec3d HomogeneousForm() const;

  friend std::ostream &operator<< (std::ostream &stream, const Line2d &line);
private:
  bool empty_;
  cv::Vec2d pt_from_;
  cv::Vec2d pt_to_;
};


/** @brief Returns the direction vector from 'from' to 'to'. */
inline cv::Vec2d DirectionVector(const cv::Vec2d &from, const cv::Vec2d &to)
{
  return to - from;
}


/** @brief Returns the squared length of the vector. */
inline double LengthSquared(const cv::Vec2d &v)
{
  return v[0]*v[0] + v[1]*v[1];
}


inline long LengthSquared(const cv::Point &p)
{
  const long x = static_cast<long>(p.x);
  const long y = static_cast<long>(p.y);
  return x*x + y*y;
}


/** @brief Returns the length of the vector. */
inline double Length(const cv::Vec2d &v)
{
  return std::sqrt(LengthSquared(v));
}


/** @brief Returns the Euclidean distance between the two vectors. */
inline double Distance(const cv::Vec2d &v, const cv::Vec2d &w)
{
  return Length(v-w);
}


/** @brief Returns the unit vector and optionally sets the length to the original vector magnitude (before normalization). */
inline cv::Vec2d Normalize(const cv::Vec2d &v, double *length=nullptr)
{
  const double len = Length(v);
  if (length)
    *length = len;

  if (len > 0.0)
    return cv::Vec2d(v[0]/len, v[1]/len);
  else
    return v;
}


/** @brief Dot product. */
inline double Dot(const cv::Vec2d &v, const cv::Vec2d &w)
{
  return v[0]*w[0] + v[1]*w[1];
}


/** @brief 2D cross product, see http://mathworld.wolfram.com/CrossProduct.html */
inline double Cross(const cv::Vec2d &v, const cv::Vec2d &w)
{
  return v[0]*w[1] - v[1]*w[0];
}

/** @brief Project point onto line. */
cv::Vec2d ProjectPointOntoLine(const cv::Vec2d &point, const Line2d &line);


/** @brief Get closest point on line segment, i.e. either the point's projection or the start/end point. */
cv::Vec2d ClosestPointOnLineSegment(const cv::Vec2d &point, const Line2d &segment);


/** @brief Distance between point and line. */
double DistancePointLine(const cv::Vec2d &point, const Line2d &line);


/** @brief Distance between point and line segment. */
double DistancePointLineSegment(const cv::Vec2d &point, const Line2d &line_segment);


/** @brief Returns the vector projection of vector a onto b. */
cv::Vec2d VectorProjection(const cv::Vec2d &a, const cv::Vec2d &b);


/** @brief Returns the scalar projection of vector a onto b, i.e. the component of a in the b direction (or the magnitude of the vector projection of a onto b). */
double ScalarProjection(const cv::Vec2d &a, const cv::Vec2d &b);


/** @brief Returns true if the point is left of the line line.from--line.to. */
bool IsPointLeftOfLine(const cv::Vec2d &pt, const Line2d &line, bool *is_on_line=nullptr);


/** @brief Rotates the vector by the given radians, assuming a right-handed coordinate system! */
cv::Vec2d RotateVector(const cv::Vec2d &vec, double theta);


/** @brief Rotates the vector by the given radians about the given rotation center, assuming a right-handed coordinate system! */
cv::Vec2d RotateVector(const cv::Vec2d &vec, const cv::Vec2d &rotation_center, double theta);

// Python bindings cannot resolve overloaded functions.
cv::Vec2d RotateVectorPy(const cv::Vec2d &vec, const cv::Vec2d &rotation_center, double theta);


/** @brief Performs RDP line simplification, returns the reduced point set. */
std::vector<cv::Vec2d> SimplifyRamerDouglasPeucker(const std::vector<cv::Vec2d> &points, double epsilon);


/** @brief Performs RDP line simplification, returns the indices to get the reduced point set. */
std::vector<size_t> SimplifyRamerDouglasPeuckerIndices(const std::vector<cv::Vec2d> &points, double epsilon);


/** @brief Removes duplicate points along the trajectory (i.e. ensures that traj[i] != traj[i+1] afterwards). */
std::vector<cv::Point> RemoveDuplicatePoints(const std::vector<cv::Point> &trajectory);


/** @brief Returns the Bresenham line between the two points. */
std::vector<cv::Point> BresenhamLine(const cv::Point &from, const cv::Point &to);


/** @brief Computes Bresenham lines for each segment of the given trajectory. */
std::vector<cv::Point> BresenhamTrajectory(const std::vector<cv::Point> &trajectory);


/** @brief Smoothes the given trajectory, similar to MATLAB's smooth. */
std::vector<cv::Vec2d> SmoothTrajectory(const std::vector<cv::Vec2d> &positions, int smoothing_window);


/** @brief Checks if a point is inside (or on the boundary of) the given closed polygon.
 *
 * If the polygon is not closed it will automatically be closed ([poly[0],...,poly[end], poly[0])
 */
bool IsPointInClosedPolygon(const cv::Vec2d &point, const std::vector<cv::Vec2d> &polygon);


/** @brief Computes the signed distance between the point and a (closed) polygon.
  * Returns result <= 0 if the point is inside (or on the polygon), result > 0 otherwise.
  *
  * If the polygon is not closed (i.e. polygon[end] != polygon[0]), the polygon will be
  * closed automatically (by adding a point).
  */
double DistancePointClosedPolygon(const cv::Vec2d &point, const std::vector<cv::Vec2d> &polygon);


/** @brief Checks if the point is inside the given rectangle. */
bool IsPointInsideRectangle(const cv::Vec2d &pt, const cv::Rect2d &rect);


/** @brief Returns the axis-aligned bounding box of the given polygon. */
cv::Rect2d AxisAlignedBoundingRect(const std::vector<cv::Vec2d> &polygon);


/** @brief Rotating calipers (TODO maybe implement custom version able to work with floats) CURRENTLY, converting... TODO doc */
cv::RotatedRect RotatedBoundingRect(const std::vector<cv::Vec2d> &polygon);


/** @brief Checks if two lines are collinear. */
bool IsCollinear(const Line2d &a, const Line2d &b);


/** @brief Checks if the lines intersect and sets the intersection point. */
bool IntersectionLineLine(const Line2d &a, const Line2d &b, cv::Vec2d &intersection);


/** @brief Checks if the line intersects the line segment and sets the intersection point. */
bool IntersectionLineLineSegment(const Line2d &line, const Line2d &segment, cv::Vec2d &intersection);


/** @brief Checks if the line segments intersect and sets the intersection point. */
bool IntersectionLineSegmentLineSegment(const Line2d &a, const Line2d &b, cv::Vec2d &intersection);


/** @brief Checks if the line intersects the circle and sets intersection points accordingly.
 * @return Number of intersection points, i.e. 0, 1, or 2.
 */
int IntersectionLineCircle(const Line2d &a, const cv::Vec2d &center, double radius, cv::Vec2d &intersection1, cv::Vec2d &intersection2);
// Collision detection via projection: https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm

int IntersectionLineSegmentCircle(const Line2d &a, const cv::Vec2d &center, double radius, cv::Vec2d &intersection1, cv::Vec2d &intersection2);

bool CircleFrom3Points(const cv::Vec2d &pt1, const cv::Vec2d &pt2, const cv::Vec2d &pt3, cv::Vec2d &center, double &radius);

/** @brief Clips the line such that only the part within the rectangle remains. Check result.empty() afterwards. */
Line2d ClipLineByRectangle(const Line2d &line, const cv::Rect2d &rect);
Line2d ClipLineByRectangle(const Line2d &line, const cv::Rect &rect);

// Python bindings don't play nicely with overloaded functions.
Line2d ClipLineByRectanglePy(const Line2d &line, const cv::Rect2d &rect);


/** @brief Clips the line segment such that only the part within the rectangle remains. Check for result.empty() afterwards. */
Line2d ClipLineSegmentByRectangle(const Line2d &segment, const cv::Rect2d &rect);
Line2d ClipLineSegmentByRectangle(const Line2d &segment, const cv::Rect &rect);

// Python bindings don't play nicely with overloaded functions.
Line2d ClipLineSegmentByRectanglePy(const Line2d &segment, const cv::Rect2d &rect);


/** @brief Returns a the projected line of horizon for the given camera intrinsics/extrinsics. If a valid image size
  * is given, the line will be clipped to the visible region (in this case, check result.empty() as the horizon may lie
  * outside of the image.
  */
Line2d GetProjectionOfHorizon(const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, const cv::Size &image_size=cv::Size());


/** @brief Computes the convex hull for the given point set.
 * If you are 100% sure that there are no duplicates, disable the duplicate check (for negligible speedup).
 */
std::vector<cv::Point> ConvexHullGrahamScan(const std::vector<cv::Point> &points, bool check_for_duplicates=true);


/** @brief Convenience wrapper to @see ConvexHullGrahamScan. Vec2d will be scaled and casted (truncated) to integer points.
 *
 * pts_for_convex_hull[i] = (int)(points[i] * scale)
 * returned hull: vec2d(pt)/scale
*/
std::vector<cv::Vec2d> ConvexHullGrahamScan(const std::vector<cv::Vec2d> &points, bool check_for_duplicates=true, double scale=1.0);


/** @brief Rotates the given vectors about the rotation center by theta radians.*/
std::vector<cv::Vec2d> RotateVecs(const std::vector<cv::Vec2d> &pts, const cv::Vec2d &rotation_center, double theta);


/** @brief IOU for axis-aligned rectangles. */
double IntersectionOverUnionRects(const cv::Rect &b1, const cv::Rect &b2);


/** @brief IOU for rotated rectangles. */
double IntersectionOverUnionRotatedRects(const cv::RotatedRect &r1, const cv::RotatedRect &r2);


/** @brief Checks if the points are ordered cw or ccw. */
bool IsPolygonClockwise(const std::vector<cv::Vec2d> &polygon);


/** @brief Returns the intersection of the two convex polygons (or empty). */
std::vector<cv::Vec2d> IntersectionConvexPolygons(const std::vector<cv::Vec2d> &subject, const std::vector<cv::Vec2d> &clip);


/** @brief Triangle area from its three corners. */
double AreaTriangle(const cv::Vec2d &a, const cv::Vec2d &b, const cv::Vec2d &c);


/** @brief Rotated rect area from its corners. */
double AreaRotatedRect(const cv::Point2f *vertices);


/** @brief Rotated rect area. */
double AreaRotatedRect(const cv::RotatedRect &r);


/** @brief Rotated rect area for python bindings (overloaded methods don't play well with automatic type casting...). */
double AreaRotatedRectPy(const cv::RotatedRect &r);


/** @brief Area of a polygon - may fail for self-crossing, twisted (and other weird) polygons. */
double AreaPolygon(const std::vector<cv::Vec2d> &pts);


/** @brief Checks if given point lies within (or exactly on) the circle.
 * @return False, if the point lies outside the circle. True otherwise.
 */
bool IsPointInCircle(const cv::Vec2d &pt, const cv::Vec2d &center, double radius, bool *is_on_circle=nullptr);


/** Computes the point(s) of tangency between a point and the circle. Returns true if at least one tangent exists. */
bool PointsOfTangencyPointToCircle(const cv::Vec2d &pt, const cv::Vec2d &center, double radius, cv::Vec2d &pot1, cv::Vec2d &pot2);


/** @brief Compute the transverse common tangents (german innere Tangenten) between two circles.
  * @return Number of tangents: 0 (circles overlap), 1 (circles intersect in exactly one point), 2 (circles don't touch)
  * Note that for the special case of 1 tangent, the point of tangency has to be computed (midpoint of the computed tangent1).
  */
int TransverseCommonTangentsBetweenCircles(const cv::Vec2d &center1, double radius1, const cv::Vec2d &center2, double radius2, Line2d &tangent1, Line2d &tangent2);

// Note that for the special case of 1 tangent, the point of tangency has to be computed (midpoint of the computed tangent1).
int DirectCommonTangentsBetweenCircles(const cv::Vec2d &center1, double radius1, const cv::Vec2d &center2, double radius2, Line2d &tangent1, Line2d &tangent2);

} // namespace geometry3d
} // namespace math
} // namespace vcp

#endif // __VCP_MATH_GEOMETRY2D_H__
