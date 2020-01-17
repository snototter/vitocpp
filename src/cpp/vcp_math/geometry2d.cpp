#include "geometry2d.h"
#include "geometry3d.h"
#include "common.h"
#include "conversions.h"
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
#include "../vcp_imvis/trajectories.h"
#include <algorithm>
#include <vector>
#include <iterator>
#include <opencv2/imgproc/imgproc.hpp>


#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::math::geo2d"

namespace vcp
{
namespace math
{
namespace geo2d
{
namespace utils
{

/** @brief Three points are a counter-clockwise turn if CCW() > 0, clockwise if CCW() < 0, and collinear else.
 *
 * CCW is a determinant that gives twice the signed area of the triangle formed by p1, p2, p3.
 * See pseudocode from Wiki https://en.wikipedia.org/wiki/Graham_scan
 */
inline int CounterClockWise(const cv::Vec2d &p1, const cv::Vec2d &p2, const cv::Vec2d &p3)
{
  // Cross product of the vectors (p2-p1) and (p3-p1) yields the signed area:
  const double area_rect = (p2[0] - p1[0])*(p3[1] - p1[1]) - (p2[1] - p1[1])*(p3[0] - p1[0]);
  if (vcp::math::eps_zero(area_rect))
    return 0;
  if (area_rect > 0.0)
    return +1;
  return -1;
}

inline int CounterClockWise(const cv::Point &p1, const cv::Point &p2, const cv::Point &p3)
{
  // Cross product of the vectors (p2-p1) and (p3-p1) yields the signed area:
  const long area_rect = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
  if (area_rect == 0)
    return 0;
  if (area_rect > 0)
    return +1;
  return -1;
}


/** @brief Provides access to a stack's top. Required by Graham scan. */
inline const cv::Vec2d &Top(const std::vector<cv::Vec2d> &stack)
{
  return stack[stack.size()-1];
}
inline const cv::Point &Top(const std::vector<cv::Point> &stack)
{
  return stack[stack.size()-1];
}

/** @brief Provides access to the element below a stack's top. Required by Graham scan. */
inline const cv::Vec2d &NextToTop(const std::vector<cv::Vec2d> &stack)
{
  return stack[stack.size()-2];
}
inline const cv::Point &NextToTop(const std::vector<cv::Point> &stack)
{
  return stack[stack.size()-2];
}

/** @brief Allows to sort 2D points with increasing y coordinate. Required by Graham scan.
 * Ties (a.y == b.y) are broken in favor of the point with the lower x coordinate.
 */
inline bool VecIsLower(const cv::Vec2d &a, const cv::Vec2d &b)
{
  if (vcp::math::eps_equal(a[1], b[1]))
    return a[0] < b[0]; // If y is equal, compare x coordinate.
  return a[1] < b[1]; // Compare y coordinate.
}

/** @brief Same as @see VecIsLower, but for cv::Point. */
bool PointIsLower(const cv::Point &a, const cv::Point &b)
{
  if (a.y == b.y)
    return a.x < b.x;
  return a.y < b.y;
}

std::vector<cv::Point> RemoveDuplicates(const std::vector<cv::Point> &points)
{
  // Sort points by their y coordinate (and use x coord. to break ties).
  std::vector<cv::Point> in;
  if (points.empty())
    return in;

  in.insert(in.end(), points.begin(), points.end());
  std::sort(in.begin(), in.end(), PointIsLower);

  std::vector<cv::Point> unique;
  unique.push_back(in[0]);
  for (size_t i = 1; i < in.size(); ++i)
  {
    const cv::Point &prev = in[i-1];
    const cv::Point &curr = in[i];
    if (prev.x != curr.x || prev.y != curr.y)
      unique.push_back(curr);
  }
  return unique;
}

std::vector<cv::Vec2d> RemoveDuplicates(const std::vector<cv::Vec2d> &points)
{
  // Sort points by their y coordinate (and use x coord. to break ties).
  std::vector<cv::Vec2d> in;
  if (points.empty())
    return in;

  in.insert(in.end(), points.begin(), points.end());
  std::sort(in.begin(), in.end(), VecIsLower);

  std::vector<cv::Vec2d> unique;
  unique.push_back(in[0]);
  for (size_t i = 1; i < in.size(); ++i)
  {
    const cv::Vec2d &prev = in[i-1];
    const cv::Vec2d &curr = in[i];
    if (!vcp::math::IsVecEqual(prev, curr))
      unique.push_back(curr);
  }
  return unique;
}
} // namespace utils

Line2d Line2d::Flipped() const
{
  return Line2d(this->pt_to_, this->pt_from_);
}


const cv::Vec2d &Line2d::From() const
{
  return pt_from_;
}


const cv::Vec2d &Line2d::To() const
{
  return pt_to_;
}


cv::Vec2d Line2d::Direction() const
{
  return DirectionVector(pt_from_, pt_to_);
}


cv::Vec2d Line2d::UnitDirection() const
{
  return Normalize(DirectionVector(pt_from_, pt_to_));
}


cv::Vec2d Line2d::MidPoint() const
{
  return 0.5 * (pt_from_ + pt_to_);
}

bool Line2d::empty() const
{
  return empty_ || vcp::math::IsVecEqual(pt_from_, pt_to_);
}


double Line2d::Angle(const cv::Vec2d &v) const
{
  return std::acos(Dot(Normalize(v), UnitDirection()));
}


cv::Vec3d Line2d::HomogeneousForm() const
{
  // For more on lines in projective space:
  // http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/BEARDSLEY/node2.html
  // http://robotics.stanford.edu/~birch/projective/node4.html
  return cv::Vec3d(pt_from_[0], pt_from_[1], 1.0).cross(cv::Vec3d(pt_to_[0], pt_to_[1], 1.0));
}


std::ostream &operator<< (std::ostream &stream, const Line2d &line)
{
  stream << line.pt_from_ << "-->" << line.pt_to_;
  return stream;
}


cv::Vec2d ProjectPointOntoLine(const cv::Vec2d &point, const Line2d &line)
{
  // Vector from line start to point:
  const cv::Vec2d v = DirectionVector(line.From(), point);

  // Project onto line and get closest point on line:
  const cv::Vec2d unit_direction = Normalize(line.Direction());
  const double lambda = Dot(unit_direction, v);
  return line.From() + lambda * unit_direction;
}


cv::Vec2d ClosestPointOnLineSegment(const cv::Vec2d &point, const Line2d &segment)
{
  // Vector from segment start to point:
  const cv::Vec2d v = DirectionVector(segment.From(), point);

  // Project v onto segment:
  const cv::Vec2d direction = segment.Direction();
  const cv::Vec2d unit_direction = Normalize(direction);
  const double lambda = Dot(unit_direction, v);

  if (lambda < 0.0)
    return segment.From();

  const double segment_length = Length(direction);
  if (lambda > segment_length)
    return segment.To();

  return segment.From() + lambda * unit_direction;
}


double DistancePointLine(const cv::Vec2d &point, const Line2d &line)
{
//  // Vector from line start to point:
//  const cv::Vec2d v = DirectionVector(line.From(), point);

//  // Project onto line and get closest point on line:
//  const cv::Vec2d unit_direction = Normalize(line.Direction());
//  const double lambda = Dot(unit_direction, v);
//  const cv::Vec2d closest_point = line.From() + lambda * unit_direction;
  const cv::Vec2d closest_point = ProjectPointOntoLine(point, line);
  return Distance(point, closest_point);
}

double DistancePointLineSegment(const cv::Vec2d &point, const Line2d &line_segment)
{
  // Compute lambda s.t. l=0: start point, l=1: end point of the segment.
  const cv::Vec2d line_direction = line_segment.Direction();
  const double lambda = Dot(point - line_segment.From(), line_direction) / LengthSquared(line_direction);

  const cv::Vec2d closest_point = line_segment.From() + std::min(1.0, std::max(0.0, lambda)) * line_direction;
  return Distance(point, closest_point);
}

cv::Vec2d VectorProjection(const cv::Vec2d &a, const cv::Vec2d &b)
{
  // https://math.oregonstate.edu/home/programs/undergrad/CalculusQuestStudyGuides/vcalc/dotprod/dotprod.html
  // https://en.wikipedia.org/wiki/Vector_projection

  // Vector projection is the unit vector b times the scalar projection of a onto b:
  //return ScalarProjection(a, b) * Normalize(b); is the same (result, but more computation) as:
  return Dot(a,b) / LengthSquared(b) * b;
}

double ScalarProjection(const cv::Vec2d &a, const cv::Vec2d &b)
{
  // https://en.wikipedia.org/wiki/Vector_projection
  return Dot(a, Normalize(b));
}


std::vector<cv::Vec2d> SimplifyRamerDouglasPeucker(const std::vector<cv::Vec2d> &points, double epsilon, size_t start_index, size_t end_index)
{
  double max_dist = 0.0;
  size_t index = start_index;
  const Line2d segment(points[start_index], points[end_index-1]);
  for (size_t i = index + 1; i < end_index-1; ++i)
  {
    // Don't use line distance (pseudocode on Wikipedia) but line-segment!
    const double dist = DistancePointLineSegment(points[i], segment);
//    const double dist = DistancePointLine(points[i], segment);
    if (dist > max_dist)
    {
      index = i;
      max_dist = dist;
    }
  }

  if (max_dist > epsilon)
  {
    // Recurse
    auto part1 = SimplifyRamerDouglasPeucker(points, epsilon, start_index, index+1);
    const auto part2 = SimplifyRamerDouglasPeucker(points, epsilon, index, end_index);
    // Concatenate - skip last element of first half since the end/start point are the same
    part1.pop_back();
    part1.insert(part1.end(), part2.begin(), part2.end());
    return part1;
  }
  else
  {
    std::vector<cv::Vec2d> pt = {points[start_index], points[end_index-1]};
    return pt;
  }
}



std::vector<size_t> SimplifyRamerDouglasPeuckerIndices(const std::vector<cv::Vec2d> &points, double epsilon, size_t start_index, size_t end_index)
{
  double max_dist = 0.0;
  size_t index = start_index;
  const Line2d segment(points[start_index], points[end_index-1]);
  for (size_t i = index + 1; i < end_index-1; ++i)
  {
    // Don't use line distance (pseudocode on Wikipedia) but line-segment!
    const double dist = DistancePointLineSegment(points[i], segment);
//    const double dist = DistancePointLine(points[i], segment);
    if (dist > max_dist)
    {
      index = i;
      max_dist = dist;
    }
  }

  if (max_dist > epsilon)
  {
    // Recurse
    auto part1 = SimplifyRamerDouglasPeuckerIndices(points, epsilon, start_index, index+1);
    const auto part2 = SimplifyRamerDouglasPeuckerIndices(points, epsilon, index, end_index);
    // Concatenate - skip last element of first half since the end/start point are the same
    part1.pop_back();
    part1.insert(part1.end(), part2.begin(), part2.end());
    return part1;
  }
  else
  {
    std::vector<size_t> pt_indices = {start_index, end_index-1};
    return pt_indices;
  }
}


std::vector<cv::Vec2d> SmoothTrajectory(const std::vector<cv::Vec2d> &positions, int smoothing_window)
{
  return vcp::imvis::trajectories::SmoothTrajectoryMovingAverage<std::vector<cv::Vec2d>>(positions, smoothing_window);
}


bool IsPointLeftOfLine(const cv::Vec2d &pt, const Line2d &line, bool *is_on_line)
{
  const cv::Vec2d line_dir = line.Direction();
  const cv::Vec2d tmp = pt - line.To();

  const double x = Cross(line_dir, tmp);//(line_dir.val[0] * tmp.val[1]) - (line_dir.val[1] * tmp.val[0]);

  // TODO eps equal
  if (is_on_line)
    *is_on_line = false;

  if (x < 0.0)
  {
    return false;
  }
  if (x > 0.0)
  {
    return true;
  }

  // 2d cross product == 0 => collinear points
  if (is_on_line)
    *is_on_line = true;
  return true;
}


cv::Vec2d RotateVector(const cv::Vec2d &vec, double theta)
{
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  /*
   * R = [ct -st
   *      st  ct]
   */
  return cv::Vec2d(ct*vec.val[0] - st*vec.val[1], st*vec.val[0] + ct*vec.val[1]);
}


cv::Vec2d RotateVector(const cv::Vec2d &vec, const cv::Vec2d &rotation_center, double theta)
{
  return RotateVector(vec - rotation_center, theta) + rotation_center;
}

cv::Vec2d RotateVectorPy(const cv::Vec2d &vec, const cv::Vec2d &rotation_center, double theta)
{
  return RotateVector(vec, rotation_center, theta);
}


std::vector<cv::Vec2d> SimplifyRamerDouglasPeucker(const std::vector<cv::Vec2d> &points, double epsilon)
{
  if (points.size() < 3)
    return points;
  return SimplifyRamerDouglasPeucker(points, epsilon, 0, points.size());
}

std::vector<size_t> SimplifyRamerDouglasPeuckerIndices(const std::vector<cv::Vec2d> &points, double epsilon)
{
  if (points.size() < 3)
  {
    std::vector<size_t> idx;
    for (size_t i = 0; i < points.size(); ++i)
      idx.push_back(i);
    return idx;
   }
  return SimplifyRamerDouglasPeuckerIndices(points, epsilon, 0, points.size());
}


std::vector<cv::Point> RemoveDuplicatePoints(const std::vector<cv::Point> &trajectory)
{
  std::vector<cv::Point> path;
  if (trajectory.size() == 0)
    return path;

  path.push_back(trajectory[0]);
  for (size_t i = 1; i < trajectory.size(); ++i)
  {
    if (trajectory[i].x == path[path.size()-1].x && trajectory[i].y == path[path.size()-1].y)
      continue;

    path.push_back(trajectory[i]);
  }
  return path;
}


std::vector<cv::Point> BresenhamLine(const cv::Point &from, const cv::Point &to)
{
  std::vector<cv::Point> line;
  int x1 = from.x;
  int y1 = from.y;
  int x2 = to.x;
  int y2 = to.y;
  const bool is_steep = std::abs(y2 - y1) > std::abs(x2 - x1);
  if (is_steep)
  {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }

  const bool is_reverse = x1 > x2;
  if (is_reverse)
  {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }

  const int dx = x2 - x1;
  const int dy = std::abs(y2 - y1);

  double error = dx / 2.0;
  const int step_y = (y1 < y2) ? 1 : -1;
  int y = y1;

  const int max_x = x2;

  for(int x = x1; x <= max_x; ++x)
  {
    if (is_steep)
    {
      line.push_back(cv::Point(y, x));
    } else
    {
      line.push_back(cv::Point(x, y));
    }

    error -= dy;
    if (error < 0)
    {
      y += step_y;
      error += dx;
    }
  }

  if (is_reverse)
  {
    std::reverse(line.begin(), line.end());
  }
  return line;
}


std::vector<cv::Point> BresenhamTrajectory(const std::vector<cv::Point> &trajectory)
{
  const auto cleaned_trajectory = RemoveDuplicatePoints(trajectory);

  std::vector<cv::Point> bresenham_points;
  for (size_t i = 1; i < cleaned_trajectory.size(); ++i)
  {
    const auto from = cleaned_trajectory[i-1];
    const auto to = cleaned_trajectory[i];

    std::vector<cv::Point> line = BresenhamLine(from, to);

    // Starting from the second approximated segment, we would add its start
    // point twice (as it is already in bresenham_points as the end point
    // of the previously inserted line).
    if (i > 1)
    {
      const size_t offset = line.size() > 1 ? 1 : 0;
      bresenham_points.insert(bresenham_points.end(), line.begin() + offset, line.end());
    }
    else
    {
      bresenham_points.insert(bresenham_points.end(), line.begin(), line.end());
    }
  }
  return bresenham_points;
}


bool IsPointInClosedPolygonHelper(const cv::Vec2d &point, const std::vector<cv::Vec2d> &polygon)
{
  // Based on https://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
  // Currently, we need a closed polygon!
  for (size_t i = 1; i < polygon.size(); ++i)
  {
    const double d = DistancePointLineSegment(point, Line2d(polygon[i-1], polygon[i]));
    if (vcp::math::eps_zero(d))
    {
      VCP_LOG_DEBUG("Point " << point << " lies on polygon edge " << polygon[i-1] << "--" << polygon[i]);
      return true;
    }
  }

  // Inside-out-test traversing a ray through the poly.
  VCP_LOG_DEBUG("Performing inside-out-test for point " << point);
  bool c = false;
  for(size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
  {
    if( ( (polygon[i].val[1] >= point.val[1] ) != (polygon[j].val[1] >= point.val[1]) ) &&
        (point.val[0] <= (polygon[j].val[0] - polygon[i].val[0]) * (point.val[1] - polygon[i].val[1]) / (polygon[j].val[1] - polygon[i].val[1]) + polygon[i].val[0])
      )
      c = !c;
  }
  return c;
}


bool IsPointInClosedPolygon(const cv::Vec2d &point, const std::vector<cv::Vec2d> &polygon)
{
  const bool is_closed = IsVecEqual(polygon[0], polygon[polygon.size()-1], 2);
  if (is_closed)
    return IsPointInClosedPolygonHelper(point, polygon);

  VCP_LOG_DEBUG("IsPointInClosedPolygon() called with open polygon - closing it for you.");
  std::vector<cv::Vec2d> closed;
  closed.insert(closed.end(), polygon.begin(), polygon.end());
  closed.push_back(polygon[0]);
  return IsPointInClosedPolygonHelper(point, closed);
}


double DistancePointClosedPolygonHelper(const cv::Vec2d &point, const std::vector<cv::Vec2d> &polygon)
{
  const bool inside = IsPointInClosedPolygonHelper(point, polygon);

  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 1; i < polygon.size(); ++i)
  {
    const double dist = DistancePointLineSegment(point, Line2d(polygon[i-1], polygon[i]));
    if (dist < min_dist)
      min_dist = dist;
  }
  return inside ? -min_dist : min_dist;
}

double DistancePointClosedPolygon(const cv::Vec2d &point, const std::vector<cv::Vec2d> &polygon)
{
  const bool is_closed = IsVecEqual(polygon[0], polygon[polygon.size()-1], 2);
  if (is_closed)
    return DistancePointClosedPolygonHelper(point, polygon);

  VCP_LOG_DEBUG("DistancePointClosedPolygon() called with open polygon - closing it for you.");
  std::vector<cv::Vec2d> closed;
  closed.insert(closed.end(), polygon.begin(), polygon.end());
  closed.push_back(polygon[0]);
  return DistancePointClosedPolygonHelper(point, closed);
}

bool IsPointInsideRectangle(const cv::Vec2d &pt, const cv::Rect2d &rect)
{
  return (pt[0] >= rect.x) && (pt[0] <= rect.x + rect.width - 1.0)
      && (pt[1] >= rect.y) && (pt[1] <= rect.y + rect.height - 1.0);
}


cv::Rect2d AxisAlignedBoundingRect(const std::vector<cv::Vec2d> &polygon)
{
  if (polygon.empty())
    return cv::Rect2d();
  cv::Vec2d min(polygon[0]), max(polygon[0]);
  for (size_t i = 1; i < polygon.size(); ++i)
  {
    if (polygon[i].val[0] < min.val[0])
      min.val[0] = polygon[i].val[0];
    if (polygon[i].val[1] < min.val[1])
      min.val[1] = polygon[i].val[1];

    if (polygon[i].val[0] > max.val[0])
      max.val[0] = polygon[i].val[0];
    if (polygon[i].val[1] > max.val[1])
      max.val[1] = polygon[i].val[1];
  }
  return cv::Rect2d(min.val[0], min.val[1], max.val[0]-min.val[0], max.val[1]-min.val[1]);
}


cv::RotatedRect RotatedBoundingRect(const std::vector<cv::Vec2d> &polygon)
{
  std::vector<cv::Vec2f> poly2f;
  poly2f.reserve(polygon.size());
  for (const auto &p : polygon)
    poly2f.push_back(cv::Vec2f(static_cast<float>(p.val[0]), static_cast<float>(p.val[1])));
  return cv::minAreaRect(poly2f);
}


bool IsCollinear(const Line2d &a, const Line2d &b)
{
  // See IntersectionLineSegmentLineSegment() for more documentation and further links!
  // Line 1 goes from p to p + r
  const cv::Vec2d p = a.From();
  const cv::Vec2d r = a.Direction(); //a.To() - p;
  // Line 2 goes from q to q + s
  const cv::Vec2d q = b.From();
  const cv::Vec2d s = b.Direction(); //b.To() - q;

  const double rxs = Cross(r, s);
  const double qmpxr = Cross((q-p), r);

  if (vcp::math::eps_zero(rxs) && eps_zero(qmpxr))
    return true;
  return false;
}

bool IntersectionLineLine(const Line2d &a, const Line2d &b, cv::Vec2d &intersection)
{
  cv::Vec3d ip = a.HomogeneousForm().cross(b.HomogeneousForm());
  //TODO add tests for: a) no intersection, b) point, c) line (if parallel and the same)

  if (vcp::math::eps_zero(ip[2]))
  {
    VCP_LOG_DEBUG("IntersectionLineLine(): no intersection/lines are parallel.");
    return false;
  }
  intersection = cv::Vec2d(ip[0]/ip[2], ip[1]/ip[2]);
  return true;
}


bool IntersectionLineLineSegment(const Line2d &line, const Line2d &segment, cv::Vec2d &intersection)
{
  // Line 1 goes from p to p + r
  const cv::Vec2d p = line.From();
  const cv::Vec2d r = line.Direction(); //a.To() - p;
  // Line 2 goes from q to q + s
  const cv::Vec2d q = segment.From();
  const cv::Vec2d s = segment.Direction(); //b.To() - q;

  const double rxs = Cross(r, s);
  const double qmpxr = Cross((q-p), r);

  if (vcp::math::eps_zero(rxs) && eps_zero(qmpxr))
  {
    // Line and segment are collinear.
    VCP_LOG_WARNING("IntersectionLineLineSegment(): Line and segment are collinear - returning start of segment as intersection point.");
    intersection = segment.From();
    return true;
  }

  if (vcp::math::eps_zero(rxs))
  {
    // Parallel and not intersecting
    return false;
  }

  // Otherwise, they intersect if u in [0,1] (i.e. on the segment)
  const double u = qmpxr / rxs;

  if (u >= 0.0 && u <= 1.0)
  {
    intersection = q + u * s;
    return true;
  }
  return false;
}


bool IntersectionLineSegmentLineSegment(const Line2d &a, const Line2d &b, cv::Vec2d &intersection)
{
  // Based on https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
  // Line 1 goes from p to p + r
  const cv::Vec2d p = a.From();
  const cv::Vec2d r = a.Direction(); //a.To() - p;
  // Line 2 goes from q to q + s
  const cv::Vec2d q = b.From();
  const cv::Vec2d s = b.Direction(); //b.To() - q;

  const double rxs = Cross(r, s);
  const double qmpxr = Cross((q-p), r);

  if (vcp::math::eps_zero(rxs) && eps_zero(qmpxr))
  {
    // Lines are collinear. They intersect if there is any overlap.
    const double t0 = Dot((q-p), r) / Dot(r, r);
    const double t1 = t0 + Dot(s, r) / Dot(r,r);

    if (t0 >= 0.0 && t0 <= 1.0)
    {
      intersection = p + t0 * r;
      return true;
    }
    if (t1 >= 0.0 && t1 <= 1.0)
    {
      intersection = p + t1 * r;
      return true;
    }
    // Otherwise, the segments don't intersect
    return false;
  }

  if (vcp::math::eps_zero(rxs))
  {
    // Segments are parallel and not intersecting
    return false;
  }

  // Otherwise, the segments meet if u in [0,1] and t in [0,1]
  const double u = qmpxr / rxs;
  const double t = Cross((q-p), s) / Cross(r, s);

  if (u >= 0.0 && u <= 1.0 && t >= 0.0 && t <= 1.0)
  {
    intersection = p + t * r;
    return true;
  }
  return false;
}


int IntersectionLineCircle(const Line2d &a, const cv::Vec2d &center, double radius, cv::Vec2d &intersection1, cv::Vec2d &intersection2)
{
  // Based on http://mathworld.wolfram.com/Circle-LineIntersection.html, so shift primitives such that circle is centered at (0,0)
  const double x1 = a.From().val[0] - center.val[0];
  const double x2 = a.To().val[0] - center.val[0];
  const double y1 = a.From().val[1] - center.val[1];
  const double y2 = a.To().val[1] - center.val[1];

  const double dx = x2 - x1;
  const double dy = y2 - y1;
  const double dr = std::sqrt(dx*dx + dy*dy);
  const double dr_sqr = dr * dr;
  const double D = x1*y2 - x2*y1;

  const double discriminant = radius*radius * dr_sqr - D*D;
  const double discriminant_sqrt = sqrt(discriminant);
  const double sgn = dy < 0.0 ? -1.0 : 1.0;
  if (std::fabs(discriminant) < 1e-7) // TODO eps_equal/eps_zero!
  {
    // discriminant = 0: tangent
    intersection1.val[0] = (D * dy + sgn * dx * discriminant_sqrt) / dr_sqr + center.val[0];
    intersection1.val[1] = (-D * dx + std::fabs(dy) * discriminant_sqrt) / dr_sqr + center.val[1];
    return 1;
  }
  else if (discriminant > 0.0)
  {
    // 2 intersection points
    intersection1.val[0] = (D * dy + sgn * dx * discriminant_sqrt) / dr_sqr + center.val[0];
    intersection1.val[1] = (-D * dx + std::fabs(dy) * discriminant_sqrt) / dr_sqr + center.val[1];

    intersection2.val[0] = (D * dy - sgn * dx * discriminant_sqrt) / dr_sqr + center.val[0];
    intersection2.val[1] = (-D * dx - std::fabs(dy) * discriminant_sqrt) / dr_sqr + center.val[1];
    return 2;
  }
  else
  {
    // No intersection
    return 0;
  }
}


int IntersectionLineSegmentCircle(const Line2d &a, const cv::Vec2d &center, double radius, cv::Vec2d &intersection1, cv::Vec2d &intersection2)
{
  // Compute intersection points with line
  cv::Vec2d line_intersect1, line_intersect2;
  const int line_intersections = IntersectionLineCircle(a, center, radius, line_intersect1, line_intersect2);

  // Check if they're on the segment
  if (line_intersections == 0)
    return 0;

  const double dist1 = DistancePointLineSegment(line_intersect1, a);
  const bool on_segment1 = eps_zero(dist1);

  if (line_intersections == 1)
  {
    if (on_segment1)
    {
      intersection1 = line_intersect1;
      return 1;
    }
    return 0;
  }

  const double dist2 = DistancePointLineSegment(line_intersect2, a);
  const bool on_segment2 = eps_zero(dist2);

  int num_intersections = 0;
  if (on_segment1)
  {
    intersection1 = line_intersect1;
    ++num_intersections;

    if (on_segment2)
    {
      intersection2 = line_intersect2;
      ++num_intersections;
    }
  }
  else
  {
    if (on_segment2)
    {
      intersection1 = line_intersect2;
      ++num_intersections;
    }
  }
  return num_intersections;
}


bool CircleFrom3Points(const cv::Vec2d &pt1, const cv::Vec2d &pt2, const cv::Vec2d &pt3, cv::Vec2d &center, double &radius)
{
  // Check if the points are collinear
  const Line2d segment1(pt1, pt2);
  const Line2d segment2(pt2, pt3);

  if (IsCollinear(segment1, segment2))
      return false;

  // Compute center as the intersection point of the segments:
  // First, we need the bisecting lines
  const cv::Vec2d mid1 = segment1.MidPoint();
  const cv::Vec2d dir1 = segment1.Direction();
  const cv::Vec2d orth1(dir1.val[1], -dir1.val[0]);
  const Line2d bisect1(mid1, mid1 + orth1);
  // ... same for the second chord
  const cv::Vec2d mid2 = segment2.MidPoint();
  const cv::Vec2d dir2 = segment2.Direction();
  const cv::Vec2d orth2(dir2.val[1], -dir2.val[0]);
  const Line2d bisect2(mid2, mid2 + orth2);
  // Intersect them
  const bool exists = IntersectionLineLine(bisect1, bisect2, center);
  if (!exists)
    VCP_ERROR("CircleFrom3Points(): Intersection of the bisecting lines does not exist (maybe numerical issue?)");

  radius = Distance(pt1, center);
  return true;
}



Line2d ClipLineByRectangle(const Line2d &line, const cv::Rect2d &rect)
{
  const cv::Vec2d tl(rect.x, rect.y);
  const cv::Vec2d tr(rect.x + rect.width-1.0, rect.y);
  const cv::Vec2d br(tr[0], rect.y + rect.height - 1.0);
  const cv::Vec2d bl(tl[0], br[1]);

  std::vector<Line2d> edges;
  edges.reserve(4);
  edges.push_back(Line2d(tl, tr));
  edges.push_back(Line2d(tr, br));
  edges.push_back(Line2d(br, bl));
  edges.push_back(Line2d(bl, tl));

  std::vector<cv::Vec2d> intersection_points;
  for (size_t i = 0; i < edges.size(); ++i)
  {
    if (IsCollinear(line, edges[i]))
    {
      return edges[i];
    }

    cv::Vec2d ip;
    if (IntersectionLineLineSegment(line, edges[i], ip))
    {
      // We iterate the rect edges clockwise. Thus, if a intersection
      // point falls exactly on the corner, we would have found it when
      // testing the previous edge.
      // If an intersection is the top-left corner, it would still be added
      // twice (once as int_points[0], then as int_points[2]) - however, this
      // can easily be handled by only building a line from int_points[0] to int_points[1] ;-)
      if (intersection_points.empty() || !IsVecEqual(ip, intersection_points[intersection_points.size()-1]))
      {
        intersection_points.push_back(ip);
      }
    }
  }

  if (intersection_points.size() < 2)
    return Line2d();
  return Line2d(intersection_points[0], intersection_points[1]);
}


Line2d ClipLineByRectangle(const Line2d &line, const cv::Rect &rect)
{
  return ClipLineByRectangle(line, cv::Rect2d(rect.x, rect.y, rect.width, rect.height));
}

Line2d ClipLineByRectanglePy(const Line2d &line, const cv::Rect2d &rect)
{
  return ClipLineByRectangle(line, rect);
}


Line2d ClipLineSegmentByRectangle(const Line2d &segment, const cv::Rect2d &rect)
{
  const bool from_inside = IsPointInsideRectangle(segment.From(), rect);
  const bool to_inside = IsPointInsideRectangle(segment.To(), rect);

  // Both inside
  if (from_inside && to_inside)
    return segment;

  const cv::Vec2d tl(rect.x, rect.y);
  const cv::Vec2d tr(rect.x + rect.width-1.0, rect.y);
  const cv::Vec2d br(tr[0], rect.y + rect.height - 1.0);
  const cv::Vec2d bl(tl[0], br[1]);

  std::vector<Line2d> edges;
  edges.reserve(4);
  edges.push_back(Line2d(tl, tr));
  edges.push_back(Line2d(tr, br));
  edges.push_back(Line2d(br, bl));
  edges.push_back(Line2d(bl, tl));

  std::vector<cv::Vec2d> intersections;

  for (size_t i = 0; i < 4; ++i)
  {
    cv::Vec2d intersection;
    if (IntersectionLineSegmentLineSegment(segment, edges[i], intersection))
    {
      // We iterate the rect edges clockwise. Thus, if a intersection
      // point falls exactly on the corner, we would have found it when
      // testing the previous edge.
      // If an intersection is the top-left corner, it would still be added
      // twice (once as int_points[0], then as int_points[2]) - however, this
      // can easily be handled by only building a line from int_points[0] to int_points[1] ;-)
      if (intersections.empty() || !IsVecEqual(intersection, intersections[intersections.size()-1]))
      {
        intersections.push_back(intersection);
      }
    }
  }

  if (intersections.empty())
    return Line2d();

  if (from_inside != to_inside)
  {
    // One in, one out - there should be only 1 intersection point.
    // If the intersection falls exactly on a corner, there may be two (but never more)
    VCP_CHECK(intersections.size() < 3);
    if (from_inside)
      return Line2d(segment.From(), intersections[0]);
    else
      return Line2d(intersections[0], segment.To());
  }
  else
  {
    return Line2d(intersections[0], intersections[1]);
  }
}


Line2d ClipLineSegmentByRectangle(const Line2d &segment, const cv::Rect &rect)
{
  return ClipLineSegmentByRectangle(segment, cv::Rect2d(rect.x, rect.y, rect.width, rect.height));
}

Line2d ClipLineSegmentByRectanglePy(const Line2d &segment, const cv::Rect2d &rect)
{
  return ClipLineSegmentByRectangle(segment, rect);
}


Line2d GetProjectionOfHorizon(const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, const cv::Size &image_size)
{
  // Get a vector pointing along the camera's optical axis, which is orthogonal to the ground plane normal:
  const cv::Vec3d img_plane_normal = vcp::math::geo3d::PlaneNormal(vcp::math::geo3d::ImagePlaneInWorldCoordinateSystem(R, t));
  cv::Vec2d horizon_dir(img_plane_normal[0], img_plane_normal[1]);
  if (IsVecEqual(horizon_dir, cv::Vec2d(0.0, 0.0)))
  {
    VCP_LOG_WARNING("Camera points along the world's z-axis. Horizon is not visible!");
    return Line2d();
  }
  // Unit vector
  horizon_dir = Normalize(horizon_dir);

  // Get two points in front of the camera, which project onto the horizon line, i.e. all points at the same height as the camera
  // see e.g., http://web.engr.illinois.edu/~slazebni/spring16/lec02_perspective-daf.pdf
  const cv::Vec2d perpendicular_dir(horizon_dir[1], -horizon_dir[0]);
  const cv::Vec3d camera_center = vcp::convert::ToVec3d(-R.t() * t);
  const cv::Vec2d camera_center2d(camera_center[0], camera_center[1]);

  const cv::Vec2d pt1 = camera_center2d + 1000.0 * horizon_dir;
  const cv::Vec2d pt2 = pt1 + 500.0 * perpendicular_dir;

  const cv::Mat P = vcp::math::geo3d::ProjectionMatrixFromKRt(K, R, t);
  const cv::Vec2d prj1 = vcp::math::geo3d::ProjectVec(P, cv::Vec3d(pt1[0], pt1[1], camera_center[2]));
  const cv::Vec2d prj2 = vcp::math::geo3d::ProjectVec(P, cv::Vec3d(pt2[0], pt2[1], camera_center[2]));

  Line2d horizon(prj1, prj2);

  if (image_size.width > 0 && image_size.height > 0)
  {
    return ClipLineByRectangle(horizon, cv::Rect(0, 0, image_size.width, image_size.height));
  }
  return horizon;
}

std::vector<cv::Point> ConvexHullGrahamScan(const std::vector<cv::Point> &points, bool check_for_duplicates)
{
  std::vector<cv::Point> hull;
  if (points.size() < 3)
    return hull;

  std::vector<cv::Point> pts;
  if (check_for_duplicates)
    pts = utils::RemoveDuplicates(points);
  else
    pts.insert(pts.end(), points.begin(), points.end());

  // Find point with the lowest y-coordinate (will always be on the convex hull)
  size_t lowest_point = 0;
  for (size_t i = 1; i < pts.size(); ++i)
  {
    if (utils::PointIsLower(pts[i], pts[lowest_point]))
      lowest_point = i;
  }

  // Swap first point with the found pivot element
  if (lowest_point != 0)
  {
    const cv::Point tmp = pts[0];
    pts[0] = pts[lowest_point];
    pts[lowest_point] = tmp;
  }
  const cv::Point ref_pt = pts[0];


  // Sort points by their angle w.r.t. the pivot
  std::sort(pts.begin(), pts.end(), [ref_pt](const cv::Point &a, const cv::Point &b) -> bool
    {
      // Basically, we use the (sign of the) cross product (a-R) x (b-R) to decide which has the smaller angle:
      // The CCW() util returns the sign of (a-R) x (b-R):
      const int ccw = utils::CounterClockWise(ref_pt, a, b);
      // If all three points are collinear, sort a and b by their distance to REF
      if (ccw == 0)
        return LengthSquared(a-ref_pt) < LengthSquared(b-ref_pt);
      // If the points (REF, a, b) make a left-turn (counter-clockwise), then a has a smaller polar coordinate (w.r.t. REF) than b
      return ccw < 0;
    });


  // Do the scan line ;-)
  hull.push_back(pts[0]);
  hull.push_back(pts[1]);
  hull.push_back(pts[2]);
  for (size_t i = 3; i < pts.size(); ++i)
  {
    while (utils::CounterClockWise(utils::Top(hull), utils::NextToTop(hull), pts[i]) <= 0)
    {
      hull.pop_back();
      if (hull.size() <= 2)
        break;
    }

    hull.push_back(pts[i]);
  }
  return hull;
}

//TODO add scale parameter: scale, convert to point, hull, rescale, return
std::vector<cv::Vec2d> ConvexHullGrahamScan(const std::vector<cv::Vec2d> &points, bool check_for_duplicates, double scale)
{
  std::vector<cv::Point> converted;

  converted.reserve(points.size());
  for (const auto &pt : points)
  {
    converted.push_back(vcp::convert::ToPoint(scale * pt));
  }

  const std::vector<cv::Point> hull = ConvexHullGrahamScan(converted, check_for_duplicates);

  std::vector<cv::Vec2d> hull2d;
  hull2d.reserve(hull.size());

  for (const auto &pt : hull)
    hull2d.push_back(vcp::convert::ToVec2d(pt) / scale);
  return hull2d;
}


std::vector<cv::Vec2d> RotateVecs(const std::vector<cv::Vec2d> &pts, const cv::Vec2d &rotation_center, double theta)
{
  std::vector<cv::Vec2d> rot;
  rot.reserve(pts.size());
  for (size_t i = 0; i < pts.size(); ++i)
    rot.push_back(vcp::math::geo2d::RotateVector(pts[i], rotation_center, theta));
  return rot;
}

double IntersectionOverUnionRects(const cv::Rect &b1, const cv::Rect &b2)
{
  // WATCH OUT: OpenCV Rect area measure excludes the right/bottom edges. We INclude them, so we cannot use rect.area()!

  // Top-left intersection corner (if any)
  const int xA = std::max(b1.x, b2.x);
  const int yA = std::max(b1.y, b2.y);
  // Bottom-right intersection corner (if any)
  const int xB = std::min(b1.x + b1.width, b2.x + b2.width);
  const int yB = std::min(b1.y + b1.height, b2.y + b2.height);
  // Intersection area (or 0). Include both left/right and top/bottom edges, e.g. Width = Right - Left + 1 (!!)
  const int a_intersection = std::max(0, xB - xA + 1) * std::max(0, yB - yA + 1);
  if (a_intersection == 0)
    return 0.0;

  // Union
  const int a_union = (b1.width + 1) * (b1.height + 1) + (b2.width + 1) * (b2.height + 1) - a_intersection;
  if (a_union > 0)
    return static_cast<double>(a_intersection) / static_cast<double>(a_union);
  return 0.0;
}

double IntersectionOverUnionRotatedRects(const cv::RotatedRect &r1, const cv::RotatedRect &r2)
{
  cv::Point2f vertices1[4], vertices2[4];
  r1.points(vertices1);
  r2.points(vertices2);

  // Intersection area: https://stackoverflow.com/questions/44797713/calculate-the-area-of-intersection-of-two-rotated-rectangles-in-python/45141648#45141648
  std::vector<cv::Vec2d> poly1, poly2;
  for (size_t i = 0; i < 4; ++i)
  {
    poly1.push_back(vcp::convert::ToVec2d(vertices1[i]));
    poly2.push_back(vcp::convert::ToVec2d(vertices2[i]));
  }
  const std::vector<cv::Vec2d> intersection = IntersectionConvexPolygons(poly1, poly2);
  if (intersection.empty())
    return 0.0;

  const double a_intersection = AreaPolygon(intersection);

  const double area1 = AreaRotatedRect(vertices1);
  const double area2 = AreaRotatedRect(vertices2);
  const double a_union = area1 + area2 - a_intersection;

  if (a_union > 0.0)
    return static_cast<double>(a_intersection) / static_cast<double>(a_union);
  return 0.0;
}


bool IsPolygonClockwise(const std::vector<cv::Vec2d> &polygon)
{
  if (polygon.size() < 3)
    VCP_ERROR("IsPolygonClockwise(): Polygon must have at least 3 vertices");

  for (size_t idx = 2; idx < polygon.size(); ++idx)
  {
    bool on_line;
    const bool is_left = IsPointLeftOfLine(polygon[idx], Line2d(polygon[0], polygon[1]), &on_line);
    if (!on_line)
      return !is_left;
  }
  VCP_ERROR("IsPolygonClockwise(): All polygon points are collinear");
}

std::vector<cv::Vec2d> IntersectionConvexPolygons(const std::vector<cv::Vec2d> &subject, const std::vector<cv::Vec2d> &clip)
{
  // Sutherland-Hodgman
  // http://rosettacode.org/wiki/Sutherland-Hodgman_polygon_clipping#C.23
  // Maybe more efficient: but harder to implement (?)
  //http://www-cgrl.cs.mcgill.ca/~godfried/teaching/cg-projects/97/Plante/CompGeomProject-EPlante/algorithm.html

  // We clip the "subject" poly
  std::vector<cv::Vec2d> output;
  output.insert(output.end(), subject.begin(), subject.end());

  if (!IsPolygonClockwise(subject))
  {
    VCP_LOG_DEBUG("IntersectionConvexPolygons(): Need to reverse the subject polygon (1st param) to ensure clockwise ordering.");
    std::reverse(std::begin(output), std::end(output));
  }

  const bool is_clip_clockwise = IsPolygonClockwise(clip);

  // Inside/Outside check (rename left/right to follow standard nomenclature)
  auto _IsInside = [](const Line2d &line, const cv::Vec2d &test) {
    bool on_line;
    const bool left = IsPointLeftOfLine(test, line, &on_line);
    if (on_line) // Collinear points are considered inside
      return true;
    return !left;
  };

  // Process the clip polygon clockwise
  cv::Vec2d intersection_point;
  for (size_t i = 0; i < clip.size(); ++i)
  {
    const cv::Vec2d from = is_clip_clockwise ? clip[i] : clip[clip.size()-1-i];
    const cv::Vec2d to = is_clip_clockwise ? clip[(i+1) % clip.size()] : ((i < clip.size() - 1) ? clip[clip.size()-2-i] : clip[clip.size()-1]);
    const Line2d clip_edge(from, to);

    std::vector<cv::Vec2d> input;
    input.insert(input.end(), output.begin(), output.end());

    output.clear();
    if (input.empty()) // Can happen if the polygons don't intersect
      break;

    cv::Vec2d s = input[input.size()-1];
    for (const auto &e : input)
    {
      if (_IsInside(clip_edge, e))
      {
        if (!_IsInside(clip_edge, s))
        {

          if (IntersectionLineLine(Line2d(s, e), clip_edge, intersection_point))
          {
            output.push_back(intersection_point);
          }
          else
          {
            VCP_ERROR("IntersectionConvexPolygons(): Line segments don't intersect"); // maybe parallel
          }
        }
        output.push_back(e);
      }
      else if (_IsInside(clip_edge, s))
      {
        if (IntersectionLineLine(Line2d(s, e), clip_edge, intersection_point))
        {
          output.push_back(intersection_point);
        }
        else
        {
          VCP_ERROR("IntersectionConvexPolygons(): Line segments don't intersect"); // maybe parallel
        }
      }

      s = e;
    }
  }

  return output;
}

double AreaTriangle(const cv::Vec2d &a, const cv::Vec2d &b, const cv::Vec2d &c)
{
  //return std::fabs(ax * (by - cy) + bx * (cy - ay) + cx * (ay - by)) / 2.0;
  return std::fabs(a.val[0] * (b.val[1] - c.val[1]) + b.val[0] * (c.val[1] - a.val[1]) + c.val[0] * (a.val[1] - b.val[1])) / 2.0;
}


double AreaRotatedRect(const cv::Point2f *vertices)
{
  const cv::Point2f a = vertices[0];
  const cv::Point2f b = vertices[1];
  const cv::Point2f c = vertices[2];
  return std::fabs(a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
}


double AreaRotatedRect(const cv::RotatedRect &r)
{
  cv::Point2f vertices[4];
  r.points(vertices);
  return AreaRotatedRect(vertices);
}

double AreaRotatedRectPy(const cv::RotatedRect &r)
{
  return AreaRotatedRect(r);
}


double AreaPolygon(const std::vector<cv::Vec2d> &pts)
{
  // Points must be clockwise or counterclockwise
  // WATCH OUT for self-crossing, twisted, etc. weird polygons, this usually doesn't work
  // http://alienryderflex.com/polygon_area/
  double area = 0.0;
  for (int i = 0, j = static_cast<int>(pts.size())-1; i < static_cast<int>(pts.size()); ++i)
  {
    area += (pts[j].val[0] + pts[i].val[0]) * (pts[j].val[1] - pts[i].val[1]);
    j=i;
  }
  return std::fabs(area * 0.5);
}


bool IsPointInCircle(const cv::Vec2d &pt, const cv::Vec2d &center, double radius, bool *is_on_circle)
{
  const double dist = Distance(pt, center);
  if (is_on_circle)
    *is_on_circle = false;

  //TODO eps equal!
  if (dist < radius)
    return true;
  if (dist > radius)
    return false;

  *is_on_circle = true;
  return true;
}

bool PointsOfTangencyPointToCircle(const cv::Vec2d &pt, const cv::Vec2d &center, double radius, cv::Vec2d &pot1, cv::Vec2d &pot2)
{
  bool on_circle;
  bool inside_circle = IsPointInCircle(pt, center, radius, &on_circle);

  if (inside_circle)
    return false;

  if (on_circle)
  {
    pot1 = pt;
    pot2 = pt;
    return true;
  }

  /*
                    __...------__    pot1
               _.-''             -(+)
            ,-'                   |----
          ,'                     ||    ----
        ,'                      | |     '  ----
       /                       |  |      `     ----
      /               radius  |   |  h    `.       ----  distance
     /                       |    |        \           ----
    |                       |     |         |              ----
    |                      |      |          |                ( ----
    |                     |   q   |          |    p          (  alpha ----
    |                   (+)---------------------------------------------(+) pt
    |                                        .'    hypotenuse
    |                    C                   |
     |                                       '
      \                                     /
       \                                  ,'
        `                                /
         '.                            ,'
           '-.                      _,'
              '-._              _,(+)  pot
                  '`--......---'
  */

  // Pythagoras
  const double hypotenuse = Distance(pt, center);
  const double radius_sqr = radius*radius;
  const double distance = std::sqrt(hypotenuse*hypotenuse - radius_sqr);

  // Kathetensatz
  const double q = radius_sqr / hypotenuse;
  const double p = hypotenuse - q;
  // Hoehensatz
  const double h = std::sqrt(p*q);
  // Trigonometrie tan(α) = h/p = a/b
  const double alpha = std::atan2(h, p);


  const Line2d hypo(pt, center);
  const cv::Vec2d to_rotate = distance*hypo.UnitDirection();

  cv::Vec2d dir = RotateVector(to_rotate, alpha);
  pot1 = pt + dir;
  dir = RotateVector(to_rotate, -alpha);
  pot2 = pt + dir;

  return true;
}

int TransverseCommonTangentsBetweenCircles(const cv::Vec2d &center1, double radius1, const cv::Vec2d &center2, double radius2, Line2d &tangent1, Line2d &tangent2)
{
  // Nice visualizations and explanations: https://gradestack.com/Complete-CAT-Prep/Geometry/Direct-common-Tangents/19146-3883-35862-study-wtw
  const double distance = Distance(center1, center2);
  const double sum_radii = radius1 + radius2;

  if (distance < sum_radii)
  {
    // More than 1 intersection points, there are no transverse common tangents!
    return 0;
  }

  const Line2d center_line(center1, center2);
  // TODO eps compare for zero!
  if (distance > sum_radii)
  {
    // Transverse common tangents divide the line between the two centers in the ratio r1/r2 (similar triangles to the rescue).
    // Compute distance to intersection point:
    const double distance_intersection = distance * radius1 / sum_radii;
    const cv::Vec2d intersection = center1 + distance_intersection * center_line.UnitDirection();
    // https://doubleroot.in/lessons/coordinate-geometry-basics/section-formula/

    // Pythagoras
    const double hypotenuse = Distance(intersection, center1);
    const double radius1_sqr = radius1*radius1;
    // Kathetensatz
    const double q = radius1_sqr / hypotenuse;
    const double p = hypotenuse - q;
    // Hoehensatz
    const double h = std::sqrt(p*q);
    // Trigonometrie tan(α) = h/p = a/b
    const double alpha = std::atan2(h, p);

    // Compute points of tangency
    // The tangent is also divided by the ratio r1/r2
    const double tangent_length = std::sqrt(distance*distance - sum_radii*sum_radii);
    const double tangent_length1 = std::sqrt(hypotenuse*hypotenuse - radius1_sqr);
    const double tangent_length2 = tangent_length - tangent_length1;

    const Line2d hypo(intersection, center1);
    const cv::Vec2d hypo_dir = hypo.UnitDirection();

    cv::Vec2d dir = RotateVector(hypo_dir, alpha);
    const cv::Vec2d t11 = intersection + tangent_length1*dir;
    const cv::Vec2d t12 = intersection - tangent_length2*dir;
    tangent1 = Line2d(t11, t12);

    dir = RotateVector(hypo_dir, -alpha);
    const cv::Vec2d t21 = intersection + tangent_length1*dir;
    const cv::Vec2d t22 = intersection - tangent_length2*dir;
    tangent2 = Line2d(t21, t22);

    return 2;
  }

  // Compute the single intersection point
  const cv::Vec2d unit_dir = center_line.UnitDirection();
  const cv::Vec2d intersection_point = center1 + radius1 * unit_dir;

  // Choose any reference point perpendicular to the center line (to create the tangent line)
  const cv::Vec2d ref1 = intersection_point + radius1*cv::Vec2d(unit_dir.val[1], -unit_dir.val[0]);
  const cv::Vec2d ref2 = intersection_point - radius1*cv::Vec2d(unit_dir.val[1], -unit_dir.val[0]);

  tangent1 = Line2d(ref1, ref2);
  return 1;
}


int DirectCommonTangentsBetweenCircles(const cv::Vec2d &center1, double radius1, const cv::Vec2d &center2, double radius2, Line2d &tangent1, Line2d &tangent2)
{
  // r1 + r2 < center_dist) 4 tangents
  // r1+r2 = center_dist 3 tangents
  // |r1-r2| < center_dist < r1+r2 2 tangents
  // abs(r1-r2) = centerdist (1 tan)
  // centerdist < abs(r1-r2) no tangent
  const double center_distance = Distance(center1, center2);
//  const double sum_radii = radius1+radius2;

  if (center_distance < std::fabs(radius1 - radius2))
  {
    // C1 is inside C2 (or vice versa) and doesn't intersect.
    return 0;
  }
  if (eps_equal(center_distance, std::fabs(radius1 - radius2)))
  {
    // C1 is inside C2 (or vice versa) and intersect in exactly one point (unless they're the same size).
    if (eps_equal(radius1, radius2))
      VCP_ERROR("DirectCommonTangentsBetweenCircles(): The two circles cannot be the same. Any point on the circle would be a point of tangency!");

    const Line2d center_line(center1, center2);
    const cv::Vec2d unit_dir = center_line.UnitDirection();
    const cv::Vec2d intersection = center1 + radius1 * unit_dir;
    const cv::Vec2d ref1 = intersection + radius1 * cv::Vec2d(unit_dir.val[1], -unit_dir.val[0]);
    const cv::Vec2d ref2 = intersection - radius1 * cv::Vec2d(unit_dir.val[1], -unit_dir.val[0]);
    tangent1 = Line2d(ref1, ref2);
    return 1;
  }

  // In all other cases, there exist two direct common tangents.
  if (eps_equal(radius1, radius2))
  {
    const Line2d center_line(center1, center2);
    const cv::Vec2d dir = radius1 * center_line.UnitDirection();
    const cv::Vec2d orth_dir(dir.val[1], -dir.val[0]);

    // Intersection points are perpendicular to the center line:
    const cv::Vec2d t11 = center1 + orth_dir;
    const cv::Vec2d t12 = center1 - orth_dir;

    const cv::Vec2d t21 = center2 + orth_dir;
    const cv::Vec2d t22 = center2 - orth_dir;

    tangent1 = Line2d(t11, t21);
    tangent2 = Line2d(t12, t22);
    return 2;
  }
  else
  {
    // Similar triangles: (distance c1 to intersection) / (distance c2 to intersection) = radius1 / radius2
    const double distance_intersection = center_distance + center_distance * radius2 / (radius1 - radius2);
    const Line2d center_line(center1, center2);
    const cv::Vec2d intersection = center1 + distance_intersection * center_line.UnitDirection();

    // Now compute the points of tangency w.r.t. both circles.
    cv::Vec2d t11, t12;
    bool pots_exist = PointsOfTangencyPointToCircle(intersection, center1, radius1, t11, t12);
    if (!pots_exist)
      VCP_ERROR("DirectCommonTangentsBetweenCircles(): No points of tangency for intersection point " << intersection << " and circle: " << center1 << ", r=" << radius1);

    cv::Vec2d t21, t22;
    pots_exist = PointsOfTangencyPointToCircle(intersection, center2, radius2, t21, t22);
    if (!pots_exist)
      VCP_ERROR("DirectCommonTangentsBetweenCircles(): No points of tangency for intersection point " << intersection << " and circle: " << center2 << ", r=" << radius2);

    tangent1 = Line2d(t11, t21);
    tangent2 = Line2d(t12, t22);
    return 2;
  }
}
} // namespace geometry2d
} // namespace math
} // namespace vcp
