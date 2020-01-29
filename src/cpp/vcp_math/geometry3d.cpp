#include "geometry3d.h"
#include "geometry2d.h"
#include "common.h"
#include "conversions.h"
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>

// Plane stuff based on tutorials and summaries at:
// * http://mathworld.wolfram.com/Plane.html
// * http://tutorial.math.lamar.edu/Classes/CalcIII/EqnsOfPlanes.aspx
// * http://www.easy-math.net/transforming-between-plane-forms/


#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::math::geo3d"

namespace vcp
{
namespace math
{
namespace geo3d
{
Line3d Line3d::Flipped() const
{
  return Line3d(this->pt_to_, this->pt_from_);
}

const cv::Vec3d &Line3d::From() const
{
  return pt_from_;
}

const cv::Vec3d &Line3d::To() const
{
  return pt_to_;
}

cv::Vec3d Line3d::Direction() const
{
  return DirectionVector(pt_from_, pt_to_);
}

cv::Vec3d Line3d::UnitDirection() const
{
  return Normalize(DirectionVector(pt_from_, pt_to_));
}

cv::Vec3d Line3d::MidPoint() const
{
  return 0.5 * (pt_from_ + pt_to_);
}

bool Line3d::empty() const
{
  return empty_ || vcp::math::IsVecEqual(pt_from_, pt_to_);
}

std::ostream &operator<< (std::ostream &stream, const Line3d &line)
{
  stream << line.pt_from_ << "-->" << line.pt_to_;
  return stream;
}

Plane3d::Plane3d(const cv::Vec4d &normal_form, const cv::Vec3d &origin, const cv::Vec3d &e1, const cv::Vec3d &e2) :
  empty_(false), normal_form_(EnsurePlaneNormalForm(normal_form)), origin_(origin), e1_(e1), e2_(e2)
{}

cv::Vec2d Plane3d::ProjectOntoPlane(const cv::Vec3d &pt) const
{
  //Based on https://stackoverflow.com/a/23474396/400948
  // r_P = (x,y,z)
  // r_O = (ox, oy, oz)
  const cv::Vec3d vec = pt - origin_;

//  const double s = Dot(PlaneNormal(normal_form_), vec);
  const double x = Dot(e1_, vec);
  const double y = Dot(e2_, vec);
  return cv::Vec2d(x, y);
}

bool Plane3d::IntersectionLineSegment(const Line3d &segment, cv::Vec3d *intersection) const
{
  cv::Vec3d ip;
  const bool segment_intersects = IntersectionLineSegmentPlane(segment, normal_form_, &ip);

  if (intersection)
    *intersection = ip;

  if (segment_intersects && polygon_.size() >= 3)
    return IsInsideClosedPolygon(ip);
  else
    return segment_intersects;
}

double Plane3d::DistanceToPoint(const cv::Vec3d &pt) const
{
  return DistancePointPlane(pt, normal_form_);
}

void Plane3d::AddPolygonPoint(const cv::Vec3d &pt)
{
  polygon_.push_back(ProjectOntoPlane(pt));
}

void Plane3d::AddPolygonPoints(const std::vector<cv::Vec3d> &pts)
{
  for (const cv::Vec3d &pt : pts)
    AddPolygonPoint(pt);
}

bool Plane3d::IsInsideClosedPolygon(const cv::Vec3d &pt) const
{
  return vcp::math::geo2d::IsPointInClosedPolygon(ProjectOntoPlane(pt), polygon_);
}


std::vector<cv::Vec3d> Plane3d::Polygon3d() const
{
  std::vector<cv::Vec3d> pts;
  pts.reserve(polygon_.size());
  for (const cv::Vec2d &v : polygon_)
    pts.push_back(origin_ + v[0] * e1_ + v[1] * e2_);
  return pts;
}

double Plane3d::Angle(const cv::Vec3d &v) const
{
  return std::acos(Dot(Normalize(v), Normal()));
}

cv::Vec3d Plane3d::Normal() const
{
  return PlaneNormal(normal_form_);
}

std::ostream &operator<< (std::ostream &stream, const Plane3d &plane)
{
  stream << plane.normal_form_ << ", origin " << plane.origin_ << ", 2D reference frame " << plane.e1_ << "x" << plane.e2_ << ", " << plane.polygon_.size() << " points in ROI";
  return stream;
}


cv::Vec4d EnsurePlaneNormalForm(const cv::Vec4d &plane)
{
  return MakePlane(Normalize(PlaneNormal(plane)), plane[3]);
}


cv::Vec4d PlaneFrom3Points(const cv::Vec3d &p, const cv::Vec3d &q, const cv::Vec3d &r)
{
  const auto pq = DirectionVector(p,q);
  const auto pr = DirectionVector(p,r);
  double len;
  const auto n = Normalize(pq.cross(pr), &len);

  if (vcp::math::eps_equal(len, 0.0))
    VCP_LOG_WARNING("Cannot deduce a valid plane, points are collinear!");

  const double d = -Dot(n, p);

  return cv::Vec4d(n[0], n[1], n[2], d);
}


cv::Vec3d PlaneXYZIntercepts(const cv::Vec4d &plane)
{
  const double x = vcp::math::eps_equal(plane[0], 0.0) ? 0.0 : -plane[3]/plane[0];
  const double y = vcp::math::eps_equal(plane[1], 0.0) ? 0.0 : -plane[3]/plane[1];
  const double z = vcp::math::eps_equal(plane[2], 0.0) ? 0.0 : -plane[3]/plane[2];
  return cv::Vec3d(x, y, z);
}

cv::Vec3d GetPointOnPlane(const cv::Vec4d &plane)
{
  if (!vcp::math::eps_equal(plane[0], 0.0))
    return cv::Vec3d(-plane[3]/plane[0], 0.0, 0.0);
  if (!vcp::math::eps_equal(plane[1], 0.0))
    return cv::Vec3d(0.0, -plane[3]/plane[1], 0.0);
  if (!vcp::math::eps_equal(plane[2], 0.0))
    return cv::Vec3d(0.0, 0.0, -plane[3]/plane[2]);
  VCP_ERROR("GetPointOnPlane(): Invalid plane " << plane << ", cannot compute a point on it!");
}


double DihedralAngle(const cv::Vec4d &plane1, const cv::Vec4d &plane2)
{
  // cos theta = n1 dot n2
  return std::acos(Dot(PlaneNormal(plane1), PlaneNormal(plane2)));
}


//http://onlinemschool.com/math/library/analytic_geometry/plane_line/
double AngleLinePlane(const Line3d &line, const cv::Vec4d &plane)
{
  const cv::Vec3d unit_dir = Normalize(line.Direction());
  const cv::Vec3d unit_normal = Normalize(PlaneNormal(plane));
  return std::asin(Dot(unit_dir, unit_normal));
}


double DistancePointPlane(const cv::Vec3d &point, const cv::Vec4d &plane)
{
  return Dot(PlaneNormal(plane), point) + plane[3];
}

bool IsPointInFrontOfPlane(const cv::Vec3d &point, const cv::Vec4d &plane)
{
  return DistancePointPlane(point, plane) >= 0.0;
}


double DistancePointLine(const cv::Vec3d &point, const Line3d &line)
{
  // See also VectorProjection(), i.e. scalar_projection times unit direction vector.
  // http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
  // https://math.stackexchange.com/questions/1905533/find-perpendicular-distance-from-point-to-line-in-3d

  // Vector from line start to point:
  const cv::Vec3d v = DirectionVector(line.From(), point);

  // Project onto line and get closest point on line:
  const cv::Vec3d unit_direction = Normalize(line.Direction());
  const double lambda = Dot(unit_direction, v);
  const cv::Vec3d closest_point = line.From() + lambda * unit_direction;
  return Distance(point, closest_point);
}


double DistancePointLineSegment(const cv::Vec3d &point, const Line3d &line_segment)
{
  // See also VectorProjection(), i.e. scalar_projection times unit direction vector.
  // Nice formalism: https://math.stackexchange.com/questions/330269/the-distance-from-a-point-to-a-line-segment

  // Compute lambda s.t. l=0: start point, l=1: end point of the segment.
  const cv::Vec3d line_direction = line_segment.Direction();
  const double lambda = Dot(point - line_segment.From(), line_direction) / LengthSquared(line_direction);

  const cv::Vec3d closest_point = line_segment.From() + std::min(1.0, std::max(0.0, lambda)) * line_direction;
  return Distance(point, closest_point);
}

cv::Vec3d VectorProjection(const cv::Vec3d &a, const cv::Vec3d &b)
{
  // https://math.oregonstate.edu/home/programs/undergrad/CalculusQuestStudyGuides/vcalc/dotprod/dotprod.html
  // https://en.wikipedia.org/wiki/Vector_projection

  // Vector projection is the unit vector b times the scalar projection of a onto b:
  //return ScalarProjection(a, b) * Normalize(b); is the same (result, but more computation) as:
  return Dot(a,b) / LengthSquared(b) * b;
}

double ScalarProjection(const cv::Vec3d &a, const cv::Vec3d &b)
{
  // https://en.wikipedia.org/wiki/Vector_projection
  return Dot(a, Normalize(b));
}


bool IntersectionLinePlane(const Line3d &line, const cv::Vec4d &plane, cv::Vec3d *intersection_point)
{
  // Project ray onto plane normal
  const cv::Vec3d unit_normal = PlaneNormal(plane);
  const cv::Vec3d line_direction = line.Direction();
  const double prj = Dot(unit_normal, line_direction);

  if (vcp::math::eps_equal(prj, 0.0))
  {
    // The plane is parallel to the line segment. Check if it's on the plane.
    const double dist = DistancePointPlane(line.From(), plane);
    if (std::fabs(dist) > 0.0)
      return false;
    // All line points are on the plane.
    if (intersection_point)
      *intersection_point = line.From();
  }
  else
  {
    if (intersection_point)
    {
      const double lambda = (-plane[3] - Dot(unit_normal, line.From())) / prj;
      *intersection_point = line.From() + lambda * line_direction;
    }
  }

  return true;
}

bool IntersectionLineSegmentPlane(const Line3d &line_segment, const cv::Vec4d &plane, cv::Vec3d *intersection_point)
{
  // Each point x on the plane satisfies dot(N,x) = -p, where N is the unit normal and p is the plane offset.
  // Each point x on the line satisfies x = from + lambda * dir, where dir = to-from
  // If we know that the segment crosses the plane (lie on different sides => check signed distance), we can
  // plug in:
  //   dot(N, from + lambda*dir) = -p
  //   N^t*from + lambda*N^t*dir = -p
  //   lambda = (-p - dot(N,from)) / dot(N,dir)
  //
  // Helpful resources:
  // http://mathworld.wolfram.com/Line-PlaneIntersection.html
  // https://math.stackexchange.com/questions/47594/plane-intersecting-line-segment
  // https://stackoverflow.com/questions/7168484/3d-line-segment-and-plane-intersection
  const double d1 = DistancePointPlane(line_segment.From(), plane);
  const double d2 = DistancePointPlane(line_segment.To(), plane);

  if (d1 * d2 > 0.0) // Signs are the same, both points are on the same side of the plane
    return false;

  if (intersection_point)
  {
    // Project ray onto plane normal
    const cv::Vec3d unit_normal = PlaneNormal(plane);
    const cv::Vec3d line_direction = line_segment.Direction();
    const double prj = Dot(unit_normal, line_direction);

    if (vcp::math::eps_equal(prj, 0.0))
    {
      // The plane is parallel to the line segment - since we checked the distance
      // before, this means that the line lies on the plane.
      *intersection_point = line_segment.From();
    }
    else
    {
      const double lambda = (-plane[3] - Dot(unit_normal, line_segment.From())) / prj;
      *intersection_point = line_segment.From() + lambda * line_direction;
    }
  }
  return true;
}


cv::Vec3d RotationMatrixToEulerAngles(const cv::Mat &R)
{
  VCP_CHECK(R.depth() == CV_64F);
  VCP_CHECK(R.rows == 3 && R.cols == 3);

  double e1, e2, e3;
  // Adapted from https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-rotation-matrix
  if (vcp::math::eps_equal(std::fabs(R.at<double>(0,2)), 1.0, 2)) // -1 or + 1
  {
    // Special case, exactly +/-90 deg rotation around y-axis
    const double
      e3 = 0.0; // Set arbitrarily
      const double delta = std::atan2(R.at<double>(0,1), R.at<double>(0,2));
      if (vcp::math::eps_equal(std::fabs(R.at<double>(0,2)), -1.0, 2))
      {
        e2 = MATH_PI/2.0;
        e1 = e3 + delta;
      }
      else
      {
        e2 = -MATH_PI/2.0;
        e1 = -e3 + delta;
      }
  }
  else
  {
    e2 = -std::asin(R.at<double>(0,2));
    const double c2 = std::cos(e2);
    e1 = std::atan2(R.at<double>(1,2)/c2, R.at<double>(2,2)/c2);
    e3 = std::atan2(R.at<double>(0,1)/c2, R.at<double>(0,0)/c2);
  }
  return cv::Vec3d(e1, e2, e3);
}


cv::Mat RotationX(double theta, bool angles_in_deg)
{
  const double rad = angles_in_deg ? vcp::math::Deg2Rad(theta) : theta;
  const double ct = std::cos(rad);
  const double st = std::sin(rad);
  return (cv::Mat_<double>(3, 3)
          << 1.0, 0.0, 0.0,
          0.0, ct, -st,
          0.0, st, ct);
}

cv::Mat RotationY(double theta, bool angles_in_deg)
{
  const double rad = angles_in_deg ? vcp::math::Deg2Rad(theta) : theta;
  const double ct = std::cos(rad);
  const double st = std::sin(rad);
  return (cv::Mat_<double>(3, 3)
          << ct, 0.0, st,
          0.0, 1.0, 0.0,
          -st, 0.0, ct);
}

cv::Mat RotationZ(double theta, bool angles_in_deg)
{
  const double rad = angles_in_deg ? vcp::math::Deg2Rad(theta) : theta;
  const double ct = std::cos(rad);
  const double st = std::sin(rad);
  return (cv::Mat_<double>(3, 3)
          << ct, -st, 0.0,
          st, ct, 0.0,
          0.0, 0.0, 1.0);
}

cv::Mat RotationMatrix(double theta_x, double theta_y, double theta_z, bool angles_in_deg)
{
  const cv::Mat Rx = RotationX(theta_x, angles_in_deg);
  const cv::Mat Ry = RotationY(theta_y, angles_in_deg);
  const cv::Mat Rz = RotationZ(theta_z, angles_in_deg);
  return Rx * (Ry * Rz);
}


cv::Mat ProjectionMatrixFromKRt(const cv::Mat &K, const cv::Mat &R, const cv::Mat &t)
{
  // Don't use imutils (this would cause a cyclic library/module dependency)
  cv::Mat Rt(3, 4, t.type());
  cv::Mat roi = Rt.colRange(0, 3);
  R.copyTo(roi);
  roi = Rt.col(3);
  t.copyTo(roi);
  return K * Rt;
}

cv::Mat ProjectionMatrixFromKRt(const cv::Mat &K, const cv::Mat &Rt)
{
  return K * Rt;
}


cv::Vec3d CameraCenterFromRt(const cv::Mat &R, const cv::Mat &t)
{
  return vcp::convert::Mat2Vec<double,3>(-R.t() * t);
}


cv::Vec3d CameraCenterFromRt(const cv::Mat &Rt)
{
  return CameraCenterFromRt(Rt.colRange(0, 3), Rt.col(3));
}


cv::Vec4d ImagePlaneInWorldCoordinateSystem(const cv::Mat &R, const cv::Mat &t)
{
  // Camera looks along the positive z-axis.
  // Get the distance between the image plane and the world origin (in camera coordinates).
  // World origin in camera coordinates is t = -RC = [R|t] (0,0,0,1).
  // Image plane in camera coordinates is the xy plane, at z=1. Thus, its Hessian form is
  // [0,0,1,-1] (distance=-1 because the camera center (origin) is *behind* the image plane!
  const cv::Vec3d unit_vec_z(0.0, 0.0, 1.0);
  const double distance = DistancePointPlane(vcp::convert::ToVec3d(t), MakePlane(unit_vec_z, -1.0));
  // Rotate plane normal to express it in the world reference frame:
  const cv::Mat Rinv = R.t();
  cv::Vec3d plane_normal = Apply3x3(Rinv, unit_vec_z); // In world reference
  return MakePlane(plane_normal, distance);
}


Line3d ClipLineSegmentByPlane(const Line3d &line_segment, const cv::Vec4d &plane)
{
  const bool from_in_front = IsPointInFrontOfPlane(line_segment.From(), plane);
  const bool to_in_front = IsPointInFrontOfPlane(line_segment.To(), plane);

  if (from_in_front && to_in_front)
  {
    // Both in front of plane
    return line_segment;
  }

  if (!from_in_front && !to_in_front)
  {
    // Both behind the plane, so clipping yields no line:
    return Line3d();
  }

  // One point is in front, the other behind - so we need to clip it:
  // Project ray onto plane normal
  const cv::Vec3d unit_normal = PlaneNormal(plane);
  const cv::Vec3d line_direction = line_segment.Direction();
  const double prj = Dot(unit_normal, line_direction);

  if (vcp::math::eps_equal(prj, 0.0))
  {
    // The plane is parallel to the line segment. Here, this can only mean
    // that the segment lies exactly on the plane (and rounding issues prevented
    // us from detecting this case, because _in_front should've been true for
    // both end points.
    return line_segment;
  }
  // Compute the intersection.
  const double lambda = (-plane[3] - Dot(unit_normal, line_segment.From())) / prj;
  const cv::Vec3d intersection_point = line_segment.From() + lambda * line_direction;

  // Replace the point behind the plane by the intersection:
  if (from_in_front)
    return Line3d(line_segment.From(), intersection_point);
  else
    return Line3d(intersection_point, line_segment.To());
}


cv::Vec3d Centroid(const std::vector<cv::Vec3d> &points)
{
  // TODO we should check for potential overflow:
  // https://stackoverflow.com/a/15655590/400948
  cv::Vec3d sum(0.0, 0.0, 0.0);
  for (const cv::Vec3d &v : points)
    sum += v;
  return sum / static_cast<double>(points.size());
}


bool IsInFrontOfImagePlane(const cv::Vec3d &pt_world, const cv::Mat &Rt)
{
  const cv::Vec3d pt_cam = Apply3x4(Rt, pt_world);
  // Image plane in camera reference system:
  const cv::Vec4d image_plane(0.0, 0.0, 1.0, -1.0);
  return IsPointInFrontOfPlane(pt_cam, image_plane);
}


bool IsInFrontOfImagePlane(const cv::Vec3d &pt_world, const cv::Mat &R, const cv::Mat &t)
{
  // Concatenate R and t (we cannot use vcp::imutils::ColumnStack due to cyclic build dependencies)
  cv::Mat Rt(3, 4, R.type());
  cv::Mat roi = Rt.colRange(0, 3);
  R.copyTo(roi);
  roi = Rt.colRange(3, 4);
  t.copyTo(roi);
  return IsInFrontOfImagePlane(pt_world, Rt);
}


bool ProjectsOntoImage(const cv::Vec3d &pt_world, const cv::Mat &P, const cv::Size &image_size, cv::Vec2d *projected)
{
  const cv::Vec2d pt_img = ProjectVec(P, pt_world);
  if (projected)
  {
    projected->val[0] = pt_img[0];
    projected->val[1] = pt_img[1];
  }
  return geo2d::IsPointInsideRectangle(pt_img, cv::Rect2d(0.0, 0.0, image_size.width, image_size.height));
}


bool ProjectsOntoImage(const cv::Vec3d &pt_world, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, const cv::Size &image_size, cv::Vec2d *projected)
{
  const cv::Mat P = ProjectionMatrixFromKRt(K, R, t);
  return ProjectsOntoImage(pt_world, P, image_size, projected);
}

} // namespace geometry3d
} // namespace math
} // namespace vcp
