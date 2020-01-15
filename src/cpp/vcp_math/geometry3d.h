#ifndef __VCP_MATH_GEOMETRY3D_H__
#define __VCP_MATH_GEOMETRY3D_H__

#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>

// TODO document: a plane is given by [a,b,c,p] where unit_normal=[a,b,c] and all points on the plane satisfy: dot(unit_normal, point_position_vector) = -p

namespace vcp
{
namespace math
{
namespace geo3d
{
/** @brief Capsulates lines and line segments. */
class Line3d
{
public:
  Line3d() : empty_(true), pt_from_(), pt_to_() {}

  /** @brief Construct a line from 2 points. In case of a segment, these are the start and end points. */
  Line3d(const cv::Vec3d &from, const cv::Vec3d &to) : empty_(false), pt_from_(from), pt_to_(to) {}

  /** @brief Returns a line with flipped direction vector. */
  Line3d Flipped() const;

  /** @brief For a segment, this returns the start point. For a line its one of the given points to construct the line in the first place. */
  const cv::Vec3d &From() const;

  /** @brief For a segment, this returns the end point. For a line its one of the given points to construct the line in the first place. */
  const cv::Vec3d &To() const;

  /** @brief Returns the direction vector (not normalized!) from the start point to the end point. */
  cv::Vec3d Direction() const;

  /** @brief Returns the unit direction vector from the start point to the end point. */
  cv::Vec3d UnitDirection() const;

  /** @brief Returns the point halfway between from and to. */
  cv::Vec3d MidPoint() const;

  /** @brief Returns true, if the line object is empty, i.e. not yet set, or from == to. */
  bool empty() const;


  friend std::ostream &operator<< (std::ostream &stream, const Line3d &line);
private:
  bool empty_;
  cv::Vec3d pt_from_;
  cv::Vec3d pt_to_;
};


/** @brief Wrapper class if you want to have a plane with coordinate center, its own coordinate frame, etc.
  * Usually, you just want to use the 4-element vector (Hessian normal form) to work with planes.
  */
class Plane3d
{
public:
  Plane3d() : empty_(true) {}
  Plane3d(const cv::Vec4d &normal_form, const cv::Vec3d &origin, const cv::Vec3d &e1, const cv::Vec3d &e2);

  /** @brief Returns the 2D coordinates of the 3D point projected onto this plane. */
  cv::Vec2d ProjectOntoPlane(const cv::Vec3d &pt) const;

  /** @brief Computes the intersection point of the line segment with this plane. If a polygon was set (at least 3 points), additionally
   * checks whether the line_segment|plane intersection lies within the polygon!
   */
  bool IntersectionLineSegment(const Line3d &segment, cv::Vec3d *intersection = nullptr) const;

  /** @brief Distance from plane (not ROI!) to point. */
  double DistanceToPoint(const cv::Vec3d &pt) const;

  /** @brief Returns true, if the plane object is empty, i.e. not valid. */
  bool empty() const { return empty_; }

  /** @brief Add a point to the internal polygon (used for @see IsInsidePolygon). */
  void AddPolygonPoint(const cv::Vec3d &pt);

  /** @brief Add a point to the internal polygon (used for @see IsInsidePolygon). */
  void AddPolygonPoints(const std::vector<cv::Vec3d> &pts);

  /** @brief Checks whether the given point is inside the polygon. */
  bool IsInsideClosedPolygon(const cv::Vec3d &pt) const;

  cv::Vec3d Origin() const { return origin_; }

  cv::Vec3d Normal() const;

  std::vector<cv::Vec3d> Polygon3d() const;

  /** @brief Returns the angle between v and the plane normal. */
  double Angle(const cv::Vec3d &v) const;

  friend std::ostream &operator<< (std::ostream &stream, const Plane3d &line);
private:
  bool empty_;
  cv::Vec4d normal_form_;
  cv::Vec3d origin_;
  cv::Vec3d e1_;
  cv::Vec3d e2_;
  std::vector<cv::Vec2d> polygon_; /** A polygon on the plane, can be used for inside/outside tests. */
};


/** @brief Returns the direction vector from 'from' to 'to'. */
inline cv::Vec3d DirectionVector(const cv::Vec3d &from, const cv::Vec3d &to)
{
  return to - from;
}


/** @brief Returns the squared length of the vector. */
inline double LengthSquared(const cv::Vec3d &v)
{
  return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
}


/** @brief Returns the length of the vector. */
inline double Length(const cv::Vec3d &v)
{
  return std::sqrt(LengthSquared(v));
}


/** @brief Returns the Euclidean distance between the two vectors. */
inline double Distance(const cv::Vec3d &v, const cv::Vec3d &w)
{
  return Length(v-w);
}


/** @brief Returns the unit vector and optionally sets the length to the original vector magnitude (before normalization). */
inline cv::Vec3d Normalize(const cv::Vec3d &v, double *length=nullptr)
{
  const double len = Length(v);
  if (length)
    *length = len;

  if (len > 0.0)
    return cv::Vec3d(v[0]/len, v[1]/len, v[2]/len);
  else
    return v;
}


/** @brief Dot product. */
inline double Dot(const cv::Vec3d &v, const cv::Vec3d &w)
{
  return v[0]*w[0] + v[1]*w[1] + v[2]*w[2];
}

/** @brief Cross product. */
inline cv::Vec3d Cross(const cv::Vec3d &a, const cv::Vec3d &b)
{
  return cv::Vec3d(a[1]*b[2] - a[2]*b[1],
      a[2]*b[0] - a[0]*b[2],
      a[0]*b[1]-a[1]*b[0]);
}

/** @brief Extracts the plane normal from the 4-element plane representation. Note: we assume Hessian normal form - thus the plane normal is already a unit vector. */
inline cv::Vec3d PlaneNormal(const cv::Vec4d &plane)
{
  return cv::Vec3d(plane[0], plane[1], plane[2]);
}


/** @brief Convenience function to build a plane representation. */
inline cv::Vec4d MakePlane(const cv::Vec3d &unit_normal, double d)
{
  return cv::Vec4d(unit_normal[0], unit_normal[1], unit_normal[2], d);
}

/** @brief Ensures that the normal vector of the 4-element Hessian normal form is a unit vector. */
cv::Vec4d EnsurePlaneNormalForm(const cv::Vec4d &plane);


/** @brief Returns the plane in Hessian normal form (n,d) where n is the unit normal. */
cv::Vec4d PlaneFrom3Points(const cv::Vec3d &p, const cv::Vec3d &q, const cv::Vec3d &r);


/** @brief Returns the x-, y- and z-intercepts of the plane (must be given in Hessian normal form). */
cv::Vec3d PlaneXYZIntercepts(const cv::Vec4d &plane);


/** @brief Returns an arbitrary point on the plane by checking the axis intercept points.
 * Will first return x-intercept and return unless the plane rotates about the x-axis, then the y-intercept, ...
 */
cv::Vec3d GetPointOnPlane(const cv::Vec4d &plane);


/** @brief Dihedral angle in radians (angle between the two planes). */
double DihedralAngle(const cv::Vec4d &plane1, const cv::Vec4d &plane2);


/** @brief Angle (radians) between line and a plane. */
double AngleLinePlane(const Line3d &line, const cv::Vec4d &plane);


/** @brief Signed point to plane distance (positive if point is on the same side as the normal vector of the plane). */
double DistancePointPlane(const cv::Vec3d &point, const cv::Vec4d &plane);


/** @brief Returns true if point is in front of (or exactly on) the plane. */
bool IsPointInFrontOfPlane(const cv::Vec3d &point, const cv::Vec4d &plane);


/** @brief Distance between point and line. */
double DistancePointLine(const cv::Vec3d &point, const Line3d &line);


/** @brief Distance between point and line segment. */
double DistancePointLineSegment(const cv::Vec3d &point, const Line3d &line_segment);


/** @brief Returns the vector projection of vector a onto b. */
cv::Vec3d VectorProjection(const cv::Vec3d &a, const cv::Vec3d &b);


/** @brief Returns the scalar projection of vector a onto b, i.e. the component of a in the b direction (or the magnitude of the vector projection of a onto b). */
double ScalarProjection(const cv::Vec3d &a, const cv::Vec3d &b);


/** @brief Check if line intersects the plane (must be in Hessian normal form). Optionally returns the intersection point. */
bool IntersectionLinePlane(const Line3d &line, const cv::Vec4d &plane, cv::Vec3d *intersection_point=nullptr);


/** @brief Check if the line segment intersects the plane (given in Hessian normal form). Optionally returns the intersection point. */
bool IntersectionLineSegmentPlane(const Line3d &line_segment, const cv::Vec4d &plane, cv::Vec3d *intersection_point=nullptr);


/** @brief Returns the 3 Euler angles from the given rotation matrix. */
cv::Vec3d RotationMatrixToEulerAngles(const cv::Mat &R);


/** @brief Computes x_3x1 = P_3x4 * pt_3x1. */
inline cv::Vec3d Apply3x4(const cv::Mat &P, const cv::Vec3d &pt)
{
  const double x = P.at<double>(0,0) * pt[0] + P.at<double>(0,1) * pt[1] + P.at<double>(0,2) * pt[2] + P.at<double>(0,3);
  const double y = P.at<double>(1,0) * pt[0] + P.at<double>(1,1) * pt[1] + P.at<double>(1,2) * pt[2] + P.at<double>(1,3);
  const double z = P.at<double>(2,0) * pt[0] + P.at<double>(2,1) * pt[1] + P.at<double>(2,2) * pt[2] + P.at<double>(2,3);
  return cv::Vec3d(x, y, z);
}


/** @brief Computes x_3x1 = H_3x3 * pt_3x3. */
inline cv::Vec3d Apply3x3(const cv::Mat &H, const cv::Vec3d &pt)
{
  const double x = H.at<double>(0,0) * pt[0] + H.at<double>(0,1) * pt[1] + H.at<double>(0,2) * pt[2];
  const double y = H.at<double>(1,0) * pt[0] + H.at<double>(1,1) * pt[1] + H.at<double>(1,2) * pt[2];
  const double z = H.at<double>(2,0) * pt[0] + H.at<double>(2,1) * pt[1] + H.at<double>(2,2) * pt[2];
  return cv::Vec3d(x, y, z);
}

//TODO doc
cv::Mat ProjectionMatrixFromKRt(const cv::Mat &K, const cv::Mat &R, const cv::Mat &t);
cv::Mat ProjectionMatrixFromKRt(const cv::Mat &K, const cv::Mat &Rt);
cv::Vec3d CameraCenterFromRt(const cv::Mat &R, const cv::Mat &t);
cv::Vec3d CameraCenterFromRt(const cv::Mat &Rt);


/** @brief Project a 3-element Vector by a 3x4 projection matrix (by adding a homogeneous coordinate dimension)
 * or 3x3 transformation matrix (homography). In the latter case, you have to already provide the pt in
 * homogeneous form.
 */
inline cv::Vec2d ProjectVec(const cv::Mat &P, const cv::Vec3d &pt)
{
  const cv::Vec3d prj = (P.cols == 4) ? Apply3x4(P, pt) : Apply3x3(P, pt);
  return cv::Vec2d(prj[0]/prj[2], prj[1]/prj[2]);
}


/** @brief Extract the ground plane to image plane homography from the 3x4 camera projection matrix. */
inline cv::Mat GroundplaneToImageHomographyFromP(const cv::Mat &P)
{
  // H_gp2cam = [p1, p2, p4], pi is the i-th column of P
  cv::Mat H(3, 3, P.type());
  P.col(0).copyTo(H.col(0));
  P.col(1).copyTo(H.col(1));
  P.col(3).copyTo(H.col(2));
  return H;
}


/** @brief Extract the image plane to ground plane homography from the 3x4 camera projection matrix. */
inline cv::Mat ImageToGroundplaneHomographyFromP(const cv::Mat &P)
{
  const cv::Mat H_gp2cam = GroundplaneToImageHomographyFromP(P);
  return H_gp2cam.inv();
}


/** @brief Compute the Hessian normal form of the image plane given the camera's extrinsic parameters. */
cv::Vec4d ImagePlaneInWorldCoordinateSystem(const cv::Mat &R, const cv::Mat &t);


/** @brief Clips the line segment such that only parts in front of the plane (watch out for the plane normal!) will be kept. */
Line3d ClipLineSegmentByPlane(const Line3d &line_segment, const cv::Vec4d &plane);


/** @brief Computes the centroid (mean) of the given 3D points. */
cv::Vec3d Centroid(const std::vector<cv::Vec3d> &points);


/** @brief Returns true if the world point lies in front of the image plane. */
bool IsInFrontOfImagePlane(const cv::Vec3d &pt_world, const cv::Mat &Rt);


/** @brief Wrapper to @see IsInFrontOfImagePlane in case you store extrinsics as R and t separately. */
bool IsInFrontOfImagePlane(const cv::Vec3d &pt_world, const cv::Mat &R, const cv::Mat &t);


/** @brief Returns true if the world point would project onto the camera's image. If projected is a valid pointer, it will be set (in case it projects onto the image only). */
bool ProjectsOntoImage(const cv::Vec3d &pt_world, const cv::Mat &P, const cv::Size &image_size, cv::Vec2d *projected=nullptr);


/** @brief Wrapper to @see ProjectsOntoImage */
bool ProjectsOntoImage(const cv::Vec3d &pt_world, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, const cv::Size &image_size, cv::Vec2d *projected=nullptr);

} // namespace geometry3d
} // namespace math
} // namespace vcp

#endif // __VCP_MATH_GEOMETRY3D_H__
