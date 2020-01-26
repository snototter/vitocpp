#include "conversion/cv_core_conversion.h"
#include "conversion/np_cv_conversion.h"
#include "conversion/vcp_conversion.h"

#include <vcp_math/geometry3d.h>
#include <vcp_math/common.h>

// TODO move to math3d! geo2d::GetProjectionOfHorizon(const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, const cv::Size &image_size)


//-----------------------------------------------------------------------------
// Wrapper/Helper code

namespace vpc = vcp::python::conversion;
namespace geo3d = vcp::math::geo3d;

namespace vcp
{
namespace python
{
namespace math
{
namespace geometry3d
{

py::object Normalize(const cv::Vec3d &v)
{
  double len;
  const cv::Vec3d n = geo3d::Normalize(v, &len);
  if (len > 0.0)
    return py::make_tuple(n[0], n[1], n[2]);
  return py::none();
}

py::object UnitDirectionVector(const cv::Vec3d &from, const cv::Vec3d &to)
{
  return Normalize(geo3d::DirectionVector(from, to));
}


py::object IntersectionLinePlane(const geo3d::Line3d &line, const cv::Vec4d &plane)
{
  cv::Vec3d intersection_point;
  const bool intersects = geo3d::IntersectionLinePlane(line, plane, &intersection_point);
  if (intersects)
    return vpc::VecToTuple<double,3>(intersection_point);
  else
    return py::none();
}

py::object IntersectionLineSegmentPlane(const geo3d::Line3d &line, const cv::Vec4d &plane)
{
  cv::Vec3d intersection_point;
  const bool intersects = geo3d::IntersectionLineSegmentPlane(line, plane, &intersection_point);
  if (intersects)
    return vpc::VecToTuple<double,3>(intersection_point);
  else
    return py::none();
}


py::object ClipLineSegmentByPlane(const geo3d::Line3d &segment, const cv::Vec4d &plane)
{
  geo3d::Line3d clipped = geo3d::ClipLineSegmentByPlane(segment, plane);
  if (clipped.empty())
    return py::none();
  else
    return py::make_tuple(vpc::VecToTuple<double,3>(clipped.From()), vpc::VecToTuple<double,3>(clipped.To()));
}

bool IsPointInFrontOfImagePlane(const cv::Vec3d &pt, const cv::Mat &R, const cv::Mat &t)
{
  return geo3d::IsInFrontOfImagePlane(pt, R, t);
}

py::list ProjectPointsOntoImagePlane(const std::vector<cv::Vec3d> &pts_world,
                                     const cv::Mat &K, const cv::Mat &R, const cv::Mat &t,
                                     const cv::Size &image_size)
{
  const cv::Mat P = geo3d::ProjectionMatrixFromKRt(K, R, t);

  py::list result;
  for (const auto &pt_world : pts_world)
  {
    cv::Vec2d projected;
    if (geo3d::ProjectsOntoImage(pt_world, P, image_size, &projected))
    {
      result.append(vpc::VecToTuple<double,2>(projected));
    }
    else
      result.append(py::none());
  }
  return result;
}

} // namespace geometry3d
} // namespace math
} // namespace python
} // namespace vcp


//-----------------------------------------------------------------------------
// Python module declarations

PYBIND11_MODULE(math3d, m)
{
  namespace vpmg = vcp::python::math::geometry3d;
  m.doc() = "3D-geometry-related math utils.";

  // #######################################################################################
  // Vector stuff

  m.def("cross3d", &geo3d::Cross,
        "3D cross product of the two\n"
        "vectors v = (x,y,z).\n"
        ":return: (x,y,z)",
        py::arg("v1"), py::arg("v2"));

  m.def("dot3d", &geo3d::Dot,
        "3D dot product of the two vectors v = (x,y, z).\n"
        ":return: double",
        py::arg("v1"), py::arg("v2"));

  m.def("direction3d", &geo3d::DirectionVector,
        "Returns the 3d direction vector (x,y,z)",
        py::arg("from"), py::arg("to"));

  m.def("unit_direction3d", &vpmg::UnitDirectionVector,
        "Returns the normalized 3d direction vector (x,y,z)\n"
        "or None if direction vector 'from' to 'to' is (0,0,0)\n",
        py::arg("from"), py::arg("to"));

  m.def("distance3d", &geo3d::Distance,
        "Returns the distance (double) between the two\n"
        "(x,y,z) points.",
        py::arg("v"), py::arg("w"));

  m.def("length3d", &geo3d::Length,
        "Returns the vector's length, vector is given as (x,y,z).",
        py::arg("v"));

  m.def("normalize3d", &vpmg::Normalize,
        "Normalize the vector (x,y,z) to unit length.",
        py::arg("v"));

  m.def("vector_projection", &geo3d::VectorProjection,
        "Returns the vector projection of a onto b.\n"
        "Vector projection is:\n"
        "  unit_vector(b) * scalar_projection(a onto b).\n"
        ":params a,b: 3d vectors, i.e. (x,y,z)\n"
        ":return: projection (x,y,z)",
        py::arg("a"), py::arg("b"));

  m.def("scalar_projection", &geo3d::ScalarProjection,
        "Returns the scalar projection of vector a onto b,\n"
        "i.e. the component of a in the b direction.\n"
        ":params a,b: 3d vectors, i.e. (x,y,z)\n"
        ":return: double",
        py::arg("a"), py::arg("b"));


  m.def("centroid", &geo3d::Centroid,
        "Computes the centroid (mean) of the list of 3D points.\n"
        ":param pts: [(x0,y0,z0), (x1,y1,z1), ...]\n"
        ":return: (x,y,z)", py::arg("points"));

  // #######################################################################################
  // Line stuff

  m.def("intersection_line_plane", &vpmg::IntersectionLinePlane,
        "Returns the intersection point (x,y,z) of the line [(x0,y0,z0), (x1,y1,z1)]\n"
        "with the plane (n0, n1, n2, d) or None if it doesn't exist.",
        py::arg("line"), py::arg("plane"));

  m.def("intersection_line_segment_plane", &vpmg::IntersectionLineSegmentPlane,
        "Returns the intersection point (x,y,z) of the line segment [(x0,y0,z0), (x1,y1,z1)]\n"
        "with the plane (n0, n1, n2, d) or None if it doesn't exist.",
        py::arg("segment"), py::arg("plane"));


  m.def("angle_line_plane", &geo3d::AngleLinePlane,
        "Angle (radians) between line and a plane.",
        py::arg("line"), py::arg("plane"));

  m.def("distance3d_point_line", &geo3d::DistancePointLine,
        "Distance (double) between point and line.",
        py::arg("point"), py::arg("line"));

  m.def("distance3d_point_line_segment", &geo3d::DistancePointLineSegment,
        "Distance (double) between point and line segment.",
        py::arg("point"), py::arg("line"));

  m.def("clip_line_segment_by_plane", &vpmg::ClipLineSegmentByPlane,
        "Clips the line segment such that only parts in front\n"
        "of the plane (watch out for the plane normal!) will be kept.\n"
        "Returns the clipped segment [(x0,y0,z0), (x1,y1,z1)] or None\n"
        "if the segment is completely behind the plane.",
        py::arg("segment"), py::arg("plane"));


  // #######################################################################################
  // Plane stuff
  m.def("plane_from_three_points", &geo3d::PlaneFrom3Points,
        "Returns the plane in Hessian normal form (n0, n1, n2, d) where n is the unit normal.",
        py::arg("p"), py::arg("q"), py::arg("r"));

  m.def("plane_xyz_intercepts", &geo3d::PlaneXYZIntercepts,
        "Returns the x-, y- and z-intercepts of the plane (must be given in Hessian normal form).",
        py::arg("plane"));

  m.def("plane_dihedral_angle", &geo3d::DihedralAngle,
        "Dihedral angle in radians (angle between the two planes).",
        py::arg("plane1"), py::arg("plane2"));

  m.def("distance_point_plane", &geo3d::DistancePointPlane,
        "Signed point to plane distance (positive if point is\n"
        "on the same side as the normal vector of the plane).\n"
        "point = (x,y,z), plane = (n0,n1,n2,d), i.e. Hessian normal form.",
        py::arg("point"), py::arg("plane"));

  m.def("is_point_in_front_of_plane", &geo3d::IsPointInFrontOfPlane,
        "Returns true if point is in front of (or exactly on) the plane.",
        py::arg("point"), py::arg("plane"));

  m.def("ensure_plane_normal_form", &geo3d::EnsurePlaneNormalForm,
        "Ensures that the normal vector of the 4-element Hessian normal form is a unit vector.",
        py::arg("plane"));

  m.def("get_point_on_plane", &geo3d::GetPointOnPlane,
        "Returns an arbitrary point on the plane by checking\n"
        "the axis intercept points. Will first text the x-intercept\n"
        "and return it unless the plane rotates about the x-axis, then\n"
        "the y-intercept, etc.", py::arg("plane"));

  // No need to wrap, easier in python: geo3d::MakePlane();
  // No need to wrap, easier in python: geo3d::PlaneNormal();


  // #######################################################################################
  // Camera geometry stuff
  //TODO ambiguous c++: geo3d::CameraCenterFromRt();
//  geo3d::GroundplaneToImageHomographyFromP();
//  geo3d::ImagePlaneInWorldCoordinateSystem();
//  geo3d::ImageToGroundplaneHomographyFromP();

  m.def("is_point_in_front_of_image_plane", &vpmg::IsPointInFrontOfImagePlane,
        "Returns true if the world point lies in front of the image plane.\n"
        ":param pt: (x,y,z)\n"
        ":param R: 3x3 camera rotation matrix\n"
        ":param t: 3x1 camera translation or (tx,ty,tz)\n"
        ":return: True/False",
        py::arg("pt"), py::arg("R"), py::arg("t"));


  m.def("project_world_points_onto_image", &vpmg::ProjectPointsOntoImagePlane,
        "Project the given 3d points onto the image.\n"
        ":param pts_world: list of 3d points, e.g. [(x0,y0,z0), ...]\n"
        ":param K: 3x3 camera intrinsics\n"
        ":param R: 3x3 camera rotation\n"
        ":param t: 3x1 camera translation or (tx,ty,tz)\n"
        ":param image_size: (w,h) to determine whether a projection\n"
        "                   lies within the image boundaries\n"
        ":return: list of 2d points",
        py::arg("pts_world"), py::arg("K"), py::arg("R"), py::arg("t"),
        py::arg("image_size"));


  //TODO  bpy::def("euler_angles_from_rotation_matrix", &ppm3::RotationMatrixToEulerAngles);
//  geo3d::RotationMatrixToEulerAngles(cvR)

  // #######################################################################################
  // Projection stuff
  // Binding these is not needed, this stuff is available in pure python (camera_projection_utils)
//  geo3d::Apply3x3();
//  geo3d::Apply3x4();
//  geo3d::ProjectVec(); // (x,y,w) = P*input; ret (x/w, y/w)
//  geo3d::ProjectionMatrixFromKRt();
}
