#include "conversion/cv_core_conversion.h"
#include "conversion/np_cv_conversion.h"
#include "conversion/vcp_conversion.h"

#include <vcp_math/geometry2d.h>
#include <vcp_math/common.h>
#include <vcp_utils/vcp_error.h>


// TODO move to math3d! geo2d::GetProjectionOfHorizon(const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, const cv::Size &image_size)
// TODO nice-to-have: reconstruct triangle from two sides, angles, etc.


//-----------------------------------------------------------------------------
// Wrapper/Helper code

namespace vpc = vcp::python::conversion;
namespace geo2d = vcp::math::geo2d;

namespace vcp
{
namespace python
{
namespace math
{
namespace geometry2d
{

// Needed due to ambiguous overloads (at least for pybind).
std::vector<cv::Vec2d> ConvexHull(const std::vector<cv::Vec2d> &points, bool check_for_duplicates, double scale)
{
  return geo2d::ConvexHullGrahamScan(points, check_for_duplicates, scale);
}

py::tuple CircleFrom3Points(const cv::Vec2d &pt1, const cv::Vec2d &pt2, const cv::Vec2d &pt3)
{
  cv::Vec2d center;
  double radius;

  const bool exists = geo2d::CircleFrom3Points(pt1, pt2, pt3, center, radius);
  if (exists)
    return py::make_tuple(true, center, radius);
  return py::make_tuple(false, py::none(), py::none());
}

py::tuple IsPointInCircle(const cv::Vec2d &pt, const cv::Vec2d &center, double radius)
{
  bool is_on_circle;
  const bool inside = geo2d::IsPointInCircle(pt, center, radius, &is_on_circle);
  return py::make_tuple(inside, is_on_circle);
}



py::object IntersectionLineLine(const geo2d::Line2d &line1, const geo2d::Line2d &line2)
{
  cv::Vec2d pt;
  const bool intersects = geo2d::IntersectionLineLine(line1, line2, pt);
  if (intersects)
    return vpc::VecToTuple(pt);
  return py::none();
}


py::object IntersectionLineLineSegment(const geo2d::Line2d &line, const geo2d::Line2d &segment)
{
  cv::Vec2d pt;
  const bool intersects = geo2d::IntersectionLineLineSegment(line, segment, pt);
  if (intersects)
    return vpc::VecToTuple(pt);
  return py::none();
}


py::object IntersectionLineSegmentLineSegment(const geo2d::Line2d &segment1, const geo2d::Line2d &segment2)
{
  cv::Vec2d pt;
  const bool intersects = geo2d::IntersectionLineSegmentLineSegment(segment1, segment2, pt);
  if (intersects)
    return vpc::VecToTuple(pt);
  return py::none();
}


py::tuple IsPointLeftOfLine(const cv::Vec2d &pt, const geo2d::Line2d &line)
{
  bool on_line;
  bool left_of = geo2d::IsPointLeftOfLine(pt, line, &on_line);
  return py::make_tuple(left_of, on_line);
}


py::tuple IntersectionLineCircle(const geo2d::Line2d &line, const cv::Vec2d &center, double radius)
{
  cv::Vec2d intersection1, intersection2;
  int num_intersections = geo2d::IntersectionLineCircle(line, center, radius, intersection1, intersection2);
  if (num_intersections == 0)
    return py::make_tuple(num_intersections, py::none(), py::none());
  if (num_intersections == 1)
    return py::make_tuple(num_intersections, intersection1, py::none());
  if (num_intersections == 2)
    return py::make_tuple(num_intersections, intersection1, intersection2);
  VCP_ERROR("Invalid number (" + std::to_string(num_intersections) + ") of intersection points!");
}


py::tuple IntersectionLineSegmentCircle(const geo2d::Line2d &segment, const cv::Vec2d &center, double radius)
{
  cv::Vec2d intersection1, intersection2;
  int num_intersections = geo2d::IntersectionLineSegmentCircle(segment, center, radius, intersection1, intersection2);
  if (num_intersections == 0)
    return py::make_tuple(num_intersections, py::none(), py::none());
  if (num_intersections == 1)
    return py::make_tuple(num_intersections, intersection1, py::none());
  if (num_intersections == 2)
    return py::make_tuple(num_intersections, intersection1, intersection2);
  VCP_ERROR("Invalid number (" + std::to_string(num_intersections) + ") of intersection points!");
}



py::tuple PointsOfTangencyPointToCircle(const cv::Vec2d &pt, const cv::Vec2d &center, double radius)
{
  cv::Vec2d pot1, pot2;
  const bool success = geo2d::PointsOfTangencyPointToCircle(pt, center, radius, pot1, pot2);
  if (!success)
    return py::none();

  if (vcp::math::IsVecEqual(pot1, pot2))
    return py::make_tuple(pot1, py::none());

  return py::make_tuple(pot1, pot2);
}


py::tuple TransverseCommonTangentsBetweenCircles(const cv::Vec2d &center1, double radius1, const cv::Vec2d &center2, double radius2)
{
  geo2d::Line2d tangent1, tangent2;
  const int num_tangents = geo2d::TransverseCommonTangentsBetweenCircles(center1, radius1, center2, radius2, tangent1, tangent2);
  if (num_tangents == 0)
    return py::make_tuple(num_tangents, py::none(), py::none());
  if (num_tangents == 1)
    return py::make_tuple(num_tangents, tangent1, py::none());
  if (num_tangents == 2)
    return py::make_tuple(num_tangents, tangent1, tangent2);
  VCP_ERROR("Invalid number (" + std::to_string(num_tangents) + ") of transverse tangents!");
}


py::tuple DirectCommonTangentsBetweenCircles(const cv::Vec2d &center1, double radius1, const cv::Vec2d &center2, double radius2)
{
  geo2d::Line2d tangent1, tangent2;
  const int num_tangents = geo2d::DirectCommonTangentsBetweenCircles(center1, radius1, center2, radius2, tangent1, tangent2);
  if (num_tangents == 0)
    return py::make_tuple(num_tangents, py::none(), py::none());
  if (num_tangents == 1)
    return py::make_tuple(num_tangents, tangent1, py::none());
  if (num_tangents == 2)
    return py::make_tuple(num_tangents, tangent1, tangent2);
  VCP_ERROR("Invalid number (" + std::to_string(num_tangents) + ") of direct tangents!");
}


py::object Normalize(const cv::Vec2d &v)
{
  double len;
  const cv::Vec2d n = geo2d::Normalize(v, &len);
  if (len > 0.0)
    return py::make_tuple(n[0], n[1]);
  return py::none();
}

py::object UnitDirectionVector(const cv::Vec2d &from, const cv::Vec2d &to)
{
  return Normalize(geo2d::DirectionVector(from, to));
}

} // namespace geometry2d
} // namespace math
} // namespace python
} // namespace vcp


//-----------------------------------------------------------------------------
// Python module declarations

PYBIND11_MODULE(math2d, m)
{
  namespace vpmg = vcp::python::math::geometry2d;
  m.doc() = "2D-geometry-related math utils.";


  // #######################################################################################
  // Area stuff

  m.def("area_polygon", &geo2d::AreaPolygon,
        "Computes the polygon area - may fail for\n"
        "self-crossing, twisted and other 'weird' polygons.\n\n"
        ":param polygon: a list of 2D coordinates,\n"
        "                e.g. [(x0,y0), (x1,y1),...].\n"
        ":return: area as double",
        py::arg("polygon"));

  m.def("area_rotated_rect", &geo2d::AreaRotatedRectPy,
        "Computes the area of the rotated rect, which\n"
        "is given as (cx, cy, w, h, angle), where (cx,cy)\n"
        "denotes the center and angle is in degrees.\n"
        ":return: area as double",
        py::arg("rotated_rect"));

  m.def("area_triangle", &geo2d::AreaTriangle,
        "Computes the area of the triangle given by its\n"
        "three corners, i.e. ptX = (x,y)\n"
        ":return: area as double",
        py::arg("pt1"), py::arg("pt2"), py::arg("pt3"));


  // #######################################################################################
  // Path/trajectory stuff

  m.def("simplify_trajectory_rdp", &geo2d::SimplifyRamerDouglasPeucker,
        "Simplify a path via Ramer-Douglas-Peucker's algorithm.\n\n"
        ":param trajectory: list of (x,y) coordinates.\n"
        ":param epsilon:    RDP threshold.\n"
        ":return: path as list of (x,y) coordinates.",
        py::arg("points"), py::arg("epsilon"));


  m.def("simplify_trajectory_rdp_indices", &geo2d::SimplifyRamerDouglasPeuckerIndices,
        "Simplify a path via Ramer-Douglas-Peucker's algorithm\n"
        "and return the INDICES to construct the simplified path.\n\n"
        ":param trajectory: list of (x,y) coordinates.\n"
        ":param epsilon:    RDP threshold.\n"
        ":return: list of indices (int)",
        py::arg("points"), py::arg("epsilon"));


  m.def("smooth_trajectory", &geo2d::SmoothTrajectory,
        "Smooth a trajectory via moving average (sliding window).\n\n"
        ":param trajectory: list of (x,y) coordinates.\n"
        ":param window_size: odd (!) int\n"
        ":return: smoothed path as list of (x,y) coordinates",
        py::arg("trajectory"), py::arg("window_size"));


  m.def("remove_duplicate_points", &geo2d::RemoveDuplicatePoints,
        "Removes duplicate points along the trajectory (i.e.\n"
        "ensures that traj[i] != traj[i+1] afterwards).\n\n"
        ":param trajectory: list of (x,y) coordinates as integer!\n"
        ":return: list of (x,y) coordinates (integer!)",
        py::arg("trajectory"));


  m.def("bresenham_line", &geo2d::BresenhamLine,
        "Returns the Bresenham line between the two (int) points.\n\n"
        ":param from: Start point (x,y) with int coordinates\n"
        ":param to: End point (x,y) with int coordinates\n"
        ":return: list of (x,y) coordinates (int!).",
        py::arg("from"), py::arg("to"));


  m.def("bresenham_trajectory", &geo2d::BresenhamTrajectory,
        "Returns the Bresenham 'trajectory' which visits each\n"
        "pixel along the input trajectory.\n"
        "Duplicate coordinates will be removed automatically.\n\n"
        ":param trajectory: list of (x,y) integral coordinates\n"
        ":return: list of (x,y) coordinates (int!).",
        py::arg("trajectory"));


  // #######################################################################################
  // Polygon stuff

  m.def("convex_hull", &vpmg::ConvexHull,
        "Computes the convex hull of the given point set via\n"
        "Graham's scanline algorithm.\n\n"
        ":param points: list of (x,y) coordinates.\n"
        ":param check_for_duplicates: set to False only\n"
        "               if you're 100 % sure that there are\n"
        "               no duplicate points in the list!\n"
        ":param scale:  each point will be cast to int after\n"
        "               scaling, i.e. x_int = int(x * scale).\n"
        "               The result will be downscaled again,\n"
        "               i.e. x_out = x_int / scale.\n"
        ":return: convex hull as list of (x,y) coordinates.",
        py::arg("points"),
        py::arg("check_for_duplicates") = true,
        py::arg("scale") = 100.0);


  m.def("is_point_in_closed_polygon", &geo2d::IsPointInClosedPolygon,
        "Checks if the point is inside (or on the boundary of) the given\n"
        "(closed) polygon. If the polygon is not closed, it will automatically\n"
        "be closed ([poly[0],...,poly[end], poly[0]).\n\n"
        ":param point:   (x,y) coordinate as list, tuple or numpy.array.\n"
        ":param polygon: list of (x,y) coordinates.\n"
        ":return: True if point is inside (or on the boundary of) the polygon.",
        py::arg("point"), py::arg("polygon"));


  m.def("distance_point_closed_polygon", &geo2d::DistancePointClosedPolygon,
        "Computes the signed distance between the point and the (closed) polygon.\n"
        "If the polygon is not closed (i.e. polygon[end] != polygon[0]), it will be\n"
        "closed automatically (by adding a point).\n\n"
        ":param point:   (x,y) coordinate as list, tuple or numpy.array.\n"
        ":param polygon: list of (x,y) coordinates.\n"
        ":return: distance <= 0 if the point is inside (or on the polygon),\n"
        "         distance > 0 otherwise.",
        py::arg("point"), py::arg("polygon"));


  m.def("axis_aligned_bounding_rect", &geo2d::AxisAlignedBoundingRect,
        "Returns the axis-aligned bounding box of the given polygon.\n"
        ":param polygon: list of (x,y) coordinates.\n"
        ":return: tuple(l,r,w,h)",
        py::arg("polygon"));


  m.def("rotated_bounding_rect", &geo2d::RotatedBoundingRect,
        "Returns the minimum-area bounding rectangle for this polygon.\n\n"
        ":param polygon: list of (x,y) coordinates.\n"
        ":return: tuple(cx,cy,w,h,angle), angle in degrees.",
        py::arg("polygon"));

  m.def("is_polygon_clockwise", &geo2d::IsPolygonClockwise,
        "Checks if the polygon points are ordered clockwise.\n"
        "Watch out for right- vs. left-handed coordinate systems!\n"
        ":param polygon: list of (x,y) coordinates.\n"
        ":return: True if clockwise",
        py::arg("polygon"));


  m.def("intersection_convex_polygons", &geo2d::IntersectionConvexPolygons,
        "Returns the intersection of the two convex polygons\n"
        ":param a,b: polygons where each is a list of \n"
        "            (x,y) coordinates, e.g. [(x0,y0), ...].\n"
        ":return: list of (x,y) coordinates of the intersection\n"
        "            or empty, if they don't intersect.",
        py::arg("a"), py::arg("b"));


  // #######################################################################################
  // Line stuff

  m.def("is_point_left_of_line", &vpmg::IsPointLeftOfLine,
        "Checks if the point is left-of the line, assuming\n"
        "a right-handed(!) coordinate system.\n\n"
        ":param point: (x,y) coordinate as list, tuple or numpy.array.\n"
        ":param line:  list or tuple holding two points on the line,\n"
        "              i.e. [(x0,y0), (x1,y1)].\n"
        ":return: tuple(left_of, on_line) indicating whether the point\n"
        "              is left (or on) or exactly on the line.",
        py::arg("point"), py::arg("line"));


  m.def("clip_line_by_rect", &geo2d::ClipLineByRectanglePy,
        "Clip the line by the rectangle if possible.\n"
        ":param line: [(x0,y0), (x1,y1)]\n"
        ":param rect: (l,t,w,h))\n"
        ":return: None or clipped segment [(x0,y0),(x1,y1)]",
        py::arg("line"), py::arg("rect"));


  m.def("clip_line_segment_by_rect", &geo2d::ClipLineSegmentByRectanglePy,
        "Clip the line segment(!) by the rectangle if possible.\n"
        ":param line: [(x0,y0), (x1,y1)]\n"
        ":param rect: (l,t,w,h))\n"
        ":return: None or clipped segment [(x0,y0),(x1,y1)]",
        py::arg("segment"), py::arg("rect"));


  m.def("distance_point_line", &geo2d::DistancePointLine,
        ":param point: (x,y)\n"
        ":param line: given as a list of two points on it [(x0,y0), (x1,y1)]\n"
        ":return: distance to closest point as double",
        py::arg("point"), py::arg("line"));


  m.def("distance_point_line_segment", &geo2d::DistancePointLineSegment,
        ":param point: (x,y)\n"
        ":param segment: given as a list of its end-points [(x0,y0), (x1,y1)]\n"
        ":return: distance to closest point as double",
        py::arg("point"), py::arg("segment"));


  m.def("intersection_line_line", &vpmg::IntersectionLineLine,
        "Computes the intersection of the two lines.\n\n"
        ":params line1, line2: a line is a 2-element\n"
        "          tuple or list holding two points on\n"
        "          on the line, e.g. ((x0,y0), (x1,y1)).\n"
        ":return: None or intersection point as (x,y).",
        py::arg("line1"), py::arg("line2"));


  m.def("intersection_line_line_segment", &vpmg::IntersectionLineLineSegment,
        "Computes the intersection of line and the line segment.\n\n"
        ":param line: is a 2-element tuple holding two points on\n"
        "       the line, e.g. ((x0,y0), (x1,y1)).\n"
        ":param segment: is a 2-element tuple holding the end\n"
        "       points of the segment, e.x. ((x0,y0), (x1,y1)).\n"
        ":return: None or intersection point as (x,y).",
        py::arg("line"), py::arg("segment"));


  m.def("intersection_line_segment_line_segment", &vpmg::IntersectionLineSegmentLineSegment,
        "Computes the intersection point of the two line segments.\n\n"
        ":params segment1, segment2: are 2-element tuples holding the\n"
        "       end points of each segment, e.x. ((x0,y0), (x1,y1)).\n"
        ":return: None or intersection point as (x,y).",
        py::arg("segment1"), py::arg("segment2"));


  m.def("are_lines_collinear", &geo2d::IsCollinear,
        "Checks if two lines are collinear.\n"
        ":params line1, line2: a line is a 2-element\n"
        "          tuple or list holding two points on\n"
        "          on the line, e.g. ((x0,y0), (x1,y1)).\n"
        ":return: True/False",
        py::arg("line1"), py::arg("line2"));


  m.def("intersection_line_circle", &vpmg::IntersectionLineCircle,
        "Computes the line-circle intersection.\n"
        ":param line: a 2-element tuple or list holding two points\n"
        "         on the line, e.g. ((x0,y0), (x1,y1)).\n"
        ":param center: (cx,cy) of the circle.\n"
        ":param radius: radius of the circle.\n"
        ":return: 3-element tuple (num_intersections, i0, i1),\n"
        "         where num_intersections in [0,1,2] and i0, i1\n"
        "         are the (x,y) intersection points or None",
        py::arg("line"), py::arg("center"), py::arg("radius"));


  m.def("intersection_line_segment_circle", &vpmg::IntersectionLineSegmentCircle,
        "Computes the intersection of the line segment(!) with the circle.\n"
        ":param segment: a 2-element tuple or list holding the end points\n"
        "         of the segment, e.g. ((x0,y0), (x1,y1)).\n"
        ":param center: (cx,cy) of the circle.\n"
        ":param radius: radius of the circle.\n"
        ":return: 3-element tuple (num_intersections, i0, i1),\n"
        "         where num_intersections in [0,1,2] and i0, i1\n"
        "         are the (x,y) intersection points or None",
        py::arg("segment"), py::arg("center"), py::arg("radius"));


  m.def("project_point_onto_line", &geo2d::ProjectPointOntoLine,
        "Project point onto line.\n"
        ":param point: (x,y)\n"
        ":param line: list of two points on the\n"
        "             line, i.e. [(x0,y0), (x1,y1)].\n"
        ":return: closest point on line as tuple (x,y).",
        py::arg("point"), py::arg("line"));


  m.def("closest_point_on_line", &geo2d::ProjectPointOntoLine,
        "Returns the point on the line closest to 'point'.\n"
        "This is a convenience wrapper to project_point_onto_line().\n"
        ":param point: (x,y)\n"
        ":param line: list of two points on the\n"
        "             line, i.e. [(x0,y0), (x1,y1)].\n"
        ":return: closest point on line as tuple (x,y).",
        py::arg("point"), py::arg("line"));


  m.def("closest_point_on_line_segment", &geo2d::ClosestPointOnLineSegment,
        "Returns the point on the line segment(!) closest to 'point'.\n"
        ":param point: (x,y)\n"
        ":param segment: list of the two end points of the\n"
        "             segment, i.e. [(x0,y0), (x1,y1)].\n"
        ":return: closest point on segment as tuple (x,y), i.e.\n"
        "             either start/end or a point in between",
        py::arg("point"), py::arg("line"));


  // #######################################################################################
  // Circle stuff

  m.def("circle_from_three_points", &vpmg::CircleFrom3Points,
        "Reconstructs the circle from 3 points.\n"
        ":params pt1, pt2, pt3: 3 points on the circle given\n"
        "        by their (x,y) coordinates (as list, tuple,\n"
        "        or np.array).\n"
        ":return: tuple(exists, center, radius) where exists\n"
        "          is a boolean flag, center a (x,y) tuple and\n"
        "          radius a double.",
        py::arg("pt1"), py::arg("pt2"), py::arg("pt3"));


  m.def("is_point_in_circle", &vpmg::IsPointInCircle,
        "Checks if given point lies within (or exactly on) the circle.\n"
        ":param pt: (x,y) point to test.\n"
        ":param center: (cx,cy) of the circle.\n"
        ":param radius: radius of the circle."
        ":return: tuple of two boolean flags: (in_or_on, exactly_on)",
        py::arg("pt"), py::arg("center"), py::arg("radius"));


  m.def("transverse_common_tangents_circles", &vpmg::TransverseCommonTangentsBetweenCircles,
          "Compute the transverse common tangents between two circles.\n"
          ":params center1, center2: the centers of the circles, given\n"
          "         as (x,y), i.e. a tuple, list, or np.array.\n"
          ":params radius1, radius2: the radii of the circles, given\n"
          "         as numbers.\n"
          ":return: 3-element tuple (num_tangents, t0, t1),\n"
          "         where num_tangents in [0,1,2] and t0, t1\n"
          "         are the tangent lines given as [(x0,y0), (x1,y1)].",
          py::arg("center1"), py::arg("radius1"), py::arg("center2"), py::arg("radius2"));


  m.def("direct_common_tangents_circles", &vpmg::DirectCommonTangentsBetweenCircles,
        "Compute the direct common tangents between two circles.\n"
        ":params center1, center2: the centers of the circles, given\n"
        "         as (x,y), i.e. a tuple, list, or np.array.\n"
        ":params radius1, radius2: the radii of the circles, given\n"
        "         as numbers.\n"
        ":return: 3-element tuple (num_tangents, t0, t1),\n"
        "         where num_tangents in [0,1,2] and t0, t1\n"
        "         are the tangent lines given as [(x0,y0), (x1,y1)].",
        py::arg("center1"), py::arg("radius1"), py::arg("center2"), py::arg("radius2"));


  m.def("points_of_tangency_point_circle", &vpmg::PointsOfTangencyPointToCircle,
        "Computes the point(s) of tangency between a point and the circle.\n"
        ":param center: the center of the circle, given as (x,y).\n"
        ":param radius: the radius of the circle, scalar number.\n"
        ":return: None if point is inside the circle.\n"
        "         (pt, None) if point is exactly on the circle.\n"
        "         ((x1,y1), (x2,y2)) the points of tangency otherwise.",
        py::arg("pt"), py::arg("center"), py::arg("radius"));

  // #######################################################################################
  // Vector stuff

  m.def("cross2d", &geo2d::Cross,
        "2D cross product (i.e. wedge product) of the two\n"
        "vectors v = (x,y).\n"
        ":return: double",
        py::arg("v1"), py::arg("v2"));

  m.def("dot2d", &geo2d::Dot,
        "2D dot product of the two vectors v = (x,y).\n"
        ":return: double",
        py::arg("v1"), py::arg("v2"));

  m.def("direction2d", &geo2d::DirectionVector,
        "Returns the 2d direction vector (x,y)",
        py::arg("from"), py::arg("to"));

  m.def("unit_direction2d", &vpmg::UnitDirectionVector,
        "Returns the normalized 2d direction vector (x,y)\n"
        "or None if direction vector 'from' to 'to' is (0,0)\n",
        py::arg("from"), py::arg("to"));

  m.def("distance2d", &geo2d::Distance,
        "Returns the distance (double) between the two\n"
        "(x,y) points.",
        py::arg("v"), py::arg("w"));

  m.def("length2d", &geo2d::Length,
        "Returns the vector's length, vector is given as (x,y).",
        py::arg("v"));

  m.def("normalize2d", &vpmg::Normalize,
        "Normalize the vector (x,y) to unit length.",
        py::arg("v"));

  m.def("rotate_vectors", &geo2d::RotateVecs,
        "Rotates the 2d vectors.\n\n"
        ":param vectors: list of 2d points, [(x0,y0), (x1,y1), ...].\n"
        ":param rotation_center: 2d rotation center, (x,y).\n"
        ":param theta:           rotation angle in radians.\n"
        ":return: list of rotated points [(x0,y0),...]",
        py::arg("vectors"),
        py::arg("rotation_center"),
        py::arg("theta"));

  m.def("rotate_vector", &geo2d::RotateVectorPy,
        "Rotates the 2d vector.\n\n"
        ":param vector: 2d vector, (x,y).\n"
        ":param rotation_center: 2d rotation center, (rx, ry).\n"
        ":param theta:           rotation angle in radians.\n"
        ":return: rotated point (x,y)",
        py::arg("vector"),
        py::arg("rotation_center"),
        py::arg("theta"));

  m.def("is_point_inside_rectangle", &geo2d::IsPointInsideRectangle,
        "Checks if the point is inside the given rectangle.\n"
        ":param pt: (x,y)\n"
        ":param rect: (l,t,w,h)\n"
        ":return: bool",
        py::arg("pt"), py::arg("rect"));


  m.def("vector_projection", &geo2d::VectorProjection,
        "Returns the vector projection of a onto b.\n"
        "Vector projection is:\n"
        "  unit_vector(b) * scalar_projection(a onto b).\n"
        ":params a,b: 2d vectors, i.e. (x,y)\n"
        ":return: projection (x,y)",
        py::arg("a"), py::arg("b"));

  m.def("scalar_projection", &geo2d::ScalarProjection,
        "Returns the scalar projection of vector a onto b,\n"
        "i.e. the component of a in the b direction.\n"
        ":params a,b: 2d vectors, i.e. (x,y)\n"
        ":return: double",
        py::arg("a"), py::arg("b"));


  // #######################################################################################
  // IOU stuff
  m.def("iou_rect", &geo2d::IntersectionOverUnionRects,
        "Intersection over union of two axis-aligned rectangles.\n"
        ":params a,b: rects given as (l,t,w,h).\n"
        ":return: iou, a double in [0,1]",
        py::arg("a"), py::arg("b"));

  m.def("iou_rotated_rect", &geo2d::IntersectionOverUnionRotatedRects,
        "Intersection over union of two rotated rectangles.\n"
        ":params a,b: rotated rects given as (cx, cy, w, h, angle),\n"
        "             with (cx,cy) denoting the center and angle in\n"
        "             degrees.\n"
        ":return: iou, a double in [0,1].",
        py::arg("a"), py::arg("b"));
}
