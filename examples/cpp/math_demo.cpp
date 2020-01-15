#include <iostream>
#include <string>
#include <vector>
#include <cstdio>
#include <fstream>

#include <vcp_math/geometry3d.h>
#include <vcp_math/geometry2d.h>
#include <vcp_math/common.h>
#include <vcp_imutils/matutils.h>

#define VCP_LOG_LEVEL_DEBUG
#include <vcp_utils/vcp_logging.h>

namespace geo3 = vcp::math::geo3d;
namespace geo2 = vcp::math::geo2d;
namespace vm = vcp::math;
namespace vi = vcp::imutils;

int main(int argc, char **argv)
{
  VCP_UNUSED_VAR(argc);
  VCP_UNUSED_VAR(argv);

  // Plane from 3 points
  auto plane = geo3::PlaneFrom3Points(cv::Vec3d(10,2342.234,0), cv::Vec3d(243,1234,0), cv::Vec3d(12,-3455,0));
  VCP_LOG_INFO("Plane from 3 points:\n  " << plane << "\n  XYZ intercepts: " << geo3::PlaneXYZIntercepts(plane));

  // Colinear points
  plane = geo3::PlaneFrom3Points(cv::Vec3d(10, -2, 2), cv::Vec3d(10, 0.1, 2), cv::Vec3d(10, 0, 2));
  VCP_LOG_INFO("Invalid plane from 3 collinear points:\n  " << plane << "\n  XYZ intercepts: " << geo3::PlaneXYZIntercepts(plane) << std::endl);

  // Plane from 3 points, distance to plane:
  plane = geo3::PlaneFrom3Points(cv::Vec3d(-1, -2, 2), cv::Vec3d(-1, 2, 2), cv::Vec3d(1, 0, 1));
  VCP_LOG_INFO("Plane from 3 points:\n  " << plane << "\n  XYZ intercepts: " << geo3::PlaneXYZIntercepts(plane));
  const cv::Vec3d p1(0,-15,2);
  const cv::Vec3d p2 = cv::Vec3d(0,0,1.5) + 3.14*geo3::Normalize(cv::Vec3d(1,0,2)); // Exactly 3.14 away from the plane's z-intercept.
  const cv::Vec3d p3(3, 0, 0); // on plane (x intercept)
  VCP_LOG_INFO("Point to plane distance:\n  "
                 << p1 << ": " << geo3::DistancePointPlane(p1, plane) << std::endl
                 << " " << p2 << ": " << geo3::DistancePointPlane(p2, plane) << "==3.14\n  "
                 << p3 << ": " << geo3::DistancePointPlane(p3, plane) << "==0, " << geo3::DistancePointPlane(-plane[3]*geo3::PlaneNormal(plane), plane) << "==0" << std::endl);

  // Line segment intersection:
  cv::Vec3d at1, at2, at3;
  bool ints1, ints2, ints3;
  const auto line1 = geo3::Line3d(cv::Vec3d(0,0,0), cv::Vec3d(0,0,1));
  const auto line2 = geo3::Line3d(cv::Vec3d(0,1,0), cv::Vec3d(3,-1,1));
  const auto line3 = geo3::Line3d(cv::Vec3d(0,1.9,0), cv::Vec3d(0,-1,1.5));
  ints1 = geo3::IntersectionLineSegmentPlane(line1, plane);
  ints2 = geo3::IntersectionLineSegmentPlane(line2, plane, &at2);
  ints3 = geo3::IntersectionLineSegmentPlane(line3, plane, &at3);
  VCP_LOG_INFO("Intersection Plane - Line Segment:" << std::endl
                 << "  Plane:    " << plane << std::endl
                 << "  Line seg: " << line1 << std::endl
                 << "    Intersects " << ints1 << "==false?");
  VCP_LOG_INFO("Intersection Plane - Line Segment:" << std::endl
                 << "  Plane:    " << plane << std::endl
                 << "  Line seg: " << line2 << std::endl
                 << "    Intersects " << ints2 << "==true? at " << at2);
  VCP_LOG_INFO("Intersection Plane - Line Segment:" << std::endl
                 << "  Plane:    " << plane << std::endl
                 << "  Line seg: " << line3 << std::endl
                 << "    Intersects " << ints3 << "==true? at " << at3 << std::endl);

  // Line intersection:
  ints1 = geo3::IntersectionLinePlane(line1, plane, &at1);
  VCP_LOG_INFO("Intersection Plane - Line:" << std::endl
                 << "  Plane: " << plane << std::endl
                 << "  Line:  " << line1 << std::endl
                 << "    Intersects " << ints1 << "==true at " << at1 << std::endl);

  // Point to line/line segment distance
  const geo3::Line3d segment(cv::Vec3d(1,1,1), cv::Vec3d(1,1,3));
  std::vector<cv::Vec3d> points;
  // to line: norm([2,-1,0.5] - [1,1,0.5]) = 2.2361
  // to segment norm([2,-1,0.5] - [1,1,1]) = 2.2913
  points.push_back(cv::Vec3d(2,-1,0.5));
  // to line: sqrt(2); to segment: sqrt(3)
  points.push_back(cv::Vec3d(0,0,0));
  points.push_back(cv::Vec3d(1,1,1));
  points.push_back(cv::Vec3d(1,1,2.9));
  points.push_back(cv::Vec3d(1,1,3.25));
  points.push_back(cv::Vec3d(1,-1,2));
  VCP_LOG_INFO("Point to line distance:\n  " << segment);
  for (const auto pt : points)
    VCP_LOG_INFO("  " << pt << ": " << geo3::DistancePointLine(pt, segment));
  VCP_LOG_INFO("\nPoint to line segment distance:\n  " << segment);
  for (const auto pt : points)
    VCP_LOG_INFO("  " << pt << ": " << geo3::DistancePointLineSegment(pt, segment));

  VCP_LOG_INFO("EPS Equal:\n  1... vs 1, 2: " << vm::eps_equal(1.00000001, 1.0, 2)
               << "\n1... vs 1, 5:  " << vm::eps_equal(1.00000000000001, 1.0, 5)
               << "\n1.... vs 1, 5: |" << vm::eps_equal(1.000000000000001, 1.0, 5)
               << "\n1... vs 1e5, 1: |" << vm::eps_equal(1.000000000000001, 100000.0, 1));


  VCP_LOG_INFO("OpenCV Mat types: " << vi::CVMatDepthToString(CV_8U,1) << ", "
                 << vi::CVMatDepthToString(CV_8S,2) << ", "
                 << vi::CVMatDepthToString(CV_64F,3) << ", "
                 << vi::CVMatDepthToString(CV_32S));

  // 2D lines
  geo2::Line2d line2d(cv::Vec2d(10,3), cv::Vec2d(10,5));
  cv::Vec2d pt2d(9.2, 5.7);
  VCP_LOG_INFO("Distance 2d:\n  Line: " << line2d << ", pt: " << pt2d << std::endl << "  Distance: " << geo2::DistancePointLine(pt2d, line2d) << " to segment: " << geo2::DistancePointLineSegment(pt2d, line2d));
  pt2d = cv::Vec2d(10.0, 2.8);
  VCP_LOG_INFO("Distance 2d:\n  Line: " << line2d << ", pt: " << pt2d << std::endl << "  Distance: " << geo2::DistancePointLine(pt2d, line2d) << " to segment: " << geo2::DistancePointLineSegment(pt2d, line2d));

  // Mod
  VCP_LOG_INFO("\nModulo (mathematic instead of symmetric):" << std::endl << "-2 mod 5: " << vm::Mod(-2, 5) << "\n-5 mod 4: " << vm::Mod(-5,4)
                  << "\n-3 mod 4: " << vm::Mod(-3,4) << "\n2 mod 3: " << vm::Mod(2,3) << "\n19 mod 2: " << vm::Mod(19,2));

  // GCD
  VCP_LOG_INFO("\nGCD(8,12): " << vm::GreatestCommonDivisor(8,12) << "\nGCD(12345678,982764534): "
               << vm::GreatestCommonDivisor(12345678,982764534));
  // LCM
  VCP_LOG_INFO("\nLCM(8,12): " << vm::LeastCommonMultiple(8,12) << ", (12,8): " << vm::LeastCommonMultiple(12,8)
                 << "\nLCM(21,6): " << vm::LeastCommonMultiple(21,6));
  return 0;
}
