#include "common.h"
#include "conversions.h"
#include "geometry3d.h"
#include "geometry2d.h"

#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <string>

namespace vcp {
namespace math {
namespace test {
const double kEps = 1e-10;
//TODO add remaining functions


TEST(Math, VecEquality)
{
  const cv::Vec2d a(2.123456789e13, 1.9);
  EXPECT_TRUE(vcp::math::IsVecEqual(a,a,5));
  EXPECT_TRUE(vcp::math::IsVecEqual(a,a));

  EXPECT_FALSE(vcp::math::IsVecEqual(a, cv::Vec2d(2.123456788e13, 1.9)));
  EXPECT_TRUE(vcp::math::IsVecEqual(a, cv::Vec2d(2.1234567889999999e13, 1.9)));

  EXPECT_FALSE(vcp::math::IsVecEqual(a, cv::Vec2d(2.123456788e13, 1.9), 2));
  EXPECT_FALSE(vcp::math::IsVecEqual(a, cv::Vec2d(2.12345678899999e13, 1.9), 2));
  EXPECT_TRUE( vcp::math::IsVecEqual(a, cv::Vec2d(2.123456788999999e13, 1.9), 2));
}



TEST(Math, Line3d)
{
  const geo3d::Line3d line(cv::Vec3d(1,1,1), cv::Vec3d(1,1,3));
  EXPECT_NEAR(geo3d::DistancePointLine(cv::Vec3d(2.0,1.0,1), line), 1.0, kEps);
  EXPECT_NEAR(geo3d::DistancePointLine(cv::Vec3d(2.0,-1.0,0.5), line), std::sqrt(5), kEps);
  EXPECT_NEAR(geo3d::DistancePointLine(cv::Vec3d(0,0,0), line), std::sqrt(2.0), kEps);
  EXPECT_NEAR(geo3d::DistancePointLine(cv::Vec3d(1,1,2.9), line), 0.0, kEps);
  EXPECT_NEAR(geo3d::DistancePointLine(cv::Vec3d(1,1,3.25), line), 0.0, kEps);
  EXPECT_NEAR(geo3d::DistancePointLine(cv::Vec3d(1,-1,2), line), 2.0, kEps);


  EXPECT_NEAR(geo3d::DistancePointLineSegment(cv::Vec3d(2.0,1.0,1), line), 1.0, kEps);
  EXPECT_NEAR(geo3d::DistancePointLineSegment(cv::Vec3d(2.0,-1.0,0.5), line), std::sqrt(5.25), kEps);
  EXPECT_NEAR(geo3d::DistancePointLineSegment(cv::Vec3d(0,0,0), line), std::sqrt(3.0), kEps);
  EXPECT_NEAR(geo3d::DistancePointLineSegment(cv::Vec3d(1,1,2.9), line), 0.0, kEps);
  EXPECT_NEAR(geo3d::DistancePointLineSegment(cv::Vec3d(1,1,3.25), line), 0.25, kEps);
  EXPECT_NEAR(geo3d::DistancePointLineSegment(cv::Vec3d(1,-1,2), line), 2.0, kEps);
}

TEST(Math, Line2d)
{
  const geo2d::Line2d line(cv::Vec2d(10,3), cv::Vec2d(10,5));
  cv::Vec2d pt(9.2, 5.7);
  EXPECT_NEAR(geo2d::DistancePointLine(pt, line), 0.8, kEps);
  EXPECT_NEAR(geo2d::DistancePointLineSegment(pt, line), 1.0630145812734657, kEps);
  pt = cv::Vec2d(10.0, 2.8);
  EXPECT_NEAR(geo2d::DistancePointLine(pt, line), 0.0, kEps);
  EXPECT_NEAR(geo2d::DistancePointLineSegment(pt, line), 0.2, kEps);
}

TEST(Math, Plane)
{
  const cv::Vec4d gp(0,0,-1,0.1);
  auto plane = geo3d::PlaneFrom3Points(cv::Vec3d(10,2342.234,0.1), cv::Vec3d(243,1234,0.1), cv::Vec3d(12,-3455,0.1));
  EXPECT_TRUE(IsVecEqual(gp, plane));
  EXPECT_TRUE(IsVecEqual(geo3d::PlaneXYZIntercepts(plane), cv::Vec3d(0, 0, 0.1)));

  EXPECT_NEAR(geo3d::DistancePointPlane(cv::Vec3d::all(0), plane), 0.1, kEps);
  EXPECT_NEAR(geo3d::DistancePointPlane(cv::Vec3d::all(0.1), plane), 0.0, kEps);

  EXPECT_NEAR(geo3d::AngleLinePlane(geo3d::Line3d(cv::Vec3d::all(0), cv::Vec3d(0, 0, 1)), plane), -Deg2Rad(90), kEps);
  EXPECT_NEAR(geo3d::AngleLinePlane(geo3d::Line3d(cv::Vec3d::all(0), cv::Vec3d(0, 0, -1)), plane), Deg2Rad(90), kEps);
  EXPECT_NEAR(geo3d::AngleLinePlane(geo3d::Line3d(cv::Vec3d::all(0), cv::Vec3d(1, 1, 0)), plane), 0.0, kEps);
  EXPECT_NEAR(geo3d::AngleLinePlane(geo3d::Line3d(cv::Vec3d::all(0), cv::Vec3d(0, 1, 1)), plane), Deg2Rad(-45), kEps);

  EXPECT_NEAR(Rad2Deg(geo3d::DihedralAngle(plane, cv::Vec4d(1,0,0,0))), 90, kEps);
  EXPECT_NEAR(Rad2Deg(geo3d::DihedralAngle(plane, cv::Vec4d(1.0/std::sqrt(2),1.0/std::sqrt(2),0,0))), 90, kEps);
  EXPECT_NEAR(Rad2Deg(geo3d::DihedralAngle(plane, cv::Vec4d(0, 1.0/std::sqrt(2),1.0/std::sqrt(2),0))), 135, kEps);
  EXPECT_NEAR(Rad2Deg(geo3d::DihedralAngle(plane, cv::Vec4d(0, -1.0/std::sqrt(2),-1.0/std::sqrt(2),0))), 45, kEps);

  plane = geo3d::PlaneFrom3Points(cv::Vec3d(-1, -2, 2), cv::Vec3d(-1, 2, 2), cv::Vec3d(1, 0, 1));
  const cv::Vec3d p1 = cv::Vec3d(0,0,1.5) + 3.14*geo3d::Normalize(cv::Vec3d(1,0,2)); // Exactly 3.14 away from the plane's z-intercept (behind the plane)
  const cv::Vec3d p2 = cv::Vec3d(0,0,1.5) - 4.2*geo3d::Normalize(cv::Vec3d(1,0,2)); // Exactly 4.2 away from the plane's z-intercept (in front of the plane)
  const cv::Vec3d p3(3, 0, 0); // on plane (x intercept)
  EXPECT_TRUE(IsVecEqual(geo3d::Normalize(cv::Vec3d(1,0,2)), cv::Vec3d(1.0/std::sqrt(5), 0, 2.0/std::sqrt(5))));
  EXPECT_NEAR(geo3d::DistancePointPlane(p1, plane), -3.14, kEps);
  EXPECT_NEAR(geo3d::DistancePointPlane(p2, plane), 4.2, kEps);
  EXPECT_NEAR(geo3d::DistancePointPlane(p3, plane), 0.0, kEps);
}

TEST(Math, Conversion)
{
  EXPECT_TRUE(IsPointEqual(vcp::convert::ToPoint(cv::Vec2d(0.99, -0.3)), cv::Point(0,0)));
  EXPECT_FALSE(IsPointEqual(vcp::convert::ToPoint(cv::Vec2d(0.9, -1)), cv::Point(0,0)));
  EXPECT_TRUE(IsPointEqual(vcp::convert::ToPoint(cv::Vec2d(0.9, -1)), cv::Point(0,-1)));
}

TEST(Math, Polygon)
{
  const std::vector<cv::Point> polygon = {cv::Point(1,1), cv::Point(2,4), cv::Point(4,4), cv::Point(2,2), cv::Point(4,1)};
  std::vector<cv::Vec2d> poly2d;
  for (const auto &p : polygon)
    poly2d.push_back(vcp::convert::ToVec2d(p));

  EXPECT_FALSE(geo2d::IsPointInClosedPolygon(cv::Vec2d(0,0), poly2d));
  EXPECT_TRUE(geo2d::IsPointInClosedPolygon(cv::Vec2d(1,1), poly2d));
  EXPECT_TRUE(geo2d::IsPointInClosedPolygon(cv::Vec2d(2,1.5), poly2d));
  EXPECT_TRUE(geo2d::IsPointInClosedPolygon(cv::Vec2d(2,1), poly2d));
  EXPECT_FALSE(geo2d::IsPointInClosedPolygon(cv::Vec2d(3,2), poly2d));
  EXPECT_FALSE(geo2d::IsPointInClosedPolygon(cv::Vec2d(4.5,4), poly2d));

  EXPECT_NEAR(geo2d::DistancePointClosedPolygon(cv::Vec2d(0,0), poly2d), std::sqrt(2.0), kEps);
  EXPECT_NEAR(geo2d::DistancePointClosedPolygon(cv::Vec2d(1,1), poly2d), 0.0, kEps);
  EXPECT_NEAR(geo2d::DistancePointClosedPolygon(cv::Vec2d(2,1.5), poly2d), -std::sqrt(0.2), kEps);
  EXPECT_NEAR(geo2d::DistancePointClosedPolygon(cv::Vec2d(2,1), poly2d), 0.0, kEps);
  EXPECT_NEAR(geo2d::DistancePointClosedPolygon(cv::Vec2d(3,2), poly2d), std::sqrt(0.2), kEps);
  EXPECT_NEAR(geo2d::DistancePointClosedPolygon(cv::Vec2d(4.5,4), poly2d), 0.5, kEps);
}
// TODO line simplification, intersections, etc take from pypvt demo

} // namespace test
} // namespace math
} // namespace vcp

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
