#include "math_utils.h"

namespace vcp
{
namespace utils
{
namespace math
{
std::vector<double> Poly2Rect(const std::vector<double> &polygon)
{
  std::vector<double> rect;
  rect.reserve(4);

  const double cx = VecMean(polygon, 0, polygon.size(), 2);
  const double cy = VecMean(polygon, 1, polygon.size(), 2);
  const double x1 = VecMin(polygon, 0, polygon.size(), 2);
  const double x2 = VecMax(polygon, 0, polygon.size(), 2);
  const double y1 = VecMin(polygon, 1, polygon.size(), 2);
  const double y2 = VecMax(polygon, 1, polygon.size(), 2);
  const double d1 = polygon[0] - polygon[2];
  const double d2 = polygon[1] - polygon[3];
  const double d3 = polygon[2] - polygon[4];
  const double d4 = polygon[3] - polygon[5];
  const double A1 = std::sqrt(d1*d1 + d2*d2) * std::sqrt(d3*d3 + d4*d4);
  const double A2 = (x2 - x1) * (y2 - y1);
  const double scl = std::sqrt(A1/A2);
  const double width = scl * (x2 - x1) + 1.0;
  const double height = scl * (y2 - y1) + 1.0;
  const double left = cx - width/2.0;
  const double top = cy - height/2.0;

  rect.push_back(left);
  rect.push_back(top);
  rect.push_back(width);
  rect.push_back(height);
  return rect;
}

std::vector<double> Rect2Poly(const std::vector<double> &rect)
{
  std::vector<double> poly;
  poly.reserve(8);

  const double l = rect[0];
  const double t = rect[1];
  const double r = l + rect[2] - 1.0;
  const double b = t + rect[3] - 1.0;

  // Top left.
  poly.push_back(l);
  poly.push_back(t);
  // Top right.
  poly.push_back(r);
  poly.push_back(t);
  // Bottom right.
  poly.push_back(r);
  poly.push_back(b);
  // Bottom left.
  poly.push_back(l);
  poly.push_back(b);

  return poly;
}

double RoundBase(double val, double base)
{
  return base * std::round(val/base);
}

} // namespace math
} // namespace utils
} // namespace vcp

