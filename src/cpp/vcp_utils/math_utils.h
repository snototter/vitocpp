#ifndef __VCP_UTILS_MATH_UTILS_H__
#define __VCP_UTILS_MATH_UTILS_H__

#include <stddef.h>
#include <vector>
#include <numeric>
#include <cmath>

namespace vcp
{
namespace utils
{
/** @brief Basic math utilities to analyse data collections (std::vector) of built-in types. */
namespace math
{
// TODO Computing statistics from a vector of built-in types: see https://stackoverflow.com/a/7616783/400948

/** @brief Mean over all values of a vector. */
template <typename T>
double VecMean(const std::vector<T> &values)
{
  double sum = std::accumulate(values.begin(), values.end(), 0.0);
  return sum / values.size();
}


/** @brief Mean over a subset (from index_from to (excluding) index_to, with index_step step size) of a vector's values. */
template <typename T>
double VecMean(const std::vector<T> &values, size_t index_from, size_t index_to, size_t index_step = 1)
{
  double sum = 0.0;
  for (size_t i = index_from; i < index_to; i += index_step)
    sum += static_cast<double>(values[i]);

  return sum / std::ceil((static_cast<double>(index_to - index_from)) / index_step);
}


/** @brief Min value of a potential vector subset. Similar to MATLAB's min(vector(2:5:end)). */
template <typename T>
T VecMin(const std::vector<T> &values, size_t index_from, size_t index_to, size_t index_step = 1)
{
  T min = values[index_from];
  for (size_t i = index_from + index_step; i < index_to; i += index_step)
  {
    if (values[i] < min)
      min = values[i];
  }
  return min;
}


/** @brief Max value of a potential vector subset. Similar to MATLAB's max(vector(2:5:end)). */
template <typename T>
T VecMax(const std::vector<T> &values, size_t index_from, size_t index_to, size_t index_step = 1)
{
  T max = values[index_from];
  for (size_t i = index_from + index_step; i < index_to; i += index_step)
  {
    if (values[i] > max)
      max = values[i];
  }
  return max;
}


/** @brief Round to the nearest base, e.g. rb(13,5) = 15 or rb(0.967,0.05) = 0.95.*/
double RoundBase(double val, double base);


/** @brief Return an axis-aligned bounding box (4-element vector [l,t,w,h]) with equivalent area for the given polygon (which is {x1,y1,x2,y2,...}).
 * Useful for our single object trackers (find a tight axis-aligned bounding box for the VOT framework annotations).
 */
std::vector<double> Poly2Rect(const std::vector<double> &polygon);


/** @brief Return a polygon representation (8-element vector [l,t,r,t,r,b,l,b]) of an axis aligned bounding box.
 * Useful for tracking within the VOT framework (which expects tracking results formated in such a polygon representation...)
*/
std::vector<double> Rect2Poly(const std::vector<double> &rect);

//TODO implement the following:
// VecElemDivide
// VecElemMul
// VecElemPlus
// VecElemMinus

} // namespace math
} // namespace utils
} // namespace vcp

#endif // __VCP_UTILS_MATH_UTILS_H__
