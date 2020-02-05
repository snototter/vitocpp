#ifndef __VCP_MATH_COMMON_H__
#define __VCP_MATH_COMMON_H__

#include <cmath>
#include <limits>
#include <type_traits>
#include <vcp_utils/vcp_error.h>

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/core/core.hpp>
#else
    #include <opencv2/core.hpp>
#endif

#define MATH_PI   3.1415926535897932384626433832795
#define MATH_2PI  6.283185307179586476925286766559
#define MATH_LOG2 0.69314718055994530941723212145818

namespace vcp
{
/** @brief Math utilities, e.g. geometry (2D and 3D), trigonometry, ... */
namespace math
{
/** @brief Uses the machine epsilon to properly check for equality based on the desired precision in ULPs (units
  * in the last place.
  * Watch out: 0.0 is NOT equal to 1.1e-16!
  */
template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
    eps_equal(T x, T y, int ulp=2)
{
  // Adapted (using fabs) from STL Reference https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::fabs(x-y) <= std::numeric_limits<T>::epsilon() * std::fabs(x+y) * ulp
  // unless the result is subnormal
         || std::fabs(x-y) < std::numeric_limits<T>::min();
}
//FIXME check against boost eps_equal
// https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
/** @brief Check if |x| < 1e-10. */
template<typename T>
bool eps_zero(T x)
{//FIXME std::numeric_limits<T>::epsilon()
  return std::fabs(x) < 1e-10;
}

//FIXME check where it is used and whether check is correct!
/** @brief Checks if two cv::Vec are equal w.r.t. to the machine epsilon (and the desired precision in ULPs), @see eps_equal (Notes on what is equal and what is not!) */
template<typename T, int D>
bool IsVecEqual(const cv::Vec<T,D> &a, const cv::Vec<T,D> &b, int ulp)
{
  for (int i = 0; i < D; ++i)
  {
    if (!eps_equal<T>(a.val[i], b.val[i], ulp))
      return false;
  }
  return true;
}


/** @brief Checks if two vectors are equal, i.e. |a[i]-b[i]| < 1e-10 must hold for each dimension. */
template<typename T, int D>
bool IsVecEqual(const cv::Vec<T,D> &a, const cv::Vec<T,D> &b)
{
  const cv::Vec<T,D> diff = a - b;
  for (int i = 0; i < D; ++i)
  {
    if (!eps_zero<T>(diff.val[i]))
      return false;
  }
  return true;
}


/** @brief Checks if two points are equal. */
bool IsPointEqual(const cv::Point &a, const cv::Point &b);


/** @brief Compute the min/max values from the given vector. */
template <typename T, int Dim>
void MinMaxVec(const std::vector<cv::Vec<T,Dim>> &vecs, cv::Vec<T,Dim> &min_v, cv::Vec<T,Dim> &max_v) {
  if (vecs.empty())
    return;
  min_v = vecs[0];
  max_v = vecs[0];

  for (const auto &v : vecs)
  {
    for (int i = 0; i < Dim; ++i)
    {
      if (v.val[i] < min_v.val[i])
        min_v.val[i] = v.val[i];
      if (v.val[i] > max_v.val[i])
        max_v.val[i] = v.val[i];
    }
  }
}


template<typename T>
T MaxPixelValue(int cv_depth) {
  switch (cv_depth) {
    case CV_8U:
      return static_cast<T>(255);
    case CV_8S:
      return static_cast<T>(127);
    case CV_16U:
      return static_cast<T>(65535);
    case CV_16S:
      return static_cast<T>(32767);
    case CV_32S:
      return static_cast<T>(2147483647);
    case CV_32F:
      return static_cast<T>(std::numeric_limits<float>::max());
    case CV_64F:
      return static_cast<T>(std::numeric_limits<double>::max());
    default:
      VCP_ERROR("Invalid OpenCV depth '" << cv_depth << "'");
  }
}



/** @brief Angle conversion. */
inline double Deg2Rad(double deg)
{
  return deg * MATH_PI / 180.0;
}


/** @brief Angle conversion. */
inline double Rad2Deg(double rad)
{
  return rad * 180.0 / MATH_PI;
}


/** @brief Computes the mathematical modulus (C++'s % operator implements the symmetric variant).
 * Since C++11, long long is guaranteed to have at least 64 bits: https://en.cppreference.com/w/cpp/language/types
 */
long long Mod(long long a, long long divisor);


/** @brief Computes the GCD using the Euclidean algorithm. */
long long GreatestCommonDivisor(long long a, long long b);


/** @brief Computes the LCM by leveraging GCD. */
long long LeastCommonMultiple(long long a, long long b);


/** @brief Unoptimized prime factorization for the given number.
 * Exemplary returns: {2, 2} for num = 4; input must be >= 2;
 */
std::vector<long long> PrimeFactors(long long num);

// TODO prime generator, kind of: https://paoloseverini.wordpress.com/2014/06/09/generator-functions-in-c/
//    input iterator: https://stackoverflow.com/questions/9059187/equivalent-c-to-python-generator-pattern

} // namespace math
} // namespace vcp

#endif // __VCP_MATH_COMMON_H__
