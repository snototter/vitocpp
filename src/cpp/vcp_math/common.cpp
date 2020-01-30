#include "common.h"

#include <sstream>
#include <vcp_utils/vcp_logging.h>

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::math"
namespace vcp
{
namespace math
{
bool IsPointEqual(const cv::Point &a, const cv::Point &b)
{
  if (a.x != b.x || a.y != b.y)
    return false;
  return true;
}

// TODO should we allow negative divisors? => Nope
// Since C++11, long long is guaranteed to have at least 64 bits: https://en.cppreference.com/w/cpp/language/types
long long Mod(long long a, long long divisor)
{
  if (a < 0)
    return ((a % divisor) + divisor) % divisor;
  else
    return a % divisor;
}


long long GreatestCommonDivisor(long long a, long long b)
{
  while (a != b)
  {
    if (a > b)
      a = a - b;
    else
      b = b - a;
  }
  return a;
}

long long LeastCommonMultiple(long long a, long long b)
{
  if (a > b)
    return (std::abs(a)/GreatestCommonDivisor(a,b)) * std::abs(b);
  else
    return (std::abs(b)/GreatestCommonDivisor(a,b)) * std::abs(a);
}


std::vector<long long> PrimeFactors(long long num)
{
  std::vector<long long> factors;
  if (num == 0)
  {
    VCP_LOG_FAILURE("0 is an invalid input for prime factorization!");
    return factors;
  }

  if (num == 1)
  {
    VCP_LOG_WARNING("1 is an invalid input for prime factorization. I'm returning '1'.");
    return {1};
  }

  while (num % 2 == 0)
  {
    factors.push_back(2);
    num >>= 1;
  }

  // The remaining number is odd
  const long long sqrt_num = static_cast<long long>(std::ceil(std::sqrt(static_cast<long double>(num))));
  for (long long i = 3; i <= sqrt_num; i += 2)
  {
    while (num % i == 0)
    {
      factors.push_back(i);
      num /= i;
    }
  }
  if (num > 2)
    factors.push_back(num);
  return factors;
}

//} // namespace utils
} // namespace math
} // namespace vcp
