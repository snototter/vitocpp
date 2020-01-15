#include "common.h"
#include "conversions.h"

#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <string>

namespace vcp {
namespace math {
namespace test {
const double kEps = 1e-10;

//TODO test:
// IsPointEqual
// eps_zero
// IsVecEqual ulp
// IsVecEqual 1e-10
// Deg2Rad & Rad2Deg

// TODO separate conversion test

TEST(Math, DEFINITIONS)
{
  EXPECT_TRUE(vcp::math::eps_equal(MATH_PI, 3.141592653589793238462643383279, 1));
  EXPECT_TRUE(vcp::math::eps_equal(MATH_2PI, 6.283185307179586476925286766559, 1));
  EXPECT_TRUE(vcp::math::eps_equal(MATH_LOG2, 0.69314718055994530941723212145818, 1));
}

TEST(Math, EpsEquality)
{
  double a = 5e-5;
  double b = 5e-8;
  for (int i = 0; i < 20; ++i)
  {
//    std::cout << "a == b ? " << a << ", " << b << std::endl;
    if (i == 3)
      EXPECT_TRUE(vcp::math::eps_equal(a, b, 2));
    else
      EXPECT_FALSE(vcp::math::eps_equal(a, b, 2));
    b *= 10.0;
  }

  EXPECT_TRUE(vcp::math::eps_equal(a, a + 1.13e-21, 2));
  EXPECT_FALSE(vcp::math::eps_equal(13.75*a, 13.75*a + 1.13e-18, 2));

  EXPECT_FALSE(vcp::math::eps_zero(1e-3));
  EXPECT_FALSE(vcp::math::eps_zero(1e-7));
  EXPECT_FALSE(vcp::math::eps_zero(-1e-9));
  EXPECT_TRUE(vcp::math::eps_zero(-1e-11));
  EXPECT_TRUE(vcp::math::eps_zero(1e-12));
}


void CheckPrimeFactorization(const std::vector<long long> &pf)
{
  long long num = 1;
  for (size_t i = 0; i < pf.size(); ++i)
    num *= pf[i];
  const auto factors = PrimeFactors(num);
  EXPECT_EQ(factors.size(), pf.size());
  for (size_t i = 0; i < factors.size(); ++i)
    EXPECT_EQ(factors[i], pf[i]);
}

TEST(Math, PrimeFactors)
{
  CheckPrimeFactorization({ 2, 2, 3, 7, 23, 101});
  CheckPrimeFactorization({ 5, 11, 19, 41});
  CheckPrimeFactorization({1}); // Should yield a warning
}

TEST(Math, Mod)
{
  // Symmetric modulo
  EXPECT_EQ(vcp::math::Mod(3,4), 3);
  EXPECT_EQ(vcp::math::Mod(-3,4), 1);
  EXPECT_EQ(vcp::math::Mod(-2,5), 3);
  EXPECT_EQ(vcp::math::Mod(2,5), 2);
  EXPECT_EQ(vcp::math::Mod(-5,4), 3);
  EXPECT_EQ(vcp::math::Mod(5,4), 1);
  EXPECT_EQ(vcp::math::Mod(19,19), 0);
  EXPECT_EQ(vcp::math::Mod(-19,-19), 0);
  EXPECT_EQ(vcp::math::Mod(19,2), 1);
}
TEST(Math, GcdLcm)
{
  EXPECT_EQ(GreatestCommonDivisor(8,12), 4);
  EXPECT_EQ(GreatestCommonDivisor(12,8), 4);
  EXPECT_EQ(GreatestCommonDivisor(12345678,982764534),6);
  EXPECT_EQ(LeastCommonMultiple(12,8), 24);
  EXPECT_EQ(LeastCommonMultiple(8,12), 24);
  EXPECT_EQ(LeastCommonMultiple(21,6), 42);
}
} // namespace test
} // namespace math
} // namespace vcp

