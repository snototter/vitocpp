#include "math_utils.h"
#include <gtest/gtest.h>

namespace vcp {
namespace utils {
namespace test {
//TODO add remaining functions
TEST(MathUtils, VecMean)
{
  const double kEps = 1e-5;
  const std::vector<int> vec = {1, 2, 3, 4, 5, 6, 7, 8, 9};

  EXPECT_NEAR(math::VecMean(vec), 5.0, kEps);
  EXPECT_NEAR(math::VecMean(vec, 0, vec.size(), 1), 5.0, kEps);
  EXPECT_NEAR(math::VecMean(vec, 0, 1, 2), 1.0, kEps);
  EXPECT_NEAR(math::VecMean(vec, 8, vec.size()), 9.0, kEps);
  EXPECT_NEAR(math::VecMean(vec, 7, vec.size()), 8.5, kEps);
  EXPECT_NEAR(math::VecMean(vec, 3, vec.size(), vec.size()), vec[3], kEps);
}

TEST(MathUtils, VecMin)
{
  const double kEps = 1e-5;
  const std::vector<int> vec = {75, 23, 3, 4, 5, 6, 7, 8, 9};

  EXPECT_NEAR(math::VecMin(vec, 0, vec.size(), 1), 3.0, kEps);
  EXPECT_NEAR(math::VecMin(vec, 0, 1, 2), 75.0, kEps);
  EXPECT_NEAR(math::VecMin(vec, 8, vec.size()), 9.0, kEps);
  EXPECT_NEAR(math::VecMin(vec, 7, vec.size()), 8.0, kEps);
  EXPECT_NEAR(math::VecMin(vec, 4, vec.size(), vec.size()), vec[4], kEps);
}

TEST(MathUtils, VecMax)
{
  const double kEps = 1e-5;
  const std::vector<int> vec = {75, 23, 3, 33, 5, 6, 7, 8, 9};

  EXPECT_NEAR(math::VecMax(vec, 0, vec.size(), 1), 75.0, kEps);
  EXPECT_NEAR(math::VecMax(vec, 0, 1, 2), 75.0, kEps);
  EXPECT_NEAR(math::VecMax(vec, 0, 1, 2), 75.0, kEps);
  EXPECT_NEAR(math::VecMax(vec, 8, vec.size()), 9.0, kEps);
  EXPECT_NEAR(math::VecMax(vec, 7, vec.size()), 9.0, kEps);
  EXPECT_NEAR(math::VecMax(vec, 1, vec.size(), 2), vec[3], kEps);
}

TEST(MathUtils, RoundBase)
{
  const double kEps = 1e-7;

  // Base 5
  EXPECT_NEAR(math::RoundBase(13,5), 15.0, kEps);
  EXPECT_NEAR(math::RoundBase(12.4,5), 10.0, kEps);

  // Base 2
  EXPECT_NEAR(math::RoundBase(1,2), 2.0, kEps);
  EXPECT_NEAR(math::RoundBase(0.1, 2), 0.0, kEps);

  // Base 10
  EXPECT_NEAR(math::RoundBase(1,10), 0.0, kEps);
  EXPECT_NEAR(math::RoundBase(14.9,10), 10.0, kEps);
  EXPECT_NEAR(math::RoundBase(15,10), 20.0, kEps);
  EXPECT_NEAR(math::RoundBase(12345,10), 12350.0, kEps);

  // Base < 1
  EXPECT_NEAR(math::RoundBase(0.999, 0.1), 1.0, kEps);
  EXPECT_NEAR(math::RoundBase(0.921, 0.05), 0.9, kEps);
  EXPECT_NEAR(math::RoundBase(0.671, 0.01), 0.67, kEps);
  EXPECT_NEAR(math::RoundBase(0.671, 0.1), 0.7, kEps);
}

} // namespace test
} // namespace utils
} // namespace vcp
