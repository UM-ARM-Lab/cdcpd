#include <gtest/gtest.h>

#include "arc_utilities/math_helpers.hpp"

using namespace EigenHelpers;

TEST(MathHelpers, CloseEnough_accurate_for_doubles_and_vectors) {
  EXPECT_TRUE(CloseEnough(1.0, 2.0, 1.1));
  EXPECT_TRUE(CloseEnough(-1.0, -2.0, 1.1));
  EXPECT_FALSE(CloseEnough(1.0, 2.0, 0.9));
  EXPECT_FALSE(CloseEnough(-1.0, -2.0, 0.9));

  std::vector<double> a{1.0, -1.0};
  std::vector<double> b{2.0, -2.0};
  EXPECT_TRUE(CloseEnough(a, b, 1.1));
  EXPECT_FALSE(CloseEnough(a, b, 0.9));
}

TEST(MathHelpers, Interpolate_accurate_for_double_and_vectors) {
  EXPECT_EQ(Interpolate(1.0, 2.0, 0.0), 1.0);
  EXPECT_EQ(Interpolate(1.0, 2.0, 1.0), 2.0);
  EXPECT_EQ(Interpolate(1.0, 2.0, 0.5), 1.5);

  std::vector<double> a{1.0, 1.0};
  std::vector<double> b{2.0, 3.0};
  EXPECT_TRUE(CloseEnough(Interpolate(a, b, 0.0), a, 0.0000001));
  EXPECT_TRUE(CloseEnough(Interpolate(a, b, 1.0), b, 0.0000001));
  EXPECT_TRUE(CloseEnough(Interpolate(a, b, 0.5), std::vector<double>{1.5, 2.0}, 0.0000001));
}

TEST(MathHelpers, Distance_functions_are_accurate_for_doubles) {
  double eps = 0.000000000001;
  std::vector<double> z{0.0, 0.0};
  std::vector<double> v{3.0, 4.0};
  std::vector<double> nv{-3.0, -4.0};
  EXPECT_NEAR(SquaredDistance(z, v), 25.0, eps);
  EXPECT_NEAR(Distance(z, v), 5.0, eps);
  EXPECT_NEAR(SquaredDistance(z, nv), 25.0, eps);
  EXPECT_NEAR(Distance(z, nv), 5.0, eps);
  EXPECT_NEAR(Distance(v, nv), 10.0, eps);
}

GTEST_API_ int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
