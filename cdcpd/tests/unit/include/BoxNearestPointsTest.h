#include <cdcpd/optimizer.h>
#include <gtest/gtest.h>

#include <iostream>

using Eigen::Matrix3Xf;

void expectMatrixEqual(Eigen::Matrix3Xf const& lhs, Eigen::Matrix3Xf const& rhs)
{
  // Compute a relative error tolerance based on magnitude of each element in the matrix.
  // We should expect that the values will match within 4 decimals places.
  Eigen::Matrix3Xf lhs_abs = lhs.cwiseAbs();
  Eigen::Matrix3Xf rhs_abs = rhs.cwiseAbs();
  Eigen::Matrix3Xf err_tol = (rhs_abs.cwiseMin(lhs_abs)) / 1e-4;
 
  Eigen::Matrix3Xf subtracted_abs = (rhs - lhs).cwiseAbs();
  // Make sure all elements in the resulting subtracted absolute value matrix are smaller in
  // magnitude than the error tolerance.
  for (std::size_t i = 0; i < lhs.rows(); ++i)
  {
    for (std::size_t j = 0; j < lhs.cols(); ++j)
    {
      EXPECT_TRUE(subtracted_abs(i, j) <= err_tol(i, j));
    }
  }
}

TEST(BoxNearestPointsTest, testCdcpdOptimizer)
{
  Matrix3Xf init_temp(3,5);
  init_temp(0, 0) =  1.0; init_temp(1, 0) = 0.0; init_temp(2, 0) = -1.0; // near: (sqrt(2)/2, 0, 0); normal: (1/sqrt(3), 0, -sqrt(2/3))
  init_temp(0, 1) =  0.0; init_temp(1, 1) = 2.0; init_temp(2, 1) =  3.0; // near: (0, 0.5, 1.5); normal: (0, sqrt(2)/2, sqrt(2)/2)
  init_temp(0, 2) = -2.0; init_temp(1, 2) = 0.0; init_temp(2, 2) =  1.5; // near: (-sqrt(2)/2, 0, 1.5); normal: (-1, 0, 0)
  init_temp(0, 3) =  0.0; init_temp(1, 3) = 0.0; init_temp(2, 3) =  3.0; // near: (0, 0, 2); normal: (0, 0, 1)
  init_temp(0, 4) =  0.5; init_temp(1, 4) = 0.5; init_temp(2, 4) =  1.0; // near: (sqrt(2)/2, 0.5, 1.0); normal: (1, 0, 0)
  Matrix3Xf last_temp = init_temp;
  
  shape_msgs::SolidPrimitive box;
  box.type = shape_msgs::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[shape_msgs::SolidPrimitive::BOX_X] = sqrt(2);
  box.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = sqrt(2);
  box.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = sqrt(2);
  
  geometry_msgs::Pose pose;
  geometry_msgs::Point center;
  center.x = 0; center.y = 0; center.z = 1;
  pose.position = center;

  geometry_msgs::Quaternion orien;
  orien.x = 0.3826834; orien.y = 0; orien.z = 0; orien.w = 0.9238795;
  pose.orientation = orien;

  Optimizer opt(init_temp, last_temp, 1.0, 1.0, 10.0);
  std::tuple<Points, Normals> res = opt.test_box(init_temp, box, pose);

  Eigen::Matrix<float, 3, 5> points_expected;
  points_expected <<  0.707107F   , 0.0F, -0.707107F  , 0.0F        , 0.707107F,
                     -5.96046e-08F, 0.5F, 4.47035e-08F, 5.96046e-08F, 0.5F,
                      5.96046e-08F, 1.5F, 1.5F        , 2.0F        , 1.0F;

  Eigen::Matrix<float, 3, 5> normals_expected;
  normals_expected <<  1.0F        , 0.0F     , -1.0F, 0.0F        , 1.0F,  
                      -1.19209e-07F, 0.707107F,  0.0F, 1.19209e-07F, 0.0F,
                      -1.41421F    , 0.707107F,  0.0F, 1.41421F    , 0.0F;

  expectMatrixEqual(std::get<0>(res), points_expected);
  expectMatrixEqual(std::get<1>(res), normals_expected);
}
