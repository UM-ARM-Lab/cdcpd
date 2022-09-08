#pragma once

#include <Eigen/Dense>

void expectEigenMatsEqual(Eigen::Matrix3Xf const& lhs, Eigen::Matrix3Xf const& rhs)
{
    EXPECT_TRUE(lhs.rows() == rhs.rows())<< "Eigen::Matrix3Xf Failed! Test:\n" << lhs.rows() << "\nTruth:\n" << rhs.rows();
    EXPECT_TRUE(lhs.cols() == rhs.cols())<< "Eigen::Matrix3Xf Failed! Test:\n" << lhs.cols() << "\nTruth:\n" << rhs.cols();
    EXPECT_TRUE(lhs.isApprox(rhs, 1e-3)) << "Eigen::Matrix3Xf Failed! Test:\n" << lhs << "\nTruth:\n" << rhs;
}

void expectEigenMatsEqual(Eigen::Matrix2Xi const& lhs, Eigen::Matrix2Xi const& rhs)
{
    EXPECT_TRUE(lhs.rows() == rhs.rows())<< "Eigen::Matrix2Xi Failed! Test:\n" << lhs.rows() << "\nTruth:\n" << rhs.rows();
    EXPECT_TRUE(lhs.cols() == rhs.cols())<< "Eigen::Matrix2Xi Failed! Test:\n" << lhs.cols() << "\nTruth:\n" << rhs.cols();
    EXPECT_TRUE(lhs.isApprox(rhs))       << "Eigen::Matrix2Xi Failed! Test:\n" << lhs << "\nTruth:\n" << rhs;
}
