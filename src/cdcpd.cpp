#include "cdcpd/cdcpd.h"
#include <cassert>
#include <Eigen/Dense>
#include "opencv2/imgcodecs.hpp" // TODO remove after not writing images
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/eigen.hpp"
#include <opencv2/rgbd.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

// TODO rm
#include <iostream>
#include <fstream>
// end TODO

using std::cout;
using std::endl;
using cv::Mat;
using Eigen::MatrixXf;
using Eigen::MatrixXi;
using Eigen::Vector3f;
using Eigen::Vector4f;

double initial_sigma2(const MatrixXf& X, const MatrixXf& Y)
{
    double total_error = 0.0;
    assert(X.rows() == Y.rows());
    for (int i = 0; i < X.cols(); ++i)
    {
        for (int j = 0; j < Y.cols(); ++j)
        {
            total_error += (X.col(i) - Y.col(j)).squaredNorm();
        }
    }
    return total_error / (X.cols() * Y.cols() * X.rows());
}

MatrixXf gaussian_kernel(const MatrixXf& Y, double beta)
{
    MatrixXf diff(Y.cols(), Y.cols());
    diff.setZero();
    for (int i = 0; i < Y.cols(); ++i)
    {
        for (int j = 0; j < Y.cols(); ++j)
        {
            diff(i, j) = (Y.col(i) - Y.col(j)).squaredNorm();
        }
    }
    MatrixXf kernel = (-diff / (2 * beta)).array().exp();
    return kernel;
}

CDCPD::CDCPD(const Mat& _intrinsics) : 
    intrinsics(_intrinsics),
    last_lower_bounding_box(-5.0, -5.0, -5.0, 1.0),
    last_upper_bounding_box(5.0, 5.0, 5.0, 1.0),
    tolerance(1e-4),
    alpha(3.0),
    beta(1.0),
    w(0.1),
    initial_sigma_scale(1.0 / 8),
    max_iterations(100)
{
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CDCPD::operator()(
        const cv::Mat& rgb,
        const cv::Mat& depth,
        const cv::Mat& mask,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud,
        const MatrixXi& template_edges)
{
    assert(rgb.type() == CV_8UC3);
    assert(depth.type() == CV_16U);
    assert(mask.type() == CV_8U);
    assert(intrinsics.type() == CV_64F);
    assert(intrinsics.rows == 3 && intrinsics.cols == 3);
    assert(rgb.rows == depth.rows && rgb.cols == depth.cols);

    /// Filtering step
    // Get a mask for all the valid depths
    cv::Mat depth_mask = depth != 0;
    cv::Mat combined_mask = mask & depth_mask;

    cv::Mat points_mat;
    cv::rgbd::depthTo3d(depth, intrinsics, points_mat, combined_mask);

    // TODO later we will need correspondence between depth image and points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // TODO this should be a function if it's still used when we have correspondence
    for (int i = 0; i < points_mat.cols; ++i)
    {
        auto cv_point = points_mat.at<cv::Vec3d>(0, i);
        pcl::PointXYZ pt(cv_point(0), cv_point(1), cv_point(2));
        cloud->push_back(pt);
    }

    /// Box filter
    // TODO what is last element in vector?
    pcl::CropBox<pcl::PointXYZ> box_filter;
    // Set the filters, allowing a bit more flexibility
    box_filter.setMin(last_lower_bounding_box - Vector4f(0.1, 0.1, 0.1, 0.0));
    box_filter.setMax(last_upper_bounding_box + Vector4f(0.1, 0.1, 0.1, 0.0));
    box_filter.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    box_filter.filter(*cloud_box_filtered);
    // last_lower_bounding_box, last_upper_bounding_box set after the VoxelGrid filter

    /// VoxelGrid filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fully_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);   
    sor.setLeafSize(0.02f, 0.02f, 0.02f);   
    sor.filter(*cloud_fully_filtered);

    // Set last_lower_bounding_box, last_upper_bounding_box to remember bounds
    // TODO this should probably be lower, for template?
    // pcl::getMinMax3D(*cloud_fully_filtered, last_lower_bounding_box, last_upper_bounding_box);

    /// CPD step

    // TODO maybe use a PCL point cloud for the template
    // CPD (TODO make into a function)

    const MatrixXf& X = cloud_fully_filtered->getMatrixXfMap().topRows(3);
    const MatrixXf& Y = template_cloud->getMatrixXfMap().topRows(3);

    cout << "X" << endl;
    cout << X.sum() << endl;
    cout << "Y" << endl;
    cout << Y.sum() << endl;

    std::ofstream file("../../points.txt");
    file << X << endl;
    file.close();

    MatrixXf G = gaussian_kernel(Y, beta);
    MatrixXf TY = Y;
    double sigma2 = initial_sigma2(X, TY) * initial_sigma_scale;
    cout << "sigma2 is: " << sigma2 << endl;

    int iterations = 0;
    double error = tolerance + 1; // loop runs the first time

    while (iterations <= max_iterations && error > tolerance)
    {
        double qprev = sigma2;
        cout << "qprev" << endl;
        cout << qprev << endl;

        // Expectation step
        int N = X.cols();
        int M = Y.cols();
        int D = Y.rows();

        // P(i, j) is the distance squared from template point i to the observed point j
        MatrixXf P(M, N);
        for (int i = 0; i < M; ++i)
        {
            for (int j = 0; j < N; ++j)
            {
                P(i, j) = (X.col(j) - TY.col(i)).squaredNorm();
            }
        }

        cout << "P1" << endl;
        cout << P.sum() << endl;

        float c = std::pow(2 * M_PI * sigma2, static_cast<double>(D) / 2);
        c *= w / (1 - w);
        c *= static_cast<double>(M) / N;

        cout << "c" << endl;
        cout << c << endl;

        P = (-P / (2 * sigma2)).array().exp().matrix();
        cout << "P2" << endl;
        // cout << P << endl;
        cout << P.sum() << endl;
        // TODO prior
        // if self.params.Y_emit_prior is not None:
        //      P *= self.params.Y_emit_prior[:, np.newaxis]

        MatrixXf den = P.colwise().sum().replicate(M, 1);
        // TODO ignored den[den == 0] = np.finfo(float).eps because seriously
        den.array() += c;

        cout << "den" << endl;
        // cout << den << endl;
        cout << den.sum() << endl;

        P = P.cwiseQuotient(den);

        cout << "P3" << endl;
        cout << P.sum() << endl;

        // Maximization step
        MatrixXf Pt1 = P.colwise().sum();
        MatrixXf P1 = P.rowwise().sum();
        float Np = P1.sum();
        
        cout << "Np" << endl;
        cout << Np << endl;

        // TODO use LLE
        // this should be in the else of an if/else for LLE
        MatrixXf A = (P1.asDiagonal() * G) + alpha * sigma2 * MatrixXf::Identity(M, M);
        MatrixXf B = (P * X.transpose()) - P1.asDiagonal() * Y.transpose();
        cout << "A" << endl;
        cout << A.sum() << endl;
        cout << "B" << endl;
        cout << B.sum() << endl;
        MatrixXf W = A.colPivHouseholderQr().solve(B);
        cout << "W.rows()" << endl;
        cout << W.rows() << endl;
        cout << "W.cols()" << endl;
        cout << W.cols() << endl;
        cout << "W" << endl;
        cout << W.sum() << endl;

        TY = Y + (G * W).transpose();
        cout << "TY" << endl;
        cout << TY.sum() << endl;
        MatrixXf xPxtemp = (X.array() * X.array()).colwise().sum();
        MatrixXf xPxMat = Pt1 * xPxtemp.transpose();
        assert(xPxMat.rows() == 1 && xPxMat.cols() == 1);
        double xPx = xPxMat.sum();
        cout << "xPx" << endl;
        cout << xPx << endl;
        MatrixXf yPytemp = (TY.array() * TY.array()).colwise().sum();
        MatrixXf yPyMat = P1.transpose() * yPytemp.transpose();
        assert(yPyMat.rows() == 1 && yPyMat.cols() == 1);
        double yPy = yPyMat.sum();
        cout << "yPy" << endl;
        cout << yPy << endl;
        double trPXY = (TY.array() * (P * X.transpose()).transpose().array()).sum();
        cout << "trPXY" << endl;
        cout << trPXY << endl;
        sigma2 = (xPx - 2 * trPXY + yPy) / (Np * D);

        if (sigma2 <= 0)
        {
            sigma2 = tolerance / 10;
        }
        error = std::abs(sigma2 - qprev);

        // TODO do I need to care about the callback?

        iterations++;
        // cout << "TY: " << endl;
        // cout << TY << endl;
    }

    cout << "Final check" << endl;
    cout << TY << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cpd_out(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < TY.cols(); ++i)
    {
        const Vector3f& pt = TY.col(i);
        cpd_out->push_back(pcl::PointXYZ(pt(0), pt(1), pt(2)));
    }

    return cpd_out;
}
