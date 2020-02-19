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
// end TODO

using std::cout;
using std::endl;
using Eigen::MatrixXf;
using Eigen::MatrixXi;

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

pcl::PointCloud<pcl::PointXYZ>::Ptr cdcpd(
        const cv::Mat& rgb,
        const cv::Mat& depth,
        const cv::Mat& mask,
        const cv::Mat& intrinsics)
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // TODO this should be a function if it's still used when we have correspondence
    for (int i = 0; i < points_mat.cols; ++i)
    {
        auto cv_point = points_mat.at<cv::Vec3d>(0, i);
        pcl::PointXYZ pt(cv_point(0), cv_point(1), cv_point(2));
        cloud->push_back(pt);
    }

    /// Box filter
    // TODO what is last element in vector?
    // TODO this needs to be a class so these can be members.
    // TODO also, we need to find the max and min and store them for next time
    // TODO also, we need to add .1m to each one.
    Eigen::Vector4f last_lower_bounding_box(-5.0, -5.0, -5.0, 1.0);
    Eigen::Vector4f last_upper_bounding_box(5.0, 5.0, 5.0, 1.0);
    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(last_lower_bounding_box);
    box_filter.setMax(last_upper_bounding_box);
    box_filter.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    box_filter.filter(*cloud_box_filtered);

    /// VoxelGrid filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fully_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);   
    sor.setLeafSize(0.02f, 0.02f, 0.02f);   
    sor.filter(*cloud_fully_filtered);

    /// CPD step

    // Construct the template
    // TODO template should probably be passed into the library
    // TODO you should use a PCL point cloud for the template
    MatrixXf template_vertices(3, 50);
    template_vertices.setZero();
    template_vertices.row(0).setLinSpaced(50, 0, 1);
    MatrixXi template_edges(49, 2);
    template_edges(0, 0) = 0;
    template_edges(template_edges.rows() - 1, 1) = 49;
    for (int i = 1; i <= template_edges.rows() - 1; ++i)
    {
        template_edges(i, 0) = i;
        template_edges(i - 1, 1) = i;
    }

    cout << "template_vertices" << endl;
    cout << template_vertices << endl;
    cout << "template_edges" << endl;
    cout << template_edges << endl;

    // CPD (TODO make into a function)
    double tolerance = 1e-4;
    double alpha = 3.0;
    double beta = 1.0;
    double w = 0.1;
    int max_iterations = 100;
    double init_sigma_scale = 1.0 / 8;

    // TODO getMatrixXfMap is not the right size. For some reason there are 4 dims?
    const MatrixXf& X = cloud_fully_filtered->getMatrixXfMap().topRows(3);
    const MatrixXf& Y = template_vertices;

    double sigma2 = initial_sigma2(X, Y);
    MatrixXf G = gaussian_kernel(Y, beta);
    cout << "sigma2 is: " << sigma2 << endl;

    int iterations = 0;
    double error = tolerance + 1; // loop runs the first time
    while (iterations <= max_iterations && error > tolerance)
    {
        double qprev = sigma2;

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
                P(i, j) = (X.col(j) - Y.col(i)).squaredNorm();
            }
        }

        float c = std::pow(2 * M_PI * sigma2, static_cast<double>(D) / 2);
        c *= w / (1 - w);
        c *= static_cast<double>(M) / N;

        P = (-P / (2 * sigma2)).array().exp();
        // TODO prior
        // if self.params.Y_emit_prior is not None:
        //      P *= self.params.Y_emit_prior[:, np.newaxis]

        MatrixXf den = P.colwise().sum().replicate(M, 1);
        // TODO ignored den[den == 0] = np.finfo(float).eps because seriously
        den.array() += c;

        P.array() /= den.array();

        // Maximization step
        MatrixXf Pt1 = P.colwise().sum();
        MatrixXf P1 = P.rowwise().sum();
        float Np = P1.sum();

        // TODO use LLE
        // this should be in the else of an if/else for LLE
        MatrixXf A = (P1.asDiagonal() * G) + alpha * sigma2 * MatrixXf::Identity(M, M);
        MatrixXf B = (P * X.transpose()) - P1.asDiagonal() * Y.transpose();
        cout << "A is: " << endl;
        cout << A << endl;
        cout << "B is: " << endl;
        cout << B << endl;
        // TODO solve this
        MatrixXf W = A.llt().solve(B);
        cout << "W is: " << endl;
        cout << W << endl;

        MatrixXf TY = Y + (G * W).transpose();
        MatrixXf xPxtemp = (X.array() * X.array()).colwise().sum();
        MatrixXf xPxMat = Pt1 * xPxtemp.transpose();
        assert(xPxMat.rows() == 1 && xPxMat.cols() == 1);
        double xPx = xPxMat.sum();
        MatrixXf yPytemp = (TY.array() * TY.array()).colwise().sum();
        MatrixXf yPyMat = P1.transpose() * yPytemp.transpose();
        assert(yPyMat.rows() == 1 && yPyMat.cols() == 1);
        double yPy = yPyMat.sum();
        double trPXY = (TY.array() * (P * X.transpose()).transpose().array()).sum();
        sigma2 = (xPx - 2 * trPXY + yPy) / (Np * D);

        if (sigma2 <= 0)
        {
            sigma2 = tolerance / 10;
        }
        error = std::abs(sigma2 - qprev);

        // TODO do I need to care about the callback?

        iterations++;
        cout << "TY: " << endl;
        cout << TY << endl;
    }

    return cloud_fully_filtered;
}
