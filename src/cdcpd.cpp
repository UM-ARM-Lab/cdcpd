#include "cdcpd/cdcpd.h"
#include <cassert>
#include <Eigen/Dense>
#include "opencv2/imgcodecs.hpp" // TODO remove after not writing images
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/eigen.hpp"
#include <opencv2/rgbd.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

// TODO rm
#include <iostream>
#include <fstream>
// end TODO

using std::vector;
using std::cout;
using std::endl;
using cv::Mat;
using Eigen::MatrixXf;
using Eigen::MatrixXi;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::VectorXf;

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

MatrixXf barycenter_kneighbors_graph(const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                                     int lle_neighbors,
                                     double reg)
{
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud = kdtree.getInputCloud();
    assert(cloud->height == 1);
    MatrixXi adjacencies = MatrixXi(cloud->width, lle_neighbors);
    MatrixXf B = MatrixXf::Zero(cloud->width, lle_neighbors);
    MatrixXf v = VectorXf::Ones(lle_neighbors);
    for (size_t i = 0; i < cloud->width; ++i)
    {
        vector<int> neighbor_inds(lle_neighbors + 1);
        vector<float> neighbor_dists(lle_neighbors + 1);
        kdtree.nearestKSearch(i, lle_neighbors + 1, neighbor_inds, neighbor_dists);
        MatrixXf C(lle_neighbors, 3);
        for (size_t j = 1; j < neighbor_inds.size(); ++j)
        {
            C.row(j - 1) = cloud->points[neighbor_inds[j]].getVector3fMap()
                - cloud->points[i].getVector3fMap();
            adjacencies(i, j - 1) = neighbor_inds[j];
        }
        MatrixXf G = C * C.transpose();
        float R = reg;
        float tr = G.trace();
        if (tr > 0)
        {
            R *= tr;
        }
        G.diagonal().array() += R;
        VectorXf w = G.llt().solve(v);
        B.row(i) = w / w.sum();
    }
    MatrixXf graph = MatrixXf::Zero(cloud->width, cloud->width);
    for (size_t i = 0; i < graph.rows(); ++i)
    {
        for (size_t j = 0; j < lle_neighbors; ++j)
        {
            graph(i, adjacencies(i, j)) = B(i, j);
        }
    }
    return graph;
}

MatrixXf locally_linear_embedding(pcl::PointCloud<pcl::PointXYZ>::ConstPtr template_cloud,
                                  int lle_neighbors,
                                  double reg)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (template_cloud);
    MatrixXf W = barycenter_kneighbors_graph(kdtree, lle_neighbors, reg);
    MatrixXf M = (W.transpose() * W) - W.transpose() - W;
    M.diagonal().array() += 1;
    return M;
}

CDCPD::CDCPD(pcl::PointCloud<pcl::PointXYZ>::ConstPtr template_cloud,
             const Mat& _intrinsics) : 
    intrinsics(_intrinsics),
    last_lower_bounding_box(-5.0, -5.0, -5.0, 1.0),
    last_upper_bounding_box(5.0, 5.0, 5.0, 1.0),
    lle_neighbors(8),
    m_lle(locally_linear_embedding(template_cloud, lle_neighbors, 1e-3)),
    tolerance(1e-4),
    alpha(3.0),
    beta(1.0),
    w(0.1),
    initial_sigma_scale(1.0 / 8),
    start_lambda(1.0),
    annealing_factor(0.6),
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

    Eigen::IOFormat np_fmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "");

    const MatrixXf& X = cloud_fully_filtered->getMatrixXfMap().topRows(3);
    const MatrixXf& Y = template_cloud->getMatrixXfMap().topRows(3);

    auto to_file = [&np_fmt](const std::string& fname, const MatrixXf& mat) {
        std::ofstream(fname) << mat.format(np_fmt);
    };

    // TODO TESTING remove this
    to_file("cpp_X.txt", X);
    to_file("cpp_Y.txt", Y);
    
    // const MatrixXf X = 
    // const MatrixXf Y = 

    MatrixXf G = gaussian_kernel(Y, beta);
    to_file("cpp_G.txt", G);
    MatrixXf TY = Y;
    double sigma2 = initial_sigma2(X, TY) * initial_sigma_scale;

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
                P(i, j) = (X.col(j) - TY.col(i)).squaredNorm();
            }
        }

        float c = std::pow(2 * M_PI * sigma2, static_cast<double>(D) / 2);
        c *= w / (1 - w);
        c *= static_cast<double>(M) / N;

        cout << "sigma2" << endl;
        cout << sigma2 << endl;
        cout << "c" << endl;
        cout << c << endl;

        P = (-P / (2 * sigma2)).array().exp().matrix();
        // TODO prior
        // if self.params.Y_emit_prior is not None:
        //      P *= self.params.Y_emit_prior[:, np.newaxis]

        MatrixXf den = P.colwise().sum().replicate(M, 1);
        // TODO ignored den[den == 0] = np.finfo(float).eps because seriously
        den.array() += c;
        to_file(std::string("cpp_den_") + std::to_string(iterations) + ".txt", den);

        P = P.cwiseQuotient(den);
        to_file(std::string("cpp_P_") + std::to_string(iterations) + ".txt", P);

        // Maximization step
        MatrixXf Pt1 = P.colwise().sum();
        MatrixXf P1 = P.rowwise().sum();
        float Np = P1.sum();
        
        // This is the code to use if you are not using LLE
        // MatrixXf A = (P1.asDiagonal() * G) + alpha * sigma2 * MatrixXf::Identity(M, M);
        // MatrixXf B = (P * X.transpose()) - P1.asDiagonal() * Y.transpose();
        // MatrixXf W = A.colPivHouseholderQr().solve(B);
        float lambda = start_lambda * std::pow(annealing_factor, iterations + 1);
        MatrixXf A = (P1.asDiagonal() * G) 
            + alpha * sigma2 * MatrixXf::Identity(M, M)
            + sigma2 * lambda * (m_lle * G);
        MatrixXf p1d = P1.asDiagonal();
        MatrixXf B = (P * X.transpose()) - (p1d + sigma2 * lambda * m_lle) * Y.transpose();
        MatrixXf W = A.colPivHouseholderQr().solve(B);
        cout << "p1d" << endl;
        cout << p1d << endl;
        cout << "sigma2" << endl;
        cout << sigma2 << endl;
        cout << "lambda" << endl;
        cout << lambda << endl;
        // cout << std::scientific << std::setprecision(8);
        // cout << "m_lle" << endl;
        // cout << m_lle << endl;
        to_file("cpp_m_lle.txt", m_lle);
        // cout << "A" << endl;
        // cout << A << endl;
        to_file(std::string("cpp_A_") + std::to_string(iterations) + ".txt", A);
        // cout << "B" << endl;
        // cout << B << endl;
        to_file(std::string("cpp_B_") + std::to_string(iterations) + ".txt", B);
        // cout << "W" << iterations <<  endl;
        // cout << W << endl;
        to_file(std::string("cpp_W_") + std::to_string(iterations) + ".txt", W);

        TY = Y + (G * W).transpose();
        // cout << "TY" << endl;
        // cout << TY << endl;
        to_file(std::string("cpp_TY_") + std::to_string(iterations) + ".txt", TY);

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
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cpd_out(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < TY.cols(); ++i)
    {
        const Vector3f& pt = TY.col(i);
        cpd_out->push_back(pcl::PointXYZ(pt(0), pt(1), pt(2)));
    }

    return cpd_out;
}
