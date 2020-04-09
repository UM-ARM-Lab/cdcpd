#include <algorithm>
#include "cdcpd/cdcpd.h"
#include "cdcpd/optimizer.h"
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
using Eigen::Matrix3Xf;
using Eigen::MatrixXi;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::VectorXf;
using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PointXYZRGB;

PointCloud<PointXYZ>::Ptr mat_to_cloud(const Eigen::Matrix3Xf& mat)
{
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    for (int i = 0; i < mat.cols(); ++i)
    {
        const Vector3f& pt = mat.col(i);
        cloud->push_back(PointXYZ(pt(0), pt(1), pt(2)));
    }
    return cloud;
}

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

MatrixXf barycenter_kneighbors_graph(const pcl::KdTreeFLANN<PointXYZ>& kdtree,
                                     int lle_neighbors,
                                     double reg)
{
    PointCloud<PointXYZ>::ConstPtr cloud = kdtree.getInputCloud();
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

MatrixXf locally_linear_embedding(PointCloud<PointXYZ>::ConstPtr template_cloud,
                                  int lle_neighbors,
                                  double reg)
{
    pcl::KdTreeFLANN<PointXYZ> kdtree;
    kdtree.setInputCloud (template_cloud);
    MatrixXf W = barycenter_kneighbors_graph(kdtree, lle_neighbors, reg);
    MatrixXf M = (W.transpose() * W) - W.transpose() - W;
    M.diagonal().array() += 1;
    return M;
}

CDCPD::CDCPD(PointCloud<PointXYZ>::ConstPtr template_cloud,
             const Mat& _P_matrix) : 
    original_template(template_cloud->getMatrixXfMap().topRows(3)),
    P_matrix(_P_matrix),
    last_lower_bounding_box(-5.0, -5.0, -5.0),
    last_upper_bounding_box(5.0, 5.0, 5.0),
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

/*
 * Return a non-normalized probability that each of the tracked vertices produced any detected point.
 */
Eigen::VectorXf visibility_prior(const Matrix3Xf vertices, 
                                 const Eigen::Matrix3f& intrinsics,
                                 const cv::Mat& depth,
                                 const cv::Mat& mask,
                                 float k=1e1)
{
    Eigen::IOFormat np_fmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "");
    auto to_file = [&np_fmt](const std::string& fname, const MatrixXf& mat) {
        std::ofstream(fname) << mat.format(np_fmt);
    };

    to_file("/home/steven/catkin/cpp_verts.txt", vertices);
    to_file("/home/steven/catkin/cpp_intrinsics.txt", intrinsics);
    // cout << "vertices" << endl;
    // cout << vertices << endl;
    // cout << "intrinsics" << endl;
    // cout << intrinsics << endl;
    // Project the vertices to get their corresponding pixel coordinate
    Eigen::Matrix3Xf image_space_vertices = intrinsics * vertices;
    // Homogeneous coords: divide by third row
    // cout << "image_space_vertices" << endl;
    // cout << image_space_vertices << endl;
    image_space_vertices.row(0).array() /= image_space_vertices.row(2).array();
    image_space_vertices.row(1).array() /= image_space_vertices.row(2).array();
    // cout << "image_space_vertices, updated" << endl;
    // cout << image_space_vertices << endl;
    for (int i = 0; i < image_space_vertices.cols(); ++i)
    {
        float x = image_space_vertices(0, i);
        float y = image_space_vertices(1, i);
        image_space_vertices(0, i) = std::min(std::max(x, 0.0f), static_cast<float>(depth.cols));
        image_space_vertices(1, i) = std::min(std::max(y, 0.0f), static_cast<float>(depth.rows));
    }
    // image_space_vertices.topRows(2) = image_space_vertices.topRows(2).array().min(0);
    // image_space_vertices.row(0) = image_space_vertices.row(0).array().max(depth.cols);
    // image_space_vertices.row(1) = image_space_vertices.row(1).array().max(depth.rows);
    to_file("/home/steven/catkin/cpp_projected.txt", image_space_vertices);
    // cout << "image_space_vertices, updated again" << endl;
    // cout << image_space_vertices << endl;

    // Get image coordinates
    Eigen::Matrix2Xi coords = image_space_vertices.topRows(2).cast<int>();

    for (int i = 0; i < coords.cols(); ++i)
    {
        coords(0, i) = std::min(std::max(coords(0, i), 0), depth.cols - 1);
        coords(1, i) = std::min(std::max(coords(1, i), 1), depth.rows - 1);
    }
    // coords.row(0) = coords.row(0).array().min(0);
    // coords.row(0) = coords.row(0).array().max(depth.cols - 1);
    // coords.row(1) = coords.row(1).array().min(1);
    // coords.row(1) = coords.row(1).array().max(depth.rows - 1);

    to_file("/home/steven/catkin/cpp_coords.txt", coords.cast<float>());

    // Find difference between point depth and image depth
    Eigen::VectorXf depth_diffs = Eigen::VectorXf::Zero(vertices.cols());
    for (int i = 0; i < vertices.cols(); ++i)
    {
        // cout << "depth at " << coords(1, i) << " " << coords(0, i) << endl;
        uint16_t raw_depth = depth.at<uint16_t>(coords(1, i), coords(0, i));
        if (raw_depth != 0)
        {
            depth_diffs(i) = vertices(2, i) - static_cast<float>(raw_depth) / 1000.0;
        }
        else
        {
            depth_diffs(i) = 0.02; // prevent numerical problems; taken from the Python code
        }
    }
    depth_diffs = depth_diffs.array().max(0);
    to_file("/home/steven/catkin/cpp_depth_diffs.txt", depth_diffs);

    Eigen::VectorXf depth_factor = depth_diffs.array().max(0.0);
    cv::Mat dist_img(depth.rows, depth.cols, depth.type());
    int maskSize = 5;
    cv::distanceTransform(~mask, dist_img, cv::noArray(), cv::DIST_L2, maskSize);
    cv::normalize(dist_img, dist_img, 0.0, 1.0, cv::NORM_MINMAX);
    imwrite("/home/steven/catkin/mask.png", mask);
    cv::Mat dist_copy = dist_img.clone();
    imwrite("/home/steven/catkin/dist_img.png", dist_copy * 255.0);

    Eigen::VectorXf dist_to_mask(vertices.cols());
    for (int i = 0; i < vertices.cols(); ++i)
    {
        dist_to_mask(i) = dist_img.at<float>(coords(1, i), coords(0, i));
    }
    VectorXf score = (dist_to_mask.array() * depth_factor.array()).matrix();
    VectorXf prob = (-k * score).array().exp().matrix();
    to_file("/home/steven/catkin/cpp_prob.txt", prob);
    return prob;
}

// TODO return both point clouds
// TODO based on the implementation here: 
// https://github.com/ros-perception/image_pipeline/blob/ \
// melodic/depth_image_proc/src/nodelets/point_cloud_xyzrgb.cpp
// Note that we expect that cx, cy, fx, fy are in the appropriate places in P
std::tuple<PointCloud<PointXYZRGB>::Ptr, PointCloud<PointXYZ>::Ptr, std::vector<cv::Point>>
point_clouds_from_images(const cv::Mat& depth_image,
                         const cv::Mat& rgb_image,
                         const cv::Mat& mask,
                         const Eigen::MatrixXf& P,
                         const Eigen::Vector3f lower_bounding_box_vec,
                         const Eigen::Vector3f upper_bounding_box_vec)
{
    assert(depth_image.cols == rgb_image.cols);
    assert(depth_image.rows == rgb_image.rows);
    assert(depth_image.type() == CV_16U);
    assert(P.rows() == 3 && P.cols() == 4);
    // Assume unit_scaling is the standard Kinect 1mm
    float unit_scaling = 1.0 / 1000.0;
    float constant_x = unit_scaling / P(0, 0);
    float constant_y = unit_scaling / P(1, 1);
    float center_x = P(0, 2);
    float center_y = P(1, 2);
    float bad_point = std::numeric_limits<float>::quiet_NaN();
    Eigen::Array<float, 3, 1> lower_bounding_box = lower_bounding_box_vec.array();
    Eigen::Array<float, 3, 1> upper_bounding_box = upper_bounding_box_vec.array();

    const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth_image.data[0]);
    int row_step = depth_image.step / sizeof(uint16_t);
    const uint8_t* rgb = &rgb_image.data[0];
    int rgb_step = 3; // TODO check on this
    int rgb_skip = rgb_image.step - rgb_image.cols * rgb_step;

    PointCloud<PointXYZRGB>::Ptr unfiltered_cloud(
            new PointCloud<PointXYZRGB>(depth_image.cols, depth_image.rows));
    PointCloud<PointXYZ>::Ptr filtered_cloud(new PointCloud<PointXYZ>);
    std::vector<cv::Point> pixel_coords;
    auto unfiltered_iter = unfiltered_cloud->begin();

    float infinity = std::numeric_limits<float>::infinity();
    for (int v = 0; v < unfiltered_cloud->height; ++v, depth_row += row_step, rgb += rgb_skip)
    {
        for (int u = 0; u < unfiltered_cloud->width; ++u, rgb += rgb_step, ++unfiltered_iter)
        {
            uint16_t depth = depth_row[u];

            // Assume depth = 0 is the standard was to note invalid
            if (depth == 0)
            {
                unfiltered_iter->x = unfiltered_iter->y = unfiltered_iter->z = bad_point;
            }
            else
            {
                float x = (u - center_x) * depth * constant_x;
                float y = (v - center_y) * depth * constant_y;
                float z = depth * unit_scaling;
                // Add to unfiltered cloud
                unfiltered_iter->x = x;
                unfiltered_iter->y = y;
                unfiltered_iter->z = z;
                unfiltered_iter->r = rgb[0];
                unfiltered_iter->g = rgb[1];
                unfiltered_iter->b = rgb[2];

                cv::Point2i pixel = cv::Point2i(u, v);
                Eigen::Array<float, 3, 1> point(x, y, z);
                if (mask.at<bool>(pixel) &&
                    point.min(upper_bounding_box).isApprox(point) &&
                    point.max(lower_bounding_box).isApprox(point))
                {
                    filtered_cloud->push_back(PointXYZ(x, y, z));
                    pixel_coords.push_back(pixel);
                }
                else if (mask.at<bool>(pixel))
                {
                    cout << "Point ignored because it was outside the boundaries." << endl;
                }
            }
        }
    }
    assert(unfiltered_iter == unfiltered_cloud->end());
    return { unfiltered_cloud, filtered_cloud, pixel_coords };
}

CDCPD::Output CDCPD::operator()(
        const cv::Mat& rgb,
        const cv::Mat& depth,
        const cv::Mat& mask,
        const PointCloud<PointXYZ>::Ptr template_cloud,
        const MatrixXi& template_edges)
{
    assert(rgb.type() == CV_8UC3);
    assert(depth.type() == CV_16U);
    assert(mask.type() == CV_8U);
    assert(P_matrix.type() == CV_64F);
    assert(P_matrix.rows == 3 && P_matrix.cols == 4);
    assert(rgb.rows == depth.rows && rgb.cols == depth.cols);

    // We'd like an Eigen version of the P matrix
    Eigen::MatrixXf P_eigen(3, 4);
    cv::cv2eigen(P_matrix, P_eigen);
    // cout << "test1" << endl;

    // For testing purposes, compute the whole cloud, though it's not really necessary TODO
    Eigen::Vector3f bounding_box_extend = Vector3f(0.1, 0.1, 0.1);
    auto [entire_cloud, cloud, pixel_coords]
        = point_clouds_from_images(
                depth,
                rgb,
                mask,
                P_eigen,
                last_lower_bounding_box - bounding_box_extend,
                last_upper_bounding_box + bounding_box_extend);
    cout << "Points in filtered: " << cloud->width << endl;
    // cout << "test2" << endl;

    cv::Mat points_mat;
    // TODO rm intrinsics
    cv::Mat intrinsics;
    // cout << "test3" << endl;
    Eigen::Matrix3f intr = P_eigen.leftCols(3);
    cv::eigen2cv(intr, intrinsics);
    // cout << "test4" << endl;

    /// VoxelGrid filter
    PointCloud<PointXYZ>::Ptr cloud_fully_filtered(new PointCloud<PointXYZ>);
    pcl::VoxelGrid<PointXYZ> sor;
    cout << "Points in cloud before leaf: " << cloud->width << endl;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.02f, 0.02f, 0.02f);
    sor.filter(*cloud_fully_filtered);
    cout << "Points in fully filtered: " << cloud_fully_filtered->width << endl;
    // cout << "test7" << endl;

    const MatrixXf& X = cloud_fully_filtered->getMatrixXfMap().topRows(3);
    const MatrixXf& Y = template_cloud->getMatrixXfMap().topRows(3);

    // TODO be able to disable?
    // TODO the combined mask doesn't account for the PCL filter, does that matter?
    // TODO less trash way?
    Eigen::Matrix3f intrinsics_eigen = Eigen::Matrix3f::Zero();
    intrinsics_eigen(0, 0) = intrinsics.at<float>(0, 0);
    intrinsics_eigen(0, 2) = intrinsics.at<float>(0, 2);
    intrinsics_eigen(1, 1) = intrinsics.at<float>(1, 1);
    intrinsics_eigen(1, 2) = intrinsics.at<float>(1, 2);
    intrinsics_eigen(2, 2) = intrinsics.at<float>(2, 2);
    // cout << "test8" << endl;
    // TODO add back in
    Eigen::VectorXf Y_emit_prior = visibility_prior(Y, intrinsics_eigen, depth, mask);
    // cout << "test9" << endl;
    /// CPD step

    // TODO maybe use a PCL point cloud for the template
    // CPD (TODO make into a function)

    Eigen::IOFormat np_fmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "");

    auto to_file = [&np_fmt](const std::string& fname, const MatrixXf& mat) {
        std::ofstream(fname) << mat.format(np_fmt);
    };

    // TODO TESTING remove this
    to_file("/home/steven/catkin/cpp_X.txt", X);
    to_file("/home/steven/catkin/cpp_Y.txt", Y);
    
    // const MatrixXf X = 
    // const MatrixXf Y = 

    MatrixXf G = gaussian_kernel(Y, beta);
    // to_file("cpp_G.txt", G);
    MatrixXf TY = Y;
    double sigma2 = initial_sigma2(X, TY) * initial_sigma_scale;

    int iterations = 0;
    double error = tolerance + 1; // loop runs the first time

    std::vector<PointCloud<PointXYZ>::Ptr> cpd_iters;
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

        P = (-P / (2 * sigma2)).array().exp().matrix();
        // TODO prior
        // TODO add back in
        // cout << "P before" << endl;
        // cout << P << endl;
        // cout << "Y_emit_prior" << endl;
        // cout << Y_emit_prior << endl;
        P.array().colwise() *= Y_emit_prior.array();
        // cout << "P after" << endl;
        // cout << P << endl;
        // if self.params.Y_emit_prior is not None:
        //      P *= self.params.Y_emit_prior[:, np.newaxis]

        MatrixXf den = P.colwise().sum().replicate(M, 1);
        // TODO ignored den[den == 0] = np.finfo(float).eps because seriously
        den.array() += c;
        // to_file(std::string("cpp_den_") + std::to_string(iterations) + ".txt", den);

        P = P.cwiseQuotient(den);
        // to_file(std::string("cpp_P_") + std::to_string(iterations) + ".txt", P);

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
        // to_file("cpp_m_lle.txt", m_lle);
        // to_file(std::string("cpp_A_") + std::to_string(iterations) + ".txt", A);
        // to_file(std::string("cpp_B_") + std::to_string(iterations) + ".txt", B);
        // to_file(std::string("cpp_W_") + std::to_string(iterations) + ".txt", W);

        TY = Y + (G * W).transpose();
        cpd_iters.push_back(mat_to_cloud(TY));
        // to_file(std::string("cpp_TY_") + std::to_string(iterations) + ".txt", TY);

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

    // Next step: optimization.

    // TODO is really 1.0?
    to_file("/home/steven/catkin/cpp_TY.txt", TY);
    // cout << original_template << endl;
    Optimizer opt(original_template, 1.0);

    // TODO add back in
    Matrix3Xf Y_opt = opt(TY, template_edges);
    to_file("/home/steven/catkin/cpp_Y_opt.txt", Y_opt);

    PointCloud<PointXYZ>::Ptr cpd_out = mat_to_cloud(Y_opt);

    // Set the min and max for the box filter for next time
    last_lower_bounding_box = Y_opt.rowwise().minCoeff();
    last_upper_bounding_box = Y_opt.rowwise().maxCoeff();

    return CDCPD::Output {
        entire_cloud, // TODO get full cloud?
        cloud, 
        cloud_fully_filtered,
        template_cloud,
        cpd_iters,
        cpd_out
    };
}
