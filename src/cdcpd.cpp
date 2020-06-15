#include <algorithm>
#include <cmath>
#include <cassert>
#include <chrono>
#include <string>

#include <Eigen/Dense>
#include <opencv2/imgcodecs.hpp> // TODO remove after not writing images
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/core/core.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <fgt.hpp>

#include "cdcpd/optimizer.h"
#include "cdcpd/cdcpd.h"
#include "cdcpd/past_template_matcher.h"

// TODO rm
#include <iostream>
#include <fstream>
// end TODO

using std::vector;
using std::cout;
using std::endl;
using cv::Mat;
using cv::Vec3b;
using Eigen::ArrayXf;
using Eigen::ArrayXd;
using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::Matrix3Xf;
using Eigen::Matrix3f;
using Eigen::MatrixXi;
using Eigen::Matrix2Xi;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::VectorXf;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::RowVectorXf;
using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PointXYZRGB;

std::string workingDir = "/home/deformtrack/catkin_ws/src/cdcpd_test/result";

// void test_nearest_line() {
//     Matrix3Xf Y(3, 4);
//     Y(0, 0) = 0.0f; Y(0, 1) = 0.0f; Y(0, 2) = 0.0f; Y(0, 3) = 0.0f;
//     Y(1, 0) = 1.0f; Y(1, 1) = 2.0f; Y(1, 2) = 3.0f; Y(1, 3) = 5.0f;
//     Y(2, 0) = 0.0f; Y(2, 1) = 0.0f; Y(2, 2) = 0.0f; Y(2, 3) = 0.0f;
//     Matrix2Xi E(2, 3);
//     E(0, 0) = 0; E(0, 1) = 1; E(0, 2) = 2;
//     E(1, 0) = 1; E(1, 1) = 2; E(1, 2) = 3;
//     auto [startPts, endPts] = nearest_points_line_segments(Y, E);
//     cout << "startPts" << endl;
//     cout << startPts << endl << endl;
//     cout << "endPts" << endl;
//     cout << endPts << endl << endl;

//     exit(1);
// }

double abs_derivative(double x) {
    double eps = 0.000000001;
    if (x < eps && x > -eps) {
        return 0.0;
    }
    else if (x > eps) {
        return 1.0;
    }
    else if (x < -eps) {
        return -1.0;
    }
}

double calculate_lle_reg(const MatrixXf& L, const Matrix3Xf pts_matrix) {
    double reg = 0;
    for (int ind = 0; ind < L.rows(); ++ind)
    {
        Matrix3Xf lle_pt(3,1);
        lle_pt(0,0) = 0; lle_pt(1,0) = 0; lle_pt(2,0) = 0;
        for (int nb_ind = 0; nb_ind < L.cols(); ++nb_ind)
        {
            lle_pt = lle_pt + L(ind, nb_ind) * pts_matrix.col(nb_ind);
        }
        reg += (lle_pt - pts_matrix.col(ind)).squaredNorm();
    }
    return reg;
}

double calculate_prob_reg(const Matrix3Xf& X, const Matrix3Xf& TY, const MatrixXf& G, const MatrixXf& W, const double sigma2, const VectorXf Y_emit_prior) {
    int M = TY.cols();
    int N = X.cols();
    int D = X.rows();
    MatrixXf P(M, N); // end = std::chrono::system_clock::now(); cout << "526: " << (end-start).count() << endl;
    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < N; ++j) {
            P(i, j) = (X.col(j) - TY.col(i)).squaredNorm();
        }
    }

    float c = std::pow(2 * M_PI * sigma2, static_cast<double>(D) / 2);
    float w = 0.1;
    c *= w / (1 - w);
    c *= static_cast<double>(M) / N;

    P = (-P / (2 * sigma2)).array().exp().matrix();
    P.array().colwise() *= Y_emit_prior.array();

    RowVectorXf den = P.colwise().sum();
    den.array() += c;

    P = P.array().rowwise() / den.array();

    double reg = 0.0;
    for (int i = 0; i < M; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            reg += P(i,j)*((X.col(j) - (TY.col(i) + (G*W).transpose().col(i))).squaredNorm());
        }
    }
    reg = reg/(2*sigma2);
    reg = reg + P.sum()*3.0*log(sigma2)/2.0;
    return reg;
}

double calculate_prob_reg(const Matrix3Xf& X, const Matrix3Xf& TY, const MatrixXf& G, const MatrixXf& W, const double sigma2, const VectorXf Y_emit_prior, const MatrixXf& P) {
    int M = TY.cols();
    int N = X.cols();
    int D = X.rows();

    double reg = 0.0;
    Matrix3Xf Y = TY + (G*W).transpose();
    for (int i = 0; i < M; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            reg += P(i,j)*((X.col(j) - Y.col(i))).squaredNorm();
        }
    }
    reg = reg/(2*sigma2);
    reg = reg + P.sum()*3.0*log(sigma2)/2.0;
    return reg;
}

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
    // X: (3, N) matrix, X^t in Algorithm 1
    // Y: (3, M) matrix, Y^(t-1) in Algorithm 1
    // Implement Line 2 of Algorithm 1
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
    // Y: (3, M) matrix, corresponding to Y^(t-1) in Eq. 13.5 (Y^t in VI.A)
    // beta: beta in Eq. 13.5 (between 13 and 14)
    MatrixXf diff(Y.cols(), Y.cols());
    diff.setZero();
    for (int i = 0; i < Y.cols(); ++i)
    {
        for (int j = 0; j < Y.cols(); ++j)
        {
            diff(i, j) = (Y.col(i) - Y.col(j)).squaredNorm();
        }
    }
    // ???: beta should be beta^2
    MatrixXf kernel = (-diff / (2 * beta * beta)).array().exp();
    return kernel;
}

MatrixXf barycenter_kneighbors_graph(const pcl::KdTreeFLANN<PointXYZ>& kdtree,
                                     int lle_neighbors,
                                     double reg)
{
    // calculate L in Eq. (15) and Eq. (16)
    // ENHANCE: use tapkee lib to accelarate
    // kdtree: kdtree from Y^0
    // lle_neighbors: parameter for lle calculation
    // reg: regularization term (necessary when lle_neighbor > dimension)
    PointCloud<PointXYZ>::ConstPtr cloud = kdtree.getInputCloud();
    assert(cloud->height == 1);
    // adjacencies: save index of adjacent points
    MatrixXi adjacencies = MatrixXi(cloud->width, lle_neighbors);
    // B: save weight W_ij
    MatrixXf B = MatrixXf::Zero(cloud->width, lle_neighbors);
    MatrixXf v = VectorXf::Ones(lle_neighbors);
    // algorithm: see https://cs.nyu.edu/~roweis/lle/algorithm.html
    for (size_t i = 0; i < cloud->width; ++i)
    {
        vector<int> neighbor_inds(lle_neighbors + 1);
        vector<float> neighbor_dists(lle_neighbors + 1);
        kdtree.nearestKSearch(i, lle_neighbors + 1, neighbor_inds, neighbor_dists);
        // C: transpose of Z in Eq [d] and [e]
        MatrixXf C(lle_neighbors, 3);
        for (size_t j = 1; j < neighbor_inds.size(); ++j)
        {
            C.row(j - 1) = cloud->points[neighbor_inds[j]].getVector3fMap()
                - cloud->points[i].getVector3fMap();
            adjacencies(i, j - 1) = neighbor_inds[j];
        }
        // G: C in Eq [f]
        MatrixXf G = C * C.transpose();
        // ???: why += R for G
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
    for (ssize_t i = 0; i < graph.rows(); ++i)
    {
        for (ssize_t j = 0; j < lle_neighbors; ++j)
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
    // calculate H in Eq. (18)
    // template_cloud: Y^0 in Eq. (15) and (16)
    // lle_neighbors: parameter for lle calculation
    // reg: regularization term (seems unnecessary)
    pcl::KdTreeFLANN<PointXYZ> kdtree;
    kdtree.setInputCloud (template_cloud);
    // W: (M, M) matrix, corresponding to L in Eq. (15) and (16)
    MatrixXf W = barycenter_kneighbors_graph(kdtree, lle_neighbors, reg);
    // M: (M, M) matrix, corresponding to H in Eq. (18)
    MatrixXf M = (W.transpose() * W) - W.transpose() - W;
    M.diagonal().array() += 1;
    return M;
}

void Qconstructor(const Matrix2Xi& E, vector<MatrixXf>& Q, int M) {
    for (int i = 0; i < E.cols(); ++i)
    {
        MatrixXf Qi = MatrixXf::Zero(M, M);
        Qi(E(0, i), E(0, i)) = 1.0f;
        Qi(E(1, i), E(1, i)) = 1.0f;
        Qi(E(0, i), E(1, i)) = -1.0f;
        Qi(E(1, i), E(0, i)) = -1.0f;
        Q.push_back(Qi);
    }
}

CDCPD::CDCPD(
            PointCloud<PointXYZ>::ConstPtr template_cloud,
            const Matrix2Xi& _template_edges,
            const Mat& _P_matrix,
            bool _use_recovery
            ):
    template_matcher(1500), // TODO make configurable?
    original_template(template_cloud->getMatrixXfMap().topRows(3)),
    template_edges(_template_edges),
    P_matrix(_P_matrix),
    last_lower_bounding_box(-6.0, -6.0, -6.0), // TODO make configurable?
    last_upper_bounding_box(6.0, 6.0, 6.0), // TODO make configurable?
    lle_neighbors(8), // TODO make configurable?
    // ENHANCE & ???: 1e-3 seems to be unnecessary
    m_lle(locally_linear_embedding(template_cloud, lle_neighbors, 1e-3)), // TODO make configurable?
    tolerance(1e-4), // TODO make configurable?
    alpha(50), // TODO make configurable?
    beta(1.0), // TODO make configurable?
    w(0.1), // TODO make configurable?
    initial_sigma_scale(1.0 / 8), // TODO make configurable?
    start_lambda(1.0), // TODO make configurable?
    annealing_factor(0.6), // TODO make configurable?
    k(10.0),
    max_iterations(100), // TODO make configurable?
    kvis(1e1),
    use_recovery(_use_recovery)
{
    pcl::KdTreeFLANN<PointXYZ> kdtree;
    kdtree.setInputCloud (template_cloud);
    // W: (M, M) matrix, corresponding to L in Eq. (15) and (16)
    MatrixXf L = barycenter_kneighbors_graph(kdtree, lle_neighbors, 0.001);

    Qconstructor(template_edges, Q, original_template.cols());
}

/*
 * Return a non-normalized probability that each of the tracked vertices produced any detected point.
 * Implement Eq. (7) in the paper
 */
Eigen::VectorXf CDCPD::visibility_prior(const Matrix3Xf vertices,
                                 const cv::Mat& depth,
                                 const cv::Mat& mask,
                                 float k)
{
    // vertices: (3, M) matrix Y^t (Y in IV.A) in the paper
    // intrinsics: (3, 3) intrinsic matrix of the camera
    // depth_image: CV_16U depth image
    // mask: CV_8U mask for segmentation
    // k: k_vis in the paper

#ifdef DEBUG
    Eigen::IOFormat np_fmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "");
    auto to_file = [&np_fmt](const std::string& fname, const MatrixXf& mat) {
        std::ofstream(fname) << mat.format(np_fmt);
    };

    // save matrices
    to_file(workingDir + "/cpp_verts.txt", vertices);
    to_file(workingDir + "/cpp_intrinsics.txt", intrinsics);
#endif

    // Project the vertices to get their corresponding pixel coordinate
    // ENHANCE: replace P_eigen.leftCols(3) with intrinsics
    Eigen::MatrixXf P_eigen(3, 4);
    cv::cv2eigen(P_matrix, P_eigen);
    Eigen::Matrix3Xf image_space_vertices = P_eigen.leftCols(3) * vertices;

    // Homogeneous coords: divide by third row
    // the for-loop is unnecessary
    image_space_vertices.row(0).array() /= image_space_vertices.row(2).array();
    image_space_vertices.row(1).array() /= image_space_vertices.row(2).array();
    for (int i = 0; i < image_space_vertices.cols(); ++i)
    {
        float x = image_space_vertices(0, i);
        float y = image_space_vertices(1, i);
        image_space_vertices(0, i) = std::min(std::max(x, 0.0f), static_cast<float>(depth.cols));
        image_space_vertices(1, i) = std::min(std::max(y, 0.0f), static_cast<float>(depth.rows));
    }
#ifdef DEBUG
    to_file(workingDir + "/cpp_projected.txt", image_space_vertices);
#endif

    // Get image coordinates
    Eigen::Matrix2Xi coords = image_space_vertices.topRows(2).cast<int>();

    for (int i = 0; i < coords.cols(); ++i)
    {
        coords(0, i) = std::min(std::max(coords(0, i), 0), depth.cols - 1);
        coords(1, i) = std::min(std::max(coords(1, i), 1), depth.rows - 1);
    }
#ifdef DEBUG
    to_file(workingDir + "/cpp_coords.txt", coords.cast<float>());
#endif

    // Find difference between point depth and image depth
    // depth_diffs: (1, M) vector
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
    // ENHANCE: repeating max
    depth_diffs = depth_diffs.array().max(0);
#ifdef DEBUG
    to_file(workingDir + "/cpp_depth_diffs.txt", depth_diffs);
#endif

    Eigen::VectorXf depth_factor = depth_diffs.array().max(0.0);
    cv::Mat dist_img(depth.rows, depth.cols, CV_32F); // TODO haven't really tested this but seems right
    int maskSize = 5;
    cv::distanceTransform(~mask, dist_img, cv::noArray(), cv::DIST_L2, maskSize);
    // ???: why cv::normalize is needed
    cv::normalize(dist_img, dist_img, 0.0, 1.0, cv::NORM_MINMAX);
    // ENHANCE: rm unused dist_copr
    cv::Mat dist_copy = dist_img.clone();
    imwrite(workingDir + "/dist_img.png", dist_copy * 255.0);

    Eigen::VectorXf dist_to_mask(vertices.cols());
    for (int i = 0; i < vertices.cols(); ++i)
    {
        dist_to_mask(i) = dist_img.at<float>(coords(1, i), coords(0, i));
    }
    VectorXf score = (dist_to_mask.array() * depth_factor.array()).matrix();
    VectorXf prob = (-k * score).array().exp().matrix();
    // ???: unnormalized prob
#ifdef DEBUG
    to_file(workingDir + "/cpp_prob.txt", prob);
#endif
    return prob;
}

/*
 * Used for failure recovery. Very similar to the visibility_prior function, but with a different K and
 * image_depth - vertex_depth, not vice-versa. There's also no normalization.
 * Calculate Eq. (22) in the paper
 */
float smooth_free_space_cost(const Matrix3Xf vertices,
                                 const Eigen::Matrix3f& intrinsics,
                                 const cv::Mat& depth,
                                 const cv::Mat& mask,
                                 float k=1e2)
{
    // vertices: Y^t in Eq. (22)
    // intrinsics: intrinsic matrix of the camera
    // depth: depth image
    // mask: mask image
    // k: constant defined in Eq. (22)

    Eigen::IOFormat np_fmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "");
#ifdef DEBUG
    auto to_file = [&np_fmt](const std::string& fname, const MatrixXf& mat) {
        std::ofstream(fname) << mat.format(np_fmt);
    };
#endif

    // Project the vertices to get their corresponding pixel coordinate
    Eigen::Matrix3Xf image_space_vertices = intrinsics * vertices;
    // Homogeneous coords: divide by third row
    image_space_vertices.row(0).array() /= image_space_vertices.row(2).array();
    image_space_vertices.row(1).array() /= image_space_vertices.row(2).array();
    for (int i = 0; i < image_space_vertices.cols(); ++i)
    {
        float x = image_space_vertices(0, i);
        float y = image_space_vertices(1, i);
        // ENHANCE: no need to create coords again
        image_space_vertices(0, i) = std::min(std::max(x, 0.0f), static_cast<float>(depth.cols));
        image_space_vertices(1, i) = std::min(std::max(y, 0.0f), static_cast<float>(depth.rows));
    }
#ifdef DEBUG
    to_file(workingDir + "/cpp_projected.txt", image_space_vertices);
#endif

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

    // Find difference between point depth and image depth
    Eigen::VectorXf depth_diffs = Eigen::VectorXf::Zero(vertices.cols());
    for (int i = 0; i < vertices.cols(); ++i)
    {
        // cout << "depth at " << coords(1, i) << " " << coords(0, i) << endl;
        uint16_t raw_depth = depth.at<uint16_t>(coords(1, i), coords(0, i));
        if (raw_depth != 0)
        {
            depth_diffs(i) = static_cast<float>(raw_depth) / 1000.0 - vertices(2, i); // unit: m
        }
        else
        {
            depth_diffs(i) = std::numeric_limits<float>::quiet_NaN();
        }
    }
    Eigen::VectorXf depth_factor = depth_diffs.array().max(0.0);
#ifdef DEBUG
    to_file(workingDir + "/cpp_depth_diffs.txt", depth_diffs);
#endif


    cv::Mat dist_img(depth.rows, depth.cols, CV_32F); // TODO should the other dist_img be this, too?
    int maskSize = 5;
    cv::distanceTransform(~mask, dist_img, cv::noArray(), cv::DIST_L2, maskSize);

    Eigen::VectorXf dist_to_mask(vertices.cols());
    for (int i = 0; i < vertices.cols(); ++i)
    {
        dist_to_mask(i) = dist_img.at<float>(coords(1, i), coords(0, i));
    }
    VectorXf score = (dist_to_mask.array() * depth_factor.array()).matrix();
    // NOTE: Eq. (22) lost 1 in it
    VectorXf prob = (1 - (-k * score).array().exp()).matrix();
#ifdef DEBUG
    to_file(workingDir + "/cpp_prob.txt", prob);
#endif
    float cost = prob.array().isNaN().select(0, prob.array()).sum();
    cost /= prob.array().isNaN().select(0, VectorXi::Ones(prob.size())).sum();
    return cost;
}

// TODO return both point clouds
// TODO based on the implementation here:
// https://github.com/ros-perception/image_pipeline/blob/melodic/depth_image_proc/src/nodelets/point_cloud_xyzrgb.cpp
// Note that we expect that cx, cy, fx, fy are in the appropriate places in P
std::tuple<
#ifdef ENTIRE
        PointCloud<PointXYZRGB>::Ptr,
#endif
        PointCloud<PointXYZ>::Ptr>
        //std::vector<cv::Point>>
point_clouds_from_images(const cv::Mat& depth_image,
                         const cv::Mat& rgb_image,
                         const cv::Mat& mask,
                         const Eigen::MatrixXf& P,
                         const Eigen::Vector3f lower_bounding_box_vec,
                         const Eigen::Vector3f upper_bounding_box_vec)
{
    // depth_image: CV_16U depth image
    // rgb_image: CV_8U3C rgb image
    // mask: CV_8U mask for segmentation
    // P: camera matrix of Kinect
    //  [[fx 0  px 0];
    //   [0  fy py 0];
    //   [0  0  1  0]]
    // lower_bounding_box_vec: bounding for mask
    // upper_bounding_box_vec: bounding for mask

    // assert(depth_image.cols == rgb_image.cols);
    // assert(depth_image.rows == rgb_image.rows);
    // assert(depth_image.type() == CV_16U);
    // assert(P.rows() == 3 && P.cols() == 4);
    // Assume unit_scaling is the standard Kinect 1mm
    float unit_scaling = 0.001;
    float pixel_len = 0.0002645833;
    float constant_x = 1 / (P(0, 0) * pixel_len);
    float constant_y = 1 / (P(1, 1) * pixel_len);
    float center_x = P(0, 2);
    float center_y = P(1, 2);
    float bad_point = std::numeric_limits<float>::quiet_NaN();
    Eigen::Array<float, 3, 1> lower_bounding_box = lower_bounding_box_vec.array();
    Eigen::Array<float, 3, 1> upper_bounding_box = upper_bounding_box_vec.array();

    // ENHANCE: change the way to get access to depth image and rgb image to be more readable
    // from "depth_row += row_step" to ".at<>()"
    /*
    const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth_image.data[0]);
    int row_step = depth_image.step / sizeof(uint16_t); // related with accessing data in cv::Mat
    const uint8_t* rgb = &rgb_image.data[0];
    int rgb_step = 3; // TODO check on this
    int rgb_skip = rgb_image.step - rgb_image.cols * rgb_step;
    */
#ifdef ENTIRE
    PointCloud<PointXYZRGB>::Ptr unfiltered_cloud(
            new PointCloud<PointXYZRGB>(depth_image.cols, depth_image.rows));
#endif
    PointCloud<PointXYZ>::Ptr filtered_cloud(new PointCloud<PointXYZ>);
    //std::vector<cv::Point> pixel_coords;
#ifdef ENTIRE
    auto unfiltered_iter = unfiltered_cloud->begin();
#endif

    for (int v = 0; v < depth_image.rows; ++v)
    {
#ifdef ENTIRE
        for (uint32_t u = 0; u < depth_image.cols; ++u, ++unfiltered_iter)
#else
        for (uint32_t u = 0; u < depth_image.cols; ++u)
#endif
        {
            uint16_t depth = depth_image.at<uint16_t>(v, u);

            // Assume depth = 0 is the standard was to note invalid
            if (depth != 0)
            {
                float x = (float(u) - center_x) * pixel_len * float(depth) * unit_scaling * constant_x; // * pixel_len * depth * unit_scaling * constant_x;
                float y = (float(v) - center_y) * pixel_len * float(depth) * unit_scaling * constant_y; // * pixel_len * depth * unit_scaling * constant_y;
                float z = float(depth) * unit_scaling;
                // Add to unfiltered cloud
                // ENHANCE: be more concise
#ifdef ENTIRE
                unfiltered_iter->x = x;
                unfiltered_iter->y = y;
                unfiltered_iter->z = z;
                unfiltered_iter->r = rgb_image.at<Vec3b>(v, u)[0];
                unfiltered_iter->g = rgb_image.at<Vec3b>(v, u)[1];
                unfiltered_iter->b = rgb_image.at<Vec3b>(v, u)[2];
#endif

                // cv::Point2i pixel = cv::Point2i(u, v);
                Eigen::Array<float, 3, 1> point(x, y, z);
                if (mask.at<bool>(v, u) &&
                    point.min(upper_bounding_box).isApprox(point) &&
                    point.max(lower_bounding_box).isApprox(point))
                {
                    filtered_cloud->push_back(PointXYZ(x, y, z));
                    // pixel_coords.push_back(pixel);
                }
                // else if (mask.at<bool>(pixel))
                // {
                //     cout << "Point ignored because it was outside the boundaries." << endl;
                // }
            }
#ifdef ENTIRE
            else
            {
                unfiltered_iter->x = unfiltered_iter->y = unfiltered_iter->z = bad_point;
            }
#endif
        }
    }
//    assert(unfiltered_iter == unfiltered_cloud->end());
    return {
#ifdef ENTIRE
        unfiltered_cloud,
#endif
        filtered_cloud};
        // pixel_coords };
}

Matrix3Xf CDCPD::cpd(pcl::PointCloud<pcl::PointXYZ>::ConstPtr downsampled_cloud,
                     const Matrix3Xf& Y,
                     const cv::Mat& depth,
                     const cv::Mat& mask,
                     const Matrix3f& intr)
{
    // downsampled_cloud: PointXYZ pointer to downsampled point clouds
    // Y: (3, M) matrix Y^t (Y in IV.A) in the paper
    // depth: CV_16U depth image
    // mask: CV_8U mask for segmentation label
    // intr: (3, 3) intrinsic matrix of the camera

    const Matrix3Xf& X = downsampled_cloud->getMatrixXfMap().topRows(3);
    Eigen::VectorXf Y_emit_prior = visibility_prior(Y, depth, mask, kvis);
    /// CPD step

    // G: (M, M) Guassian kernel matrix
    MatrixXf G = gaussian_kernel(original_template, beta);//Y, beta);

    // TY: Y^(t) in Algorithm 1
    Matrix3Xf TY = Y;
    double sigma2 = initial_sigma2(X, TY) * initial_sigma_scale;

    int iterations = 0;
    double error = tolerance + 1; // loop runs the first time

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

#ifdef CPDLOG
    cout << "\nCPD loop\n";
    cout << std::setw(20) << "loop" << std::setw(20) << "prob term" << std::setw(20) << "CPD term" << std::setw(20) << "LLE term" << endl;
#endif

    while (iterations <= max_iterations && error > tolerance)
    {
        double qprev = sigma2;
        // Expectation step
        int N = X.cols();
        int M = Y.cols();
        int D = Y.rows();

        // P: P in Line 5 in Algorithm 1 (mentioned after Eq. (18))
        // Calculate Eq. (9) (Line 5 in Algorithm 1)
        // NOTE: Eq. (9) misses M in the denominator
        
        MatrixXf P(M, N); // end = std::chrono::system_clock::now(); cout << "526: " << (end-start).count() << endl;
        {
            for (int i = 0; i < M; ++i) {
                for (int j = 0; j < N; ++j) {
                    P(i, j) = (X.col(j) - TY.col(i)).squaredNorm();
                }
            }

            float c = std::pow(2 * M_PI * sigma2, static_cast<double>(D) / 2);
            c *= w / (1 - w);
            c *= static_cast<double>(M) / N;

            P = (-P / (2 * sigma2)).array().exp().matrix();
            P.array().colwise() *= Y_emit_prior.array();

            RowVectorXf den = P.colwise().sum();
            den.array() += c;

            P = P.array().rowwise() / den.array();
        }  //end = std::chrono::system_clock::now(); cout << "545: " << (end-start).count() << endl;
        
        // Fast Gaussian Transformation to calculate Pt1, P1, PX
        
        // double bandwidth = std::sqrt(2.0 * sigma2);
        // double epsilon = 1e-4;
        // fgt::Matrix Y_fgt = TY.transpose().cast<double>(); //cout << "575\n";
        // fgt::Matrix X_fgt = X.transpose().cast<double>(); //cout << "576\n";
        // fgt::DirectTree fgt1(Y_fgt, bandwidth, epsilon); //cout << "577\n";
        // VectorXd kt1 = fgt1.compute(X_fgt, Y_emit_prior.cast<double>()); //cout << "578\n";// N*1 vector
        
        // float c = std::pow(2 * M_PI * sigma2, static_cast<double>(D) / 2);
        // c *= w / (1 - w);
        // c *= static_cast<double>(M) / N;
        // ArrayXd a = 1 / (kt1.array() + c); // N*1 array

        // VectorXf Pt1 = (1 - c * a).cast<float>(); // M*1 array

        // fgt::DirectTree fgt2(X_fgt, bandwidth, epsilon);  //cout << "587\n";
        // VectorXf P1 = (fgt2.compute(Y_fgt, a)).cast<float>(); //cout << "588\n";
        // P1 = P1.array()*Y_emit_prior.array();

        // MatrixXd PX_fgt(TY.rows(), TY.cols());
        // for (size_t i = 0; i < TY.rows(); ++i) {
        //     // ArrayXd Xi = X_fgt.col(i).array();
        //     ArrayXd aXarray = X_fgt.col(i).array() * a;
        //     fgt::Vector aX = fgt::Vector(aXarray);  //cout << "594\n";
        //     PX_fgt.row(i) = (fgt2.compute(Y_fgt, aX)).array() * Y_emit_prior.array().cast<double>();
        // }
        // MatrixXf PX = PX_fgt.cast<float>();
        MatrixXf PX = (P * X.transpose()).transpose();

        // // Maximization step
        VectorXf Pt1 = P.colwise().sum();// end = std::chrono::system_clock::now(); cout << "548: " << (end-start).count() << endl;
        VectorXf P1 = P.rowwise().sum();// end = std::chrono::system_clock::now(); cout << "549: " << (end-start).count() << endl;
        float Np = P1.sum(); //end = std::chrono::system_clock::now(); cout << "550: " << (end-start).count() << endl;
        
        // This is the code to use if you are not using LLE
        // NOTE: lambda means gamma here
        // ENHANCE: some terms in the equation are not changed during the loop, which can be calculated out of the loop
        // Corresponding to Eq. (18) in the paper
        float lambda = start_lambda;// * std::pow(annealing_factor, iterations + 1);
        MatrixXf p1d = P1.asDiagonal(); //end = std::chrono::system_clock::now(); cout << "557: " << (end-start).count() << endl;
        MatrixXf A = (P1.asDiagonal() * G)
            + alpha * sigma2 * MatrixXf::Identity(M, M)
            + sigma2 * lambda * (m_lle * G);// end = std::chrono::system_clock::now();cout << "560: " << (end-start).count() << endl;
        MatrixXf B = PX.transpose() - (p1d + sigma2 * lambda * m_lle) * Y.transpose(); //end = std::chrono::system_clock::now();cout << "561: " << (end-start).count() << endl;        
        for (int i = 0; i < template_edges.cols(); ++i)
        {
            MatrixXf eit = Y*Q[i]*Y.transpose();
            MatrixXf ei0 = original_template*Q[i]*original_template.transpose();
            double delta = abs_derivative(eit.trace()-ei0.trace());
            A = A + k*sigma2*delta*Q[i]*G;
            B = B - k*sigma2*delta*Q[i]*Y.transpose();
        }
        MatrixXf W = A.householderQr().solve(B); //end = std::chrono::system_clock::now(); cout << "562: " << (end-start).count() << endl;

        TY = Y + (G * W).transpose();// end = std::chrono::system_clock::now(); cout << "564: " << (end-start).count() << endl;

        // Corresponding to Eq. (19) in the paper
        VectorXf xPxtemp = (X.array() * X.array()).colwise().sum();
//        float xPx = (Pt1 * xPxtemp.transpose())(0,0);
        double xPx = Pt1.dot(xPxtemp);// end = std::chrono::system_clock::now();cout << "569: " << (end-start).count() << endl;
//        assert(xPxMat.rows() == 1 && xPxMat.cols() == 1);
//        double xPx = xPxMat.sum();
        VectorXf yPytemp = (TY.array() * TY.array()).colwise().sum();
//        MatrixXf yPyMat = P1.transpose() * yPytemp.transpose();
//        assert(yPyMat.rows() == 1 && yPyMat.cols() == 1);
        double yPy = P1.dot(yPytemp); //end = std::chrono::system_clock::now();cout << "575: " << (end-start).count() << endl;
        double trPXY = (TY.array() * PX.array()).sum(); //end = std::chrono::system_clock::now();cout << "576: " << (end-start).count() << endl;
        sigma2 = (xPx - 2 * trPXY + yPy) / (Np * D); //end = std::chrono::system_clock::now();cout << "577: " << (end-start).count() << endl;

        if (sigma2 <= 0)
        {
            sigma2 = tolerance / 10;
        }

#ifdef CPDLOG
        double prob_reg = calculate_prob_reg(X, TY, G, W, sigma2, Y_emit_prior, P);
        double lle_reg = start_lambda / 2 * ((TY*m_lle*TY.transpose()).trace() + 2*(W.transpose()*G*m_lle*TY).trace() + (W.transpose()*G*m_lle*G*W).trace());
        double cpd_reg = alpha * (W.transpose()*G*W).trace()/2;
        cout << std::setw(20) << iterations;
        cout << std::setw(20) << prob_reg;
        cout << std::setw(20) << cpd_reg;
        cout << std::setw(20) << lle_reg << endl;
#endif


        error = std::abs(sigma2 - qprev);
        iterations++;
    }
    return TY;
}

CDCPD::Output CDCPD::operator()(
        const cv::Mat& rgb,
        const cv::Mat& depth,
        const cv::Mat& mask,
        const PointCloud<PointXYZ>::Ptr template_cloud,
        const bool self_intersection,
        const bool interation_constrain,
        const std::vector<CDCPD::FixedPoint>& fixed_points
        )
{
    // rgb: CV_8U3C rgb image
    // depth: CV_16U depth image
    // mask: CV_8U mask for segmentation
    // template_cloud: point clouds corresponding to Y^t (Y in IV.A) in the paper
    // template_edges: (2, K) matrix corresponding to E in the paper
    // fixed_points: fixed points during the tracking

    assert(rgb.type() == CV_8UC3);
    assert(depth.type() == CV_16U);
    assert(mask.type() == CV_8U);
    assert(P_matrix.type() == CV_64F);
    assert(P_matrix.rows == 3 && P_matrix.cols == 4);
    assert(rgb.rows == depth.rows && rgb.cols == depth.cols);

    size_t recovery_knn_k = 12; // TODO configure this?
    float recovery_cost_threshold = 0.5; // TODO configure this?

#ifdef DEBUG
    Eigen::IOFormat np_fmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "");

    // Useful utility for outputting an Eigen matrix to a file
    auto to_file = [&np_fmt](const std::string& fname, const MatrixXf& mat) {
        std::ofstream(fname, std::ofstream::app) << mat.format(np_fmt) << "\n\n";
    };
#endif

    // We'd like an Eigen version of the P matrix
    Eigen::MatrixXf P_eigen(3, 4);
    cv::cv2eigen(P_matrix, P_eigen);

    // TODO: For testing purposes, compute the whole cloud, though it's not really necessary
    // ENHANCE: debug macro to remove entire_cloud
    // ENHANCE: pixel_coords is not used
    // entire_cloud: pointer to the entire point cloud
    // cloud: pointer to the point clouds selected
    // pixel_coords: pixel coordinate corresponds to the point in the cloud
    Eigen::Vector3f bounding_box_extend = Vector3f(0.2, 0.2, 0.2);
    auto [
#ifdef ENTIRE
            entire_cloud,
#endif
                    cloud]//, pixel_coords]
        = point_clouds_from_images(
                depth,
                rgb,
                mask,
                P_eigen,
                last_lower_bounding_box - bounding_box_extend,
                last_upper_bounding_box + bounding_box_extend);
    cout << "Points in filtered: " << cloud->width << endl;

    // intr: intrinsics of the camera
    Eigen::Matrix3f intr = P_eigen.leftCols(3);

    /// VoxelGrid filter downsampling
    PointCloud<PointXYZ>::Ptr cloud_downsampled(new PointCloud<PointXYZ>);

    pcl::VoxelGrid<PointXYZ> sor;
    cout << "Points in cloud before leaf: " << cloud->width << endl;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.02f, 0.02f, 0.02f);
    sor.filter(*cloud_downsampled);
    cout << "Points in fully filtered: " << cloud_downsampled->width << endl;
    const Matrix3Xf& X = cloud_downsampled->getMatrixXfMap().topRows(3);
    const Matrix3Xf& Y = template_cloud->getMatrixXfMap().topRows(3);
#ifdef ENTIRE
    const Matrix3Xf& entire = entire_cloud->getMatrixXfMap().topRows(3);
#endif
    Eigen::Matrix3Xf TY = cpd(cloud_downsampled, Y, depth, mask, intr);
#ifdef DEBUG
    to_file(workingDir + "/cpp_entire_cloud.txt", entire);
    to_file(workingDir + "/cpp_downsample.txt", X);
    to_file(workingDir + "/cpp_TY-1.txt", template_cloud->getMatrixXfMap().topRows(3));
    to_file(workingDir + "/cpp_TY.txt", TY);
#endif

    // Next step: optimization.
    // ???: most likely not 1.0
    Optimizer opt(original_template, Y, 1.00);

    Matrix3Xf Y_opt = opt(TY, template_edges, fixed_points, self_intersection, interation_constrain); // TODO perhaps optionally disable optimization?
#ifdef DEBUG
    to_file(workingDir + "/cpp_Y_opt.txt", Y_opt);
#endif

    // If we're doing tracking recovery, do that now
    if (use_recovery)
    {
        float cost = smooth_free_space_cost(Y_opt, intr, depth, mask);
        cout << "cost" << endl;
        cout << cost << endl;

        if (cost > recovery_cost_threshold && template_matcher.size() > recovery_knn_k)
        {
            float best_cost = cost;
            Matrix3Xf final_tracking_result = Y_opt;

            std::vector<Matrix3Xf> matched_templates = template_matcher.query_template(cloud_downsampled, recovery_knn_k);

            // TODO there's potentially a lot of copying going on here. We should be able to avoid that.
            for (const Matrix3Xf& templ : matched_templates)
            {
                Matrix3Xf proposed_recovery = cpd(cloud_downsampled, templ, depth, mask, intr);
                // TODO if we end up being able to disable optimization, we should not call this
                proposed_recovery = opt(proposed_recovery, template_edges, fixed_points);
                float proposal_cost = smooth_free_space_cost(proposed_recovery, intr, depth, mask);
                if (proposal_cost < best_cost)
                {
                    final_tracking_result = proposed_recovery;
                    best_cost = proposal_cost;
                }
            }
            Y_opt = final_tracking_result;
        }
        else
        {
            template_matcher.add_template(cloud_downsampled, Y_opt);
        }
    }
    cout << "matcher size: " << template_matcher.size() << endl;

    // Set the min and max for the box filter for next time
    last_lower_bounding_box = Y_opt.rowwise().minCoeff();
    last_upper_bounding_box = Y_opt.rowwise().maxCoeff();

    PointCloud<PointXYZ>::Ptr cpd_out = mat_to_cloud(Y_opt);

    return CDCPD::Output {
#ifdef ENTIRE
        entire_cloud,
#endif
        cloud,
        cloud_downsampled,
        template_cloud,
        cpd_out
    };
}
