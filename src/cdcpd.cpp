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

#include <smmap/ros_communication_helpers.h>
#include <sdf_tools/collision_map.hpp>
#include <sdf_tools/sdf.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <fgt.hpp>

#include "cdcpd/obs_util.h"
#include "cdcpd/cdcpd.h"
#include "cdcpd/optimizer.h"
#include "cdcpd/past_template_matcher.h"
#include "depth_traits.h"

// TODO rm
#include <iostream>
#include <fstream>
#include <random>
// end TODO

using cv::Mat;
using cv::Vec3b;
using Eigen::ArrayXf;
using Eigen::ArrayXd;
using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::Matrix3Xf;
using Eigen::Matrix3Xd;
using Eigen::Matrix3f;
using Eigen::MatrixXi;
using Eigen::Matrix2Xi;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::VectorXf;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::RowVectorXf;
using Eigen::Isometry3d;
using smmap::AllGrippersSinglePose;
using smmap::AllGrippersSinglePoseDelta;
using std::cout;
using std::endl;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

#ifdef SIMULATION
std::string workingDir = "/home/deformtrack/catkin_ws/src/cdcpd_test_blender/log";
#else
std::string workingDir = "/home/deformtrack/catkin_ws/src/cdcpd_test/log";
#endif

static double abs_derivative(double x)
{
    return x;
    // double eps = 0.000000001;
    // if (x < eps && x > -eps) {
    //     return 0.0;
    // }
    // else if (x > eps) {
    //     return 1.0;
    // }
    // else if (x < -eps) {
    //     return -1.0;
    // }
}

static double calculate_lle_reg(const MatrixXf& L,
                                const Matrix3Xf pts_matrix)
{
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

static double calculate_prob_reg(const Matrix3Xf& X,
                                 const Matrix3Xf& TY,
                                 const MatrixXf& G,
                                 const MatrixXf& W,
                                 const double sigma2,
                                 const VectorXf& Y_emit_prior)
{
    int M = TY.cols();
    int N = X.cols();
    int D = X.rows();
    MatrixXf P(M, N); // end = std::chrono::system_clock::now(); std::cout << "526: " << (end-start).count() << std::endl;
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

static double calculate_prob_reg(const Matrix3Xf& X,
                                 const Matrix3Xf& TY,
                                 const MatrixXf& G,
                                 const MatrixXf& W,
                                 const double sigma2,
                                 const VectorXf Y_emit_prior,
                                 const MatrixXf& P)
{
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

static PointCloud::Ptr mat_to_cloud(const Eigen::Matrix3Xf& mat)
{
    PointCloud::Ptr cloud(new PointCloud);
    cloud->points.reserve(mat.cols());
    for (ssize_t i = 0; i < mat.cols(); ++i)
    {
        cloud->push_back(pcl::PointXYZ(mat(0, i), mat(1, i), mat(2, i)));
    }
    return cloud;
}

static double initial_sigma2(const MatrixXf& X, const MatrixXf& Y)
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

static MatrixXf gaussian_kernel(const MatrixXf& Y, double beta)
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

static std::vector<smmap::CollisionData> fake_collision_check(const AllGrippersSinglePose& gripper_poses) {
    std::vector<smmap::CollisionData> nothing;
    return nothing;
}

MatrixXf barycenter_kneighbors_graph(const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                                            int lle_neighbors,
                                            double reg)
{
    // calculate L in Eq. (15) and Eq. (16)
    // ENHANCE: use tapkee lib to accelarate
    // kdtree: kdtree from Y^0
    // lle_neighbors: parameter for lle calculation
    // reg: regularization term (necessary when lle_neighbor > dimension)
    PointCloud::ConstPtr cloud = kdtree.getInputCloud();
    assert(cloud->height == 1);
    // adjacencies: save index of adjacent points
    MatrixXi adjacencies = MatrixXi(cloud->width, lle_neighbors);
    // B: save weight W_ij
    MatrixXf B = MatrixXf::Zero(cloud->width, lle_neighbors);
    MatrixXf v = VectorXf::Ones(lle_neighbors);
    // algorithm: see https://cs.nyu.edu/~roweis/lle/algorithm.html
    for (size_t i = 0; i < cloud->width; ++i)
    {
        std::vector<int> neighbor_inds(lle_neighbors + 1);
        std::vector<float> neighbor_dists(lle_neighbors + 1);
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

MatrixXf locally_linear_embedding(PointCloud::ConstPtr template_cloud,
                                         int lle_neighbors,
                                         double reg)
{
    // calculate H in Eq. (18)
    // template_cloud: Y^0 in Eq. (15) and (16)
    // lle_neighbors: parameter for lle calculation
    // reg: regularization term (seems unnecessary)
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (template_cloud);
    // W: (M, M) matrix, corresponding to L in Eq. (15) and (16)
    MatrixXf W = barycenter_kneighbors_graph(kdtree, lle_neighbors, reg);
    // M: (M, M) matrix, corresponding to H in Eq. (18)
    MatrixXf M = (W.transpose() * W) - W.transpose() - W;
    M.diagonal().array() += 1;
    return M;
}

// static void Qconstructor(const Matrix2Xi& E, std::vector<MatrixXf>& Q, int M)
// {
//     for (int i = 0; i < E.cols(); ++i)
//     {
//         MatrixXf Qi = MatrixXf::Zero(M, M);
//         Qi(E(0, i), E(0, i)) = 1.0f;
//         Qi(E(1, i), E(1, i)) = 1.0f;
//         Qi(E(0, i), E(1, i)) = -1.0f;
//         Qi(E(1, i), E(0, i)) = -1.0f;
//         Q.push_back(Qi);
//     }
// }

CDCPD::CDCPD(PointCloud::ConstPtr template_cloud,
             const Matrix2Xi& _template_edges,
             #ifdef PREDICT
             std::shared_ptr<ros::NodeHandle> nh,
             const double translation_dir_deformability,
             const double translation_dis_deformability,
             const double rotation_deformability,
             const Eigen::MatrixXi& grippers,
             #endif
             #ifdef SHAPE_COMP
			 const obsParam& _obs_param,
             #endif
             const bool _use_recovery,
             const double _alpha,
             const double _beta,
             const double _lambda,
             const double _k) :
    template_matcher(1500), // TODO make configurable?
    original_template(template_cloud->getMatrixXfMap().topRows(3)),
    template_edges(_template_edges),
    last_lower_bounding_box(-6.0, -6.0, -6.0), // TODO make configurable?
    last_upper_bounding_box(6.0, 6.0, 6.0), // TODO make configurable?
    lle_neighbors(8), // TODO make configurable?
    // ENHANCE & ???: 1e-3 seems to be unnecessary
    m_lle(locally_linear_embedding(template_cloud, lle_neighbors, 1e-3)), // TODO make configurable?
    tolerance(1e-4), // TODO make configurable?
    alpha(_alpha), // TODO make configurable?
    beta(_beta), // TODO make configurable?
    w(0.1), // TODO make configurable?
    initial_sigma_scale(1.0 / 8), // TODO make configurable?
    start_lambda(_lambda), // TODO make configurable?
    annealing_factor(0.6), // TODO make configurable?
    k(_k),
    max_iterations(100), // TODO make configurable?
    kvis(1e3),
    use_recovery(_use_recovery),
    #ifdef PREDICT
    gripper_idx(grippers)
    #endif
    #ifdef SHAPE_COMP
	, obs_param(_obs_param),
	mesh(initObstacle(_obs_param))
    #endif
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(template_cloud);
    // W: (M, M) matrix, corresponding to L in Eq. (15) and (16)
    L_lle = barycenter_kneighbors_graph(kdtree, lle_neighbors, 0.001);
    // Qconstructor(template_edges, Q, original_template.cols());

    #ifdef PREDICT
    // TODO: how to configure nh so that the it can get correct sdf
    // grippers: indices of points gripped, a X*G matrix (X: depends on the case)
    // const auto sdf = smmap::GetEnvironmentSDF(*nh);

    const double res = 1.0;
    const double size = 10.0;
    const Eigen::Isometry3d origin_transform
        = Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Quaterniond(
            Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ()));
    auto map = sdf_tools::CollisionMapGrid(origin_transform, "world", res, size, size, 1.0, sdf_tools::COLLISION_CELL(0.0));
    const auto sdf = map.ExtractSignedDistanceField(1e6, true, false).first;
    auto sdf_ptr = std::make_shared<const sdf_tools::SignedDistanceField>(sdf);

    // initializa gripper data
    std::vector<smmap::GripperData> grippers_data;

    // format grippers_data
    std::cout << "gripper data when constructing CDCPD" << std::endl;
    std::cout << grippers << std::endl << std::endl;
    for (int gripper_idx = 0; gripper_idx < grippers.cols(); gripper_idx++) {
        std::vector<long> grip_node_idx;
        for (int node_idx = 0; node_idx < grippers.rows(); node_idx++) {
            grip_node_idx.push_back(long(grippers(node_idx, gripper_idx)));
        }
        std::string gripper_name;
        gripper_name = "gripper" + std::to_string(gripper_idx);
        smmap::GripperData gripper(gripper_name, grip_node_idx);
        grippers_data.push_back(gripper);
    }

    model->SetGrippersData(grippers_data);

    // set up collision check function
    model->SetCallbackFunctions(fake_collision_check);

    // set initial point configuration
    Matrix3Xf eigen_template_cloud = template_cloud->getMatrixXfMap().topRows(3);
    model->SetInitialObjectConfiguration(eigen_template_cloud.cast<double>());

    model = std::make_shared<smmap::ConstraintJacobianModel>(
                        nh,
                        translation_dir_deformability,
                        translation_dis_deformability,
                        rotation_deformability,
                        sdf_ptr);
    
	deformModel->SetInitialObjectConfiguration(eigen_template_cloud.cast<double>());

	deformModel = std::make_shared<smmap::DiminishingRigidityModel>(
						nh,
						translation_dir_deformability,
						rotation_deformability);
	#endif
	#ifdef SHAPE_COMP
	fnormals = mesh.add_property_map<face_descriptor, Vector>("f:normals", CGAL::NULL_VECTOR).first;
    vnormals = mesh.add_property_map<vertex_descriptor, Vector>("v:normals", CGAL::NULL_VECTOR).first;
	CGAL::Polygon_mesh_processing::compute_normals(mesh,
        										   vnormals,
        										   fnormals,
        										   CGAL::Polygon_mesh_processing::parameters::vertex_point_map(mesh.points()).
        										   geom_traits(K()));
	#endif
}

/*
 * Return a non-normalized probability that each of the tracked vertices produced any detected point.
 * Implement Eq. (7) in the paper
 */
VectorXf CDCPD::visibility_prior(const Matrix3Xf& vertices,
                                 const Mat& depth,
                                 const Mat& mask,
                                 const Matrix3f& intrinsics,
                                 const float kvis)
{
    // vertices: (3, M) matrix Y^t (Y in IV.A) in the paper
    // depth: CV_16U depth image
    // mask: CV_8U mask for segmentation
    // kvis: k_vis in the paper

#ifdef DEBUG
    Eigen::IOFormat np_fmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "");
    auto to_file = [&np_fmt](const std::string& fname, const MatrixXf& mat) {
        std::ofstream(fname) << mat.format(np_fmt);
    };

    // save matrices
    to_file(workingDir + "/cpp_verts.txt", vertices);
#endif

    // Project the vertices to get their corresponding pixel coordinate
    // ENHANCE: replace P_eigen.leftCols(3) with intrinsics
    Matrix3Xf image_space_vertices = intrinsics * vertices;

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
        // std::cout << "depth at " << coords(1, i) << " " << coords(0, i) << std::endl;
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
    VectorXf prob = (-kvis * score).array().exp().matrix();
    // ???: unnormalized prob
#ifdef DEBUG
    to_file(workingDir + "/cpp_prob.txt", prob);
#endif
    return prob;
}

VectorXi CDCPD::is_occluded(const Matrix3Xf& vertices,
                            const Mat& depth,
                            const Mat& mask,
                            const Matrix3f& intrinsics)
{
    // vertices: (3, M) matrix Y^t (Y in IV.A) in the paper
    // depth: CV_16U depth image
    // mask: CV_8U mask for segmentation
    // kvis: k_vis in the paper

    // Project the vertices to get their corresponding pixel coordinate
    Eigen::Matrix3Xf image_space_vertices = intrinsics * vertices;

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

    // Get image coordinates
    Eigen::Matrix2Xi coords = image_space_vertices.topRows(2).cast<int>();

    for (int i = 0; i < coords.cols(); ++i)
    {
        coords(0, i) = std::min(std::max(coords(0, i), 0), depth.cols - 1);
        coords(1, i) = std::min(std::max(coords(1, i), 1), depth.rows - 1);
    }

    // Find difference between point depth and image depth
    // depth_diffs: (1, M) vector
    Eigen::VectorXf depth_diffs = Eigen::VectorXf::Zero(vertices.cols());
    for (int i = 0; i < vertices.cols(); ++i)
    {
        // std::cout << "depth at " << coords(1, i) << " " << coords(0, i) << std::endl;
        #ifdef SIMULATION
        assert(depth.depth() == CV_32F);
        float raw_depth = depth.at<float>(coords(1, i), coords(0, i));
        #else
        assert(depth.depth() == CV_16U);
        uint16_t raw_depth = depth.at<uint16_t>(coords(1, i), coords(0, i));
        #endif
        if (raw_depth != 0)
        {
            depth_diffs(i) = vertices(2, i) - static_cast<float>(raw_depth) / 1000.0;
        }
        else
        {
            depth_diffs(i) = 0.02; // prevent numerical problems; taken from the Python code
        }
    }

    depth_diffs = depth_diffs.array().max(0.0);
    VectorXi occl_idx(depth_diffs.size());
    for (int i = 0; i < depth_diffs.size(); ++i)
    {
        if (depth_diffs(i) > 0.5) {
            occl_idx(i) = 1;
        }
        else
        {
            occl_idx(i) = 0;
        }
    }

    return occl_idx;
}

Matrix3Xf CDCPD::blend_result(const Matrix3Xf& Y_pred,
                       const Matrix3Xf& Y_cpd,
                       const VectorXi& is_occluded)
{
    Matrix3Xf result(3, Y_pred.cols());
    for (int i = 0; i < is_occluded.size(); ++i)
    {
        if (is_occluded(i))
        {
            result.col(i) = Y_pred.col(i);
        }
        else
        {
            result.col(i) = Y_cpd.col(i);
        }
    }

    return result;
}

/*
 * Used for failure recovery. Very similar to the visibility_prior function, but with a different K and
 * image_depth - vertex_depth, not vice-versa. There's also no normalization.
 * Calculate Eq. (22) in the paper
 */
static float smooth_free_space_cost(const Matrix3Xf vertices,
                                    const Eigen::Matrix3f& intrinsics,
                                    const cv::Mat& depth,
                                    const cv::Mat& mask,
                                    const float k = 1e2)
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
        // std::cout << "depth at " << coords(1, i) << " " << coords(0, i) << std::endl;
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
#ifdef ENTIRE
static std::tuple<PointCloudRGB::Ptr, PointCloud::Ptr>
#else
static std::tuple<PointCloud::Ptr>
#endif
point_clouds_from_images(const cv::Mat& depth_image,
                         const cv::Mat& rgb_image,
                         const cv::Mat& mask,
                         const Eigen::Matrix3f& intrinsics,
                         const Eigen::Vector3f& lower_bounding_box,
                         const Eigen::Vector3f& upper_bounding_box)
{
    // depth_image: CV_16U depth image
    // rgb_image: CV_8U3C rgb image
    // mask: CV_8U mask for segmentation
    // intrinsic matrix of Kinect using the Pinhole camera model
    //  [[fx 0  px];
    //   [0  fy py];
    //   [0  0  1 ]]
    // lower_bounding_box_vec: bounding for mask
    // upper_bounding_box_vec: bounding for mask

    #ifdef SIMULATION
    using T = float;
    float pixel_len = 0.0000222222;
    #else
    using T = uint16_t;
    float pixel_len = 0.0002645833;
    #endif
    using DepthTraits = cdcpd::DepthTraits<T>;

    // Use correct principal point from calibration
    auto const center_x = intrinsics(0, 2);
    auto const center_y = intrinsics(1, 2);

    auto const unit_scaling = DepthTraits::toMeters(T(1));
    auto const constant_x = 1.0f / (intrinsics(0, 0) * pixel_len);
    auto const constant_y = 1.0f / (intrinsics(1, 1) * pixel_len);
    auto constexpr bad_point = std::numeric_limits<float>::quiet_NaN();

    // ENHANCE: change the way to get access to depth image and rgb image to be more readable
    // from "depth_row += row_step" to ".at<>()"
    /*
    const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth_image.data[0]);
    int row_step = depth_image.step / sizeof(uint16_t); // related with accessing data in cv::Mat
    const uint8_t* rgb = &rgb_image.data[0];
    int rgb_step = 3; // TODO check on this
    int rgb_skip = rgb_image.step - rgb_image.cols * rgb_step;
    */

    PointCloud::Ptr filtered_cloud(new PointCloud);
    #ifdef ENTIRE
    PointCloudRGB::Ptr unfiltered_cloud(new PointCloudRGB(depth_image.cols, depth_image.rows));
    auto unfiltered_iter = unfiltered_cloud->begin();
    #endif

    for (int v = 0; v < depth_image.rows; ++v)
    {
        for (int u = 0; u < depth_image.cols; ++u)
        {
            T depth = depth_image.at<T>(v, u);

            // Assume depth = 0 is the standard was to note invalid
            if (DepthTraits::valid(depth))
            {
                float x = (float(u) - center_x) * pixel_len * float(depth) * unit_scaling * constant_x;
                float y = (float(v) - center_y) * pixel_len * float(depth) * unit_scaling * constant_y;
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

                Eigen::Array<float, 3, 1> point(x, y, z);
                if (mask.at<bool>(v, u) &&
                    point.min(upper_bounding_box.array()).isApprox(point) &&
                    point.max(lower_bounding_box.array()).isApprox(point))
                {
                    filtered_cloud->push_back(pcl::PointXYZ(x, y, z));
                }
            }
            #ifdef ENTIRE
            else
            {
                unfiltered_iter->x = unfiltered_iter->y = unfiltered_iter->z = bad_point;
            }
            ++unfiltered_iter;
            #endif
        }
    }

    #ifdef ENTIRE
    assert(unfiltered_iter == unfiltered_cloud->end());
    return { unfiltered_cloud, filtered_cloud };
    #else
    return { filtered_cloud };
    #endif
}

static void sample_X_points(const Matrix3Xf& Y,
                            const VectorXf& Y_emit_prior,
                            const double sigma2,
                            PointCloud::Ptr& cloud)
{
    int N = 40;
    for (int m = 0; m < Y.cols(); ++m)
    {
        int num_pts = int(float(N)*(1.0f-Y_emit_prior(m)));
        std::default_random_engine generator;
        std::normal_distribution<double> distrib_x(Y(0, m), sigma2);
        std::normal_distribution<double> distrib_y(Y(1, m), sigma2);
        std::normal_distribution<double> distrib_z(Y(2, m), sigma2);

        for (int i = 0; i < num_pts; ++i)
        {
            double x = distrib_x(generator);
            double y = distrib_y(generator);
            double z = distrib_z(generator);

            cloud->push_back(pcl::PointXYZ(x, y, z));
        }
    }
}

Matrix3Xf CDCPD::cpd(const Matrix3Xf& X,
                     const Matrix3Xf& Y,
                     const Matrix3Xf& Y_pred,
                     const cv::Mat& depth,
                     const cv::Mat& mask)
{
    // downsampled_cloud: PointXYZ pointer to downsampled point clouds
    // Y: (3, M) matrix Y^t (Y in IV.A) in the paper
    // depth: CV_16U depth image
    // mask: CV_8U mask for segmentation label

    // Eigen::VectorXf Y_emit_prior = visibility_prior(Y, depth, mask, kvis);
    Eigen::VectorXf Y_emit_prior(Y.cols());
    for (int i = 0; i < Y.cols(); ++i)
    {
        Y_emit_prior(i) = 1.0f;
    }

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
    std::cout << "\nCPD loop\n";
    std::cout << std::setw(20) << "loop" << std::setw(20) << "prob term" << std::setw(20) << "CPD term" << std::setw(20) << "LLE term" << std::endl;
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

        MatrixXf P(M, N); // end = std::chrono::system_clock::now(); std::cout << "526: " << (end-start).count() << std::endl;
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
        }  //end = std::chrono::system_clock::now(); std::cout << "545: " << (end-start).count() << std::endl;

        // Fast Gaussian Transformation to calculate Pt1, P1, PX

        // double bandwidth = std::sqrt(2.0 * sigma2);
        // double epsilon = 1e-4;
        // fgt::Matrix Y_fgt = TY.transpose().cast<double>(); //std::cout << "575\n";
        // fgt::Matrix X_fgt = X.transpose().cast<double>(); //std::cout << "576\n";
        // fgt::DirectTree fgt1(Y_fgt, bandwidth, epsilon); //std::cout << "577\n";
        // VectorXd kt1 = fgt1.compute(X_fgt, Y_emit_prior.cast<double>()); //std::cout << "578\n";// N*1 vector

        // float c = std::pow(2 * M_PI * sigma2, static_cast<double>(D) / 2);
        // c *= w / (1 - w);
        // c *= static_cast<double>(M) / N;
        // ArrayXd a = 1 / (kt1.array() + c); // N*1 array

        // VectorXf Pt1 = (1 - c * a).cast<float>(); // M*1 array

        // fgt::DirectTree fgt2(X_fgt, bandwidth, epsilon);  //std::cout << "587\n";
        // VectorXf P1 = (fgt2.compute(Y_fgt, a)).cast<float>(); //std::cout << "588\n";
        // P1 = P1.array()*Y_emit_prior.array();

        // MatrixXd PX_fgt(TY.rows(), TY.cols());
        // for (size_t i = 0; i < TY.rows(); ++i) {
        //     // ArrayXd Xi = X_fgt.col(i).array();
        //     ArrayXd aXarray = X_fgt.col(i).array() * a;
        //     fgt::Vector aX = fgt::Vector(aXarray);  //std::cout << "594\n";
        //     PX_fgt.row(i) = (fgt2.compute(Y_fgt, aX)).array() * Y_emit_prior.array().cast<double>();
        // }
        // MatrixXf PX = PX_fgt.cast<float>();
        MatrixXf PX = (P * X.transpose()).transpose();

        // // Maximization step
        VectorXf Pt1 = P.colwise().sum();// end = std::chrono::system_clock::now(); std::cout << "548: " << (end-start).count() << std::endl;
        VectorXf P1 = P.rowwise().sum();// end = std::chrono::system_clock::now(); std::cout << "549: " << (end-start).count() << std::endl;
        float Np = P1.sum(); //end = std::chrono::system_clock::now(); std::cout << "550: " << (end-start).count() << std::endl;

        // NOTE: lambda means gamma here
        // Corresponding to Eq. (18) in the paper
        float zeta = 2.0;
        float lambda = start_lambda;
        MatrixXf p1d = P1.asDiagonal(); //end = std::chrono::system_clock::now(); std::cout << "557: " << (end-start).count() << std::endl;

		// obstacle normalization
		// float obs_cons = 100.0;
		// auto [nearestPts, normalVecs] = nearest_points_and_normal_help(TY, mesh, vnormals);
		// MatrixXf dist_to_obs = ((TY-nearestPts).array() * normalVecs.array()).colwise().sum();
		// VectorXf mask_vec(dist_to_obs.cols());
		// for(int idx = 0; idx < dist_to_obs.cols(); idx++)
		// {
		//	if (dist_to_obs(0, idx) < 0.2)
		//	{
		//		mask_vec(idx) = 1.0;
		//	} else
		//	{
		//		mask_vec(idx) = 0.0;
		//	}
		// }
		// cout << "mask:" << endl;
		// cout << mask_vec << endl << endl; 
		// MatrixXf obs_reg = obs_cons * sigma2 * mask_vec.asDiagonal() * normalVecs.transpose();
        
		MatrixXf A = (P1.asDiagonal() * G)
            + alpha * sigma2 * MatrixXf::Identity(M, M)
            + sigma2 * lambda * (m_lle * G)
            + zeta * G;// end = std::chrono::system_clock::now();std::cout << "560: " << (end-start).count() << std::endl;

        MatrixXf B = PX.transpose() - (p1d + sigma2 * lambda * m_lle) * Y.transpose() + zeta * (Y_pred.transpose() - Y.transpose()); //end = std::chrono::system_clock::now();std::cout << "561: " << (end-start).count() << std::endl;

        MatrixXf W = (A).householderQr().solve(B); //+obs_reg);
        // MatrixXf lastW = W;

        // int W_int = 0;
        // std::cout << std::setw(20) << "W solver loop";
        // std::cout << std::setw(20) << "W difference" << std::endl;
        // while (W_int == 0 || (W_int <= max_iterations && !W.isApprox(lastW))) {
        //     lastW = W;
        //     MatrixXf A_new = A;
        //     MatrixXf B_new = B;
        //     for (int i = 0; i < template_edges.cols(); ++i)
        //     {
        //         MatrixXf eit = (Y+G*lastW)*Q[i]*(Y+G*lastW).transpose();
        //         MatrixXf ei0 = original_template*Q[i]*original_template.transpose();
        //         double delta = abs_derivative(eit.trace()-ei0.trace());
        //         // double delta = -1.0;
        //         A_new = A_new + k*sigma2*delta*Q[i]*G;
        //         B_new = B_new - k*sigma2*delta*Q[i]*Y.transpose();
        //     }
        //     W = A_new.householderQr().solve(B_new); //end = std::chrono::system_clock::now(); std::cout << "562: " << (end-start).count() << std::endl;
        //     std::cout << std::setw(20) << W_int;
        //     std::cout << std::setw(20) << (W-lastW).squaredNorm() << std::endl;
        //     W_int++;
        // }

        // MatrixXf W(M, D);
        // Wsolver(P, X, Y, G, L_lle, sigma2, alpha, start_lambda, W);

        TY = Y + (G * W).transpose();// end = std::chrono::system_clock::now(); std::cout << "564: " << (end-start).count() << std::endl;

        // Corresponding to Eq. (19) in the paper
        VectorXf xPxtemp = (X.array() * X.array()).colwise().sum();
        // float xPx = (Pt1 * xPxtemp.transpose())(0,0);
        double xPx = Pt1.dot(xPxtemp);// end = std::chrono::system_clock::now();std::cout << "569: " << (end-start).count() << std::endl;
        // assert(xPxMat.rows() == 1 && xPxMat.cols() == 1);
        // double xPx = xPxMat.sum();
        VectorXf yPytemp = (TY.array() * TY.array()).colwise().sum();
        // MatrixXf yPyMat = P1.transpose() * yPytemp.transpose();
        // assert(yPyMat.rows() == 1 && yPyMat.cols() == 1);
        double yPy = P1.dot(yPytemp); //end = std::chrono::system_clock::now();std::cout << "575: " << (end-start).count() << std::endl;
        double trPXY = (TY.array() * PX.array()).sum(); //end = std::chrono::system_clock::now();std::cout << "576: " << (end-start).count() << std::endl;
        sigma2 = (xPx - 2 * trPXY + yPy) / (Np * D); //end = std::chrono::system_clock::now();std::cout << "577: " << (end-start).count() << std::endl;

        if (sigma2 <= 0)
        {
            sigma2 = tolerance / 10;
        }

        #ifdef CPDLOG
        double prob_reg = calculate_prob_reg(X, TY, G, W, sigma2, Y_emit_prior, P);
        double lle_reg = start_lambda / 2 * ((TY*m_lle*TY.transpose()).trace() + 2*(W.transpose()*G*m_lle*TY).trace() + (W.transpose()*G*m_lle*G*W).trace());
        double cpd_reg = alpha * (W.transpose()*G*W).trace()/2;
        std::cout << std::setw(20) << iterations;
        std::cout << std::setw(20) << prob_reg;
        std::cout << std::setw(20) << cpd_reg;
        std::cout << std::setw(20) << lle_reg << std::endl;
        #endif


        error = std::abs(sigma2 - qprev);
        iterations++;
    }
    return TY;
}

Matrix3Xf CDCPD::cpd(const Matrix3Xf& X,
                     const Matrix3Xf& Y,
                     const cv::Mat& depth,
                     const cv::Mat& mask)
{
    // downsampled_cloud: PointXYZ pointer to downsampled point clouds
    // Y: (3, M) matrix Y^t (Y in IV.A) in the paper
    // depth: CV_16U depth image
    // mask: CV_8U mask for segmentation label

    // Eigen::VectorXf Y_emit_prior = visibility_prior(Y, depth, mask, kvis);
    Eigen::VectorXf Y_emit_prior(Y.cols());
    for (int i = 0; i < Y.cols(); ++i)
    {
        Y_emit_prior(i) = 1.0f;
    }

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
    std::cout << "\nCPD loop\n";
    std::cout << std::setw(20) << "loop" << std::setw(20) << "prob term" << std::setw(20) << "CPD term" << std::setw(20) << "LLE term" << std::endl;
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

        MatrixXf P(M, N); // end = std::chrono::system_clock::now(); std::cout << "526: " << (end-start).count() << std::endl;
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
        }  //end = std::chrono::system_clock::now(); std::cout << "545: " << (end-start).count() << std::endl;

        // Fast Gaussian Transformation to calculate Pt1, P1, PX

        // double bandwidth = std::sqrt(2.0 * sigma2);
        // double epsilon = 1e-4;
        // fgt::Matrix Y_fgt = TY.transpose().cast<double>(); //std::cout << "575\n";
        // fgt::Matrix X_fgt = X.transpose().cast<double>(); //std::cout << "576\n";
        // fgt::DirectTree fgt1(Y_fgt, bandwidth, epsilon); //std::cout << "577\n";
        // VectorXd kt1 = fgt1.compute(X_fgt, Y_emit_prior.cast<double>()); //std::cout << "578\n";// N*1 vector

        // float c = std::pow(2 * M_PI * sigma2, static_cast<double>(D) / 2);
        // c *= w / (1 - w);
        // c *= static_cast<double>(M) / N;
        // ArrayXd a = 1 / (kt1.array() + c); // N*1 array

        // VectorXf Pt1 = (1 - c * a).cast<float>(); // M*1 array

        // fgt::DirectTree fgt2(X_fgt, bandwidth, epsilon);  //std::cout << "587\n";
        // VectorXf P1 = (fgt2.compute(Y_fgt, a)).cast<float>(); //std::cout << "588\n";
        // P1 = P1.array()*Y_emit_prior.array();

        // MatrixXd PX_fgt(TY.rows(), TY.cols());
        // for (size_t i = 0; i < TY.rows(); ++i) {
        //     // ArrayXd Xi = X_fgt.col(i).array();
        //     ArrayXd aXarray = X_fgt.col(i).array() * a;
        //     fgt::Vector aX = fgt::Vector(aXarray);  //std::cout << "594\n";
        //     PX_fgt.row(i) = (fgt2.compute(Y_fgt, aX)).array() * Y_emit_prior.array().cast<double>();
        // }
        // MatrixXf PX = PX_fgt.cast<float>();
        MatrixXf PX = (P * X.transpose()).transpose();

        // // Maximization step
        VectorXf Pt1 = P.colwise().sum();// end = std::chrono::system_clock::now(); std::cout << "548: " << (end-start).count() << std::endl;
        VectorXf P1 = P.rowwise().sum();// end = std::chrono::system_clock::now(); std::cout << "549: " << (end-start).count() << std::endl;
        float Np = P1.sum(); //end = std::chrono::system_clock::now(); std::cout << "550: " << (end-start).count() << std::endl;

        // NOTE: lambda means gamma here
        // Corresponding to Eq. (18) in the paper
        float lambda = start_lambda;
        MatrixXf p1d = P1.asDiagonal(); //end = std::chrono::system_clock::now(); std::cout << "557: " << (end-start).count() << std::endl;
        
        MatrixXf A = (P1.asDiagonal() * G)
            + alpha * sigma2 * MatrixXf::Identity(M, M)
            + sigma2 * lambda * (m_lle * G);// end = std::chrono::system_clock::now();std::cout << "560: " << (end-start).count() << std::endl;

        MatrixXf B = PX.transpose() - (p1d + sigma2 * lambda * m_lle) * Y.transpose(); //end = std::chrono::system_clock::now();std::cout << "561: " << (end-start).count() << std::endl;

        MatrixXf W = A.householderQr().solve(B);
        // MatrixXf lastW = W;

        // int W_int = 0;
        // std::cout << std::setw(20) << "W solver loop";
        // std::cout << std::setw(20) << "W difference" << std::endl;
        // while (W_int == 0 || (W_int <= max_iterations && !W.isApprox(lastW))) {
        //     lastW = W;
        //     MatrixXf A_new = A;
        //     MatrixXf B_new = B;
        //     for (int i = 0; i < template_edges.cols(); ++i)
        //     {
        //         MatrixXf eit = (Y+G*lastW)*Q[i]*(Y+G*lastW).transpose();
        //         MatrixXf ei0 = original_template*Q[i]*original_template.transpose();
        //         double delta = abs_derivative(eit.trace()-ei0.trace());
        //         // double delta = -1.0;
        //         A_new = A_new + k*sigma2*delta*Q[i]*G;
        //         B_new = B_new - k*sigma2*delta*Q[i]*Y.transpose();
        //     }
        //     W = A_new.householderQr().solve(B_new); //end = std::chrono::system_clock::now(); std::cout << "562: " << (end-start).count() << std::endl;
        //     std::cout << std::setw(20) << W_int;
        //     std::cout << std::setw(20) << (W-lastW).squaredNorm() << std::endl;
        //     W_int++;
        // }

        // MatrixXf W(M, D);
        // Wsolver(P, X, Y, G, L_lle, sigma2, alpha, start_lambda, W);

        TY = Y + (G * W).transpose();// end = std::chrono::system_clock::now(); std::cout << "564: " << (end-start).count() << std::endl;

        // Corresponding to Eq. (19) in the paper
        VectorXf xPxtemp = (X.array() * X.array()).colwise().sum();
        // float xPx = (Pt1 * xPxtemp.transpose())(0,0);
        double xPx = Pt1.dot(xPxtemp);// end = std::chrono::system_clock::now();std::cout << "569: " << (end-start).count() << std::endl;
        // assert(xPxMat.rows() == 1 && xPxMat.cols() == 1);
        // double xPx = xPxMat.sum();
        VectorXf yPytemp = (TY.array() * TY.array()).colwise().sum();
        // MatrixXf yPyMat = P1.transpose() * yPytemp.transpose();
        // assert(yPyMat.rows() == 1 && yPyMat.cols() == 1);
        double yPy = P1.dot(yPytemp); //end = std::chrono::system_clock::now();std::cout << "575: " << (end-start).count() << std::endl;
        double trPXY = (TY.array() * PX.array()).sum(); //end = std::chrono::system_clock::now();std::cout << "576: " << (end-start).count() << std::endl;
        sigma2 = (xPx - 2 * trPXY + yPy) / (Np * D); //end = std::chrono::system_clock::now();std::cout << "577: " << (end-start).count() << std::endl;

        if (sigma2 <= 0)
        {
            sigma2 = tolerance / 10;
        }

        #ifdef CPDLOG
        double prob_reg = calculate_prob_reg(X, TY, G, W, sigma2, Y_emit_prior, P);
        double lle_reg = start_lambda / 2 * ((TY*m_lle*TY.transpose()).trace() + 2*(W.transpose()*G*m_lle*TY).trace() + (W.transpose()*G*m_lle*G*W).trace());
        double cpd_reg = alpha * (W.transpose()*G*W).trace()/2;
        std::cout << std::setw(20) << iterations;
        std::cout << std::setw(20) << prob_reg;
        std::cout << std::setw(20) << cpd_reg;
        std::cout << std::setw(20) << lle_reg << std::endl;
        #endif


        error = std::abs(sigma2 - qprev);
        iterations++;
    }
    return TY;
}

#ifdef PREDICT
Matrix3Xd CDCPD::predict(const Matrix3Xd& P,
                         const AllGrippersSinglePoseDelta& q_dot,
                         const AllGrippersSinglePose& q_config,
						 const int pred_choice)
{
    // P: template
    // q_dot: velocity of gripper, a 6*G matrix
    // q_config: indices of points gripped, a X*G matrix (X: depends on the case)
	// pred_choice:
	// 	- 0: no movement
	// 	- 1: Dmitry's prediction
	//  - 2: Mengyao's prediction

    // NOTE:
    //      - WorldState definition
    //      struct WorldState
    //      {
    //      public:
    //          ObjectPointSet object_configuration_;
    //          EigenHelpers::VectorIsometry3d rope_node_transforms_;
    //          AllGrippersSinglePose all_grippers_single_pose_;
    //          Eigen::VectorXd robot_configuration_;
    //          bool robot_configuration_valid_;
    //          std::vector<CollisionData> gripper_collision_data_;
    //          double sim_time_;

    //          uint64_t serializeSelf(std::vector<uint8_t>& buffer) const;

    //          static uint64_t Serialize(const WorldState& state, std::vector<uint8_t>& buffer);

    //          static std::pair<WorldState, uint64_t> Deserialize(const std::vector<uint8_t>& buffer, const uint64_t current);

    //          bool operator==(const WorldState& other) const;
    //      };
    //      - Needed attribute:
    //          - object_configuration_
    //          - all_grippers_single_pose_
    //      - AllGrippersSinglePoseDelta: kinematics::VectorVector6d (std::vector of eigen's 6D vector)
    //      - ObjectPointSet: Eigen::Matrix3Xd
    //      - AllGrippersSinglePose: EigenHelpers::VectorIsometry3d (std::vector of Isometry3d)
    if (pred_choice == 0) {
		return P;
	}
	else {
		smmap::WorldState world;
    	world.object_configuration_ = P;
    	world.all_grippers_single_pose_ = q_config;
    	smmap::AllGrippersSinglePoseDelta grippers_pose_delta = q_dot;
    	// cout << "gripper configuration:" << endl;
    	// for (int i = 0; i < q_config.size(); ++i)
    	// {
        	// cout << q_config[i].rotation() << endl << endl;
        	// cout << q_config[i].translation() << endl << endl;
    	// }
    	// cout << "gripper velocity" << endl;
        // for (int i = 0; i < q_dot.size(); ++i)
    	// {
        	// cout << q_dot[i] << endl << endl;
    	// }
		if (pred_choice == 1) {
			return model->getObjectDelta(world, grippers_pose_delta) + P;
		} else {
			return deformModel->getObjectDelta(world, grippers_pose_delta) + P;
		}
	}
}
#endif


CDCPD::Output CDCPD::operator()(
        const Mat& rgb,
        const Mat& depth,
        const Mat& mask,
        const cv::Matx33d& intrinsics,
        const PointCloud::Ptr template_cloud,
        #ifdef PREDICT
        const AllGrippersSinglePoseDelta& q_dot,
        const AllGrippersSinglePose& q_config,
        #endif
        const bool self_intersection,
        const bool interation_constrain,
        const bool is_prediction,
		const int pred_choice,
        const std::vector<CDCPD::FixedPoint>& fixed_points)
{
    // rgb: CV_8U3C rgb image
    // depth: CV_16U depth image
    // mask: CV_8U mask for segmentation
    // template_cloud: point clouds corresponding to Y^t (Y in IV.A) in the paper
    // template_edges: (2, K) matrix corresponding to E in the paper
    // fixed_points: fixed points during the tracking

    assert(rgb.type() == CV_8UC3);
    #ifdef SIMULATION
    assert(depth.type() == CV_32F);
    #else
    assert(depth.type() == CV_16U);
    #endif
    assert(mask.type() == CV_8U);
    assert(rgb.rows == depth.rows && rgb.cols == depth.cols);

    size_t recovery_knn_k = 12; // TODO configure this?
    float recovery_cost_threshold = 0.5; // TODO configure this?

    Eigen::IOFormat np_fmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "");

    // Useful utility for outputting an Eigen matrix to a file
    auto to_file = [&np_fmt](const std::string& fname, const MatrixXf& mat) {
        std::ofstream(fname, std::ofstream::app) << mat.format(np_fmt) << "\n\n";
    };

    Eigen::Matrix3d intrinsics_eigen_tmp;
    cv::cv2eigen(intrinsics, intrinsics_eigen_tmp);
    Eigen::Matrix3f intrinsics_eigen = intrinsics_eigen_tmp.cast<float>();

    Eigen::Vector3f const bounding_box_extend = Vector3f(0.2, 0.2, 0.2);

    // entire_cloud: pointer to the entire point cloud
    // cloud: pointer to the point clouds selected
    #ifdef ENTIRE
    auto [entire_cloud, cloud]
    #else
    auto [cloud]
    #endif
        = point_clouds_from_images(
                depth,
                rgb,
                mask,
                intrinsics_eigen,
                last_lower_bounding_box - bounding_box_extend,
                last_upper_bounding_box + bounding_box_extend);
    std::cout << "Points in filtered: (" << cloud->height << " x " << cloud->width << ")\n";

    /// VoxelGrid filter downsampling
    PointCloud::Ptr cloud_downsampled(new PointCloud);
    const Matrix3Xf Y = template_cloud->getMatrixXfMap().topRows(3);
    double sigma2 = 0.002;
    // TODO: check whether the change is needed here for unit conversion
    Eigen::VectorXf Y_emit_prior = visibility_prior(Y, depth, mask, intrinsics_eigen, kvis);
    // std::cout << "P_vis:" << std::endl;
    // std::cout << Y_emit_prior << std::endl << std::endl;
    // sample_X_points(Y, Y_emit_prior, sigma2, cloud);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    std::cout << "Points in cloud before leaf: " << cloud->width << std::endl;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.02f, 0.02f, 0.02f);
    sor.filter(*cloud_downsampled);
    std::cout << "Points in fully filtered: " << cloud_downsampled->width << std::endl;
    Matrix3Xf X = cloud_downsampled->getMatrixXfMap().topRows(3);
    // Add points to X according to the previous template

    #ifdef ENTIRE
    const Matrix3Xf& entire = entire_cloud->getMatrixXfMap().topRows(3);
    #endif

    #ifdef PREDICT
    // NOTE: order of P cannot influence delta_P, but influence P+delta_P
    // std::chrono::time_point<std::chrono::system_clock> start, end;
    std::vector<FixedPoint> pred_fixed_points;
	cout << "gripper_idx" << endl;
	cout << gripper_idx << endl << endl;
    for (int col = 0; col < gripper_idx.cols(); ++col)
    {
        FixedPoint pt;
		#ifdef ROPE
        pt.template_index = gripper_idx(2, col);
		#else
		pt.template_index = gripper_idx(0, col);
		#endif
        pt.position(0) = q_config[col](0, 3);
        pt.position(1) = q_config[col](1, 3);
        pt.position(2) = q_config[col](2, 3);
        pred_fixed_points.push_back(pt);
    }
    Matrix3Xf TY, TY_pred;
    if (is_prediction) {
        // start = std::chrono::system_clock::now(); 
        TY_pred = predict(Y.cast<double>(), q_dot, q_config, pred_choice).cast<float>(); // end = std::chrono::system_clock::now(); std::cout << "predict: " <<  std::chrono::duration<double>(end - start).count() << std::endl;
        // TY_pred = Y;
        // for (int col = 0; col < gripper_idx.cols(); ++col)
        // {
        //     for (int row = gripper_idx.rows() - 1; row < gripper_idx.rows(); ++row)
        //     {
        //         FixedPoint pt;
        //         pt.template_index = gripper_idx(row, col);
        //         pt.position = TY_pred.col(pt.template_index);
        //         pred_fixed_points.push_back(pt);
        //     }
        // }
        // VectorXi occl_idx = is_occluded(TY_pred, depth, mask, intrinsics_eigen);
        // std::ofstream(workingDir + "/occluded_index.txt", std::ofstream::out) << occl_idx << "\n\n";
		TY = cpd(X, Y, TY_pred, depth, mask); // end = std::chrono::system_clock::now(); std::cout << "cpd: " <<  std::chrono::duration<double>(end - start).count() << std::endl;
        // Matrix3Xf TY = blend_result(TY_pred, TY_cpd, occl_idx);
    }
    else
    {
        // for (int col = 0; col < gripper_idx.cols(); ++col)
        // {
        //     for (int row = gripper_idx.rows() - 1; row < gripper_idx.rows(); ++row)
        //     {
        //         FixedPoint pt;
        //         pt.template_index = gripper_idx(row, col);
        //         pt.position = TY_pred.col(pt.template_index);
        //         pred_fixed_points.push_back(pt);
        //     }
        // }
        TY = cpd(X, Y, depth, mask);
    } 
    #else
    Eigen::Matrix3Xf TY = cpd(X, Y, depth, mask);
    #endif


    #ifdef DEBUG
    to_file(workingDir + "/cpp_entire_cloud.txt", entire);
    to_file(workingDir + "/cpp_downsample.txt", X);
    to_file(workingDir + "/cpp_TY-1.txt", template_cloud->getMatrixXfMap().topRows(3));
    to_file(workingDir + "/cpp_TY.txt", TY);
    #endif

    // Next step: optimization.
    // ???: most likely not 1.0
    #ifdef SHAPE_COMP
    Optimizer opt(original_template, Y, 1.0, obs_param);
    #else
    Optimizer opt(original_template, Y, 1.0);
    #endif

    Matrix3Xf Y_opt = opt(TY, template_edges, pred_fixed_points, self_intersection, interation_constrain);
    // end = std::chrono::system_clock::now(); std::cout << "opt: " <<  std::chrono::duration<double>(end - start).count() << std::endl;
    #ifdef DEBUG
    to_file(workingDir + "/cpp_Y_opt.txt", Y_opt);
    #endif

    // If we're doing tracking recovery, do that now
    #ifndef PREDICT
    if (use_recovery)
    {
        std::cout << "matcher size: " << template_matcher.size() << std::endl;

        float cost = smooth_free_space_cost(Y_opt, intrinsics_eigen, depth, mask);
        std::cout << "cost" << std::endl;
        std::cout << cost << std::endl;

        if (cost > recovery_cost_threshold && template_matcher.size() > recovery_knn_k)
        {
            float best_cost = cost;
            Matrix3Xf final_tracking_result = Y_opt;

            std::vector<Matrix3Xf> matched_templates = template_matcher.query_template(cloud_downsampled, recovery_knn_k);

            // TODO there's potentially a lot of copying going on here. We should be able to avoid that.
            for (const Matrix3Xf& templ : matched_templates)
            {
                Matrix3Xf proposed_recovery = cpd(cloud_downsampled->getMatrixXfMap().topRows(3), templ, depth, mask);
                // TODO if we end up being able to disable optimization, we should not call this
                proposed_recovery = opt(proposed_recovery, template_edges, fixed_points);
                float proposal_cost = smooth_free_space_cost(proposed_recovery, intrinsics_eigen, depth, mask);
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
    #endif

    // Set the min and max for the box filter for next time
    last_lower_bounding_box = Y_opt.rowwise().minCoeff();
    last_upper_bounding_box = Y_opt.rowwise().maxCoeff();

    PointCloud::Ptr cdcpd_out = mat_to_cloud(Y_opt);
    PointCloud::Ptr cdcpd_cpd = mat_to_cloud(TY);
    #ifdef PREDICT
    PointCloud::Ptr cdcpd_pred = mat_to_cloud(TY_pred);
    #endif

    return CDCPD::Output {
        #ifdef ENTIRE
        entire_cloud,
        #endif
        cloud,
        cloud_downsampled,
        cdcpd_cpd,
        #ifdef PREDICT
        cdcpd_pred,
        #endif
        cdcpd_out
    };
}
