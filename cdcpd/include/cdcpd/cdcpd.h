#ifndef CDCPD_H
#define CDCPD_H

#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <arc_utilities/ros_helpers.hpp>

#include <smmap_models/constraint_jacobian_model.h>
#include <smmap_models/diminishing_rigidity_model.h>
#include <smmap_utilities/grippers.h>

#include "cdcpd/past_template_matcher.h"
#include "cdcpd/obs_util.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/draw_surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/subdivision_method_3.h>

#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/smooth_mesh.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>


// #ifndef DEBUG
// #define DEBUG
// #endif

#ifndef ENTIRE
#define ENTIRE
#endif

// #ifndef COMP
// #define COMP
// #endif

// #ifndef CPDLOG
// #define CPDLOG
// #endif

// #ifndef CYLINDER_INTER
// #define CYLINDER_INTER
// #endif

// #ifndef CYL_CLOTH4
// #define CYL_CLOTH4
// #endif

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::FT FT;
typedef K::Point_3 Point_3;
typedef K::Ray_3 Ray_3;
typedef K::Vector_3 Vector;
typedef CGAL::Surface_mesh<Point_3> Mesh;
typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> AABB_face_graph_primitive;
typedef CGAL::AABB_traits<K, AABB_face_graph_primitive> AABB_face_graph_traits;

namespace PMP = CGAL::Polygon_mesh_processing;

typedef PMP::Face_location<Mesh, FT> Face_location;

Eigen::MatrixXf barycenter_kneighbors_graph(const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,
                                            int lle_neighbors,
                                            double reg);

Eigen::MatrixXf locally_linear_embedding(pcl::PointCloud<pcl::PointXYZ>::ConstPtr template_cloud,
                                         int lle_neighbors,
                                         double reg);

class CDCPD
{
 public:
  struct Output
  {
#ifdef ENTIRE
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud;
#endif
    pcl::PointCloud<pcl::PointXYZ>::Ptr masked_point_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cpd_output;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cpd_predict;
    pcl::PointCloud<pcl::PointXYZ>::Ptr gurobi_output;
  };

  struct FixedPoint
  {
    Eigen::Vector3f position;
    int template_index;
  };

  CDCPD(ros::NodeHandle nh,
        ros::NodeHandle ph,
        PointCloud::ConstPtr template_cloud,
        const Eigen::Matrix2Xi &_template_edges,
        const Eigen::MatrixXi &gripper_idx,
        bool _use_recovery = true,
        double alpha = 0.5,
        double beta = 1.0,
        double lambda = 1.0,
        double k = 100.0,
        float zeta = 10.0,
        bool is_sim = false);

  Output operator()(const cv::Mat &rgb, // RGB image
                    const cv::Mat &depth, // Depth image
                    const cv::Mat &mask,
                    const cv::Matx33d &intrinsics,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud,
                    obsParam const &obs_param = {},
                    const smmap::AllGrippersSinglePoseDelta &q_dot = {},
                    const smmap::AllGrippersSinglePose &q_config = {},
                    const std::vector<bool> &is_grasped = {},
                    bool self_intersection = true,
                    bool interaction_constrain = true,
                    bool is_prediction = true,
                    int pred_choice = 0,
                    double translation_dir_deformability = 0,
                    double translation_dis_deformability = 0,
                    double rotation_deformability = 0);

 private:
  Eigen::VectorXf visibility_prior(const Eigen::Matrix3Xf &vertices,
                                   const cv::Mat &depth,
                                   const cv::Mat &mask,
                                   const Eigen::Matrix3f &intrinsics,
                                   float kvis);

#ifdef NOUSE
  Eigen::VectorXi is_occluded(const Eigen::Matrix3Xf& vertices,
                              const cv::Mat& depth,
                              const cv::Mat& mask,
                              const Eigen::Matrix3f& intrinsics,
              const bool is_sim);
#endif

  Eigen::Matrix3Xf blend_result(const Eigen::Matrix3Xf &Y_pred,
                                const Eigen::Matrix3Xf &Y_cpd,
                                const Eigen::VectorXi &is_occluded);

  // TODO instead of transforming the P matrix continually, we should just store P as an Eigen matrix
  // and not have to pass around intr in here
  Eigen::MatrixXf calcP(int N,
                        int M,
                        int D,
                        double sigma2,
                        const Eigen::Matrix3Xf &X,
                        const Eigen::Matrix3Xf &TY,
                        const Eigen::Matrix3Xf &Y_emit_prior);

  Eigen::Matrix3Xf cpd(const Eigen::Matrix3Xf &X,
                       const Eigen::Matrix3Xf &Y,
                       const Eigen::Matrix3Xf &Y_pred,
                       const cv::Mat &depth,
                       const cv::Mat &mask,
                       const Eigen::Matrix3f &intr);

  Eigen::Matrix3Xf cpd(const Eigen::Matrix3Xf &X,
                       const Eigen::Matrix3Xf &Y,
                       const cv::Mat &depth,
                       const cv::Mat &mask,
                       const Eigen::Matrix3f &intr);

  Eigen::Matrix3Xf cheng_cpd(const Eigen::Matrix3Xf &X,
                             const Eigen::Matrix3Xf &Y,
                             const cv::Mat &depth,
                             const cv::Mat &mask,
                             const Eigen::Matrix3f &intr);

  Eigen::Matrix3Xd predict(const Eigen::Matrix3Xd &P,
                           const smmap::AllGrippersSinglePoseDelta &q_dot,
                           const smmap::AllGrippersSinglePose &q_config,
                           int pred_choice);

  ros::NodeHandle nh;
  ros::NodeHandle ph;

  std::shared_ptr<smmap::ConstraintJacobianModel> model;
  std::shared_ptr<smmap::DiminishingRigidityModel> deformModel;

  // PastTemplateMatcher template_matcher;
  Eigen::Matrix3Xf original_template;
  Eigen::Matrix2Xi template_edges;

  Eigen::Vector3f last_lower_bounding_box;
  Eigen::Vector3f last_upper_bounding_box;
  int lle_neighbors;
  Eigen::MatrixXf m_lle;
  Eigen::MatrixXf L_lle;
  double tolerance;
  double alpha;
  double beta;
  double w;
  double initial_sigma_scale;
  double start_lambda;
  double annealing_factor;
  double k;
  int max_iterations;
  float kvis;
  float zeta;
  bool use_recovery;
  double last_sigma2;
  Eigen::MatrixXi gripper_idx;
  std::shared_ptr<const sdf_tools::SignedDistanceField> sdf_ptr;
  std::vector<bool> last_grasp_status;
  bool is_sim;
};

#endif
