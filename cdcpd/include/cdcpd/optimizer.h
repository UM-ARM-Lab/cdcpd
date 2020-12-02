#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#define CGAL_PMP_USE_CERES_SOLVER
#define CGAL_USE_BASIC_VIEWER

#include <algorithm>

#include <Eigen/Dense>
#include <gurobi_c++.h>
#include "cdcpd.h"
#include "obs_util.h"

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
#include <CGAL/AABB_tree.h>

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


////////////////////////////////////////////////////////////////
// Internally used static objects
////////////////////////////////////////////////////////////////

// std::tuple<Eigen::Matrix3Xf, Eigen::Matrix3Xf>
// nearest_points_and_normal(const Eigen::Matrix3Xf& last_template);

// std::tuple<Eigen::Matrix3Xf, Eigen::Matrix3Xf>
//    nearest_points_line_segments(const Eigen::Matrix3Xf& last_template, const Eigen::Matrix2Xi& E);

void Wsolver(const Eigen::MatrixXf &P, const Eigen::Matrix3Xf &X, const Eigen::Matrix3Xf &Y, const Eigen::MatrixXf &G,
             const Eigen::MatrixXf &L, const double sigma2, const double alpha, const double lambda,
             Eigen::MatrixXf &W);


class Optimizer
{
 public:
  Optimizer(const Eigen::Matrix3Xf _init_temp, const Eigen::Matrix3Xf _last_temp, const float _stretch_lambda,
            const obsParam &obstacle_param);

  Optimizer(const Eigen::Matrix3Xf _init_temp, const Eigen::Matrix3Xf _last_temp, const float _stretch_lambda,
            const std::vector<float> cylinder_data);

  Optimizer(const Eigen::Matrix3Xf _init_temp, const Eigen::Matrix3Xf _last_temp, const float _stretch_lambda);

  Eigen::Matrix3Xf operator()(const Eigen::Matrix3Xf &Y,
                              const Eigen::Matrix2Xi &E,
                              const std::vector<CDCPD::FixedPoint> &fixed_points,
                              const bool self_intersection = true,
                              const bool interation_constrain = true);

 private:
  bool all_constraints_satisfiable(const std::vector<CDCPD::FixedPoint> &fixed_points) const;

  std::tuple<Eigen::Matrix3Xf, Eigen::Matrix3Xf> nearest_points_and_normal(const Eigen::Matrix3Xf &last_template);

  std::tuple<Eigen::Matrix3Xf, Eigen::Matrix3Xf> nearest_points_and_normal_cyl(const Eigen::Matrix3Xf &last_template);

  Eigen::Matrix3Xf initial_template;
  Eigen::Matrix3Xf last_template;
  float stretch_lambda;
  // const Eigen::Matrix3Xf obs_mesh;
  // const Eigen::Matrix3Xf obs_normal;
  Mesh::Property_map <face_descriptor, Vector> fnormals;
  Mesh::Property_map <vertex_descriptor, Vector> vnormals;
  Mesh mesh;
  CGAL::AABB_tree<AABB_face_graph_traits> tree;
  Eigen::Vector3f cylinder_orien;
  Eigen::Vector3f cylinder_center;
  float cylinder_radius;
  float cylinder_height;
  bool is_shape_comp;
};

#endif
