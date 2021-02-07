#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <algorithm>

#include <Eigen/Dense>
#include <gurobi_c++.h>
#include <moveit_msgs/CollisionObject.h>

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

#include "obs_util.h"

struct FixedPoint
{
  Eigen::Vector3f position;
  int template_index;
};

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

using Objects = std::vector<moveit_msgs::CollisionObject>;
using Points = Eigen::Matrix3Xf;
using Normals = Eigen::Matrix3Xf;
using Point = Eigen::Vector3f;
using Normal = Eigen::Vector3f;
using PointNormal = std::tuple<Point, Normal>; // Hyperplane, for enforcing that tracked points aren't inside obstacles
struct ObstacleConstraint
{
  unsigned int point_idx;
  Eigen::Vector3f point;
  Eigen::Vector3f normal;
};
using InteractionConstraints = std::vector<ObstacleConstraint>;

class Optimizer
{
 public:
  Optimizer(const Eigen::Matrix3Xf init_temp, const Eigen::Matrix3Xf last_temp, float stretch_lambda, float obstacle_cost_weight);

  [[nodiscard]] Eigen::Matrix3Xf operator()(const Eigen::Matrix3Xf &Y,
                                            const Eigen::Matrix2Xi &E,
                                            const std::vector<FixedPoint> &fixed_points,
                                            InteractionConstraints const &points_normals);

  std::tuple<Points, Normals> test_box(const Eigen::Matrix3Xf &last_template,
                                       shape_msgs::SolidPrimitive const &box,
                                       geometry_msgs::Pose const &pose);

 private:
  [[nodiscard]] bool gripper_constraints_satisfiable(const std::vector<FixedPoint> &fixed_points) const;

  [[nodiscard]] std::tuple<Points, Normals> nearest_points_and_normal_box(const Eigen::Matrix3Xf &last_template,
                                                                          shape_msgs::SolidPrimitive const &box,
                                                                          geometry_msgs::Pose const &pose);

  [[nodiscard]]  std::tuple<Points, Normals> nearest_points_and_normal_sphere(const Eigen::Matrix3Xf &last_template,
                                                                              shape_msgs::SolidPrimitive const &sphere,
                                                                              geometry_msgs::Pose const &pose);

  [[nodiscard]]  std::tuple<Points, Normals> nearest_points_and_normal_plane(const Eigen::Matrix3Xf &last_template,
                                                                             shape_msgs::Plane const &plane);

  [[nodiscard]]  std::tuple<Points, Normals> nearest_points_and_normal_cylinder(const Eigen::Matrix3Xf &last_template,
                                                                                shape_msgs::SolidPrimitive const &cylinder,
                                                                                geometry_msgs::Pose const &pose);

  [[nodiscard]]  std::tuple<Points, Normals>
  nearest_points_and_normal_mesh(const Eigen::Matrix3Xf &last_template,
                                 shape_msgs::Mesh const &shapes_mesh);

  [[nodiscard]]  std::tuple<Points, Normals>
  nearest_points_and_normal(const Eigen::Matrix3Xf &last_template, Objects const &objects);

  Eigen::Matrix3Xf initial_template;
  Eigen::Matrix3Xf last_template;
  float stretch_lambda;
  float obstacle_cost_weight;
};

#endif
