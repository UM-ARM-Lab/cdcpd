#ifndef OBS_UTIL_H
#define OBS_UTIL_H

#include <Eigen/Dense>

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

typedef CGAL::Exact_predicates_inexact_constructions_kernel             K;
typedef K::FT                                                           FT;
typedef K::Point_3                                                      Point_3;
typedef K::Ray_3                                                        Ray_3;
typedef K::Vector_3                                                     Vector;
typedef CGAL::Surface_mesh<Point_3>                                     Mesh;
typedef boost::graph_traits<Mesh>::vertex_descriptor                    vertex_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor                      face_descriptor;
typedef CGAL::AABB_face_graph_triangle_primitive<Mesh>                  AABB_face_graph_primitive;
typedef CGAL::AABB_traits<K, AABB_face_graph_primitive>                 AABB_face_graph_traits;

struct obsParam
{
    Eigen::Matrix3Xf verts;
    Eigen::Matrix3Xf normals;
    Eigen::Matrix3Xf faces;
};

Mesh initObstacle(obsParam obs_param);
std::tuple<Eigen::Matrix3Xf, Eigen::Matrix3Xf>
    nearest_points_and_normal_help(const Eigen::Matrix3Xf& last_template,
                                   const Mesh& mesh,
                                   const Mesh::Property_map<vertex_descriptor, Vector>& vnormals);

#endif
