#include <iostream>

#include <ros/console.h>
#include <arc_utilities/eigen_ros_conversions.hpp>

#include "cdcpd/optimizer.h"

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

using Eigen::Matrix3Xf;
using Eigen::MatrixXf;
using Eigen::Matrix3Xd;
using Eigen::Matrix2Xi;
using Eigen::Vector3f;
using Eigen::VectorXf;

using std::min;
using std::max;
constexpr auto const LOGNAME = "optimizer";

// Builds the quadratic term ||point_a - point_b||^2
// This is equivalent to [point_a' point_b'] * Q * [point_a' point_b']'
// where Q is [ I, -I
//             -I,  I]
static GRBQuadExpr buildDifferencingQuadraticTerm(GRBVar *point_a, GRBVar *point_b, const size_t num_vars_per_point)
{
  GRBQuadExpr expr;

  // Build the main diagonal
  const std::vector<double> main_diag(num_vars_per_point, 1.0);
  expr.addTerms(main_diag.data(), point_a, point_a, (int) num_vars_per_point);
  expr.addTerms(main_diag.data(), point_b, point_b, (int) num_vars_per_point);

  // Build the off diagonal - use -2 instead of -1 because the off diagonal terms are the same
  const std::vector<double> off_diagonal(num_vars_per_point, -2.0);
  expr.addTerms(off_diagonal.data(), point_a, point_b, (int) num_vars_per_point);

  return expr;
}

static GRBEnv &getGRBEnv()
{
  static GRBEnv env;
  return env;
}

static Vector3f cgalVec2EigenVec(Vector cgal_v)
{
  return Vector3f(cgal_v[0], cgal_v[1], cgal_v[2]);
}

static Matrix3Xf force_pts(const Matrix3Xf &nearest,
                           const Matrix3Xf &normal,
                           const Matrix3Xf &Y)
{
  Matrix3Xf Y_force = Y;
  MatrixXf dist_to_obs = ((Y - nearest).array() * normal.array()).colwise().sum();
  for (int idx = 0; idx < Y.cols(); idx++)
  {
    if (dist_to_obs(0, idx) < 0)
    {
      Y_force.col(idx) = nearest.col(idx);
    }
  }
  return Y_force;
}

static Vector3f Pt3toVec(const Point_3 pt)
{
  return Vector3f(float(pt.x()), float(pt.y()), float(pt.z()));
}

PointsNormals
Optimizer::nearest_points_and_normal(const Matrix3Xf &last_template, Objects const &objects)
{
  for (auto const &object : objects)
  {
    // Meshes
    if (object.meshes.size() != object.mesh_poses.size())
    {
      ROS_ERROR_STREAM_THROTTLE_NAMED(1, LOGNAME,
                                      "got " << object.meshes.size() << " meshes but " << object.mesh_poses.size()
                                             << " mesh poses, they should match.");
    } else
    {
      for (auto mesh_idx = 0u; mesh_idx < object.meshes.size(); ++mesh_idx)
      {
        auto const mesh = object.meshes[mesh_idx];
        auto const mesh_pose = object.mesh_poses[mesh_idx];
        // apply the pose transform to all the vertices in the mesh
        shape_msgs::Mesh mesh_transformed;
        for (auto &vertex : mesh_transformed.vertices)
        {
          auto const transform = ConvertTo<Eigen::Isometry3d>(mesh_pose);
          auto const transformed_vertex = transform * ConvertTo<Eigen::Vector3d>(vertex);
          vertex = ConvertTo<geometry_msgs::Point>(transformed_vertex);
        }
        auto const points_normals = nearest_points_and_normal_mesh(last_template, mesh_transformed);
      }
    }

    // Planes
    if (object.planes.size() != object.plane_poses.size())
    {
      ROS_ERROR_STREAM_THROTTLE_NAMED(1, LOGNAME,
                                      "got " << object.planes.size() << " planes but " << object.plane_poses.size()
                                             << " plane poses, they should match.");
    } else
    {
      for (auto plane_idx = 0u; plane_idx < object.planes.size(); ++plane_idx)
      {
        auto plane = object.planes[plane_idx];
        auto const plane_pose = object.plane_poses[plane_idx];
        shape_msgs::Plane plane_transformed;
        auto const transform = ConvertTo<Eigen::Isometry3d>(plane_pose);
        // NOTE: We ignore the d coefficient here because the gazebo plugin I use to generate these always sets it to 0.
        // the d coefficient is redundant since the plane also has a pose
        Eigen::Vector3d plane_normal(plane.coef[0], plane.coef[1], plane.coef[2]);
        auto const transformed_coef = transform * plane_normal;
        plane.coef[0] = transformed_coef[0];
        plane.coef[1] = transformed_coef[1];
        plane.coef[2] = transformed_coef[2];
        auto const points_normals = nearest_points_and_normal_plane(last_template, plane_transformed);
      }
    }

    // Primitives
    if (object.primitives.size() != object.primitive_poses.size())
    {
      ROS_ERROR_STREAM_THROTTLE_NAMED(1, LOGNAME,
                                      "got " << object.primitives.size() << " primitives but "
                                             << object.primitive_poses.size()
                                             << " primitive poses, they should match.");
    } else
    {
      for (auto primitive_idx = 0u; primitive_idx < object.primitives.size(); ++primitive_idx)
      {
        auto const primitive = object.primitives[primitive_idx];
        auto const primitive_pose = object.primitive_poses[primitive_idx];

        switch (primitive.type)
        {
          case shape_msgs::SolidPrimitive::BOX:
          {
            auto const points_normals = nearest_points_and_normal_box(last_template, primitive, primitive_pose);
            break;
          }
          case shape_msgs::SolidPrimitive::CYLINDER:
          {
            auto const points_normals = nearest_points_and_normal_cylinder(last_template, primitive, primitive_pose);
            break;
          }
          case shape_msgs::SolidPrimitive::SPHERE:
          {
            auto const points_normals = nearest_points_and_normal_sphere(last_template, primitive, primitive_pose);
            break;
          }
          default:
            ROS_ERROR_STREAM_THROTTLE_NAMED(1, LOGNAME, "Unsupported shape type " << primitive.type);
            break;
        }
      }
    }
  }

  Matrix3Xf nearestPts(3, last_template.cols());
  Matrix3Xf normalVecs(3, last_template.cols());
  return {nearestPts, normalVecs};
}

PointsNormals
Optimizer::nearest_points_and_normal_box(const Matrix3Xf &last_template, shape_msgs::SolidPrimitive const &box,
                                         geometry_msgs::Pose const &pose)
{
  auto const position = ConvertTo<Vector3f>(pose.position);
  auto const orientation = ConvertTo<Eigen::Quaternionf>(pose.orientation).toRotationMatrix();
  auto const box_x = box.dimensions[shape_msgs::SOlidPrimitive::BOX_X];
  auto const box_y = box.dimensions[shape_msgs::SOlidPrimitive::BOX_Y];
  auto const box_z = box.dimensions[shape_msgs::SOlidPrimitive::BOX_Z];

  Matrix3Xf nearestPts(3, last_template.cols());
  Matrix3Xf normalVecs(3, last_template.cols());

  Matrix4Xf homo_last_template = last_template.colwise().homogeneous();
  Matrix4Xf transform(4,4);
  transform<3,3>(0,0) = orientation;
  trnasform(3,3) = 1.0;
  transform<3,1>(0,3) = position;

  Matrix4Xf tf_inv = transform.inverse();

  Matrix4Xf pts_box = tf_inv * homo_last_template.colwise();

  Vector3f box_x_dir = orientaion.col(0);
  Vector3f box_y_dir = orientaion.col(1);
  Vector3f box_z_dir = orientaion.col(2);

  for (int i = 0; i < last_template.cols(); i++)
  {
    float x = pts_box(0, i);
    float y = pts_box(1, i);
    float z = pts_box(2, i);

    if (x > box_x/2 || x < -box_x/2 || y > box_y/2 || y < -box_y/2 || z > box_z/2 || z < -box_z/2)
    // If the point is not inside the box
    {
      int c_x, c_y, c_z; // coeffient in front of box_x_dir
      c_x = (x > box_x/2) ? 1 : ((x < -box_x/2) ? -1 : 0);
      c_y = (y > box_y/2) ? 1 : ((y < -box_y/2) ? -1 : 0);
      c_z = (z > box_z/2) ? 1 : ((z < -box_z/2) ? -1 : 0);
      normalVecs.col(i) = c_x * box_x_dir + \
			  c_y * box_y_dir + \
			  c_z * box_z_dir;
      auto const nearestPt_box_frame = c_x * box_x/2 + (1-abs(c_x)) * x + \
                                       c_y * box_y/2 + (1-abs(c_y)) * y + \
                                       c_z * box_z/2 + (1-abs(c_z)) * z;
      nearestPts.col(i) = (transform * nearestPt_box_frame.homogeneous()).head(3);
    } else
    {
      float ratio_x = 2*x/box_x; float ratio_y = 2*y/box_y; float ratio_z = 2*z/box_z;
      if (abs(ratio_x) > abs(ratio_y) && abs(ratio_x) > abs(ratio_z))
      {
        // AFTER DINNER
      }
    }
  }

  return {nearestPts, normalVecs};
}


PointsNormals
Optimizer::nearest_points_and_normal_sphere(const Matrix3Xf &last_template, shape_msgs::SolidPrimitive const &sphere,
                                            geometry_msgs::Pose const &pose)
{
  Matrix3Xf nearestPts(3, last_template.cols());
  Matrix3Xf normalVecs(3, last_template.cols());
  return {nearestPts, normalVecs};
}


PointsNormals
Optimizer::nearest_points_and_normal_plane(const Matrix3Xf &last_template, shape_msgs::Plane const &plane)
{
  Matrix3Xf nearestPts(3, last_template.cols());
  Matrix3Xf normalVecs(3, last_template.cols());
  return {nearestPts, normalVecs};
}

PointsNormals
Optimizer::nearest_points_and_normal_cylinder(const Matrix3Xf &last_template,
                                              shape_msgs::SolidPrimitive const &cylinder,
                                              geometry_msgs::Pose const &pose)
{
  auto const position = ConvertTo<Vector3f>(pose.position);
  // NOTE: Yixuan, should orientation be roll, pitch, yaw here?
  // Answer: As what I can recall, the orientation is the unit vector along center axis
  auto const orientation = ConvertTo<Eigen::Quaternionf>(pose.orientation).toRotationMatrix().eulerAngles(0, 1, 2);
  auto const radius = cylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
  auto const height = cylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];

  // find of the nearest points and corresponding normal vector on the cylinder
  Matrix3Xf nearestPts(3, last_template.cols());
  Matrix3Xf normalVecs(3, last_template.cols());
  for (int i = 0; i < last_template.cols(); i++)
  {
    Vector3f pt;
    pt << last_template.col(i);
    Vector3f unitVecH = orientation / orientation.norm();
    Vector3f unitVecR = (pt - position) - ((pt - position).transpose() * unitVecH) * unitVecH;
    unitVecR = unitVecR / unitVecR.norm();
    float h = unitVecH.transpose() * (pt - position);
    float r = unitVecR.transpose() * (pt - position);
    Vector3f nearestPt;
    Vector3f normalVec;
    if (h > height / 2 && r >= radius)
    {
      nearestPt = unitVecR * radius + unitVecH * height / 2 + position;
      normalVec = unitVecR + unitVecH;
    } else if (h < -height / 2 && r >= radius)
    {
      nearestPt = unitVecR * radius - unitVecH * height / 2 + position;
      normalVec = unitVecR - orientation / orientation.norm();
    } else if (h > height / 2 && r < radius)
    {
      nearestPt = r * unitVecR + height / 2 * unitVecH + position;
      normalVec = unitVecH;
    } else if (h < -height / 2 && r < radius)
    {
      nearestPt = r * unitVecR - height / 2 * unitVecH + position;
      normalVec = -unitVecH;
    } else if (h <= height / 2 && h >= -height / 2 && r < radius)
    {
      if (height / 2 - h < radius - r)
      {
        nearestPt = r * unitVecR + height / 2 * unitVecH + position;
        normalVec = unitVecH;
      } else if (h + height / 2 < radius - r)
      {
        nearestPt = r * unitVecR + height / 2 * unitVecH + position;
        normalVec = -unitVecH;
      } else
      {
        nearestPt = radius * unitVecR + h * unitVecH + position;
        normalVec = unitVecR;
      }
    } else if (h <= height / 2 && h >= -height / 2 && r >= radius)
    {
      nearestPt = radius * unitVecR + h * unitVecH + position;
      normalVec = unitVecR;
    }
    normalVec = normalVec / normalVec.norm();
    for (int j = 0; j < 3; ++j)
    {
      nearestPts(j, i) = nearestPt(j);
      normalVecs(j, i) = normalVec(j);
    }
  }
  return {nearestPts, normalVecs};
}


PointsNormals
Optimizer::nearest_points_and_normal_mesh(const Matrix3Xf &last_template,
                                          shape_msgs::Mesh const &shapes_mesh)
{
  auto mesh = shapes_mesh_to_cgal_mesh(shapes_mesh);
  auto const fnormals = mesh.add_property_map<face_descriptor, Vector>("f:normals", CGAL::NULL_VECTOR).first;
  auto const vnormals = mesh.add_property_map<vertex_descriptor, Vector>("v:normals", CGAL::NULL_VECTOR).first;

  auto const mesh_map = CGAL::Polygon_mesh_processing::parameters::vertex_point_map(mesh.points()).geom_traits(K());
  CGAL::Polygon_mesh_processing::compute_normals(mesh, vnormals, fnormals, mesh_map);

  Matrix3Xf nearestPts(3, last_template.cols());
  Matrix3Xf normalVecs(3, last_template.cols());

  for (int pt_ind = 0; pt_ind < last_template.cols(); pt_ind++)
  {
    Point_3 pt(last_template(0, pt_ind),
               last_template(1, pt_ind),
               last_template(2, pt_ind));
    Ray_3 ray(pt, pt);
    Face_location query_location = PMP::locate(pt, mesh);
    // NOTE: this might be faster
    // Face_location query_location = PMP::locate_with_AABB_tree(ray, tree, mesh);
    // Point_3 nearestPt = PMP::construct_point(query_location, mesh);
    // nearestPts.col(pt_ind) = Pt3toVec(nearestPt);

    double w[3];
    for (int i = 0; i < 3; i++)
    {
      w[i] = query_location.second[i];
    }

    if (isnan(w[0]) || isnan(w[1]) || isnan(w[2]))
    {
      w[0] = w[1] = w[2] = 1.0 / 3;
    }

    MatrixXf verts_of_face(3, 3);
    verts_of_face.col(0) = Pt3toVec(mesh.point(source(halfedge(query_location.first, mesh), mesh)));
    verts_of_face.col(1) = Pt3toVec(mesh.point(target(halfedge(query_location.first, mesh), mesh)));
    verts_of_face.col(2) = Pt3toVec(mesh.point(target(next(halfedge(query_location.first, mesh), mesh), mesh)));
    nearestPts.col(pt_ind) = verts_of_face.col(0) * w[0] + verts_of_face.col(1) * w[1] + verts_of_face.col(2) * w[2];

    Vector3f normalVec(0.0, 0.0, 0.0);
    normalVec = cgalVec2EigenVec(
        vnormals[source(halfedge(query_location.first, mesh), mesh)] * w[0] +
        vnormals[target(halfedge(query_location.first, mesh), mesh)] * w[1] +
        vnormals[target(next(halfedge(query_location.first, mesh), mesh), mesh)] * w[2]
    );

    normalVecs.col(pt_ind) = normalVec;
  }
  return {nearestPts, normalVecs};
}

PointsNormals
nearest_points_line_segments(const Matrix3Xf &last_template, const Matrix2Xi &E)
{
  // find the nearest points on the line segments
  // refer to the website https://math.stackexchange.com/questions/846054/closest-points-on-two-line-segments
  MatrixXf startPts(4, E.cols() *
                       E.cols()); // Matrix: 3 * E^2: startPts.col(E*cols()*i + j) is the nearest point on edge i w.r.t. j
  MatrixXf endPts(4, E.cols() *
                     E.cols()); // Matrix: 3 * E^2: endPts.col(E*cols()*i + j) is the nearest point on edge j w.r.t. i
  for (int i = 0; i < E.cols(); ++i)
  {
    Vector3f P1 = last_template.col(E(0, i));
    Vector3f P2 = last_template.col(E(1, i));


    for (int j = 0; j < E.cols(); ++j)
    {
      Vector3f P3 = last_template.col(E(0, j));
      Vector3f P4 = last_template.col(E(1, j));


      float R21 = (P2 - P1).squaredNorm();
      float R22 = (P4 - P3).squaredNorm();
      float D4321 = (P4 - P3).dot(P2 - P1);
      float D3121 = (P3 - P1).dot(P2 - P1);
      float D4331 = (P4 - P3).dot(P3 - P1);


      float s;
      float t;

      if (R21 * R22 - D4321 * D4321 != 0)
      {
        s = min(max((-D4321 * D4331 + D3121 * R22) / (R21 * R22 - D4321 * D4321), 0.0f), 1.0f);
        t = min(max((D4321 * D3121 - D4331 * R21) / (R21 * R22 - D4321 * D4321), 0.0f), 1.0f);
      } else
      {
        // means P1 P2 P3 P4 are on the same line
        float P13 = (P3 - P1).squaredNorm();
        s = 0;
        t = 0;
        float P14 = (P4 - P1).squaredNorm();
        if (P14 < P13)
        {
          s = 0;
          t = 1;
        }
        float P23 = (P3 - P2).squaredNorm();
        if (P23 < P14 && P23 < P13)
        {
          s = 1;
          t = 0;
        }
        float P24 = (P4 - P2).squaredNorm();
        if (P24 < P23 && P24 < P14 && P24 < P13)
        {
          s = 1;
          t = 1;
        }
      }


      for (int dim = 0; dim < 3; ++dim)
      {
        startPts(dim, E.cols() * i + j) = (1 - s) * P1(dim) + s * P2(dim);
        endPts(dim, E.cols() * i + j) = (1 - t) * P3(dim) + t * P4(dim);
      }
      startPts(3, E.cols() * i + j) = s;
      endPts(3, E.cols() * i + j) = t;
    }
  }
  return {startPts, endPts};
}

Optimizer::Optimizer(const Eigen::Matrix3Xf _init_temp, const Eigen::Matrix3Xf _last_temp, const float _stretch_lambda)
    : initial_template(_init_temp),
      last_template(_last_temp),
      stretch_lambda(_stretch_lambda)
{
  // NOTE: simplifying the mesh?
  // typedef boost::property_map<Mesh, CGAL::edge_is_feature_t>::type EIFMap;
  // EIFMap eif = get(CGAL::edge_is_feature, mesh);
  // PMP::detect_sharp_edges(mesh, 30, eif);
  // PMP::smooth_mesh(mesh, PMP::parameters::number_of_iterations(3)
  //                                      .use_safety_constraints(false) // authorize all moves
  //                                      .edge_is_constrained_map(eif));
  // CGAL::Subdivision_method_3::CatmullClark_subdivision(mesh, CGAL::parameters::number_of_iterations(4));

  // NOTE: this might be faster
  // PMP::build_AABB_tree(mesh, tree);
}

Matrix3Xf
Optimizer::operator()(const Matrix3Xf &Y,
                      const Matrix2Xi &E,
                      const std::vector<FixedPoint> &fixed_points,
                      Objects const &objects,
                      const bool self_intersection,
                      const bool interaction_constrain)
{
  // Y: Y^t in Eq. (21)
  // E: E in Eq. (21)
  Matrix3Xf Y_opt(Y.rows(), Y.cols());
  GRBVar *vars = nullptr;
  const ssize_t num_vectors = Y.cols();
  const ssize_t num_vars = 3 * num_vectors;

  GRBEnv &env = getGRBEnv();

  // Disables logging to file and logging to console (with a 0 as the value of the flag)
  env.set(GRB_IntParam_OutputFlag, 0);
  GRBModel model(env);
  model.set("ScaleFlag", "0");
  // model.set("DualReductions", 0);
  model.set("FeasibilityTol", "0.01");
  // model.set("OutputFlag", "1");

  // Add the vars to the model
  {
    // Note that variable bound is important, without a bound, Gurobi defaults to 0, which is clearly unwanted
    const std::vector<double> lb(num_vars, -GRB_INFINITY);
    const std::vector<double> ub(num_vars, GRB_INFINITY);
    vars = model.addVars(lb.data(), ub.data(), nullptr, nullptr, nullptr, (int) num_vars);
    model.update();
  }

  // Add the edge constraints
  {
    for (ssize_t i = 0; i < E.cols(); ++i)
    {
      model.addQConstr(
          buildDifferencingQuadraticTerm(&vars[E(0, i) * 3], &vars[E(1, i) * 3], 3),
          GRB_LESS_EQUAL,
          stretch_lambda * stretch_lambda *
          (initial_template.col(E(0, i)) - initial_template.col(E(1, i))).squaredNorm(),
          "upper_edge_" + std::to_string(E(0, i)) + "_to_" + std::to_string(E(1, i)));
    }
    model.update();
  }

  // Add interaction constraints
  if (interaction_constrain)
  {
    ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME, "added interaction constraint");
    auto const &[nearestPts, normalVecs] = nearest_points_and_normal(last_template, objects);

    for (ssize_t i = 0; i < num_vectors; ++i)
    {
      model.addConstr(
          (vars[i * 3 + 0] - nearestPts(0, i)) * normalVecs(0, i) +
          (vars[i * 3 + 1] - nearestPts(1, i)) * normalVecs(1, i) +
          (vars[i * 3 + 2] - nearestPts(2, i)) * normalVecs(2, i) >= 0.0,
          "interaction constrain for point " + std::to_string(i));
    }
  }

  if (self_intersection)
  {
    ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME, "adding self intersection constrain");
    auto[startPts, endPts] = nearest_points_line_segments(last_template, E);
    for (int row = 0; row < E.cols(); ++row)
    {
      Vector3f P1 = last_template.col(E(0, row));
      Vector3f P2 = last_template.col(E(1, row));
      for (int col = 0; col < E.cols(); ++col)
      {
        float s = startPts(3, row * E.cols() + col);
        float t = endPts(3, row * E.cols() + col);
        Vector3f P3 = last_template.col(E(0, col));
        Vector3f P4 = last_template.col(E(1, col));
        float l = (endPts.col(row * E.cols() + col).topRows(3) -
                   startPts.col(row * E.cols() + col).topRows(3)).norm();
        if (!P1.isApprox(P3) && !P1.isApprox(P4) && !P2.isApprox(P3) && !P2.isApprox(P4) && l <= 0.02)
        {
          // model.addConstr((vars[E(0, col)*3 + 0] - startPts(0, row*E.cols() + col))*(endPts(0, row*E.cols() + col) - startPts(0, row*E.cols() + col)) +
          //                 (vars[E(0, col)*3 + 1] - startPts(1, row*E.cols() + col))*(endPts(1, row*E.cols() + col) - startPts(1, row*E.cols() + col)) +
          //                 (vars[E(0, col)*3 + 2] - startPts(2, row*E.cols() + col))*(endPts(2, row*E.cols() + col) - startPts(2, row*E.cols() + col)) >= 0);
          // model.addConstr((vars[E(1, col)*3 + 0] - startPts(0, row*E.cols() + col))*(endPts(0, row*E.cols() + col) - startPts(0, row*E.cols() + col)) +
          //                 (vars[E(1, col)*3 + 1] - startPts(1, row*E.cols() + col))*(endPts(1, row*E.cols() + col) - startPts(1, row*E.cols() + col)) +
          //                 (vars[E(1, col)*3 + 2] - startPts(2, row*E.cols() + col))*(endPts(2, row*E.cols() + col) - startPts(2, row*E.cols() + col)) >= 0);
          model.addConstr(((vars[E(0, col) * 3 + 0] * (1 - t) + vars[E(1, col) * 3 + 0] * t) -
                           (vars[E(0, row) * 3 + 0] * (1 - s) + vars[E(1, row) * 3 + 0] * s))
                          * (endPts(0, row * E.cols() + col) - startPts(0, row * E.cols() + col)) +
                          ((vars[E(0, col) * 3 + 1] * (1 - t) + vars[E(1, col) * 3 + 1] * t) -
                           (vars[E(0, row) * 3 + 1] * (1 - s) + vars[E(1, row) * 3 + 1] * s))
                          * (endPts(1, row * E.cols() + col) - startPts(1, row * E.cols() + col)) +
                          ((vars[E(0, col) * 3 + 2] * (1 - t) + vars[E(1, col) * 3 + 2] * t) -
                           (vars[E(0, row) * 3 + 2] * (1 - s) + vars[E(1, row) * 3 + 2] * s))
                          * (endPts(2, row * E.cols() + col) - startPts(2, row * E.cols() + col)) >= 0.01 * l);
        }
      }
    }
  }

  Matrix3Xd Y_copy = Y.cast<double>(); // TODO is this exactly what we want?

  // Next, add the fixed point constraints that we might have.
  // TODO make this more
  // First, make sure that the constraints can be satisfied
  GRBQuadExpr gripper_objective_fn(0);
  if (gripper_constraints_satisfiable(fixed_points))
  {
    // If that's possible, we'll require that all constraints are equal
    for (const auto &fixed_point : fixed_points)
    {
      model.addConstr(vars[3 * fixed_point.template_index + 0], GRB_EQUAL, fixed_point.position(0), "fixed_point");
      model.addConstr(vars[3 * fixed_point.template_index + 1], GRB_EQUAL, fixed_point.position(1), "fixed_point");
      model.addConstr(vars[3 * fixed_point.template_index + 2], GRB_EQUAL, fixed_point.position(2), "fixed_point");
    }
  } else
  {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Gripper constraint cannot be satisfied.");
    for (const auto &fixed_point : fixed_points)
    {
      const auto expr0 = vars[fixed_point.template_index + 0] - Y_copy(0, fixed_point.template_index);
      const auto expr1 = vars[fixed_point.template_index + 1] - Y_copy(1, fixed_point.template_index);
      const auto expr2 = vars[fixed_point.template_index + 2] - Y_copy(2, fixed_point.template_index);
      gripper_objective_fn += 100.0 * (expr0 * expr0 + expr1 * expr1 + expr2 * expr2);
    }
  }

  // Build the objective function
  {
    GRBQuadExpr objective_fn = gripper_objective_fn;
    for (ssize_t i = 0; i < num_vectors; ++i)
    {
      const auto expr0 = vars[i * 3 + 0] - Y_copy(0, i);
      const auto expr1 = vars[i * 3 + 1] - Y_copy(1, i);
      const auto expr2 = vars[i * 3 + 2] - Y_copy(2, i);
      objective_fn += expr0 * expr0;
      objective_fn += expr1 * expr1;
      objective_fn += expr2 * expr2;
    }
    model.setObjective(objective_fn, GRB_MINIMIZE);
    model.update();
  }

  // Find the optimal solution, and extract it
  {
    model.optimize();
    if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL || model.get(GRB_IntAttr_Status) == GRB_SUBOPTIMAL)
      // || model.get(GRB_IntAttr_Status) == GRB_SUBOPTIMAL || model.get(GRB_IntAttr_Status) == GRB_NUMERIC || modelGRB_INF_OR_UNBD)
    {
      // std::cout << "Y" << std::endl;
      // std::cout << Y << std::endl;
      for (ssize_t i = 0; i < num_vectors; i++)
      {
        Y_opt(0, i) = vars[i * 3 + 0].get(GRB_DoubleAttr_X);
        Y_opt(1, i) = vars[i * 3 + 1].get(GRB_DoubleAttr_X);
        Y_opt(2, i) = vars[i * 3 + 2].get(GRB_DoubleAttr_X);
      }
    } else
    {
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "Status: " << model.get(GRB_IntAttr_Status));
      exit(-1);
    }
  }

  delete[] vars;
  return Y_opt;
}

bool Optimizer::gripper_constraints_satisfiable(const std::vector<FixedPoint> &fixed_points) const
{
  for (auto const &p1 : fixed_points)
  {
    for (auto const &p2 : fixed_points)
    {
      float const current_distance = (p1.position - p2.position).squaredNorm();
      float const original_distance = (initial_template.col(p1.template_index) -
                                       initial_template.col(p2.template_index)).squaredNorm();

      if (current_distance > original_distance * stretch_lambda * stretch_lambda)
      {
        return false;
      }
    }
  }
  return true;
}



