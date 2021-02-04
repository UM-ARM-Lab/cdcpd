#include "cdcpd/obs_util.h"

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
using Eigen::Vector3f;

static Vector3f Pt3toVec(const Point_3 pt)
{
  return Vector3f(float(pt.x()), float(pt.y()), float(pt.z()));
}

static Vector3f cgalVec2EigenVec(Vector cgal_v)
{
  return Vector3f(cgal_v[0], cgal_v[1], cgal_v[2]);
}


Mesh shapes_mesh_to_cgal_mesh(shapes::Mesh const &input_mesh)
{
  Mesh output_mesh;
  // first copy all the vertices
  // create a mapping between vertex indices in the input mesh and the output mesh
  std::unordered_map<unsigned int, CGAL::SM_Vertex_index> vertex_index_map;
  for (auto vertex_idx = 0u; vertex_idx < input_mesh.vertex_count; ++vertex_idx)
  {
    auto const vertex = Point_3(input_mesh.vertices[3 * vertex_idx],
                                input_mesh.vertices[3 * vertex_idx + 1],
                                input_mesh.vertices[3 * vertex_idx + 2]);
    auto const output_v_index = output_mesh.add_vertex(vertex);
    vertex_index_map.emplace(vertex_idx, output_v_index);
  }

  // now copy the faces
  for (auto face_idx = 0u; face_idx < input_mesh.triangle_count; ++face_idx)
  {
    auto const input_v_index1 = input_mesh.triangles[3 * face_idx];
    auto const input_v_index2 = input_mesh.triangles[3 * face_idx + 1];
    auto const input_v_index3 = input_mesh.triangles[3 * face_idx + 2];
    CGAL::SM_Vertex_index output_v_index1 = vertex_index_map[input_v_index1];
    CGAL::SM_Vertex_index output_v_index2 = vertex_index_map[input_v_index2];
    CGAL::SM_Vertex_index output_v_index3 = vertex_index_map[input_v_index3];
    output_mesh.add_face(output_v_index1, output_v_index2, output_v_index3);
  }
  return output_mesh;
}
