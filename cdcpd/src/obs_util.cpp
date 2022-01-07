#include "cdcpd/obs_util.h"

namespace PMP = CGAL::Polygon_mesh_processing;

typedef PMP::Face_location<Mesh, FT> Face_location;

using Eigen::Matrix3Xf;
using Eigen::MatrixXf;
using Eigen::Vector3f;

Point_3 pointMsgtoPoint(const geometry_msgs::Point pt)
{
  return Point_3(pt.x, pt.y, pt.z);
}

static Vector3f cgalVec2EigenVec(Vector cgal_v)
{
  return Vector3f(cgal_v[0], cgal_v[1], cgal_v[2]);
}


Mesh shapes_mesh_to_cgal_mesh(shape_msgs::Mesh const &input_mesh)
{
  Mesh output_mesh;
  // first copy all the vertices
  // create a mapping between vertex indices in the input mesh and the output mesh
  std::unordered_map<uint32_t, CGAL::SM_Vertex_index> vertex_index_map;
  for (auto vertex_idx = 0u; vertex_idx < input_mesh.vertices.size(); ++vertex_idx)
  {
    auto const vertex = pointMsgtoPoint(input_mesh.vertices[vertex_idx]);
    auto const output_v_index = output_mesh.add_vertex(vertex);
    vertex_index_map.emplace(vertex_idx, output_v_index);
  }

  // now copy the faces
  for (auto triangle : input_mesh.triangles)
  {
    auto const input_v_index0 = triangle.vertex_indices[0];
    auto const input_v_index1 = triangle.vertex_indices[1];
    auto const input_v_index2 = triangle.vertex_indices[2];
    CGAL::SM_Vertex_index output_v_index0 = vertex_index_map[input_v_index0];
    CGAL::SM_Vertex_index output_v_index1 = vertex_index_map[input_v_index1];
    CGAL::SM_Vertex_index output_v_index2 = vertex_index_map[input_v_index2];
    output_mesh.add_face(output_v_index0, output_v_index1, output_v_index2);
  }
  return output_mesh;
}
