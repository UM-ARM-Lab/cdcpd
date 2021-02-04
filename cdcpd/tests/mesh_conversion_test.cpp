#include <cdcpd/obs_util.h>

int main()
{
//  Mesh mesh;
//  auto const v0 = mesh.add_vertex({0, 0, 0});
//  auto const v1 = mesh.add_vertex({1, 0, 0});
//  auto const v2 = mesh.add_vertex({0, 1, 0});
//  auto const v3 = mesh.add_vertex({0, 0, 1});
//  mesh.add_face(v0, v1, v2);
//  mesh.add_face(v0, v3, v1);
//  mesh.add_face(v0, v2, v3);
//  mesh.add_face(v1, v3, v2);

  std::vector<double> vertices{
      0, 0, 0,
      1, 0, 0,
      0, 1, 0,
      0, 0, 1
  };
  std::vector<unsigned int> triangles{
      0, 1, 2,
      0, 3, 1,
      0, 2, 3,
      1, 3, 2
  };
  shapes::Mesh input_mesh;
  input_mesh.vertex_count = 4;
  input_mesh.triangle_count = 4;
  input_mesh.vertices = vertices.data();
  input_mesh.triangles = triangles.data();
  auto const out_mesh = shapes_mesh_to_cgal_mesh(input_mesh);
//  CGAL::draw(out_mesh);
}