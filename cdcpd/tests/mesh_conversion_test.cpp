#include <cdcpd/obs_util.h>

#include <arc_utilities/geometry_msgs_builders.hpp>
#include <arc_utilities/shape_msgs_builders.hpp>

int main()
{
  namespace a = arc_utilities::rmb;

  shape_msgs::Mesh input_mesh;
  input_mesh.vertices = {
      a::MakePoint(0, 0, 0),
      a::MakePoint(1, 0, 0),
      a::MakePoint(0, 1, 0),
      a::MakePoint(0, 0, 1)
  };
  input_mesh.triangles = {
      a::MakeMeshTriangle(0, 1, 2),
      a::MakeMeshTriangle(0, 3, 1),
      a::MakeMeshTriangle(0, 2, 3),
      a::MakeMeshTriangle(1, 3, 2)
  };
  auto const out_mesh = shapes_mesh_to_cgal_mesh(input_mesh);
  CGAL::draw(out_mesh);
}