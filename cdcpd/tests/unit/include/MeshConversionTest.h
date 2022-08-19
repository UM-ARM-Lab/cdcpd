#include <cdcpd/obs_util.h>

#include <arc_utilities/geometry_msgs_builders.hpp>
#include <arc_utilities/shape_msgs_builders.hpp>
#include <gtest/gtest.h>

#include <iostream>

void testPointsEqual(K::Point_3 const& lhs, K::Point_3 const& rhs)
{
  for (std::size_t i = 0; i < 3U; ++i)
  {
    EXPECT_EQ(lhs.cartesian(i), rhs.cartesian(i));
  }
}

TEST(MeshConversionTest, shapesMeshToCgalMesh)
{
  std::size_t const num_points = 4U;
  K::Point_3 const expected_points[num_points] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
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
  
  Mesh const out_mesh = shapes_mesh_to_cgal_mesh(input_mesh);

  EXPECT_EQ(out_mesh.num_faces(), 4U);
  EXPECT_EQ(out_mesh.num_edges(), 6U);

  Mesh::Property_map<vertex_descriptor, K::Point_3> point_locations = out_mesh.points();
  std::size_t point_idx = 0;
  for (vertex_descriptor vertex : out_mesh.vertices())
  {
    testPointsEqual(point_locations[vertex], expected_points[point_idx]);
    ++point_idx;
  }
}