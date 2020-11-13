#include "cdcpd/obs_util.h"

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

namespace PMP = CGAL::Polygon_mesh_processing;

typedef PMP::Face_location<Mesh, FT>                                    Face_location;

using Eigen::Matrix3Xf;
using Eigen::MatrixXf;
using Eigen::Vector3f;

static Vector3f Pt3toVec(const Point_3 pt) 
{
    return Vector3f(float(pt.x()), float(pt.y()), float(pt.z()));
}

static Vector3f cgalVec2EigenVec(Vector cgal_v) {
    return Vector3f(cgal_v[0], cgal_v[1], cgal_v[2]);
}


Mesh initObstacle(obsParam obs_param)
{
	Mesh mesh;
    std::vector<Point_3> points;
	for(int pt_ind = 0; pt_ind < obs_param.verts.cols(); pt_ind++)
	{
		// std::vector<Mesh::Vertex_index> indices;
		points.push_back(Point_3(obs_param.verts(0, pt_ind),
							 	 obs_param.verts(1, pt_ind),
							 	 obs_param.verts(2, pt_ind)));
		// int pt_ind = int(obs_param.faces(i, face_ind));
		// indices.push_back(mesh.add_vertex(Point_3(obs_param.verts(0, pt_ind),
		// 									      obs_param.verts(1, pt_ind),
		// 									      obs_param.verts(2, pt_ind))));
		// mesh.add_face(indices[0], indices[1], indices[2]);
	}
    CGAL::convex_hull_3(points.begin(), points.end(), mesh);
	// std::vector<face_descriptor> newfaces;
	// std::vector<vertex_descriptor> newvertices;
    // CGAL::Polygon_mesh_processing::refine(mesh,
	// 									  faces(mesh),
	// 									  std::back_inserter(newfaces),
	// 									  std::back_inserter(newvertices),
	// 									  CGAL::Polygon_mesh_processing::parameters::density_control_factor(10.));
	// CGAL::draw(mesh);
	// std::vector<vertex_descriptor> region;
	// CGAL::Polygon_mesh_processing::fair(mesh, region);
	return mesh;
}

std::tuple<Matrix3Xf, Matrix3Xf> nearest_points_and_normal_help(const Matrix3Xf& last_template,
																const Mesh& mesh,
																const Mesh::Property_map<vertex_descriptor, Vector>& vnormals)
{
	Matrix3Xf nearestPts(3, last_template.cols());
    Matrix3Xf normalVecs(3, last_template.cols());

	for(int pt_ind = 0; pt_ind < last_template.cols(); pt_ind++)
	{
		Point_3 pt(last_template(0, pt_ind),
				   last_template(1, pt_ind),
				   last_template(2, pt_ind));
        Ray_3 ray(pt, pt);
		Face_location query_location = PMP::locate(pt, mesh);
  //       Face_location query_location = PMP::locate_with_AABB_tree(ray, tree, mesh);
		// Point_3 nearestPt = PMP::construct_point(query_location, mesh);
  //       nearestPts.col(pt_ind) = Pt3toVec(nearestPt);
        // cout << "nearestPt of " << pt << " is on " << source(halfedge(query_location.first,mesh),mesh) << endl;

		double w[3];
		for(int i = 0; i < 3; i++){
			w[i] = query_location.second[i];
		}

        if (isnan(w[0]) || isnan(w[1]) || isnan(w[2])) {
            w[0] = w[1] = w[2] = 1/3;
        }

        // for (vertex_descriptor vd: vertices_around_face(mesh.halfedge(query_location.first), mesh)) {
        //     cout << vd << endl;
        // }        
        // cout << "242" << endl;

        MatrixXf verts_of_face(3, 3); // cout << "243" << endl;
        verts_of_face.col(0) = Pt3toVec(mesh.point(source(halfedge(query_location.first,mesh),mesh))); // cout << "244" << endl;
        verts_of_face.col(1) = Pt3toVec(mesh.point(target(halfedge(query_location.first,mesh),mesh))); // cout << "245" << endl;
        verts_of_face.col(2) = Pt3toVec(mesh.point(target(next(halfedge(query_location.first,mesh),mesh),mesh))); // cout << "246" << endl;
        nearestPts.col(pt_ind) = verts_of_face.col(0) * w[0] + verts_of_face.col(1)* w[1] + verts_of_face.col(2) * w[2];
        // cout << "248" << endl;


        // std::cout << "Vertex normals :" << std::endl; 
        // for(vertex_descriptor vd: vertices(mesh)){
        //     std::cout << vnormals[vd][0] << std::endl;
        // }

		// MatrixXf verts_of_face(3, 3);
		// verts_of_face.col(0) = Pt3toVec(mesh.point(source(halfedge(query_location.first,mesh),mesh)));
		// verts_of_face.col(1) = Pt3toVec(mesh.point(target(halfedge(query_location.first,mesh),mesh)));
		// verts_of_face.col(2) = Pt3toVec(mesh.point(target(next(halfedge(query_location.first,mesh),mesh),mesh)));
		
		Vector3f normalVec(0.0, 0.0, 0.0);
        // cout << "262" << endl;
        normalVec = cgalVec2EigenVec(
                        vnormals[source(halfedge(query_location.first,mesh),mesh)] * w[0] +
                        vnormals[target(halfedge(query_location.first,mesh),mesh)] * w[1] +
                        vnormals[target(next(halfedge(query_location.first,mesh),mesh),mesh)] * w[2]
                    );
        // normalVec = cgalVec2EigenVec(fnormals[query_location.first]);

		// for (int i = 0; i < 3; i++) {
		// 	for (int mesh_ind = 0; mesh_ind < obs_mesh.cols(); mesh_ind++)
		// 	{
		// 		if (verts_of_face.col(i).isApprox(obs_mesh.col(mesh_ind))) {
		// 			normalVec = normalVec + obs_normal.col(mesh_ind) * w[i];
		// 		}
		// 	}
		// }
		
		normalVecs.col(pt_ind) = normalVec;
	}
    return {nearestPts, normalVecs};
}
