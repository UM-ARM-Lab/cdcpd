#pragma once

#include <map>

#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "cdcpd/deformable_object_configuration.h"

// Stores the vertex indices that belong to each template in the large point cloud/matrix formed by
// TrackingMap::form_vertices_cloud()
struct TemplateVertexAssignment
{
    TemplateVertexAssignment(int const template_id_in, int const vertex_idx_start,
        int const vertex_idx_end)
        : template_id(template_id_in)
        , idx_start(vertex_idx_start)
        , idx_end(vertex_idx_end)
    {}

    int const template_id;
    int const idx_start;
    int const idx_end;
};

class TrackingMap
{
public:
    TrackingMap();

    // Returns the total number of tracked points.
    int get_total_num_points() const;

    // Returns the total number of tracked edges.
    int get_total_num_edges() const;

    // Returns a map that indicates which vertices belong to which template.
    // The tuple indicates the start and end vertex indexes that belong to that template.
    std::vector<TemplateVertexAssignment> get_vertex_assignments() const;

    // Adds the given deformable object configuration to our tracking map.
    void add_def_obj_configuration(std::shared_ptr<DeformableObjectConfiguration> const def_obj_config);

    // Updates the vertices for all deformable objects given new points predicted from a CDCPD run.
    void update_def_obj_vertices(pcl::shared_ptr<PointCloud> const vertices_new);

    // Forms the vertices matrix for all tracked templates expected by CDCPD
    PointCloud::Ptr form_vertices_cloud(bool const use_initial_state=false) const;

    // Forms the edges matrix for all tracked templates expected by CDCPD
    Eigen::Matrix2Xi form_edges_matrix(bool const use_initial_state=false) const;

    // Returns the matrix describing the maximum length each edge can have.
    Eigen::RowVectorXd form_max_segment_length_matrix() const;

    // The map that holds the deformable objects we're tracking.
    std::map<int, std::shared_ptr<DeformableObjectConfiguration> > tracking_map;

protected:
    // The next ID we'll assign to an incoming deformable object configuration to track.
    int deformable_object_id_next_;

    // Holds the IDs of the tracked objects in a way that preserves order as the map does not.
    // This is necessary for formation of the vertex, edge, and max segment length matrices.
    std::vector<int> ordered_def_obj_ids_;

    // Return the appropriate DeformableObjectTracking given if we should take the initial or
    // tracked states.
    std::shared_ptr<DeformableObjectTracking> get_appropriate_tracking(
        std::shared_ptr<DeformableObjectConfiguration> const def_obj_config,
        bool take_initial_state) const;
};