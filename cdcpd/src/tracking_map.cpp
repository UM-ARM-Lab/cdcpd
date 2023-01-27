#include "cdcpd/tracking_map.h"

#include <iostream>

TrackingMap::TrackingMap()
    : tracking_map(),
      deformable_object_id_next_(0),
      ordered_def_obj_ids_()
{}

int TrackingMap::get_total_num_points() const
{
    int num_points_total = 0;
    for (auto const& def_obj_id : ordered_def_obj_ids_)
    {
        std::shared_ptr<DeformableObjectConfiguration> def_obj_config =
            tracking_map.at(def_obj_id);
        num_points_total += def_obj_config->num_points_;
    }
    return num_points_total;
}

int TrackingMap::get_total_num_edges() const
{
    int num_edges_total = 0;
    for (auto const& tracked_pair : tracking_map)
    {
        std::shared_ptr<DeformableObjectConfiguration> const& def_obj_config = tracked_pair.second;
        int num_edges = def_obj_config->tracked_.getEdges().cols();
        num_edges_total += num_edges;
    }
    return num_edges_total;
}

std::vector<TemplateVertexAssignment> TrackingMap::get_vertex_assignments() const
{
    // NOTE: This could be kept track of as a member variable and instead updated any time a
    // deformable object is added/removed.
    std::vector<TemplateVertexAssignment> vertex_assignments;
    int idx_begin = 0;
    for (int const def_obj_id : ordered_def_obj_ids_)
    {
        std::shared_ptr<DeformableObjectConfiguration> configuration = tracking_map.at(def_obj_id);
        int const idx_end = idx_begin + configuration->num_points_;

        // Using emplace_back here avoids an unnecessary copy constructor I believe.
        vertex_assignments.emplace_back(def_obj_id, idx_begin, idx_end);

        idx_begin = idx_end;
    }
    return vertex_assignments;
}

std::vector<std::shared_ptr<ConnectivityGraph> > TrackingMap::get_tracked_graphs() const
{
    std::vector<std::shared_ptr<ConnectivityGraph> > tracked_graphs;
    std::vector<TemplateVertexAssignment> const vertex_assignments = get_vertex_assignments();

    int def_obj_idx = 0;
    for (int const def_obj_id : ordered_def_obj_ids_)
    {
        TemplateVertexAssignment const& assignment = vertex_assignments.at(def_obj_idx);

        // Including just as a sanity check that iteration order is preserved.
        assert(assignment.template_id == def_obj_id);

        auto const def_obj_config = tracking_map.at(def_obj_id);
        ConnectivityGraph const& old_graph = def_obj_config->tracked_.getConnectivityGraph();

        // Get a new connectivity graph that matches the vertex indices in the aggregate
        // point cloud/matrix
        auto updated_graph = std::make_shared<ConnectivityGraph>(old_graph, assignment.idx_start);

        tracked_graphs.push_back(updated_graph);
        ++def_obj_idx;
    }
    return tracked_graphs;
}

void TrackingMap::add_def_obj_configuration(
    std::shared_ptr<DeformableObjectConfiguration> const def_obj_config)
{
    tracking_map.emplace(deformable_object_id_next_, def_obj_config);

    // Just insert the deformable object ID at the back or our track ID list since we know this ID
    // is the greatest ID (since we just increment the ID each time).
    ordered_def_obj_ids_.push_back(deformable_object_id_next_);

    ++deformable_object_id_next_;
}

void TrackingMap::update_def_obj_vertices(pcl::shared_ptr<PointCloud> const vertices_new)
{
    // std::vector<TemplateVertexAssignment> vertex_assignments = get_vertex_assignments();
    // auto const& cloud_it_begin = vertices_new->begin();
    // for (auto const& assignment : vertex_assignments)
    // {
    //     // Get the deformable object configuration we're updating.
    //     std::shared_ptr<DeformableObjectConfiguration>& def_obj_config =
    //         tracking_map[assignment.template_id];

    //     // Use the point cloud iterators and std::copy to efficiently update the tracked points.
    //     // auto const& it_begin = cloud_it_begin + assignment.idx_start;
    //     // auto const& it_end = cloud_it_begin + assignment.idx_end;
    //     // def_obj_config->tracked_.setPointCloud(it_begin, it_end);

    //     // PointCloud::ConstPtr tracked_cloud = def_obj_config->tracked_.getPointCloud();
    //     // def_obj_config->tracked_.setVertices(tracked_cloud->getMatrixXfMap().topRows(3));
    // }
    update_def_obj_vertices_from_mat(vertices_new->getMatrixXfMap().topRows(3));
}

void TrackingMap::update_def_obj_vertices_from_mat(Eigen::Matrix3Xf const& verts_new)
{
    std::vector<TemplateVertexAssignment> vertex_assignments = get_vertex_assignments();
    for (auto const& assignment : vertex_assignments)
    {
        // Get the deformable object configuration we're updating.
        std::shared_ptr<DeformableObjectConfiguration>& def_obj_config =
            tracking_map[assignment.template_id];

        // std::cout << "Updating template #" << assignment.template_id << " between indexes = "
        //     << assignment.idx_start << ", " << assignment.idx_end << std::endl;
        int const num_cols = assignment.idx_end - assignment.idx_start;
        def_obj_config->tracked_.setVertices(
            verts_new.block(0, assignment.idx_start, 3, num_cols));
    }
}

PointCloud::Ptr TrackingMap::form_vertices_cloud(bool const use_initial_state) const
{
    // std::cout << "At least made it here!\n";
    PointCloud::Ptr vertices_cloud(new PointCloud);
    // I don't think we actually care about the stamp at this point.
    // bool stamp_copied = false;
    for (auto const& def_obj_id : ordered_def_obj_ids_)
    {
        // std::cout << "\tUpdating def obj id #" << def_obj_id << std::endl;
        // Grab the deformable object configuration.
        std::shared_ptr<DeformableObjectConfiguration> const def_obj_config =
            tracking_map.at(def_obj_id);

        std::shared_ptr<DeformableObjectTracking> tracking =
            get_appropriate_tracking(def_obj_config, use_initial_state);

        // std::cout << "\tWith point cloud width = " << tracking->getPointCloudCopy()->width
        //     << std::endl;

        // Concatenate this deformable object's point cloud with the aggregate point cloud.
        (*vertices_cloud) += (*tracking->getPointCloud());
    }
    // std::cout << "End form_vertices_cloud\n";
    return vertices_cloud;
}

Eigen::Matrix2Xi TrackingMap::form_edges_matrix(bool const use_initial_state) const
{
    // std::cout << "Start form_edges_matrix\n";
    int const num_edges_total = get_total_num_edges();

    // Initialize an Eigen matrix for keeping track of the edges.
    Eigen::Matrix2Xi edges_total(2, num_edges_total);

    // Populate that edge matrix based on all of our edges.
    int edge_idx_aggregate = 0;
    for (auto const& def_obj_id : ordered_def_obj_ids_)
    {
        // Grab the deformable object configuration.
        std::shared_ptr<DeformableObjectConfiguration> const def_obj_config =
            tracking_map.at(def_obj_id);

        std::shared_ptr<DeformableObjectTracking> tracking =
            get_appropriate_tracking(def_obj_config, use_initial_state);

        Eigen::Matrix2Xi const& edges = tracking->getEdges();
        for (int edge_col = 0; edge_col < edges.cols(); ++edge_col)
        {
            edges_total.col(edge_idx_aggregate) = edges.col(edge_col);
            ++edge_idx_aggregate;
        }
    }
    // std::cout << "End form_edges_matrix\n";
    return edges_total;
}

Eigen::RowVectorXd TrackingMap::form_max_segment_length_matrix() const
{
    // Construct the structure that will hold the edge lengths.
    int const num_edges_total = get_total_num_edges();
    Eigen::RowVectorXd max_segment_lengths(num_edges_total);

    // Populate the edge length structure.
    // Right now this isn't very interesting but one could envisage dynamically refining tracking
    // by placing more Gaussians in areas of uncertainty, changing the maximum segment length based
    // on the new edges created.
    size_t edge_idx_aggregate = 0;
    for (auto const& def_obj_id : ordered_def_obj_ids_)
    {
        std::shared_ptr<DeformableObjectConfiguration> const def_obj_config =
            tracking_map.at(def_obj_id);

        double const def_obj_max_segment_length = def_obj_config->max_segment_length_;
        for (int i = 0; i < def_obj_config->tracked_.getEdges().cols(); ++i)
        {
            max_segment_lengths(0, edge_idx_aggregate) = def_obj_max_segment_length;
            ++edge_idx_aggregate;
        }
    }

    return max_segment_lengths;
}

std::shared_ptr<DeformableObjectTracking> TrackingMap::get_appropriate_tracking(
        std::shared_ptr<DeformableObjectConfiguration> const def_obj_config,
        bool take_initial_state) const
{
    std::shared_ptr<DeformableObjectTracking> tracking;
    if (take_initial_state)
    {
        tracking = std::make_shared<DeformableObjectTracking>(def_obj_config->initial_);
    }
    else
    {
        tracking = std::make_shared<DeformableObjectTracking>(def_obj_config->tracked_);
    }
    return tracking;
}