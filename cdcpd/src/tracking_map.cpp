#include "cdcpd/tracking_map.h"

DeformableObjectConfigurationMap::DeformableObjectConfigurationMap()
    : tracking_map(),
      deformable_object_id_next_(0),
      ordered_def_obj_ids_()
{}

int DeformableObjectConfigurationMap::get_total_num_points() const
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

int DeformableObjectConfigurationMap::get_total_num_edges() const
{
    int num_edges_total = 0;
    for (auto const& tracked_pair : tracking_map)
    {
        std::shared_ptr<DeformableObjectConfiguration> const& def_obj_config = tracked_pair.second;
        int num_edges = def_obj_config->tracked_.edges_.cols();
        num_edges_total += num_edges;
    }
    return num_edges_total;
}

std::map<int, std::tuple<int, int> > DeformableObjectConfigurationMap::get_vertex_assignments() const
{
    // NOTE: This could be kept track of as a member variable and instead updated any time a
    // deformable object is added/removed.
    std::map<int, std::tuple<int, int> > vertex_assignments;
    int idx_begin = 0;
    for (auto const& configuration : tracking_map)
    {
        int idx_end = idx_begin + configuration.second->num_points_;
        std::tuple<int, int> idx_range{idx_begin, idx_end};
        idx_begin = idx_end;

        vertex_assignments.emplace(configuration.first, idx_range);
    }
    return vertex_assignments;
}

void DeformableObjectConfigurationMap::add_def_obj_configuration(
    std::shared_ptr<DeformableObjectConfiguration> const def_obj_config)
{
    tracking_map.emplace(deformable_object_id_next_, def_obj_config);

    // Just insert the deformable object ID at the back or our track ID list since we know this ID
    // is the greatest ID (since we just increment the ID each time).
    ordered_def_obj_ids_.push_back(deformable_object_id_next_);

    ++deformable_object_id_next_;
}

void DeformableObjectConfigurationMap::update_def_obj_vertices(
    pcl::shared_ptr<PointCloud> const vertices_new)
{
    auto vertex_assignments = get_vertex_assignments();
    auto const& cloud_it_begin = vertices_new->begin();
    for (auto const& assignment_range : vertex_assignments)
    {
        // Get the deformable object configuration we're updating.
        int const& def_obj_id = assignment_range.first;
        std::shared_ptr<DeformableObjectConfiguration>& def_obj_config = tracking_map[def_obj_id];

        // Grab the range of indices where this configuration's points will be stored in the point
        // cloud.
        std::tuple<int, int> const& idx_range = assignment_range.second;
        int const& idx_start = std::get<0>(idx_range);
        int const& idx_end = std::get<1>(idx_range);

        // Use the point cloud iterators and std::copy to efficiently update the tracked points.
        auto const& it_begin = cloud_it_begin + idx_start;
        auto const& it_end = cloud_it_begin + idx_end;
        std::copy(it_begin, it_end, def_obj_config->tracked_.points_->begin());

        def_obj_config->tracked_.vertices_ = def_obj_config->tracked_.points_->getMatrixXfMap().topRows(3);
    }
}

PointCloud::Ptr DeformableObjectConfigurationMap::form_vertices_cloud(
    bool const use_initial_state) const
{
    PointCloud::Ptr vertices_cloud(new PointCloud);
    // I don't think we actually care about the stamp at this point.
    // bool stamp_copied = false;
    for (auto const& def_obj_id : ordered_def_obj_ids_)
    {
        // Grab the deformable object configuration.
        std::shared_ptr<DeformableObjectConfiguration> const def_obj_config =
            tracking_map.at(def_obj_id);

        std::shared_ptr<DeformableObjectTracking> tracking =
            get_appropriate_tracking(def_obj_config, use_initial_state);

        // Concatenate this deformable object's point cloud with the aggregate point cloud.
        (*vertices_cloud) += (*tracking->points_);
    }
    return vertices_cloud;
}

Eigen::Matrix2Xi DeformableObjectConfigurationMap::form_edges_matrix(
    bool const use_initial_state) const
{
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

        Eigen::Matrix2Xi const& edges = tracking->edges_;
        for (int edge_col = 0; edge_col < edges.cols(); ++edge_col)
        {
            edges_total.col(edge_idx_aggregate) = edges.col(edge_col);
            ++edge_idx_aggregate;
        }
    }
    return edges_total;
}

Eigen::RowVectorXd DeformableObjectConfigurationMap::form_max_segment_length_matrix() const
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
        for (int i = 0; i < def_obj_config->tracked_.edges_.cols(); ++i)
        {
            max_segment_lengths(0, edge_idx_aggregate) = def_obj_max_segment_length;
            ++edge_idx_aggregate;
        }
    }

    return max_segment_lengths;
}

std::shared_ptr<DeformableObjectTracking> DeformableObjectConfigurationMap::get_appropriate_tracking(
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