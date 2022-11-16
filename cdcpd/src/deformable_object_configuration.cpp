#include "cdcpd/deformable_object_configuration.h"

// Unravels the index for row, col and gives the total index (row major)
int unravel_indices(int row, int col, int num_points_width)
{
    return row * num_points_width + col;
}

DeformableObjectTracking::DeformableObjectTracking(DeformableObjectTracking const& other)
{
    vertices_ = other.vertices_;
    edges_ = other.edges_;
    // Need to be careful so that the newly tracked points point to different memory than the
    // "other" tracked points
    points_ = PointCloud::Ptr(new PointCloud(*other.points_));
}

DeformableObjectConfiguration::DeformableObjectConfiguration(int const num_points)
    : num_points_(num_points)
{}

void DeformableObjectTracking::makeCloud(Eigen::Matrix3Xf const& points_mat)
{
  // TODO: Can we do this cleaner via some sort of data mapping?
  PointCloud::Ptr cloud(new PointCloud);
  for (int i = 0; i < points_mat.cols(); ++i) {
    auto const& c = points_mat.col(i);
    cloud->push_back(pcl::PointXYZ(c(0), c(1), c(2)));
  }
  points_ = cloud;
}

void DeformableObjectTracking::setVertices(std::vector<cv::Point3f> const& vertex_points)
{
    Eigen::Matrix3Xf vertices_new(3, vertex_points.size());
    int i = 0;
    for (auto const& pt : vertex_points)
    {
        vertices_new(0, i) = pt.x;
        vertices_new(1, i) = pt.y;
        vertices_new(2, i) = pt.z;

        ++i;
    }
    vertices_ = vertices_new;
}

void DeformableObjectTracking::setEdges(std::vector<std::tuple<int, int>> const& edges)
{
    Eigen::Matrix2Xi edges_new(2, edges.size());
    int i = 0;
    for (auto const& edge : edges)
    {
        edges_new(0, i) = std::get<0>(edge);
        edges_new(1, i) = std::get<1>(edge);
        ++i;
    }
    edges_ = edges_new;
}

void DeformableObjectConfiguration::initializeTracking()
{
    DeformableObjectTracking object_tracking = makeTemplate();
    tracked_ = DeformableObjectTracking(object_tracking);
    initial_ = DeformableObjectTracking(object_tracking);
}

DeformableObjectConfigurationMap::DeformableObjectConfigurationMap()
    : tracking_map_(),
      deformable_object_id_next_(0),
      ordered_def_obj_ids_()
{}

int DeformableObjectConfigurationMap::get_total_num_points() const
{
    int num_points_total = 0;
    for (auto const& def_obj_id : ordered_def_obj_ids_)
    {
        std::shared_ptr<DeformableObjectConfiguration> def_obj_config =
            tracking_map_.at(def_obj_id);
        num_points_total += def_obj_config->num_points_;
    }
    return num_points_total;
}

int DeformableObjectConfigurationMap::get_total_num_edges() const
{
    int num_edges_total = 0;
    for (auto const& tracked_pair : tracking_map_)
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
    for (auto const& configuration : tracking_map_)
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
    tracking_map_.emplace(deformable_object_id_next_, def_obj_config);

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
        std::shared_ptr<DeformableObjectConfiguration>& def_obj_config = tracking_map_[def_obj_id];

        // Grab the range of indices where this configuration's points will be stored in the point
        // cloud.
        std::tuple<int, int> const& idx_range = assignment_range.second;
        int const& idx_start = std::get<0>(idx_range);
        int const& idx_end = std::get<1>(idx_range);

        // Use the point cloud iterators and std::copy to efficiently update the tracked points.
        auto const& it_begin = cloud_it_begin + idx_start;
        auto const& it_end = cloud_it_begin + idx_end;
        std::copy(it_begin, it_end, def_obj_config->tracked_.points_->begin());
    }
}

PointCloud::Ptr DeformableObjectConfigurationMap::form_vertices_cloud(
    bool const use_initial_state=false) const
{
    PointCloud::Ptr vertices_cloud;
    // I don't think we actually care about the stamp at this point.
    // bool stamp_copied = false;
    for (auto const& def_obj_id : ordered_def_obj_ids_)
    {
        // Grab the deformable object configuration.
        std::shared_ptr<DeformableObjectConfiguration> const def_obj_config =
            tracking_map_.at(def_obj_id);

        std::shared_ptr<DeformableObjectTracking> tracking =
            get_appropriate_tracking(def_obj_config, use_initial_state);

        // Concatenate this deformable object's point cloud with the aggregate point cloud.
        (*vertices_cloud) += (*tracking->points_);
    }
    return vertices_cloud;
}

Eigen::Matrix2Xi DeformableObjectConfigurationMap::form_edges_matrix(
    bool const use_initial_state=false) const
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
            tracking_map_.at(def_obj_id);

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
            tracking_map_.at(def_obj_id);

        double const def_obj_max_segment_length = def_obj_config->max_segment_length_;
        for (int i = 0; i < def_obj_config->tracked_.edges_.cols(); ++i)
        {
            max_segment_lengths[1, edge_idx_aggregate] = def_obj_max_segment_length;
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
        tracking = std::make_shared<DeformableObjectTracking>(&def_obj_config->initial_);
    }
    else
    {
        tracking = std::make_shared<DeformableObjectTracking>(&def_obj_config->tracked_);
    }
    return tracking;
}

RopeConfiguration::RopeConfiguration(int const num_points, float const max_rope_length,
    Eigen::Vector3f const rope_start_position, Eigen::Vector3f const rope_end_position)
    : DeformableObjectConfiguration{num_points},
      max_rope_length_(max_rope_length),
      rope_start_position_initial_(rope_start_position),
      rope_end_position_initial_(rope_end_position)
{
    max_segment_length_ = max_rope_length / (static_cast<float>(num_points) - 1);
}

DeformableObjectTracking RopeConfiguration::makeTemplate()
{
    Eigen::Matrix3Xf template_vertices(3, num_points_);  // Y^0 in the paper
    Eigen::VectorXf thetas = Eigen::VectorXf::LinSpaced(num_points_, 0, 1);
    for (int i = 0; i < num_points_; ++i) {
        auto const theta = thetas.row(i);
        template_vertices.col(i) = (rope_end_position_initial_ - rope_start_position_initial_)
            * theta + rope_start_position_initial_;
    }
    Eigen::Matrix2Xi template_edges(2, num_points_ - 1);
    template_edges(0, 0) = 0;
    template_edges(1, template_edges.cols() - 1) = num_points_ - 1;
    for (int i = 1; i <= template_edges.cols() - 1; ++i) {
        template_edges(0, i) = i;
        template_edges(1, i - 1) = i;
    }
    DeformableObjectTracking configuration;
    configuration.vertices_ = template_vertices;
    configuration.edges_ = template_edges;
    configuration.makeCloud(template_vertices);

    return configuration;
}

ClothConfiguration::ClothConfiguration(float const length_initial, float const width_initial,
    float const grid_size_initial_guess)
    : DeformableObjectConfiguration{},
      length_initial_{length_initial},
      width_initial_{width_initial},
      grid_size_initial_guess_{grid_size_initial_guess},
      num_points_length_{std::ceil(length_initial / grid_size_initial_guess) + 1},
      num_points_width_{std::ceil(width_initial / grid_size_initial_guess) + 1},
      num_edges_{(num_points_length_ - 1) * num_points_width_
                  + (num_points_width_ - 1) * num_points_length_}
{
    num_points_ = num_points_length_ * num_points_width_;

    // Calculate the actual max_segment_length_ based off of the number of segments in both the
    // length and the width directions.
    int num_segments_length = num_points_length_ - 1;
    int num_segments_width = num_points_width_ - 1;
    float grid_size_initial_actual_length =
        length_initial_ / static_cast<float>(num_segments_length);
    float grid_size_initial_actual_width = width_initial_ / static_cast<float>(num_segments_width);
    // TODO(Dylan): This needs to be updated to include the stretch limit parameter!
    max_segment_length_ = std::min({grid_size_initial_actual_length,
        grid_size_initial_actual_width});
}

DeformableObjectTracking ClothConfiguration::makeTemplate()
{
    DeformableObjectTracking configuration;
    std::vector<std::tuple<int, int>> edges_list;

    std::vector<cv::Point3f> template_original_points;
    int idx = 0;
    for (int i = 0; i < num_points_length_; ++i)
    {
        for (int j = 0; j < num_points_width_; ++j)
        {
            cv::Point3f pt(i * max_segment_length_, j * max_segment_length_, 0);
            template_original_points.push_back(pt);

            if (i + 1 < num_points_length_)
            {
                edges_list.push_back({idx, unravel_indices(i + 1, j, num_points_width_)});
            }
            if (j + 1 < num_points_length_)
            {
                edges_list.push_back({idx, unravel_indices(i, j + 1, num_points_width_)});
            }

            ++idx;
        }
    }

    // Get the warped template points based on supplied affine transformation.
    std::vector<cv::Point3f> template_warped_points;
    cv::perspectiveTransform(template_original_points, template_warped_points,
        template_affine_transform_);


    // Store the warped template in the returned configuration.
    configuration.setVertices(template_warped_points);
    configuration.setEdges(edges_list);

    configuration.makeCloud(configuration.vertices_);

    return configuration;
}