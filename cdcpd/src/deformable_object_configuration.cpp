#include "cdcpd/deformable_object_configuration.h"

// Unravels the index for row, col and gives the total index (row major)
int unravel_indices(int row, int col, int num_points_width)
{
    return row * num_points_width + col;
}

DeformableObjectType get_deformable_object_type(std::string const& def_obj_type_str)
{
    std::map<std::string, DeformableObjectType> obj_type_map{
        {"rope", DeformableObjectType::rope},
        {"cloth", DeformableObjectType::cloth}
    };
    return obj_type_map[def_obj_type_str];
}

ConnectivityNode::ConnectivityNode(int const node_id)
    : id_(node_id),
      neighbors_()
{}

void ConnectivityNode::insert_neighbor_node(std::shared_ptr<ConnectivityNode> const neighbor)
{
    neighbors_.insert({neighbor->id_, neighbor});
}

bool ConnectivityNode::is_neighbor_node(int const id_neighbor) const
{
    return neighbors_.count(id_neighbor) != 0;
}

ConnectivityGraph::ConnectivityGraph()
    : nodes_()
{}

ConnectivityGraph::ConnectivityGraph(Eigen::Matrix2Xi const& edge_list)
    : ConnectivityGraph()
{
    for (int edge_idx = 0; edge_idx < edge_list.cols(); ++edge_idx)
    {
        auto const& edge = edge_list.col(edge_idx);
        int const& id_1 = edge(0, 0);
        int const& id_2 = edge(1, 0);

        insert_edge(id_1, id_2);
    }
}

ConnectivityGraph::ConnectivityGraph(ConnectivityGraph const& old_graph, int const id_offset)
{
    for (auto const& old_node_pair : old_graph.nodes_)
    {
        int const id_old = old_node_pair.first;
        std::shared_ptr<ConnectivityNode> node_old = old_node_pair.second;

        int const id_new = id_old + id_offset;
        std::shared_ptr<ConnectivityNode> node_new = insert_node(id_new);

        // Add neighbors to the new node if they aren't present.
        for (auto const& neighbor_pair_old : node_old->get_neighbors())
        {
            int const id_neighbor_new = neighbor_pair_old.first + id_offset;
            auto node_neighbor = insert_node(id_neighbor_new);

            insert_edge(node_new, node_neighbor);
        }
    }
}

std::shared_ptr<ConnectivityNode> ConnectivityGraph::insert_node(int const node_id)
{
    auto node = std::make_shared<ConnectivityNode>(node_id);

    // Using map::insert here as insert respects if the key, value pair is already in the map.
    auto insertion_pair = nodes_.insert({node_id, node});
    auto const& map_iterator = insertion_pair.first;

    // This either points to the node we just created or the node that was already in the map with
    // the node_id key.
    node = map_iterator->second;

    return node;
}

void ConnectivityGraph::insert_edge(int const id_node_1, int const id_node_2)
{
    auto node_1 = insert_node(id_node_1);
    auto node_2 = insert_node(id_node_2);

    node_1->insert_neighbor_node(node_2);
    node_2->insert_neighbor_node(node_1);
}

void ConnectivityGraph::insert_edge(std::shared_ptr<ConnectivityNode> node_1,
    std::shared_ptr<ConnectivityNode> node_2)
{
    node_1->insert_neighbor_node(node_2);
    node_2->insert_neighbor_node(node_1);
}

DeformableObjectTracking::DeformableObjectTracking()
    : vertices_(),
      edges_(),
      points_(),
      connectivity_graph_(edges_)
{}

DeformableObjectTracking::DeformableObjectTracking(DeformableObjectTracking const& other)
{
    vertices_ = other.vertices_;
    edges_ = other.edges_;
    points_ = other.getPointCloudCopy();
    connectivity_graph_ = other.connectivity_graph_;
}

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
    setVertices(vertices_new);
}

void DeformableObjectTracking::setVertices(Eigen::Matrix3Xf const& vertices_in)
{
    vertices_ = vertices_in;
    makeCloud(vertices_);
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
    setEdges(edges_new);
}

void DeformableObjectTracking::setEdges(Eigen::Matrix2Xi const& edges_in)
{
    edges_ = edges_in;
    connectivity_graph_ = ConnectivityGraph(edges_);
}

void DeformableObjectTracking::setPointCloud(PointCloud::const_iterator const& it_in_begin, PointCloud::const_iterator const& it_in_end)
{
    std::copy(it_in_begin, it_in_end, points_->begin());
}

DeformableObjectConfiguration::DeformableObjectConfiguration(int const num_points)
    : num_points_(num_points),
      max_segment_length_(0),
      initial_(),
      tracked_()
{}

void DeformableObjectConfiguration::initializeTracking()
{
    DeformableObjectTracking object_tracking = makeTemplate();
    tracked_ = DeformableObjectTracking(object_tracking);
    initial_ = DeformableObjectTracking(object_tracking);
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
    configuration.setVertices(template_vertices);
    configuration.setEdges(template_edges);

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
            if (j + 1 < num_points_width_)
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

    return configuration;
}