#include "cdcpd/deformable_object_configuration.h"

DeformableObjectTracking::DeformableObjectTracking(DeformableObjectTracking const& other)
{
    vertices = other.vertices;
    edges = other.edges;
    // Need to be careful so that the newly tracked points point to different memory than the
    // "other" tracked points
    points = PointCloud::Ptr(new PointCloud(*other.points));
}

DeformableObjectConfiguration::DeformableObjectConfiguration(int const num_points_)
    : num_points(num_points_)
{}

void DeformableObjectTracking::makeCloud(Eigen::Matrix3Xf const& points_mat)
{
  // TODO: Can we do this cleaner via some sort of data mapping?
  PointCloud::Ptr cloud(new PointCloud);
  for (int i = 0; i < points_mat.cols(); ++i) {
    auto const& c = points_mat.col(i);
    cloud->push_back(pcl::PointXYZ(c(0), c(1), c(2)));
  }
  points = cloud;
}

void DeformableObjectConfiguration::initializeTracking(Eigen::Vector3f const& start_position,
    Eigen::Vector3f const& end_position)
{
    DeformableObjectTracking object_tracking = makeTemplate(start_position, end_position);
    tracked = DeformableObjectTracking(object_tracking);
    initial = DeformableObjectTracking(object_tracking);
}

RopeConfiguration::RopeConfiguration(int const num_points_, float const max_rope_length_)
    : DeformableObjectConfiguration{num_points_},
      max_rope_length(max_rope_length_),
      max_segment_length(max_rope_length / static_cast<float>(num_points))
{}

DeformableObjectTracking RopeConfiguration::makeTemplate(Eigen::Vector3f const& start_position,
    Eigen::Vector3f const& end_position)
{
    Eigen::Matrix3Xf template_vertices(3, num_points);  // Y^0 in the paper
    Eigen::VectorXf thetas = Eigen::VectorXf::LinSpaced(num_points, 0, 1);
    for (int i = 0; i < num_points; ++i) {
        auto const theta = thetas.row(i);
        template_vertices.col(i) = (end_position - start_position) * theta + start_position;
    }
    Eigen::Matrix2Xi template_edges(2, num_points - 1);
    template_edges(0, 0) = 0;
    template_edges(1, template_edges.cols() - 1) = num_points - 1;
    for (int i = 1; i <= template_edges.cols() - 1; ++i) {
        template_edges(0, i) = i;
        template_edges(1, i - 1) = i;
    }
    DeformableObjectTracking configuration;
    configuration.vertices = template_vertices;
    configuration.edges = template_edges;
    configuration.makeCloud(template_vertices);

    return configuration;
}