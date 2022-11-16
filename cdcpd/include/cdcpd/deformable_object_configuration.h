#pragma once

#include <tuple>
#include <map>

#include <Eigen/Dense>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "opencv2/imgproc.hpp"
#include <opencv2/core/eigen.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

enum DeformableObjectType
{
    rope,
    cloth
};

DeformableObjectType get_deformable_object_type(std::string const& def_obj_type_str)
{
    std::map<std::string, DeformableObjectType> obj_type_map{
        {"rope", DeformableObjectType::rope},
        {"cloth", DeformableObjectType::cloth}
    };
    return obj_type_map[def_obj_type_str];
}

struct DeformableObjectTracking
{
    DeformableObjectTracking(){}

    // Turns the passed in Matrix of XYZ points to a point cloud.
    // TODO(dylan.colli): remove argument. Should just use the vertices member.
    void makeCloud(Eigen::Matrix3Xf const& points);

    // Copy constructor.
    DeformableObjectTracking(DeformableObjectTracking const& other);

    // Sets the vertices from input point vector.
    void setVertices(std::vector<cv::Point3f> const& vertex_points);

    // Sets the edges from input edge vector.
    void setEdges(std::vector<std::tuple<int, int>> const& edges);

    Eigen::Matrix3Xf vertices_{};
    Eigen::Matrix2Xi edges_{};
    PointCloud::Ptr points_;
};

class DeformableObjectConfiguration
{
public:
    DeformableObjectConfiguration(){}
    DeformableObjectConfiguration(int const num_points_);

    // Forms the deformable object tracking template. This needs to be overriden by derived classes
    // since this is dependent upon the type of deformable object we're tracking.
    DeformableObjectTracking virtual makeTemplate() = 0;

    virtual ~DeformableObjectConfiguration(){}

    void initializeTracking();

    int num_points_;
    float max_segment_length_;
    DeformableObjectTracking tracked_;
    DeformableObjectTracking initial_;
};

class DeformableObjectConfigurationMap
{
public:
    DeformableObjectConfigurationMap(){}

    // Returns the total number of tracked points.
    int get_total_num_points() const;

    // Returns the total number of tracked edges.
    int get_total_num_edges() const;

    // Returns a map that indicates which vertices belong to which template.
    // The tuple indicates the start and end vertex indexes that belong to that template.
    std::map<int, std::tuple<int, int> > get_vertex_assignments() const;

    // Adds the given deformable object configuration to our tracking map.
    void add_def_obj_configuration(std::shared_ptr<DeformableObjectConfiguration> const def_obj_config);

    // Updates the vertices for all deformable objects given new points predicted from a CDCPD run.
    void update_def_obj_vertices(pcl::shared_ptr<PointCloud> const vertices_new);

    // Forms the vertices matrix for all tracked templates expected by CDCPD
    PointCloud::Ptr form_vertices_cloud(bool const use_initial_state=false) const;

    // Forms the edges matrix for all tracked templates expected by CDCPD
    Eigen::Matrix2Xi form_edges_matrix(bool const use_initial_state=false) const;

    // Returns the matrix describing the maximum length each edge can have.
    // Eigen::Matrix2Xf form_max_segment_length_matrix() const;

protected:
    // The map that holds the deformable objects we're tracking.
    std::map<int, std::shared_ptr<DeformableObjectConfiguration> > tracking_map;

    // The next ID we'll assign to an incoming deformable object configuration to track.
    int deformable_object_id_next;

    // Return the appropriate DeformableObjectTracking given if we should take the initial or
    // tracked states.
    std::shared_ptr<DeformableObjectTracking> get_appropriate_tracking(
        std::shared_ptr<DeformableObjectConfiguration> const def_obj_config,
        bool take_initial_state) const;
};

class RopeConfiguration : public DeformableObjectConfiguration
{
public:
    RopeConfiguration(int const num_points_, float const max_rope_length_,
        Eigen::Vector3f const rope_start_position, Eigen::Vector3f const rope_end_position);

    DeformableObjectTracking virtual makeTemplate();

    float const max_rope_length_;
    Eigen::Vector3f const rope_start_position_initial_;
    Eigen::Vector3f const rope_end_position_initial_;
};


class ClothConfiguration : public DeformableObjectConfiguration
{
public:
    ClothConfiguration(float const length_initial, float const width_initial,
        float const grid_size_initial_guess);

    DeformableObjectTracking virtual makeTemplate();

    // Describes the affine transformation the initial template grid underwent to warp to the
    // initial tracked template.
    // NOTE: this is initialized to an identity affine transform.
    cv::Mat template_affine_transform_ = (cv::Mat_<float>(4, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);

    // Describes the number of points the template is initialized with in the length direction.
    // Length is downwards direction!
    int const num_points_length_;

    // Describes the number of points the template is initialized with in the width direction.
    // Width is rightwards direction!
    int const num_points_width_;

    int const num_edges_;

    // Initial length of the template.
    float const length_initial_;

    // Initial width of the template.
    float const width_initial_;

    // The supplied initial guess for the grid size.
    float const grid_size_initial_guess_;
};