#include "smmap_utilities/visualization_tools.h"

#include <thread>
#include <std_srvs/Empty.h>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/geometry_msgs_builders.hpp>
#include <arc_utilities/ros_helpers.hpp>

using namespace smmap;
namespace vm = visualization_msgs;
namespace au = arc_utilities;
namespace ehc = EigenHelpersConversions;

////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////

bool Visualizer::standard_colors_initialized_ = false;
std_msgs::ColorRGBA Visualizer::red_;
std_msgs::ColorRGBA Visualizer::green_;
std_msgs::ColorRGBA Visualizer::blue_;
std_msgs::ColorRGBA Visualizer::black_;
std_msgs::ColorRGBA Visualizer::magenta_;
std_msgs::ColorRGBA Visualizer::yellow_;
std_msgs::ColorRGBA Visualizer::cyan_;
std_msgs::ColorRGBA Visualizer::white_;
std_msgs::ColorRGBA Visualizer::silver_;
std_msgs::ColorRGBA Visualizer::coral_;
std_msgs::ColorRGBA Visualizer::olive_;
std_msgs::ColorRGBA Visualizer::orange_;
std_msgs::ColorRGBA Visualizer::seafoam_;

void Visualizer::InitializeStandardColors()
{
    red_.r = 1.0;
    red_.g = 0.0;
    red_.b = 0.0;
    red_.a = 1.0;

    green_.r = 0.0;
    green_.g = 1.0;
    green_.b = 0.0;
    green_.a = 1.0;

    blue_.r = 0.0;
    blue_.g = 0.0;
    blue_.b = 1.0;
    blue_.a = 1.0;

    black_.r = 0.0;
    black_.g = 0.0;
    black_.b = 0.0;
    black_.a = 1.0;

    magenta_.r = 1.0f;
    magenta_.g = 0.0f;
    magenta_.b = 1.0f;
    magenta_.a = 1.0f;

    yellow_.r = 1.0f;
    yellow_.g = 1.0f;
    yellow_.b = 0.0f;
    yellow_.a = 1.0f;

    cyan_.r = 0.0f;
    cyan_.g = 1.0f;
    cyan_.b = 1.0f;
    cyan_.a = 1.0f;

    white_.r = 1.0f;
    white_.g = 1.0f;
    white_.b = 1.0f;
    white_.a = 1.0f;

    silver_.r = 0.75f;
    silver_.g = 0.75f;
    silver_.b = 0.75f;
    silver_.a = 1.0f;

    coral_.r = 0.8f;
    coral_.g = 0.36f;
    coral_.b = 0.27f;
    coral_.a = 1.0f;

    olive_.r = 0.31f;
    olive_.g = 0.31f;
    olive_.b = 0.18f;
    olive_.a = 1.0f;

    orange_.r = 0.8f;
    orange_.g = 0.2f;
    orange_.b = 0.2f;
    orange_.a = 1.0f;

    seafoam_.r = 112.f/255.f;
    seafoam_.g = 235.f/255.f;
    seafoam_.b = 204.f/255.f;
    seafoam_.a = 1.0f;

    standard_colors_initialized_ = true;
}

std_msgs::ColorRGBA Visualizer::Red(const float alpha)
{
    assert(standard_colors_initialized_);
    auto color = red_;
    color.a = alpha;
    return color;
}

std_msgs::ColorRGBA Visualizer::Green(const float alpha)
{
    assert(standard_colors_initialized_);
    auto color = green_;
    color.a = alpha;
    return color;
}

std_msgs::ColorRGBA Visualizer::Blue(const float alpha)
{
    assert(standard_colors_initialized_);
    auto color = blue_;
    color.a = alpha;
    return color;
}

std_msgs::ColorRGBA Visualizer::Black(const float alpha)
{
    assert(standard_colors_initialized_);
    auto color = black_;
    color.a = alpha;
    return color;
}

std_msgs::ColorRGBA Visualizer::Magenta(const float alpha)
{
    assert(standard_colors_initialized_);
    auto color = magenta_;
    color.a = alpha;
    return color;
}

std_msgs::ColorRGBA Visualizer::Yellow(const float alpha)
{
    assert(standard_colors_initialized_);
    auto color = yellow_;
    color.a = alpha;
    return color;
}

std_msgs::ColorRGBA Visualizer::Cyan(const float alpha)
{
    assert(standard_colors_initialized_);
    auto color = cyan_;
    color.a = alpha;
    return color;
}

std_msgs::ColorRGBA Visualizer::White(const float alpha)
{
    assert(standard_colors_initialized_);
    auto color = white_;
    color.a = alpha;
    return color;
}

std_msgs::ColorRGBA Visualizer::Silver(const float alpha)
{
    assert(standard_colors_initialized_);
    auto color = silver_;
    color.a = alpha;
    return color;
}

std_msgs::ColorRGBA Visualizer::Coral(const float alpha)
{
    assert(standard_colors_initialized_);
    auto color = coral_;
    color.a = alpha;
    return color;
}

std_msgs::ColorRGBA Visualizer::Olive(const float alpha)
{
    assert(standard_colors_initialized_);
    auto color = olive_;
    color.a = alpha;
    return color;
}

std_msgs::ColorRGBA Visualizer::Orange(const float alpha)
{
    assert(standard_colors_initialized_);
    auto color = orange_;
    color.a = alpha;
    return color;
}

std_msgs::ColorRGBA Visualizer::Seafoam(const float alpha)
{
    assert(standard_colors_initialized_);
    auto color = seafoam_;
    color.a = alpha;
    return color;
}

////////////////////////////////////////////////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////

Visualizer::Visualizer(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph,
        const bool publish_async)
    : nh_(nh)
    , ph_(ph)
    , publish_async_(publish_async)
    , running_async_(false)
    , disable_all_visualizations_(GetDisableSmmapVisualizations(*ph_))
    , clear_markers_srv_(nh_->serviceClient<std_srvs::Empty>(GetClearVisualizationsTopic(*nh_), true))
    , world_frame_name_(GetWorldFrameName())
    , gripper_apperture_(GetGripperApperture(*nh_))
{
    InitializeStandardColors();
    if (disable_all_visualizations_)
    {
        return;
    }
    if (ROSHelpers::GetParam<bool>(*nh_, "deform_simulator_node/start_bullet_viewer", false))
    {
        clear_markers_srv_.waitForExistence();
    }
    visualization_marker_pub_ = nh_->advertise<vm::Marker>(GetVisualizationMarkerTopic(*nh_), 256);
    visualization_maker_array_pub_ = nh_->advertise<vm::MarkerArray>(GetVisualizationMarkerArrayTopic(*nh_), 1);

    if (publish_async_)
    {
        async_markers_.markers.clear();
        publish_thread_ = std::thread(&Visualizer::publishAsyncMain, this);
    }
}

Visualizer::~Visualizer()
{
    if (!disable_all_visualizations_ && publish_async_)
    {
        // Tell the async thread to stop looping
        running_async_ = false;
        // Clean up our thread
        publish_thread_.join();
    }
}

void Visualizer::publish(const vm::Marker& marker) const
{
    if (disable_all_visualizations_)
    {
        return;
    }
    if (publish_async_)
    {
        std::lock_guard<std::mutex> lock(markers_mtx_);
        updateMarkerList(marker);
    }
    else
    {
        visualization_marker_pub_.publish(marker);
    }
}

void Visualizer::publish(const vm::MarkerArray& marker_array) const
{
    if (disable_all_visualizations_)
    {
        return;
    }
    if (publish_async_)
    {
        std::lock_guard<std::mutex> lock(markers_mtx_);
        async_markers_.markers.reserve(
                    async_markers_.markers.size() + marker_array.markers.size());
        for (const auto& marker : marker_array.markers)
        {
            updateMarkerList(marker);
        }
    }
    else
    {
        visualization_maker_array_pub_.publish(marker_array);
    }
}

void Visualizer::purgeAndPublishDeleteAllAction() const
{
    purgeMarkerList();
    visualization_msgs::Marker marker;
    marker.ns = "delete_markers";
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker.header.frame_id = "world_origin";
    marker.header.stamp = ros::Time::now();
    publish(marker);
    forcePublishNow();
    purgeMarkerList();
}

void Visualizer::forcePublishNow(const double last_delay) const
{
    if (disable_all_visualizations_)
    {
        return;
    }
    if (publish_async_)
    {
        std::lock_guard<std::mutex> lock(markers_mtx_);
        visualization_maker_array_pub_.publish(async_markers_);
        visualization_maker_array_pub_.publish(async_markers_);
        visualization_maker_array_pub_.publish(async_markers_);
        visualization_maker_array_pub_.publish(async_markers_);
        visualization_maker_array_pub_.publish(async_markers_);
        ros::spinOnce();
        arc_helpers::Sleep(0.01);

        visualization_maker_array_pub_.publish(async_markers_);
        visualization_maker_array_pub_.publish(async_markers_);
        visualization_maker_array_pub_.publish(async_markers_);
        visualization_maker_array_pub_.publish(async_markers_);
        visualization_maker_array_pub_.publish(async_markers_);
        ros::spinOnce();
        arc_helpers::Sleep(0.01);

        arc_helpers::Sleep(last_delay);
    }
    else
    {
        ROS_WARN_THROTTLE_NAMED(1.0, "visualizer", "forcePublishNow() does nothing if async publishing is not enabled.");
    }
}

////////////////////////////////////////////////////////////////////////////////

void Visualizer::clearVisualizationsBullet()
{
    if (disable_all_visualizations_)
    {
        return;
    }
    if (!ROSHelpers::GetParam<bool>(*nh_, "deform_simulator_node/start_bullet_viewer", false))
    {
        ROS_WARN_NAMED("visualizer", "Attempted to clear Bullet visualizations, but start_bullet_viewer is set to false");
        return;
    }
    std_srvs::Empty srv_data;
    while (!clear_markers_srv_.call(srv_data))
    {
        ROS_WARN_THROTTLE_NAMED(1.0, "visualizer", "Clear visualization data failed, reconnecting");
        clear_markers_srv_ = nh_->serviceClient<std_srvs::Empty>(GetClearVisualizationsTopic(*nh_), true);
        clear_markers_srv_.waitForExistence();
    }
}

void Visualizer::deleteAll() const
{
    if (disable_all_visualizations_)
    {
        return;
    }
    if (publish_async_)
    {
        std::lock_guard<std::mutex> lock(markers_mtx_);
        for (size_t idx = 0; idx < async_markers_.markers.size(); ++idx)
        {
            vm::Marker& marker = async_markers_.markers[idx];
            marker.action = vm::Marker::DELETE;
            marker.header.stamp = ros::Time::now();
            marker.lifetime = ros::Duration(0.1);
            marker.points.clear();
            marker.colors.clear();
            marker.text = "";
            marker.mesh_resource = "";
        }
    }
    else
    {
        ROS_WARN_THROTTLE_NAMED(1.0, "visualizer",
                                "Visualizer::deleteAll() called when publishing synchronously; no "
                                "marker data is stored in this mode, so no markers will be deleted."
                                "Use Visualizer::deleteObjects(...) to specify which objects to delete.");
    }
}

void Visualizer::purgeMarkerList() const
{
    if (disable_all_visualizations_)
    {
        return;
    }
    if (publish_async_)
    {
        std::lock_guard<std::mutex> lock(markers_mtx_);
        async_markers_.markers.clear();
    }
    else
    {
        ROS_WARN_THROTTLE_NAMED(1.0, "visualizer", "purgeMarkerList() does nothing if async publishing is not enabled.");
    }
}

void Visualizer::deleteObject(
        const std::string& marker_name,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return;
    }
    vm::Marker marker;
    marker.header.frame_id = world_frame_name_;
    marker.action = vm::Marker::DELETE;
    marker.ns = marker_name;
    marker.id = id;
    marker.lifetime = ros::Duration(1.0);
    publish(marker);

    if (!publish_async_)
    {
        ros::spinOnce();
        arc_helpers::Sleep(0.001);
    }
}

void Visualizer::deleteObjects(
        const std::string& marker_name,
        const int32_t start_id,
        const int32_t end_id) const
{
    if (disable_all_visualizations_)
    {
        return;
    }
    vm::Marker marker;
    marker.header.frame_id = world_frame_name_;
    marker.action = vm::Marker::DELETE;
    marker.ns = marker_name;
    marker.lifetime = ros::Duration(1.0);

    if (publish_async_)
    {
        std::lock_guard<std::mutex> lock(markers_mtx_);

        // Flag any existing markers for deletion
        std::vector<size_t> markers_to_delete;
        for (size_t idx = 0; idx < async_markers_.markers.size(); ++idx)
        {
            const vm::Marker& marker = async_markers_.markers[idx];
            if (marker.ns == marker_name &&
                start_id <= marker.id &&
                marker.id < end_id)
            {
                markers_to_delete.push_back(idx);
            }
        }

        // Delete the flaged markers
        vm::MarkerArray new_markers;
        new_markers.markers.reserve(async_markers_.markers.size() + end_id - start_id);
        for (size_t idx = 0; idx < async_markers_.markers.size(); ++idx)
        {
            const auto itr = std::find(markers_to_delete.begin(), markers_to_delete.end(), idx);
            if (itr != markers_to_delete.end())
            {
                new_markers.markers.push_back(async_markers_.markers[idx]);
            }
        }
        async_markers_ = new_markers;

        // Add new "DELETE" markers
        for (int32_t id = start_id; id < end_id; ++id)
        {
            marker.id = id;
            marker.header.stamp = ros::Time::now();
            async_markers_.markers.push_back(marker);
        }
    }
    else
    {
        for (int32_t id = start_id; id < end_id; ++id)
        {
            marker.id = id;
            marker.header.stamp = ros::Time::now();
            publish(marker);

            if (id % 100 == 0)
            {
                ros::spinOnce();
                arc_helpers::Sleep(0.001);
            }
        }

        ros::spinOnce();
        arc_helpers::Sleep(0.001);
    }
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Visualizer::NamespaceId> Visualizer::visualizePoint(
        const std::string& marker_name,
        const Eigen::Vector3d& point,
        const std_msgs::ColorRGBA& color,
        const int32_t id,
        const double scale) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    const EigenHelpers::VectorVector3d points(1, point);
    return visualizePoints(marker_name, points, color, id, scale);
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizePoints(
        const std::string& marker_name,
        const EigenHelpers::VectorVector3d& points,
        const std_msgs::ColorRGBA& color,
        const int32_t id,
        const double scale) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker = createMarker(marker_name, points, id);

    marker.type = vm::Marker::POINTS;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.color = color;

    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizePoints(
        const std::string& marker_name,
        const EigenHelpers::VectorVector3d& points,
        const std::vector<std_msgs::ColorRGBA>& colors,
        const int32_t id,
        const double scale) const
{
    if (disable_all_visualizations_)
    {

        return {};
    }
    vm::Marker marker = createMarker(marker_name, points, id);

    marker.type = vm::Marker::POINTS;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.colors = colors;

    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizePoints(
        const std::string& marker_name,
        const ObjectPointSet& points,
        const std_msgs::ColorRGBA& color,
        const int32_t id,
        const double scale) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker = createMarker(marker_name, points, id);

    marker.type = vm::Marker::POINTS;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.color = color;

    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizePoints(
        const std::string& marker_name,
        const ObjectPointSet& points,
        const std::vector<std_msgs::ColorRGBA>& colors,
        const int32_t id,
        const double scale) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker = createMarker(marker_name, points, id);

    marker.type = vm::Marker::POINTS;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.colors = colors;

    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeCubes(
        const std::string& marker_name,
        const EigenHelpers::VectorVector3d& points,
        const Eigen::Vector3d& scale,
        const std::vector<std_msgs::ColorRGBA>& colors,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker = createMarker(marker_name, points, id);

    marker.type = vm::Marker::CUBE_LIST;
    marker.scale = ehc::EigenVector3dToGeometryVector3(scale);
    marker.colors = colors;

    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeCubes(
        const std::string& marker_name,
        const EigenHelpers::VectorVector3d& points,
        const Eigen::Vector3d& scale,
        const std_msgs::ColorRGBA& color,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker = createMarker(marker_name, points, id);

    marker.type = vm::Marker::CUBE_LIST;
    marker.scale = ehc::EigenVector3dToGeometryVector3(scale);
    marker.color = color;

    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeSpheres(
        const std::string& marker_name,
        const EigenHelpers::VectorVector3d& points,
        const std_msgs::ColorRGBA& color,
        const int32_t id,
        const double radius) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker = createMarker(marker_name, points, id);

    marker.type = vm::Marker::SPHERE_LIST;
    marker.scale = au::rmb::MakeVector3(radius * 2.0, radius * 2.0, radius * 2.0);
    marker.color = color;

    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeSpheres(
        const std::string& marker_name,
        const EigenHelpers::VectorVector3d& points,
        const std_msgs::ColorRGBA& color,
        const int32_t starting_id,
        const std::vector<double>& radiuses) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    const std::vector<std_msgs::ColorRGBA> colors(points.size(), color);
    return visualizeSpheres(marker_name, points, colors, starting_id, radiuses);
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeSpheres(
        const std::string& marker_name,
        const EigenHelpers::VectorVector3d& points,
        const std::vector<std_msgs::ColorRGBA>& colors,
        const int32_t starting_id,
        const std::vector<double>& radiuses) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    if (points.size() != radiuses.size())
    {
        ROS_ERROR_NAMED("visualizer", "Invalid sphere list, need number of points and radiuses to match");
    }

    vm::Marker marker;
    marker.header.frame_id = world_frame_name_;
    marker.header.stamp = ros::Time::now();

    std::vector<NamespaceId> marker_ids;
    for (size_t idx = 0; idx < points.size(); ++idx)
    {
        marker.type = vm::Marker::SPHERE;
        marker.ns = marker_name;
        marker.id = starting_id + (int32_t)idx;
        marker.scale.x = radiuses[idx] * 2.0;
        marker.scale.y = radiuses[idx] * 2.0;
        marker.scale.z = radiuses[idx] * 2.0;
        marker.pose.position = ehc::EigenVector3dToGeometryPoint(points[idx]);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color = colors[idx];

        publish(marker);
        marker_ids.push_back(NamespaceId{marker.ns, marker.id});
    }
    return marker_ids;
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeCapsuleRope(
        const std::string& marker_name,
        const EigenHelpers::VectorIsometry3d& rope_node_transforms,
        const std_msgs::ColorRGBA& color,
        const int32_t id) const
{
    static const double rope_radius = GetRopeRadius(*nh_);
    static const double rope_segment_length = GetRopeSegmentLength(*nh_);

    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker = createMarker(marker_name, EigenHelpers::VectorVector3d{}, id);

    marker.type = vm::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://smmap_utilities/meshes/capsule.stl";
    marker.mesh_use_embedded_materials = false;
    marker.color = color;

    marker.scale.x = rope_radius * 2.0;
    marker.scale.y = rope_radius * 2.0;
    marker.scale.z = rope_segment_length;

    std::vector<NamespaceId> marker_ids;
    for (size_t idx = 0; idx < rope_node_transforms.size(); ++idx)
    {
        marker.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(
                    rope_node_transforms[idx] * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY()));
        publish(marker);
        marker_ids.push_back(NamespaceId{marker.ns, marker.id});
        marker.id++;
    }
    return marker_ids;
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeCapsuleRope(
        const std::string& marker_name,
        const EigenHelpers::VectorIsometry3d& rope_node_transforms,
        const std::vector<std_msgs::ColorRGBA>& colors,
        const int32_t id) const
{
    assert(rope_node_transforms.size() == colors.size());

    static const double rope_radius = GetRopeRadius(*nh_);
    static const double rope_segment_length = GetRopeSegmentLength(*nh_);

    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker = createMarker(marker_name, EigenHelpers::VectorVector3d{}, id);

    marker.type = vm::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://smmap_utilities/meshes/capsule.stl";
    marker.mesh_use_embedded_materials = false;

    marker.scale.x = rope_radius * 2.0;
    marker.scale.y = rope_radius * 2.0;
    marker.scale.z = rope_segment_length;

    std::vector<NamespaceId> marker_ids;
    for (size_t idx = 0; idx < rope_node_transforms.size(); ++idx)
    {
        marker.color = colors[idx];
        marker.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(
                    rope_node_transforms[idx] * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY()));
        publish(marker);
        marker_ids.push_back(NamespaceId{marker.ns, marker.id});
        marker.id++;
    }
    return marker_ids;
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeRope(
        const std::string& marker_name,
        const ObjectPointSet& rope,
        const std_msgs::ColorRGBA& color,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker = createMarker(marker_name, rope, id);
    std::vector<NamespaceId> marker_ids;

    marker.type = vm::Marker::POINTS;
    marker.scale.x = ROPE_POINTS_SCALE;
    marker.scale.y = ROPE_POINTS_SCALE;
    marker.color = color;

    publish(marker);
    marker_ids.push_back(NamespaceId{marker.ns, marker.id});

    marker.type = vm::Marker::LINE_STRIP;
    marker.id++;
    publish(marker);
    marker_ids.push_back(NamespaceId{marker.ns, marker.id});

    return marker_ids;
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeRope(
        const std::string& marker_name,
        const ObjectPointSet& rope,
        const std::vector<std_msgs::ColorRGBA>& colors,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker = createMarker(marker_name, rope, id);
    std::vector<NamespaceId> marker_ids;

    marker.type = vm::Marker::POINTS;
    marker.scale.x = ROPE_POINTS_SCALE;
    marker.scale.y = ROPE_POINTS_SCALE;
    marker.colors = colors;

    publish(marker);
    marker_ids.push_back(NamespaceId{marker.ns, marker.id});

    marker.type = vm::Marker::LINE_STRIP;
    marker.id++;
    publish(marker);
    marker_ids.push_back(NamespaceId{marker.ns, marker.id});

    return marker_ids;
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeCloth(
        const std::string& marker_name,
        const ObjectPointSet& cloth,
        const std_msgs::ColorRGBA& color,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    return visualizePoints(marker_name, cloth, color, id, CLOTH_POINTS_SCALE);
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeCloth(
        const std::string& marker_name,
        const ObjectPointSet& cloth,
        const std::vector<std_msgs::ColorRGBA>& colors,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    return visualizePoints(marker_name, cloth, colors, id, CLOTH_POINTS_SCALE);
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeGripper(
        const std::string& marker_name,
        const Eigen::Isometry3d& eigen_pose,
        const std_msgs::ColorRGBA& color,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker;

    marker.header.frame_id = world_frame_name_;
    marker.header.stamp = ros::Time::now();

    marker.type = vm::Marker::CUBE_LIST;
    marker.ns = marker_name;
    marker.id = id;
    marker.scale.x = GRIPPER_X_SCALE;
    marker.scale.y = GRIPPER_Y_SCALE;
    marker.scale.z = GRIPPER_Z_SCALE;
    marker.pose = ehc::EigenIsometry3dToGeometryPose(eigen_pose);
    marker.color = color;

    marker.points.push_back(au::rmb::MakePoint(0.0, 0.0, gripper_apperture_ * 0.5));
    marker.points.push_back(au::rmb::MakePoint(0.0, 0.0, -gripper_apperture_ * 0.5));

    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeGrippers(
        const std::string& marker_name,
        const EigenHelpers::VectorIsometry3d eigen_poses,
        const std_msgs::ColorRGBA& color,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    std::vector<NamespaceId> marker_ids;
    for (int32_t idx = 0; idx < (int32_t)eigen_poses.size(); ++idx)
    {
        const auto new_ids = visualizeGripper(marker_name, eigen_poses[idx], color, id + idx);
        marker_ids.insert(marker_ids.end(),
                          std::make_move_iterator(new_ids.begin()),
                          std::make_move_iterator(new_ids.end()));
    }
    return marker_ids;
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeGrippers(
        const std::string& marker_name,
        const std::pair<Eigen::Vector3d, Eigen::Vector3d> eigen_positions,
        const std_msgs::ColorRGBA& color,
        const int32_t id) const
{
    EigenHelpers::VectorIsometry3d eigen_poses(2, Eigen::Isometry3d::Identity());
    eigen_poses[0].translation() = eigen_positions.first;
    eigen_poses[1].translation() = eigen_positions.second;
    return visualizeGrippers(marker_name, eigen_poses, color, id);
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeObjectDelta(
        const std::string& marker_name,
        const ObjectPointSet& current,
        const ObjectPointSet& desired,
        const std_msgs::ColorRGBA& color,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker;

    marker.header.frame_id = world_frame_name_;
    marker.header.stamp = ros::Time::now();

    marker.type = vm::Marker::LINE_LIST;
    marker.ns = marker_name;
    marker.id = id;
    marker.scale.x = OBJECT_DELTA_SCALE;
    marker.points.reserve((size_t)current.cols() * 2);
    for (ssize_t col = 0; col < current.cols(); col++)
    {
        marker.points.push_back(ehc::EigenVector3dToGeometryPoint(current.col(col)));
        marker.points.push_back(ehc::EigenVector3dToGeometryPoint(desired.col(col)));
    }
    marker.color = color;

    // Assumes that all non specified values are 0.0
    marker.pose.orientation.w = 1.0;

    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeTranslation(
        const std::string& marker_name,
        const geometry_msgs::Point& start,
        const geometry_msgs::Point& end,
        const std_msgs::ColorRGBA& color,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker;

    marker.header.frame_id = world_frame_name_;
    marker.header.stamp = ros::Time::now();

    marker.type = vm::Marker::LINE_STRIP;
    marker.ns = marker_name;
    marker.id = id;
    marker.scale.x = TRANSLATION_SCALE;
    marker.points.push_back(start);
    marker.points.push_back(end);
    marker.color = color;

    // Assumes that all non specified values are 0.0
    marker.pose.orientation.w = 1.0;

    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeTranslation(
        const std::string& marker_name,
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& end,
        const std_msgs::ColorRGBA& color,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    return visualizeTranslation(
                marker_name,
                ehc::EigenVector3dToGeometryPoint(start),
                ehc::EigenVector3dToGeometryPoint(end),
                color,
                id);
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeTranslation(
        const std::string& marker_name,
        const Eigen::Isometry3d &start,
        const Eigen::Isometry3d &end,
        const std_msgs::ColorRGBA& color,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    return Visualizer::visualizeTranslation(
                marker_name,
                start.translation(),
                end.translation(),
                color,
                id);
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeArrows(
        const std::string& marker_name,
        const EigenHelpers::VectorVector3d& start,
        const EigenHelpers::VectorVector3d& end,
        const std_msgs::ColorRGBA& color,
        const double scale_x,
        const double scale_y,
        const double scale_z,
        const int32_t start_id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    assert(start.size() == end.size());

    vm::MarkerArray msg;
    msg.markers.reserve(start.size());

    vm::Marker marker;
    marker.header.frame_id = world_frame_name_;
    marker.header.stamp = ros::Time::now();

    // Assumes that all non specified values are 0.0
    marker.pose.orientation.w = 1.0;

    marker.type = vm::Marker::ARROW;
    marker.action = vm::Marker::ADD;
    marker.ns = marker_name;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;
    marker.color = color;

    std::vector<NamespaceId> marker_ids;
    for (size_t ind = 0; ind < start.size(); ind++)
    {
        marker.id = start_id + (int32_t)ind;

        marker.points = {
            ehc::EigenVector3dToGeometryPoint(start[ind]),
            ehc::EigenVector3dToGeometryPoint(end[ind])};

        msg.markers.push_back(marker);
        marker_ids.push_back(NamespaceId{marker.ns, marker.id});
    }

    publish(msg);
    return marker_ids;
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeLines(
        const std::string& marker_name,
        const EigenHelpers::VectorVector3d& start,
        const EigenHelpers::VectorVector3d& end,
        const std_msgs::ColorRGBA& color,
        const int32_t id,
        const double scale) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    assert(start.size() == end.size());

    vm::Marker marker;

    marker.header.frame_id = world_frame_name_;

    marker.type = vm::Marker::LINE_LIST;
    marker.ns = marker_name;
    marker.id = id;
    marker.scale.x = scale;

    for (size_t ind = 0; ind < start.size(); ind++)
    {
        marker.points.push_back(ehc::EigenVector3dToGeometryPoint(start[ind]));
        marker.points.push_back(ehc::EigenVector3dToGeometryPoint(end[ind]));
        marker.colors.push_back(color);
        marker.colors.push_back(color);
    }

    // Assumes that all non specified values are 0.0
    marker.pose.orientation.w = 1.0;

    marker.header.stamp = ros::Time::now();
    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeLines(
        const std::string& marker_name,
        const EigenHelpers::VectorVector3d& start,
        const EigenHelpers::VectorVector3d& end,
        const std::vector<std_msgs::ColorRGBA>& colors,
        const int32_t id,
        const double scale) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    assert(start.size() == end.size());
    assert(start.size() == colors.size());

    vm::Marker marker;

    marker.header.frame_id = world_frame_name_;

    marker.type = vm::Marker::LINE_LIST;
    marker.ns = marker_name;
    marker.id = id;
    marker.scale.x = scale;

    for (size_t ind = 0; ind < start.size(); ind++)
    {
        marker.points.push_back(ehc::EigenVector3dToGeometryPoint(start[ind]));
        marker.points.push_back(ehc::EigenVector3dToGeometryPoint(end[ind]));
        marker.colors.push_back(colors[ind]);
        marker.colors.push_back(colors[ind]);
    }

    // Assumes that all non specified values are 0.0
    marker.pose.orientation.w = 1.0;

    marker.header.stamp = ros::Time::now();
    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeLineStrip(
        const std::string& marker_name,
        const EigenHelpers::VectorVector3d& point_sequence,
        const std_msgs::ColorRGBA& color,
        const int32_t id,
        const double scale) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker = createMarker(marker_name, point_sequence, id);

    marker.type = vm::Marker::LINE_STRIP;
    marker.scale.x = scale;
    marker.colors = std::vector<std_msgs::ColorRGBA>(marker.points.size(), color);

    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeLineStrip(
        const std::string& marker_name,
        const ObjectPointSet& point_sequence,
        const std_msgs::ColorRGBA& color,
        const int32_t id,
        const double scale) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker = createMarker(marker_name, point_sequence, id);

    marker.type = vm::Marker::LINE_STRIP;
    marker.scale.x = scale;
    marker.colors = std::vector<std_msgs::ColorRGBA>(marker.points.size(), color);

    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeXYZTrajectory(
        const std::string& marker_name,
        const EigenHelpers::VectorVector3d& point_sequence,
        const std_msgs::ColorRGBA& color,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    return visualizeLineStrip(marker_name, point_sequence, color, id, 0.002);
}

std::vector<Visualizer::NamespaceId> Visualizer::visualizeAxes(
        const std::string& marker_name,
        const Eigen::Isometry3d& axes,
        const double length,
        const double thickness,
        const int32_t id) const
{
    if (disable_all_visualizations_)
    {
        return {};
    }
    vm::Marker marker;
    marker.header.frame_id = world_frame_name_;

    marker.type = vm::Marker::LINE_LIST;
    marker.ns = marker_name;
    marker.id = id;
    marker.scale.x = thickness;

    marker.pose = ehc::EigenIsometry3dToGeometryPose(axes);
    // X-axis
    marker.points.push_back(au::rmb::MakePoint(0.0, 0.0, 0.0));
    marker.points.push_back(au::rmb::MakePoint(length, 0.0, 0.0));
    marker.colors.push_back(Red());
    marker.colors.push_back(Red());
    // Y-axis
    marker.points.push_back(au::rmb::MakePoint(0.0, 0.0, 0.0));
    marker.points.push_back(au::rmb::MakePoint(0.0, length, 0.0));
    marker.colors.push_back(Green());
    marker.colors.push_back(Green());
    // Z-axis
    marker.points.push_back(au::rmb::MakePoint(0.0, 0.0, 0.0));
    marker.points.push_back(au::rmb::MakePoint(0.0, 0.0, length));
    marker.colors.push_back(Blue());
    marker.colors.push_back(Blue());

    marker.header.stamp = ros::Time::now();
    publish(marker);
    return {NamespaceId{marker.ns, marker.id}};
}

////////////////////////////////////////////////////////////////////////////////

vm::Marker Visualizer::createMarker(
        const std::string& marker_name,
        const EigenHelpers::VectorVector3d& points,
        const int32_t id) const
{
    vm::Marker marker;

    marker.header.frame_id = world_frame_name_;
    marker.header.stamp = ros::Time::now();

    marker.type = vm::Marker::POINTS;
    marker.ns = marker_name;
    marker.id = id;
    marker.points = ehc::VectorEigenVector3dToVectorGeometryPoint(points);

    // Assumes that all non specified values are 0.0
    marker.pose.orientation.w = 1.0;

    return marker;
}

vm::Marker Visualizer::createMarker(
        const std::string& marker_name,
        const ObjectPointSet& points,
        const int32_t id) const
{
    vm::Marker marker;

    marker.header.frame_id = world_frame_name_;
    marker.header.stamp = ros::Time::now();

    marker.type = vm::Marker::POINTS;
    marker.ns = marker_name;
    marker.id = id;
    marker.points = ehc::EigenMatrix3XdToVectorGeometryPoint(points);

    // Assumes that all non specified values are 0.0
    marker.pose.orientation.w = 1.0;

    return marker;
}

////////////////////////////////////////////////////////////////////////////////

// Shall only be used if the markers_mtx has already been acquired
void Visualizer::updateMarkerList(const vm::Marker& marker) const
{
    bool marker_found = false;
    for (size_t idx = 0; idx < async_markers_.markers.size(); ++idx)
    {
        vm::Marker& old_marker = async_markers_.markers[idx];
        if (old_marker.id == marker.id && old_marker.ns == marker.ns)
        {
            old_marker = marker;
            marker_found = true;
            break;
        }
    }

    if (!marker_found)
    {
        async_markers_.markers.push_back(marker);
    }
}

void Visualizer::publishAsyncMain()
{
    running_async_ = true;
    const auto freq = ROSHelpers::GetParam<double>(*ph_, "async_publish_frequency", 5.0);
    ros::Rate rate(freq);
    while (running_async_ && ros::ok())
    {
        {
            std::lock_guard<std::mutex> lock(markers_mtx_);
            if (!async_markers_.markers.empty())
            {
                visualization_maker_array_pub_.publish(async_markers_);
            }
        }
        rate.sleep();
    }
}
