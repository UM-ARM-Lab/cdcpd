#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/transform_listener.h>
#include "cdcpd/eigen_ros_conversions.hpp"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef Eigen::Isometry3d Pose;

ros::Publisher pub;
tf2_ros::Buffer tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener;
std::string table_frame = "table_surface";

Pose lookupTransform(
    tf2_ros::Buffer& buffer,
    std::string const& parent_frame,
    std::string const& child_frame,
    ros::Time const& target_time)
    // ros::Duration const& timeout)
{
    // Wait for up to timeout amount of time, then try to lookup the transform,
    // letting TF2's exception handling throw if needed
    // buffer.canTransform(parent_frame, child_frame, target_time, timeout);
    auto const tform = buffer.lookupTransform(parent_frame, child_frame, target_time);
    return ConvertTo<Pose>(tform.transform);
}

void cloud_cb(const PointCloud::ConstPtr& input_cloud)
{
    if (!tfBuffer.canTransform(table_frame, input_cloud->header.frame_id, ros::Time(0)))
    {
        ROS_WARN_STREAM("Dropping input cloud, cannot transform from "
                        << input_cloud->header.frame_id << " to " << table_frame);
        return;
    }

    // Record the transform in both directions so that we leave things in the frame they arrived in
    Eigen::Isometry3f const target_tf = lookupTransform(
        tfBuffer, table_frame, input_cloud->header.frame_id, ros::Time(0)).cast<float>();
    Eigen::Isometry3f const target_tf_inv = target_tf.inverse(Eigen::Isometry);

    // TODO: replace with pcl::copyPointCloud?
    auto output_cloud = boost::make_shared<PointCloud>();
    output_cloud->header = input_cloud->header;
    // Note that we do keep it in the input frame,
    // otherwise cdcpd has problems later

    output_cloud->height = input_cloud->height;
    output_cloud->width = input_cloud->width;
    output_cloud->is_dense = input_cloud->is_dense;
    output_cloud->sensor_origin_ = input_cloud->sensor_origin_;
    output_cloud->sensor_orientation_ = input_cloud->sensor_orientation_;
    output_cloud->points.resize(input_cloud->points.size());

    for (size_t idx = 0; idx < output_cloud->points.size(); ++idx)
    {
        // Copy the RGB data
        output_cloud->points[idx].rgba = input_cloud->points[idx].rgba;
        // Transform the XYZ data - Don't use the vector4f map
        Eigen::Vector3f point_in_frame = target_tf * input_cloud->points[idx].getVector3fMap();
        point_in_frame.z() = 0;
        output_cloud->points[idx].getVector3fMap() = target_tf_inv * point_in_frame;
    }

    pub.publish(output_cloud);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "project_to_table");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);

    // Our output is cdcpd's input
    std::string source_cloud_topic = "should_never_return_this";
    std::string cdcpd_input_cloud_topic = "should_never_return_this";
    ph.getParam("source_cloud_topic", source_cloud_topic);
    ph.getParam("intermediary_topic", cdcpd_input_cloud_topic);
    assert(source_cloud_topic != "should_never_return_this");
    assert(cdcpd_input_cloud_topic != "should_never_return_this");
    ROS_INFO_STREAM("Subscribing to " << source_cloud_topic);
    ROS_INFO_STREAM("Publishing to " << cdcpd_input_cloud_topic);

    ros::Subscriber sub = nh.subscribe(source_cloud_topic, 1, cloud_cb);
    pub = nh.advertise<PointCloud>(cdcpd_input_cloud_topic, 1);
    ros::spin();

    return EXIT_SUCCESS;
}

