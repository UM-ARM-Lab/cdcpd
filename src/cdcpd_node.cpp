#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <map>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <arc_utilities/ros_helpers.hpp>
#include <cdcpd/cdcpd.h>
#include "cdcpd_ros/kinect_sub.h"

using Eigen::MatrixXf;
using Eigen::MatrixXi;
using Eigen::Matrix3Xf;
using pcl::PointXYZ;

using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char* argv[])
{
    // ENHANCE: more smart way to get Y^0 and E
    ros::init(argc, argv, "cdcpd_ros_node");

    // initial connectivity model of rope
    int points_on_rope = 11;
    float rope_length = 1.0f;
    MatrixXf template_vertices(3, points_on_rope); // Y^0 in the paper
    template_vertices.setZero();
    template_vertices.row(0).setLinSpaced(points_on_rope, -rope_length/2, rope_length/2);
    template_vertices.row(2).array() += 1.4f;
    MatrixXi template_edges(2, points_on_rope - 1);
    template_edges(0, 0) = 0;
    template_edges(1, template_edges.cols() - 1) = points_on_rope - 1;
    for (int i = 1; i <= template_edges.cols() - 1; ++i)
    {
        template_edges(0, i) = i;
        template_edges(1, i - 1) = i;
    }

    // Construct the initial template
    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < template_vertices.cols(); ++i)
    {
        const auto& c = template_vertices.col(i);
        template_cloud->push_back(pcl::PointXYZ(c(0), c(1), c(2)));
    }

    // comes from the default P matrix on the /kinect2/qhd/camera_info channel.
    cv::Mat P_mat(3, 4, CV_64FC1);

    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    ros::Publisher original_publisher = ph.advertise<PointCloud> ("original", 1);
    ros::Publisher masked_publisher = ph.advertise<PointCloud> ("masked", 1);
    ros::Publisher downsampled_publisher = ph.advertise<PointCloud> ("downsampled", 1);
    ros::Publisher template_publisher = ph.advertise<PointCloud> ("template", 1);
    ros::Publisher cpd_iters_publisher = ph.advertise<PointCloud> ("cpd_iters", 1);
    ros::Publisher output_publisher = ph.advertise<PointCloud> ("output", 1);
    ros::Publisher output_without_constrain_publisher = ph.advertise<PointCloud> ("output_without_constrain", 1);
    ros::Publisher left_gripper_pub = nh.advertise<geometry_msgs::TransformStamped>("cdcpd/left_gripper_prior", 1);
    ros::Publisher right_gripper_pub = nh.advertise<geometry_msgs::TransformStamped>("cdcpd/right_gripper_prior", 1);
    ros::Publisher cylinder_pub = ph.advertise<visualization_msgs::Marker>( "cylinder", 0 );
    ros::Publisher order_pub = ph.advertise<visualization_msgs::Marker>("order", 10);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    const double alpha = ROSHelpers::GetParam<double>(ph, "alpha", 0.5);
    const double lambda = ROSHelpers::GetParam<double>(ph, "lambda", 1.0);
    const double k_spring = ROSHelpers::GetParam<double>(ph, "k", 100.0);
    const double beta = ROSHelpers::GetParam<double>(ph, "beta", 1.0);
    const bool use_recovery = ROSHelpers::GetParam<bool>(ph, "use_recovery", false);
    const std::string kinect_channel = ROSHelpers::GetParam<std::string>(ph, "kinect_channel", "kinect2/qhd");

    CDCPD cdcpd(template_cloud, template_edges, P_mat, use_recovery, alpha, beta, lambda, k_spring);
    auto const callback = [&cdcpd] (cv::Mat rgb, cv::Mat depth, cv::Matx33d cam)
    {
        
    };

    auto const options = KinectSub::SubscriptionOptions(kinect_channel);
    KinectSub sub(callback, options);

    // Let's also grab the gripper positions. Note that in practice, you'd do this in real time.
    geometry_msgs::TransformStamped leftTS;
    geometry_msgs::TransformStamped rightTS;

    while (nh.ok())
    {
        try{
            leftTS = tfBuffer.lookupTransform("kinect2_rgb_optical_frame", "cdcpd_ros/left_gripper_prior", ros::Time(0));
            rightTS = tfBuffer.lookupTransform("kinect2_rgb_optical_frame", "cdcpd_ros/right_gripper_prior", ros::Time(0));
            break;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("Tried to get a transform and failed!");
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    // Eigen::Vector3f left_pos((float)leftTS.transform.translation.x, (float)leftTS.transform.translation.y, (float)leftTS.transform.translation.z);
    // Eigen::Vector3f right_pos((float)rightTS.transform.translation.x, (float)rightTS.transform.translation.y, (float)rightTS.transform.translation.z);
    // cout << "Left gripper: " << left_pos << endl;
    // cout << "Right gripper: " << right_pos << endl;
    // CDCPD::FixedPoint left_gripper = { left_pos, (int) template_vertices.cols() - 1 };
    // CDCPD::FixedPoint right_gripper = { right_pos, 0 };

    return EXIT_SUCCESS;
}
