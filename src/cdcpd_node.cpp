#include <string>
#include <cdcpd/cdcpd.h>
#include <ros/ros.h>
// #include "cdcpd_ros/kinect_sub.h"
#include <thread>
#include <chrono>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <rosbag/bag.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include "cdcpd/cdcpd.h"

using std::cout;
using std::endl;
using Eigen::MatrixXf;
using Eigen::MatrixXi;

using namespace cv;

// auto constexpr PARAM_NAME_WIDTH = 40;
// 
// template <typename T>
// inline T GetParam(const ros::NodeHandle& nh,
//                   const std::string& param_name,
//                   const T& default_val)
// {
//     T param_val;
//     if (nh.getParam(param_name, param_val))
//     {
//         ROS_INFO_STREAM_NAMED("params",
//                 "Retrieving " << std::left << std::setw(PARAM_NAME_WIDTH)
//                 << param_name << " as " << param_val);
//     }
//     else
//     {
//         param_val = default_val;
//         ROS_WARN_STREAM_NAMED("params",
//                 "Defaulting " << std::left << std::setw(PARAM_NAME_WIDTH)
//                 << param_name << " to " << param_val);
//     }
//     return param_val;
// }

// void callback_fn(cv::Mat, cv::Mat, cv::Matx33d)
// {
//     // TODO
//     ROS_INFO("Got message!!!!");
// }

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cdcpd_node");

    rosbag::Bag bag;
    bag.open("data/dark-room-rope.bag", rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/kinect2/qhd/image_color_rect"));
    topics.push_back(std::string("/kinect2/qhd/image_depth_rect"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(rosbag::MessageInstance const m: view)
    {
        cout << "topic: " << m.getTopic() << endl;
        sensor_msgs::Image::ConstPtr i = m.instantiate<sensor_msgs::Image>();
        if (m.getTopic() == topics[0])
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(i, sensor_msgs::image_encodings::BGR8);
            cout << "color timestamp: " << m.getTime() << endl;
            cv::Mat color_image = cv_ptr->image;
        }
        else if (m.getTopic() == topics[1])
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(i, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat depth_image = cv_ptr->image;
            cout << "depth timestamp: " << m.getTime() << endl;
        }
        else
        {
            cerr << "Invalid topic: " << m.getTopic() << endl;
            exit(1);
        }
        assert(i != nullptr);
        if (i != nullptr)
        {
            std::cout << i << std::endl;
        }
    }
    
    bag.close();

    cout << "Test started" << endl;

    MatrixXf template_vertices(3, 50);
    template_vertices.setZero();
    template_vertices.row(0).setLinSpaced(50, 0, 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < template_vertices.cols(); ++i)
    {
        const auto& c = template_vertices.col(i);
        template_cloud->push_back(pcl::PointXYZ(c(0), c(1), c(2)));
    }

    MatrixXi template_edges(2, 49);
    template_edges(0, 0) = 0;
    template_edges(1, template_edges.cols() - 1) = 49;
    for (int i = 1; i <= template_edges.cols() - 1; ++i)
    {
        template_edges(0, i) = i;
        template_edges(1, i - 1) = i;
    }

    FileStorage color_calib_fs("../../kinect2_calibration_files/data/000792364047/calib_color.yaml", FileStorage::READ);
    cv::Mat intrinsics;
    color_calib_fs["cameraMatrix"] >> intrinsics;
    // cout << "intrinsics type: " << intrinsics.type() << endl;

    CDCPD cdcpd(template_cloud, intrinsics);

    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    ros::Publisher original_publisher = ph.advertise<PointCloud> ("original", 1);
    ros::Publisher masked_publisher = ph.advertise<PointCloud> ("masked", 1);
    ros::Publisher downsampled_publisher = ph.advertise<PointCloud> ("downsampled", 1);
    ros::Publisher template_publisher = ph.advertise<PointCloud> ("template", 1);
    ros::Publisher cpd_iters_publisher = ph.advertise<PointCloud> ("cpd_iters", 1);
    ros::Publisher output_publisher = ph.advertise<PointCloud> ("output", 1);

    for (int i = 0; i < 30; ++i)
    {

        /// Color filter
        // For the red rope, (h > 0.85) & (s > 0.5). For the flag, (h < 1.0) & (h > 0.9)
        // The flag isn't tested or implemented
        Mat color_image_bgr = imread("../../../data/image_color_rect_screenshot_10.03.2020.png", IMREAD_COLOR);
        Mat rgb_image;
        cv::cvtColor(color_image_bgr, rgb_image, cv::COLOR_BGR2RGB);
        // TODO I'm pretty sure this is an 8-bit image.
        Mat depth_image = imread("../../../data/image_depth_rect_screenshot_10.03.2020.png", IMREAD_UNCHANGED);

        imwrite("rgb.png", rgb_image);
        imwrite("depth.png", depth_image);

        cv::Mat rgb_f;
        rgb_image.convertTo(rgb_f, CV_32FC3);
        rgb_f /= 255.0; // get RGB 0.0-1.0
        cv::Mat color_hsv;
        cvtColor(rgb_f, color_hsv, CV_RGB2HSV);
        cv::Scalar low_hsv = cv::Scalar(.85 * 360.0, 0.5, 0.0);
        cv::Scalar high_hsv = cv::Scalar(360.0, 1.0, 1.0);
        cv::Mat hsv_mask;
        cv::inRange(color_hsv, low_hsv, high_hsv, hsv_mask);
        cv::imwrite("hsv_mask.png", hsv_mask);

        CDCPD::Output out = cdcpd(rgb_image, depth_image, hsv_mask, template_cloud, template_edges);
        template_cloud = out.gurobi_output;

        auto frame_id = "map";
        out.original_cloud->header.frame_id = frame_id;
        out.masked_point_cloud->header.frame_id = frame_id;
        out.downsampled_cloud->header.frame_id = frame_id;
        out.last_template->header.frame_id = frame_id;
        out.cpd_iterations[0]->header.frame_id = frame_id;
        out.gurobi_output->header.frame_id = frame_id;

        auto time = ros::Time::now();
        pcl_conversions::toPCL(time, out.original_cloud->header.stamp);
        pcl_conversions::toPCL(time, out.masked_point_cloud->header.stamp);
        pcl_conversions::toPCL(time, out.downsampled_cloud->header.stamp);
        pcl_conversions::toPCL(time, out.last_template->header.stamp);
        pcl_conversions::toPCL(time, out.cpd_iterations[0]->header.stamp);
        pcl_conversions::toPCL(time, out.gurobi_output->header.stamp);

        original_publisher.publish(out.original_cloud);
        masked_publisher.publish(out.masked_point_cloud);
        downsampled_publisher.publish(out.downsampled_cloud);
        template_publisher.publish(out.last_template);
        cpd_iters_publisher.publish(out.cpd_iterations[0]); // TODO all
        output_publisher.publish(out.gurobi_output);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    cout << "Test ended" << endl;
    // std::string const kinect_name = GetParam<std::string>(ph, "pov", "victor_head");
    // std::string const stream = GetParam<std::string>(ph, "stream", "qhd");

    // KinectSub kinect_sub(callback_fn,
    //                      KinectSub::SubscriptionOptions("kinect2_" + kinect_name + "/" + stream));
    ros::spin();

    return EXIT_SUCCESS;
}
