#include <string>
#include <cdcpd/cdcpd.h>
#include <ros/ros.h>
#include "cdcpd_ros/kinect_sub.h"

auto constexpr PARAM_NAME_WIDTH = 40;

template <typename T>
inline T GetParam(const ros::NodeHandle& nh,
                  const std::string& param_name,
                  const T& default_val)
{
    T param_val;
    if (nh.getParam(param_name, param_val))
    {
        ROS_INFO_STREAM_NAMED("params",
                "Retrieving " << std::left << std::setw(PARAM_NAME_WIDTH)
                << param_name << " as " << param_val);
    }
    else
    {
        param_val = default_val;
        ROS_WARN_STREAM_NAMED("params",
                "Defaulting " << std::left << std::setw(PARAM_NAME_WIDTH)
                << param_name << " to " << param_val);
    }
    return param_val;
}


void callback_fn(cv::Mat, cv::Mat, cv::Matx33d)
{
    // TODO
    ROS_INFO("Got message!!!!");
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cdcpd_node");

    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    std::string const kinect_name = GetParam<std::string>(ph, "pov", "victor_head");
    std::string const stream = GetParam<std::string>(ph, "stream", "qhd");

    KinectSub kinect_sub(callback_fn,
                         KinectSub::SubscriptionOptions("kinect2_" + kinect_name + "/" + stream));
    ros::spin();

    return EXIT_SUCCESS;
}
