#ifndef KINECT_SUB_H
#define KINECT_SUB_H

#include <string>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <depth_image_proc/depth_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

class KinectSub
{
    public:
        using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;
	    using DepthTraits = depth_image_proc::DepthTraits<uint16_t>;

        struct SubscriptionOptions
        {
            ros::NodeHandle nh;
            ros::NodeHandle pnh;
            image_transport::TransportHints hints;
            int queue_size;
            std::string topic_prefix;
            std::string rgb_topic;
            std::string depth_topic;
            std::string cam_topic;

            explicit
            SubscriptionOptions(const std::string& prefix = "kinect2_victor_head/hd")
                : nh()
                , pnh("~")
                , hints("compressed", ros::TransportHints(), pnh)
                , queue_size(1)
                , topic_prefix(prefix)
                , rgb_topic(topic_prefix + "/image_color_rect")
                , depth_topic(topic_prefix + "/image_depth_rect")
                , cam_topic(topic_prefix + "/camera_info")
            {}
        };

        std::unique_ptr<image_transport::ImageTransport> it;
        std::unique_ptr<image_transport::SubscriberFilter> rgb_sub;
        std::unique_ptr<image_transport::SubscriberFilter> depth_sub;
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> cam_sub;
        std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

        SubscriptionOptions options;

        std::function<void(cv::Mat, cv::Mat, cv::Matx33d)> externCallback;
        ros::CallbackQueue callbackQueue;
        ros::AsyncSpinner spinner;

        // Callback is in the form (rbg, depth, cameraIntrinsics)
        explicit
        KinectSub(const std::function<void(cv::Mat, cv::Mat, cv::Matx33d)>& _externCallback,
                  const SubscriptionOptions _options = SubscriptionOptions());

        void imageCb(const sensor_msgs::ImageConstPtr& rgb_msg,
                     const sensor_msgs::ImageConstPtr& depth_msg,
                     const sensor_msgs::CameraInfoConstPtr& cam_msg);
};


#endif // KINECT_SUB_H
