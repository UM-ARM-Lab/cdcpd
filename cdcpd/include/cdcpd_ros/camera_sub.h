#ifndef KINECT_SUB_H
#define KINECT_SUB_H

#include <depth_image_proc/depth_conversions.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <memory>
#include <string>

using SyncPolicy =
    message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;
using DepthTraits = depth_image_proc::DepthTraits<uint16_t>;

struct CameraSubSetup {
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  image_transport::TransportHints hints;
  int queue_size;
  std::string rgb_topic;
  std::string depth_topic;
  std::string info_topic;
  ros::CallbackQueue queue;
  ros::AsyncSpinner spinner;

  explicit CameraSubSetup(const std::string& rgb_topic, const std::string& depth_topic, const std::string& info_topic);
};

class KinectSub {
 public:
  CameraSubSetup& options;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter rgb_sub;
  image_transport::SubscriberFilter depth_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam_sub;
  message_filters::Synchronizer<SyncPolicy> sync;

  std::function<void(cv::Mat, cv::Mat, cv::Matx33d)> externCallback;

  // Callback is in the form (rbg, depth, cameraIntrinsics)
  explicit KinectSub(const std::function<void(cv::Mat, cv::Mat, cv::Matx33d)>& _externCallback,
                     CameraSubSetup& _options);

  void imageCb(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& cam_msg);
};

#endif  // KINECT_SUB_H
