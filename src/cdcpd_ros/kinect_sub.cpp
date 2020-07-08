#include "cdcpd_ros/kinect_sub.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

KinectSub::KinectSub(const std::function<void(cv::Mat, cv::Mat, cv::Matx33d)>& _externCallback,
                     const SubscriptionOptions _options)
    : options(_options)
    , externCallback(_externCallback)
    , callbackQueue()
    , spinner(1, &callbackQueue)
{
    options.nh.setCallbackQueue(&callbackQueue);
    options.pnh.setCallbackQueue(&callbackQueue);

    if (_options.hints.getTransport() == "compressed")
    {
        // TODO: when creating these subscribers, both the rgb and depth try to
        //       create a `cdcpd_node/compressed/set_parameters` service, this is
        //       presumably not an issue for now, but it is messy
        ROS_INFO("Ignore the 'Tried to advertise a service that is already advertised'"
                " ... message, see cpp file.");
    }
    it = std::make_unique<image_transport::ImageTransport>(options.nh);
    rgb_sub = std::make_unique<image_transport::SubscriberFilter>(
            *it, options.rgb_topic, options.queue_size, options.hints);
    depth_sub = std::make_unique<image_transport::SubscriberFilter>(
            *it, options.depth_topic, options.queue_size, options.hints);
    cam_sub = std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>(
            options.nh, options.cam_topic, options.queue_size);

    sync = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(options.queue_size), *rgb_sub, *depth_sub, *cam_sub);
    sync->registerCallback(boost::bind(&KinectSub::imageCb, this, _1, _2, _3));

    spinner.start();
}

void KinectSub::imageCb(const sensor_msgs::ImageConstPtr& rgb_msg,
                        const sensor_msgs::ImageConstPtr& depth_msg,
                        const sensor_msgs::CameraInfoConstPtr& cam_msg)
{
    cv_bridge::CvImagePtr cv_rgb_ptr;
    try
    {
        cv_rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("RGB cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImagePtr cv_depth_ptr;
    try
    {
        cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Depth cv_bridge exception: %s", e.what());
        return;
    }

    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        cv::Mat convertedDepthImg(cv_depth_ptr->image.size(), CV_16UC1);

        const int V = cv_depth_ptr->image.size().height;
        const int U = cv_depth_ptr->image.size().width;

        for (int v = 0; v < V; ++v)
        {
            for (int u = 0; u < U; ++u)
            {
                convertedDepthImg.at<uint16_t>(v, u) =
                        depth_image_proc::DepthTraits<uint16_t>::fromMeters(
                            cv_depth_ptr->image.at<float>(v, u));
            }
        }

        cv_depth_ptr->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        cv_depth_ptr->image = convertedDepthImg;
    }

    if (externCallback)
    {
        image_geometry::PinholeCameraModel cameraModel;
        cameraModel.fromCameraInfo(cam_msg);
        externCallback(cv_rgb_ptr->image, cv_depth_ptr->image, cameraModel.fullIntrinsicMatrix());
    }
}

