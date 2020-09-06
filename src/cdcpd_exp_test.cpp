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
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/simple_filter.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <arc_utilities/ros_helpers.hpp>
#include <cdcpd/cdcpd.h>

using std::cout;
using std::endl;
using std::string;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::MatrixXi;
using Eigen::Matrix3Xf;
using Eigen::Matrix3Xd;
using Eigen::Isometry3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using pcl::PointXYZ;
using smmap::AllGrippersSinglePose;
using smmap::AllGrippersSinglePoseDelta;
using kinematics::Vector6d;

namespace gm = geometry_msgs;
namespace vm = visualization_msgs;
namespace sm = sensor_msgs;
namespace stdm = std_msgs;

using namespace cv;
using namespace std::chrono_literals;

std::vector<sm::Image::ConstPtr> color_images;
std::vector<sm::Image::ConstPtr> depth_images;
std::vector<sm::CameraInfo::ConstPtr> camera_infos;
std::vector<stdm::Float32MultiArray::ConstPtr> grippers_config;
std::vector<stdm::Float32MultiArray::ConstPtr> grippers_dot;
std::vector<stdm::Float32MultiArray::ConstPtr> grippers_ind;

std::string workingDir = "/home/deformtrack/catkin_ws/src/cdcpd_test/log";

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
    void newMessage(const boost::shared_ptr<M const> &msg)
    {
        this->signalMessage(msg);
    }
};

void callback(
    const sm::Image::ConstPtr &rgb_img,
    const sm::Image::ConstPtr &depth_img,
    const sm::CameraInfo::ConstPtr &cam_info
    // #ifdef PREDICT
    // ,
    // const stdm::Float32MultiArray::ConstPtr &g_config,
    // const stdm::Float32MultiArray::ConstPtr &g_dot,
    // const stdm::Float32MultiArray::ConstPtr &g_ind,
    // const stdm::Float32MultiArray::ConstPtr &one_truth
    // #endif
    )
{
    color_images.push_back(rgb_img);
    depth_images.push_back(depth_img);
    camera_infos.push_back(cam_info);
    // #ifdef PREDICT
    // grippers_config.push_back(g_config);
    // grippers_dot.push_back(g_dot);
    // grippers_ind.push_back(g_ind);
    // ground_truth.push_back(one_truth);
    // #endif
}

std::tuple<cv::Mat, cv::Mat, cv::Matx33d> toOpenCv(
    const sm::Image::ConstPtr &rgb_img,
    const sm::Image::ConstPtr &depth_img,
    const sm::CameraInfo::ConstPtr &cam_info)
{
    #ifdef SIMULATION
    cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(rgb_img, sm::image_encodings::TYPE_8UC3);
    #else
    cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(rgb_img, sm::image_encodings::BGR8);
    #endif
    cv::Mat color_image = rgb_ptr->image.clone();

    #ifdef SIMULATION
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_img, sm::image_encodings::TYPE_32FC1);
    #else
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_img, sm::image_encodings::TYPE_16UC1);
    #endif
    cv::Mat depth_image = depth_ptr->image.clone();

    image_geometry::PinholeCameraModel cameraModel;
    cameraModel.fromCameraInfo(cam_info);

    return { color_image, depth_image, cameraModel.fullIntrinsicMatrix() };
}

Matrix3Xf toGroundTruth(
    const stdm::Float32MultiArray::ConstPtr &one_frame_truth)
{
    uint32_t num_dim = (one_frame_truth->layout).dim[0].size;
    uint32_t num_points = (one_frame_truth->layout).dim[1].size;

    Matrix3Xf one_frame_truth_eigen(num_dim, num_points);
    for (uint32_t pt = 0; pt < num_points; ++pt)
    {
        for (uint32_t dim = 0; dim < num_dim; ++dim)
        {
            one_frame_truth_eigen(dim, pt) = (one_frame_truth->data)[pt*num_dim + dim];
        }
    }

    return one_frame_truth_eigen;
}

std::tuple<AllGrippersSinglePose,
           AllGrippersSinglePoseDelta,
           MatrixXi> toGripperConfig(
    const stdm::Float32MultiArray::ConstPtr &g_config,
    const stdm::Float32MultiArray::ConstPtr &g_dot,
    const stdm::Float32MultiArray::ConstPtr &g_ind)
{
    uint32_t num_gripper = (g_config->layout).dim[0].size;
    uint32_t num_config = (g_config->layout).dim[1].size;
    uint32_t num_dot = (g_dot->layout).dim[1].size;
    uint32_t num_ind = (g_ind->layout).dim[1].size;

    std::cout << "num of gripper " << num_gripper << std::endl;
    std::cout << "num of config " << num_config << std::endl;
    std::cout << "num of gripper dot " << num_dot << std::endl;
    std::cout << "num of gripper index " << num_ind << std::endl;

    AllGrippersSinglePose one_frame_config;
    AllGrippersSinglePoseDelta one_frame_velocity;
    MatrixXi one_frame_ind(num_ind, num_gripper);

    for (uint32_t g = 0; g < num_gripper; ++g)
    {
        Isometry3d one_config;
        Vector6d one_velocity;

        for (uint32_t row = 0; row < 4; ++row)
        {
            for (uint32_t col = 0; col < 4; ++col)
            {
                one_config(row, col) = double((g_config->data)[num_config*g + row*4 + col]);
            }
        }

        for (uint32_t i = 0; i < num_dot; ++i)
        {
            one_velocity(i) = double((g_dot->data)[num_dot*g + i]);
        }

        for (uint32_t i = 0; i < num_ind; ++i)
        {
            one_frame_ind(i, g) = int((g_ind->data)[num_ind*g + i]);
        }

        one_frame_config.push_back(one_config);
        one_frame_velocity.push_back(one_velocity);
    }

    return {one_frame_config, one_frame_velocity, one_frame_ind};
}

int main(int argc, char* argv[])
{
    // test_nearest_line();
    // test_lle();
    // ENHANCE: more smart way to get Y^0 and E
    //test_velocity_calc();
    ros::init(argc, argv, "cdcpd_bagfile");
    cout << "Starting up..." << endl;

    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    BagSubscriber<sm::Image> rgb_sub, depth_sub;
    BagSubscriber<sm::CameraInfo> info_sub;

    std::vector<std::string> topics;

    topics.push_back(std::string("/kinect2/qhd/image_color_rect"));
    topics.push_back(std::string("/kinect2/qhd/image_depth_rect"));
    topics.push_back(std::string("/kinect2/qhd/camera_info"));
    topics.push_back(std::string("/kinect2/qhd/gripper_velocity"));
    topics.push_back(std::string("/kinect2/qhd/gripper_info"));
    topics.push_back(std::string("/kinect2/qhd/gripper_config"));

    auto const bagfile = ROSHelpers::GetParam<std::string>(ph, "bagfile", "normal");
    auto const folder = ros::package::getPath("cdcpd_ros") + "/../cdcpd_test/dataset/";

    rosbag::Bag bag(folder + bagfile + ".bag", rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    auto sync = message_filters::TimeSynchronizer<sm::Image, sm::Image, sm::CameraInfo>(
            rgb_sub, depth_sub, info_sub, 25);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    for(rosbag::MessageInstance const& m: view)
    {
        if (m.getTopic() == topics[0])
        {
            auto i = m.instantiate<sm::Image>();
            if (i != nullptr)
            {
                rgb_sub.newMessage(i);
            }
            else
            {
                cout << "NULL initiation!" << endl;
            }
        }
        else if (m.getTopic() == topics[1])
        {
            auto i = m.instantiate<sm::Image>();
            if (i != nullptr)
            {
                depth_sub.newMessage(i);
            }
            else
            {
                cout << "NULL initiation!" << endl;
            }
        }
        else if (m.getTopic() == topics[2])
        {
            auto info = m.instantiate<sm::CameraInfo>();
            if (info != nullptr)
            {
                info_sub.newMessage(info);
            }
            else
            {
                cout << "NULL initiation!" << endl;
            }
        }
        else if (m.getTopic() == topics[3])
        {
            auto info = m.instantiate<stdm::Float32MultiArray>();
            if (info != nullptr)
            {
                ground_truth.push_back(info);
                // truth_sub.newMessage(info);
            }
            else
            {
                cout << "NULL initiation!" << endl;
            }
        }
        else if (m.getTopic() == topics[4])
        {
            auto info = m.instantiate<stdm::Float32MultiArray>();
            if (info != nullptr)
            {
                grippers_dot.push_back(info);
                // dot_sub.newMessage(info);
            }
            else
            {
                cout << "NULL initiation!" << endl;
            }   
        }
        else if (m.getTopic() == topics[5])
        {
            auto info = m.instantiate<stdm::Float32MultiArray>();
            if (info != nullptr)
            {
                grippers_ind.push_back(info);
                // ind_sub.newMessage(info);
            }
            else
            {
                cout << "NULL initiation!" << endl;
            }
        }
        else if (m.getTopic() == topics[6])
        {
            auto info = m.instantiate<stdm::Float32MultiArray>();
            if (info != nullptr)
            {
                grippers_config.push_back(info);
                // config_sub.newMessage(info);
            }
            else
            {
                cout << "NULL initiation!" << endl;
            }
        }
        else
        {
            cerr << "Invalid topic: " << m.getTopic() << endl;
            exit(1);
        }
    }
    bag.close();

    auto color_iter = color_images.cbegin();
    auto depth_iter = depth_images.cbegin();
    auto info_iter = camera_infos.cbegin();
    auto config_iter = grippers_config.cbegin();
    auto velocity_iter = grippers_dot.cbegin();
    auto ind_iter = grippers_ind.cbegin();

    cout << "rgb images size: " << color_images.size() << endl;
    cout << "depth images size: " << depth_images.size() << endl;
    cout << "camera infos size: " << camera_infos.size() << endl;
    cout << "gripper configuration size: " << g_config.size() << endl;
    cout << "gripper velocity size: " << g_dot.size() << endl;
    cout << "gripper index size: " << grippers_ind.size() << endl;

    auto [g_config, g_dot, g_ind] = toGripperConfig(*config_iter, *velocity_iter, *ind_iter);
    auto [color_image_bgr, depth_image, intrinsics] = toOpenCv(*color_iter, *depth_iter, *info_iter);

    cout << "rbg image size: " << color_image_bgr.rows << "-" << color_image_bgr.cols << endl;
    cout << "depth image size: " << depth_image.rows << "-" << depth_image.cols << endl;
    cout << "intrinsics: " << endl;
    cout << intrinsics << endl << endl;
    cout << "gripper configuration: " << endl;
    cout << g_config << endl << endl;
    cout << "gripper velocity: " << endl;
    cout << g_dot << endl << endl;
    cout << "gripper index" << endl;
    cout << grippers_ind << endl << endl;

    return 0;
}
