#include <string>
#include <vector>
#include <ros/ros.h>
#include "cdcpd_ros/kinect_sub.h"
#include <thread>
#include <chrono>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <map>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <message_filters/subscriber.h>
#include <message_filters/simple_filter.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "cdcpd/cdcpd.h"

using std::cout;
using std::endl;
using Eigen::MatrixXf;
using Eigen::MatrixXi;
using Eigen::Matrix3Xf;

using namespace cv;

std::vector<cv::Mat> color_images;
std::vector<cv::Mat> depth_images;

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

std::string workingDir = "/home/deformtrack/catkin_ws/src/cdcpd_test/result";

void clean_file(const std::string& fname) {
    std::ofstream ofs(fname, std::ofstream::out);
    ofs << "";
    ofs.close();
};

void to_file(const std::string fname, const cv::Mat m) {
    std::ofstream ofs(fname, std::ofstream::app);
    ofs << "Matrix begins\n";
    ofs << m << "\n\n";
    ofs << "Matrix ends\n";
    ofs.close();
}

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
    void newMessage(const boost::shared_ptr<M const> &msg)
    {
        this->signalMessage(msg);
    }
};

void callback(const sensor_msgs::Image::ConstPtr &rgb_img,
              const sensor_msgs::Image::ConstPtr &depth_img)
{
    cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(rgb_img, sensor_msgs::image_encodings::BGR8);
    cv::Mat color_image = rgb_ptr->image.clone();
    color_images.push_back(color_image);

    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depth_image = depth_ptr->image.clone();
    depth_images.push_back(depth_image);
}

std::tuple<Eigen::Matrix3Xf, Eigen::Matrix2Xi> make_rectangle(float width, float height, int num_width, int num_height)
{
    Eigen::Matrix3Xf vertices = Eigen::Matrix3Xf::Zero(3, num_width * num_height);
    Eigen::Matrix2Xi edges = Eigen::Matrix2Xi::Zero(2, (num_width - 1) * num_height + (num_height - 1) * num_width);

    int edge_count = 0;
    for (int i = 0; i < num_height; ++i) {
        // cout << i << endl;
        for (int j = 0; j < num_width; ++j)
        {
            // cout << j << endl;
            int index = j * num_height + i;
            vertices(0, index) = static_cast<float>(i) * height / static_cast<float>(num_height - 1);
            vertices(1, index) = static_cast<float>(j) * width / static_cast<float>(num_width - 1);
            vertices(2, index) = 0.1f;
            if (i + 1 < num_height)
            {
                int next_index = j * num_height + i + 1;
                edges(0, edge_count) = index;
                edges(1, edge_count) = next_index;
                edge_count++;
            }
            if (j + 1 < num_width)
            {
                int next_index = (j + 1) * num_height + i;
                edges(0, edge_count) = index;
                edges(1, edge_count) = next_index;
                edge_count++;
            }
        }
    }
    assert(edge_count == (num_width - 1) * num_height + (num_height - 1) * num_width);
    return std::make_tuple(vertices, edges);
}

int main(int argc, char* argv[])
{
    // ENHANCE: more smart way to get Y^0 and E
    ros::init(argc, argv, "cdcpd_ros_node");
    cout << "Starting up..." << endl;

    #ifdef ROPE

    // initial connectivity model of rope
    int points_on_rope = 50;
    float rope_length = 1.0;
    MatrixXf template_vertices(3, points_on_rope); // Y^0 in the paper
    template_vertices.setZero();
    template_vertices.row(1).setLinSpaced(points_on_rope, -rope_length, 0);
    template_vertices.row(2).array() += 1.0f;
    MatrixXi template_edges(2, points_on_rope - 1);
    template_edges(0, 0) = 0;
    template_edges(1, template_edges.cols() - 1) = points_on_rope - 1;
    for (int i = 1; i <= template_edges.cols() - 1; ++i)
    {
        template_edges(0, i) = i;
        template_edges(1, i - 1) = i;
    }

    #else

    int cloth_width_num = 23;
    int cloth_height_num = 18;
    float cloth_width = 0.44f;
    float cloth_height = 0.34f;
    auto [template_vertices, template_edges] = make_rectangle(cloth_width, cloth_height, cloth_width_num, cloth_height_num);

    #endif

    #ifdef DEBUG

    clean_file(workingDir + "/cpp_entire_cloud.txt");
    clean_file(workingDir + "/cpp_downsample.txt");
    clean_file(workingDir + "/cpp_TY.txt");
    clean_file(workingDir + "/cpp_Y_opt.txt");
    clean_file(workingDir + "/cpp_TY-1.txt");
    clean_file(workingDir + "/cpp_hsv.txt");
    clean_file(workingDir + "/cpp_mask.txt");

    #endif

    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < template_vertices.cols(); ++i)
    {
        const auto& c = template_vertices.col(i);
        template_cloud->push_back(pcl::PointXYZ(c(0), c(1), c(2)));
    }

#ifdef COMP
    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud_without_constrain(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < template_vertices.cols(); ++i)
    {
        const auto& c = template_vertices.col(i);
        template_cloud_without_constrain->push_back(pcl::PointXYZ(c(0), c(1), c(2)));
    }
#endif

    // comes from the default P matrix on the /kinect2/qhd/camera_info channel.
    cv::Mat P_mat(3, 4, CV_64FC1);

    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

#ifdef ENTIRE
    ros::Publisher original_publisher = ph.advertise<PointCloud> ("original", 1);
#endif
    ros::Publisher masked_publisher = ph.advertise<PointCloud> ("masked", 1);
    ros::Publisher downsampled_publisher = ph.advertise<PointCloud> ("downsampled", 1);
    ros::Publisher template_publisher = ph.advertise<PointCloud> ("template", 1);
    ros::Publisher cpd_iters_publisher = ph.advertise<PointCloud> ("cpd_iters", 1);
    ros::Publisher output_publisher = ph.advertise<PointCloud> ("output", 1);
    ros::Publisher output_without_constrain_publisher = ph.advertise<PointCloud> ("output_without_constrain", 1);
    ros::Publisher left_gripper_pub = nh.advertise<geometry_msgs::TransformStamped>("/cdcpd/left_gripper_prior", 1);
    ros::Publisher right_gripper_pub = nh.advertise<geometry_msgs::TransformStamped>("/cdcpd/right_gripper_prior", 1);
    ros::Publisher cylinder_pub = ph.advertise<visualization_msgs::Marker>( "cylinder", 0 );
    ros::Publisher order_pub = ph.advertise<visualization_msgs::Marker>("order", 10);



    BagSubscriber<sensor_msgs::Image> rgb_sub, depth_sub;

    cout << "Making buffer" << endl;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    rosbag::Bag bag;
#ifdef CYL4
    bag.open("/home/deformtrack/catkin_ws/src/cdcpd_test/dataset/interaction_cylinder_4.bag", rosbag::bagmode::Read);
#endif
#ifdef CYL5
    bag.open("/home/deformtrack/catkin_ws/src/cdcpd_test/dataset/interaction_cylinder_5.bag", rosbag::bagmode::Read);
#endif
#ifdef CYL6
    bag.open("/home/deformtrack/catkin_ws/src/cdcpd_test/dataset/interaction_cylinder_6.bag", rosbag::bagmode::Read);
#endif
#ifdef CYL7
    bag.open("/home/deformtrack/catkin_ws/src/cdcpd_test/dataset/interaction_cylinder_7.bag", rosbag::bagmode::Read);
#endif
#ifdef CYL8
    bag.open("/home/deformtrack/catkin_ws/src/cdcpd_test/dataset/interaction_cylinder_8.bag", rosbag::bagmode::Read);
#endif
#ifdef CYL9
    bag.open("/home/deformtrack/catkin_ws/src/cdcpd_test/dataset/interaction_cylinder_9.bag", rosbag::bagmode::Read);
#endif
#ifdef NORMAL
    bag.open("/home/deformtrack/catkin_ws/src/cdcpd_test/dataset/normal.bag", rosbag::bagmode::Read);
#endif
#ifdef NMCLOTH1
    bag.open("/home/deformtrack/catkin_ws/src/cdcpd_test/dataset/normal_cloth1.bag", rosbag::bagmode::Read);
#endif
#ifdef NMCLOTH2
    bag.open("/home/deformtrack/catkin_ws/src/cdcpd_test/dataset/normal_cloth2.bag", rosbag::bagmode::Read);
#endif

    std::vector<std::string> topics;
    topics.push_back(std::string("/kinect2/qhd/image_color_rect"));
    topics.push_back(std::string("/kinect2/qhd/image_depth_rect"));
    topics.push_back(std::string("/kinect2/qhd/camera_info"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(rgb_sub, depth_sub, 25);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // TODO: Switch to using full callback style
    // KinectSub kinect_sub(callback_fn,
    //                      KinectSub::SubscriptionOptions("kinect2_" + kinect_name + "/" + stream));

    // TODO this might be too much memory at some point

    for(rosbag::MessageInstance const m: view)
    {
        cout << "topic: " << m.getTopic() << endl;
        if (m.getTopic() == topics[0])
        {
            sensor_msgs::Image::ConstPtr i = m.instantiate<sensor_msgs::Image>();
            if (i != nullptr) {
                rgb_sub.newMessage(i);
            }
            else
                cout << "NULL initiation!" << endl;
        }
        else if (m.getTopic() == topics[1])
        {
            sensor_msgs::Image::ConstPtr i = m.instantiate<sensor_msgs::Image>();
            if (i != nullptr) {
                depth_sub.newMessage(i);
            }
            else
                cout << "NULL initiation!" << endl;
        }
        else if (m.getTopic() == topics[2])
        {
            sensor_msgs::CameraInfo::ConstPtr info_msg = m.instantiate<sensor_msgs::CameraInfo>();
            // the following is similar to https://github.com/ros-perception/image_pipeline/blob/melodic/depth_image_proc/src/nodelets/point_cloud_xyzrgb.cpp
            // Check if the input image has to be resized
            sensor_msgs::CameraInfo info_msg_tmp = *info_msg;
            if (!depth_images.empty() && !color_images.empty() && (depth_images.back().cols != color_images.back().cols || depth_images.back().rows != color_images.back().rows))
            {
                cout << "resized" << endl;
                info_msg_tmp.width = depth_images.back().cols;
                info_msg_tmp.height = depth_images.back().rows;
                float ratio = float(depth_images.back().cols)/float(color_images.back().cols);
                info_msg_tmp.K[0] *= ratio;
                info_msg_tmp.K[2] *= ratio;
                info_msg_tmp.K[4] *= ratio;
                info_msg_tmp.K[5] *= ratio;
                info_msg_tmp.P[0] *= ratio;
                info_msg_tmp.P[2] *= ratio;
                info_msg_tmp.P[5] *= ratio;
                info_msg_tmp.P[6] *= ratio;
            }
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 4; ++col) {
                    P_mat.at<double>(row, col) = info_msg_tmp.P[row*4+col];
                }
            }
            cout << "P matrix: " << endl;
            cout << P_mat << endl << endl;
        }
        else
        {
            cerr << "Invalid topic: " << m.getTopic() << endl;
            exit(1);
        }
    }

    CDCPD cdcpd(template_cloud, P_mat, false);
#ifdef COMP
    CDCPD cdcpd_without_constrain(template_cloud, P_mat);
#endif

    bag.close();

    auto color_iter = color_images.cbegin();
    auto depth_iter = depth_images.cbegin();

    // Let's also grab the gripper positions. Note that in practice, you'd do this in real time.
    geometry_msgs::TransformStamped leftTS;
    geometry_msgs::TransformStamped rightTS;
    bool use_grippers = false; // TODO don't error if no gripper broadcast

    if (use_grippers) {
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
    }

    // Eigen::Vector3f left_pos(-1.5f, -0.5f, 2.2f);
    // Eigen::Vector3f right_pos(-1.06f, -0.55f, 2.23f);
    Eigen::Vector3f left_pos((float)leftTS.transform.translation.x, (float)leftTS.transform.translation.y, (float)leftTS.transform.translation.z);
    Eigen::Vector3f right_pos((float)rightTS.transform.translation.x, (float)rightTS.transform.translation.y, (float)rightTS.transform.translation.z);
    cout << "Left gripper: " << left_pos << endl;
    cout << "Right gripper: " << right_pos << endl;
    CDCPD::FixedPoint left_gripper = { left_pos, (int) template_vertices.cols() - 1 };
    CDCPD::FixedPoint right_gripper = { right_pos, 0 };

    std::string stepper;
    ros::Rate rate(15); // 5 hz, maybe allow changes, or mimicking bag?

    while(color_iter != color_images.cend() && depth_iter != depth_images.cend())
    {
        if (stepper != "r") {
            cin >> stepper;
        }
        else {
            rate.sleep();
        }
        if (!ros::ok())
        {
            exit(-1);
        }
        left_gripper_pub.publish(leftTS);
        right_gripper_pub.publish(rightTS);
        auto color_image_bgr = *color_iter;
        auto depth_image = *depth_iter;
        cout << "Matched" << endl;
        /// Color filter
        // For the red rope, (h > 0.85) & (s > 0.5). For the flag, (h < 1.0) & (h > 0.9)
        // The flag isn't tested or implemented
        Mat rgb_image;
        cv::cvtColor(color_image_bgr, rgb_image, cv::COLOR_BGR2RGB);
        // TODO I'm pretty sure this is an 8-bit image.

#ifdef DEBUG
        imwrite("rgb.png", rgb_image);
        imwrite("depth.png", depth_image);
#endif

        cv::Mat rgb_f;
        rgb_image.convertTo(rgb_f, CV_32FC3);
        rgb_f /= 255.0; // get RGB 0.0-1.0
        cv::Mat color_hsv;
        cvtColor(rgb_f, color_hsv, CV_RGB2HSV);
#ifdef DEBUG
        to_file(workingDir + "/cpp_hsv.txt", color_hsv);
#endif

        // White
        // cv::Scalar low_hsv = cv::Scalar(0.0 * 360.0, 0.0, 0.98);
        // cv::Scalar high_hsv = cv::Scalar(1.0 * 360.0, 0.02, 1.0);

#ifdef ROPE
        // Red
        cv::Mat mask1;
        cv::Mat mask2;
        cv::inRange(color_hsv, cv::Scalar(0, 0.2, 0.2), cv::Scalar(20, 1.0, 1.0), mask1);
        cv::inRange(color_hsv, cv::Scalar(340, 0.2, 0.2), cv::Scalar(360, 1.0, 1.0), mask2);
        cv::Mat hsv_mask;
        bitwise_or(mask1, mask2, hsv_mask);
#else
        // Purple
        cv::Mat hsv_mask;
        cv::inRange(color_hsv, cv::Scalar(210, 0.1, 0.4), cv::Scalar(250, 0.5, 0.8), hsv_mask);
#endif

#ifdef DEBUG
        to_file(workingDir + "/cpp_mask.txt", hsv_mask);
        cv::imwrite("hsv_mask.png", hsv_mask);
#endif

        // Without the grippers vs with the grippers
        CDCPD::Output out;
#ifdef COMP
        CDCPD::Output out_without_constrain;
#endif
        if (use_grippers)
        {
            out = cdcpd(rgb_image, depth_image, hsv_mask, template_cloud, template_edges, true, {left_gripper, right_gripper});
        }
        else
        {
            out = cdcpd(rgb_image, depth_image, hsv_mask, template_cloud, template_edges);
#ifdef COMP
            out_without_constrain = cdcpd_without_constrain(rgb_image, depth_image, hsv_mask, template_cloud_without_constrain, template_edges, false);
#endif
        }

        template_cloud = out.gurobi_output;
#ifdef COMP
        template_cloud_without_constrain = out_without_constrain.gurobi_output;
#endif

        auto frame_id = "kinect2_rgb_optical_frame";
#ifdef ENTIRE
        out.original_cloud->header.frame_id = frame_id;
#endif
        out.masked_point_cloud->header.frame_id = frame_id;
        out.downsampled_cloud->header.frame_id = frame_id;
        out.cpd_output->header.frame_id = frame_id;
        out.gurobi_output->header.frame_id = frame_id;
#ifdef COMP
        out_without_constrain.gurobi_output->header.frame_id = frame_id;
#endif

        // draw cylinder
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time();
        marker.ns = "cylinder";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;

#ifdef CYL2
        // interaction_cylinder_2.bag
        marker.pose.position.x = 0.145124522395497;
        marker.pose.position.y = -0.152708792314512;
        marker.pose.position.z = 1.095150852162702;
        marker.pose.orientation.x = -0.3540;
        marker.pose.orientation.y = -0.0155;
        marker.pose.orientation.z = 0.0408;
        marker.pose.orientation.w = 0.9342;
        marker.scale.x = 0.033137245873063*2;
        marker.scale.y = 0.033137245873063*2;
        marker.scale.z = 0.153739168519654;
#endif

#ifdef CYL4
        // interaction_cylinder_4.bag
        marker.pose.position.x = -0.001783838376740;
        marker.pose.position.y = -0.202407765852103;
        marker.pose.position.z = 1.255950979292225;
        marker.pose.orientation.x = -0.2134;
        marker.pose.orientation.y = -0.0024;
        marker.pose.orientation.z = 0.0110;
        marker.pose.orientation.w = 0.9769;
        marker.scale.x = 0.05*2;
        marker.scale.y = 0.05*2;
        marker.scale.z = 0.21;
#endif

#ifdef CYL5
        // interaction_cylinder_5.bag
        marker.pose.position.x = -0.007203971514259;
        marker.pose.position.y = -0.282011643023486;
        marker.pose.position.z = 1.351697407251410;
        marker.pose.orientation.x = -0.7884;
        marker.pose.orientation.y = -0.0193;
        marker.pose.orientation.z = 0.0150;
        marker.pose.orientation.w = 0.6147;
        marker.scale.x = 0.05*2;
        marker.scale.y = 0.05*2;
        marker.scale.z = 0.21;
#endif

#ifdef CYL6
        // interation_cylinder_6.bag
        marker.pose.position.x = -0.025889295027034;
        marker.pose.position.y = -0.020591825574503;
        marker.pose.position.z = 1.200787565152055;
        marker.pose.orientation.x = -0.7852;
        marker.pose.orientation.y = -0.0016;
        marker.pose.orientation.z = 0.0013;
        marker.pose.orientation.w = 0.6193;
        marker.scale.x = 0.05*2;
        marker.scale.y = 0.05*2;
        marker.scale.z = 0.21;
#endif

#ifdef CYL7
        // interation_cylinder_7.bag
        marker.pose.position.x = -0.025889295027034;
        marker.pose.position.y = -0.020591825574503;
        marker.pose.position.z = 1.200787565152055;
        marker.pose.orientation.x = -0.7952;
        marker.pose.orientation.y = -0.0010;
        marker.pose.orientation.z = 0.0008;
        marker.pose.orientation.w = 0.6064;
        marker.scale.x = 0.05*2;
        marker.scale.y = 0.05*2;
        marker.scale.z = 0.21;
#endif

#ifdef CYL8
        // interation_cylinder_7.bag
        marker.pose.position.x = -0.239953252695972;
        marker.pose.position.y = -0.326861315788172;
        marker.pose.position.z = 1.459887097878595;
        marker.pose.orientation.x = -0.1877;
        marker.pose.orientation.y = -0.0009;
        marker.pose.orientation.z = 0.0046;
        marker.pose.orientation.w = 0.9822;
        marker.scale.x = 0.05*2;
        marker.scale.y = 0.05*2;
        marker.scale.z = 0.21;
#endif

#ifdef CYL9
        // interation_cylinder_9.bag
        marker.pose.position.x = -0.239953252695972;
        marker.pose.position.y = -0.28;
        marker.pose.position.z = 1.459887097878595;
        marker.pose.orientation.x = -0.1877;
        marker.pose.orientation.y = -0.0009;
        marker.pose.orientation.z = 0.0046;
        marker.pose.orientation.w = 0.9822;
        marker.scale.x = 0.05*2;
        marker.scale.y = 0.05*2;
        marker.scale.z = 0.21;
#endif

        marker.color.a = 0.5; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        //only if using a MESH_RESOURCE marker type:
        marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

#ifdef CYLINDER_INTER
        cylinder_pub.publish( marker );
#endif

        // draw line order
        visualization_msgs::Marker order;
        order.header.frame_id = frame_id;
        order.header.stamp = ros::Time();
        order.ns = "line_order";
        order.action = visualization_msgs::Marker::ADD;
        order.pose.orientation.w = 1.0;
        order.id = 1;
        order.scale.x = 0.002;
        order.color.b = 1.0;
        order.color.a = 1.0;
        auto pc_iter = out.gurobi_output->begin();

#ifdef ROPE

        // rope order
        order.type = visualization_msgs::Marker::LINE_STRIP;
        for (int i = 0; i < points_on_rope; ++i, ++pc_iter) {
            geometry_msgs::Point p;
            p.x = pc_iter->x;
            p.y = pc_iter->y;
            p.z = pc_iter->z;

            order.points.push_back(p);
        }

#else

        // cloth order
        order.type = visualization_msgs::Marker::LINE_LIST;
        for (int row = 0; row < cloth_height_num; ++row) {
            for (int col = 0; col < cloth_width_num; ++col) {
                if (row != cloth_height_num - 1 && col != cloth_width_num - 1) {
                    geometry_msgs::Point cur;
                    geometry_msgs::Point right;
                    geometry_msgs::Point below;
                    int cur_ind = col*cloth_height_num + row;
                    int right_ind = cur_ind + cloth_height_num;
                    int below_ind = cur_ind + 1;

                    cur.x = (pc_iter+cur_ind)->x;
                    cur.y = (pc_iter+cur_ind)->y;
                    cur.z = (pc_iter+cur_ind)->z;

                    right.x = (pc_iter+right_ind)->x;
                    right.y = (pc_iter+right_ind)->y;
                    right.z = (pc_iter+right_ind)->z;

                    below.x = (pc_iter+below_ind)->x;
                    below.y = (pc_iter+below_ind)->y;
                    below.z = (pc_iter+below_ind)->z;

                    order.points.push_back(cur);
                    order.points.push_back(right);

                    order.points.push_back(cur);
                    order.points.push_back(below);
                }
                else if (row == cloth_height_num - 1 && col != cloth_width_num - 1) {
                    geometry_msgs::Point cur;
                    geometry_msgs::Point right;
                    int cur_ind = col*cloth_height_num + row;
                    int right_ind = cur_ind + cloth_height_num;

                    cur.x = (pc_iter+cur_ind)->x;
                    cur.y = (pc_iter+cur_ind)->y;
                    cur.z = (pc_iter+cur_ind)->z;

                    right.x = (pc_iter+right_ind)->x;
                    right.y = (pc_iter+right_ind)->y;
                    right.z = (pc_iter+right_ind)->z;

                    order.points.push_back(cur);
                    order.points.push_back(right);
                }
                else if (row != cloth_height_num - 1 && col == cloth_width_num - 1) {
                    geometry_msgs::Point cur;
                    geometry_msgs::Point below;
                    int cur_ind = col*cloth_height_num + row;
                    int below_ind = cur_ind + 1;

                    cur.x = (pc_iter+cur_ind)->x;
                    cur.y = (pc_iter+cur_ind)->y;
                    cur.z = (pc_iter+cur_ind)->z;

                    below.x = (pc_iter+below_ind)->x;
                    below.y = (pc_iter+below_ind)->y;
                    below.z = (pc_iter+below_ind)->z;

                    order.points.push_back(cur);
                    order.points.push_back(below);
                }
            }
        }
#endif

        order_pub.publish(order);

        auto time = ros::Time::now();
#ifdef ENTIRE
        pcl_conversions::toPCL(time, out.original_cloud->header.stamp);
#endif
        pcl_conversions::toPCL(time, out.masked_point_cloud->header.stamp);
        pcl_conversions::toPCL(time, out.downsampled_cloud->header.stamp);
        pcl_conversions::toPCL(time, out.cpd_output->header.stamp);
        pcl_conversions::toPCL(time, out.gurobi_output->header.stamp);
#ifdef COMP
        pcl_conversions::toPCL(time, out_without_constrain.gurobi_output->header.stamp);
#endif

#ifdef ENTIRE
        original_publisher.publish(out.original_cloud);
#endif
        masked_publisher.publish(out.masked_point_cloud);
        downsampled_publisher.publish(out.downsampled_cloud);
        template_publisher.publish(out.cpd_output);
        output_publisher.publish(out.gurobi_output);
#ifdef COMP
        output_without_constrain_publisher.publish(out_without_constrain.gurobi_output);
#endif

        ++color_iter;
        ++depth_iter;
    }

    cout << "Test ended" << endl;

    return EXIT_SUCCESS;
}
