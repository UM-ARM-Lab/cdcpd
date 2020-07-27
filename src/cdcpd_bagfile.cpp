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
#ifdef SIMULATION
std::vector<stdm::Float32MultiArray::ConstPtr> grippers_config;
std::vector<stdm::Float32MultiArray::ConstPtr> grippers_dot;
std::vector<stdm::Float32MultiArray::ConstPtr> grippers_ind;
std::vector<stdm::Float32MultiArray::ConstPtr> ground_truth;
#endif

#ifdef SIMULATION
std::string workingDir = "/home/deformtrack/catkin_ws/src/cdcpd_test_blender/result";
#else
std::string workingDir = "/home/deformtrack/catkin_ws/src/cdcpd_test/result";
#endif

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

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

double calculate_lle_reg(MatrixXf& L, PointCloud& pts) {
    double reg = 0;
    Matrix3Xf pts_matrix = pts.getMatrixXfMap();
    for (int ind = 0; ind < L.rows(); ++ind)
    {
        Matrix3Xf lle_pt(3,1);
        lle_pt(0,0) = 0; lle_pt(1,0) = 0; lle_pt(2,0) = 0;
        for (int nb_ind = 0; nb_ind < L.cols(); ++nb_ind)
        {
            lle_pt = lle_pt + L(ind, nb_ind) * pts_matrix.col(nb_ind);
        }
        reg += (lle_pt - pts_matrix.col(ind)).squaredNorm();
    }
    return reg;
}

#if 0
void test_lle() {
    // test LLE
    PointCloud::Ptr init(new PointCloud);
    for (int i = 0; i < 100; ++i)
    {
        init->push_back(PointXYZ(float(i), 0.0f, 0.0f));
    }

    pcl::KdTreeFLANN<PointXYZ> kdtree;
    kdtree.setInputCloud (init);
    // W: (M, M) matrix, corresponding to L in Eq. (15) and (16)
    MatrixXf L = barycenter_kneighbors_graph(kdtree, 12, 0.001);

    // strech
    PointCloud::Ptr strech(new PointCloud);
    for (int i = 0; i < 100; ++i)
    {
        strech->push_back(PointXYZ(1.1f*float(i), 0.0f, 0.0f));
    }
    cout << "strech regularization: ";
    cout << calculate_lle_reg(L, *strech) << endl;

    // compress
    PointCloud::Ptr comp(new PointCloud);
    for (int i = 0; i < 100; ++i)
    {
        comp->push_back(PointXYZ(0.9f*float(i), 0.0f, 0.0f));
    }
    cout << "compress regularization: ";
    cout << calculate_lle_reg(L, *comp) << endl;


    // bending
    PointCloud::Ptr bend(new PointCloud);
    for (int i = 0; i < 50; ++i)
    {
        bend->push_back(PointXYZ(float(i), 0.0f, 0.0f));
    }
    for (int i = 0; i < 50; ++i)
    {
        bend->push_back(PointXYZ(50.0f, float(i), 0.0f));
    }
    cout << "bending regularization: ";
    cout << calculate_lle_reg(L, *bend) << endl;

    // circle
    PointCloud::Ptr circle(new PointCloud);
    for (int i = 0; i < 100; ++i)
    {
        circle->push_back(PointXYZ(float(cos(double(i)/double(100.0)*M_PI/2)), float(sin(double(i)/double(100.0)*M_PI/2)), 0.0f));
    }
    cout << "circle regularization: ";
    cout << calculate_lle_reg(L, *circle) << endl;

    // original
    cout << "original regularization: ";
    cout << calculate_lle_reg(L, *init) << endl;

    exit(1);
}
#endif

std::tuple<Eigen::Matrix3Xf, Eigen::Matrix2Xi> make_rectangle(float width, float height, int num_width, int num_height)
{
    float left_bottom_y = 0.0f;
    float left_bottom_x = 0.0f;
    float z = 1.45f;
    Eigen::Matrix3Xf vertices = Eigen::Matrix3Xf::Zero(3, num_width * num_height);
    Eigen::Matrix2Xi edges = Eigen::Matrix2Xi::Zero(2, (num_width - 1) * num_height + (num_height - 1) * num_width);

    int edge_count = 0;
    for (int i = 0; i < num_height; ++i) {
        for (int j = 0; j < num_width; ++j)
        {
            int index = j * num_height + i;
            vertices(0, index) = static_cast<float>(j) * width / static_cast<float>(num_width - 1)+left_bottom_x;
            vertices(1, index) = static_cast<float>(i) * height / static_cast<float>(num_height - 1)+left_bottom_y;
            vertices(2, index) = z;
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
    // test_nearest_line();
    // test_lle();
    // ENHANCE: more smart way to get Y^0 and E
    ros::init(argc, argv, "cdcpd_bagfile");
    cout << "Starting up..." << endl;

    #ifdef ROPE

    // initial connectivity model of rope
    int points_on_rope = 50;
    float rope_length = 1.0f;
    MatrixXf template_vertices(3, points_on_rope); // Y^0 in the paper
    template_vertices.setZero();
    template_vertices.row(0).setLinSpaced(points_on_rope, -rope_length/2-1.5f, rope_length/2-1.5f);
    template_vertices.row(2).array() += 6.0f;
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
    clean_file(workingDir + "/cpp_intrinsics.txt");
    #endif
    clean_file(workingDir + "/occluded_index.txt");

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

    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    // Publsihers for the data, some visualizations, others consumed by other nodes
    auto original_publisher = nh.advertise<PointCloud> ("cdcpd/original", 1);
    auto masked_publisher = nh.advertise<PointCloud> ("cdcpd/masked", 1);
    auto downsampled_publisher = nh.advertise<PointCloud> ("cdcpd/downsampled", 1);
    auto template_publisher = nh.advertise<PointCloud> ("cdcpd/template", 1);
    #ifdef PREDICT
    auto pred_publisher = nh.advertise<PointCloud>("cdcpd/prediction", 1);
    #endif
    // auto cpd_iters_publisher = nh.advertise<PointCloud> ("cdcpd/cpd_iters", 1);
    auto output_publisher = nh.advertise<PointCloud> ("cdcpd/output", 1);
    #ifdef COMP
    auto output_without_constrain_publisher = nh.advertise<PointCloud> ("cdcpd/output_without_constrain", 1);
    #endif
    auto left_gripper_pub = nh.advertise<gm::TransformStamped>("cdcpd/left_gripper_prior", 1);
    auto right_gripper_pub = nh.advertise<gm::TransformStamped>("cdcpd/right_gripper_prior", 1);
    #ifdef CYLINDER
    auto cylinder_pub = nh.advertise<vm::Marker>("cdcpd/cylinder", 0);
    #endif
    auto order_pub = nh.advertise<vm::Marker>("cdcpd/order", 10);

    BagSubscriber<sm::Image> rgb_sub, depth_sub;
    BagSubscriber<sm::CameraInfo> info_sub;
    #ifdef SIMULATION
    BagSubscriber<stdm::Float32MultiArray> config_sub, dot_sub, ind_sub, truth_sub;
    #endif

    cout << "Making buffer" << endl;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    const double alpha = ROSHelpers::GetParam<double>(ph, "alpha", 0.5);
    const double lambda = ROSHelpers::GetParam<double>(ph, "lambda", 1.0);
    const double k_spring = ROSHelpers::GetParam<double>(ph, "k", 100.0);
    const double beta = ROSHelpers::GetParam<double>(ph, "beta", 1.0);

    std::vector<std::string> topics;
    #ifdef SIMULATION
    topics.push_back(std::string("image_color_rect"));
    topics.push_back(std::string("image_depth_rect"));
    topics.push_back(std::string("camera_info"));
    topics.push_back(std::string("groud_truth"));
    topics.push_back(std::string("gripper_velocity"));
    topics.push_back(std::string("gripper_info"));
    topics.push_back(std::string("gripper_config"));
    #else
    topics.push_back(std::string("/kinect2/qhd/image_color_rect"));
    topics.push_back(std::string("/kinect2/qhd/image_depth_rect"));
    topics.push_back(std::string("/kinect2/qhd/camera_info"));
    #endif

    auto const bagfile = ROSHelpers::GetParam<std::string>(ph, "bagfile", "normal");
    #ifdef SIMULATION
    auto const folder = ros::package::getPath("cdcpd_ros") + "/../cdcpd_test_blender/dataset/";
    #else
    auto const folder = ros::package::getPath("cdcpd_ros") + "/../cdcpd_test/dataset/";
    #endif
    rosbag::Bag bag(folder + bagfile + ".bag", rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Go through the bagfile, storing matched image pairs
    // TODO this might be too much memory at some point
    // #ifndef SIMULATION
    auto sync = message_filters::TimeSynchronizer<sm::Image, sm::Image, sm::CameraInfo>(
            rgb_sub, depth_sub, info_sub, 25);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    // #endif

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
        #ifdef SIMULATION
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
        #endif
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
    #ifdef SIMULATION
    auto config_iter = grippers_config.cbegin();
    auto velocity_iter = grippers_dot.cbegin();
    auto ind_iter = grippers_ind.cbegin();
    auto truth_iter = ground_truth.cbegin();
    #endif

    cout << "rgb images size: " << color_images.size() << endl;
    cout << "depth images size: " << depth_images.size() << endl;
    cout << "camera infos size: " << camera_infos.size() << endl;

    // Used to republish the images at the current timestamp for other usage
    // Exits at the end of the if statement
    auto const republish_bagfile = ROSHelpers::GetParam<bool>(ph, "republish_bagfile", false);
    if (republish_bagfile)
    {
        cout << "Republishing bag only, and then exiting" << endl;

        image_transport::ImageTransport it(nh);
        auto color_pub = it.advertise(topics[0], 1);
        auto depth_pub = it.advertise(topics[1], 1);
        auto info_pub = nh.advertise<sm::CameraInfo>(topics[2], 1);

        std::string stepper;
        ros::Rate rate(30); // 30 hz, maybe allow changes, or mimicking bag?
        while(ros::ok() &&
            color_iter != color_images.cend() &&
            depth_iter != depth_images.cend() &&
            info_iter != camera_infos.cend())
        {
            if (stepper != "r")
            {
                cout << "Waiting for input, enter 'r' to run without stopping, anything else to step once ... " << std::flush;
                cin >> stepper;
            }

            auto time = ros::Time::now();

            auto color_msg = **color_iter;
            color_msg.header.stamp = time;
            color_pub.publish(color_msg);

            auto depth_msg = **depth_iter;
            depth_msg.header.stamp = time;
            depth_pub.publish(depth_msg);

            auto info_msg = **info_iter;
            info_msg.header.stamp = time;
            info_pub.publish(info_msg);

            rate.sleep();
            ++color_iter;
            ++depth_iter;
            ++info_iter;
        }

        cout << "End of sycnronized bag, terminating." << endl;
        return EXIT_SUCCESS;
    }

    #ifdef SIMULATION
    auto [g_config, g_dot, g_ind] = toGripperConfig(*config_iter, *velocity_iter, *ind_iter);
    std::shared_ptr<ros::NodeHandle> nh_ptr = std::make_shared<ros::NodeHandle>(nh);
    double translation_dir_deformability = 10.0;
    double translation_dis_deformability = 10.0;
    double rotation_deformability = 10.0;
    CDCPD cdcpd(template_cloud,
                template_edges,
                nh_ptr,
                translation_dir_deformability,
                translation_dis_deformability,
                rotation_deformability,
                g_ind,
                false,
                alpha,
                beta,
                lambda,
                k_spring);
    #else
    CDCPD cdcpd(template_cloud, template_edges, false, alpha, beta, lambda, k_spring);
    #endif
    #ifdef COMP
    CDCPD cdcpd_without_constrain(template_cloud, template_edges, intrinsics, false, alpha, beta, lambda, k_spring);
    #endif


    // Let's also grab the gripper positions. Note that in practice, you'd do this in real time.
    geometry_msgs::TransformStamped leftTS;
    geometry_msgs::TransformStamped rightTS;
    bool use_grippers = false; // TODO don't error if no gripper broadcast

    std::vector<CDCPD::FixedPoint> fixed_points;
    if (use_grippers)
    {
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
        // Eigen::Vector3f left_pos(-1.5f, -0.5f, 2.2f);
        // Eigen::Vector3f right_pos(-1.06f, -0.55f, 2.23f);
        Eigen::Vector3f left_pos((float)leftTS.transform.translation.x, (float)leftTS.transform.translation.y, (float)leftTS.transform.translation.z);
        Eigen::Vector3f right_pos((float)rightTS.transform.translation.x, (float)rightTS.transform.translation.y, (float)rightTS.transform.translation.z);
        cout << "Left gripper: " << left_pos.transpose() << endl;
        cout << "Right gripper: " << right_pos.transpose() << endl;
        CDCPD::FixedPoint left_gripper = { left_pos, (int) template_vertices.cols() - 1 };
        CDCPD::FixedPoint right_gripper = { right_pos, 0 };
        fixed_points.push_back(left_gripper);
        fixed_points.push_back(right_gripper);
    }

    std::string stepper;
    ros::Rate rate(30); // 30 hz, maybe allow changes, or mimicking bag?
    while(color_iter != color_images.cend() && depth_iter != depth_images.cend() && info_iter != camera_infos.cend())
    {
        // if(kbhit())
        // {
        //     interprete key;
        // }
        // if (!pause)
        // {
        //     pause;
        // }
        // else
        // {
        //     rate.sleep();
        // }

        if (stepper != "r")
        {
            cout << "Waiting for input, enter 'r' to run without stopping, anything else to step once ... " << std::flush;
            cin >> stepper;
        }
        else
        {
            rate.sleep();
        }
        if (!ros::ok())
        {
            exit(-1);
        }
        if (use_grippers)
        {
            left_gripper_pub.publish(leftTS);
            right_gripper_pub.publish(rightTS);
        }

        auto [color_image_bgr, depth_image, intrinsics] = toOpenCv(*color_iter, *depth_iter, *info_iter);
        #ifdef SIMULATION
        Matrix3Xf one_frame_truth = toGroundTruth(*truth_iter);
        tie(g_config, g_dot, g_ind) = toGripperConfig(*config_iter, *velocity_iter, *ind_iter);
        #endif

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
        cv::Mat hsv_mask;
        cv::inRange(color_hsv, cv::Scalar(0, 0.2, 0.2), cv::Scalar(20, 1.0, 1.0), mask1);
        cv::inRange(color_hsv, cv::Scalar(340, 0.2, 0.2), cv::Scalar(360, 1.0, 1.0), mask2);
        bitwise_or(mask1, mask2, hsv_mask);
        #else
        // Purple
        cv::Mat hsv_mask;
        // cv::inRange(color_hsv, cv::Scalar(210, 0.0, 0.4), cv::Scalar(250, 0.5, 0.8), hsv_mask);
        // cv::inRange(color_hsv, cv::Scalar(50, 0.1, 0.2), cv::Scalar(70, 0.3, 0.6), hsv_mask);
        cv::Mat mask1;
        cv::Mat mask2;
        cv::inRange(color_hsv, cv::Scalar(0, 0.2, 0.2), cv::Scalar(20, 1.0, 1.0), mask1);
        cv::inRange(color_hsv, cv::Scalar(340, 0.2, 0.2), cv::Scalar(360, 1.0, 1.0), mask2);
        bitwise_or(mask1, mask2, hsv_mask);
        #endif

        #ifdef DEBUG
        to_file(workingDir + "/cpp_mask.txt", hsv_mask);
        to_file(workingDir + "/cpp_intrinsics.txt", cv::Mat(intrinsics));
        cv::imwrite(workingDir + "/hsv_mask.png", hsv_mask);
        #endif

        #ifdef SIMULATION
        auto out = cdcpd(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud, g_dot, g_config, true, false, fixed_points);
        #else
        auto out = cdcpd(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud, true, false, fixed_points);
        #endif
        template_cloud = out.gurobi_output;
        #ifdef COMP
        auto out_without_constrain = cdcpd_without_constrain(rgb_image, depth_image, hsv_mask, template_cloud_without_constrain, template_edges, false, false);
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
        #ifdef PREDICT
        out.cpd_predict->header.frame_id = frame_id;
        #endif
        #ifdef COMP
        out_without_constrain.gurobi_output->header.frame_id = frame_id;
        #endif

        // draw cylinder
        {
            vm::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time();
            marker.ns = "cylinder";
            marker.id = 0;
            marker.type = vm::Marker::CYLINDER;
            marker.action = vm::Marker::ADD;

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

            #ifdef CYL_CLOTH1
            // interation_cloth1.bag
            marker.pose.position.x = -0.114121248950204;
            marker.pose.position.y = -0.180876677250917;
            marker.pose.position.z = 1.384255148567173;
            marker.pose.orientation.x = -0.1267;
            marker.pose.orientation.y = 0.0142;
            marker.pose.orientation.z = -0.1107;
            marker.pose.orientation.w = 0.9857;
            marker.scale.x = 0.05*2;
            marker.scale.y = 0.05*2;
            marker.scale.z = 0.21;
            #endif

            #ifdef CYL_CLOTH3
            // interation_cloth1.bag
            marker.pose.position.x = -0.134121248950204;
            marker.pose.position.y = -0.110876677250917;
            marker.pose.position.z = 1.384255148567173;
            marker.pose.orientation.x = -0.1267;
            marker.pose.orientation.y = 0.0142;
            marker.pose.orientation.z = -0.1107;
            marker.pose.orientation.w = 0.9857;
            marker.scale.x = 0.05*2;
            marker.scale.y = 0.05*2;
            marker.scale.z = 0.21;
            #endif

            #ifdef CYL_CLOTH4
            // interation_cloth1.bag
            marker.pose.position.x = -0.134121248950204;
            marker.pose.position.y = -0.110876677250917;
            marker.pose.position.z = 1.384255148567173;
            marker.pose.orientation.x = -0.1267;
            marker.pose.orientation.y = 0.0142;
            marker.pose.orientation.z = -0.1107;
            marker.pose.orientation.w = 0.9857;
            marker.scale.x = 0.05*2;
            marker.scale.y = 0.05*2;
            marker.scale.z = 0.21;
            #endif

            marker.color.a = 0.5; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            #ifdef CYLINDER_INTER
            cylinder_pub.publish( marker );
            #endif
        }

        // draw line order
        {
            vm::Marker order;
            order.header.frame_id = frame_id;
            order.header.stamp = ros::Time();
            order.ns = "line_order";
            order.action = vm::Marker::ADD;
            order.pose.orientation.w = 1.0;
            order.id = 1;
            order.scale.x = 0.002;
            order.color.r = 1.0;
            order.color.a = 1.0;
            auto pc_iter = out.gurobi_output->begin();

            #ifdef COMP
            vm::Marker order_without_constrain;
            order_without_constrain.header.frame_id = frame_id;
            order_without_constrain.header.stamp = ros::Time();
            order_without_constrain.ns = "line_order_comp";
            order_without_constrain.action = vm::Marker::ADD;
            order_without_constrain.pose.orientation.w = 1.0;
            order_without_constrain.id = 2;
            order_without_constrain.scale.x = 0.002;
            order_without_constrain.color.b = 1.0;
            order_without_constrain.color.a = 1.0;
            auto pc_iter_comp = out_without_constrain.gurobi_output->begin();
            #endif

            #ifdef ROPE
            // rope order
            order.type = vm::Marker::LINE_STRIP;
            for (int i = 0; i < points_on_rope; ++i, ++pc_iter) {
                geometry_msgs::Point p;
                p.x = pc_iter->x;
                p.y = pc_iter->y;
                p.z = pc_iter->z;
                order.points.push_back(p);
            }

            #ifdef COMP
            order_without_constrain.type = vm::Marker::LINE_STRIP;
            for (int i = 0; i < points_on_rope; ++i, ++pc_iter_comp) {
                geometry_msgs::Point p;
                p.x = pc_iter_comp->x;
                p.y = pc_iter_comp->y;
                p.z = pc_iter_comp->z;

                order_without_constrain.points.push_back(p);
            }
            #endif
            #else
            // cloth order
            order.type = vm::Marker::LINE_LIST;
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

            #ifdef COMP
                    order_without_constrain.type = vm::Marker::LINE_LIST;
                    for (int row = 0; row < cloth_height_num; ++row) {
                        for (int col = 0; col < cloth_width_num; ++col) {
                            if (row != cloth_height_num - 1 && col != cloth_width_num - 1) {
                                geometry_msgs::Point cur;
                                geometry_msgs::Point right;
                                geometry_msgs::Point below;
                                int cur_ind = col*cloth_height_num + row;
                                int right_ind = cur_ind + cloth_height_num;
                                int below_ind = cur_ind + 1;

                                cur.x = (pc_iter_comp+cur_ind)->x;
                                cur.y = (pc_iter_comp+cur_ind)->y;
                                cur.z = (pc_iter_comp+cur_ind)->z;

                                right.x = (pc_iter_comp+right_ind)->x;
                                right.y = (pc_iter_comp+right_ind)->y;
                                right.z = (pc_iter_comp+right_ind)->z;

                                below.x = (pc_iter_comp+below_ind)->x;
                                below.y = (pc_iter_comp+below_ind)->y;
                                below.z = (pc_iter_comp+below_ind)->z;

                                order_without_constrain.points.push_back(cur);
                                order_without_constrain.points.push_back(right);

                                order_without_constrain.points.push_back(cur);
                                order_without_constrain.points.push_back(below);
                            }
                            else if (row == cloth_height_num - 1 && col != cloth_width_num - 1) {
                                geometry_msgs::Point cur;
                                geometry_msgs::Point right;
                                int cur_ind = col*cloth_height_num + row;
                                int right_ind = cur_ind + cloth_height_num;

                                cur.x = (pc_iter_comp+cur_ind)->x;
                                cur.y = (pc_iter_comp+cur_ind)->y;
                                cur.z = (pc_iter_comp+cur_ind)->z;

                                right.x = (pc_iter_comp+right_ind)->x;
                                right.y = (pc_iter_comp+right_ind)->y;
                                right.z = (pc_iter_comp+right_ind)->z;

                                order_without_constrain.points.push_back(cur);
                                order_without_constrain.points.push_back(right);
                            }
                            else if (row != cloth_height_num - 1 && col == cloth_width_num - 1) {
                                geometry_msgs::Point cur;
                                geometry_msgs::Point below;
                                int cur_ind = col*cloth_height_num + row;
                                int below_ind = cur_ind + 1;

                                cur.x = (pc_iter_comp+cur_ind)->x;
                                cur.y = (pc_iter_comp+cur_ind)->y;
                                cur.z = (pc_iter_comp+cur_ind)->z;

                                below.x = (pc_iter_comp+below_ind)->x;
                                below.y = (pc_iter_comp+below_ind)->y;
                                below.z = (pc_iter_comp+below_ind)->z;

                                order_without_constrain.points.push_back(cur);
                                order_without_constrain.points.push_back(below);
                            }
                        }
                    }
            #endif
            #endif

            order_pub.publish(order);
            #ifdef COMP
            order_without_constrain_pub.publish(order_without_constrain);
            #endif
        }

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
        #ifdef PREDICT
        pcl_conversions::toPCL(time, out.cpd_predict->header.stamp);
        #endif

        #ifdef ENTIRE
        original_publisher.publish(out.original_cloud);
        #endif
        masked_publisher.publish(out.masked_point_cloud);
        downsampled_publisher.publish(out.downsampled_cloud);
        template_publisher.publish(out.cpd_output);
        #ifdef PREDICT
        pred_publisher.publish(out.cpd_predict);
        #endif
        output_publisher.publish(out.gurobi_output);
        #ifdef COMP
        output_without_constrain_publisher.publish(out_without_constrain.gurobi_output);
        #endif

        ++color_iter;
        ++depth_iter;
        ++info_iter;
        #ifdef SIMULATION
        ++config_iter;
        ++velocity_iter;
        ++ind_iter;
        ++truth_iter;
        #endif
    }

    cout << "Test ended" << endl;

    return EXIT_SUCCESS;
}
