#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <map>
#include <sstream>

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
#include <message_filters/sync_policies/approximate_time.h>
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
#include <cdcpd/optimizer.h>

#include <cdcpd_ros/Float32MultiArrayStamped.h>
#include <victor_hardware_interface/Robotiq3FingerStatus_sync.h>
#include <victor_hardware_interface/Robotiq3FingerStatus.h>


using std::cout;
using std::endl;
using std::string;
using std::stringstream;
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

typedef CGAL::Exact_predicates_inexact_constructions_kernel             K;
typedef K::FT                                                           FT;
typedef K::Point_3                                                      Point_3;
typedef K::Ray_3                                                        Ray_3;
typedef K::Vector_3                                                     Vector;
typedef CGAL::Surface_mesh<Point_3>                                     Mesh;
typedef boost::graph_traits<Mesh>::vertex_descriptor                    vertex_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor                      face_descriptor;

typedef message_filters::sync_policies::ApproximateTime<sm::Image, 
                                                        sm::Image,
                                                        sm::CameraInfo,
														cdcpd_ros::Float32MultiArrayStamped,
														cdcpd_ros::Float32MultiArrayStamped,
														victor_hardware_interface::Robotiq3FingerStatus_sync,
														victor_hardware_interface::Robotiq3FingerStatus_sync> SyncPolicy;



std::vector<sm::Image::ConstPtr> color_images;
std::vector<sm::Image::ConstPtr> depth_images;
std::vector<sm::CameraInfo::ConstPtr> camera_infos;
#ifdef SIMULATION
std::vector<stdm::Float32MultiArray::ConstPtr> grippers_config;
std::vector<stdm::Float32MultiArray::ConstPtr> grippers_dot;
std::vector<stdm::Float32MultiArray::ConstPtr> grippers_ind;
std::vector<stdm::Float32MultiArray::ConstPtr> ground_truth;
stdm::Float32MultiArray::ConstPtr verts_ptr;
stdm::Float32MultiArray::ConstPtr normals_ptr;
stdm::Float32MultiArray::ConstPtr faces_ptr;
#else
std::vector<cdcpd_ros::Float32MultiArrayStamped::ConstPtr> grippers_config;
std::vector<cdcpd_ros::Float32MultiArrayStamped::ConstPtr> grippers_dot;
std::vector<victor_hardware_interface::Robotiq3FingerStatus_sync::ConstPtr> l_status;
std::vector<victor_hardware_interface::Robotiq3FingerStatus_sync::ConstPtr> r_status;
// stdm::Float32MultiArray::ConstPtr verts_ptr;
// stdm::Float32MultiArray::ConstPtr normals_ptr;
// stdm::Float32MultiArray::ConstPtr faces_ptr;
#endif

#ifdef SIMULATION
std::string workingDir = "/home/deformtrack/catkin_ws/src/cdcpd_test_blender/log";
#else
std::string workingDir = "/home/deformtrack/catkin_ws/src/cdcpd_test/log";
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

vm::Marker pc_to_marker(PointCloud::Ptr pc, MatrixXi edges, string frame_id) {
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

    order.type = vm::Marker::LINE_LIST;
	for (int e = 0; e < edges.cols(); e++) {
		geometry_msgs::Point pt1;
		geometry_msgs::Point pt2;

		pt1.x = pc->points[edges(0, e)].x;
		pt1.y = pc->points[edges(0, e)].y;
		pt1.z = pc->points[edges(0, e)].z;
		
		pt2.x = pc->points[edges(1, e)].x;
		pt2.y = pc->points[edges(1, e)].y;
		pt2.z = pc->points[edges(1, e)].z;

		order.points.push_back(pt1);
		order.points.push_back(pt2);
	}
	return order;
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

// static victor_hardware_interface::Robotiq3FingerStatus::ConstPtr gripper_status_sync_to_origin(
// 	const victor_hardware_interface::Robotiq3FingerStatus_sync::ConstPtr sync)
// {
// 	victor_hardware_interface::Robotiq3FingerStatus origin;
// 	origin.header = sync->header;
// 	origin.finger_a_status = sync->finger_a_status;
// 	return std::make_shared<victor_hardware_interface::Robotiq3FingerStatus> (origin);
// }

static victor_hardware_interface::Robotiq3FingerStatus_sync::ConstPtr gripper_status_origin_to_sync(
	const victor_hardware_interface::Robotiq3FingerStatus::ConstPtr origin, int diff)
{
	victor_hardware_interface::Robotiq3FingerStatus_sync sync;
	sync.header = origin->header;
	sync.header.stamp.sec += diff;
	sync.finger_a_status = origin->finger_a_status;
	victor_hardware_interface::Robotiq3FingerStatus_sync::ConstPtr syncptr(new victor_hardware_interface::Robotiq3FingerStatus_sync(sync));
	// cout << (syncptr->header).stamp << endl;
	return syncptr;
	// return std::make_shared<victor_hardware_interface::Robotiq3FingerStatus_sync> (sync const);
}

void callback(
    const sm::Image::ConstPtr &rgb_img,
    const sm::Image::ConstPtr &depth_img,
    const sm::CameraInfo::ConstPtr &cam_info,
    const cdcpd_ros::Float32MultiArrayStamped::ConstPtr &g_config,
    const cdcpd_ros::Float32MultiArrayStamped::ConstPtr &g_dot,
	const victor_hardware_interface::Robotiq3FingerStatus_sync::ConstPtr &l_s,
	const victor_hardware_interface::Robotiq3FingerStatus_sync::ConstPtr &r_s)
{
    color_images.push_back(rgb_img);
    depth_images.push_back(depth_img);
    camera_infos.push_back(cam_info);
    grippers_config.push_back(g_config);
    grippers_dot.push_back(g_dot);
	l_status.push_back(l_s);
	r_status.push_back(r_s);
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

std::vector<bool> toGripperStatus(
	const victor_hardware_interface::Robotiq3FingerStatus_sync::ConstPtr &l_ptr,
	const victor_hardware_interface::Robotiq3FingerStatus_sync::ConstPtr &r_ptr)
{
	std::vector<bool> gripper_status;
	gripper_status.resize(2);
	gripper_status[0] = (l_ptr->finger_a_status).position > 0.4;
	gripper_status[1] = (r_ptr->finger_a_status).position > 0.4;
	return gripper_status;
}

std::tuple<AllGrippersSinglePose,
           AllGrippersSinglePoseDelta> toGripperConfig(
    const cdcpd_ros::Float32MultiArrayStamped::ConstPtr &g_config,
    const cdcpd_ros::Float32MultiArrayStamped::ConstPtr &g_dot)
{
    uint32_t num_gripper = ((g_config->data).layout).dim[0].size;
    uint32_t num_config = ((g_config->data).layout).dim[1].size;
    uint32_t num_dot = ((g_dot->data).layout).dim[1].size;

    std::cout << "num of gripper " << num_gripper << std::endl;
    std::cout << "num of config " << num_config << std::endl;
    std::cout << "num of gripper dot " << num_dot << std::endl;

    AllGrippersSinglePose one_frame_config;
    AllGrippersSinglePoseDelta one_frame_velocity;

    for (uint32_t g = 0; g < num_gripper; ++g)
    {
        Isometry3d one_config;
        Vector6d one_velocity;

        for (uint32_t row = 0; row < 4; ++row)
        {
            for (uint32_t col = 0; col < 4; ++col)
            {
                one_config(row, col) = double(((g_config->data).data)[num_config*g + row*4 + col]);
            }
        }

        for (uint32_t i = 0; i < num_dot; ++i)
        {
            one_velocity(i) = double(((g_dot->data).data)[num_dot*g + i]);
        }

        one_frame_config.push_back(one_config);
        one_frame_velocity.push_back(one_velocity);
    }

    return {one_frame_config, one_frame_velocity};
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

MatrixXf Float32MultiArrayPtr2MatrixXf(const stdm::Float32MultiArray::ConstPtr& array_ptr)
{
	int row = (array_ptr->layout).dim[0].size;
	int col = (array_ptr->layout).dim[1].size;
	MatrixXf mat(row, col);
    cout << "row: " << row << endl;
    cout << "col: " << col << endl;
    cout << (array_ptr->data).size();

	for (int r = 0; r < row; r++)
	{
		for (int c = 0; c < col; c++)
		{
			mat(r, c) = (array_ptr->data)[r*col+c];
		}
	}
	return mat;	
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

float calc_mean_error(const Matrix3Xf Y, const Matrix3Xf truth) {
	int num = std::min(int(Y.cols()), int(truth.cols()));
	float error = 0;
	for(int i = 0; i < num; i++)
	{
		error += (Y.col(i) - truth.col(i)).norm();
	}
	error /= float(num);
    return error;
	// return ((Y-truth).colwise().norm()).mean();
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

std::tuple<Eigen::Matrix3Xf, Eigen::Matrix2Xi> init_template()
{
    #ifdef ROPE
	// This is for simulation
    // float left_x = -0.5f; float left_y = -0.5f; float left_z = 3.0f; float right_x = 0.5f; float right_y = -0.5f; float right_z = 3.0f;
    
	// rope_edge_cover_1
	float left_x = -0.5f; float left_y = -0.0f; float left_z = 1.0f; float right_x = 0.3f; float right_y = -0.0f; float right_z = 1.0f;
    
    int points_on_rope = 40;

    MatrixXf vertices(3, points_on_rope); // Y^0 in the paper
    vertices.setZero();
    vertices.row(0).setLinSpaced(points_on_rope, left_x, right_x);
    vertices.row(1).setLinSpaced(points_on_rope, left_y, right_y);
    vertices.row(2).setLinSpaced(points_on_rope, left_z, right_z);

    MatrixXi edges(2, points_on_rope - 1);
    edges(0, 0) = 0;
    edges(1, edges.cols() - 1) = points_on_rope - 1;
    for (int i = 1; i <= edges.cols() - 1; ++i)
    {
        edges(0, i) = i;
        edges(1, i - 1) = i;
    }

    #else

	// This is for simulation
    // int num_width = 20; int num_height = 20; float right_up_y = 0.19f; float right_up_x = 0.19f; float left_bottom_y = -0.19f; float left_bottom_x = -0.19f; float z = 2.0f;
    int num_width = 15; int num_height = 15; float right_up_y = 0.14f; float right_up_x = 0.14f; float left_bottom_y = -0.14f; float left_bottom_x = -0.14f; float z = 1.0f;

    Eigen::Matrix3Xf vertices = Eigen::Matrix3Xf::Zero(3, num_width * num_height);
    Eigen::Matrix2Xi edges = Eigen::Matrix2Xi::Zero(2, (num_width - 1) * num_height + (num_height - 1) * num_width);

    int edge_count = 0;
    for (int i = 0; i < num_height; ++i)
    {
        for (int j = 0; j < num_width; ++j)
        {
            int index = j * num_height + i;
            float ratio_x = static_cast<float>(j) / static_cast<float>(num_width - 1);
            float ratio_y = static_cast<float>(i) / static_cast<float>(num_height - 1);
            vertices(0, index) = (1-ratio_x) * right_up_x + (ratio_x) * left_bottom_x;
            vertices(1, index) = (1-ratio_y) * right_up_y + (ratio_y) * left_bottom_y;
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
    #endif

    return std::make_tuple(vertices, edges);
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr Matrix3Xf2pcptr(const Eigen::Matrix3Xf& template_vertices) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < template_vertices.cols(); ++i) 
    {    
        const auto& c = template_vertices.col(i);
        template_cloud->push_back(pcl::PointXYZ(c(0), c(1), c(2)));
    }
	return template_cloud;
}

#ifdef SHAPE_COMP
static vm::Marker obsParam2Mesh(const obsParam& obs, const string& frame_id, const ros::Time time)
{
	vm::Marker mesh_msg;
	mesh_msg.type = vm::Marker::TRIANGLE_LIST;
	mesh_msg.header.frame_id = frame_id;
    mesh_msg.header.stamp = time;
    mesh_msg.ns = "mesh";
    mesh_msg.id = 0; 
    mesh_msg.action = vm::Marker::ADD;
	
	mesh_msg.pose.position.x = 0.0;
    mesh_msg.pose.position.y = 0.0;
    mesh_msg.pose.position.z = 0.0;

    mesh_msg.pose.orientation.x = 0.0;
    mesh_msg.pose.orientation.y = 0.0;
    mesh_msg.pose.orientation.z = 0.0;
    mesh_msg.pose.orientation.w = 1.0;
    mesh_msg.scale.x = 1.0;
    mesh_msg.scale.y = 1.0;
    mesh_msg.scale.z = 1.0;
	
	mesh_msg.color.a = 1.0;
	mesh_msg.color.r = 1.0;
	mesh_msg.color.g = 0.0;
	mesh_msg.color.b = 0.0;
	
	Mesh mesh = initObstacle(obs);

	for(auto fd: faces(mesh))
	{
		for(auto vd: vertices_around_face(mesh.halfedge(fd), mesh))
		{
			K::Point_3 pt = mesh.point(vd);
			
			gm::Point vert;
        	vert.x = pt.x();
        	vert.y = pt.y();
        	vert.z = pt.z();

			mesh_msg.points.push_back(vert);
		}
	}
	return mesh_msg;

}
#endif

/*
static vm::Marker obsParam2Mesh(const obsParam& obs, const string& frame_id, const ros::Time time)
{
	vm::Marker mesh_msg;
	mesh_msg.type = vm::Marker::TRIANGLE_LIST;
	mesh_msg.header.frame_id = frame_id;
    mesh_msg.header.stamp = time;
    mesh_msg.ns = "mesh";
    mesh_msg.id = 0; 
    mesh_msg.action = vm::Marker::ADD;
	
	mesh_msg.pose.position.x = 0.0;
    mesh_msg.pose.position.y = 0.0;
    mesh_msg.pose.position.z = 0.0;


    mesh_msg.pose.orientation.x = 0.0;
    mesh_msg.pose.orientation.y = 0.0;
    mesh_msg.pose.orientation.z = 0.0;
    mesh_msg.pose.orientation.w = 1.0;
    mesh_msg.scale.x = 1.0;
    mesh_msg.scale.y = 1.0;
    mesh_msg.scale.z = 1.0;
	
	mesh_msg.color.a = 1.0;
	mesh_msg.color.r = 1.0;
	mesh_msg.color.g = 0.0;
	mesh_msg.color.b = 0.0;

	for(int face_ind = 0; face_ind < obs.faces.cols(); face_ind++)
	{
		for(int i = 2; i >= 0; i--)
		{
			uint32_t pt_ind = uint32_t(obs.faces(i, face_ind));
			
			gm::Point vert;
        	vert.x = obs.verts(0, pt_ind);
        	vert.y = obs.verts(1, pt_ind);
        	vert.z = obs.verts(2, pt_ind);

			mesh_msg.points.push_back(vert);
		}
	}
	return mesh_msg;
}
 */
void test_velocity_calc() {
	Eigen::Isometry3d g_current;
	Eigen::Isometry3d g_next;
	g_current(0,0) = -0.28453702; g_current(0,1) = -0.95784968; g_current(0,2) = 0.03952325; g_current(0,3) =  0.49164355;
	g_current(1,0) =  0.27036124; g_current(1,1) = -0.04062294; g_current(1,2) = 0.96190161; g_current(1,3) = -0.49191713;
	g_current(2,0) = -0.91975188; g_current(2,1) =  0.28438324; g_current(2,2) = 0.27052405; g_current(2,3) =  2.973032  ;
	g_current(3,0) =  0.        ; g_current(3,1) =  0.        ; g_current(3,2) = 0.        ; g_current(3,3) =  1.        ;
    
	g_next(0,0) = -0.44558436; g_next(0,1) = -0.88946199; g_next(0,2) =  0.10154484; g_next(0,3) =  0.47975183;
    g_next(1,0) =  0.4167676 ; g_next(1,1) = -0.1057118 ; g_next(1,2) =  0.90284544; g_next(1,3) = -0.4806639 ;
    g_next(2,0) = -0.79231262; g_next(2,1) =  0.44461477; g_next(2,2) =  0.41780227; g_next(2,3) =  2.94883704;
    g_next(3,0) =  0.        ; g_next(3,1) =  0.        ; g_next(3,2) =  0.        ; g_next(3,3) =  1.        ;
	
	cout << "v: " << kinematics::calculateVelocity(g_current, g_next, 1.0) << endl;	

	exit(-1);
}

// static void pub_pc() {
	// TODO
// }

int main(int argc, char* argv[])
{
    // test_nearest_line();
    // test_lle();
    // ENHANCE: more smart way to get Y^0 and E
	//test_velocity_calc();
    ros::init(argc, argv, "cdcpd_bagfile");
    cout << "Starting up..." << endl;
	

    auto [template_vertices, template_edges] = init_template();

	// cout << template_vertices << endl;
    // cout << template_edges << endl;
	
	#ifdef ROPE
    int points_on_rope = 50;
    #else
    // int cloth_width_num = 20;
    // int cloth_height_num = 20;
    #endif
	
	
	std::string cleancmd = "exec rm -rf " + workingDir + "/*";
	if (system(cleancmd.c_str()) != 0) {
		std::cerr << "wrong clean files" << std::endl;
		exit(1);
	}	

    // #ifdef DEBUG
    // clean_file(workingDir + "/cpp_entire_cloud.txt");
    // clean_file(workingDir + "/cpp_downsample.txt");
    // clean_file(workingDir + "/cpp_TY.txt");
    // clean_file(workingDir + "/cpp_Y_opt.txt");
    // clean_file(workingDir + "/cpp_TY-1.txt");
    // clean_file(workingDir + "/cpp_hsv.txt");
    // clean_file(workingDir + "/cpp_mask.txt");
    // clean_file(workingDir + "/cpp_intrinsics.txt");
    // #endif
    // clean_file(workingDir + "/occluded_index.txt");
    // clean_file(workingDir + "/error.txt");
    // clean_file(workingDir + "/error_no_pred.txt");

	ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    // Publsihers for the data, some visualizations, others consumed by other nodes
    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud = Matrix3Xf2pcptr(template_vertices);

	auto original_publisher = nh.advertise<PointCloud> ("cdcpd/original", 1);
    auto masked_publisher = nh.advertise<PointCloud> ("cdcpd/masked", 1);
    auto downsampled_publisher = nh.advertise<PointCloud> ("cdcpd/downsampled", 1);
    auto template_publisher = nh.advertise<PointCloud> ("cdcpd/template", 1);
    #ifdef PREDICT
    auto pred_publisher = nh.advertise<PointCloud>("cdcpd/prediction", 1);
    #endif
    // auto cpd_iters_publisher = nh.advertise<PointCloud> ("cdcpd/cpd_iters", 1);
    auto output_publisher = nh.advertise<PointCloud> ("cdcpd/output", 1);
    auto left_gripper_pub = nh.advertise<gm::TransformStamped>("cdcpd/left_gripper_prior", 1);
    auto right_gripper_pub = nh.advertise<gm::TransformStamped>("cdcpd/right_gripper_prior", 1);
    #ifdef CYLINDER
    auto cylinder_pub = nh.advertise<vm::Marker>("cdcpd/cylinder", 0);
    #endif
    auto order_pub = nh.advertise<vm::Marker>("cdcpd/order", 10);
	auto mesh_pub = nh.advertise<vm::Marker>("cdcpd/mesh", 10);
	auto cpd_physics_pub = nh.advertise<PointCloud> ("cdcpd/cpd_physics", 1);
	BagSubscriber<sm::Image> rgb_sub, depth_sub;
    BagSubscriber<sm::CameraInfo> info_sub;
    // message_filters::Subscriber<sm::Image> rgb_sub(nh, "/kinect2_victor_head/qhd/image_color_rect", 10);
	// message_filters::Subscriber<sm::Image> depth_sub(nh, "/kinect2_victor_head/qhd/image_depth_rect", 10);
	// message_filters::Subscriber<sm::CameraInfo> info_sub(nh, "/kinect2_victor_head/qhd/camera_info", 10);
	#ifdef SIMULATION
    BagSubscriber<stdm::Float32MultiArray> config_sub, dot_sub, ind_sub, truth_sub;
	#else
	BagSubscriber<cdcpd_ros::Float32MultiArrayStamped> config_sub, dot_sub;
	BagSubscriber<victor_hardware_interface::Robotiq3FingerStatus_sync> l_sub, r_sub;
    // message_filters::Subscriber<cdcpd_ros::Float32MultiArrayStamped> config_sub(nh, "/kinect2_victor_head/qhd/gripper_config", 10);
	// message_filters::Subscriber<cdcpd_ros::Float32MultiArrayStamped> dot_sub(nh, "/kinect2_victor_head/qhd/dot_config", 10);
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
    #ifdef SHAPE_COMP
	topics.push_back(std::string("comp_vertices"));
	topics.push_back(std::string("comp_faces"));
	topics.push_back(std::string("comp_normals"));
    #endif
    #else
    topics.push_back(std::string("/kinect2_victor_head/qhd/image_color_rect"));
    topics.push_back(std::string("/kinect2_victor_head/qhd/image_depth_rect"));
    topics.push_back(std::string("/kinect2_victor_head/qhd/camera_info"));
    topics.push_back(std::string("/kinect2_victor_head/qhd/gripper_config"));
    topics.push_back(std::string("/kinect2_victor_head/qhd/dot_config"));
    // topics.push_back(std::string("/kinect2_victor_head/qhd/gripper_info"));
	topics.push_back(std::string("/left_arm/gripper_status"));
	topics.push_back(std::string("/right_arm/gripper_status"));
	#endif

    auto const bagfile = ROSHelpers::GetParam<std::string>(ph, "bagfile", "normal");
    #ifdef SIMULATION
    auto const folder = ros::package::getPath("cdcpd_ros") + "/../cdcpd_test_blender/dataset/";
    #else
    auto const folder = ros::package::getPath("cdcpd_ros") + "/../cdcpd_test/dataset/09_19_2020/";
    #endif
    rosbag::Bag bag(folder + bagfile + ".bag", rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Go through the bagfile, storing matched image pairs
    // TODO this might be too much memory at some point
    // #ifndef SIMULATION
    // auto sync = message_filters::TimeSynchronizer<sm::Image, sm::Image, sm::CameraInfo, cdcpd_ros::Float32MultiArrayStamped, cdcpd_ros::Float32MultiArrayStamped, victor_hardware_interface::Robotiq3FingerStatus, victor_hardware_interface::Robotiq3FingerStatus>(
    //        rgb_sub, depth_sub, info_sub, config_sub, dot_sub, l_sub, r_sub, 25);
	auto sync = message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), rgb_sub, depth_sub, info_sub, config_sub, dot_sub, l_sub, r_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6, _7));
    // #endif
	// ros::spin();

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
        #ifdef SHAPE_COMP
		else if (m.getTopic() == topics[7])
		{
			auto info = m.instantiate<stdm::Float32MultiArray>();
            if (info != nullptr)
            {
                verts_ptr = info;
                // config_sub.newMessage(info);
            }
            else
            {
                cout << "NULL initiation!" << endl;
            }
		}
		else if (m.getTopic() == topics[8])
        {
            auto info = m.instantiate<stdm::Float32MultiArray>();
            if (info != nullptr)
            {
                faces_ptr = info;
                // config_sub.newMessage(info);
            }
            else
            {
                cout << "NULL initiation!" << endl;
            }
        }
		else if (m.getTopic() == topics[9])
        {
            auto info = m.instantiate<stdm::Float32MultiArray>();
            if (info != nullptr)
            {
                normals_ptr = info;
                // config_sub.newMessage(info);
            }
            else
            {
                cout << "NULL initiation!" << endl;
            }
        }
        #endif
		#else
		// topics in exp bagfile
		else if (m.getTopic() == topics[3])
		{
			auto info = m.instantiate<cdcpd_ros::Float32MultiArrayStamped>();
            if (info != nullptr)
            {
				config_sub.newMessage(info);
            }
            else
            {
                cout << "NULL initiation!" << endl;
            }
		}
		else if (m.getTopic() == topics[4])
		{
			auto info = m.instantiate<cdcpd_ros::Float32MultiArrayStamped>();
            if (info != nullptr)
            {
                dot_sub.newMessage(info);
            }
            else
            {
                cout << "NULL initiation!" << endl;
            }
		}
		else if (m.getTopic() == topics[5])
		{
			auto info = m.instantiate<victor_hardware_interface::Robotiq3FingerStatus>();
            if (info != nullptr)
            {
                l_sub.newMessage(gripper_status_origin_to_sync(info, -14564));
            }
            else
            {
                cout << "NULL initiation!" << endl;
            }
		}
		else if (m.getTopic() == topics[6])
		{
			auto info = m.instantiate<victor_hardware_interface::Robotiq3FingerStatus>();
            if (info != nullptr)
            {
                r_sub.newMessage(gripper_status_origin_to_sync(info, -15097));
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

    cout << "rgb images size: " << color_images.size() << endl;
    cout << "depth images size: " << depth_images.size() << endl;
    cout << "camera infos size: " << camera_infos.size() << endl;
	cout << "config size: " << grippers_config.size() << endl;
    cout << "velocity size: " << grippers_dot.size() << endl;

	
	auto color_iter = color_images.cbegin();
    auto depth_iter = depth_images.cbegin();
    auto info_iter = camera_infos.cbegin();
    #ifdef SIMULATION
    auto config_iter = grippers_config.cbegin();
    auto velocity_iter = grippers_dot.cbegin();
    auto ind_iter = grippers_ind.cbegin();
    auto truth_iter = ground_truth.cbegin();
	#else
	auto config_iter = grippers_config.cbegin();
    auto velocity_iter = grippers_dot.cbegin();
    auto l_iter = l_status.cbegin();
	auto r_iter = r_status.cbegin();
    #endif

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

    #ifdef SHAPE_COMP
	obsParam obstacle_param;
	obstacle_param.verts = Float32MultiArrayPtr2MatrixXf(verts_ptr);
	obstacle_param.faces = Float32MultiArrayPtr2MatrixXf(faces_ptr);
	obstacle_param.normals = Float32MultiArrayPtr2MatrixXf(normals_ptr);
    #endif
	
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
                #ifdef SHAPE_COMP
				obstacle_param,
                #endif
                false,
                alpha,
                beta,
                lambda,
                k_spring);
	#else
    std::shared_ptr<ros::NodeHandle> nh_ptr = std::make_shared<ros::NodeHandle>(nh);
    double translation_dir_deformability = 10.0;
    double translation_dis_deformability = 10.0;
    double rotation_deformability = 10.0;
    CDCPD cdcpd(template_cloud,
                template_edges,
                #ifdef SHAPE_COMP
				obstacle_param,
                #endif
                false,
                alpha,
                beta,
                lambda,
                k_spring);
	#endif

    #ifdef COMP
	pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud_without_constrain = Matrix3Xf2pcptr(template_vertices);
    auto output_without_constrain_publisher = nh.advertise<PointCloud> ("cdcpd/output_without_constrain", 1);   
	CDCPD cdcpd_without_constrain(template_cloud_without_constrain, template_edges, intrinsics, false, alpha, beta, lambda, k_spring);
    #endif

    #ifdef COMP_NOPRED
    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud_without_prediction = Matrix3Xf2pcptr(template_vertices);
    auto output_without_prediction_publisher = nh.advertise<PointCloud> ("cdcpd/output_without_prediction", 1);
	CDCPD cdcpd_without_prediction(template_cloud,
                template_edges,
                nh_ptr,
                translation_dir_deformability,
                translation_dis_deformability,
                rotation_deformability,
                g_ind,
                #ifdef SHAPE_COMP
                obstacle_param,
                #endif
                false,
                alpha,
                beta,
                lambda,
                k_spring);
    #endif

	#ifdef COMP_PRED1
	pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud_pred1 = Matrix3Xf2pcptr(template_vertices);
    auto output_pred1_publisher = nh.advertise<PointCloud> ("cdcpd/output_pred_deform_model", 1);
	CDCPD cdcpd_pred1(template_cloud_pred1,
                template_edges,
                nh_ptr,
                translation_dir_deformability,
                translation_dis_deformability,
                rotation_deformability,
                g_ind,
                #ifdef SHAPE_COMP
                obstacle_param,
                #endif
                false,
                alpha,
                beta,
                lambda,
                k_spring);
    #endif	

	#ifdef COMP_PRED2
	pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud_pred2 = Matrix3Xf2pcptr(template_vertices);
    auto output_pred2_publisher = nh.advertise<PointCloud> ("cdcpd/output_pred_J_model", 1);
	CDCPD cdcpd_pred2(template_cloud_pred2,
                template_edges,
                nh_ptr,
                translation_dir_deformability,
                translation_dis_deformability,
                rotation_deformability,
                g_ind,
                #ifdef SHAPE_COMP
                obstacle_param,
                #endif
                false,
                alpha,
                beta,
                lambda,
                k_spring);
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
    int frame = 0;
    while(color_iter != color_images.cend() && depth_iter != depth_images.cend() && info_iter != camera_infos.cend())
    {
        cout << "\n-------------------- frame " << frame << " --------------------" << endl << endl;
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
		#else
    	auto [g_config, g_dot] = toGripperConfig(*config_iter, *velocity_iter);
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
        cv::inRange(color_hsv, cv::Scalar(0, 0.5, 0.5), cv::Scalar(20, 1.0, 1.0), mask1);
        cv::inRange(color_hsv, cv::Scalar(340, 0.5, 0.5), cv::Scalar(360, 1.0, 1.0), mask2);
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

        auto frame_id = "kinect2_rgb_optical_frame";
        
		#ifdef SIMULATION
		cout << "prediction choice: 0" << endl;
        auto out = cdcpd(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud, g_dot, g_config, true, true, true, 0, fixed_points);
        std::ofstream(workingDir + "/error.txt", std::ofstream::app) << calc_mean_error(out.gurobi_output->getMatrixXfMap(), one_frame_truth) << " ";
        template_cloud = out.gurobi_output;
        #else
		auto is_grasped = toGripperStatus(*l_iter, *r_iter);
		for (auto& dot: g_dot)
		{
			dot = dot/100;
		}
        auto out = cdcpd(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud, g_dot, g_config, is_grasped, nh_ptr, translation_dir_deformability, translation_dis_deformability, rotation_deformability, true, true, true, 2, fixed_points);
        template_cloud = out.gurobi_output;
        #endif

        #ifdef COMP
        auto out_without_constrain = cdcpd_without_constrain(rgb_image, depth_image, hsv_mask, template_cloud_without_constrain, template_edges, false, false);
        template_cloud_without_constrain = out_without_constrain.gurobi_output;
		out_without_constrain.gurobi_output->header.frame_id = frame_id;
        #endif

        #ifdef COMP_NOPRED
		cout << "no prediction used" << endl;
        auto out_without_prediction = cdcpd_without_prediction(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud_without_prediction, g_dot, g_config, false, false, false, 0, fixed_points);
        template_cloud_without_prediction = out_without_prediction.gurobi_output;
		out_without_prediction.gurobi_output->header.frame_id = frame_id;
		std::ofstream(workingDir + "/error_no_pred.txt", std::ofstream::app) << calc_mean_error(out_without_prediction.gurobi_output->getMatrixXfMap(), one_frame_truth) << " ";
        #endif

        #ifdef COMP_PRED1
		cout << "prediction choice: 1" << endl;
        auto out_pred1 = cdcpd_pred1(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud_pred1, g_dot, g_config, true, true, true, 1, fixed_points);
        template_cloud_pred1 = out_pred1.gurobi_output;
		out_pred1.gurobi_output->header.frame_id = frame_id;
		std::ofstream(workingDir + "/error_pred1.txt", std::ofstream::app) << calc_mean_error(out_pred1.gurobi_output->getMatrixXfMap(), one_frame_truth) << " ";
        #endif

        #ifdef COMP_PRED2
		cout << "prediction choice: 2" << endl;
        auto out_pred2 = cdcpd_pred2(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud_pred2, g_dot, g_config, true, true, true, 2, fixed_points);
        template_cloud_pred2 = out_pred2.gurobi_output;
		out_pred2.gurobi_output->header.frame_id = frame_id;
		std::ofstream(workingDir + "/error_pred2.txt", std::ofstream::app) << calc_mean_error(out_pred2.gurobi_output->getMatrixXfMap(), one_frame_truth) << " ";
        #endif

		ifstream cpd_phy_result;
		PointCloud::Ptr cpd_phy_pc(new PointCloud);
  		cpd_phy_result.open(ros::package::getPath("cdcpd_ros") + "/src/cpd_physics/" + bagfile + "/result" + std::to_string(frame) + ".txt", ios::in);
		// cout << "try to open cpd physics file " << ros::package::getPath("cdcpd_ros") + "/src/cpd_physics/" + bagfile + "/result" + std::to_string(frame) + ".txt" << endl;
		if(cpd_phy_result.is_open())
		{
			// cout << "open successfully" << endl;
			string line;
			std::vector<float> xs, ys, zs;
			for (int line_idx = 0; line_idx < 3; line_idx++)
			{
				float buf;
				getline(cpd_phy_result, line);
				stringstream stream(line);
				while(!stream.eof())
				{
					stream >> buf;
					if (line_idx == 0) {
						xs.push_back(buf);
					}
					else if (line_idx == 1) {
						ys.push_back(buf);
					}
					else {
						zs.push_back(buf);
					}
				}
			}
			cpd_phy_pc->points.reserve(xs.size());
			for(unsigned i = 0; i < xs.size(); i++)
			{
				cpd_phy_pc->push_back(pcl::PointXYZ(xs[i], ys[i], zs[i]));
			}
			// cout << "cpd physics" << endl;
			// cout << cpd_phy_pc->getMatrixXfMap().topRows(3) << endl;
			// cout << "ground truth" << endl;
			// cout << one_frame_truth << endl;
			// cout << "cpd phy error" << endl;
			// cout << calc_mean_error(cpd_phy_pc->getMatrixXfMap().topRows(3).rowwise().reverse(), one_frame_truth) << endl;
			#ifdef SIMULATION
				std::ofstream(workingDir + "/error_cpd_physics.txt", std::ofstream::app) << calc_mean_error(cpd_phy_pc->getMatrixXfMap().topRows(3).rowwise().reverse(), one_frame_truth) << " ";
			#endif
		}

		cpd_phy_pc->header.frame_id = frame_id;

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
            vm::Marker order = pc_to_marker(out.gurobi_output, template_edges, frame_id);
            order_pub.publish(order);
            #ifdef COMP
            vm::Marker order_without_constrain = pc_to_marker(out.gurobi_output, template_edges);
            order_without_constrain_pub.publish(order_without_constrain);
            #endif
        }

		auto time = ros::Time::now();
		
		#ifdef SHAPE_COMP
		vm::Marker mesh_msg = obsParam2Mesh(obstacle_param, frame_id, time);
		mesh_pub.publish(mesh_msg);
		#endif
		
        #ifdef ENTIRE
        pcl_conversions::toPCL(time, out.original_cloud->header.stamp);
        #endif
        pcl_conversions::toPCL(time, out.masked_point_cloud->header.stamp);
        pcl_conversions::toPCL(time, out.downsampled_cloud->header.stamp);
        pcl_conversions::toPCL(time, out.cpd_output->header.stamp);
        pcl_conversions::toPCL(time, out.gurobi_output->header.stamp);
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
		
		pcl_conversions::toPCL(time, cpd_phy_pc->header.stamp);	
		cpd_physics_pub.publish(cpd_phy_pc);

        #ifdef COMP
        pcl_conversions::toPCL(time, out_without_constrain.gurobi_output->header.stamp);
        output_without_constrain_publisher.publish(out_without_constrain.gurobi_output);
		#endif
		
        #ifdef COMP_NOPRED
        pcl_conversions::toPCL(time, out_without_prediction.gurobi_output->header.stamp);
		output_without_prediction_publisher.publish(out_without_prediction.gurobi_output);
        #endif

        #ifdef COMP_PRED1
        pcl_conversions::toPCL(time, out_pred1.gurobi_output->header.stamp);
		output_pred1_publisher.publish(out_pred1.gurobi_output);
        #endif

        #ifdef COMP_PRED2
        pcl_conversions::toPCL(time, out_pred2.gurobi_output->header.stamp);
		output_pred2_publisher.publish(out_pred2.gurobi_output);
        #endif

        ++color_iter;
        ++depth_iter;
        ++info_iter;
		#ifdef SIMULATION
        ++config_iter;
        ++velocity_iter;
        ++ind_iter;
        ++truth_iter;
		#else
        ++config_iter;
        ++velocity_iter;
		++l_iter;
		++r_iter;
        #endif
        ++frame;
    }

    cout << "Test ended" << endl;

    return EXIT_SUCCESS;
}
