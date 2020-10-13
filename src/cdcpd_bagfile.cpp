#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <map>
#include <sstream>
#include <ctime>

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

#include <opencv2/core/eigen.hpp>

#include <cdcpd_ros/Float32MultiArrayStamped.h>
#include <victor_hardware_interface/Robotiq3FingerStatus_sync.h>
#include <victor_hardware_interface/Robotiq3FingerStatus.h>

using std::cout;
using std::endl;
using std::string;
using std::stringstream;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using cv::Matx33d;
using Eigen::MatrixXf;
using Eigen::MatrixXi;
using Eigen::Matrix2Xi;
using Eigen::Matrix2Xf;
using Eigen::Matrix3Xf;
using Eigen::Matrix3Xd;
using Eigen::Isometry3d;
using Eigen::VectorXf;
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

/*
typedef message_filters::sync_policies::ApproximateTime<cdcpd_ros::Float32MultiArrayStamped,
														cdcpd_ros::Float32MultiArrayStamped,
														victor_hardware_interface::Robotiq3FingerStatus_sync,
														victor_hardware_interface::Robotiq3FingerStatus_sync> SyncPolicy;
 */

typedef message_filters::sync_policies::ApproximateTime<sm::Image, 
                                                        sm::Image,
                                                        sm::CameraInfo,
														cdcpd_ros::Float32MultiArrayStamped,
														cdcpd_ros::Float32MultiArrayStamped,
														victor_hardware_interface::Robotiq3FingerStatus_sync,
														victor_hardware_interface::Robotiq3FingerStatus_sync> SyncPolicy_use_gripper;

typedef message_filters::sync_policies::ApproximateTime<sm::Image, 
                                                        sm::Image,
                                                        sm::CameraInfo> SyncPolicy_img;

std::vector<sm::Image::ConstPtr> color_images;
std::vector<sm::Image::ConstPtr> depth_images;
std::vector<sm::CameraInfo::ConstPtr> camera_infos;
std::vector<stdm::Float32MultiArray::ConstPtr> grippers_config_sim;
std::vector<stdm::Float32MultiArray::ConstPtr> grippers_dot_sim;
std::vector<stdm::Float32MultiArray::ConstPtr> grippers_ind;
std::vector<stdm::Float32MultiArray::ConstPtr> ground_truth;
std::vector<cdcpd_ros::Float32MultiArrayStamped::ConstPtr> grippers_config;
std::vector<cdcpd_ros::Float32MultiArrayStamped::ConstPtr> grippers_dot;
std::vector<victor_hardware_interface::Robotiq3FingerStatus_sync::ConstPtr> l_status;
std::vector<victor_hardware_interface::Robotiq3FingerStatus_sync::ConstPtr> r_status;
	// stdm::Float32MultiArray::ConstPtr verts_ptr;
	// stdm::Float32MultiArray::ConstPtr normals_ptr;
	// stdm::Float32MultiArray::ConstPtr faces_ptr;

#ifdef SHAPE_COMP
stdm::Float32MultiArray::ConstPtr verts_ptr;
stdm::Float32MultiArray::ConstPtr normals_ptr;
stdm::Float32MultiArray::ConstPtr faces_ptr;
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

cv::Mat draw_vis(const cv::Mat& rgb_image,
				 const Matrix3Xf& template_vertices,
				 const Matrix2Xi& template_edges,
				 const Matx33d& intrinsics) {
	// project 3D point to 2D plane
	Matrix3d intri_eigen_temp;
	cv::cv2eigen(intrinsics, intri_eigen_temp);
	Matrix3f intri_eigen = intri_eigen_temp.cast<float>();
	Matrix3Xf proj_mat = intri_eigen * template_vertices;
	proj_mat.array().rowwise() /= proj_mat.row(2).array();	
	Matrix2Xf pixels_temp = proj_mat.topRows(2);
	Matrix2Xi pixels = pixels_temp.cast<int>();
	cv::Scalar red( 255, 0, 0 );
	cv::Scalar blue( 0, 0, 255 );
	cv::Scalar yellow(255, 255, 0);

	// padding the image (some points are out of image)
	cv::Mat out;
	int top, bottom, left, right;
	top = (int) (0.1*rgb_image.rows); bottom = top;
    left = (int) (0.1*rgb_image.cols); right = left;
	int borderType = cv::BORDER_CONSTANT;
	cv::Scalar white(255, 255, 255);
	cv::copyMakeBorder( rgb_image, out, top, bottom, left, right, borderType, white );
	
	for(int i = 0; i < template_vertices.cols(); i++) {
		cv::Point vertex(pixels(0, i)+left, pixels(1, i)+top);
		cv::circle(out, vertex,	2, yellow, cv::FILLED, cv::LINE_8 );
	}
	
	for(int e = 0; e < template_edges.cols(); e++) {
		int pt1 = template_edges(0, e);
		int pt2 = template_edges(1, e);
		cv::Point start(pixels(0, pt1)+left, pixels(1, pt1)+top);
		cv::Point end(pixels(0, pt2)+left, pixels(1, pt2)+top);
		cv::line(out, start, end, yellow);
	}
	cv::cvtColor(out, out, CV_BGR2RGB);
	return out;
}


vm::Marker draw_cylinder_marker(const std::vector<float>& cylinder_data,
								const std::vector<float>& quat,
								const string frame_id) {
	vm::Marker marker;

	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time();
	marker.ns = "cylinder";
	marker.id = 0;
	marker.type = vm::Marker::CYLINDER;
	marker.action = vm::Marker::ADD;

	marker.pose.position.x = cylinder_data[3];
	marker.pose.position.y = cylinder_data[4];
	marker.pose.position.z = cylinder_data[5];
	marker.pose.orientation.x = quat[0];
	marker.pose.orientation.y = quat[1];
	marker.pose.orientation.z = quat[2];
	marker.pose.orientation.w = quat[3];
	marker.scale.x = 2*cylinder_data[6];
	marker.scale.y = 2*cylinder_data[6];
	marker.scale.z = cylinder_data[7];

	marker.color.a = 0.5; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	
	return marker;
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
		
		if ((pt1.x <  0.000001 && 
			pt1.x > -0.000001 &&
			pt1.y <  0.000001 && 
			pt1.y > -0.000001 &&
			pt1.z <  0.000001 && 
			pt1.z > -0.000001) ||
			(pt2.x <  0.000001 && 
			pt2.x > -0.000001 &&
			pt2.y <  0.000001 && 
			pt2.y > -0.000001 &&
			pt2.z <  0.000001 && 
			pt2.z > -0.000001)) {
			continue;
		} else {
			order.points.push_back(pt1);
			order.points.push_back(pt2);
		}
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

/*
void im_callback(
    const sm::Image::ConstPtr &rgb_img,
    const sm::Image::ConstPtr &depth_img,
    const sm::CameraInfo::ConstPtr &cam_info)
{
    color_images.push_back(rgb_img);
    depth_images.push_back(depth_img);
    camera_infos.push_back(cam_info);
}

void callback(
    const cdcpd_ros::Float32MultiArrayStamped::ConstPtr &g_config,
    const cdcpd_ros::Float32MultiArrayStamped::ConstPtr &g_dot,
	const victor_hardware_interface::Robotiq3FingerStatus_sync::ConstPtr &l_s,
	const victor_hardware_interface::Robotiq3FingerStatus_sync::ConstPtr &r_s)
{
    grippers_config.push_back(g_config);
    grippers_dot.push_back(g_dot);
	l_status.push_back(l_s);
	r_status.push_back(r_s);
}
 */

void callback_use_gripper(
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

void callback_im(
    const sm::Image::ConstPtr &rgb_img,
    const sm::Image::ConstPtr &depth_img,
    const sm::CameraInfo::ConstPtr &cam_info)
{
    color_images.push_back(rgb_img);
    depth_images.push_back(depth_img);
    camera_infos.push_back(cam_info);
}

std::tuple<cv::Mat, cv::Mat, cv::Matx33d> toOpenCv(
    const sm::Image::ConstPtr &rgb_img,
    const sm::Image::ConstPtr &depth_img,
    const sm::CameraInfo::ConstPtr &cam_info,
	bool is_sim)
{
	cv_bridge::CvImagePtr rgb_ptr;
	cv_bridge::CvImagePtr depth_ptr;
    if (is_sim) {
    	rgb_ptr = cv_bridge::toCvCopy(rgb_img, sm::image_encodings::TYPE_8UC3);
    	depth_ptr = cv_bridge::toCvCopy(depth_img, sm::image_encodings::TYPE_32FC1);
    } else {
    	rgb_ptr = cv_bridge::toCvCopy(rgb_img, sm::image_encodings::BGR8);
    	depth_ptr = cv_bridge::toCvCopy(depth_img, sm::image_encodings::TYPE_16UC1);
    }

    cv::Mat color_image = rgb_ptr->image.clone();
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

std::tuple<Eigen::Matrix3Xf, Eigen::Matrix2Xi> init_template(const bool is_rope, const Matrix3Xf& init_points, int points_on_rope, int num_width, int num_height)
{
    Eigen::Matrix3Xf vertices;
    Eigen::Matrix2Xi edges;
    if (is_rope) {
		float left_x = init_points(0, 0);
		float left_y = init_points(1, 0);
		float left_z = init_points(2, 0);
		float right_x = init_points(0, 1);
		float right_y = init_points(1, 1);
		float right_z = init_points(2, 1);
		// This is for simulation
    	// float left_x = -0.5f; float left_y = -0.5f; float left_z = 3.0f; float right_x = 0.5f; float right_y = -0.5f; float right_z = 3.0f;
    
		// rope_edge_cover_1
		// float left_x = -0.5f; float left_y = -0.0f; float left_z = 1.0f; float right_x = 0.3f; float right_y = -0.0f; float right_z = 1.0f;
	
		// rope_winding_cylinder_exp_1_comp
		// float left_x = -0.5f; float left_y = -0.2f; float left_z = 1.0f; float right_x = 0.3f; float right_y = -0.2f; float right_z = 1.0f;
	
		// rope_winding_cylinder_exp_2_comp
		// float left_x = -0.5f; float left_y = 0.2f; float left_z = 1.0f; float right_x = 0.3f; float right_y = 0.2f; float right_z = 1.0f;
    
		// rope_by_hand_3
		// float left_x = -0.5f; float left_y = 0.2f; float left_z = 2.0f; float right_x = 0.3f; float right_y = 0.2f; float right_z = 2.0f;
    
		// int points_on_rope = 40;

    	vertices.resize(3, points_on_rope); // Y^0 in the paper
    	vertices.setZero();
    	vertices.row(0).setLinSpaced(points_on_rope, left_x, right_x);
    	vertices.row(1).setLinSpaced(points_on_rope, left_y, right_y);
    	vertices.row(2).setLinSpaced(points_on_rope, left_z, right_z);

    	edges.resize(2, points_on_rope - 1);
    	edges(0, 0) = 0;
    	edges(1, edges.cols() - 1) = points_on_rope - 1;
    	for (int i = 1; i <= edges.cols() - 1; ++i)
    	{
        	edges(0, i) = i;
        	edges(1, i - 1) = i;
    	}

    } else {
		float lb_x = init_points(0, 0);
		float lb_y = init_points(1, 0);
		float lb_z = init_points(2, 0);
		float lt_x = init_points(0, 1);
		float lt_y = init_points(1, 1);
		float lt_z = init_points(2, 1);
		float rb_x = init_points(0, 2);
		float rb_y = init_points(1, 2);
		float rb_z = init_points(2, 2);
		
		// int num_width = 15;
		// int num_height = 15;
		// This is for simulation
    	// int num_width = 20; int num_height = 20; float right_up_y = 0.19f; float right_up_x = 0.19f; float left_bottom_y = -0.19f; float left_bottom_x = -0.19f; float z = 2.0f;
    	// int num_width = 15; int num_height = 15; float right_up_y = 0.14f; float right_up_x = 0.14f; float left_bottom_y = -0.14f; float left_bottom_x = -0.14f; float z = 1.0f;
		// cloth_cover_by_hand
    	// int num_width = 15; int num_height = 15; float right_up_y = 0.0f; float right_up_x = -0.1f; float left_bottom_y = -0.28f; float left_bottom_x = -0.38f; float z = 1.5f;
		// some data on 10/10
		// int num_width = 15; int num_height = 15; float rb_y = 0.13f; float rb_x = -0.3f; float rb_z = 1.2f; float lb_y = 0.13f; float lb_x = -0.58f; float lb_z = 1.2f; float lt_y = 0.27f; float lt_x = -0.58f; float lt_z = 0.96f;

    	vertices = Eigen::Matrix3Xf::Zero(3, num_width * num_height);
    	edges = Eigen::Matrix2Xi::Zero(2, (num_width - 1) * num_height + (num_height - 1) * num_width);

    	int edge_count = 0;
    	for (int i = 0; i < num_height; ++i)
    	{
        	for (int j = 0; j < num_width; ++j)
        	{
            	int index = j * num_height + i;
            	float ratio_x = static_cast<float>(j) / static_cast<float>(num_width - 1);
            	float ratio_y = static_cast<float>(i) / static_cast<float>(num_height - 1);
            	vertices(0, index) = ratio_x * (rb_x-lb_x) + ratio_y * (lt_x-lb_x) + lb_x;
            	vertices(1, index) = ratio_x * (rb_y-lb_y) + ratio_y * (lt_y-lb_y) + lb_y;
            	vertices(2, index) = ratio_x * (rb_z-lb_z) + ratio_y * (lt_z-lb_z) + lb_z;
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
	}
    assert(edge_count == (num_width - 1) * num_height + (num_height - 1) * num_width);
    

    return std::make_tuple(vertices, edges);
}

std::tuple<Eigen::Matrix3Xf, Eigen::Matrix2Xi> init_template()
{
    #ifdef ROPE
	// This is for simulation
    // float left_x = -0.5f; float left_y = -0.5f; float left_z = 3.0f; float right_x = 0.5f; float right_y = -0.5f; float right_z = 3.0f;
    
	// rope_edge_cover_1
	// float left_x = -0.5f; float left_y = -0.0f; float left_z = 1.0f; float right_x = 0.3f; float right_y = -0.0f; float right_z = 1.0f;
	
	// rope_winding_cylinder_exp_1_comp
	// float left_x = -0.5f; float left_y = -0.2f; float left_z = 1.0f; float right_x = 0.3f; float right_y = -0.2f; float right_z = 1.0f;
	
	// rope_winding_cylinder_exp_2_comp
	// float left_x = -0.5f; float left_y = 0.2f; float left_z = 1.0f; float right_x = 0.3f; float right_y = 0.2f; float right_z = 1.0f;
    
	// rope_by_hand_3
	float left_x = -0.5f; float left_y = 0.2f; float left_z = 2.0f; float right_x = 0.44f; float right_y = 0.2f; float right_z = 2.0f;
    
	int points_on_rope = 48;

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
    // int num_width = 15; int num_height = 15; float right_up_y = 0.14f; float right_up_x = 0.14f; float left_bottom_y = -0.14f; float left_bottom_x = -0.14f; float z = 1.0f;
	// cloth_cover_by_hand
    int num_width = 15; int num_height = 15; float right_up_z = 1.28f; float right_up_x = -0.1f; float left_bottom_z = 1.00f; float left_bottom_x = -0.38f; float y = 0.0f;

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
            vertices(1, index) = y; // (1-ratio_y) * right_up_y + (ratio_y) * left_bottom_y;
            vertices(2, index) = (1-ratio_y) * right_up_z + (ratio_y) * left_bottom_z; // z;
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
	


	// cout << template_vertices << endl;
    // cout << template_edges << endl;
	

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
	auto original_publisher = nh.advertise<PointCloud> ("cdcpd/original", 1);
    auto masked_publisher = nh.advertise<PointCloud> ("cdcpd/masked", 1);
    auto downsampled_publisher = nh.advertise<PointCloud> ("cdcpd/downsampled", 1);
    auto template_publisher = nh.advertise<PointCloud> ("cdcpd/template", 1);
    auto pred_publisher = nh.advertise<PointCloud>("cdcpd/prediction", 1);
    // auto cpd_iters_publisher = nh.advertise<PointCloud> ("cdcpd/cpd_iters", 1);
    auto output_publisher = nh.advertise<PointCloud> ("cdcpd/output", 1);
    auto left_gripper_pub = nh.advertise<gm::TransformStamped>("cdcpd/left_gripper_prior", 1);
    auto right_gripper_pub = nh.advertise<gm::TransformStamped>("cdcpd/right_gripper_prior", 1);
    auto cylinder_pub = nh.advertise<vm::Marker>("cdcpd/cylinder", 0);
    auto order_pub = nh.advertise<vm::Marker>("cdcpd/order", 10);
	auto mesh_pub = nh.advertise<vm::Marker>("cdcpd/mesh", 10);
	auto cpd_physics_pub = nh.advertise<PointCloud> ("cdcpd/cpd_physics", 1);
    auto order_cpdphysics_pub = nh.advertise<vm::Marker>("cdcpd/cpdphysics_order", 10);
	BagSubscriber<sm::Image> rgb_sub_use_gripper, depth_sub_use_gripper;
    BagSubscriber<sm::CameraInfo> info_sub_use_gripper;
	BagSubscriber<sm::Image> rgb_sub, depth_sub;
    BagSubscriber<sm::CameraInfo> info_sub;
    // message_filters::Subscriber<sm::Image> rgb_sub(nh, "/kinect2_victor_head/qhd/image_color_rect", 10);
	// message_filters::Subscriber<sm::Image> depth_sub(nh, "/kinect2_victor_head/qhd/image_depth_rect", 10);
	// message_filters::Subscriber<sm::CameraInfo> info_sub(nh, "/kinect2_victor_head/qhd/camera_info", 10);
	BagSubscriber<cdcpd_ros::Float32MultiArrayStamped> config_sub, dot_sub;
	BagSubscriber<victor_hardware_interface::Robotiq3FingerStatus_sync> l_sub, r_sub;
    // message_filters::Subscriber<cdcpd_ros::Float32MultiArrayStamped> config_sub(nh, "/kinect2_victor_head/qhd/gripper_config", 10);
	// message_filters::Subscriber<cdcpd_ros::Float32MultiArrayStamped> dot_sub(nh, "/kinect2_victor_head/qhd/dot_config", 10);

    cout << "Making buffer" << endl;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
	
	std::vector<float> cylinder_data(8);
	std::vector<float> quat(4);
	Matrix3Xf init_points(3, 3);
    const double alpha = ROSHelpers::GetParam<double>(ph, "alpha", 0.5);
    const double lambda = ROSHelpers::GetParam<double>(ph, "lambda", 1.0);
    const float zeta = ROSHelpers::GetParam<float>(ph, "zeta", 10.0);
    const double k_spring = ROSHelpers::GetParam<double>(ph, "k", 100.0);
    const double beta = ROSHelpers::GetParam<double>(ph, "beta", 1.0);
	const bool is_pred1 = ROSHelpers::GetParam<bool>(ph, "is_pred1", true);
	const bool is_pred2 = ROSHelpers::GetParam<bool>(ph, "is_pred2", true);
	const bool is_no_pred = ROSHelpers::GetParam<bool>(ph, "is_no_pred", true);
	const bool is_sim = ROSHelpers::GetParam<bool>(ph, "is_sim", true);
	const bool is_rope = ROSHelpers::GetParam<bool>(ph, "is_rope", true);
	const bool is_gripper_info = ROSHelpers::GetParam<bool>(ph, "is_gripper_info", true);	
    const double translation_dir_deformability = ROSHelpers::GetParam<double>(ph, "translation_dir_deformablity", 1.0);
    const double translation_dis_deformability = ROSHelpers::GetParam<double>(ph, "translation_dis_deformablity", 1.0);
    const double rotation_deformability = ROSHelpers::GetParam<double>(ph, "rotation_deformablity", 10.0);
	const bool is_record = ROSHelpers::GetParam<bool>(ph, "is_record", false);
	const int points_on_rope = ROSHelpers::GetParam<int>(ph, "rope_points", 40);
	const int num_width = ROSHelpers::GetParam<int>(ph, "cloth_width", 15);
	const int num_height = ROSHelpers::GetParam<int>(ph, "cloth_height", 15);

	for (int i = 0; i < 8; i++) {
		cylinder_data[i] = ROSHelpers::GetParam<float>(ph, "cylinder_data_"+std::to_string(i), -1.0);
	}

	for (int i = 0; i < 4; i++) {
		quat[i] = ROSHelpers::GetParam<float>(ph, "cylinder_quaternion_"+std::to_string(i), -1.0);
	}
	
	for (int pt = 0; pt < 3; pt++) {
		for (int dim = 0; dim < 3; dim++) {
			init_points(dim, pt) = ROSHelpers::GetParam<float>(ph, "init_pt_"+std::to_string(pt*3+dim), 0.0f);
		}
	}

	std::string workingDir;
	if(is_sim) {
		workingDir = "/home/deformtrack/catkin_ws/src/cdcpd_test_blender/log";
	} else {
		workingDir = "/home/deformtrack/catkin_ws/src/cdcpd_test/log";
	}

	std::string cleancmd = "exec rm -rf " + workingDir + "/*";
	if (system(cleancmd.c_str()) != 0) {
		std::cerr << "wrong clean files" << std::endl;
		exit(1);
	}	
	
    auto [template_vertices, template_edges] = init_template(is_rope, init_points, points_on_rope, num_width, num_height);
    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud = Matrix3Xf2pcptr(template_vertices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud_init = Matrix3Xf2pcptr(template_vertices);
	
	std::vector<std::string> topics;
    if (is_sim) {
    	topics.push_back(std::string("image_color_rect"));
    	topics.push_back(std::string("image_depth_rect"));
    	topics.push_back(std::string("camera_info"));
    	topics.push_back(std::string("groud_truth"));
    	topics.push_back(std::string("gripper_velocity"));
    	topics.push_back(std::string("gripper_info"));
    	topics.push_back(std::string("gripper_config"));
    } else {
		if (is_gripper_info) {
    		topics.push_back(std::string("/kinect2_victor_head/qhd/image_color_rect"));
    		topics.push_back(std::string("/kinect2_victor_head/qhd/image_depth_rect"));
    		topics.push_back(std::string("/kinect2_victor_head/qhd/camera_info"));
    		topics.push_back(std::string("/kinect2_victor_head/qhd/gripper_config"));
    		topics.push_back(std::string("/kinect2_victor_head/qhd/dot_config"));
    		// topics.push_back(std::string("/kinect2_victor_head/qhd/gripper_info"));
			topics.push_back(std::string("/left_arm/gripper_status_repub"));
			topics.push_back(std::string("/right_arm/gripper_status_repub"));
		} else {
			topics.push_back(std::string("/kinect2/qhd/image_color_rect"));
    		topics.push_back(std::string("/kinect2/qhd/image_depth_rect"));
    		topics.push_back(std::string("/kinect2/qhd/camera_info"));
		}
	}
    #ifdef SHAPE_COMP
	topics.push_back(std::string("comp_vertices"));
	topics.push_back(std::string("comp_faces"));
	topics.push_back(std::string("comp_normals"));
    #endif

    auto const bagfile = ROSHelpers::GetParam<std::string>(ph, "bagfile", "normal");
	std::string folder;
    if (is_sim) {
    	folder = ros::package::getPath("cdcpd_ros") + "/../cdcpd_test_blender/dataset/";
    } else {
    	folder = ros::package::getPath("cdcpd_ros") + "/../cdcpd_test/dataset/09_19_2020/";
    }
    rosbag::Bag bag(folder + bagfile + ".bag", rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

	ifstream cpd_phy_test;
  	cpd_phy_test.open(ros::package::getPath("cdcpd_ros") + "/src/cpd_physics/" + bagfile + "/result0.txt", ios::in);
	bool is_cpd_physics = cpd_phy_test.is_open();
	cpd_phy_test.close();

    // Go through the bagfile, storing matched image pairs
	auto sync_use_gripper = message_filters::Synchronizer<SyncPolicy_use_gripper>(SyncPolicy_use_gripper(10), rgb_sub_use_gripper, depth_sub_use_gripper, info_sub_use_gripper, config_sub, dot_sub, l_sub, r_sub);
    sync_use_gripper.registerCallback(boost::bind(&callback_use_gripper, _1, _2, _3, _4, _5, _6, _7));
	auto sync_img = message_filters::Synchronizer<SyncPolicy_img>(SyncPolicy_img(10), rgb_sub, depth_sub, info_sub);
   	sync_img.registerCallback(boost::bind(&callback_im, _1, _2, _3));
	// ros::spin();
    for(rosbag::MessageInstance const& m: view)
    {
        if (m.getTopic() == topics[0])
        {
            auto i = m.instantiate<sm::Image>();
            if (i != nullptr)
            {
				if (is_gripper_info) {
					rgb_sub_use_gripper.newMessage(i);
				} else {
               	 	rgb_sub.newMessage(i);
				}
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
				if (is_gripper_info) {
					depth_sub_use_gripper.newMessage(i);
				} else {
               	 	depth_sub.newMessage(i);
				}
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
				if (is_gripper_info) {
					info_sub_use_gripper.newMessage(info);
				} else {
               	 	info_sub.newMessage(info);
				}
            }
            else
            {
                cout << "NULL initiation!" << endl;
            }
        }
        if (is_sim) {
        	if (m.getTopic() == topics[3]) {
            	auto info = m.instantiate<stdm::Float32MultiArray>();
            	if (info != nullptr) {
               		ground_truth.push_back(info);
                	// truth_sub.newMessage(info);
            	} else {
                	cout << "NULL initiation!" << endl;
            	}
        	}
        	else if (m.getTopic() == topics[4]) {
            	auto info = m.instantiate<stdm::Float32MultiArray>();
            	if (info != nullptr) {
                	grippers_dot_sim.push_back(info);
                	// dot_sub.newMessage(info);
        		} else {
                	cout << "NULL initiation!" << endl;
            	}   
        	}
        	else if (m.getTopic() == topics[5]) {
            	auto info = m.instantiate<stdm::Float32MultiArray>();
            	if (info != nullptr) {
                	grippers_ind.push_back(info);
                	// ind_sub.newMessage(info);
            	} else {
                	cout << "NULL initiation!" << endl;
   				}
        	}
        	else if (m.getTopic() == topics[6]) {
            	auto info = m.instantiate<stdm::Float32MultiArray>();
            	if (info != nullptr) {
                	grippers_config_sim.push_back(info);
                	// config_sub.newMessage(info);
           		} else {
                	cout << "NULL initiation!" << endl;
            	}
        	}
       		#ifdef SHAPE_COMP
			else if (m.getTopic() == topics[7] {
				auto info = m.instantiate<stdm::Float32MultiArray>();
            	if (info != nullptr) {
                	verts_ptr = info;
                	// config_sub.newMessage(info);
            	} else {
                	cout << "NULL initiation!" << endl;
            	}
			}
			else if (m.getTopic() == topics[8]) {
            	auto info = m.instantiate<stdm::Float32MultiArray>();
            	if (info != nullptr) {
                	faces_ptr = info;
                	// config_sub.newMessage(info);
            	} else {
                	cout << "NULL initiation!" << endl;
        		}
        	}
			else if (m.getTopic() == topics[9]) {
            	auto info = m.instantiate<stdm::Float32MultiArray>();
            	if (info != nullptr){
                	normals_ptr = info;
                	// config_sub.newMessage(info);
            	} else {
                	cout << "NULL initiation!" << endl;
            	}
        	}
        	#endif	
		} else {
		// topics in exp bagfile
			if (is_gripper_info) {
				if (m.getTopic() == topics[3]) {
					auto info = m.instantiate<cdcpd_ros::Float32MultiArrayStamped>();
            		if (info != nullptr) {
						config_sub.newMessage(info);
         			} else {
                		cout << "NULL initiation!" << endl;
            		}
				}
				else if (m.getTopic() == topics[4]) {
					auto info = m.instantiate<cdcpd_ros::Float32MultiArrayStamped>();
            		if (info != nullptr) {
                		dot_sub.newMessage(info);
            		} else {
                		cout << "NULL initiation!" << endl;
            		}
				}
				else if (m.getTopic() == topics[5]) {
					auto info = m.instantiate<victor_hardware_interface::Robotiq3FingerStatus>();
            		if (info != nullptr) {
                		l_sub.newMessage(gripper_status_origin_to_sync(info, 0));
            		} else {
                		cout << "NULL initiation!" << endl;
            		}
				}
				else if (m.getTopic() == topics[6]) {
					auto info = m.instantiate<victor_hardware_interface::Robotiq3FingerStatus>();
            		if (info != nullptr) {
                		r_sub.newMessage(gripper_status_origin_to_sync(info, 0));
            		} else {
                		cout << "NULL initiation!" << endl;
            		}
				}
			}
			#ifdef SHAPE_COMP
			if (m.getTopic() == topics[7]) {
				auto info = m.instantiate<stdm::Float32MultiArray>();
            	if (info != nullptr) {
                	verts_ptr = info;
                	// config_sub.newMessage(info);
            	} else {
                	cout << "NULL initiation!" << endl;
            	}
			}
			else if (m.getTopic() == topics[8]) {
            	auto info = m.instantiate<stdm::Float32MultiArray>();
            	if (info != nullptr) {
                	faces_ptr = info;
                	// config_sub.newMessage(info);
            	} else {
                	cout << "NULL initiation!" << endl;
            	}
        	}
			else if (m.getTopic() == topics[9]) {
            	auto info = m.instantiate<stdm::Float32MultiArray>();
            	if (info != nullptr) {
                	normals_ptr = info;
                	// config_sub.newMessage(info);
            	} else {
                	cout << "NULL initiation!" << endl;
           		}
        	}
			#endif
		}
    }
    bag.close();

    cout << "rgb images size: " << color_images.size() << endl;
    cout << "depth images size: " << depth_images.size() << endl;
    cout << "camera infos size: " << camera_infos.size() << endl;
	
	auto color_iter = color_images.cbegin();
    auto depth_iter = depth_images.cbegin();
    auto info_iter = camera_infos.cbegin();
	auto config_sim_iter = grippers_config_sim.cbegin();
    auto velocity_sim_iter = grippers_dot_sim.cbegin();
    auto ind_iter = grippers_ind.cbegin();
    auto truth_iter = ground_truth.cbegin();

	auto config_iter = grippers_config.cbegin();
    auto velocity_iter = grippers_dot.cbegin();
    auto l_iter = l_status.cbegin();
	auto r_iter = r_status.cbegin();

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
	
    std::shared_ptr<ros::NodeHandle> nh_ptr = std::make_shared<ros::NodeHandle>(nh);
	
	AllGrippersSinglePose g_config;
    AllGrippersSinglePoseDelta g_dot;
    MatrixXi g_ind;
	
	CDCPD cdcpd;
	
	if (is_sim) {
    	tie(g_config, g_dot, g_ind) = toGripperConfig(*config_sim_iter, *velocity_sim_iter, *ind_iter);
    	cdcpd = CDCPD(template_cloud,
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
                k_spring,
				zeta,
				cylinder_data,
				is_sim);
	} else {
    	cdcpd = CDCPD(template_cloud,
                template_edges,
                #ifdef SHAPE_COMP
				obstacle_param,
                #endif
                false,
                alpha,
                beta,
                lambda,
                k_spring,
				zeta,
				cylinder_data,
				is_sim);
	}

    #ifdef COMP
	pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud_without_constrain = Matrix3Xf2pcptr(template_vertices);
    auto output_without_constrain_publisher = nh.advertise<PointCloud> ("cdcpd/output_without_constrain", 1);   
	CDCPD cdcpd_without_constrain(template_cloud_without_constrain, template_edges, intrinsics, false, alpha, beta, lambda, k_spring);
    #endif

	CDCPD cdcpd_without_prediction;
    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud_without_prediction = Matrix3Xf2pcptr(template_vertices);
    auto output_without_prediction_publisher = nh.advertise<PointCloud> ("cdcpd/output_without_prediction", 1);
	auto output_without_prediction_order_pub = nh.advertise<vm::Marker> ("cdcpd/output_without_prediction_order", 10);
    if (is_no_pred) {
		if (is_sim) {
			cdcpd_without_prediction = CDCPD(template_cloud_without_prediction,
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
                k_spring,
				zeta,
				cylinder_data,
				is_sim);
		} else {
			cdcpd_without_prediction = CDCPD(template_cloud_without_prediction,
                template_edges,
                #ifdef SHAPE_COMP
                obstacle_param,
                #endif
                false,
                alpha,
                beta,
                lambda,
                k_spring,
				zeta,
				cylinder_data,
				is_sim);
		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud_pred1 = Matrix3Xf2pcptr(template_vertices);
    auto output_pred1_publisher = nh.advertise<PointCloud> ("cdcpd/output_pred_deform_model", 1);
	CDCPD cdcpd_pred1;
	if (is_pred1) {
		if (is_sim) {
			cdcpd_pred1 = CDCPD(template_cloud_pred1,
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
                k_spring,
				zeta,
				cylinder_data,
				is_sim);
		} else {
			cdcpd_pred1 = CDCPD(template_cloud_pred1,
                template_edges,
                #ifdef SHAPE_COMP
                obstacle_param,
                #endif
                false,
                alpha,
                beta,
                lambda,
                k_spring,
				zeta,
				cylinder_data,
				is_sim);
		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud_pred2 = Matrix3Xf2pcptr(template_vertices);
    auto output_pred2_publisher = nh.advertise<PointCloud> ("cdcpd/output_pred_J_model", 1);
	CDCPD cdcpd_pred2;
	if (is_pred2) {
		if (is_sim) {
			cdcpd_pred2 = CDCPD(template_cloud_pred2,
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
                k_spring,
				zeta,
				cylinder_data,
				is_sim);
    	} else {
			cdcpd_pred2 = CDCPD(template_cloud_pred2,
                template_edges,
                #ifdef SHAPE_COMP
                obstacle_param,
                #endif
                false,
                alpha,
                beta,
                lambda,
                k_spring,
				zeta,
				cylinder_data,
				is_sim);
		}
	}
	
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
	
	time_t now = time(0);

   	tm *ltm = localtime(&now);
	string time_str = std::to_string(1+ltm->tm_mon) + "_" +
					  std::to_string(ltm->tm_mday) + "_" + 
					  std::to_string(ltm->tm_hour) + "_" +
					  std::to_string(ltm->tm_min) + "_" +
					  std::to_string(1+ltm->tm_sec);

	cv::VideoWriter video_pred0;
	cv::VideoWriter video_pred1;
	cv::VideoWriter video_pred2;
	cv::VideoWriter video_cdcpd;
	cv::VideoWriter video_cpd_phy;

    auto [color_image_init, depth_image_init, intrinsics_init] = toOpenCv(*color_iter, *depth_iter, *info_iter, is_sim);
    to_file(workingDir + "/trash.txt", depth_image_init);
	cout << intrinsics_init << endl;
	int pad_row = (int) (0.1*color_image_init.rows);
	int pad_col = (int) (0.1*color_image_init.cols);
	mkdir((workingDir+"/../video/ICRA/" + bagfile + "_" + time_str).c_str(), 0777);
	if (is_record) {
		video_pred0.open(workingDir+"/../video/ICRA/" + bagfile + "_" + time_str + "/pred_0.avi",CV_FOURCC('M','J','P','G'), 10, cv::Size(color_image_init.cols+2*pad_col,color_image_init.rows+2*pad_row));
		if (is_pred1) {
			video_pred1.open(workingDir+"/../video/ICRA/" + bagfile + "_" + time_str + "/pred_1.avi",CV_FOURCC('M','J','P','G'), 10, cv::Size(color_image_init.cols+2*pad_col,color_image_init.rows+2*pad_row));
		}
		if (is_pred2) {
			video_pred2.open(workingDir+"/../video/ICRA/" + bagfile + "_" +time_str + "/pred_2.avi",CV_FOURCC('M','J','P','G'), 10, cv::Size(color_image_init.cols+2*pad_col,color_image_init.rows+2*pad_row));
		}
		if (is_no_pred) {
			video_cdcpd.open(workingDir+"/../video/ICRA/" + bagfile + "_" + time_str + "/cdcpd.avi",CV_FOURCC('M','J','P','G'), 10, cv::Size(color_image_init.cols+2*pad_col,color_image_init.rows+2*pad_row));
		}
		if (is_cpd_physics) {	
			video_cpd_phy.open(workingDir+"/../video/ICRA/" + bagfile + "_" + time_str + "/cpd_physics.avi",CV_FOURCC('M','J','P','G'), 10, cv::Size(color_image_init.cols+2*pad_col,color_image_init.rows+2*pad_row));
		}
	}
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
            // rate.sleep();
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

        auto [color_image_bgr, depth_image, intrinsics] = toOpenCv(*color_iter, *depth_iter, *info_iter, is_sim);
        
        Matrix3Xf one_frame_truth;
		if (is_sim) {
			one_frame_truth = toGroundTruth(*truth_iter);
        	tie(g_config, g_dot, g_ind) = toGripperConfig(*config_sim_iter, *velocity_sim_iter, *ind_iter);
		} 
		else if (is_gripper_info) {
    		tie(g_config, g_dot) = toGripperConfig(*config_iter, *velocity_iter);
		}

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

        cv::Mat hsv_mask;
        if (is_rope) {
        	// Red
        	cv::Mat mask1;
        	cv::Mat mask2;
        	cv::inRange(color_hsv, cv::Scalar(0, 0.5, 0.5), cv::Scalar(20, 1.0, 1.0), mask1);
        	cv::inRange(color_hsv, cv::Scalar(340, 0.5, 0.5), cv::Scalar(360, 1.0, 1.0), mask2);
        	bitwise_or(mask1, mask2, hsv_mask);
        } else {
        	// Purple
        	// cv::inRange(color_hsv, cv::Scalar(210, 0.0, 0.4), cv::Scalar(250, 0.5, 0.8), hsv_mask);
        	// cv::inRange(color_hsv, cv::Scalar(50, 0.1, 0.2), cv::Scalar(70, 0.3, 0.6), hsv_mask);
        	cv::Mat mask1;
        	cv::Mat mask2;
        	cv::inRange(color_hsv, cv::Scalar(0, 0.2, 0.2), cv::Scalar(20, 1.0, 1.0), mask1);
        	cv::inRange(color_hsv, cv::Scalar(340, 0.2, 0.2), cv::Scalar(360, 1.0, 1.0), mask2);
        	bitwise_or(mask1, mask2, hsv_mask);
        }

        #ifdef DEBUG
        to_file(workingDir + "/cpp_mask.txt", hsv_mask);
        to_file(workingDir + "/cpp_intrinsics.txt", cv::Mat(intrinsics));
        cv::imwrite(workingDir + "/hsv_mask.png", hsv_mask);
        #endif

        auto frame_id = "kinect2_rgb_optical_frame";
		
		std::vector<bool> is_grasped;
        
		CDCPD::Output out;
		cout << "prediction choice: 0" << endl;
        if (is_sim) {
			cout << "size of g_dot: " << g_dot.size() << endl;
			out = cdcpd(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud, g_dot, g_config, true, true, true, 0, fixed_points); cout << "1683" << endl;
        	std::ofstream(workingDir + "/error.txt", std::ofstream::app) << calc_mean_error(out.gurobi_output->getMatrixXfMap(), one_frame_truth) << " "; cout << "1684" << endl;
        }
		else if (is_gripper_info) {
			is_grasped = toGripperStatus(*l_iter, *r_iter);
			for (auto& dot: g_dot)
			{
				dot = dot/10;
			}
			if (bagfile.compare("rope_winding_cylinder_exp_3") == 0)
			{	
				cout << "hard code the bag file" << endl;
				is_grasped = {false, true};
			}
        	out = cdcpd(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud, g_dot, g_config, is_grasped, nh_ptr, translation_dir_deformability, translation_dis_deformability, rotation_deformability, true, true, true, 0, fixed_points);
        } else {
			out = cdcpd(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud, true, true, true, 0, fixed_points);
        }
        template_cloud = out.gurobi_output;

        #ifdef COMP
        auto out_without_constrain = cdcpd_without_constrain(rgb_image, depth_image, hsv_mask, template_cloud_without_constrain, template_edges, false, false);
        template_cloud_without_constrain = out_without_constrain.gurobi_output;
		out_without_constrain.gurobi_output->header.frame_id = frame_id;
        #endif

		CDCPD::Output out_without_prediction;
        if (is_no_pred) {
			cout << "no prediction used" << endl;
        	out_without_prediction = cdcpd_without_prediction(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud_without_prediction, false, false, false, 0, fixed_points);
			if (is_sim) {
				std::ofstream(workingDir + "/error_no_pred.txt", std::ofstream::app) << calc_mean_error(out_without_prediction.gurobi_output->getMatrixXfMap(), one_frame_truth) << " ";
        	}
			template_cloud_without_prediction = out_without_prediction.gurobi_output;
			out_without_prediction.gurobi_output->header.frame_id = frame_id;
		}

		CDCPD::Output out_pred1;
        if (is_pred1) {
			cout << "prediction choice: 1" << endl;
			if (is_sim) {
        		out_pred1 = cdcpd_pred1(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud_pred1, g_dot, g_config, true, true, true, 1, fixed_points);
				std::ofstream(workingDir + "/error_pred1.txt", std::ofstream::app) << calc_mean_error(out_pred1.gurobi_output->getMatrixXfMap(), one_frame_truth) << " ";
        	} else {
        		out_pred1 = cdcpd_pred1(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud_pred1, g_dot, g_config, is_grasped, nh_ptr, translation_dir_deformability, translation_dis_deformability, rotation_deformability, true, true, true, 1, fixed_points);
			}
			template_cloud_pred1 = out_pred1.gurobi_output;
			out_pred1.gurobi_output->header.frame_id = frame_id;
        }
		
		CDCPD::Output out_pred2;
        if (is_pred2) {
			cout << "prediction choice: 2" << endl;
			if (is_sim) {
        		out_pred2 = cdcpd_pred2(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud_pred2, g_dot, g_config, true, true, true, 2, fixed_points);
				std::ofstream(workingDir + "/error_pred2.txt", std::ofstream::app) << calc_mean_error(out_pred2.gurobi_output->getMatrixXfMap(), one_frame_truth) << " ";
        	} else {
        		out_pred2 = cdcpd_pred2(rgb_image, depth_image, hsv_mask, intrinsics, template_cloud_pred2, g_dot, g_config, is_grasped, nh_ptr, translation_dir_deformability, translation_dis_deformability, rotation_deformability, true, true, true, 2, fixed_points);
			}
			template_cloud_pred2 = out_pred2.gurobi_output;
			out_pred2.gurobi_output->header.frame_id = frame_id;
        }

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
			if (is_sim) {
				std::ofstream(workingDir + "/error_cpd_physics.txt", std::ofstream::app) << calc_mean_error(cpd_phy_pc->getMatrixXfMap().topRows(3).rowwise().reverse(), one_frame_truth) << " ";
			}

    		Matrix2Xi cpd_phy_edges(2, xs.size() - 1);
    		cpd_phy_edges(0, 0) = 0;
    		cpd_phy_edges(1, cpd_phy_edges.cols() - 1) = int(xs.size() - 1);
    		for (int i = 1; i <= cpd_phy_edges.cols() - 1; ++i)
    		{
        		cpd_phy_edges(0, i) = i;
        		cpd_phy_edges(1, i - 1) = i;
    		}
			cv::Mat demo_cpd_phy = draw_vis(rgb_image,
									  	  	cpd_phy_pc->getMatrixXfMap().topRows(3),
									  	  	cpd_phy_edges,
									  	  	intrinsics);
			video_cpd_phy.write(demo_cpd_phy);
		}

		cpd_phy_pc->header.frame_id = frame_id;

        #ifdef ENTIRE
        out.original_cloud->header.frame_id = frame_id;
        #endif

        out.masked_point_cloud->header.frame_id = frame_id;
        out.downsampled_cloud->header.frame_id = frame_id;
        out.cpd_output->header.frame_id = frame_id;
        out.gurobi_output->header.frame_id = frame_id;
		template_cloud_init->header.frame_id = frame_id;
        out.cpd_predict->header.frame_id = frame_id;

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

            cylinder_pub.publish( draw_cylinder_marker(cylinder_data, quat, frame_id) );
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
        pcl_conversions::toPCL(time, template_cloud_init->header.stamp);
        pcl_conversions::toPCL(time, out.cpd_predict->header.stamp);
	
        #ifdef ENTIRE
        original_publisher.publish(out.original_cloud);
        #endif
        masked_publisher.publish(out.masked_point_cloud);
        downsampled_publisher.publish(out.downsampled_cloud);
        template_publisher.publish(out.cpd_output);
        pred_publisher.publish(out.cpd_predict);
        output_publisher.publish(out.gurobi_output);
		
		pcl_conversions::toPCL(time, cpd_phy_pc->header.stamp);	
		cpd_physics_pub.publish(cpd_phy_pc);
		if(cpd_phy_result.is_open())
		{
			vm::Marker order_cpdphysics = pc_to_marker(cpd_phy_pc, template_edges, frame_id);
			order_cpdphysics_pub.publish(order_cpdphysics);
			cpd_phy_result.close();
		}        

        #ifdef COMP
        pcl_conversions::toPCL(time, out_without_constrain.gurobi_output->header.stamp);
        output_without_constrain_publisher.publish(out_without_constrain.gurobi_output);
		#endif
		
        if (is_no_pred) {
        	pcl_conversions::toPCL(time, out_without_prediction.gurobi_output->header.stamp);
			output_without_prediction_publisher.publish(out_without_prediction.gurobi_output);
			vm::Marker order_without_pred = pc_to_marker(out_without_prediction.gurobi_output, template_edges, frame_id);
			output_without_prediction_order_pub.publish(order_without_pred);
       	}

        if (is_pred1) {
        	pcl_conversions::toPCL(time, out_pred1.gurobi_output->header.stamp);
			output_pred1_publisher.publish(out_pred1.gurobi_output);
        }

        if (is_pred2) {
        	pcl_conversions::toPCL(time, out_pred2.gurobi_output->header.stamp);
			output_pred2_publisher.publish(out_pred2.gurobi_output);
        }
		
		if (is_record) {
			cv::Mat demo_im = draw_vis(rgb_image,
									  out.gurobi_output->getMatrixXfMap().topRows(3),
									  template_edges,
									  intrinsics);
			video_pred0.write(demo_im);
			if (is_pred1) {
				cv::Mat demo_pred1 = draw_vis(rgb_image,
									  	  	  out_pred1.gurobi_output->getMatrixXfMap().topRows(3),
									  	  	  template_edges,
									  	  	  intrinsics);
				video_pred1.write(demo_pred1);
			}
			if (is_pred2) {
				cv::Mat demo_pred2 = draw_vis(rgb_image,
									  	  	  out_pred2.gurobi_output->getMatrixXfMap().topRows(3),
									  	  	  template_edges,
									  	  	  intrinsics);
				video_pred2.write(demo_pred2);
			}
			if (is_no_pred) {
				cv::Mat demo_no_pred = draw_vis(rgb_image,
									  	  	  	out_without_prediction.gurobi_output->getMatrixXfMap().topRows(3),
									  	  	  	template_edges,
									  	  	  	intrinsics);
				video_cdcpd.write(demo_no_pred);
				cv::imwrite(workingDir+"/../temp/result"+std::to_string(frame)+".png", demo_no_pred);
			}
		}
        ++color_iter;
        ++depth_iter;
        ++info_iter;
		if (is_sim) {
        	++config_sim_iter;
        	++velocity_sim_iter;
        	++ind_iter;
        	++truth_iter;
		}
		else if (is_gripper_info) {
        	++config_iter;
        	++velocity_iter;
			++l_iter;
			++r_iter;
		}
        ++frame;
    }
	video_pred0.release();
	if (is_pred1) {
		video_pred1.release();
	}
	if (is_pred2) {
		video_pred2.release();
	}
	if (is_no_pred) {
		video_cdcpd.release();
	}
	if (is_cpd_physics) {
		video_cpd_phy.release();
	}

    cout << "Test ended" << endl;

    return EXIT_SUCCESS;
}
