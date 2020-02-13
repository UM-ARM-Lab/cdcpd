#include "cdcpd/cdcpd.h"
#include <Eigen/Dense>
#include "opencv2/imgcodecs.hpp" // TODO remove after not writing images
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/eigen.hpp"
#include <pcl/filters/voxel_grid.h>

// TODO rm
#include <iostream>

using std::cout;
using std::endl;
// end TODO

// Eigen::Vector3f toPoint3D(u, v, d, intrinsics)
// {
// 
// }

pcl::PointCloud<pcl::PointXYZRGB>::Ptr images_to_cloud(
        const cv::Mat& rgb,
        const cv::Mat& depth,
        const cv::Mat& mask,
        const Eigen::Matrix3f& intrinsics
        )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
 

	for (int v = 0; v < (int)rgb.rows; ++v)
	{
		for (int u = 0; u < (int)rgb.cols; ++u)
		{
            if (mask.at<bool>(v, u)) {
                auto pixel_color = rgb.at<cv::Vec3b>(v, u);
                auto pixel_depth = depth.at<float>(v, u);
                pcl::PointXYZRGB pt(pixel_color[0], pixel_color[1], pixel_color[2]);
                pt.getVector3fMap() = Eigen::Vector3f(
                    (u - intrinsics(0, 2)) * pixel_depth / intrinsics(0, 0),
                    (v - intrinsics(1, 2)) * pixel_depth / intrinsics(1, 1),
                    pixel_depth
                );
                cloud->points.push_back(pt);
                cout << cloud->points.size() << endl;
            }
		}
    }
}

void print_matf(const cv::Mat& m) {
    for (int v = 0; v < m.rows; ++v)
    {
        for (int u = 0; u < m.cols; ++u)
        {
            cout << m.at<float>(v, u) << " ";
        }
        cout << endl;
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cdcpd(
        const cv::Mat& rgb,
        const cv::Mat& depth,
        const cv::Mat& intrinsics)
{
    /// Color filter
    // For the red rope, (h > 0.85) & (s > 0.5). For the flag, (h < 1.0) & (h > 0.9)
    // The flag isn't tested or implemented

    cv::Mat rgb_f;
    rgb.convertTo(rgb_f, CV_32FC3);
    rgb_f /= 255.0; // get RGB 0.0-1.0
    cv::Mat color_hsv;
    cvtColor(rgb_f, color_hsv, CV_RGB2HSV);
    cv::Scalar low_hsv = cv::Scalar(.85 * 360.0, 0.5, 0.0);
    cv::Scalar high_hsv = cv::Scalar(360.0, 1.0, 1.0);
    cv::Mat hsv_mask;
    cv::inRange(color_hsv, low_hsv, high_hsv, hsv_mask);
    cv::imwrite("hsv_mask.png", hsv_mask);

    /// Depth mask
    // Get a mask for all the valid depths
    cv::Mat depth_mask = depth != 0;
    cv::Mat depths_f;
    depth.convertTo(depths_f, CV_32F);
    // Millimeters to meters
    depths_f /= 1000.0;
    cv::imwrite("depth_mask.png", depth_mask);

    cv::Mat combined_mask = hsv_mask & depth_mask;
    cv::imwrite("combined_mask.png", combined_mask);

    Eigen::Matrix3f camera_matrix = Eigen::Matrix3f::Constant(0.0);
    cv::cv2eigen(intrinsics, camera_matrix);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = images_to_cloud(
            rgb,
            depths_f,
            combined_mask,
            camera_matrix
            );



    /// XYZ Filter
    // TODO

    /// Other filter
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);   
	sor.setLeafSize(0.02f, 0.02f, 0.02f);   
	sor.filter(*cloud_filtered);
    return cloud;
}
