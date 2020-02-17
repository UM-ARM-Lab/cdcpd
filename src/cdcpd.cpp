#include "cdcpd/cdcpd.h"
#include <cassert>
#include <Eigen/Dense>
#include "opencv2/imgcodecs.hpp" // TODO remove after not writing images
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/eigen.hpp"
#include <opencv2/rgbd.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

// TODO rm
#include <iostream>
// end TODO

using std::cout;
using std::endl;

// Eigen::Vector3f toPoint3D(u, v, d, intrinsics)
// {
// 
// }

pcl::PointCloud<pcl::PointXYZ>::Ptr images_to_cloud(
        const cv::Mat& rgb,
        const cv::Mat& depth,
        const cv::Mat& mask,
        const Eigen::Matrix3f& intrinsics
        )
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
 

    for (int v = 0; v < (int)rgb.rows; ++v)
    {
        for (int u = 0; u < (int)rgb.cols; ++u)
        {
            if (mask.at<uint8_t>(v, u) != 0 && depth.at<uint16_t>(v, u) != 0) {
                assert(depth.at<uint16_t>(v, u) != 0);
                auto pixel_color = rgb.at<cv::Vec3b>(v, u);
                auto pixel_depth = depth.at<uint16_t>(v, u) / 1000.0f;
                // cout << "One point: " << endl;
                // cout << u << " " << v << " " << pixel_depth << " " 
                //     << intrinsics(0, 0) << " "
                //     << intrinsics(0, 1) << " "
                //     << intrinsics(0, 2) << " "
                //     << intrinsics(1, 0) << " "
                //     << intrinsics(1, 1) << " "
                //     << intrinsics(1, 2) << " "
                //     << intrinsics(2, 0) << " "
                //     << intrinsics(2, 1) << " "
                //     << intrinsics(2, 2) << endl;
                pcl::PointXYZ pt(pixel_color[0], pixel_color[1], pixel_color[2]);
                pt.getVector3fMap() = Eigen::Vector3f(
                    (u - intrinsics(0, 2)) * pixel_depth / intrinsics(0, 0),
                    (v - intrinsics(1, 2)) * pixel_depth / intrinsics(1, 1),
                    pixel_depth
                );
                cloud->points.push_back(pt);
            }
        }
    }
    return cloud;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr cdcpd(
        const cv::Mat& rgb,
        const cv::Mat& depth,
        const cv::Mat& mask,
        const cv::Mat& intrinsics)
{
    assert(rgb.type() == CV_8UC3);
    assert(depth.type() == CV_16U);
    assert(mask.type() == CV_8U);
    assert(intrinsics.type() == CV_64F);
    assert(intrinsics.rows == 3 && intrinsics.cols == 3);
    assert(rgb.rows == depth.rows && rgb.cols == depth.cols);

    /// Depth mask
    // Get a mask for all the valid depths
    cv::Mat depth_mask = depth != 0;
    // cv::Mat depths_f;
    // depth.convertTo(depths_f, CV_32F);
    // // cout << "Pre-conversion depths" << endl;
    // // print_matf(depths_f);
    // // Millimeters to meters
    // depths_f /= 1000.0;
    // cv::imwrite("depth_mask.png", depth_mask);

    cv::Mat combined_mask = mask & depth_mask;
    cv::imwrite("combined_mask.png", combined_mask);

    // cout << "Depth of rope: " << endl;
    // cout << depth.at<uint16_t>(175, 150) << endl;

    // Eigen::Matrix3f camera_matrix = Eigen::Matrix3f::Constant(0.0);
    // cv::cv2eigen(intrinsics, camera_matrix);
    
    cv::Mat points_mat;
    cv::rgbd::depthTo3d(depth, intrinsics, points_mat, combined_mask);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < points_mat.cols; ++i)
    {
        auto cv_point = points_mat.at<cv::Vec3d>(0, i);
        pcl::PointXYZ pt(cv_point(0), cv_point(1), cv_point(2));
        cloud->push_back(pt);
    }

    cout << "All right, we have the cloud." << endl;
    cout << "It's of type " << points_mat.type() << endl;
    cout << "It has " << points_mat.rows << " rows and " << points_mat.cols << " cols." << endl;

    // This should be all we need?
    // TODO now we need to convert the OpenCV point cloud to a PCL one.

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = images_to_cloud(
    //         rgb,
    //         depth,
    //         combined_mask,
    //         camera_matrix
    //         );


    // cout << "Cloud: " << endl;
    // for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it
    // loud->end(); it++){
    //     cout << it->x << ", " << it->y << ", " << it->z << endl;
    // }
    
    // TODO nooooooooooooo
    return cloud;
    
    
    /// box Filter
    // TODO what is last element in vector?
    // TODO this needs to be a class so these can be members.
    // TODO also, we need to find the max and min and store them for next time
    // TODO also, we need to add .1m to each one.
    Eigen::Vector4f last_lower_bounding_box(-5.0, -5.0, -5.0, 1.0);
    Eigen::Vector4f last_upper_bounding_box(5.0, 5.0, 5.0, 1.0);
    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(last_lower_bounding_box);
    box_filter.setMax(last_upper_bounding_box);
    box_filter.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    box_filter.filter(*cloud_box_filtered);
    // TODO finish up

    /// Other filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fully_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);   
    sor.setLeafSize(0.02f, 0.02f, 0.02f);   
    sor.filter(*cloud_fully_filtered);

    /// CPD step


    return cloud;
}
