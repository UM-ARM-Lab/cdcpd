#pragma once

#include <vector>
#include <string>

#include <opencv2/imgproc.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types_conversion.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>

#include "cdcpd/deformable_object_configuration.h"

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;

// CornerCandidateDetection for point clouds.
// class CornerCandidateDetection
// {
// public:
//     CornerCandidateDetection(int const detection_id,
//         Eigen::Vector3f const corner_point_camera_frame,
//         cv::Mat const template_affine_transform,
//         std::vector<Eigen::Vector3f> const local_cloud_neighborhood_bounds,
//         std::vector<Eigen::Vector3f> const detection_mask_bounds);

//     // Do a bounding box filter to get the local neighborhood.
//     boost::shared_ptr<PointCloudRGB> get_local_point_cloud_neighborhood(boost::shared_ptr<PointCloudRGB> point_cloud_full) const;

//     // Do a bounding box filter to get the masked points.
//     boost::shared_ptr<PointCloud> get_masked_points(boost::shared_ptr<PointCloudRGB> point_cloud_full) const;

//     // Prints the corner candidate to ROS_INFO
//     void print() const;

//     // The ID associated with this corner cadidate.
//     int const id_;

//     Eigen::Vector3f const corner_point_camera_frame_;

//     // The affine transform to go from template initial position (TBD) (0, 0, 0)? to the position in
//     // the camera frame that would align the template with the corner candidate.
//     // The purpose of this is to provide a solid initialization for the tracked template instead of
//     // hoping the tracking converges from some naiive initialized position.
//     cv::Mat const template_affine_transform_;

//     // The bounds of the local neighborhood of the detected corner candidate. Used to pass in
//     // partial point clouds to the CDCPD::operator()
//     // We'll use an expanded bounding box filter on to grab the appropriate points in the point
//     // cloud.
//     std::vector<Eigen::Vector3f> const local_cloud_neighborhood_bounds_;

//     // The bounds of the detection mask that we'll use an expanded bounding box filter on to grab
//     // the appropriate points in the point cloud.
//     std::vector<Eigen::Vector3f> const detection_mask_bounds_;

// protected:
//     void print_vector(std::vector<Eigen::Vector3f> vec_of_vecs) const;
// };

// CornerCandidateDetection for RGBD
class CornerCandidateDetection
{
public:
    CornerCandidateDetection(int const detection_id,
        Eigen::Vector3f const corner_point_camera_frame, cv::Mat const template_affine_transform,
        std::vector<std::vector<cv::Point>> mask_contour);

    // Draws the mask onto an image like the one provided.
    cv::Mat get_mask(cv::Mat const& depth_img) const;

    // Prints the corner candidate to ROS_INFO
    void print() const;

    // The ID associated with this corner cadidate.
    int const id_;

    // The affine transform to go from template initial position (TBD) (0, 0, 0)? to the position in
    // the camera frame that would align the template with the corner candidate.
    // The purpose of this is to provide a solid initialization for the tracked template instead of
    // hoping the tracking converges from some naiive initialized position.
    cv::Mat const template_affine_transform_;

    std::vector<std::vector<cv::Point>> mask_contour_;

    Eigen::Vector3f const corner_point_camera_frame_;

protected:
    template<typename T>
    void print_vector(std::vector<T> vec) const;
};


class CornerCandidateDetector
{
public:
    CornerCandidateDetector();

    // Executes the corner candidate detection routine Zixuan prototyped
    std::vector<CornerCandidateDetection> do_corner_candidate_detection(
        const PointCloudRGB::Ptr &points_full_cloud);

    // Executes the corner candidate detection routine Zixuan prototyped
    std::vector<CornerCandidateDetection> do_corner_candidate_detection(
        cv::Mat const& rgb_img, cv::Mat const& depth_img);

    // Returns a vector of tuples of 2 ints, <cluster index, deformable object index>
    // -1 for either of the indexes indicates no association was found for the cluster/tracked
    // object.
    std::vector<std::tuple<int const, int const>> associate_corner_candidates_with_tracked_objects(
        std::vector<CornerCandidateDetection> const& corner_candidate_detections,
        std::map<int const, std::shared_ptr<DeformableObjectConfiguration>> deformable_object_configurations);


protected:
    void run();

    void write_point_cloud(const PointCloudRGB::Ptr &point_cloud);
    void write_cv_mat(std::string const& output_path, cv::Mat const& img);

    // Call the corner candidate detection script/routine.
    std::vector<CornerCandidateDetection> read_detector_output();

    std::string root_dir_ = "/home/dcolli23/cdcpd_corner_detector_runtime_data/";
    std::string full_cloud_output_path_ = root_dir_ + "points_full_cloud.pcd";
    std::string rgb_output_path_ = root_dir_ + "rgb_img.png";
    std::string depth_output_path_ = root_dir_ + "depth_img.png";
    std::string yaml_output_path_ = root_dir_ + "corner_candidate_detections.yaml";
};