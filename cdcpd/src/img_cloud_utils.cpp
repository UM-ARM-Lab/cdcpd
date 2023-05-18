#include "cdcpd/img_cloud_utils.h"

inline static Eigen::Vector3f const bounding_box_extend(0.1, 0.2, 0.1);

// TODO: Refactor this with the getHsvMask function in cdcpd_node.
cv::Mat getHsvMask(cv::Mat const& rgb, double const hue_min, double const sat_min,
    double const val_min, double const hue_max, double const sat_max,
    double const val_max)
{
    cv::Mat rgb_f;
    rgb.convertTo(rgb_f, CV_32FC3);
    rgb_f /= 255.0;  // get RGB 0.0-1.0
    cv::Mat color_hsv;
    cvtColor(rgb_f, color_hsv, CV_RGB2HSV);

    cv::Mat mask1;
    cv::Mat mask2;
    cv::Mat hsv_mask;
    auto hue_min1 = hue_min;
    auto hue_max2 = hue_max;
    if (hue_min > hue_max) {
        hue_max2 = 360;
        hue_min1 = 0;
    }
    cv::inRange(color_hsv, cv::Scalar(hue_min, sat_min, val_min), cv::Scalar(hue_max2, sat_max, val_max), mask1);
    cv::inRange(color_hsv, cv::Scalar(hue_min1, sat_min, val_min), cv::Scalar(hue_max, sat_max, val_max), mask2);
    bitwise_or(mask1, mask2, hsv_mask);

    return hsv_mask;
}

PointCloud::Ptr mat_to_cloud(const Eigen::Matrix3Xf &mat)
{
  PointCloud::Ptr cloud(new PointCloud);
  cloud->points.reserve(mat.cols());
  for (ssize_t i = 0; i < mat.cols(); ++i) {
    cloud->push_back(pcl::PointXYZ(mat(0, i), mat(1, i), mat(2, i)));
  }
  return cloud;
}

// Perform VoxelGrid filter downsampling on an Eigen matrix representing a point cloud.
// NOTE: I was getting ROS errors due to the ROS logging in the downsamplePointCloud routine that
// is a member of the CDCPD class. Due to this, I just replicated the functionality here instead
// of refactoring.
Eigen::Matrix3Xf downsampleMatrixCloud(Eigen::Matrix3Xf mat_in)
{
    PointCloud::Ptr cloud_in = mat_to_cloud(mat_in);

    PointCloud::Ptr cloud_downsampled(new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(0.02f, 0.02f, 0.02f);
    sor.filter(*cloud_downsampled);

    return cloud_downsampled->getMatrixXfMap().topRows(3);
}

// Runs point cloud through a box filter to mitigate segmentation outliers.
Eigen::Matrix3Xf boxFilterMatrixCloud(const Eigen::Matrix3Xf &mat_in,
    const Eigen::Vector3f &lower_bounding_box, const Eigen::Vector3f &upper_bounding_box)
{
    PointCloud::Ptr cloud_in = mat_to_cloud(mat_in);

    const Eigen::Vector4f box_min = (lower_bounding_box - bounding_box_extend).homogeneous();
    const Eigen::Vector4f box_max = (upper_bounding_box + bounding_box_extend).homogeneous();

    PointCloud::Ptr cloud_clipped(new PointCloud);
    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(box_min);
    box_filter.setMax(box_max);
    box_filter.setInputCloud(cloud_in);
    box_filter.filter(*cloud_clipped);

    return cloud_clipped->getMatrixXfMap().topRows(3);
}

// NOTE: based on the implementation here:
// https://github.com/ros-perception/image_pipeline/blob/melodic/depth_image_proc/src/nodelets/point_cloud_xyzrgb.cpp
// we expect that cx, cy, fx, fy are in the appropriate places in P
std::tuple<PointCloudRGB::Ptr, PointCloud::Ptr> point_clouds_from_images(
    const cv::Mat &depth_image, const cv::Mat &rgb_image, const cv::Mat &mask,
    const Eigen::Matrix3f &intrinsics, const Eigen::Vector3f &lower_bounding_box,
    const Eigen::Vector3f &upper_bounding_box)
{
  // depth_image: CV_16U depth image
  // rgb_image: CV_8U3C rgb image
  // mask: CV_8U mask for segmentation
  // intrinsic matrix of Kinect using the Pinhole camera model
  //  [[fx 0  px];
  //   [0  fy py];
  //   [0  0  1 ]]
  // lower_bounding_box_vec: bounding for mask
  // upper_bounding_box_vec: bounding for mask

  float pixel_len;
  cv::Mat local_depth_image;
  depth_image.convertTo(local_depth_image, CV_32F);
  pixel_len = 0.0002645833;

  // Use correct principal point from calibration
  auto const center_x = intrinsics(0, 2);
  auto const center_y = intrinsics(1, 2);

  auto const unit_scaling = 0.001;
  auto const constant_x = 1.0f / (intrinsics(0, 0) * pixel_len);
  auto const constant_y = 1.0f / (intrinsics(1, 1) * pixel_len);
  auto constexpr bad_point = std::numeric_limits<float>::quiet_NaN();

  PointCloud::Ptr filtered_cloud(new PointCloud);
  PointCloudRGB::Ptr unfiltered_cloud(new PointCloudRGB(depth_image.cols, depth_image.rows));
  auto unfiltered_iter = unfiltered_cloud->begin();

  for (int v = 0; v < depth_image.rows; ++v) {
    for (int u = 0; u < depth_image.cols; ++u) {
      float depth = local_depth_image.at<float>(v, u);

      // Assume depth = 0 is the standard was to note invalid
      if (std::isfinite(depth)) {
        float x = (float(u) - center_x) * pixel_len * float(depth) * unit_scaling * constant_x;
        float y = (float(v) - center_y) * pixel_len * float(depth) * unit_scaling * constant_y;
        float z = float(depth) * unit_scaling;
        // Add to unfiltered cloud
        // ENHANCE: be more concise
        unfiltered_iter->x = x;
        unfiltered_iter->y = y;
        unfiltered_iter->z = z;
        unfiltered_iter->r = rgb_image.at<Vec3b>(v, u)[0];
        unfiltered_iter->g = rgb_image.at<Vec3b>(v, u)[1];
        unfiltered_iter->b = rgb_image.at<Vec3b>(v, u)[2];

        Eigen::Array<float, 3, 1> point(x, y, z);
        if (mask.at<bool>(v, u) && point.min(upper_bounding_box.array()).isApprox(point) &&
            point.max(lower_bounding_box.array()).isApprox(point)) {
          filtered_cloud->push_back(pcl::PointXYZ(x, y, z));
        }
      } else {
        unfiltered_iter->x = unfiltered_iter->y = unfiltered_iter->z = bad_point;
      }
      ++unfiltered_iter;
    }
  }

  assert(unfiltered_iter == unfiltered_cloud->end());
  return {unfiltered_cloud, filtered_cloud};
}

std::tuple<Eigen::Matrix3Xf, Eigen::Matrix3Xf> point_cloud_mats_from_images(
    const Eigen::MatrixXi &depth_image,
    const Eigen::MatrixXi &red_channel, const Eigen::MatrixXi &green_channel,
    const Eigen::MatrixXi &blue_channel,
    const Eigen::MatrixXi &mask,
    const Eigen::Matrix3f &intrinsics, const Eigen::Vector3f &lower_bounding_box,
    const Eigen::Vector3f &upper_bounding_box)
{
  cv::Mat depth_image_mat, red_mat, green_mat, blue_mat, rgb_image_mat, mask_mat;
  eigen2cv(depth_image, depth_image_mat);
  eigen2cv(red_channel, red_mat);
  eigen2cv(green_channel, green_mat);
  eigen2cv(blue_channel, blue_mat);
  eigen2cv(mask, mask_mat);

  // Stitch the channels together to a CV matrix.
  std::vector<cv::Mat> channels;
  channels.push_back(blue_mat);
  channels.push_back(green_mat);
  channels.push_back(red_mat);
  cv::merge(channels, rgb_image_mat);

  auto [unfiltered_cloud, filtered_cloud] = point_clouds_from_images(depth_image_mat, rgb_image_mat,
    mask_mat, intrinsics, lower_bounding_box, upper_bounding_box);

  // std::cout << "Unfiltered cloud height: " << unfiltered_cloud->height << std::endl;
  // std::cout << "Unfiltered cloud width: " << unfiltered_cloud->width << std::endl;

  // Convert clouds to Eigen matrices.
  Eigen::Matrix3f unfiltered_cloud_mat = unfiltered_cloud->getMatrixXfMap();//.topRows(3);
  Eigen::Matrix3f filtered_cloud_mat = filtered_cloud->getMatrixXfMap().topRows(3);

  return {unfiltered_cloud_mat, filtered_cloud_mat};
}

