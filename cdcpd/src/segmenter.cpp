#include "cdcpd/segmenter.h"

Segmenter::Segmenter()
    : segmented_points_{new PointCloud()}
{}

PointCloud::Ptr Segmenter::get_segmented_cloud()
{
    return PointCloud::Ptr(new PointCloud(*segmented_points_));
}

SegmenterHSV::SegmenterParameters::SegmenterParameters()
    : hue_min(340.0)
    , hue_max(20.0)
    , sat_min(0.3)
    , sat_max(1.0)
    , val_min(0.4)
    , val_max(1.0)
{}

SegmenterHSV::SegmenterHSV(Eigen::Vector3f const last_lower_bounding_box,
        Eigen::Vector3f const last_upper_bounding_box)
    : Segmenter()
    , last_lower_bounding_box_{last_lower_bounding_box}
    , last_upper_bounding_box_{last_upper_bounding_box}
{}

void SegmenterHSV::segment()
{
    auto points_cropped = boost::make_shared<PointCloudRGB>();
    auto points_hsv = boost::make_shared<PointCloudHSV>();
    auto filtered_points_h = boost::make_shared<PointCloudHSV>();
    auto filtered_points_hs = boost::make_shared<PointCloudHSV>();
    auto filtered_points_hsv = boost::make_shared<PointCloudHSV>();

    const Eigen::Vector4f box_min = (last_lower_bounding_box_ - bounding_box_extend).homogeneous();
    const Eigen::Vector4f box_max = (last_upper_bounding_box_ + bounding_box_extend).homogeneous();
    ROS_DEBUG_STREAM_NAMED(LOGNAME_SEGMENTER + ".points", "box min: " << box_min.head(3)
        << " box max " << box_max.head(3));

    // BBOX filter
    pcl::CropBox<PointRGB> box_filter;
    box_filter.setMin(box_min);
    box_filter.setMax(box_max);
    box_filter.setInputCloud(input_cloud_);
    box_filter.filter(*points_cropped);

    // convert to HSV
    pcl::PointCloudXYZRGBtoXYZHSV(*points_cropped, *points_hsv);

    // HSV Filter
    auto const hue_negative = params_.hue_min > params_.hue_max;
    auto const sat_negative = params_.sat_min > params_.sat_max;
    auto const val_negative = params_.val_min > params_.val_max;

    pcl::PassThrough<PointHSV> hue_filter;
    hue_filter.setInputCloud(points_hsv);
    hue_filter.setFilterFieldName("h");
    hue_filter.setFilterLimits(std::min(params_.hue_min, params_.hue_max),
        std::max(params_.hue_min, params_.hue_max));
    hue_filter.setFilterLimitsNegative(hue_negative);
    hue_filter.filter(*filtered_points_h);
    ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME_SEGMENTER + ".points",
        "hue filtered " << filtered_points_h->size());

    pcl::PassThrough<PointHSV> sat_filter;
    sat_filter.setInputCloud(filtered_points_h);
    sat_filter.setFilterFieldName("s");
    sat_filter.setFilterLimits(std::min(params_.sat_min, params_.sat_max),
        std::max(params_.sat_min, params_.sat_max));
    sat_filter.setFilterLimitsNegative(sat_negative);
    sat_filter.filter(*filtered_points_hs);
    ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME_SEGMENTER + ".points",
        "sat filtered " << filtered_points_hs->size());

    pcl::PassThrough<PointHSV> val_filter;
    val_filter.setInputCloud(filtered_points_hs);
    val_filter.setFilterFieldName("v");
    val_filter.setFilterLimits(std::min(params_.val_min, params_.val_max),
        std::max(params_.val_min, params_.val_max));
    val_filter.setFilterLimitsNegative(val_negative);
    val_filter.filter(*filtered_points_hsv);
    ROS_DEBUG_STREAM_THROTTLE_NAMED(1, LOGNAME_SEGMENTER + ".points",
        "val filtered " << filtered_points_hsv->size());

    // Drop color info from the segmented points and store in member variable.
    pcl::copyPointCloud(*filtered_points_hsv, *segmented_points_);
}

void SegmenterHSV::set_last_lower_bounding_box(Eigen::Vector3f const& last_lower_bounding_box)
{
    last_lower_bounding_box_ = last_lower_bounding_box;
}

void SegmenterHSV::set_last_upper_bounding_box(Eigen::Vector3f const& last_upper_bounding_box)
{
    last_upper_bounding_box_ = last_upper_bounding_box;
}