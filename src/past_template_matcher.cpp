#include <random>
#include <iostream> // TODO rm
#include <vector>

#include "cdcpd/past_template_matcher.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>

PastTemplateMatcher::PastTemplateMatcher(int _sample_size) :
    matcher(cv::makePtr<cv::flann::KDTreeIndexParams>(5), cv::makePtr<cv::flann::SearchParams>(50)),
    gen(rd()),
    index_generator(0, _sample_size - 1), 
    sample_size(_sample_size) {}

size_t PastTemplateMatcher::size()
{
    assert(recovery_templates.size() == recovery_templates.size());
    return recovery_templates.size();
}

std::vector<Eigen::Matrix3Xf> PastTemplateMatcher::query_template(pcl::PointCloud<pcl::PointXYZ>::ConstPtr filtered_points, int k)
{
    cv::Vec<float, 308> feature = downsampled_feature(filtered_points);
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(feature, recovery_features, matches, k);
    std::cout << "TEST1" << matches.size() << std::endl;
    std::cout << "TEST2" << matches[0].size() << std::endl;
    std::vector<Eigen::Matrix3Xf> output;
    assert(matches.size() == 1);
    for ( const cv::DMatch& match : matches[0] )
    {
        output.push_back(recovery_templates[match.trainIdx]);
    }
    return output;
}

void PastTemplateMatcher::add_template(pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points, Eigen::Matrix3Xf tracking_result)
{
    recovery_templates.push_back(tracking_result);
    recovery_features.push_back(downsampled_feature(filtered_points));
}

/*
 * Down sample filtered_points (X^t) to sample size
 */
cv::Vec<float, 308> PastTemplateMatcher::downsampled_feature(pcl::PointCloud<pcl::PointXYZ>::ConstPtr filtered_points)
{
    // filtered_points: 
    if (filtered_points->size() > sample_size)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>(sample_size, 1));
        std::generate(
                downsampled_cloud->begin(),
                downsampled_cloud->end(),
                [this, &filtered_points]() { return (*filtered_points)[this->index_generator(gen)]; });
        return vfh(downsampled_cloud);
    }
    else
    {
        return vfh(filtered_points);
    }
}

cv::Vec<float, 308> PastTemplateMatcher::vfh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (input);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*normals);

    // Create the VFH estimation class, and pass the input dataset+normals to it
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (input);
    vfh.setInputNormals (normals);
    // alternatively, if input is of type PointNormal, do vfh.setInputNormals (input);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr vfh_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    vfh.setSearchMethod (vfh_tree);

    // Output datasets
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

    // Compute the features
    vfh.compute(*vfhs);
    cv::Vec<float, 308> feature(vfhs->points[0].histogram);
    return feature;
}
