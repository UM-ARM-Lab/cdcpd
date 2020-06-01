#include <random>
#include <iostream> // TODO rm
#include <vector>
#include <algorithm>

#include "cdcpd/past_template_matcher.h"
#include <faiss/IndexFlat.h>
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

/*
 * find k nearest neighbors of filtered points
 */
std::vector<Eigen::Matrix3Xf> PastTemplateMatcher::query_template(pcl::PointCloud<pcl::PointXYZ>::ConstPtr filtered_points, int k)
{
    cv::Mat feature = downsampled_feature(filtered_points);
    // faiss version
    int d = 308;                           // dimension
    int nb = recovery_features.size();     // database size
    int nq = 1;                            // number of queries
    float *xb = new float[d * nb];
    float *xq = new float[d * nq];

    // convert feature dataset and feature to float*
    for (int i = 0; i < nb; ++i) {
        for (int j = 0; j < d; ++j) {
            xb[i * d + j] = recovery_features[i].at<float>(0, j);
        }
    }

    for (int l = 0; l < d; ++l) {
        xq[l] = feature.at<float>(0, l);
    }

    faiss::IndexFlatL2 index(d);           // call constructor
    index.add(nb, xb);                     // add vectors to the index

    long *I = new long[(k+1) * nq];
    float *D = new float[(k+1) * nq];

    std::cout << "is trained: " << index.is_trained << "\n";
    index.search(nq, xq, k+1, D, I);

    // convert back to std::vector<Eigen::Matrix3Xf>
    std::vector<Eigen::Matrix3Xf> output;
    /*
    std::cout << "created output vector\n";
    std::cout << "total templates " << recovery_templates.size() << "\n";
    std::cout << "search vector:\n";
    for (int n = 0; n < d; ++n) {
        std::cout << xq[n] << " ";
    }
    std::cout << "\n";
    std::cout << "last one of the search base:\n";
    for (int i1 = 0; i1 < d; ++i1) {
        std::cout << xb[(nb-1)*d+i1] << " ";
    }
    std::cout << "\n";
    std::cout << "search indexing result\n";
    for (int m = 0; m < k+1; ++m) {
        std::cout << I[m] << " ";
    }
    std::cout << "\n";
     */
    for (int i = 1; i <= k; i++) {
        if (I[i] != -1) {
            Eigen::Matrix3Xf newOutput(recovery_templates[I[i]]);
            output.push_back(newOutput);
        }
    }

    delete[] xb;
    delete[] xq;
    delete[] I;
    delete[] D;

    return output;

    /*
    // filtered_points: X^t in the paper
    // k: k nearest neighbors
    cv::Mat feature = downsampled_feature(filtered_points);
    std::vector<std::vector<cv::DMatch>> matches;
    std::cout << "feature: " << feature.rows << " times " << feature.cols << std::endl;
    std::cout << "recovery feature: size: " << recovery_features.size() << " and " << feature.rows << " times " << feature.cols << std::endl;
    matcher.knnMatch(feature, recovery_features, matches, std::min(k, int(recovery_features.size())));
    // std::cout << "TEST1" << matches.size() << std::endl;
    // std::cout << "TEST2" << matches[0].size() << std::endl;
    std::vector<Eigen::Matrix3Xf> output;
    assert(matches.size() == 1);
    for ( const cv::DMatch& match : matches[0] )
    {
        output.push_back(recovery_templates[match.trainIdx]);
    }
    return output;
     */
}

void PastTemplateMatcher::add_template(pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_points, Eigen::Matrix3Xf tracking_result)
{
    recovery_templates.push_back(tracking_result);
    recovery_features.push_back(downsampled_feature(filtered_points));
}

/*
 * Down sample filtered_points (X^t) to sample size
 */
cv::Mat PastTemplateMatcher::downsampled_feature(pcl::PointCloud<pcl::PointXYZ>::ConstPtr filtered_points)
{
    // filtered_points: X^t in the paper
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

/*
 * generate vfh from X^t
 * NOTE: didn't check in details and assumed it to be true
 */
cv::Mat PastTemplateMatcher::vfh(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input)
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
    cv::Mat feature(1, 308, CV_32F, vfhs->points[0].histogram);
    return feature;
}
