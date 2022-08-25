#pragma once

#include <gtest/gtest.h>

#include "cdcpd/deformable_object_configuration.h"

// Tests that the members of DeformableObjectTracking are properly copied in the copy constructor
// and not just the pointers
TEST(DeformableObjectTrackingTest, copyConstructor)
{
    DeformableObjectTracking* track_1 = new DeformableObjectTracking();
    track_1->vertices = Eigen::Matrix<float, 3, 1>{};
    track_1->vertices << 0.0F, 1.0F, 2.0F;
    track_1->edges = Eigen::Matrix<int, 2, 1>{};
    track_1->edges << 0, 1;
    track_1->points = PointCloud::Ptr(new PointCloud);
    track_1->points->push_back(pcl::PointXYZ{0, 1, 2});

    DeformableObjectTracking* track_2 = new DeformableObjectTracking(*track_1);

    track_1->vertices = Eigen::Matrix<float, 3, 1>{};
    track_1->vertices << 3.0F, 4.0F, 5.0F;
    track_1->edges = Eigen::Matrix<int, 2, 1>{};
    track_1->edges << 2, 3;
    track_1->points = PointCloud::Ptr(new PointCloud);
    track_1->points->push_back(pcl::PointXYZ{3, 4, 5});

    EXPECT_FALSE(track_1->vertices.isApprox(track_2->vertices));

    EXPECT_FALSE(track_1->edges.isApprox(track_2->edges));

    EXPECT_FALSE(track_1->points->points[0].x == track_2->points->points[0].x);
    EXPECT_FALSE(track_1->points->points[0].y == track_2->points->points[0].y);
    EXPECT_FALSE(track_1->points->points[0].z == track_2->points->points[0].z);
}