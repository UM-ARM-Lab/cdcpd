#pragma once

#include <gtest/gtest.h>

#include "cdcpd/deformable_object_configuration.h"

class DeformableObjectTrackingTest : public ::testing::Test
{
protected:
    DeformableObjectTrackingTest()
    {
        track_1 = std::make_shared<DeformableObjectTracking>();
        Eigen::Matrix3Xf vertices_1 = Eigen::Matrix<float, 3, 1>{};
        vertices_1 << 0.0F, 1.0F, 2.0F;
        Eigen::Matrix2Xi edges_1 = Eigen::Matrix<int, 2, 1>{};
        edges_1 << 0, 1;

        track_1->setVertices(vertices_1);
        track_1->setEdges(edges_1);

        // Copy track 1 into track 2.
        track_2 = std::make_shared<DeformableObjectTracking>(*track_1);

        // Modify track 1.
        Eigen::Matrix3Xf vertices_2 = Eigen::Matrix<float, 3, 1>{};
        vertices_2 << 3.0F, 4.0F, 5.0F;
        Eigen::Matrix2Xi edges_2 = Eigen::Matrix<int, 2, 1>{};
        edges_2 << 2, 3;

        track_1->setVertices(vertices_2);
        track_1->setEdges(edges_2);

    }

    std::shared_ptr<DeformableObjectTracking> track_1;
    std::shared_ptr<DeformableObjectTracking> track_2;
};

// Tests that the members of DeformableObjectTracking are properly copied in the copy constructor
// and not just the pointers.
// NOTE: This also tests getVertices, getEdges, and getPointCloud methods.
TEST_F(DeformableObjectTrackingTest, copyConstructor)
{
    // Check that modifications to track 1 didn't carry over to track 2.
    auto const& track_1_vertices = track_1->getVertices();
    auto const& track_1_edges = track_1->getEdges();
    auto const track_1_point_cloud = track_1->getPointCloud();
    auto const track_2_point_cloud = track_2->getPointCloud();

    EXPECT_FALSE(track_1_vertices.isApprox(track_2->getVertices()));

    EXPECT_FALSE(track_1_edges.isApprox(track_2->getEdges()));

    EXPECT_FALSE(track_1_point_cloud->points[0].x == track_2_point_cloud->points[0].x);
    EXPECT_FALSE(track_1_point_cloud->points[0].y == track_2_point_cloud->points[0].y);
    EXPECT_FALSE(track_1_point_cloud->points[0].z == track_2_point_cloud->points[0].z);
}