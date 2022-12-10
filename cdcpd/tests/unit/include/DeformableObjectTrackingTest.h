#pragma once

#include <gtest/gtest.h>

#include "cdcpd/deformable_object_configuration.h"

#include <iostream>

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

    bool areConnectivityNodesEqual(ConnectivityNode const& node_1, ConnectivityNode const& node_2)
    {
        if (node_1.get_id() != node_2.get_id())
        {
            return false;
        }

        // Check if the nodes have the same neighbors.
        for (auto const& neighbor_pair : node_1.get_neighbors())
        {
            int const neighbor_id = neighbor_pair.first;
            if (!node_2.is_neighbor_node(neighbor_id))
            {
                return false;
            }
        }
        for (auto const& neighbor_pair : node_2.get_neighbors())
        {
            int const neighbor_id = neighbor_pair.first;
            if (!node_1.is_neighbor_node(neighbor_id))
            {
                return false;
            }
        }

        return true;
    }

    bool doGraphNodeMapsMatch(ConnectivityGraph const& graph_1, ConnectivityGraph const& graph_2)
    {
        // Check if all nodes in graph 1 are in graph 2
        for (auto const& pair_1 : graph_1.get_node_map())
        {
            int const id_node_1 = pair_1.first;
            auto const node_1 = pair_1.second;

            int const num_matching_nodes = graph_2.get_node_map().count(id_node_1);
            if (num_matching_nodes > 1)
            {
                std::cout << "Found " << num_matching_nodes
                    << " matching nodes between graphs. This is very unexpected!" << std::endl;
            }

            bool const is_node_1_in_graph_2 = num_matching_nodes == 1;
            if (!is_node_1_in_graph_2)
            {
                return false;
            }
        }
        return true;
    }

    bool areConnectivityGraphsEqual(ConnectivityGraph const& graph_1,
        ConnectivityGraph const& graph_2)
    {
        bool graphs_equal = true;

        // Check that all nodes in graph 1 are in graph 2 and vice versa.
        graphs_equal = graphs_equal && doGraphNodeMapsMatch(graph_1, graph_2);
        graphs_equal = graphs_equal && doGraphNodeMapsMatch(graph_2, graph_1);
        if (!graphs_equal)
        {
            return false;
        }

        NodeMap const& node_map_2 = graph_2.get_node_map();
        for (auto const& pair_1 : graph_1.get_node_map())
        {
            int const id_node_1 = pair_1.first;
            auto const node_1 = pair_1.second;

            // First check if the node from graph 1 is even in graph 2
            auto it_potential_node_2 = node_map_2.find(id_node_1);
            bool const is_node_1_in_graph_2 = it_potential_node_2 != node_map_2.end();
            if (is_node_1_in_graph_2)
            {
                auto const node_2 = it_potential_node_2->second;
                bool const are_nodes_equal = areConnectivityNodesEqual(*node_1, *node_2);
                graphs_equal = graphs_equal && are_nodes_equal;
            }
            else  // If the node from graph 1 isn't in graph 2, we know the graphs aren't equal.
            {
                return false;
            }
        }

        // If we got to this point, everything is equal.
        return true;
    }

    std::shared_ptr<DeformableObjectTracking> track_1;
    std::shared_ptr<DeformableObjectTracking> track_2;
};

// Tests that the members of DeformableObjectTracking are properly copied in the copy constructor
// and not just the pointers.
// NOTE: This also tests getVertices, getEdges, and getPointCloud methods.
TEST_F(DeformableObjectTrackingTest, copyConstructorDifferentTracks)
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

    bool are_graphs_equal = areConnectivityGraphsEqual(track_1->getConnectivityGraph(),
        track_2->getConnectivityGraph());
    EXPECT_FALSE(are_graphs_equal);

    are_graphs_equal = areConnectivityGraphsEqual(track_2->getConnectivityGraph(),
        track_1->getConnectivityGraph());
    EXPECT_FALSE(are_graphs_equal);
}

TEST_F(DeformableObjectTrackingTest, copyConstructorSameTracks)
{
    // Copy track 1 back into track 2 to make sure everything is the same.
    track_2 = std::make_shared<DeformableObjectTracking>(*track_1);
    auto const& track_1_vertices = track_1->getVertices();
    auto const& track_1_edges = track_1->getEdges();
    auto const track_1_point_cloud = track_1->getPointCloud();
    auto const track_2_point_cloud = track_2->getPointCloud();

    EXPECT_TRUE(track_1_vertices.isApprox(track_2->getVertices()));

    EXPECT_TRUE(track_1_edges.isApprox(track_2->getEdges()));

    EXPECT_TRUE(track_1_point_cloud->points[0].x == track_2_point_cloud->points[0].x);
    EXPECT_TRUE(track_1_point_cloud->points[0].y == track_2_point_cloud->points[0].y);
    EXPECT_TRUE(track_1_point_cloud->points[0].z == track_2_point_cloud->points[0].z);

    bool are_graphs_equal = areConnectivityGraphsEqual(track_1->getConnectivityGraph(),
        track_2->getConnectivityGraph());
    EXPECT_TRUE(are_graphs_equal);

    are_graphs_equal = areConnectivityGraphsEqual(track_2->getConnectivityGraph(),
        track_1->getConnectivityGraph());
    EXPECT_TRUE(are_graphs_equal);
}