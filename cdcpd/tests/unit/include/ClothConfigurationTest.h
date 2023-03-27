#pragma once

#include <gtest/gtest.h>

#include "cdcpd/deformable_object_configuration.h"
#include "test_utils.h"

// Tests the initialization of a very simple template, a 2x2 cloth
TEST(ClothConfigurationTest, simpleClothConfigInitializationBagGridSizeGuess)
{
    // The following values will give a 2x2 template
    float const cloth_length = 1.0;
    float const cloth_width = 1.0;
    float const grid_size_initial_guess = 0.76;
    ClothConfiguration cloth_config{cloth_length, cloth_width, grid_size_initial_guess};

    EXPECT_FLOAT_EQ(cloth_config.length_initial_, cloth_length);
    EXPECT_FLOAT_EQ(cloth_config.width_initial_, cloth_width);
    EXPECT_FLOAT_EQ(cloth_config.grid_size_initial_guess_, grid_size_initial_guess);
    EXPECT_FLOAT_EQ(cloth_config.max_segment_length_, 0.5F);
    EXPECT_EQ(cloth_config.num_points_length_, 3);
    EXPECT_EQ(cloth_config.num_points_width_, 3);
    EXPECT_EQ(cloth_config.num_points_, 9);
    EXPECT_EQ(cloth_config.num_edges_, 12);
}

// Tests the tracking initialization of a very simple template, a 3x3 cloth
TEST(ClothConfigurationTest, simpleClothConfigurationTemplateInitializationBadGridSizeGuess)
{
    // The following values give a 3x3 template
    float const cloth_length = 1.0;
    float const cloth_width = 1.0;
    float const grid_size_initial_guess = 0.76;
    ClothConfiguration config_test{cloth_length, cloth_width, grid_size_initial_guess};
    config_test.initializeTracking();

    // Setup the expected points and vertices
    std::vector<cv::Point3f> verts_expected = {
        {0, 0, 0},
        {0, 0.5, 0},
        {0, 1, 0},
        {0.5, 0, 0},
        {0.5, 0.5, 0},
        {0.5, 1, 0},
        {1, 0, 0},
        {1, 0.5, 0},
        {1, 1, 0}
    };
    Eigen::Matrix<float, 3, 9> verts_expected_mat;
    int i = 0;
    for (auto const& vert : verts_expected)
    {
        verts_expected_mat(0, i) = vert.x;
        verts_expected_mat(1, i) = vert.y;
        verts_expected_mat(2, i) = vert.z;
        ++i;
    }

    int const num_edges_expected = 12;
    std::vector<std::tuple<int, int>> edges_expected = {
        {0, 3},
        {0, 1},
        {1, 4},
        {1, 2},
        {2, 5},
        {3, 6},
        {3, 4},
        {4, 7},
        {4, 5},
        {5, 8},
        {6, 7},
        {7, 8}
    };
    Eigen::Matrix<int, 2, num_edges_expected> edges_expected_mat;
    i = 0;
    for (auto const& edge : edges_expected)
    {
        edges_expected_mat(0, i) = std::get<0>(edge);
        edges_expected_mat(1, i) = std::get<1>(edge);
        ++i;
    }

    expectEigenMatsEqual(config_test.tracked_.getVertices(), verts_expected_mat);
    expectEigenMatsEqual(config_test.tracked_.getEdges(), edges_expected_mat);
    EXPECT_TRUE(config_test.num_edges_ == num_edges_expected);
}

TEST(ClothConfigurationTest, rectangularTemplate)
{
    double const cloth_length = 1.0;
    double const cloth_width = 1.5;
    double const grid_size_initial_guess = 0.74;
    ClothConfiguration config_test{cloth_length, cloth_width, grid_size_initial_guess};
    config_test.initializeTracking();

    // Setup the expected points and vertices
    std::vector<cv::Point3f> verts_expected = {
        {0, 0, 0},
        {0, 0.5, 0},
        {0, 1, 0},
        {0, 1.5, 0},
        {0.5, 0, 0},
        {0.5, 0.5, 0},
        {0.5, 1, 0},
        {0.5, 1.5, 0},
        {1.0, 0, 0},
        {1.0, 0.5, 0},
        {1.0, 1, 0},
        {1.0, 1.5, 0},
    };
    Eigen::Matrix<float, 3, 12> verts_expected_mat;
    int i = 0;
    for (auto const& vert : verts_expected)
    {
        verts_expected_mat(0, i) = vert.x;
        verts_expected_mat(1, i) = vert.y;
        verts_expected_mat(2, i) = vert.z;
        ++i;
    }

    int const num_edges_expected = 17;
    std::vector<std::tuple<int, int>> edges_expected = {
        {0, 4},
        {0, 1},
        {1, 5},
        {1, 2},
        {2, 6},
        {2, 3},
        {3, 7},
        {4, 8},
        {4, 5},
        {5, 9},
        {5, 6},
        {6, 10},
        {6, 7},
        {7, 11},
        {8, 9},
        {9, 10},
        {10, 11}
    };
    Eigen::Matrix<int, 2, num_edges_expected> edges_expected_mat;
    i = 0;
    for (auto const& edge : edges_expected)
    {
        edges_expected_mat(0, i) = std::get<0>(edge);
        edges_expected_mat(1, i) = std::get<1>(edge);
        ++i;
    }

    expectEigenMatsEqual(config_test.tracked_.getVertices(), verts_expected_mat);
    expectEigenMatsEqual(config_test.tracked_.getEdges(), edges_expected_mat);
    EXPECT_TRUE(config_test.num_edges_ == num_edges_expected);
}