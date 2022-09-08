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

// Tests the tracking initialization of a very simple template, a 2x2 cloth
TEST(ClothConfigurationTest, simpleClothConfigurationTemplateInitializationBadGridSizeGuess)
{
    // The following values give a 2x2 template
    float const cloth_length = 1.0;
    float const cloth_width = 1.0;
    float const grid_size_initial_guess = 0.76;
    ClothConfiguration config_test{cloth_length, cloth_width, grid_size_initial_guess};
    config_test.initializeTracking();

    // Setup the expected points and vertices
    DeformableObjectTracking tracking_expected;
    std::vector<cv::Point3f> vertices_expected = {
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
    tracking_expected.setVertices(vertices_expected);
    tracking_expected.setEdges(edges_expected);

    expectEigenMatsEqual(config_test.tracked_.vertices_, tracking_expected.vertices_);
    expectEigenMatsEqual(config_test.tracked_.edges_, tracking_expected.edges_);
}