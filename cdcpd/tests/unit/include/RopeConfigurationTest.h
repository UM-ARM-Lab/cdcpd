#pragma once

#include <tuple>
#include <iostream>

#include <gtest/gtest.h>

#include "cdcpd/deformable_object_configuration.h"
#include "test_utils.h"

// Tests a simple rope configuration for correctness.
TEST(RopeConfigurationTest, simpleRopeConfigInitialization)
{
    int const num_points = 5;
    float const max_rope_length = 1.0;
    Eigen::Vector3f start_position{0, 0, 0};
    Eigen::Vector3f end_position{max_rope_length, 0, 0};
    RopeConfiguration config{num_points, max_rope_length, start_position, end_position};

    int num_rope_segments = num_points - 1;
    float const max_segment_length = max_rope_length / static_cast<float>(num_rope_segments);

    EXPECT_EQ(config.num_points_, num_points);
    EXPECT_FLOAT_EQ(config.max_rope_length_, max_rope_length);
    EXPECT_FLOAT_EQ(config.max_segment_length_, max_segment_length);
}

TEST(RopeConfigurationTest, simpleRopeConfigTemplateInitialization)
{
    int const num_points = 5;
    float const max_rope_length = 1.0;
    Eigen::Vector3f start_position{0, 0, 0};
    Eigen::Vector3f end_position{max_rope_length, 0, 0};
    RopeConfiguration config{num_points, max_rope_length, start_position, end_position};

    config.initializeTracking();

    DeformableObjectTracking track_expected;
    std::vector<cv::Point3f> vertices_expected = {
        {0, 0, 0},
        {0.25, 0, 0},
        {0.5, 0, 0},
        {0.75, 0, 0},
        {1.0, 0, 0}
    };
    std::vector<std::tuple<int, int>> edges_expected;
    for (int i = 0; i < num_points - 1; ++i)
    {
        edges_expected.push_back(std::make_tuple(i, i + 1));
    }
    track_expected.setVertices(vertices_expected);
    track_expected.setEdges(edges_expected);

    expectEigenMatsEqual(config.tracked_.vertices_, track_expected.vertices_);
    expectEigenMatsEqual(config.tracked_.edges_, track_expected.edges_);
}