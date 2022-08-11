#include <ros/ros.h>
#include <gtest/gtest.h>

// All of the test files we want to run
#include "MeshConversionTest.h"
#include "BoxNearestPointsTest.h"

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}