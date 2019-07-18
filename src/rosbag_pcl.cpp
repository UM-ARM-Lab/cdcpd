#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  sensor_msgs::PointCloud2 object_msg;
 void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
    //do stuff with temp_cloud here
    // pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    // pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.02f, 0.02f, 0.02f);
    sor.filter (*cloud_filtered);
    // std::cout<<cloud_filtered->size()<<std::endl;
    object_msg.header = input->header;
    object_msg.height = cloud_filtered->size();
    object_msg.width = 1;
    // pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pcl::toROSMsg(*cloud_filtered,object_msg );
    }

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "rosbag_pcl");
  ros::NodeHandle n;
  ros::Rate rate(10.0);
  ros::Subscriber sub = n.subscribe("/cdcpd/mask_filtered_points", 10, cloud_cb);
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/cdcpd/mask_down_sampled_points", 10);
   while(ros::ok())
  {
    ros::spinOnce();
    pub.publish(object_msg);
    rate.sleep();
  }

  return 0;
}




// #include <ros/ros.h>
// // PCL specific includes
// #include <pcl/ros/conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/filters/voxel_grid.h>

// ros::Publisher pub;

// void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud){
// sensor_msgs::PointCloud2 cloud_filtered;

//  // Perform the actual filtering
// pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
// sor.setInputCloud (cloud);
// sor.setLeafSize (0.01, 0.01, 0.01);
// sor.filter (cloud_filtered);

// // Publish the data
// pub.publish (cloud_filtered);
// }

// int main (int argc, char** argv){
// // Initialize ROS
// ros::init (argc, argv, "bag_pcl");
// ros::NodeHandle nh;

//  // Create a ROS subscriber for the input point cloud
// ros::Subscriber sub = nh.subscribe ("/cdcpd/mask_filtered_points", 10, cloud_cb);

// // Create a ROS publisher for the output point cloud
// pub = nh.advertise<sensor_msgs::PointCloud2> ("/trial/down_sampled", 10);

// // Spin
// ros::spin ();
// }