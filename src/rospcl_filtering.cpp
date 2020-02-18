#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::Ptr output_cloud;

void cloud_cb(const PointCloud::ConstPtr& input_cloud)
{
    auto cloud_filtered = std::make_shared<PointCloud>();
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);
    sor.setLeafSize(0.02f, 0.02f, 0.02f);
    sor.filter(*output_cloud);
    // std::cerr << "Input header:\n" << output_cloud->header;
    // std::cerr << "Output header:\n" << output_cloud->header;
    // std::cerr << output_cloud->width << " " << output_cloud->height << "\n\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rospcl_filtering");
    ros::NodeHandle n;
    output_cloud = boost::make_shared<PointCloud>();
    ros::Rate rate(10.0);
    ros::Subscriber sub = n.subscribe("cdcpd/mask_filtered_points", 1, cloud_cb);
    ros::Publisher pub = n.advertise<PointCloud>("cdcpd/mask_down_sampled_points", 10);
    while (ros::ok())
    {
        ros::spinOnce();
        pub.publish(output_cloud);
        rate.sleep();
    }
    return EXIT_SUCCESS;
}

