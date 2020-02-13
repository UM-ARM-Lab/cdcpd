#include <thread>
#include <chrono>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "cdcpd/cdcpd.h"

using std::cout;
using std::endl;

using namespace cv;

// pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
// {
// 	// --------------------------------------------
// 	// -----Open 3D viewer and add point cloud-----
// 	// --------------------------------------------
// 	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
// 	viewer->setBackgroundColor (0, 0, 0);
// 	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
// 	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
// 	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
// 	viewer->addCoordinateSystem (1.0);
// 	viewer->initCameraParameters();
// 	return viewer;
// }

int main() {
    cout << "Test started" << endl;

    Mat color_image_bgr = imread("../../../../data/image_color_rect.png", IMREAD_COLOR);
    Mat rgb_image;
    cv::cvtColor(color_image_bgr, rgb_image, cv::COLOR_BGR2RGB);
    Mat depth_image = imread("../../../../data/image_depth_rect.png", IMREAD_GRAYSCALE);
    FileStorage color_calib_fs("../../../kinect2_calibration_files/data/000792364047/calib_color.yaml", FileStorage::READ);
    // TODO actually read the cv::Mat
    cv::Mat intrinsics;
    color_calib_fs["cameraMatrix"] >> intrinsics;
    //FileStorage depth_calib_fs("../../../kinect2_calibration_files/data/000792364047/calib_color.yaml");

    cout << rgb_image.cols << " " << rgb_image.rows << endl;
    cout << depth_image.cols << " " << depth_image.rows << endl;

    imwrite("rgb.png", rgb_image);
    imwrite("depth.png", depth_image);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = cdcpd(rgb_image, depth_image, intrinsics);

	// pcl::visualization::PCLVisualizer::Ptr viewer = rgbVis(cloud);
    // 
	// while (!viewer->wasStopped())
	// {
	// 	viewer->spinOnce (100);
	// 	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	// }

    cout << "Test ended" << endl;
}
