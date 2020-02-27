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
using Eigen::MatrixXf;
using Eigen::MatrixXi;

using namespace cv;


pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return viewer;
}

// pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
// {
//     // --------------------------------------------
//     // -----Open 3D viewer and add point cloud-----
//     // --------------------------------------------
//     pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//     viewer->setBackgroundColor (0, 0, 0);
//     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
//     viewer->addPointCloud<pcl::PointXYZ> (cloud, rgb, "sample cloud");
//     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//     viewer->addCoordinateSystem (1.0);
//     viewer->initCameraParameters();
//     return viewer;
// }

void print_mat16(const cv::Mat& m) {
    for (int v = 0; v < m.rows; ++v)
    {
        for (int u = 0; u < m.cols; ++u)
        {
            cout << (int)m.at<uint8_t>(v, u) << " ";
        }
        cout << endl;
    }
}

int main() {
    cout << "Test started" << endl;

    MatrixXf template_vertices(3, 50);
    template_vertices.setZero();
    template_vertices.row(0).setLinSpaced(50, 0, 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < template_vertices.cols(); ++i)
    {
        const auto& c = template_vertices.col(i);
        template_cloud->push_back(pcl::PointXYZ(c(0), c(1), c(2)));
    }

    MatrixXi template_edges(49, 2);
    template_edges(0, 0) = 0;
    template_edges(template_edges.rows() - 1, 1) = 49;
    for (int i = 1; i <= template_edges.rows() - 1; ++i)
    {
        template_edges(i, 0) = i;
        template_edges(i - 1, 1) = i;
    }

    // cout << "template_vertices" << endl;
    // cout << template_vertices << endl;
    // cout << "template_edges" << endl;
    // cout << template_edges << endl;

    FileStorage color_calib_fs("../../../kinect2_calibration_files/data/000792364047/calib_color.yaml", FileStorage::READ);
    cv::Mat intrinsics;
    color_calib_fs["cameraMatrix"] >> intrinsics;
    // cout << "intrinsics type: " << intrinsics.type() << endl;

    CDCPD cdcpd(intrinsics);

    // TODO more
    for (int i = 0; i < 5; ++i)
    {

        /// Color filter
        // For the red rope, (h > 0.85) & (s > 0.5). For the flag, (h < 1.0) & (h > 0.9)
        // The flag isn't tested or implemented
        Mat color_image_bgr = imread("../../../../data/color.png", IMREAD_COLOR);
        Mat rgb_image;
        cv::cvtColor(color_image_bgr, rgb_image, cv::COLOR_BGR2RGB);
        // TODO I'm pretty sure this is an 8-bit image.
        Mat depth_image = imread("../../../../data/depth.png", IMREAD_UNCHANGED);
        // cout << "Depth image " << endl;
        // print_mat16(depth_image);

        //FileStorage depth_calib_fs("../../../kinect2_calibration_files/data/000792364047/calib_color.yaml");

        // cout << rgb_image.cols << " " << rgb_image.rows << endl;
        // cout << depth_image.cols << " " << depth_image.rows << endl;

        imwrite("rgb.png", rgb_image);
        imwrite("depth.png", depth_image);

        cv::Mat rgb_f;
        rgb_image.convertTo(rgb_f, CV_32FC3);
        rgb_f /= 255.0; // get RGB 0.0-1.0
        cv::Mat color_hsv;
        cvtColor(rgb_f, color_hsv, CV_RGB2HSV);
        cv::Scalar low_hsv = cv::Scalar(.85 * 360.0, 0.5, 0.0);
        cv::Scalar high_hsv = cv::Scalar(360.0, 1.0, 1.0);
        cv::Mat hsv_mask;
        cv::inRange(color_hsv, low_hsv, high_hsv, hsv_mask);
        cv::imwrite("hsv_mask.png", hsv_mask);

        template_cloud = cdcpd(rgb_image, depth_image, hsv_mask, template_cloud, template_edges);

        pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(template_cloud);
        
        while (!viewer->wasStopped())
        {
            viewer->spinOnce (100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    }

    cout << "Test ended" << endl;
}
