#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "cdcpd/cdcpd.h"

using std::cout;
using std::endl;

using namespace cv;

int main() {
    cout << "Test started" << endl;

    Mat color_image = imread("../../../../data/image_color_rect.png", IMREAD_COLOR);
    Mat depth_image = imread("../../../../data/image_depth_rect.png", IMREAD_GRAYSCALE);

    cout << color_image.cols << " " << color_image.rows << endl;
    cout << depth_image.cols << " " << depth_image.rows << endl;

    imwrite("color.png", color_image);
    imwrite("depth.png", depth_image);

    cout << "Test ended" << endl;
}
