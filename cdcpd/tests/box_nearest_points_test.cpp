#include <cdcpd/optimizer.h>

using Eigen::Matrix3Xf;

int main()
{
  Matrix3Xf init_temp(3,5);
  init_temp(0, 0) =  1.0; init_temp(1, 0) = 0.0; init_temp(2, 0) = -1.0; // near: (sqrt(2)/2, 0, 0); normal: (1/sqrt(3), 0, -sqrt(2/3))
  init_temp(0, 1) =  0.0; init_temp(1, 1) = 2.0; init_temp(2, 1) =  3.0; // near: (0, 0.5, 1.5); normal: (0, sqrt(2)/2, sqrt(2)/2)
  init_temp(0, 2) = -2.0; init_temp(1, 2) = 0.0; init_temp(2, 2) =  1.5; // near: (-sqrt(2)/2, 0, 1.5); normal: (-1, 0, 0)
  init_temp(0, 3) =  0.0; init_temp(1, 3) = 0.0; init_temp(2, 3) =  3.0; // near: (0, 0, 2); normal: (0, 0, 1)
  init_temp(0, 4) =  0.5; init_temp(1, 4) = 0.5; init_temp(2, 4) =  1.0; // near: (sqrt(2)/2, 0.5, 1.0); normal: (1, 0, 0)
  Matrix3Xf last_temp = init_temp;
  
  shape_msgs::SolidPrimitive box;
  box.type = shape_msgs::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[shape_msgs::SolidPrimitive::BOX_X] = sqrt(2);
  box.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = sqrt(2);
  box.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = sqrt(2);
  
  geometry_msgs::Pose pose;
  geometry_msgs::Point center;
  center.x = 0; center.y = 0; center.z = 1;
  pose.position = center;

  geometry_msgs::Quaternion orien;
  orien.x = 0.3826834; orien.y = 0; orien.z = 0; orien.w = 0.9238795;
  pose.orientation = orien;

  Optimizer opt(init_temp, last_temp, 1.0);
  auto res = opt.test_box(init_temp, box, pose);

  std::cout << std::get<0>(res) << std::endl;
  std::cout << std::get<1>(res) << std::endl;
}
