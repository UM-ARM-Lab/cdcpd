#include <ros/ros.h>
#include <termios.h>
#include <visualization_msgs/Marker.h>

#include "arc_utilities/arc_helpers.hpp"
#include "arc_utilities/eigen_helpers_conversions.hpp"
#include "arc_utilities/thin_plate_spline.hpp"

using namespace Eigen;

int main(int argc, char* argv[]) {
  // Read in all ROS parameters
  ros::init(argc, argv, "test_thin_plate_spline_node");

  auto nh = ros::NodeHandle();
  auto visualization_marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 4);
  arc_helpers::Sleep(0.5);

  // Testing 2D
  {
    const auto num_grid_x_points = 18;
    const auto num_grid_y_points = 18;
    const auto num_grid_points = num_grid_x_points * num_grid_y_points;
    const VectorXd X_grid = VectorXd::LinSpaced(num_grid_x_points, -0.85, 0.85);
    const VectorXd Y_grid = VectorXd::LinSpaced(num_grid_y_points, -0.85, 0.85);
    Matrix2Xd grid_points = Matrix2Xd(2, num_grid_points);
    for (auto x_ind = 0; x_ind < num_grid_x_points; ++x_ind) {
      for (auto y_ind = 0; y_ind < num_grid_y_points; ++y_ind) {
        grid_points.col(x_ind * num_grid_y_points + y_ind) << X_grid(x_ind), Y_grid(y_ind);
      }
    }

    while (ros::ok()) {
      const auto num_x_points = 3;
      const auto num_y_points = 3;
      const auto num_points = num_x_points * num_y_points;
      const VectorXd X = VectorXd::LinSpaced(num_x_points, -0.5, 0.5);
      const VectorXd Y = VectorXd::LinSpaced(num_y_points, -0.5, 0.5);
      Matrix2Xd template_points = Matrix2Xd(2, num_points);
      for (auto x_ind = 0; x_ind < num_x_points; ++x_ind) {
        for (auto y_ind = 0; y_ind < num_y_points; ++y_ind) {
          template_points.col(x_ind * num_y_points + y_ind) << X(x_ind), Y(y_ind);
        }
      }
      template_points += 0.1 * Matrix2Xd::Random(2, num_points);
      const Matrix2Xd target_points =
          template_points + 2.5 * Matrix2Xd::Ones(2, num_points) + 0.2 * Matrix2Xd::Random(2, num_points);

      arc_utilities::ThinPlateSpline<2> tps_2d(template_points, target_points);
      const Matrix2Xd warped_grid_points = tps_2d.interpolate(grid_points);

      // Basic test for obvious errors
      const Matrix2Xd interpolate_test = tps_2d.interpolate(template_points);
      assert(target_points.isApprox(interpolate_test));

      // Visualization
      {
        using namespace visualization_msgs;
        {
          Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = ros::Time::now();
          marker.ns = "template_points";
          marker.id = 1;
          marker.type = Marker::POINTS;
          marker.action = Marker::ADD;
          marker.scale.x = 0.05;
          marker.scale.y = 0.05;
          marker.scale.z = 0.05;
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          marker.color.a = 1.0;
          marker.points.resize(num_points);
          for (auto idx = 0; idx < num_points; ++idx) {
            marker.points[idx].x = template_points.col(idx).x();
            marker.points[idx].y = template_points.col(idx).y();
            marker.points[idx].z = 0.0;
          }
          visualization_marker_pub.publish(marker);
        }
        {
          Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = ros::Time::now();
          marker.ns = "target_points";
          marker.id = 1;
          marker.type = Marker::POINTS;
          marker.action = Marker::ADD;
          marker.scale.x = 0.05;
          marker.scale.y = 0.05;
          marker.scale.z = 0.05;
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          marker.color.a = 1.0;
          marker.points.resize(num_points);
          for (auto idx = 0; idx < num_points; ++idx) {
            marker.points[idx].x = target_points.col(idx).x();
            marker.points[idx].y = target_points.col(idx).y();
            marker.points[idx].z = 0.0;
          }
          visualization_marker_pub.publish(marker);
        }
        {
          Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = ros::Time::now();
          marker.ns = "grid_points";
          marker.id = 1;
          marker.type = Marker::POINTS;
          marker.action = Marker::ADD;
          marker.scale.x = 0.05;
          marker.scale.y = 0.05;
          marker.scale.z = 0.05;
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          marker.color.a = 0.5f;
          marker.points.resize(num_grid_points);
          for (auto idx = 0; idx < num_grid_points; ++idx) {
            marker.points[idx].x = grid_points.col(idx).x();
            marker.points[idx].y = grid_points.col(idx).y();
            marker.points[idx].z = 0.0;
          }
          visualization_marker_pub.publish(marker);
        }
        {
          Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = ros::Time::now();
          marker.ns = "warped_grid_points";
          marker.id = 1;
          marker.type = Marker::POINTS;
          marker.action = Marker::ADD;
          marker.scale.x = 0.05;
          marker.scale.y = 0.05;
          marker.scale.z = 0.05;
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          marker.color.a = 0.5f;
          marker.points.resize(num_grid_points);
          for (auto idx = 0; idx < num_grid_points; ++idx) {
            marker.points[idx].x = warped_grid_points.col(idx).x();
            marker.points[idx].y = warped_grid_points.col(idx).y();
            marker.points[idx].z = 0.0;
          }
          visualization_marker_pub.publish(marker);
        }
      }

      std::cout << "Testing 2D TPS interpolation. "
                   "Press 'n' to generate a new instance, "
                   "anything else to move on. ";
      const int c = arc_helpers::GetChar();
      std::cout << std::endl;
      if (c != 'n') {
        break;
      }
    }
  }

  // Testing 3D
  {
    const auto num_grid_x_points = 9;
    const auto num_grid_y_points = 9;
    const auto num_grid_z_points = 9;
    const auto num_grid_points = num_grid_x_points * num_grid_y_points * num_grid_z_points;
    const VectorXd X_grid = VectorXd::LinSpaced(num_grid_x_points, -0.85, 0.85);
    const VectorXd Y_grid = VectorXd::LinSpaced(num_grid_y_points, -0.85, 0.85);
    const VectorXd Z_grid = VectorXd::LinSpaced(num_grid_z_points, -0.85, 0.85);
    Matrix3Xd grid_points = Matrix3Xd(3, num_grid_points);
    for (auto x_ind = 0; x_ind < num_grid_x_points; ++x_ind) {
      for (auto y_ind = 0; y_ind < num_grid_y_points; ++y_ind) {
        for (auto z_ind = 0; z_ind < num_grid_z_points; ++z_ind) {
          grid_points.col((x_ind * num_grid_y_points + y_ind) * num_grid_z_points + z_ind) << X_grid(x_ind),
              Y_grid(y_ind), Z_grid(z_ind);
        }
      }
    }

    while (ros::ok()) {
      const auto num_x_points = 3;
      const auto num_y_points = 3;
      const auto num_z_points = 3;
      const auto num_points = num_x_points * num_y_points * num_z_points;
      const VectorXd X = VectorXd::LinSpaced(num_x_points, -0.5, 0.5);
      const VectorXd Y = VectorXd::LinSpaced(num_y_points, -0.5, 0.5);
      const VectorXd Z = VectorXd::LinSpaced(num_z_points, -0.5, 0.5);
      auto template_points = Matrix3Xd(3, num_points);
      for (auto x_ind = 0; x_ind < num_x_points; ++x_ind) {
        for (auto y_ind = 0; y_ind < num_y_points; ++y_ind) {
          for (auto z_ind = 0; z_ind < num_z_points; ++z_ind) {
            template_points.col((x_ind * num_y_points + y_ind) * num_z_points + z_ind) << X(x_ind), Y(y_ind), Z(z_ind);
          }
        }
      }
      template_points += 0.1 * Matrix3Xd::Random(3, num_points);
      const Matrix3Xd target_points =
          template_points + 2.0 * Matrix3Xd::Ones(3, num_points) + 0.2 * Matrix3Xd::Random(3, num_points);

      arc_utilities::ThinPlateSpline<3> tps_3d(template_points, target_points);
      const auto warped_grid_points = tps_3d.interpolate(grid_points);

      // Basic test for obvious errors
      const Matrix3Xd interpolate_test = tps_3d.interpolate(template_points);
      assert(target_points.isApprox(interpolate_test));

      // Visualization
      {
        using namespace visualization_msgs;
        {
          Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = ros::Time::now();
          marker.ns = "template_points";
          marker.id = 1;
          marker.type = Marker::POINTS;
          marker.action = Marker::ADD;
          marker.scale.x = 0.05;
          marker.scale.y = 0.05;
          marker.scale.z = 0.05;
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          marker.color.a = 1.0;
          marker.points = EigenHelpersConversions::EigenMatrix3XdToVectorGeometryPoint(template_points);
          visualization_marker_pub.publish(marker);
        }
        {
          Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = ros::Time::now();
          marker.ns = "target_points";
          marker.id = 1;
          marker.type = Marker::POINTS;
          marker.action = Marker::ADD;
          marker.scale.x = 0.05;
          marker.scale.y = 0.05;
          marker.scale.z = 0.05;
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          marker.color.a = 1.0;
          marker.points = EigenHelpersConversions::EigenMatrix3XdToVectorGeometryPoint(target_points);
          visualization_marker_pub.publish(marker);
        }
        {
          Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = ros::Time::now();
          marker.ns = "grid_points";
          marker.id = 1;
          marker.type = Marker::POINTS;
          marker.action = Marker::ADD;
          marker.scale.x = 0.05;
          marker.scale.y = 0.05;
          marker.scale.z = 0.05;
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          marker.color.a = 0.5f;
          marker.points = EigenHelpersConversions::EigenMatrix3XdToVectorGeometryPoint(grid_points);
          visualization_marker_pub.publish(marker);
        }
        {
          Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = ros::Time::now();
          marker.ns = "warped_grid_points";
          marker.id = 1;
          marker.type = Marker::POINTS;
          marker.action = Marker::ADD;
          marker.scale.x = 0.05;
          marker.scale.y = 0.05;
          marker.scale.z = 0.05;
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          marker.color.a = 0.5f;
          marker.points = EigenHelpersConversions::EigenMatrix3XdToVectorGeometryPoint(warped_grid_points);
          visualization_marker_pub.publish(marker);
        }
      }

      std::cout << "Testing 3D TPS interpolation. "
                   "Press 'n' to generate a new instance, "
                   "anything else to move on. ";
      const int c = arc_helpers::GetChar();
      std::cout << std::endl;
      if (c != 'n') {
        break;
      }
    }
  }

  return EXIT_SUCCESS;
}
