#include <iostream>
#include "arc_utilities/eigen_helpers.hpp"
#include "arc_utilities/pretty_print.hpp"

<<<<<<< HEAD
int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  const Eigen::Matrix3d r1 = Eigen::Matrix3d::Identity();
  const Eigen::Quaterniond q1(r1);

  for (double theta = -M_PI; theta <= M_PI; theta += M_PI / 20) {
    Eigen::Quaterniond q2;
    q2 = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX());
    Eigen::Matrix3d r2 = q2.matrix();

    const double matrix_dist = EigenHelpers::Distance(r1, r2);
    const double quat_dist = EigenHelpers::Distance(q1, q2);
    const double eigen_angular_dist = q1.angularDistance(q2);

    std::cout << "True rotation: " << theta << "    Matrix dist: " << matrix_dist << "    Quat dist: " << quat_dist
              << "    Eigen dist: " << eigen_angular_dist << std::endl;

    assert(EigenHelpers::CloseEnough(std::fabs(theta), matrix_dist, 1e-10));
    assert(EigenHelpers::CloseEnough(std::fabs(theta), quat_dist, 1e-10));
    assert(EigenHelpers::CloseEnough(std::fabs(theta), eigen_angular_dist, 1e-10));
  }

  return EXIT_SUCCESS;
=======
int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;


    const Eigen::Matrix3d r1 = Eigen::Matrix3d::Identity();
    const Eigen::Quaterniond q1(r1);

    for (double theta = -M_PI; theta <= M_PI; theta += M_PI / 20)
    {
        Eigen::Quaterniond q2; q2 = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d r2 = q2.matrix();

        const double matrix_dist = EigenHelpers::Distance(r1, r2);
        const double quat_dist = EigenHelpers::Distance(q1, q2);
        const double eigen_angular_dist = q1.angularDistance(q2);

        std::cout << "True rotation: " << theta
                  << "    Matrix dist: " << matrix_dist
                  << "    Quat dist: " << quat_dist
                  << "    Eigen dist: " << eigen_angular_dist
                  << std::endl;

        assert(EigenHelpers::CloseEnough(std::fabs(theta), matrix_dist, 1e-10));
        assert(EigenHelpers::CloseEnough(std::fabs(theta), quat_dist, 1e-10));
        assert(EigenHelpers::CloseEnough(std::fabs(theta), eigen_angular_dist, 1e-10));
    }

    return EXIT_SUCCESS;
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
}
