#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

namespace kinematics
{
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Xd;

  ////////////////////////////////////////////////////////////////////////////
  // Typedefs for aligned STL containers using Eigen types
  ////////////////////////////////////////////////////////////////////////////

  typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> VectorVector3f;
  typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VectorVector3d;
  typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> VectorVector4f;
  typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> VectorVector4d;
  typedef std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> VectorVector6d;
  typedef std::vector<Eigen::Matrix3Xd> VectorMatrix3Xd;

  typedef std::vector<Eigen::Quaternionf, Eigen::aligned_allocator<Eigen::Quaternionf>> VectorQuaternionf;
  typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> VectorQuaterniond;
  typedef std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f>> VectorAffine3f;
  typedef std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> VectorIsometry3d;

  typedef struct
  {
    Vector6d vel;
    bool valid;
  } Velocity6d;

  ////////////////////////////////////////////////////////////////////////////////
  // Basic Transforms
  ////////////////////////////////////////////////////////////////////////////////

  Eigen::Matrix4d rotX(const double theta);
  Eigen::Matrix4d rotY(const double theta);
  Eigen::Matrix4d rotZ(const double theta);
  Eigen::Matrix4d translate(const Eigen::Vector3d& p);
  Eigen::Matrix4d transX(const double x);
  Eigen::Matrix4d transY(const double y);
  Eigen::Matrix4d transZ(const double z);

  ////////////////////////////////////////////////////////////////////////////////
  // Skew and Unskew for 3x3 matrices
  ////////////////////////////////////////////////////////////////////////////////

  Eigen::Matrix3d skew(const Eigen::Vector3d& w);
  Eigen::Vector3d unskew(const Eigen::Matrix3d& w_hat);

  ////////////////////////////////////////////////////////////////////////////////
  // Twist create, cacluting twists, twist hat and unhat
  ////////////////////////////////////////////////////////////////////////////////

  Vector6d createTwist(const Eigen::Vector3d& omega, const Eigen::Vector3d& q);
  std::vector<Vector6d> createTwist(const std::vector<Eigen::Vector3d>& omega,
                                    const std::vector<Eigen::Vector3d>& q);
  std::vector<Vector6d> calculateTwists(const Eigen::Matrix4d& g_base,
                                        const std::vector<Eigen::Vector3d>& omega0,
                                        const std::vector<Eigen::Vector3d>& q0);
  Eigen::Matrix4d twistHat(const Vector6d& xi);
  Vector6d twistUnhat(const Eigen::Matrix4d& xi_hat);

  ////////////////////////////////////////////////////////////////////////////////
  // Adjoints and twist exponentials
  ////////////////////////////////////////////////////////////////////////////////

  Matrix6d adj(const Eigen::Matrix4d& g);
  Eigen::Matrix3d expmExact(const Eigen::Matrix3d& w_hat, const double theta);
  Eigen::Matrix4d expTwist(const Vector6d& xi, double theta);
  Eigen::Isometry3d expTwistIsometry3d(const Vector6d& xi, double theta);
  Eigen::Matrix4d expTwist(const std::vector<Vector6d>& xi,
                           const std::vector<double>& theta);
  Eigen::Isometry3d expTwistIsometry3d(const std::vector<Vector6d>& xi,
                                   const std::vector<double>& theta);

  VectorIsometry3d applyTwist(const VectorIsometry3d& starting_pose,
                            const VectorVector6d& xi,
                            std::vector<double> theta = std::vector<double>(0));

  ////////////////////////////////////////////////////////////////////////////////
  // Jacobians
  ////////////////////////////////////////////////////////////////////////////////

  Matrix6Xd spatialJacobian(const std::vector<Vector6d>& xi,
                            const std::vector<double>& theta);
  Matrix6Xd bodyJacobian(const std::vector<Vector6d>& xi,
                         const std::vector<double>& theta,
                         const Eigen::Matrix4d& g_zero);

  Eigen::MatrixXd dampedPinv6Xd(const kinematics::Matrix6Xd& J,
                                const std::vector<double>& theta,
                                const double joint_limit,
                                const double limit_threshold,
                                const double manipubility_threshold,
                                const double damping_ratio);

  Eigen::MatrixXd dampedPinvXd(const Eigen::MatrixXd& J,
                               const std::vector<double>& theta,
                               const double joint_limit,
                               const double limit_threshold,
                               const double manipubility_threshold,
                               const double damping_ratio);

  ////////////////////////////////////////////////////////////////////////////////
  // Other
  ////////////////////////////////////////////////////////////////////////////////

  Vector6d calculateVelocity(const Eigen::Isometry3d& g_current,
                             const Eigen::Isometry3d& g_next,
                             const double dt);

  VectorVector6d calculateVelocities(const VectorIsometry3d& g_trajectory,
                                     const double dt);

  Vector6d calculateError(const Eigen::Isometry3d& g_current,
                          const Eigen::Isometry3d& g_desired);

  Vector6d calculateError(const Eigen::Matrix4d& g_current,
                          const Eigen::Matrix4d& g_desired);

  Eigen::VectorXd calulateDeltas(const VectorIsometry3d& g);
}

#endif
