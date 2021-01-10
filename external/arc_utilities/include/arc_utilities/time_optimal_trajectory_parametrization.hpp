#include <stdlib.h>

#include <Eigen/Geometry>
#include <chrono>
#include <functional>
#include <iostream>
#include <list>
#include <vector>

#ifndef TIME_OPTIMAL_TRAJECTORY_PARAMETRIZATION_HPP
#define TIME_OPTIMAL_TRAJECTORY_PARAMETRIZATION_HPP

namespace time_optimal_trajectory_parametrization {
class PathSegment {
 public:
  PathSegment(const double length = 0.0) : length_(length) {}

  virtual ~PathSegment() {}

  double Length() const { return length_; }

  virtual Eigen::VectorXd GetConfig(const double s) const = 0;

  virtual Eigen::VectorXd GetTangent(const double s) const = 0;

  virtual Eigen::VectorXd GetCurvature(const double s) const = 0;

  virtual std::list<double> GetSwitchingPoints() const = 0;

  virtual PathSegment* Clone() const = 0;

  double position;

 protected:
  double length_;
};

class Path {
 public:
  Path(const std::list<Eigen::VectorXd>& path, const double maxDeviation = 0.0);

  Path(const Path& path);

  ~Path();

  double Length() const;

  Eigen::VectorXd GetConfig(const double s) const;

  Eigen::VectorXd GetTangent(const double s) const;

  Eigen::VectorXd GetCurvature(const double s) const;

  double GetNextSwitchingPoint(const double s, bool& discontinuity) const;

  std::list<std::pair<double, bool>> SwitchingPoints() const;

 private:
  PathSegment* GetPathSegment(double& s) const;

  double length_;

  std::list<std::pair<double, bool>> switching_points_;

  std::list<PathSegment*> path_segments_;
};

class Trajectory {
 public:
  // Generates a time-optimal trajectory
  Trajectory(const std::list<Eigen::VectorXd>& waypoints, const Eigen::VectorXd& max_velocity,
             const Eigen::VectorXd& max_acceleration, const double max_deviation, const double timestep);

  Trajectory(const Path& path_, const Eigen::VectorXd& max_velocity_, const Eigen::VectorXd& max_acceleration_,
             double timestep = 0.001);

  ~Trajectory(void);

  // Returns the optimal duration of the trajectory
  double Duration() const;

  // Return the position vector and velocity vector of the
  // robot for a given point in time within the trajectory.
  // !!! NOT THREAD SAFE - MUTABLE CACHE INSIDE !!!
  std::pair<Eigen::VectorXd, Eigen::VectorXd> GetPositionVelocity(const double time) const;

  // Outputs the phase trajectory and the velocity limit curve
  // in 2 files for debugging purposes.
  void OutputPhasePlaneTrajectory() const;

 private:
  struct TrajectoryStep {
    TrajectoryStep() {}

    TrajectoryStep(double path_pos, double path_vel) : path_pos_(path_pos), path_vel_(path_vel) {}

    double path_pos_;
    double path_vel_;
    double time_;
  };

  bool GetNextSwitchingPoint(const double path_pos, TrajectoryStep& next_switching_point, double& before_acceleration,
                             double& after_acceleration);

  bool GetNextAccelerationSwitchingPoint(const double path_pos, TrajectoryStep& next_switching_point,
                                         double& before_acceleration, double& after_acceleration);

  bool GetNextVelocitySwitchingPoint(const double path_pos, TrajectoryStep& next_switching_point,
                                     double& before_acceleration, double& after_acceleration);

  bool IntegrateForward(std::list<TrajectoryStep>& trajectory, const double acceleration);

  void IntegrateBackward(std::list<TrajectoryStep>& start_trajectory, const double path_pos, const double path_vel,
                         const double acceleration);

  double GetMinMaxPathAcceleration(const double path_position, const double path_velocity, const bool max);

  double GetMinMaxPhaseSlope(const double path_position, const double path_velocity, const bool max);

  double GetAccelerationMaxPathVelocity(const double path_pos) const;

  double GetVelocityMaxPathVelocity(const double path_pos) const;

  double GetAccelerationMaxPathVelocityDeriv(const double path_pos);

  double GetVelocityMaxPathVelocityDeriv(const double path_pos);

  std::list<TrajectoryStep>::const_iterator GetTrajectorySegment(const double time) const;

  Path path_;
  Eigen::VectorXd max_velocity_;
  Eigen::VectorXd max_acceleration_;
  unsigned int n_;
  std::list<TrajectoryStep> trajectory_;

  static const double eps;
  const double time_step_;

  mutable double cached_time_;
  mutable std::list<TrajectoryStep>::const_iterator cached_trajectory_segment_;
};
}  // namespace time_optimal_trajectory_parametrization

#endif  // TIME_OPTIMAL_TRAJECTORY_PARAMETRIZATION_HPP
