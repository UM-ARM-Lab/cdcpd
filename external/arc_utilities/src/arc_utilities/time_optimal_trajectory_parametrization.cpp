#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/time_optimal_trajectory_parametrization.hpp>
#include <fstream>

namespace time_optimal_trajectory_parametrization
{
class LinearPathSegment : public PathSegment
{
public:

  LinearPathSegment(const Eigen::VectorXd& start,
                    const Eigen::VectorXd& end)
    : PathSegment((end-start).norm()), start_(start), end_(end) {}

  Eigen::VectorXd GetConfig(const double s) const
  {
    const double ratio = s / length_;
    return EigenHelpers::Interpolate(start_, end_, ratio);
  }

  Eigen::VectorXd GetTangent(const double s) const
  {
    UNUSED(s);
    return (end_ - start_) / length_;
  }

  Eigen::VectorXd GetCurvature(const double s) const
  {
    UNUSED(s);
    return Eigen::VectorXd::Zero(start_.size());
  }

  std::list<double> GetSwitchingPoints() const
  {
    return std::list<double>();
  }

  LinearPathSegment* Clone() const
  {
    return new LinearPathSegment(*this);
  }

private:

  Eigen::VectorXd start_;
  Eigen::VectorXd end_;
};

class CircularPathSegment : public PathSegment
{
public:

  CircularPathSegment(const Eigen::VectorXd& start,
                      const Eigen::VectorXd& intersection,
                      const Eigen::VectorXd& end,
                      double max_deviation)
  {
    if ((intersection - start).norm() < 0.000001
        || (end - intersection).norm() < 0.000001)
    {
      length_ = 0.0;
      radius_ = 1.0;
      center_ = intersection;
      x_ = Eigen::VectorXd::Zero(start.size());
      y_ = Eigen::VectorXd::Zero(start.size());
      return;
    }
    const Eigen::VectorXd start_direction = (intersection - start).normalized();
    const Eigen::VectorXd end_direction = (end - intersection).normalized();
    if ((start_direction - end_direction).norm() < 0.000001)
    {
      length_ = 0.0;
      radius_ = 1.0;
      center_ = intersection;
      x_ = Eigen::VectorXd::Zero(start.size());
      y_ = Eigen::VectorXd::Zero(start.size());
      return;
    }
    double distance = std::min((start - intersection).norm(),
                               (end - intersection).norm());
    const double angle = std::acos(start_direction.dot(end_direction));
    // Enforce max deviation limit
    const double deviation_limited_distance
        = max_deviation * std::sin(0.5 * angle) / (1.0 - std::cos(0.5 * angle));
    distance = std::min(distance, deviation_limited_distance);
    radius_ = distance / std::tan(0.5 * angle);
    length_ = angle * radius_;
    center_ = intersection + (end_direction - start_direction).normalized()
              * radius_ / std::cos(0.5 * angle);
    x_ = (intersection - distance * start_direction - center_).normalized();
    y_ = start_direction;
  }

  Eigen::VectorXd GetConfig(const double s) const
  {
    const double angle = s / radius_;
    return center_ + radius_ * (x_ * std::cos(angle) + y_ * std::sin(angle));
  }

  Eigen::VectorXd GetTangent(const double s) const
  {
    const double angle = s / radius_;
    return - x_ * std::sin(angle) + y_ * std::cos(angle);
  }

  Eigen::VectorXd GetCurvature(const double s) const
  {
    const double angle = s / radius_;
    return - 1.0 / radius_ * (x_ * std::cos(angle) + y_ * std::sin(angle));
  }

  std::list<double> GetSwitchingPoints() const
  {
    std::list<double> switching_points;
    const size_t dim = (size_t)x_.size();
    for (size_t i = 0; i < dim; i++)
    {
      double switching_angle = std::atan2(y_[(ssize_t)i], x_[(ssize_t)i]);
      if(switching_angle < 0.0)
      {
        switching_angle += M_PI;
      }
      const double switching_point = switching_angle * radius_;
      if(switching_point < length_)
      {
        switching_points.push_back(switching_point);
      }
    }
    switching_points.sort();
    return switching_points;
  }

  CircularPathSegment* Clone() const
  {
    return new CircularPathSegment(*this);
  }

private:

  double radius_;
  Eigen::VectorXd center_;
  Eigen::VectorXd x_;
  Eigen::VectorXd y_;
};

Path::Path(const std::list<Eigen::VectorXd>& path, const double max_deviation)
  : length_(0.0)
{
  if (path.size() < 2)
  {
    return;
  }
  std::list<Eigen::VectorXd>::const_iterator config1 = path.begin();
  std::list<Eigen::VectorXd>::const_iterator config2 = config1;
  config2++;
  std::list<Eigen::VectorXd>::const_iterator config3;
  Eigen::VectorXd start_config = *config1;
  while (config2 != path.end())
  {
    config3 = config2;
    config3++;
    if (max_deviation > 0.0 && config3 != path.end())
    {
      CircularPathSegment* blend_segment
          = new CircularPathSegment(0.5 * (*config1 + *config2),
                                    *config2, 0.5 * (*config2 + *config3),
                                    max_deviation);
      const Eigen::VectorXd end_config = blend_segment->GetConfig(0.0);
      if ((end_config - start_config).norm() > 0.000001)
      {
        path_segments_.push_back(
              new LinearPathSegment(start_config, end_config));
      }
      path_segments_.push_back(blend_segment);
      start_config = blend_segment->GetConfig(blend_segment->Length());
    }
    else
    {
      path_segments_.push_back(new LinearPathSegment(start_config, *config2));
      start_config = *config2;
    }
    config1 = config2;
    config2++;
  }
  // Create std::list of switching point candidates,
  // calculate total path length and absolute positions of path segments
  for (std::list<PathSegment*>::iterator segment = path_segments_.begin();
       segment != path_segments_.end(); segment++)
  {
    (*segment)->position = length_;
    std::list<double> local_switching_points
        = (*segment)->GetSwitchingPoints();
    for (std::list<double>::const_iterator point
         = local_switching_points.begin();
         point != local_switching_points.end(); point++)
    {
      switching_points_.push_back(std::make_pair(length_ + *point, false));
    }
    length_ += (*segment)->Length();
    while (!switching_points_.empty()
           && switching_points_.back().first >= length_)
    {
      switching_points_.pop_back();
    }
    switching_points_.push_back(std::make_pair(length_, true));
  }
  switching_points_.pop_back();
}

Path::Path(const Path& path)
  : length_(path.length_), switching_points_(path.switching_points_)
{
  for (std::list<PathSegment*>::const_iterator it = path.path_segments_.begin();
      it != path.path_segments_.end(); it++)
  {
    path_segments_.push_back((*it)->Clone());
  }
}

Path::~Path()
{
  for(std::list<PathSegment*>::iterator it = path_segments_.begin();
      it != path_segments_.end(); it++)
  {
    delete *it;
  }
}

double Path::Length() const { return length_; }

PathSegment* Path::GetPathSegment(double& s) const
{
  std::list<PathSegment*>::const_iterator it = path_segments_.begin();
  std::list<PathSegment*>::const_iterator next = it;
  next++;
  while (next != path_segments_.end() && s >= (*next)->position)
  {
    it = next;
    next++;
  }
  s -= (*it)->position;
  return *it;
}

Eigen::VectorXd Path::GetConfig(const double s) const
{
  double mutable_s = s;
  const PathSegment* path_segment = GetPathSegment(mutable_s);
  return path_segment->GetConfig(mutable_s);
}

Eigen::VectorXd Path::GetTangent(const double s) const
{
  double mutable_s = s;
  const PathSegment* path_segment = GetPathSegment(mutable_s);
  return path_segment->GetTangent(mutable_s);
}

Eigen::VectorXd Path::GetCurvature(double s) const
{
  double mutable_s = s;
  const PathSegment* path_segment = GetPathSegment(mutable_s);
  return path_segment->GetCurvature(mutable_s);
}

double Path::GetNextSwitchingPoint(const double s, bool& discontinuity) const
{
  std::list<std::pair<double, bool> >::const_iterator it
      = switching_points_.begin();
  while (it != switching_points_.end() && it->first <= s)
  {
    it++;
  }
  if (it == switching_points_.end())
  {
    discontinuity = true;
    return length_;
  }
  else
  {
    discontinuity = it->second;
    return it->first;
  }
}

std::list<std::pair<double, bool>> Path::SwitchingPoints() const
{
  return switching_points_;
}

const double Trajectory::eps = 0.000001;

Trajectory::Trajectory(const std::list<Eigen::VectorXd>& waypoints,
                       const Eigen::VectorXd& max_velocity,
                       const Eigen::VectorXd& max_acceleration,
                       const double max_deviation,
                       const double timestep)
  : path_(Path(waypoints, max_deviation)),
    max_velocity_(max_velocity),
    max_acceleration_(max_acceleration),
    n_((unsigned int)max_velocity.size()),
    time_step_(timestep),
    cached_time_(std::numeric_limits<double>::max())
{
  trajectory_.push_back(TrajectoryStep(0.0, 0.0));
  double after_acceleration = GetMinMaxPathAcceleration(0.0, 0.0, true);
  while (!IntegrateForward(trajectory_, after_acceleration))
  {
    double before_acceleration = 0.0;
    TrajectoryStep switching_point;
    if (GetNextSwitchingPoint(trajectory_.back().path_pos_,
                              switching_point,
                              before_acceleration,
                              after_acceleration))
    {
      break;
    }
    IntegrateBackward(trajectory_,
                      switching_point.path_pos_,
                      switching_point.path_vel_,
                      before_acceleration);
  }
  const double before_acceleration
      = GetMinMaxPathAcceleration(path_.Length(), 0.0, false);
  IntegrateBackward(trajectory_, path_.Length(), 0.0, before_acceleration);
  // calculate timing
  std::list<TrajectoryStep>::iterator previous = trajectory_.begin();
  std::list<TrajectoryStep>::iterator it = previous;
  it->time_ = 0.0;
  it++;
  while (it != trajectory_.end())
  {
    it->time_ = previous->time_ + (it->path_pos_ - previous->path_pos_)
                / ((it->path_vel_ + previous->path_vel_) / 2.0);
    previous = it;
    it++;
  }
}

Trajectory::Trajectory(const Path& path,
                       const Eigen::VectorXd& max_velocity,
                       const Eigen::VectorXd& max_acceleration,
                       double timestep)
  : path_(path),
    max_velocity_(max_velocity),
    max_acceleration_(max_acceleration),
    n_((unsigned int)max_velocity.size()),
    time_step_(timestep),
    cached_time_(std::numeric_limits<double>::max())
{
  trajectory_.push_back(TrajectoryStep(0.0, 0.0));
  double after_acceleration = GetMinMaxPathAcceleration(0.0, 0.0, true);
  while (!IntegrateForward(trajectory_, after_acceleration))
  {
    double before_acceleration = 0.0;
    TrajectoryStep switching_point;
    if (GetNextSwitchingPoint(trajectory_.back().path_pos_,
                              switching_point,
                              before_acceleration,
                              after_acceleration))
    {
      break;
    }
    IntegrateBackward(trajectory_,
                      switching_point.path_pos_,
                      switching_point.path_vel_,
                      before_acceleration);
  }
  const double before_acceleration
      = GetMinMaxPathAcceleration(path.Length(), 0.0, false);
  IntegrateBackward(trajectory_, path.Length(), 0.0, before_acceleration);
  // calculate timing
  std::list<TrajectoryStep>::iterator previous = trajectory_.begin();
  std::list<TrajectoryStep>::iterator it = previous;
  it->time_ = 0.0;
  it++;
  while (it != trajectory_.end())
  {
    it->time_ = previous->time_ + (it->path_pos_ - previous->path_pos_)
                / ((it->path_vel_ + previous->path_vel_) / 2.0);
    previous = it;
    it++;
  }
}

Trajectory::~Trajectory(void) {}

void Trajectory::OutputPhasePlaneTrajectory() const
{
  std::ofstream velocity_file("max_velocity.txt");
  const double step_size = path_.Length() / 100000.0;
  for (double s = 0.0; s < path_.Length(); s += step_size)
  {
    double max_velocity = GetAccelerationMaxPathVelocity(s);
    if (max_velocity == std::numeric_limits<double>::infinity())
    {
      max_velocity = 10.0;
    }
    velocity_file << s << "  " << max_velocity
                  << "  " << GetVelocityMaxPathVelocity(s) << std::endl;
  }
  velocity_file.close();
  std::ofstream trajectory_file("trajectory.txt");
  for (std::list<TrajectoryStep>::const_iterator it = trajectory_.begin();
       it != trajectory_.end(); it++)
  {
    trajectory_file << it->path_pos_ << "  " << it->path_vel_ << std::endl;
  }
  trajectory_file.close();
}

// Returns true if end of path is reached.
bool Trajectory::GetNextSwitchingPoint(const double path_pos,
                                       TrajectoryStep& next_switching_point,
                                       double& before_acceleration,
                                       double& after_acceleration)
{
  TrajectoryStep acceleration_switching_point(path_pos, 0.0);
  double acceleration_before_acceleration = 0.0;
  double acceleration_after_acceleration = 0.0;
  bool acceleration_reached_end = false;
  do
  {
    acceleration_reached_end
        = GetNextAccelerationSwitchingPoint(
            acceleration_switching_point.path_pos_,
            acceleration_switching_point,
            acceleration_before_acceleration,
            acceleration_after_acceleration);
  }
  while (!acceleration_reached_end
         && (acceleration_switching_point.path_vel_
             > GetVelocityMaxPathVelocity(
               acceleration_switching_point.path_pos_)));

  TrajectoryStep velocity_switching_point(path_pos, 0.0);
  double velocity_before_acceleration = 0.0;
  double velocity_after_acceleration = 0.0;
  bool velocity_reached_end = false;
  do
  {
    velocity_reached_end
        = GetNextVelocitySwitchingPoint(
            velocity_switching_point.path_pos_,
            velocity_switching_point,
            velocity_before_acceleration,
            velocity_after_acceleration);
  }
  while (!velocity_reached_end
         && (velocity_switching_point.path_pos_
             <= acceleration_switching_point.path_pos_)
         && (velocity_switching_point.path_vel_
             > GetAccelerationMaxPathVelocity(
               velocity_switching_point.path_pos_ - eps)
             || (velocity_switching_point.path_vel_
                 > GetAccelerationMaxPathVelocity(
                   velocity_switching_point.path_pos_ + eps))));

  if (acceleration_reached_end && velocity_reached_end)
  {
    return true;
  }
  else if (!acceleration_reached_end
           && (velocity_reached_end
               || (acceleration_switching_point.path_pos_
                   <= velocity_switching_point.path_pos_)))
  {
    next_switching_point = acceleration_switching_point;
    before_acceleration = acceleration_before_acceleration;
    after_acceleration = acceleration_after_acceleration;
    return false;
  }
  else
  {
    next_switching_point = velocity_switching_point;
    before_acceleration = velocity_before_acceleration;
    after_acceleration = velocity_after_acceleration;
    return false;
  }
}

bool Trajectory::GetNextAccelerationSwitchingPoint(
    const double path_pos,
    TrajectoryStep& next_switching_point,
    double& before_acceleration,
    double& after_acceleration)
{
  double switching_path_pos = path_pos;
  double switching_path_vel = 0.0;
  while (true)
  {
    bool discontinuity = false;
    switching_path_pos
        = path_.GetNextSwitchingPoint(switching_path_pos, discontinuity);
    if (switching_path_pos > path_.Length() - eps)
    {
      return true;
    }
    if(discontinuity)
    {
      const double before_path_vel
          = GetAccelerationMaxPathVelocity(switching_path_pos - eps);
      const double after_path_vel
          = GetAccelerationMaxPathVelocity(switching_path_pos + eps);
      switching_path_vel = std::min(before_path_vel, after_path_vel);
      before_acceleration
          = GetMinMaxPathAcceleration(
              switching_path_pos - eps, switching_path_vel, false);
      after_acceleration
          = GetMinMaxPathAcceleration(
              switching_path_pos + eps, switching_path_vel, true);
      if (((before_path_vel > after_path_vel)
           || (GetMinMaxPhaseSlope(
                 switching_path_pos - eps, switching_path_vel, false)
               > GetAccelerationMaxPathVelocityDeriv(
                   switching_path_pos - 2.0 * eps)))
          && ((before_path_vel < after_path_vel)
              || (GetMinMaxPhaseSlope(
                    switching_path_pos + eps, switching_path_vel, true)
                  < GetAccelerationMaxPathVelocityDeriv(
                      switching_path_pos + 2.0 * eps))))
      {
        break;
      }
    }
    else
    {
      switching_path_vel = GetAccelerationMaxPathVelocity(switching_path_pos);
      before_acceleration = 0.0;
      after_acceleration = 0.0;
      if (GetAccelerationMaxPathVelocityDeriv(
            switching_path_pos - eps) < 0.0
          && GetAccelerationMaxPathVelocityDeriv(
            switching_path_pos + eps) > 0.0)
      {
        break;
      }
    }
  }
  next_switching_point = TrajectoryStep(switching_path_pos, switching_path_vel);
  return false;
}

bool Trajectory::GetNextVelocitySwitchingPoint(
    double path_pos,
    TrajectoryStep& next_switching_point,
    double& before_acceleration,
    double& after_acceleration)
{
  const double stepsize = 0.001;
  const double accuracy = 0.000001;
  bool start = false;
  double switching_path_pos = path_pos - stepsize;
  do
  {
    switching_path_pos += stepsize;


    if (GetMinMaxPhaseSlope(switching_path_pos,
                            GetVelocityMaxPathVelocity(switching_path_pos),
                            false)
        >= GetVelocityMaxPathVelocityDeriv(switching_path_pos))
    {
      start = true;
    }
  }
  while ((!start
          || (GetMinMaxPhaseSlope(switching_path_pos,
                                  GetVelocityMaxPathVelocity(
                                    switching_path_pos), false)
              > GetVelocityMaxPathVelocityDeriv(switching_path_pos)))
         && switching_path_pos < path_.Length());
  if (switching_path_pos >= path_.Length())
  {
    return true; // end of trajectory reached
  }
  double before_path_pos = switching_path_pos - stepsize;
  double after_path_pos = switching_path_pos;
  while (after_path_pos - before_path_pos > accuracy)
  {
    switching_path_pos = (before_path_pos + after_path_pos) / 2.0;
    if (GetMinMaxPhaseSlope(switching_path_pos,
                            GetVelocityMaxPathVelocity(switching_path_pos),
                            false)
        > GetVelocityMaxPathVelocityDeriv(switching_path_pos))
    {
      before_path_pos = switching_path_pos;
    }
    else
    {
      after_path_pos = switching_path_pos;
    }
  }
  before_acceleration
      = GetMinMaxPathAcceleration(
          before_path_pos, GetVelocityMaxPathVelocity(before_path_pos), false);
  after_acceleration
      = GetMinMaxPathAcceleration(
          after_path_pos, GetVelocityMaxPathVelocity(after_path_pos), true);
  next_switching_point
      = TrajectoryStep(
          after_path_pos, GetVelocityMaxPathVelocity(after_path_pos));
  return false;
}

// returns true if end of path is reached
bool Trajectory::IntegrateForward(std::list<TrajectoryStep>& trajectory,
                                  const double acceleration)
{
  double mutable_acceleration = acceleration;
  double path_pos = trajectory.back().path_pos_;
  double path_vel = trajectory.back().path_vel_;
  std::list<std::pair<double, bool>> switching_points
      = path_.SwitchingPoints();
  std::list<std::pair<double, bool> >::iterator next_discontinuity
      = switching_points.begin();
  while (true)
  {
    while ((next_discontinuity != switching_points.end())
           && (next_discontinuity->first <= path_pos
               || !next_discontinuity->second))
    {
      next_discontinuity++;
    }
    double old_path_pos = path_pos;
    double old_path_vel = path_vel;
    path_vel += time_step_ * mutable_acceleration;
    path_pos += time_step_ * 0.5 * (old_path_vel + path_vel);
    if ((next_discontinuity != switching_points.end())
        && (path_pos > next_discontinuity->first))
    {
      path_vel = old_path_vel
                 + (next_discontinuity->first - old_path_pos)
                 * (path_vel - old_path_vel)
                 / (path_pos - old_path_pos);
      path_pos = next_discontinuity->first;
    }
    if (path_pos > path_.Length())
    {
      trajectory.push_back(TrajectoryStep(path_pos, path_vel));
      return true;
    }
    else if (path_vel < 0.0)
    {
      throw std::runtime_error(
            "Error while integrating forward: Negative path velocity");
    }
    if ((path_vel > GetVelocityMaxPathVelocity(path_pos))
        && (GetMinMaxPhaseSlope(
              old_path_pos, GetVelocityMaxPathVelocity(old_path_pos), false)
            <= GetVelocityMaxPathVelocityDeriv(old_path_pos)))
    {
      path_vel = GetVelocityMaxPathVelocity(path_pos);
    }
    trajectory.push_back(TrajectoryStep(path_pos, path_vel));
    mutable_acceleration = GetMinMaxPathAcceleration(path_pos, path_vel, true);
    if ((path_vel > GetAccelerationMaxPathVelocity(path_pos))
        || (path_vel > GetVelocityMaxPathVelocity(path_pos)))
    {
      // Find more accurate intersection with max-velocity curve using bisection
      TrajectoryStep overshoot = trajectory.back();
      trajectory.pop_back();
      double before = trajectory.back().path_pos_;
      double before_path_vel = trajectory.back().path_vel_;
      double after = overshoot.path_pos_;
      double after_path_vel = overshoot.path_vel_;
      while (after - before > eps)
      {
        const double midpoint = 0.5 * (before + after);
        double midpoint_path_vel = 0.5 * (before_path_vel + after_path_vel);
        if ((midpoint_path_vel > GetVelocityMaxPathVelocity(midpoint))
            && (GetMinMaxPhaseSlope(
                  before, GetVelocityMaxPathVelocity(before), false)
                <= GetVelocityMaxPathVelocityDeriv(before)))
        {
          midpoint_path_vel = GetVelocityMaxPathVelocity(midpoint);
        }
        if ((midpoint_path_vel > GetAccelerationMaxPathVelocity(midpoint))
            || (midpoint_path_vel > GetVelocityMaxPathVelocity(midpoint)))
        {
          after = midpoint;
          after_path_vel = midpoint_path_vel;
        }
        else
        {
          before = midpoint;
          before_path_vel = midpoint_path_vel;
        }
      }
      trajectory.push_back(TrajectoryStep(before, before_path_vel));
      if (GetAccelerationMaxPathVelocity(after)
          < GetVelocityMaxPathVelocity(after))
      {
        if (after > next_discontinuity->first)
        {
          return false;
        }
        else if (GetMinMaxPhaseSlope(trajectory.back().path_pos_,
                                     trajectory.back().path_vel_,
                                     true)
                 > GetAccelerationMaxPathVelocityDeriv(
                     trajectory.back().path_pos_))
        {
          return false;
        }
      }
      else
      {
        if (GetMinMaxPhaseSlope(
              trajectory.back().path_pos_, trajectory.back().path_vel_, false)
            > GetVelocityMaxPathVelocityDeriv(trajectory.back().path_pos_))
        {
          return false;
        }
      }
    }
  }
}

void Trajectory::IntegrateBackward(std::list<TrajectoryStep>& start_trajectory,
                                   double path_pos,
                                   double path_vel,
                                   double acceleration)
{
  double mutable_path_pos = path_pos;
  double mutable_path_vel = path_vel;
  double mutable_acceleration = acceleration;
  std::list<TrajectoryStep>::iterator start2 = start_trajectory.end();
  start2--;
  std::list<TrajectoryStep>::iterator start1 = start2;
  start1--;
  std::list<TrajectoryStep> trajectory;
  double slope = 0.0;
  if (start1->path_pos_ > mutable_path_pos)
  {
    throw std::invalid_argument("Invalid path position");
  }
  while (start1 != start_trajectory.begin() || mutable_path_pos >= 0.0)
  {
    if (start1->path_pos_ <= mutable_path_pos)
    {
      trajectory.push_front(TrajectoryStep(mutable_path_pos, mutable_path_vel));
      mutable_path_vel -= time_step_ * mutable_acceleration;
      mutable_path_pos -= time_step_
                          * 0.5
                          * (mutable_path_vel + trajectory.front().path_vel_);
      mutable_acceleration
          = GetMinMaxPathAcceleration(
              mutable_path_pos, mutable_path_vel, false);
      slope = (trajectory.front().path_vel_ - mutable_path_vel)
              / (trajectory.front().path_pos_ - mutable_path_pos);
      if (mutable_path_vel < 0.0)
      {
        throw std::runtime_error(
              "Error while integrating backward: Negative path velocity");
      }
    }
    else
    {
      start1--;
      start2--;
    }
    // Check for intersection between current start trajectory
    // and backward trajectory segments
    const double start_slope = (start2->path_vel_ - start1->path_vel_)
                               / (start2->path_pos_ - start1->path_pos_);
    const double intersection_path_pos
        = (start1->path_vel_ - mutable_path_vel + slope
           * mutable_path_pos - start_slope * start1->path_pos_)
          / (slope - start_slope);
    if ((std::max(start1->path_pos_, mutable_path_pos) - eps
         <= intersection_path_pos)
        && (intersection_path_pos
            <= eps + std::min(start2->path_pos_, trajectory.front().path_pos_)))
    {
      const double intersection_path_vel
          = start1->path_vel_ + start_slope
            * (intersection_path_pos - start1->path_pos_);
      start_trajectory.erase(start2, start_trajectory.end());
      start_trajectory.push_back(
            TrajectoryStep(intersection_path_pos, intersection_path_vel));
      start_trajectory.splice(start_trajectory.end(), trajectory);
      return;
    }
  }
  throw std::runtime_error(
        "Error while integrating backward: Did not hit start trajectory");
}

double Trajectory::GetMinMaxPathAcceleration(const double path_pos,
                                             const double path_vel,
                                             const bool max)
{
  Eigen::VectorXd config_deriv = path_.GetTangent(path_pos);
  Eigen::VectorXd config_deriv2 = path_.GetCurvature(path_pos);
  double factor = max ? 1.0 : -1.0;
  double max_path_accceleration = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < n_; i++)
  {
    if (config_deriv[i] != 0.0)
    {
      const double joint_max_path_acceleration
          = max_acceleration_[i] / std::abs(config_deriv[i])
            - factor * config_deriv2[i] * path_vel * path_vel / config_deriv[i];
      max_path_accceleration
          = std::min(max_path_accceleration, joint_max_path_acceleration);
    }
  }
  return factor * max_path_accceleration;
}

double Trajectory::GetMinMaxPhaseSlope(const double path_pos,
                                       const double path_vel,
                                       const bool max)
{
  return GetMinMaxPathAcceleration(path_pos, path_vel, max) / path_vel;
}

double Trajectory::GetAccelerationMaxPathVelocity(double path_pos) const
{
  double max_path_velocity = std::numeric_limits<double>::infinity();
  const Eigen::VectorXd config_deriv = path_.GetTangent(path_pos);
  const Eigen::VectorXd config_deriv2 = path_.GetCurvature(path_pos);
  for (unsigned int i = 0; i < n_; i++)
  {
    if (config_deriv[i] != 0.0)
    {
      for (unsigned int j = i + 1; j < n_; j++)
      {
        if (config_deriv[j] != 0.0)
        {
          double a_ij = config_deriv2[i] / config_deriv[i]
                        - config_deriv2[j] / config_deriv[j];
          if (a_ij != 0.0)
          {
            const double joint_path_velocity
                = std::sqrt((max_acceleration_[i] / std::abs(config_deriv[i])
                             + max_acceleration_[j] / std::abs(config_deriv[j]))
                            / std::abs(a_ij));
            max_path_velocity
                = std::min(max_path_velocity, joint_path_velocity);
          }
        }
      }
    }
    else if(config_deriv2[i] != 0.0)
    {
      max_path_velocity = std::min(max_path_velocity,
                                   std::sqrt(max_acceleration_[i]
                                             / std::abs(config_deriv2[i])));
    }
  }
  return max_path_velocity;
}


double Trajectory::GetVelocityMaxPathVelocity(double path_pos) const
{
  const Eigen::VectorXd tangent = path_.GetTangent(path_pos);
  double max_path_velocity = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < n_; i++)
  {
    max_path_velocity
        = std::min(max_path_velocity, max_velocity_[i] / std::abs(tangent[i]));
  }
  return max_path_velocity;
}

double Trajectory::GetAccelerationMaxPathVelocityDeriv(const double path_pos)
{
  return (GetAccelerationMaxPathVelocity(path_pos + eps)
          - GetAccelerationMaxPathVelocity(path_pos - eps)) / (2.0 * eps);
}

double Trajectory::GetVelocityMaxPathVelocityDeriv(const double path_pos)
{
  const Eigen::VectorXd tangent = path_.GetTangent(path_pos);
  double max_path_velocity = std::numeric_limits<double>::max();
  unsigned int active_constraint = 0;
  for (unsigned int i = 0; i < n_; i++)
  {
    const double this_max_path_velocity
        = max_velocity_[i] / std::abs(tangent[i]);
    if (this_max_path_velocity < max_path_velocity)
    {
      max_path_velocity = this_max_path_velocity;
      active_constraint = i;
    }
  }
  return -(max_velocity_[active_constraint]
           * path_.GetCurvature(path_pos)[active_constraint])
           / (tangent[active_constraint]
              * std::abs(tangent[active_constraint]));
}

double Trajectory::Duration() const
{
  return trajectory_.back().time_;
}

std::list<Trajectory::TrajectoryStep>::const_iterator
Trajectory::GetTrajectorySegment(double time) const
{
  if (time >= trajectory_.back().time_)
  {
    std::list<TrajectoryStep>::const_iterator last = trajectory_.end();
    last--;
    return last;
  }
  else
  {
    if(time < cached_time_)
    {
      cached_trajectory_segment_ = trajectory_.begin();
    }
    while (time >= cached_trajectory_segment_->time_)
    {
      cached_trajectory_segment_++;
    }
    cached_time_ = time;
    return cached_trajectory_segment_;
  }
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> Trajectory::GetPositionVelocity(
    const double time) const
{
  std::list<TrajectoryStep>::const_iterator it = GetTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;
  double timestep = it->time_ - previous->time_;
  const double acceleration
      = 2.0
        * (it->path_pos_ - previous->path_pos_ - timestep * previous->path_vel_)
        / (timestep * timestep);

  timestep = time - previous->time_;
  const double path_pos = previous->path_pos_
                          + timestep * previous->path_vel_
                          + 0.5 * timestep * timestep * acceleration;
  const double path_vel = previous->path_vel_ + timestep * acceleration;
  return std::make_pair(path_.GetConfig(path_pos),
                        path_.GetTangent(path_pos) * path_vel);
}
}
