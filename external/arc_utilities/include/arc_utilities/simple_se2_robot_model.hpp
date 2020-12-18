#include <stdio.h>
<<<<<<< HEAD

=======
#include <vector>
#include <map>
#include <random>
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
#include <Eigen/Geometry>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/simple_robot_model_interface.hpp>
<<<<<<< HEAD
#include <map>
#include <random>
#include <vector>
=======
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default

#ifndef SIMPLE_SE2_ROBOT_MODEL_HPP
#define SIMPLE_SE2_ROBOT_MODEL_HPP

<<<<<<< HEAD
namespace simple_se2_robot_model {
using SimpleSE2Configuration = Eigen::Matrix<double, 3, 1>;
using SimpleSE2ConfigAlloc = std::allocator<Eigen::Matrix<double, 3, 1>>;

class SimpleSE2ConfigSerializer {
 public:
  static inline std::string TypeName() { return std::string("EigenMatrixD31Serializer"); }

  static inline uint64_t Serialize(const SimpleSE2Configuration& value, std::vector<uint8_t>& buffer) {
    return EigenHelpers::Serialize(value, buffer);
  }

  static inline std::pair<SimpleSE2Configuration, uint64_t> Deserialize(const std::vector<uint8_t>& buffer,
                                                                        const uint64_t current) {
    return EigenHelpers::Deserialize<Eigen::Matrix<double, 3, 1>>(buffer, current);
  }
};

class SimpleSE2Robot
    : public simple_robot_model_interface::SimpleRobotModelInterface<SimpleSE2Configuration, SimpleSE2ConfigAlloc> {
 protected:
  Eigen::Isometry3d pose_;
  SimpleSE2Configuration config_;
  double position_distance_weight_;
  double rotation_distance_weight_;
  std::string link_name_;

  inline void SetConfig(const SimpleSE2Configuration& new_config) {
    config_(0) = new_config(0);
    config_(1) = new_config(1);
    config_(2) = EigenHelpers::EnforceContinuousRevoluteBounds(new_config(2));
    // Update pose
    pose_ = ComputePose();
  }

  inline Eigen::Isometry3d ComputePose() const {
    const SimpleSE2Configuration& current_config = GetPosition();
    const Eigen::Translation3d current_position(current_config(0), current_config(1), 0.0);
    const double current_z_angle = current_config(2);
    const Eigen::Isometry3d z_joint_transform = current_position * Eigen::Quaterniond::Identity();
    const Eigen::Isometry3d body_transform =
        z_joint_transform * (Eigen::Translation3d::Identity() *
                             Eigen::Quaterniond(Eigen::AngleAxisd(current_z_angle, Eigen::Vector3d::UnitZ())));
    return body_transform;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimpleSE2Robot(const SimpleSE2Configuration& initial_position, const double position_distance_weight,
                 const double rotation_distance_weight, const std::string& link_name)
      : simple_robot_model_interface::SimpleRobotModelInterface<SimpleSE2Configuration, SimpleSE2ConfigAlloc>(),
        position_distance_weight_(std::abs(position_distance_weight)),
        rotation_distance_weight_(std::abs(rotation_distance_weight)),
        link_name_(link_name) {
    SetPosition(initial_position);
  }

  virtual simple_robot_model_interface::SimpleRobotModelInterface<SimpleSE2Configuration, SimpleSE2ConfigAlloc>* Clone()
      const {
    return new SimpleSE2Robot(static_cast<const SimpleSE2Robot&>(*this));
  }

  virtual const SimpleSE2Configuration& GetPosition() const { return config_; }

  virtual const SimpleSE2Configuration& SetPosition(const SimpleSE2Configuration& position) {
    SetConfig(position);
    return GetPosition();
  }

  virtual std::vector<std::string> GetLinkNames() const { return std::vector<std::string>{link_name_}; }

  virtual Eigen::Isometry3d GetLinkTransform(const std::string& link_name) const {
    if (link_name == link_name_) {
      return pose_;
    } else {
      throw std::invalid_argument("Invalid link_name");
    }
  }

  virtual EigenHelpers::VectorIsometry3d GetLinkTransforms() const { return EigenHelpers::VectorIsometry3d(1, pose_); }

  virtual EigenHelpers::MapStringIsometry3d GetLinkTransformsMap() const {
    EigenHelpers::MapStringIsometry3d link_transforms_map;
    link_transforms_map[link_name_] = pose_;
    return link_transforms_map;
  }

  virtual Eigen::VectorXd ComputePerDimensionConfigurationSignedDistance(const SimpleSE2Configuration& config1,
                                                                         const SimpleSE2Configuration& config2) const {
    Eigen::VectorXd dim_distances(3);
    dim_distances(0) = config2(0) - config1(0);
    dim_distances(1) = config2(1) - config1(1);
    dim_distances(2) = EigenHelpers::ContinuousRevoluteSignedDistance(config1(2), config2(2));
    ;
    return dim_distances;
  }

  virtual double ComputeConfigurationDistance(const SimpleSE2Configuration& config1,
                                              const SimpleSE2Configuration& config2) const {
    const Eigen::VectorXd dim_distances = ComputePerDimensionConfigurationSignedDistance(config1, config2);
    const double trans_dist = sqrt((dim_distances(0) * dim_distances(0)) + (dim_distances(1) * dim_distances(1)));
    const double rots_dist = std::abs(dim_distances(2));
    return (trans_dist * position_distance_weight_) + (rots_dist * rotation_distance_weight_);
  }

  virtual Eigen::Matrix<double, 3, 1> InterpolateBetweenConfigurations(const SimpleSE2Configuration& start,
                                                                       const SimpleSE2Configuration& end,
                                                                       const double ratio) const {
    Eigen::Matrix<double, 3, 1> interpolated = Eigen::Matrix<double, 3, 1>::Zero();
    interpolated(0) = EigenHelpers::Interpolate(start(0), end(0), ratio);
    interpolated(1) = EigenHelpers::Interpolate(start(1), end(1), ratio);
    interpolated(2) = EigenHelpers::InterpolateContinuousRevolute(start(2), end(2), ratio);
    return interpolated;
  }

  virtual Eigen::Matrix<double, 3, 1> AverageConfigurations(
      const std::vector<SimpleSE2Configuration, SimpleSE2ConfigAlloc>& configurations) const {
    if (configurations.size() > 0) {
      // Separate translation and rotation values
      std::vector<Eigen::VectorXd> translations(configurations.size());
      std::vector<double> zrs(configurations.size());
      for (size_t idx = 0; idx < configurations.size(); idx++) {
        const SimpleSE2Configuration& state = configurations[idx];
        Eigen::VectorXd trans_state(2);
        trans_state << state(0), state(1);
        translations[idx] = trans_state;
        zrs[idx] = state(2);
      }
      // Get the average values
      const Eigen::VectorXd average_translation = EigenHelpers::AverageEigenVectorXd(translations);
      const double average_zr = EigenHelpers::AverageContinuousRevolute(zrs);
      Eigen::Matrix<double, 3, 1> average;
      average << average_translation, average_zr;
      return average;
    } else {
      return Eigen::Matrix<double, 3, 1>::Zero();
    }
  }

  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> ComputeLinkPointTranslationJacobian(
      const std::string& link_name, const Eigen::Vector4d& link_relative_point) const {
    if (link_name == link_name_) {
      const Eigen::Matrix<double, 3, 1> current_config = GetPosition();
      // Transform the point into world frame
      const Eigen::Isometry3d current_transform = pose_;
      const Eigen::Vector3d current_position(current_config(0), current_config(1), 0.0);
      const Eigen::Vector3d world_point = (current_transform * link_relative_point).block<3, 1>(0, 0);
      // Make the jacobian
      Eigen::Matrix<double, 3, 3> jacobian = Eigen::Matrix<double, 3, 3>::Zero();
      // Prismatic joints
      // X joint
      jacobian.block<3, 1>(0, 0) = jacobian.block<3, 1>(0, 0) + Eigen::Vector3d::UnitX();
      // Y joint
      jacobian.block<3, 1>(0, 1) = jacobian.block<3, 1>(0, 1) + Eigen::Vector3d::UnitY();
      // Rotatational joints
      // Compute Z-axis joint axis
      const Eigen::Isometry3d z_joint_transform = current_transform;
      const Eigen::Vector3d z_joint_axis = (Eigen::Vector3d)(z_joint_transform.rotation() * Eigen::Vector3d::UnitZ());
      jacobian.block<3, 1>(0, 2) = jacobian.block<3, 1>(0, 2) + z_joint_axis.cross(world_point - current_position);
      return jacobian;
    } else {
      throw std::invalid_argument("Invalid link_name");
    }
  }
};
}  // namespace simple_se2_robot_model

#endif  // SIMPLE_SE2_ROBOT_MODEL_HPP
=======
namespace simple_se2_robot_model
{
    using SimpleSE2Configuration = Eigen::Matrix<double, 3, 1>;
    using SimpleSE2ConfigAlloc = std::allocator<Eigen::Matrix<double, 3, 1>>;

    class SimpleSE2ConfigSerializer
    {
    public:

        static inline std::string TypeName()
        {
            return std::string("EigenMatrixD31Serializer");
        }

        static inline uint64_t Serialize(const SimpleSE2Configuration& value, std::vector<uint8_t>& buffer)
        {
            return EigenHelpers::Serialize(value, buffer);
        }

        static inline std::pair<SimpleSE2Configuration, uint64_t> Deserialize(const std::vector<uint8_t>& buffer, const uint64_t current)
        {
            return EigenHelpers::Deserialize<Eigen::Matrix<double, 3, 1>>(buffer, current);
        }
    };

    class SimpleSE2Robot : public simple_robot_model_interface::SimpleRobotModelInterface<SimpleSE2Configuration, SimpleSE2ConfigAlloc>
    {
    protected:

        Eigen::Isometry3d pose_;
        SimpleSE2Configuration config_;
        double position_distance_weight_;
        double rotation_distance_weight_;
        std::string link_name_;

        inline void SetConfig(const SimpleSE2Configuration& new_config)
        {
            config_(0) = new_config(0);
            config_(1) = new_config(1);
            config_(2) = EigenHelpers::EnforceContinuousRevoluteBounds(new_config(2));
            // Update pose
            pose_ = ComputePose();
        }

        inline Eigen::Isometry3d ComputePose() const
        {
            const SimpleSE2Configuration& current_config = GetPosition();
            const Eigen::Translation3d current_position(current_config(0), current_config(1), 0.0);
            const double current_z_angle = current_config(2);
            const Eigen::Isometry3d z_joint_transform = current_position * Eigen::Quaterniond::Identity();
            const Eigen::Isometry3d body_transform = z_joint_transform * (Eigen::Translation3d::Identity() * Eigen::Quaterniond(Eigen::AngleAxisd(current_z_angle, Eigen::Vector3d::UnitZ())));
            return body_transform;
        }


    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SimpleSE2Robot(const SimpleSE2Configuration& initial_position,
                       const double position_distance_weight,
                       const double rotation_distance_weight,
                       const std::string& link_name)
            : simple_robot_model_interface::SimpleRobotModelInterface<SimpleSE2Configuration, SimpleSE2ConfigAlloc>(),
              position_distance_weight_(std::abs(position_distance_weight)),
              rotation_distance_weight_(std::abs(rotation_distance_weight)),
              link_name_(link_name)
        {
            SetPosition(initial_position);
        }

        virtual simple_robot_model_interface::SimpleRobotModelInterface<SimpleSE2Configuration, SimpleSE2ConfigAlloc>* Clone() const
        {
            return new SimpleSE2Robot(static_cast<const SimpleSE2Robot&>(*this));
        }

        virtual const SimpleSE2Configuration& GetPosition() const
        {
            return config_;
        }

        virtual const SimpleSE2Configuration& SetPosition(const SimpleSE2Configuration& position)
        {
            SetConfig(position);
            return GetPosition();
        }

        virtual std::vector<std::string> GetLinkNames() const
        {
            return std::vector<std::string>{link_name_};
        }

        virtual Eigen::Isometry3d GetLinkTransform(const std::string& link_name) const
        {
            if (link_name == link_name_)
            {
                return pose_;
            }
            else
            {
                throw std::invalid_argument("Invalid link_name");
            }
        }

        virtual EigenHelpers::VectorIsometry3d GetLinkTransforms() const
        {
            return EigenHelpers::VectorIsometry3d(1, pose_);
        }

        virtual EigenHelpers::MapStringIsometry3d GetLinkTransformsMap() const
        {
            EigenHelpers::MapStringIsometry3d link_transforms_map;
            link_transforms_map[link_name_] = pose_;
            return link_transforms_map;
        }

        virtual Eigen::VectorXd ComputePerDimensionConfigurationSignedDistance(const SimpleSE2Configuration& config1, const SimpleSE2Configuration& config2) const
        {
            Eigen::VectorXd dim_distances(3);
            dim_distances(0) = config2(0) - config1(0);
            dim_distances(1) = config2(1) - config1(1);
            dim_distances(2) = EigenHelpers::ContinuousRevoluteSignedDistance(config1(2), config2(2));;
            return dim_distances;
        }

        virtual double ComputeConfigurationDistance(const SimpleSE2Configuration& config1, const SimpleSE2Configuration& config2) const
        {
            const Eigen::VectorXd dim_distances = ComputePerDimensionConfigurationSignedDistance(config1, config2);
            const double trans_dist = sqrt((dim_distances(0) * dim_distances(0)) + (dim_distances(1) * dim_distances(1)));
            const double rots_dist = std::abs(dim_distances(2));
            return (trans_dist * position_distance_weight_) + (rots_dist * rotation_distance_weight_);
        }

        virtual Eigen::Matrix<double, 3, 1> InterpolateBetweenConfigurations(const SimpleSE2Configuration& start, const SimpleSE2Configuration& end, const double ratio) const
        {
            Eigen::Matrix<double, 3, 1> interpolated = Eigen::Matrix<double, 3, 1>::Zero();
            interpolated(0) = EigenHelpers::Interpolate(start(0), end(0), ratio);
            interpolated(1) = EigenHelpers::Interpolate(start(1), end(1), ratio);
            interpolated(2) = EigenHelpers::InterpolateContinuousRevolute(start(2), end(2), ratio);
            return interpolated;
        }

        virtual Eigen::Matrix<double, 3, 1> AverageConfigurations(const std::vector<SimpleSE2Configuration, SimpleSE2ConfigAlloc>& configurations) const
        {
            if (configurations.size() > 0)
            {
                // Separate translation and rotation values
                std::vector<Eigen::VectorXd> translations(configurations.size());
                std::vector<double> zrs(configurations.size());
                for (size_t idx = 0; idx < configurations.size(); idx++)
                {
                    const SimpleSE2Configuration& state = configurations[idx];
                    Eigen::VectorXd trans_state(2);
                    trans_state << state(0), state(1);
                    translations[idx] = trans_state;
                    zrs[idx] = state(2);
                }
                // Get the average values
                const Eigen::VectorXd average_translation = EigenHelpers::AverageEigenVectorXd(translations);
                const double average_zr = EigenHelpers::AverageContinuousRevolute(zrs);
                Eigen::Matrix<double, 3, 1> average;
                average << average_translation, average_zr;
                return average;
            }
            else
            {
                return Eigen::Matrix<double, 3, 1>::Zero();
            }
        }

        virtual Eigen::Matrix<double, 3, Eigen::Dynamic> ComputeLinkPointTranslationJacobian(const std::string& link_name, const Eigen::Vector4d& link_relative_point) const
        {
            if (link_name == link_name_)
            {
                const Eigen::Matrix<double, 3, 1> current_config = GetPosition();
                // Transform the point into world frame
                const Eigen::Isometry3d current_transform = pose_;
                const Eigen::Vector3d current_position(current_config(0), current_config(1), 0.0);
                const Eigen::Vector3d world_point = (current_transform * link_relative_point).block<3, 1>(0, 0);
                // Make the jacobian
                Eigen::Matrix<double, 3, 3> jacobian = Eigen::Matrix<double, 3, 3>::Zero();
                // Prismatic joints
                // X joint
                jacobian.block<3,1>(0, 0) = jacobian.block<3,1>(0, 0) + Eigen::Vector3d::UnitX();
                // Y joint
                jacobian.block<3,1>(0, 1) = jacobian.block<3,1>(0, 1) + Eigen::Vector3d::UnitY();
                // Rotatational joints
                // Compute Z-axis joint axis
                const Eigen::Isometry3d z_joint_transform = current_transform;
                const Eigen::Vector3d z_joint_axis = (Eigen::Vector3d)(z_joint_transform.rotation() * Eigen::Vector3d::UnitZ());
                jacobian.block<3,1>(0, 2) = jacobian.block<3,1>(0, 2) + z_joint_axis.cross(world_point - current_position);
                return jacobian;
            }
            else
            {
                throw std::invalid_argument("Invalid link_name");
            }
        }
    };
}

#endif // SIMPLE_SE2_ROBOT_MODEL_HPP
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
