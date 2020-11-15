#include <stdio.h>
#include <vector>
#include <map>
#include <random>
#include <Eigen/Geometry>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/simple_robot_model_interface.hpp>

#ifndef SIMPLE_SE3_ROBOT_MODEL_HPP
#define SIMPLE_SE3_ROBOT_MODEL_HPP

namespace simple_se3_robot_model
{
    using SimpleSE3Configuration = Eigen::Isometry3d;
    using SimpleSE3ConfigAlloc = Eigen::aligned_allocator<Eigen::Isometry3d>;

    class SimpleSE3ConfigSerializer
    {
    public:

        static inline std::string TypeName()
        {
            return std::string("EigenIsometry3dSerializer");
        }

        static inline uint64_t Serialize(const SimpleSE3Configuration& value, std::vector<uint8_t>& buffer)
        {
            return EigenHelpers::Serialize(value, buffer);
        }

        static inline std::pair<SimpleSE3Configuration, uint64_t> Deserialize(const std::vector<uint8_t>& buffer, const uint64_t current)
        {
            return EigenHelpers::Deserialize<Eigen::Isometry3d>(buffer, current);
        }
    };

    class SimpleSE3Robot : public simple_robot_model_interface::SimpleRobotModelInterface<SimpleSE3Configuration, SimpleSE3ConfigAlloc>
    {
    protected:

        SimpleSE3Configuration config_;
        double position_distance_weight_;
        double rotation_distance_weight_;
        std::string link_name_;

        inline void SetConfig(const Eigen::Isometry3d& new_config)
        {
            config_ = new_config;
        }

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SimpleSE3Robot(const SimpleSE3Configuration& initial_position,
                       const double position_distance_weight,
                       const double rotation_distance_weight,
                       const std::string& link_name)
            : simple_robot_model_interface::SimpleRobotModelInterface<SimpleSE3Configuration, SimpleSE3ConfigAlloc>(),
              position_distance_weight_(std::abs(position_distance_weight)),
              rotation_distance_weight_(std::abs(rotation_distance_weight)),
              link_name_(link_name)
        {
            SetPosition(initial_position);
        }

        virtual simple_robot_model_interface::SimpleRobotModelInterface<SimpleSE3Configuration, SimpleSE3ConfigAlloc>* Clone() const
        {
            return new SimpleSE3Robot(static_cast<const SimpleSE3Robot&>(*this));
        }

        inline const SimpleSE3Configuration& GetPosition() const
        {
            return config_;
        }

        inline const SimpleSE3Configuration& SetPosition(const SimpleSE3Configuration& position)
        {
            SetConfig(position);
            return GetPosition();
        }

        virtual std::vector<std::string> GetLinkNames() const
        {
            return std::vector<std::string>{link_name_};
        }

        inline Eigen::Isometry3d GetLinkTransform(const std::string& link_name) const
        {
            if (link_name == link_name_)
            {
                return GetPosition();
            }
            else
            {
                throw std::invalid_argument("Invalid link_name");
            }
        }

        virtual EigenHelpers::VectorIsometry3d GetLinkTransforms() const
        {
            return EigenHelpers::VectorIsometry3d(1, config_);
        }

        virtual EigenHelpers::MapStringIsometry3d GetLinkTransformsMap() const
        {
            EigenHelpers::MapStringIsometry3d link_transforms_map;
            link_transforms_map[link_name_] = config_;
            return link_transforms_map;
        }

        virtual Eigen::VectorXd ComputePerDimensionConfigurationSignedDistance(const SimpleSE3Configuration& config1, const SimpleSE3Configuration& config2) const
        {
            // This should change to use a 6dof twist instead
            const Eigen::Vector3d tc1 = config1.translation();
            const Eigen::Quaterniond qc1(config1.rotation());
            const Eigen::Vector3d tc2 = config2.translation();
            const Eigen::Quaterniond qc2(config2.rotation());
            Eigen::VectorXd dim_distances(4);
            dim_distances(0) = tc2.x() - tc1.x();
            dim_distances(1) = tc2.y() - tc1.y();
            dim_distances(2) = tc2.z() - tc1.z();
            dim_distances(3) = EigenHelpers::Distance(qc1, qc2);
            return dim_distances;
        }

        virtual double ComputeConfigurationDistance(const SimpleSE3Configuration& config1, const SimpleSE3Configuration& config2) const
        {
            const Eigen::Vector3d v1 = config1.translation();
            const Eigen::Quaterniond q1(config1.rotation());
            const Eigen::Vector3d v2 = config2.translation();
            const Eigen::Quaterniond q2(config2.rotation());
            const double vdist = EigenHelpers::Distance(v1, v2);
            const double qdist = EigenHelpers::Distance(q1, q2);
            return (vdist * position_distance_weight_) + (qdist * rotation_distance_weight_);
        }

        virtual Eigen::Isometry3d InterpolateBetweenConfigurations(const SimpleSE3Configuration& start, const SimpleSE3Configuration& end, const double ratio) const
        {
            return EigenHelpers::Interpolate(start, end, ratio);
        }

        virtual Eigen::Isometry3d AverageConfigurations(const std::vector<SimpleSE3Configuration, SimpleSE3ConfigAlloc>& configurations) const
        {
            if (configurations.size() > 0)
            {
                return EigenHelpers::AverageEigenIsometry3d(configurations);
            }
            else
            {
                return Eigen::Isometry3d::Identity();
            }
        }

        virtual Eigen::Matrix<double, 3, Eigen::Dynamic> ComputeLinkPointTranslationJacobian(const std::string& link_name, const Eigen::Vector4d& link_relative_point) const
        {
            if (link_name == link_name_)
            {
                const Eigen::Matrix3d rot_matrix = GetPosition().rotation();
                const Eigen::Matrix3d hatted_link_relative_point = EigenHelpers::Skew(link_relative_point.block<3, 1>(0, 0));
                Eigen::Matrix<double, 3, 6> body_velocity_jacobian = Eigen::Matrix<double, 3, 6>::Zero();
                body_velocity_jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                body_velocity_jacobian.block<3, 3>(0, 3) = -hatted_link_relative_point;
                #pragma GCC diagnostic push
                #pragma GCC diagnostic ignored "-Wconversion"
                const Eigen::Matrix<double, 3, 6> jacobian = rot_matrix * body_velocity_jacobian;
                #pragma GCC diagnostic pop
                return jacobian;
            }
            else
            {
                throw std::invalid_argument("Invalid link_name");
            }
        }
    };

}

#endif // SIMPLE_SE3_ROBOT_MODEL_HPP
