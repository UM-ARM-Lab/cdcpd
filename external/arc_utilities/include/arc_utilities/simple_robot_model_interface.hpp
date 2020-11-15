#include <stdio.h>
#include <vector>
#include <map>
#include <random>
#include <Eigen/Geometry>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>

#ifndef SIMPLE_ROBOT_MODEL_INTERFACE_HPP
#define SIMPLE_ROBOT_MODEL_INTERFACE_HPP

namespace simple_robot_model_interface
{
    template<typename Configuration, typename ConfigAlloc>
    class SimpleRobotModelInterface
    {
    public:

        virtual ~SimpleRobotModelInterface() {}

        virtual SimpleRobotModelInterface<Configuration, ConfigAlloc>* Clone() const = 0;

        virtual const Configuration& GetPosition() const = 0;

        virtual const Configuration& SetPosition(const Configuration& config) = 0;

        virtual std::vector<std::string> GetLinkNames() const = 0;

        virtual Eigen::Isometry3d GetLinkTransform(const std::string& link_name) const = 0;

        virtual EigenHelpers::VectorIsometry3d GetLinkTransforms() const = 0;

        virtual EigenHelpers::MapStringIsometry3d GetLinkTransformsMap() const = 0;

        virtual double ComputeConfigurationDistance(const Configuration& config1,
                                                    const Configuration& config2) const = 0;

        virtual Eigen::VectorXd ComputePerDimensionConfigurationDistance(const Configuration& config1,
                                                                         const Configuration& config2) const
        {
            return ComputePerDimensionConfigurationSignedDistance(config1, config2).cwiseAbs();
        }

        virtual Eigen::VectorXd ComputePerDimensionConfigurationSignedDistance(const Configuration& config1,
                                                                               const Configuration& config2) const = 0;

        virtual double ComputeConfigurationDistanceTo(const Configuration& config) const
        {
            return ComputeConfigurationDistance(GetPosition(), config);
        }

        virtual Eigen::VectorXd ComputePerDimensionConfigurationDistanceTo(const Configuration& config) const
        {
            return ComputePerDimensionConfigurationDistance(GetPosition(), config);
        }

        virtual Eigen::VectorXd ComputePerDimensionConfigurationSignedDistanceTo(const Configuration& config) const
        {
            return ComputePerDimensionConfigurationSignedDistance(GetPosition(), config);
        }

        virtual Configuration InterpolateBetweenConfigurations(const Configuration& start,
                                                               const Configuration& end,
                                                               const double ratio) const = 0;

        virtual Configuration AverageConfigurations(const std::vector<Configuration, ConfigAlloc>& configurations) const = 0;

        virtual Eigen::Matrix<double, 3, Eigen::Dynamic> ComputeLinkPointTranslationJacobian(const std::string& link_name,
                                                                                             const Eigen::Vector4d& link_relative_point) const = 0;
    };
}

#endif // SIMPLE_ROBOT_MODEL_INTERFACE_HPP
