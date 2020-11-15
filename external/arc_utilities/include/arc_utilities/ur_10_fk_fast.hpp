#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <Eigen/Geometry>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_typedefs.hpp>

#ifndef UR_10_FK_FAST_HPP
#define UR_10_FK_FAST_HPP

namespace UR_10_FK_FAST
{
    const size_t UR_10_NUM_ACTIVE_JOINTS = 6;
    const size_t UR_10_NUM_LINKS = 7;

    const std::string UR_10_ACTIVE_JOINT_1_NAME = "shoulder_pan_joint";
    const std::string UR_10_ACTIVE_JOINT_2_NAME = "shoulder_lift_joint";
    const std::string UR_10_ACTIVE_JOINT_3_NAME = "elbow_joint";
    const std::string UR_10_ACTIVE_JOINT_4_NAME = "wrist_1_joint";
    const std::string UR_10_ACTIVE_JOINT_5_NAME = "wrist_2_joint";
    const std::string UR_10_ACTIVE_JOINT_6_NAME = "wrist_3_joint";

    const std::string UR_10_LINK_1_NAME = "base_link";
    const std::string UR_10_LINK_2_NAME = "shoulder_link";
    const std::string UR_10_LINK_3_NAME = "upper_arm_link";
    const std::string UR_10_LINK_4_NAME = "forearm_link";
    const std::string UR_10_LINK_5_NAME = "wrist_1_link";
    const std::string UR_10_LINK_6_NAME = "wrist_2_link";
    const std::string UR_10_LINK_7_NAME = "wrist_3_link";

    inline Eigen::Isometry3d Get_link_0_joint_1_LinkJointTransform(const double joint_val)
    {
        const Eigen::Translation3d pre_joint_translation(0.0, 0.0, 0.1273);
        const Eigen::Quaterniond pre_joint_rotation = EigenHelpers::QuaternionFromUrdfRPY(0.0, 0.0, 0.0);
        const Eigen::Isometry3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        const Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        const Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitZ()));
        const Eigen::Isometry3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Isometry3d Get_link_1_joint_2_LinkJointTransform(const double joint_val)
    {
        const Eigen::Translation3d pre_joint_translation(0.0, 0.220941, 0.0);
        const Eigen::Quaterniond pre_joint_rotation = EigenHelpers::QuaternionFromUrdfRPY(0.0, M_PI_2, 0.0);
        const Eigen::Isometry3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        const Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        const Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitY()));
        const Eigen::Isometry3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Isometry3d Get_link_2_joint_3_LinkJointTransform(const double joint_val)
    {
        const Eigen::Translation3d pre_joint_translation(0.0, -0.1719, 0.612);
        const Eigen::Quaterniond pre_joint_rotation = EigenHelpers::QuaternionFromUrdfRPY(0.0, 0.0, 0.0);
        const Eigen::Isometry3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        const Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        const Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitY()));
        const Eigen::Isometry3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Isometry3d Get_link_3_joint_4_LinkJointTransform(const double joint_val)
    {
        const Eigen::Translation3d pre_joint_translation(0.0, 0.0, 0.5723);
        const Eigen::Quaterniond pre_joint_rotation = EigenHelpers::QuaternionFromUrdfRPY(0.0, M_PI_2, 0.0);
        const Eigen::Isometry3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        const Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        const Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitY()));
        const Eigen::Isometry3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Isometry3d Get_link_4_joint_5_LinkJointTransform(const double joint_val)
    {
        const Eigen::Translation3d pre_joint_translation(0.0, 0.1149, 0.0);
        const Eigen::Quaterniond pre_joint_rotation = EigenHelpers::QuaternionFromUrdfRPY(0.0, 0.0, 0.0);
        const Eigen::Isometry3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        const Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        const Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitZ()));
        const Eigen::Isometry3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Isometry3d Get_link_5_joint_6_LinkJointTransform(const double joint_val)
    {
        const Eigen::Translation3d pre_joint_translation(0.0, 0.0, 0.1157);
        const Eigen::Quaterniond pre_joint_rotation = EigenHelpers::QuaternionFromUrdfRPY(0.0, 0.0, 0.0);
        const Eigen::Isometry3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        const Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        const Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitY()));
        const Eigen::Isometry3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline EigenHelpers::VectorIsometry3d GetLinkTransforms(const std::vector<double>& configuration, const Eigen::Isometry3d& base_transform=Eigen::Isometry3d::Identity())
    {
        assert(configuration.size() == UR_10_NUM_ACTIVE_JOINTS);
        EigenHelpers::VectorIsometry3d link_transforms(UR_10_NUM_LINKS);
        link_transforms[0] = base_transform;
        link_transforms[1] = link_transforms[0] * Get_link_0_joint_1_LinkJointTransform(configuration[0]);
        link_transforms[2] = link_transforms[1] * Get_link_1_joint_2_LinkJointTransform(configuration[1]);
        link_transforms[3] = link_transforms[2] * Get_link_2_joint_3_LinkJointTransform(configuration[2]);
        link_transforms[4] = link_transforms[3] * Get_link_3_joint_4_LinkJointTransform(configuration[3]);
        link_transforms[5] = link_transforms[4] * Get_link_4_joint_5_LinkJointTransform(configuration[4]);
        link_transforms[6] = link_transforms[5] * Get_link_5_joint_6_LinkJointTransform(configuration[5]);
        return link_transforms;
    }

    inline EigenHelpers::VectorIsometry3d GetLinkTransforms(const std::map<std::string, double>& configuration, const Eigen::Isometry3d& base_transform=Eigen::Isometry3d::Identity())
    {
        std::vector<double> configuration_vector(UR_10_NUM_ACTIVE_JOINTS);
        configuration_vector[0] = arc_helpers::RetrieveOrDefault(configuration, UR_10_ACTIVE_JOINT_1_NAME, 0.0);
        configuration_vector[1] = arc_helpers::RetrieveOrDefault(configuration, UR_10_ACTIVE_JOINT_2_NAME, 0.0);
        configuration_vector[2] = arc_helpers::RetrieveOrDefault(configuration, UR_10_ACTIVE_JOINT_3_NAME, 0.0);
        configuration_vector[3] = arc_helpers::RetrieveOrDefault(configuration, UR_10_ACTIVE_JOINT_4_NAME, 0.0);
        configuration_vector[4] = arc_helpers::RetrieveOrDefault(configuration, UR_10_ACTIVE_JOINT_5_NAME, 0.0);
        configuration_vector[5] = arc_helpers::RetrieveOrDefault(configuration, UR_10_ACTIVE_JOINT_6_NAME, 0.0);
        return GetLinkTransforms(configuration_vector, base_transform);
    }

    inline EigenHelpers::MapStringIsometry3d GetLinkTransformsMap(const std::vector<double>& configuration, const Eigen::Isometry3d& base_transform=Eigen::Isometry3d::Identity())
    {
        const EigenHelpers::VectorIsometry3d link_transforms = GetLinkTransforms(configuration, base_transform);
        EigenHelpers::MapStringIsometry3d link_transforms_map;
        link_transforms_map[UR_10_LINK_1_NAME] = link_transforms[0];
        link_transforms_map[UR_10_LINK_2_NAME] = link_transforms[1];
        link_transforms_map[UR_10_LINK_3_NAME] = link_transforms[2];
        link_transforms_map[UR_10_LINK_4_NAME] = link_transforms[3];
        link_transforms_map[UR_10_LINK_5_NAME] = link_transforms[4];
        link_transforms_map[UR_10_LINK_6_NAME] = link_transforms[5];
        link_transforms_map[UR_10_LINK_7_NAME] = link_transforms[6];
        return link_transforms_map;
    }

    inline EigenHelpers::MapStringIsometry3d GetLinkTransformsMap(const std::map<std::string, double>& configuration, const Eigen::Isometry3d& base_transform=Eigen::Isometry3d::Identity())
    {
        const EigenHelpers::VectorIsometry3d link_transforms = GetLinkTransforms(configuration, base_transform);
        EigenHelpers::MapStringIsometry3d link_transforms_map;
        link_transforms_map[UR_10_LINK_1_NAME] = link_transforms[0];
        link_transforms_map[UR_10_LINK_2_NAME] = link_transforms[1];
        link_transforms_map[UR_10_LINK_3_NAME] = link_transforms[2];
        link_transforms_map[UR_10_LINK_4_NAME] = link_transforms[3];
        link_transforms_map[UR_10_LINK_5_NAME] = link_transforms[4];
        link_transforms_map[UR_10_LINK_6_NAME] = link_transforms[5];
        link_transforms_map[UR_10_LINK_7_NAME] = link_transforms[6];
        return link_transforms_map;
    }
}

#endif // UR_10_FK_FAST_HPP
