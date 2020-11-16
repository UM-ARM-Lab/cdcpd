#include <stdio.h>
#include <vector>
#include <map>
#include <random>
#include <Eigen/Geometry>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/simple_se2_robot_model.hpp>
#include <arc_utilities/simple_se3_robot_model.hpp>
#include <arc_utilities/simple_linked_robot_model.hpp>
#include <arc_utilities/simple_robot_model_interface.hpp>

#ifndef SIMPLE_ROBOT_MODELS_HPP
#define SIMPLE_ROBOT_MODELS_HPP

namespace simple_robot_models
{
    class NullGeometry
    {
    public:

        NullGeometry() {}
    };

    class PointSphereGeometry
    {
    public:

        enum MODEL_GEOMETRY_TYPE { POINTS, SPHERES };

    protected:

        MODEL_GEOMETRY_TYPE geometry_type_;
        std::shared_ptr<EigenHelpers::VectorVector4d> geometry_;

    public:

        PointSphereGeometry(const MODEL_GEOMETRY_TYPE geometry_type,
                            const std::shared_ptr<EigenHelpers::VectorVector4d>& geometry)
            : geometry_type_(geometry_type),
              geometry_(geometry) {}

        PointSphereGeometry() : geometry_type_(POINTS), geometry_(new EigenHelpers::VectorVector4d()) {}

        const MODEL_GEOMETRY_TYPE& GeometryType() const { return geometry_type_; }

        const std::shared_ptr<EigenHelpers::VectorVector4d>& Geometry() const { return geometry_; }

        MODEL_GEOMETRY_TYPE& GeometryType() { return geometry_type_; }

        std::shared_ptr<EigenHelpers::VectorVector4d>& Geometry() { return geometry_; }
    };

    template<typename MeshType>
    class MeshGeometry
    {
    protected:

        Eigen::Isometry3d origin_transform_;
        Eigen::Vector3d scale_;
        MeshType mesh_;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        MeshGeometry(const Eigen::Isometry3d& origin_transform,
                     const Eigen::Vector3d& scale,
                     const MeshType& mesh)
            : origin_transform_(origin_transform),
              scale_(scale),
              mesh_(mesh) {}

        MeshGeometry()
            : origin_transform_(Eigen::Isometry3d::Identity()),
              scale_(Eigen::Vector3d(0.0, 0.0, 0.0)) {}

        const Eigen::Isometry3d& OriginTransform() const { return origin_transform_; }

        const Eigen::Vector3d& Scale() const { return scale_; }

        const MeshType& Mesh() const { return mesh_; }

        Eigen::Isometry3d& OriginTransform() { return origin_transform_; }

        Eigen::Vector3d& Scale() { return scale_; }

        MeshType& Mesh() { return mesh_; }
    };

    template<typename GeometryType>
    class RobotWithGeometry
    {
    public:

        virtual const std::vector<std::pair<std::string, GeometryType>>& GetLinkGeometries() const = 0;

        virtual bool CheckIfSelfCollisionAllowed(const size_t link1_index, const size_t link2_index) const = 0;
    };

    template<typename GeometryType>
    class SE2RobotWithGeometry : public simple_se2_robot_model::SimpleSE2Robot, RobotWithGeometry<GeometryType>
    {
    protected:

        std::vector<std::pair<std::string, GeometryType>> link_geometries_;

    public:

        SE2RobotWithGeometry(const simple_se2_robot_model::SimpleSE2Configuration& initial_position,
                             const double position_distance_weight,
                             const double rotation_distance_weight,
                             const std::string& link_name,
                             const GeometryType& geometry)
            : simple_se2_robot_model::SimpleSE2Robot(initial_position,
                                                     position_distance_weight,
                                                     rotation_distance_weight,
                                                     link_name),
              RobotWithGeometry<GeometryType>()
        {
            link_geometries_.push_back(std::make_pair(link_name, geometry));
        }

        virtual simple_robot_model_interface::SimpleRobotModelInterface<simple_se2_robot_model::SimpleSE2Configuration, simple_se2_robot_model::SimpleSE2ConfigAlloc>* Clone() const
        {
            return new SE2RobotWithGeometry<GeometryType>(static_cast<const SE2RobotWithGeometry<GeometryType>&>(*this));
        }

        inline const std::vector<std::pair<std::string, GeometryType>>& GetLinkGeometries() const
        {
            return link_geometries_;
        }

        inline bool CheckIfSelfCollisionAllowed(const size_t link1_index, const size_t link2_index) const
        {
            UNUSED(link1_index);
            UNUSED(link2_index);
            return true;
        }
    };

    template<typename GeometryType>
    class SE3RobotWithGeometry : public simple_se3_robot_model::SimpleSE3Robot, RobotWithGeometry<GeometryType>
    {
    protected:

        std::vector<std::pair<std::string, GeometryType>> link_geometries_;

    public:

        SE3RobotWithGeometry(const simple_se3_robot_model::SimpleSE3Configuration& initial_position,
                             const double position_distance_weight,
                             const double rotation_distance_weight,
                             const std::string& link_name,
                             const GeometryType& geometry)
            : simple_se3_robot_model::SimpleSE3Robot(initial_position,
                                                     position_distance_weight,
                                                     rotation_distance_weight,
                                                     link_name),
              RobotWithGeometry<GeometryType>()
        {
            link_geometries_.push_back(std::make_pair(link_name, geometry));
        }

        virtual simple_robot_model_interface::SimpleRobotModelInterface<simple_se3_robot_model::SimpleSE3Configuration, simple_se3_robot_model::SimpleSE3ConfigAlloc>* Clone() const
        {
            return new SE3RobotWithGeometry<GeometryType>(static_cast<const SE3RobotWithGeometry<GeometryType>&>(*this));
        }

        inline const std::vector<std::pair<std::string, GeometryType>>& GetLinkGeometries() const
        {
            return link_geometries_;
        }

        inline bool CheckIfSelfCollisionAllowed(const size_t link1_index, const size_t link2_index) const
        {
            UNUSED(link1_index);
            UNUSED(link2_index);
            return true;
        }
    };

    template<typename GeometryType>
    class LinkedRobotWithGeometry : public simple_linked_robot_model::SimpleLinkedRobot, RobotWithGeometry<GeometryType>
    {
    protected:

        std::vector<std::pair<std::string, GeometryType>> link_geometries_;
        Eigen::MatrixXi self_collision_map_;


    public:

        LinkedRobotWithGeometry(const Eigen::Isometry3d& base_transform,
                                const std::vector<simple_linked_robot_model::RobotLink>& links,
                                const std::vector<simple_linked_robot_model::RobotJoint>& joints,
                                const simple_linked_robot_model::SimpleLinkedConfiguration& initial_position,
                                const std::vector<double>& joint_distance_weights,
                                const std::vector<std::pair<std::string, GeometryType>>& link_geometries,
                                const std::vector<std::pair<size_t, size_t>>& allowed_self_collisions)
            : simple_linked_robot_model::SimpleLinkedRobot(base_transform, links, joints, initial_position, joint_distance_weights),
              RobotWithGeometry<GeometryType>()
        {
            if (links.size() == link_geometries.size())
            {
                for (size_t idx = 0; idx < links.size(); idx++)
                {
                    const std::string& link_name = links[idx].LinkName();
                    const std::string& link_geometry_name = link_geometries[idx].first;
                    if (link_name != link_geometry_name)
                    {
                        throw std::invalid_argument("Link geometry name does not match link name");
                    }
                }
                link_geometries_ = link_geometries;
                self_collision_map_ = GenerateAllowedSelfColllisionMap(links_.size(), allowed_self_collisions);
            }
            else
            {
                throw std::invalid_argument("Number of links does not match number of link geometries");
            }
        }

        virtual simple_robot_model_interface::SimpleRobotModelInterface<simple_linked_robot_model::SimpleLinkedConfiguration, simple_linked_robot_model::SimpleLinkedConfigAlloc>* Clone() const
        {
            return new LinkedRobotWithGeometry<GeometryType>(static_cast<const LinkedRobotWithGeometry<GeometryType>&>(*this));
        }

        inline const std::vector<std::pair<std::string, GeometryType>>& GetLinkGeometries() const
        {
            return link_geometries_;
        }

        inline bool CheckIfSelfCollisionAllowed(const size_t link1_index, const size_t link2_index) const
        {
            if (link1_index < links_.size() && link2_index < links_.size())
            {
                if (link1_index == link2_index)
                {
                    return true;
                }
                const int32_t stored = self_collision_map_((int64_t)link1_index, (int64_t)link2_index);
                if (stored > 0)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                throw std::invalid_argument("Query link indices ("
                                            + std::to_string(link1_index)
                                            + ", " + std::to_string(link2_index)
                                            + " out of range for robot with "
                                            + std::to_string(links_.size()) + " links");
            }
        }

        static inline Eigen::MatrixXi GenerateAllowedSelfColllisionMap(const size_t num_links, const std::vector<std::pair<size_t, size_t>>& allowed_self_collisions)
        {
            Eigen::MatrixXi allowed_self_collision_map = Eigen::MatrixXi::Identity((ssize_t)(num_links), (ssize_t)(num_links));
            for (size_t idx = 0; idx < allowed_self_collisions.size(); idx++)
            {
                const std::pair<size_t, size_t>& allowed_self_collision = allowed_self_collisions[idx];
                const int64_t first_link_index = (int64_t)allowed_self_collision.first;
                const int64_t second_link_index = (int64_t)allowed_self_collision.second;
                if ((first_link_index >= num_links) || (second_link_index >= num_links))
                {
                    throw std::invalid_argument("Allowed self-colllision index is not a valid link index");
                }
                // Insert it both ways
                allowed_self_collision_map(first_link_index, second_link_index) = 1;
                allowed_self_collision_map(second_link_index, first_link_index) = 1;
            }
            return allowed_self_collision_map;
        }
    };

    using PointSphereBasicSE2Robot = SE2RobotWithGeometry<PointSphereGeometry>;

    using PointSphereBasicSE3Robot = SE3RobotWithGeometry<PointSphereGeometry>;

    using PointSphereBasicLinkedRobot = LinkedRobotWithGeometry<PointSphereGeometry>;

    template<typename MeshType>
    using MeshBasicSE2Robot = SE2RobotWithGeometry<MeshGeometry<MeshType>>;

    template<typename MeshType>
    using MeshBasicSE3Robot = SE3RobotWithGeometry<MeshGeometry<MeshType>>;

    template<typename MeshType>
    using MeshBasicLinkedRobot = LinkedRobotWithGeometry<MeshGeometry<MeshType>>;

    using MeshFilenameBasicSE2Robot = MeshBasicSE2Robot<std::string>;

    using MeshFilenameBasicSE3Robot = MeshBasicSE3Robot<std::string>;

    using MeshFilenameBasicLinkedRobot = MeshBasicLinkedRobot<std::string>;
}

#endif // SIMPLE_ROBOT_MODELS_HPP
