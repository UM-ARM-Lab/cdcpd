#include <stdio.h>

#include <Eigen/Geometry>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_typedefs.hpp>
#include <arc_utilities/math_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/serialization.hpp>
#include <arc_utilities/simple_robot_model_interface.hpp>
#include <map>
#include <random>
#include <vector>

#ifndef SIMPLE_LINKED_ROBOT_MODEL_HPP
#define SIMPLE_LINKED_ROBOT_MODEL_HPP

namespace simple_linked_robot_model {
class SimpleJointModel {
 public:
  enum JOINT_TYPE : uint32_t { PRISMATIC = 4, REVOLUTE = 1, CONTINUOUS = 2, FIXED = 0 };

 protected:
  std::pair<double, double> limits_;
  double value_;
  JOINT_TYPE type_;

 public:
  static inline uint64_t Serialize(const SimpleJointModel& model, std::vector<uint8_t>& buffer) {
    return model.SerializeSelf(buffer);
  }

  static inline std::pair<SimpleJointModel, uint64_t> Deserialize(const std::vector<uint8_t>& buffer,
                                                                  const uint64_t current) {
    SimpleJointModel temp_model;
    const uint64_t bytes_read = temp_model.DeserializeSelf(buffer, current);
    return std::make_pair(temp_model, bytes_read);
  }

  inline SimpleJointModel(const std::pair<double, double>& limits, const double value, const JOINT_TYPE type) {
    type_ = type;
    if (IsContinuous()) {
      limits_ =
          std::pair<double, double>(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    } else {
      if (limits.first > limits.second) {
        throw std::invalid_argument("limits.first > limits.second");
      }
      limits_ = limits;
    }
    SetValue(value);
  }

  inline SimpleJointModel() {
    limits_ = std::pair<double, double>(0.0, 0.0);
    type_ = JOINT_TYPE::FIXED;
    SetValue(0.0);
  }

  inline uint64_t SerializeSelf(std::vector<uint8_t>& buffer) const {
    const uint64_t start_buffer_size = buffer.size();
    arc_utilities::SerializeFixedSizePOD<double>(limits_.first, buffer);
    arc_utilities::SerializeFixedSizePOD<double>(limits_.second, buffer);
    arc_utilities::SerializeFixedSizePOD<double>(value_, buffer);
    arc_utilities::SerializeFixedSizePOD<uint32_t>((uint32_t)type_, buffer);
    // Figure out how many bytes were written
    const uint64_t end_buffer_size = buffer.size();
    const uint64_t bytes_written = end_buffer_size - start_buffer_size;
    return bytes_written;
  }

  inline uint64_t DeserializeSelf(const std::vector<uint8_t>& buffer, const uint64_t current) {
    uint64_t current_position = current;
    const std::pair<double, uint64_t> deserialized_limits_first =
        arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
    limits_.first = deserialized_limits_first.first;
    current_position += deserialized_limits_first.second;
    const std::pair<double, uint64_t> deserialized_limits_second =
        arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
    limits_.second = deserialized_limits_second.first;
    current_position += deserialized_limits_second.second;
    const std::pair<double, uint64_t> deserialized_value =
        arc_utilities::DeserializeFixedSizePOD<double>(buffer, current_position);
    value_ = deserialized_value.first;
    current_position += deserialized_value.second;
    const std::pair<uint32_t, uint64_t> deserialized_type =
        arc_utilities::DeserializeFixedSizePOD<uint32_t>(buffer, current_position);
    type_ = (JOINT_TYPE)deserialized_type.first;
    current_position += deserialized_type.second;
    // Figure out how many bytes were read
    const uint64_t bytes_read = current_position - current;
    return bytes_read;
  }

  inline bool IsContinuous() const {
    if (type_ == JOINT_TYPE::CONTINUOUS) {
      return true;
    } else {
      return false;
    }
  }

  inline bool IsRevolute() const {
    if ((type_ == JOINT_TYPE::REVOLUTE) || (type_ == JOINT_TYPE::CONTINUOUS)) {
      return true;
    } else {
      return false;
    }
  }

  inline bool IsPrismatic() const {
    if (type_ == JOINT_TYPE::PRISMATIC) {
      return true;
    } else {
      return false;
    }
  }

  inline bool IsFixed() const {
    if (type_ == JOINT_TYPE::FIXED) {
      return true;
    } else {
      return false;
    }
  }

  inline JOINT_TYPE GetType() const { return type_; }

  inline bool IsSameType(const SimpleJointModel& other) const {
    if (type_ == other.GetType()) {
      return true;
    } else {
      return false;
    }
  }

  inline std::string GetTypeString() const {
    if (IsContinuous()) {
      return std::string("Continuous");
    } else if (IsRevolute()) {
      return std::string("Revolute");
    } else if (IsPrismatic()) {
      return std::string("Prismatic");
    } else if (IsFixed()) {
      return std::string("Fixed");
    } else {
      throw std::runtime_error("Invalid joint model type");
    }
  }

  inline double GetValue() const { return value_; }

  inline std::pair<double, double> GetLimits() const { return limits_; }

  inline bool InLimits(const double value) const {
    if (IsContinuous()) {
      return true;
    } else {
      if ((value < limits_.first) || (value > limits_.second)) {
        return false;
      } else {
        return true;
      }
    }
  }

  inline double EnforceLimits(const double value) const {
    if (!std::isnan(value) && !std::isinf(value)) {
      if (IsContinuous()) {
        return EigenHelpers::EnforceContinuousRevoluteBounds(value);
      } else {
        if (value < limits_.first) {
          return limits_.first;
        } else if (value > limits_.second) {
          return limits_.second;
        } else {
          return value;
        }
      }
    } else {
      throw std::invalid_argument("value is NAN or INF");
    }
  }

  inline void SetValue(const double value) {
    const double real_value = EnforceLimits(value);
    value_ = real_value;
  }

  inline double SignedDistance(const double v1, const double v2) const {
    if (IsContinuous()) {
      return EigenHelpers::ContinuousRevoluteSignedDistance(v1, v2);
    } else {
      return (v2 - v1);
    }
  }

  inline double SignedDistance(const double v) const { return SignedDistance(GetValue(), v); }

  inline double SignedDistance(const SimpleJointModel& other) const {
    if (IsSameType(other)) {
      return SignedDistance(GetValue(), other.GetValue());
    } else {
      throw std::invalid_argument("Cannot compute distance between joint models of different types");
    }
  }

  inline double Distance(const double v1, const double v2) const { return std::abs(SignedDistance(v1, v2)); }

  inline double Distance(const double v) const { return std::abs(SignedDistance(GetValue(), v)); }

  inline double Distance(const SimpleJointModel& other) const { return std::abs(SignedDistance(other)); }

  inline SimpleJointModel CopyWithNewValue(const double value) const { return SimpleJointModel(limits_, value, type_); }
};

inline std::ostream& operator<<(std::ostream& strm, const SimpleJointModel& joint_model) {
  const std::pair<double, double> limits = joint_model.GetLimits();
  strm << joint_model.GetValue() << "[" << limits.first << "," << limits.second << ")";
  return strm;
}

class SimpleLinkedConfigSerializer {
 public:
  static inline std::string TypeName() { return std::string("SimpleLinkedConfigurationSerializer"); }

  static inline uint64_t Serialize(const std::vector<SimpleJointModel>& value, std::vector<uint8_t>& buffer) {
    return arc_utilities::SerializeVector<SimpleJointModel>(value, buffer, SimpleJointModel::Serialize);
  }

  static inline std::pair<std::vector<SimpleJointModel>, uint64_t> Deserialize(const std::vector<uint8_t>& buffer,
                                                                               const uint64_t current) {
    return arc_utilities::DeserializeVector<SimpleJointModel>(buffer, current, SimpleJointModel::Deserialize);
  }
};

// Typedef to make our life a bit easier
using SimpleLinkedConfiguration = std::vector<SimpleJointModel>;
using SimpleLinkedConfigAlloc = std::allocator<SimpleLinkedConfiguration>;

class RobotJoint {
 protected:
  Eigen::Isometry3d joint_transform_;
  Eigen::Vector3d joint_axis_;
  int64_t parent_link_index_;
  int64_t child_link_index_;
  SimpleJointModel joint_model_;
  std::string joint_name_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotJoint(const Eigen::Isometry3d& joint_transform, const Eigen::Vector3d& joint_axis,
             const int64_t parent_link_index, const int64_t child_link_index, const SimpleJointModel& joint_model,
             const std::string& joint_name)
      : joint_transform_(joint_transform),
        joint_axis_(joint_axis),
        parent_link_index_(parent_link_index),
        child_link_index_(child_link_index),
        joint_model_(joint_model),
        joint_name_(joint_name) {}

  RobotJoint() : parent_link_index_(-1), child_link_index_(-1) {}

  const Eigen::Isometry3d& JointTransform() const { return joint_transform_; }

  const Eigen::Vector3d& JointAxis() const { return joint_axis_; }

  const int64_t& ParentLinkIndex() const { return parent_link_index_; }

  const int64_t& ChildLinkIndex() const { return child_link_index_; }

  const SimpleJointModel& JointModel() const { return joint_model_; }

  const std::string& JointName() const { return joint_name_; }

  Eigen::Isometry3d& JointTransform() { return joint_transform_; }

  Eigen::Vector3d& JointAxis() { return joint_axis_; }

  int64_t& ParentLinkIndex() { return parent_link_index_; }

  int64_t& ChildLinkIndex() { return child_link_index_; }

  SimpleJointModel& JointModel() { return joint_model_; }

  std::string& JointName() { return joint_name_; }
};

class RobotLink {
 protected:
  Eigen::Isometry3d link_transform_;
  std::string link_name_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotLink() {}

  RobotLink(const std::string& link_name) : link_name_(link_name) {}

  const Eigen::Isometry3d& LinkTransform() const { return link_transform_; }

  const std::string& LinkName() const { return link_name_; }

  std::string& LinkName() { return link_name_; }

  void UpdateLinkTransform(const Eigen::Isometry3d& transform) { link_transform_ = transform; }
};

class SimpleLinkedRobot : public simple_robot_model_interface::SimpleRobotModelInterface<SimpleLinkedConfiguration,
                                                                                         SimpleLinkedConfigAlloc> {
 protected:
  struct KinematicSkeletonElement {
    int64_t parent_link_index_;
    int64_t parent_joint_index_;
    std::vector<int64_t> child_link_indices_;
    std::vector<int64_t> child_joint_indices_;

    KinematicSkeletonElement() : parent_link_index_(-1), parent_joint_index_(-1) {}
  };

  SimpleLinkedConfiguration config_;
  Eigen::Isometry3d base_transform_;
  Eigen::Isometry3d inverse_base_transform_;
  std::vector<RobotLink> links_;
  std::vector<RobotJoint> joints_;
  std::vector<KinematicSkeletonElement> kinematic_skeleton_;
  std::map<int64_t, int64_t> active_joint_index_map_;
  std::vector<double> joint_distance_weights_;
  size_t num_active_joints_;

  inline void UpdateTransforms() {
    // Update the transform for the first link
    links_[0].UpdateLinkTransform(base_transform_);
    // Go out the kinematic chain
    for (size_t idx = 0; idx < joints_.size(); idx++) {
      // Get the current joint
      const RobotJoint& current_joint = joints_[idx];
      // Get the parent link
      const RobotLink& parent_link = links_[(size_t)current_joint.ParentLinkIndex()];
      // Get the child link
      RobotLink& child_link = links_[(size_t)current_joint.ChildLinkIndex()];
      // Get the parent_link transform
      const Eigen::Isometry3d parent_transform = parent_link.LinkTransform();
      // Get the parent_link->joint transform
      const Eigen::Isometry3d parent_to_joint_transform = current_joint.JointTransform();
      // Compute the base->joint_transform
      const Eigen::Isometry3d to_joint_transform = parent_transform * parent_to_joint_transform;
      // Compute the joint transform
      if (current_joint.JointModel().IsRevolute()) {
        const Eigen::Isometry3d joint_transform =
            Eigen::Translation3d(0.0, 0.0, 0.0) *
            Eigen::Quaterniond(Eigen::AngleAxisd(current_joint.JointModel().GetValue(), current_joint.JointAxis()));
        const Eigen::Isometry3d child_transform = to_joint_transform * joint_transform;
        child_link.UpdateLinkTransform(child_transform);
      } else if (current_joint.JointModel().IsPrismatic()) {
        const Eigen::Translation3d joint_translation =
            (Eigen::Translation3d)(current_joint.JointAxis() * current_joint.JointModel().GetValue());
        const Eigen::Isometry3d joint_transform = joint_translation * Eigen::Quaterniond::Identity();
        const Eigen::Isometry3d child_transform = to_joint_transform * joint_transform;
        child_link.UpdateLinkTransform(child_transform);
      } else {
        // Joint is fixed
        child_link.UpdateLinkTransform(to_joint_transform);
      }
    }
  }

  inline void SetConfig(const SimpleLinkedConfiguration& new_config) {
    if (new_config.size() == num_active_joints_) {
      config_.clear();
      size_t config_idx = 0u;
      for (size_t idx = 0; idx < joints_.size(); idx++) {
        RobotJoint& current_joint = joints_[idx];
        // Skip fixed joints
        if (current_joint.JointModel().IsFixed()) {
          continue;
        } else {
          if (config_idx < new_config.size()) {
            const SimpleJointModel& new_joint = new_config[config_idx];
            current_joint.JointModel().SetValue(new_joint.GetValue());
            config_.push_back(current_joint.JointModel());
            config_idx++;
          } else {
            throw std::runtime_error("config_idx out of range");
          }
        }
      }
      // Update forward kinematics
      UpdateTransforms();
    } else {
      throw std::invalid_argument("new_config.size() != num_active_joints_");
    }
  }

  static inline std::pair<size_t, std::map<int64_t, int64_t>> MakeActiveJointIndexMap(
      const std::vector<RobotJoint>& joints) {
    size_t num_active_joints = 0;
    std::map<int64_t, int64_t> active_joint_index_map;
    for (size_t idx = 0; idx < joints.size(); idx++) {
      const RobotJoint& current_joint = joints[idx];
      // Skip fixed joints
      if (!(current_joint.JointModel().IsFixed())) {
        active_joint_index_map[(int64_t)idx] = (int64_t)num_active_joints;
        num_active_joints++;
      } else {
        active_joint_index_map[(int64_t)idx] = -1;
      }
    }
    return std::make_pair(num_active_joints, active_joint_index_map);
  }

  static bool ExploreChildren(const std::vector<KinematicSkeletonElement>& kinematic_skeleton,
                              const int64_t current_index, std::map<int64_t, uint8_t>& cycle_detection_map) {
    // Check if we've been here before
    cycle_detection_map[current_index]++;
    const uint8_t check_val = cycle_detection_map[current_index];
    if (check_val != 0x01) {
      std::cerr << "Invalid kinematic structure, link " << current_index << " is part of a cycle" << std::endl;
      return true;
    } else {
      const KinematicSkeletonElement& current_element = kinematic_skeleton[current_index];
      const std::vector<int64_t>& current_child_indices = current_element.child_link_indices_;
      for (size_t idx = 0; idx < current_child_indices.size(); idx++) {
        // Get the current child index
        const int64_t current_child_index = current_child_indices[idx];
        const bool child_cycle_detected = ExploreChildren(kinematic_skeleton, current_child_index, cycle_detection_map);
        if (child_cycle_detected) {
          return true;
        }
      }
      return false;
    }
  }

  static bool CheckKinematicSkeleton(const std::vector<KinematicSkeletonElement>& kinematic_skeleton) {
    // Step through each state in the tree. Make sure that the linkage to the parent and child states are correct
    for (size_t current_index = 0; current_index < kinematic_skeleton.size(); current_index++) {
      // For every element, make sure all the parent<->child linkages are valid
      const KinematicSkeletonElement& current_element = kinematic_skeleton[current_index];
      // Check the linkage to the parent state
      const int64_t parent_index = current_element.parent_link_index_;
      if ((parent_index >= 0) && (parent_index < (int64_t)kinematic_skeleton.size())) {
        if (parent_index != (int64_t)current_index) {
          const KinematicSkeletonElement& parent_element = kinematic_skeleton[parent_index];
          // Make sure the corresponding parent contains the current node in the list of child indices
          const std::vector<int64_t>& parent_child_indices = parent_element.child_link_indices_;
          auto index_found =
              std::find(parent_child_indices.begin(), parent_child_indices.end(), (int64_t)current_index);
          if (index_found == parent_child_indices.end()) {
            std::cerr << "Parent element " << parent_index << " does not contain child index for current element "
                      << current_index << std::endl;
            return false;
          }
        } else {
          std::cerr << "Invalid parent index " << parent_index << " for element " << current_index
                    << " [Indices can't be the same]" << std::endl;
          return false;
        }
      } else if (parent_index < -1) {
        std::cerr << "Invalid parent index " << parent_index << " for element " << current_index << std::endl;
        return false;
      }
      // Check the linkage to the child states
      const std::vector<int64_t>& current_child_indices = current_element.child_link_indices_;
      for (size_t idx = 0; idx < current_child_indices.size(); idx++) {
        // Get the current child index
        const int64_t current_child_index = current_child_indices[idx];
        if ((current_child_index > 0) && (current_child_index < (int64_t)kinematic_skeleton.size())) {
          if (current_child_index != (int64_t)current_index) {
            const KinematicSkeletonElement& child_element = kinematic_skeleton[current_child_index];
            // Make sure the child node points to us as the parent index
            const int64_t child_parent_index = child_element.parent_link_index_;
            if (child_parent_index != (int64_t)current_index) {
              std::cerr << "Parent index " << child_parent_index << " for current child element " << current_child_index
                        << " does not match index " << current_index << " for current element " << std::endl;
              return false;
            }
          } else {
            std::cerr << "Invalid child index " << current_child_index << " for element " << current_index
                      << " [Indices can't be the same]" << std::endl;
            return false;
          }
        } else {
          std::cerr << "Invalid child index " << current_child_index << " for element " << current_index << std::endl;
          return false;
        }
      }
    }
    // Now that we know all the linkage connections are valid, we need to make sure it is connected and acyclic
    std::map<int64_t, uint8_t> cycle_detection_map;
    const bool cycle_detected = ExploreChildren(kinematic_skeleton, 0, cycle_detection_map);
    if (cycle_detected) {
      return false;
    }
    // Make sure we encountered every link ONCE
    for (size_t link_idx = 0; link_idx < kinematic_skeleton.size(); link_idx++) {
      const uint8_t check_val = arc_helpers::RetrieveOrDefault(cycle_detection_map, (int64_t)link_idx, (uint8_t)0x00);
      if (check_val != 0x01) {
        std::cerr << "Invalid kinematic structure, link " << link_idx << " not reachable from the root" << std::endl;
        return false;
      }
    }
    return true;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static inline std::pair<std::vector<KinematicSkeletonElement>, bool> SanityCheckRobotModel(
      const std::vector<RobotLink>& links, const std::vector<RobotJoint>& joints) {
    // Make sure we have enough links and joints
    if (links.size() != (joints.size() + 1)) {
      std::cerr << links.size() << " links are not enough for " << joints.size() << " joints" << std::endl;
      return std::make_pair(std::vector<KinematicSkeletonElement>(), false);
    }
    // Make sure the links all have unique names
    std::map<std::string, uint32_t> link_name_check_map;
    for (size_t idx = 0; idx < links.size(); idx++) {
      const std::string& link_name = links[idx].LinkName();
      link_name_check_map[link_name]++;
      if (link_name_check_map[link_name] > 1) {
        std::cerr << "Link " << link_name << " is not unique" << std::endl;
        return std::make_pair(std::vector<KinematicSkeletonElement>(), false);
      }
    }
    // Make sure the joints all have unique names
    std::map<std::string, uint32_t> joint_name_check_map;
    for (size_t idx = 0; idx < joints.size(); idx++) {
      const std::string& joint_name = joints[idx].JointName();
      joint_name_check_map[joint_name]++;
      if (joint_name_check_map[joint_name] > 1) {
        std::cerr << "Joint " << joint_name << " is not unique" << std::endl;
        return std::make_pair(std::vector<KinematicSkeletonElement>(), false);
      }
    }
    // Make the kinematic skeleton structure for checking
    std::vector<KinematicSkeletonElement> kinematic_skeleton(links.size(), KinematicSkeletonElement());
    for (size_t joint_idx = 0; joint_idx < joints.size(); joint_idx++) {
      // Get the current joint
      const RobotJoint& current_joint = joints[joint_idx];
      // Check the joint axes
      const double joint_axis_norm = current_joint.JointAxis().norm();
      const double error = std::abs(joint_axis_norm - 1.0);
      if (error > std::numeric_limits<double>::epsilon()) {
        std::cerr << "Joint axis is not a unit vector" << std::endl;
        return std::make_pair(std::vector<KinematicSkeletonElement>(), false);
      }
      // Add the edge into the kinematic skeleton
      const int64_t parent_link_index = current_joint.ParentLinkIndex();
      const int64_t child_link_index = current_joint.ChildLinkIndex();
      // Set the child->parent relationship
      kinematic_skeleton[(size_t)child_link_index].parent_link_index_ = parent_link_index;
      kinematic_skeleton[(size_t)child_link_index].parent_joint_index_ = joint_idx;
      // Set the parent->child relationship
      kinematic_skeleton[(size_t)parent_link_index].child_link_indices_.push_back(child_link_index);
      kinematic_skeleton[(size_t)parent_link_index].child_joint_indices_.push_back(joint_idx);
    }
    // Make sure the first link is the root
    const KinematicSkeletonElement& root_element = kinematic_skeleton[0];
    if (root_element.parent_joint_index_ != -1 || root_element.parent_link_index_ != -1) {
      std::cerr << "Invalid kinematic structure, first link is not the root" << std::endl;
      return std::make_pair(std::vector<KinematicSkeletonElement>(), false);
    }
    const bool valid_kinematics = CheckKinematicSkeleton(kinematic_skeleton);
    return std::make_pair(kinematic_skeleton, valid_kinematics);
  }

  SimpleLinkedRobot(const Eigen::Isometry3d& base_transform, const std::vector<RobotLink>& links,
                    const std::vector<RobotJoint>& joints, const SimpleLinkedConfiguration& initial_position,
                    const std::vector<double>& joint_distance_weights)
      : simple_robot_model_interface::SimpleRobotModelInterface<SimpleLinkedConfiguration, SimpleLinkedConfigAlloc>() {
    base_transform_ = base_transform;
    inverse_base_transform_ = base_transform_.inverse();
    // We take a list of robot links and a list of robot joints, but we have to sanity-check them first
    links_ = links;
    joints_ = joints;
    const auto model_validity_check = SanityCheckRobotModel(links_, joints_);
    if (!model_validity_check.second) {
      throw std::invalid_argument("Attempted to construct a SimpleLinkedRobot with an invalid robot model");
    }
    kinematic_skeleton_ = model_validity_check.first;
    // Get information about the active joints
    const auto active_joint_query = MakeActiveJointIndexMap(joints_);
    num_active_joints_ = active_joint_query.first;
    active_joint_index_map_ = active_joint_query.second;
    if (joint_distance_weights.size() != num_active_joints_) {
      throw std::invalid_argument("joint_distance_weights.size() != num_active_joints_");
    }
    joint_distance_weights_ = EigenHelpers::Abs(joint_distance_weights);
    // Generate the self colllision map
    SetPosition(initial_position);
  }

  virtual simple_robot_model_interface::SimpleRobotModelInterface<SimpleLinkedConfiguration, SimpleLinkedConfigAlloc>*
  Clone() const {
    return new SimpleLinkedRobot(static_cast<const SimpleLinkedRobot&>(*this));
  }

  inline std::vector<std::string> GetActiveJointNames() const {
    std::vector<std::string> active_joint_names;
    active_joint_names.reserve(num_active_joints_);
    for (size_t idx = 0; idx < joints_.size(); idx++) {
      const RobotJoint& current_joint = joints_[idx];
      // Skip fixed joints
      if (!(current_joint.JointModel().IsFixed())) {
        active_joint_names.push_back(current_joint.JointName());
      }
    }
    active_joint_names.shrink_to_fit();
    if (active_joint_names.size() == num_active_joints_) {
      return active_joint_names;
    } else {
      throw std::runtime_error("Number of active joint names goes not match number of active joints");
    }
  }

  inline void UpdateBaseTransform(const Eigen::Isometry3d& base_transform) {
    base_transform_ = base_transform;
    UpdateTransforms();
  }

  inline Eigen::Isometry3d GetBaseTransform() const { return base_transform_; }

  inline Eigen::Isometry3d GetInverseBaseTransform() const { return inverse_base_transform_; }

  virtual const SimpleLinkedConfiguration& GetPosition() const { return config_; }

  virtual const SimpleLinkedConfiguration& SetPosition(const SimpleLinkedConfiguration& position) {
    SetConfig(position);
    return GetPosition();
  }

  virtual std::vector<std::string> GetLinkNames() const {
    std::vector<std::string> link_names(links_.size());
    for (size_t idx = 0; idx < links_.size(); idx++) {
      const RobotLink& current_link = links_[idx];
      link_names[idx] = current_link.LinkName();
    }
    return link_names;
  }

  inline Eigen::Isometry3d GetLinkTransform(const std::string& link_name) const {
    for (size_t idx = 0; idx < links_.size(); idx++) {
      const RobotLink& current_link = links_[idx];
      if (current_link.LinkName() == link_name) {
        return current_link.LinkTransform();
      }
    }
    throw std::invalid_argument("Invalid link_name");
  }

  virtual EigenHelpers::VectorIsometry3d GetLinkTransforms() const {
    EigenHelpers::VectorIsometry3d link_transforms(links_.size());
    for (size_t idx = 0; idx < links_.size(); idx++) {
      const RobotLink& current_link = links_[idx];
      link_transforms[idx] = current_link.LinkTransform();
    }
    return link_transforms;
  }

  virtual EigenHelpers::MapStringIsometry3d GetLinkTransformsMap() const {
    EigenHelpers::MapStringIsometry3d link_transforms_map;
    for (size_t idx = 0; idx < links_.size(); idx++) {
      const RobotLink& current_link = links_[idx];
      link_transforms_map[current_link.LinkName()] = current_link.LinkTransform();
    }
    return link_transforms_map;
  }

  static inline Eigen::VectorXd ComputeUnweightedPerDimensionConfigurationRawDistance(
      const SimpleLinkedConfiguration& config1, const SimpleLinkedConfiguration& config2) {
    if (config1.size() == config2.size()) {
      Eigen::VectorXd distances = Eigen::VectorXd::Zero((ssize_t)(config1.size()));
      for (size_t idx = 0; idx < config1.size(); idx++) {
        const SimpleJointModel& j1 = config1[idx];
        const SimpleJointModel& j2 = config2[idx];
        distances((int64_t)idx) = j1.SignedDistance(j2);
      }
      return distances;
    } else {
      throw std::invalid_argument("config1.size() != config2.size()");
    }
  }

  virtual Eigen::VectorXd ComputePerDimensionConfigurationSignedDistance(
      const SimpleLinkedConfiguration& config1, const SimpleLinkedConfiguration& config2) const {
    if ((config1.size() == num_active_joints_) && (config2.size() == num_active_joints_)) {
      Eigen::VectorXd distances = Eigen::VectorXd::Zero((ssize_t)(config1.size()));
      for (size_t idx = 0; idx < config1.size(); idx++) {
        const SimpleJointModel& j1 = config1[idx];
        const SimpleJointModel& j2 = config2[idx];
        const double raw_distance = j1.SignedDistance(j2);
        const double joint_distance_weight = joint_distance_weights_[idx];
        distances((int64_t)idx) = raw_distance * joint_distance_weight;
      }
      return distances;
    } else {
      throw std::invalid_argument("Size of config1 and/or config2 does not match num_active_joints_");
    }
  }

  virtual double ComputeConfigurationDistance(const SimpleLinkedConfiguration& config1,
                                              const SimpleLinkedConfiguration& config2) const {
    return ComputePerDimensionConfigurationSignedDistance(config1, config2).norm();
  }

  virtual SimpleLinkedConfiguration InterpolateBetweenConfigurations(const SimpleLinkedConfiguration& start,
                                                                     const SimpleLinkedConfiguration& end,
                                                                     const double ratio) const {
    if (start.size() == end.size()) {
      // Make the interpolated config
      SimpleLinkedConfiguration interpolated;
      interpolated.reserve(start.size());
      for (size_t idx = 0; idx < start.size(); idx++) {
        const SimpleJointModel& j1 = start[idx];
        const SimpleJointModel& j2 = end[idx];
        if (j1.IsSameType(j2)) {
          if (j1.IsContinuous()) {
            const double interpolated_value =
                EigenHelpers::InterpolateContinuousRevolute(j1.GetValue(), j2.GetValue(), ratio);
            interpolated.push_back(j1.CopyWithNewValue(interpolated_value));
          } else {
            const double interpolated_value = EigenHelpers::Interpolate(j1.GetValue(), j2.GetValue(), ratio);
            interpolated.push_back(j1.CopyWithNewValue(interpolated_value));
          }
        } else {
          throw std::invalid_argument("Joint model types do not match");
        }
      }
      return interpolated;
    } else {
      throw std::invalid_argument("start.size() != end.size()");
    }
  }

  virtual SimpleLinkedConfiguration AverageConfigurations(
      const std::vector<SimpleLinkedConfiguration, SimpleLinkedConfigAlloc>& configurations) const {
    if (configurations.size() > 0) {
      // Safety checks
      const SimpleLinkedConfiguration& representative_config = configurations.front();
      for (size_t idx = 0; idx < configurations.size(); idx++) {
        const SimpleLinkedConfiguration& current_config = configurations[idx];
        if (representative_config.size() == current_config.size()) {
          for (size_t jdx = 0; jdx < representative_config.size(); jdx++) {
            const SimpleJointModel& jr = representative_config[jdx];
            const SimpleJointModel& jc = current_config[jdx];
            if (jr.IsSameType(jc) == false) {
              throw std::invalid_argument(
                  "Joint model of configuration does not match joint model in reference configuration");
            }
          }
        } else {
          throw std::invalid_argument("Configuration does not match size of reference configuration");
        }
      }
      // Get number of DoF
      const size_t config_size = representative_config.size();
      // Separate the joint values
      std::vector<std::vector<double>> raw_values(config_size);
      for (size_t idx = 0; idx < configurations.size(); idx++) {
        const SimpleLinkedConfiguration& q = configurations[idx];
        for (size_t qdx = 0; qdx < config_size; qdx++) {
          const double jval = q[qdx].GetValue();
          raw_values[qdx].push_back(jval);
        }
      }
      // Average each joint
      SimpleLinkedConfiguration average_config;
      average_config.reserve(config_size);
      for (size_t jdx = 0; jdx < config_size; jdx++) {
        const std::vector<double>& values = raw_values[jdx];
        const SimpleJointModel& representative_joint = representative_config[jdx];
        if (representative_joint.IsContinuous()) {
          const double average_value = EigenHelpers::AverageContinuousRevolute(values);
          average_config.push_back(representative_joint.CopyWithNewValue(average_value));
        } else {
          const double average_value = EigenHelpers::AverageStdVectorDouble(values);
          average_config.push_back(representative_joint.CopyWithNewValue(average_value));
        }
      }
      return average_config;
    } else {
      return SimpleLinkedConfiguration();
    }
  }

  inline Eigen::Matrix<double, 6, Eigen::Dynamic> ComputeLinkPointJacobian(
      const std::string& link_name, const Eigen::Vector4d& link_relative_point) const {
    // Get the link transform (by extension, this ensures we've been given a valid link)
    const Eigen::Isometry3d link_transform = GetLinkTransform(link_name);
    // Transform the point into world frame
    const Eigen::Vector3d world_point = (link_transform * link_relative_point).block<3, 1>(0, 0);
    // Make the jacobian storage
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian =
        Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero((ssize_t)6, (ssize_t)num_active_joints_);
    // First, check if the link name is a valid link name
    int64_t link_idx = -1;
    for (size_t idx = 0; idx < links_.size(); idx++) {
      const std::string& current_link_name = links_[idx].LinkName();
      if (current_link_name == link_name) {
        link_idx = (int64_t)idx;
        break;
      }
    }
    if (link_idx < 0) {
      std::cerr << "Link " << link_name << " does not exist" << std::endl;
      return jacobian;
    }
    // Second, check if the link name is the first (root) link
    if (link_idx == 0) {
      // If so, return zeros for the jacobian, because the point *CANNOT MOVE*
      return jacobian;
    }
    // Walk up the kinematic tree and fill in the blocks one at a time
    int64_t current_link_index = link_idx;
    while (current_link_index > 0) {
      const KinematicSkeletonElement& current_element = kinematic_skeleton_[current_link_index];
      const int64_t parent_joint_index = current_element.parent_joint_index_;
      if (parent_joint_index < 0) {
        throw std::runtime_error("Encountered invalid parent joint index (<0)");
      }
      const int64_t active_joint_index = active_joint_index_map_.at(parent_joint_index);
      // Get the current joint
      const RobotJoint& parent_joint = joints_[parent_joint_index];
      // Get the child link
      const RobotLink& child_link = links_[(size_t)parent_joint.ChildLinkIndex()];
      // Get the transform of the current joint
      const Eigen::Isometry3d& joint_transform = child_link.LinkTransform();
      // Compute the jacobian for the current joint
      if (parent_joint.JointModel().IsRevolute()) {
        const Eigen::Vector3d joint_axis = (Eigen::Vector3d)(joint_transform.rotation() * parent_joint.JointAxis());
        jacobian.block<3, 1>(0, active_joint_index) = joint_axis.cross(world_point - joint_transform.translation());
        jacobian.block<3, 1>(3, active_joint_index) = joint_axis;
      } else if (parent_joint.JointModel().IsPrismatic()) {
        const Eigen::Vector3d joint_axis = (Eigen::Vector3d)(joint_transform * parent_joint.JointAxis());
        jacobian.block<3, 1>(0, active_joint_index) = joint_axis;
      } else {
        // We do nothing for fixed joints
        if (parent_joint.JointModel().IsFixed() == false) {
          throw std::runtime_error("Invalid parent joint model type");
        }
      }
      // Update
      current_link_index = current_element.parent_link_index_;
    }
    return jacobian;
  }

  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> ComputeLinkPointTranslationJacobian(
      const std::string& link_name, const Eigen::Vector4d& link_relative_point) const {
    const Eigen::Matrix<double, 6, Eigen::Dynamic> full_jacobian =
        ComputeLinkPointJacobian(link_name, link_relative_point);
    Eigen::Matrix<double, 3, Eigen::Dynamic> trans_only_jacobian(3, full_jacobian.cols());
    trans_only_jacobian << full_jacobian.row(0), full_jacobian.row(1), full_jacobian.row(2);
    return trans_only_jacobian;
  }
};
}  // namespace simple_linked_robot_model

#endif  // SIMPLE_LINKED_ROBOT_MODEL_HPP
