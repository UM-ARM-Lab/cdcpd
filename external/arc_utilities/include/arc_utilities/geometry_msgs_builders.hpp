#ifndef GEOMETRY_MSGS_BUILDERS_HPP
#define GEOMETRY_MSGS_BUILDERS_HPP

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

<<<<<<< HEAD
namespace arc_utilities {
namespace rmb {
inline geometry_msgs::Point MakePoint(const double x, const double y, const double z) {
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

inline geometry_msgs::Vector3 MakeVector3(const double x, const double y, const double z) {
  geometry_msgs::Vector3 vec;
  vec.x = x;
  vec.y = y;
  vec.z = z;
  return vec;
}

inline geometry_msgs::Quaternion MakeQuaternion(const double x, const double y, const double z, const double w) {
  geometry_msgs::Quaternion quat;
  quat.x = x;
  quat.y = y;
  quat.z = z;
  quat.w = w;
  return quat;
}

inline geometry_msgs::Transform MakeTransform(const geometry_msgs::Vector3& translation,
                                              const geometry_msgs::Quaternion& rotation) {
  geometry_msgs::Transform tf;
  tf.translation = translation;
  tf.rotation = rotation;
  return tf;
}

inline geometry_msgs::Transform MakeTransform(const geometry_msgs::Point& translation,
                                              const geometry_msgs::Quaternion& rotation) {
  geometry_msgs::Transform tf;
  tf.translation.x = translation.x;
  tf.translation.y = translation.y;
  tf.translation.z = translation.z;
  tf.rotation = rotation;
  return tf;
}

inline geometry_msgs::Pose MakePose(const geometry_msgs::Point& position,
                                    const geometry_msgs::Quaternion& orientation) {
  geometry_msgs::Pose pose;
  pose.position = position;
  pose.orientation = orientation;
  return pose;
}

inline geometry_msgs::Pose MakePose(const geometry_msgs::Vector3& position,
                                    const geometry_msgs::Quaternion& orientation) {
  geometry_msgs::Pose pose;
  pose.position.x = position.x;
  pose.position.y = position.y;
  pose.position.z = position.z;
  pose.orientation = orientation;
  return pose;
}
}  // namespace rmb
}  // namespace arc_utilities

#endif  // GEOMETRY_MSGS_BUILDERS_HPP
=======
namespace arc_utilities
{
    namespace rmb
    {
        inline geometry_msgs::Point MakePoint(const double x, const double y, const double z)
        {
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            return point;
        }

        inline geometry_msgs::Vector3 MakeVector3(const double x, const double y, const double z)
        {
            geometry_msgs::Vector3 vec;
            vec.x = x;
            vec.y = y;
            vec.z = z;
            return vec;
        }

        inline geometry_msgs::Quaternion MakeQuaternion(const double x, const double y, const double z, const double w)
        {
            geometry_msgs::Quaternion quat;
            quat.x = x;
            quat.y = y;
            quat.z = z;
            quat.w = w;
            return quat;
        }

        inline geometry_msgs::Transform MakeTransform(
                const geometry_msgs::Vector3& translation, const geometry_msgs::Quaternion& rotation)
        {
            geometry_msgs::Transform tf;
            tf.translation = translation;
            tf.rotation = rotation;
            return tf;
        }

        inline geometry_msgs::Transform MakeTransform(
                const geometry_msgs::Point& translation, const geometry_msgs::Quaternion& rotation)
        {
            geometry_msgs::Transform tf;
            tf.translation.x = translation.x;
            tf.translation.y = translation.y;
            tf.translation.z = translation.z;
            tf.rotation = rotation;
            return tf;
        }

        inline geometry_msgs::Pose MakePose(
                const geometry_msgs::Point& position, const geometry_msgs::Quaternion& orientation)
        {
            geometry_msgs::Pose pose;
            pose.position = position;
            pose.orientation = orientation;
            return pose;
        }

        inline geometry_msgs::Pose MakePose(
                const geometry_msgs::Vector3& position, const geometry_msgs::Quaternion& orientation)
        {
            geometry_msgs::Pose pose;
            pose.position.x = position.x;
            pose.position.y = position.y;
            pose.position.z = position.z;
            pose.orientation = orientation;
            return pose;
        }
    }
}

#endif // GEOMETRY_MSGS_BUILDERS_HPP
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
