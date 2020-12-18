#ifndef EIGEN_TYPEDEFS_HPP
#define EIGEN_TYPEDEFS_HPP

<<<<<<< HEAD
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <map>

namespace EigenHelpers {
////////////////////////////////////////////////////////////////////////////
// Typedefs for aligned STL containers using Eigen types
////////////////////////////////////////////////////////////////////////////

typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> VectorVector2f;
typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VectorVector2d;
typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> VectorVector3f;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VectorVector3d;
typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> VectorVector4f;
typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> VectorVector4d;
typedef std::vector<Eigen::Quaternionf, Eigen::aligned_allocator<Eigen::Quaternionf>> VectorQuaternionf;
typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> VectorQuaterniond;
typedef std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> VectorIsometry3f;
typedef std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> VectorIsometry3d;
typedef std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f>> VectorAffine3f;
typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> VectorAffine3d;
typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> VectorMatrix4f;
typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> VectorMatrix4d;
typedef std::vector<Eigen::Translation3f, Eigen::aligned_allocator<Eigen::Translation3f>> VectorTranslation3f;
typedef std::vector<Eigen::Translation3d, Eigen::aligned_allocator<Eigen::Translation3d>> VectorTranslation3d;
typedef std::map<std::string, Eigen::Vector2f, std::less<std::string>,
                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector2f>>>
    MapStringVector2f;
typedef std::map<std::string, Eigen::Vector2d, std::less<std::string>,
                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector2d>>>
    MapStringVector2d;
typedef std::map<std::string, Eigen::Vector3f, std::less<std::string>,
                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3f>>>
    MapStringVector3f;
typedef std::map<std::string, Eigen::Vector3d, std::less<std::string>,
                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d>>>
    MapStringVector3d;
typedef std::map<std::string, Eigen::Vector4f, std::less<std::string>,
                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector4f>>>
    MapStringVector4f;
typedef std::map<std::string, Eigen::Vector4d, std::less<std::string>,
                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector4d>>>
    MapStringVector4d;
typedef std::map<std::string, Eigen::Quaternionf, std::less<std::string>,
                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::Quaternionf>>>
    MapStringQuaternionf;
typedef std::map<std::string, Eigen::Quaterniond, std::less<std::string>,
                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::Quaterniond>>>
    MapStringQuaterniond;
typedef std::map<std::string, Eigen::Isometry3f, std::less<std::string>,
                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3f>>>
    MapStringIsometry3f;
typedef std::map<std::string, Eigen::Isometry3d, std::less<std::string>,
                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d>>>
    MapStringIsometry3d;
typedef std::map<std::string, Eigen::Affine3f, std::less<std::string>,
                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3f>>>
    MapStringAffine3f;
typedef std::map<std::string, Eigen::Affine3d, std::less<std::string>,
                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d>>>
    MapStringAffine3d;
}  // namespace EigenHelpers
=======
#include <map>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

namespace EigenHelpers
{
    ////////////////////////////////////////////////////////////////////////////
    // Typedefs for aligned STL containers using Eigen types
    ////////////////////////////////////////////////////////////////////////////

    typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> VectorVector2f;
    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VectorVector2d;
    typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> VectorVector3f;
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VectorVector3d;
    typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> VectorVector4f;
    typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> VectorVector4d;
    typedef std::vector<Eigen::Quaternionf, Eigen::aligned_allocator<Eigen::Quaternionf>> VectorQuaternionf;
    typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> VectorQuaterniond;
    typedef std::vector<Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f>> VectorIsometry3f;
    typedef std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> VectorIsometry3d;
    typedef std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f>> VectorAffine3f;
    typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> VectorAffine3d;
    typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> VectorMatrix4f;
    typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> VectorMatrix4d;
    typedef std::vector<Eigen::Translation3f, Eigen::aligned_allocator<Eigen::Translation3f>> VectorTranslation3f;
    typedef std::vector<Eigen::Translation3d, Eigen::aligned_allocator<Eigen::Translation3d>> VectorTranslation3d;
    typedef std::map<std::string, Eigen::Vector2f, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector2f>>> MapStringVector2f;
    typedef std::map<std::string, Eigen::Vector2d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector2d>>> MapStringVector2d;
    typedef std::map<std::string, Eigen::Vector3f, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3f>>> MapStringVector3f;
    typedef std::map<std::string, Eigen::Vector3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d>>> MapStringVector3d;
    typedef std::map<std::string, Eigen::Vector4f, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector4f>>> MapStringVector4f;
    typedef std::map<std::string, Eigen::Vector4d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector4d>>> MapStringVector4d;
    typedef std::map<std::string, Eigen::Quaternionf, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Quaternionf>>> MapStringQuaternionf;
    typedef std::map<std::string, Eigen::Quaterniond, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Quaterniond>>> MapStringQuaterniond;
    typedef std::map<std::string, Eigen::Isometry3f, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3f>>> MapStringIsometry3f;
    typedef std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d>>> MapStringIsometry3d;
    typedef std::map<std::string, Eigen::Affine3f, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3f>>> MapStringAffine3f;
    typedef std::map<std::string, Eigen::Affine3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d>>> MapStringAffine3d;
}
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default

#endif
