#ifndef EIGEN_HELPERS_HPP
#define EIGEN_HELPERS_HPP


#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Jacobi>
#include <Eigen/SVD>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <map>
#include <vector>
#include <functional>
#include <type_traits>
#include "eigen_typedefs.hpp"
#include "math_helpers.hpp"
#include "vector_math.hpp"
#include "arc_exceptions.hpp"


namespace EigenHelpers
{
    ////////////////////////////////////////////////////////////////////////////
    // Misc
    ////////////////////////////////////////////////////////////////////////////

    inline bool Equal3d(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
    {
        if ((v1.x() == v2.x()) && (v1.y() == v2.y()) && (v1.z() == v2.z()))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    inline bool Equal4d(const Eigen::Vector4d& v1, const Eigen::Vector4d& v2)
    {
        if ((v1(0) == v2(0)) && (v1(1) == v2(1)) && (v1(2) == v2(2)) && (v1(3) == v2(3)))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    inline bool CloseEnough(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2, const double threshold)
    {
        return CloseEnough(v1.x(), v2.x(), threshold) &&
            CloseEnough(v1.y(), v2.y(), threshold);
    }

    inline bool CloseEnough(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const double threshold)
    {
        double real_threshold = std::abs(threshold);
        if (std::abs(v1.x() - v2.x()) > real_threshold)
        {
            return false;
        }
        if (std::abs(v1.y() - v2.y()) > real_threshold)
        {
            return false;
        }
        if (std::abs(v1.z() - v2.z()) > real_threshold)
        {
            return false;
        }
        return true;
    }

    template <typename EigenType, typename Allocator=Eigen::aligned_allocator<EigenType>>
    inline bool CloseEnough(const std::vector<EigenType, Allocator>& a, const std::vector<EigenType, Allocator>& b, const double threshold)
    {
        if (a.size() != b.size())
        {
            return false;
        }
        for (size_t idx = 0; idx < a.size(); idx++)
        {
            if (!CloseEnough(a[idx], b[idx], threshold))
            {
                return false;
            }
        }
        return true;
    }


    template <typename EigenType, typename Allocator=Eigen::aligned_allocator<EigenType>>
    inline bool IsApprox(const std::vector<EigenType, Allocator>& a, const std::vector<EigenType, Allocator>& b,
                         const typename EigenType::Scalar& precision = Eigen::NumTraits<typename EigenType::Scalar>::dummy_precision())
    {
        if (a.size() != b.size())
        {
            return false;
        }
        for (size_t idx = 0; idx < a.size(); idx++)
        {
            if (!a[idx].isApprox(b[idx], precision))
            {
                return false;
            }
        }
        return true;
    }

    template <typename Derived>
    inline Eigen::MatrixXd ClampNorm(const Eigen::MatrixBase<Derived>& item_to_clamp, const double max_norm)
    {
        assert(max_norm > 0 && "You must pass a maximum norm that is positive");
        const double current_norm = item_to_clamp.norm();
        if (current_norm > max_norm)
        {
            return item_to_clamp * (max_norm / current_norm);
        }
        return item_to_clamp;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Vectors of Eigen data transformations
    ////////////////////////////////////////////////////////////////////////////

    template <typename _Scalar, int _Dim, int _Mode, int _Options, typename EigenType, typename Allocator>
    inline std::vector<EigenType, Allocator> TransformData(
            const Eigen::Transform<_Scalar, _Dim, _Mode, _Options>& transform,
            const std::vector<EigenType, Allocator>& data)
    {
        std::vector<EigenType, Allocator> retval;
        retval.reserve(data.size());
        for (const auto& item : data)
        {
            retval.push_back(transform * item);
        }
        return retval;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Kinematics functions
    ////////////////////////////////////////////////////////////////////////////

    inline Eigen::Vector3d RotateVector(const Eigen::Quaterniond& quat, const Eigen::Vector3d& vec)
    {
        const Eigen::Quaterniond temp(0.0, vec.x(), vec.y(), vec.z());
        const Eigen::Quaterniond res = quat * (temp * quat.inverse());
        return Eigen::Vector3d(res.x(), res.y(), res.z());
    }

    inline Eigen::Vector3d RotateVectorReverse(const Eigen::Quaterniond& quat, const Eigen::Vector3d& vec)
    {
        const Eigen::Quaterniond temp(0.0, vec.x(), vec.y(), vec.z());
        const Eigen::Quaterniond res = quat.inverse() * (temp * quat);
        return Eigen::Vector3d(res.x(), res.y(), res.z());
    }

    inline Eigen::VectorXd SafeNormal(const Eigen::VectorXd& vec)
    {
        const double norm = vec.norm();
        if (norm > std::numeric_limits<double>::epsilon())
        {
            return vec / norm;
        }
        else
        {
            return vec;
        }
    }

    inline double SquaredNorm(const std::vector<double>& vec)
    {
        double squared_norm = 0.0;
        for (size_t idx = 0; idx < vec.size(); idx++)
        {
            const double element = vec[idx];
            squared_norm += (element * element);
        }
        return squared_norm;
    }

    inline double Norm(const std::vector<double>& vec)
    {
        return std::sqrt(SquaredNorm(vec));
    }

    inline Eigen::Matrix3d Skew(const Eigen::Vector3d& vector)
    {
        Eigen::Matrix3d skewed;
        skewed << 0.0, -vector.z(), vector.y(),
                  vector.z(), 0.0, -vector.x(),
                  -vector.y(), vector.x(), 0.0;
        return skewed;
    }

    inline Eigen::Vector3d Unskew(const Eigen::Matrix3d& matrix)
    {
        const Eigen::Matrix3d matrix_symetric = (matrix - matrix.transpose()) / 2.0;
        const Eigen::Vector3d unskewed(matrix_symetric(2, 1), matrix_symetric(0, 2), matrix_symetric(1, 0));
        return unskewed;
    }

    inline Eigen::Matrix4d TwistHat(const Eigen::Matrix<double, 6, 1>& twist)
    {
        const Eigen::Vector3d trans_velocity = twist.segment<3>(0);
        const Eigen::Matrix3d hatted_rot_velocity = Skew(twist.segment<3>(3));
        Eigen::Matrix4d hatted_twist = Eigen::Matrix4d::Zero();
        hatted_twist.block<3, 3>(0, 0) = hatted_rot_velocity;
        hatted_twist.block<3, 1>(0, 3) = trans_velocity;
        return hatted_twist;
    }

    inline Eigen::Matrix<double, 6, 1> TwistUnhat(const Eigen::Matrix4d& hatted_twist)
    {
         const Eigen::Vector3d trans_velocity = hatted_twist.block<3, 1>(0, 3);
         const Eigen::Vector3d rot_velocity = Unskew(hatted_twist.block<3, 3>(0, 0));
         Eigen::Matrix<double, 6, 1> twist;
         twist.segment<3>(0) = trans_velocity;
         twist.segment<3>(3) = rot_velocity;
         return twist;
    }

    template <int _Mode>
    inline Eigen::Matrix<double, 6, 6> AdjointFromTransform(const Eigen::Transform<double, 3, _Mode>& transform)
    {
        EIGEN_STATIC_ASSERT(_Mode == Eigen::Affine || _Mode == Eigen::Isometry,
                            "THIS FUNCTION IS ONLY INTENDED FOR HOMOGENEOUS TRANSFORMS!!!");
        const Eigen::Matrix3d rotation = transform.rotation();
        const Eigen::Vector3d translation = transform.translation();
        const Eigen::Matrix3d translation_hat = Skew(translation);
        // Assemble the adjoint matrix
        Eigen::Matrix<double, 6, 6> adjoint;
        adjoint.block<3, 3>(0, 0) = rotation;
        adjoint.block<3, 3>(0, 3) = translation_hat * rotation;
        adjoint.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
        adjoint.block<3, 3>(3, 3) = rotation;
        return adjoint;
    }

    template <int _Mode>
    inline Eigen::Matrix<double, 6, 1> TransformTwist(const Eigen::Transform<double, 3, _Mode>& transform,
                                                      const Eigen::Matrix<double, 6, 1>& initial_twist)
    {
        EIGEN_STATIC_ASSERT(_Mode == Eigen::Affine || _Mode == Eigen::Isometry,
                            "THIS FUNCTION IS ONLY INTENDED FOR HOMOGENEOUS TRANSFORMS!!!");
        return (Eigen::Matrix<double, 6, 1>)(EigenHelpers::AdjointFromTransform(transform) * initial_twist);
    }

    template <int _Mode>
    inline Eigen::Matrix<double, 6, 1> TwistBetweenTransforms(const Eigen::Transform<double, 3, _Mode>& start,
                                                              const Eigen::Transform<double, 3, _Mode>& end)
    {
        EIGEN_STATIC_ASSERT(_Mode == Eigen::Affine || _Mode == Eigen::Isometry,
                            "THIS FUNCTION IS ONLY INTENDED FOR HOMOGENEOUS TRANSFORMS!!!");
        const Eigen::Transform<double, 3, _Mode> t_diff = start.inverse(_Mode) * end;
        return TwistUnhat(t_diff.matrix().log());
    }

    inline Eigen::Matrix3d ExpMatrixExact(const Eigen::Matrix3d& hatted_rot_velocity, const double delta_t)
    {
        assert(std::abs(Unskew(hatted_rot_velocity).norm() - 1.0) < 1e-10);
        const Eigen::Matrix3d exp_matrix = Eigen::Matrix3d::Identity()
            + (hatted_rot_velocity * sin(delta_t))
            + (hatted_rot_velocity * hatted_rot_velocity * (1.0 - cos(delta_t)));
        return exp_matrix;
    }

    inline Eigen::Isometry3d ExpTwist(const Eigen::Matrix<double, 6, 1>& twist, const double delta_t)
    {
        const Eigen::Vector3d trans_velocity = twist.segment<3>(0);
        const Eigen::Vector3d rot_velocity = twist.segment<3>(3);
        const double trans_velocity_norm = trans_velocity.norm();
        const double rot_velocity_norm = rot_velocity.norm();
        Eigen::Matrix4d raw_transform = Eigen::Matrix4d::Identity();
        if (rot_velocity_norm >= 1e-100)
        {
            const double scaled_delta_t = delta_t * rot_velocity_norm;
            const Eigen::Vector3d scaled_trans_velocity = trans_velocity / rot_velocity_norm;
            const Eigen::Vector3d scaled_rot_velocity = rot_velocity / rot_velocity_norm;
            const Eigen::Matrix3d rotation_displacement = ExpMatrixExact(Skew(scaled_rot_velocity), scaled_delta_t);
            const Eigen::Vector3d translation_displacement =
                ((Eigen::Matrix3d::Identity() - rotation_displacement) * scaled_rot_velocity.cross(scaled_trans_velocity))
                + (scaled_rot_velocity * scaled_rot_velocity.transpose() * scaled_trans_velocity * scaled_delta_t);
            raw_transform.block<3, 3>(0, 0) = rotation_displacement;
            raw_transform.block<3, 1>(0, 3) = translation_displacement;
        }
        else
        {
            if ((trans_velocity_norm >= 1e-100) || (rot_velocity_norm == 0.0))
            {
                raw_transform.block<3, 1>(0, 3) = trans_velocity * delta_t;
            }
            else
            {
                std::cerr << "*** WARNING - YOU MAY ENCOUNTER NUMERICAL INSTABILITY IN EXPTWIST(...) WITH TRANS & ROT VELOCITY NORM < 1e-100 ***" << std::endl;
                const double scaled_delta_t = delta_t * rot_velocity_norm;
                const Eigen::Vector3d scaled_trans_velocity = trans_velocity / rot_velocity_norm;
                const Eigen::Vector3d scaled_rot_velocity = rot_velocity / rot_velocity_norm;
                const Eigen::Matrix3d rotation_displacement = ExpMatrixExact(Skew(scaled_rot_velocity), scaled_delta_t);
                const Eigen::Vector3d translation_displacement =
                    ((Eigen::Matrix3d::Identity() - rotation_displacement) * scaled_rot_velocity.cross(scaled_trans_velocity))
                    + (scaled_rot_velocity * scaled_rot_velocity.transpose() * scaled_trans_velocity * scaled_delta_t);
                raw_transform.block<3, 3>(0, 0) = rotation_displacement;
                raw_transform.block<3, 1>(0, 3) = translation_displacement;
            }
        }
        Eigen::Isometry3d transform;
        transform = raw_transform;
        return transform;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Interpolation functions
    ////////////////////////////////////////////////////////////////////////////

    template <typename T, int ROWS>
    inline Eigen::Matrix<T, ROWS, 1> Interpolate(const Eigen::Matrix<T, ROWS, 1>& v1,
                                                 const Eigen::Matrix<T, ROWS, 1>& v2, const double ratio)
    {
        // Safety check sizes
        if (v1.size() != v2.size())
        {
            throw_arc_exception(std::invalid_argument, "Vectors v1 and v2 must be the same size");
        }
        // Safety check ratio
        const double real_ratio = SafetyCheckUnitInterval(ratio);
        // Interpolate
        // This is the numerically stable version, rather than  (p1 + (p2 - p1) * real_ratio)
        return ((v1 * (1.0 - real_ratio)) + (v2 * real_ratio));
    }

    inline Eigen::Quaterniond Interpolate(const Eigen::Quaterniond& q1,
                                          const Eigen::Quaterniond& q2, const double ratio)
    {
        // Safety check ratio
        const double real_ratio = SafetyCheckUnitInterval(ratio);
        // Interpolate
        return q1.slerp(real_ratio, q2);
    }

    inline Eigen::Isometry3d Interpolate(const Eigen::Isometry3d& t1,
                                         const Eigen::Isometry3d& t2, const double ratio)
    {
        // Safety check ratio
        const double real_ratio = SafetyCheckUnitInterval(ratio);
        // Interpolate
        const Eigen::Vector3d v1 = t1.translation();
        const Eigen::Quaterniond q1(t1.rotation());
        const Eigen::Vector3d v2 = t2.translation();
        const Eigen::Quaterniond q2(t2.rotation());
        const Eigen::Vector3d vint = Interpolate(v1, v2, real_ratio);
        const Eigen::Quaterniond qint = Interpolate(q1, q2, real_ratio);
        const Eigen::Isometry3d tint = ((Eigen::Translation3d)vint) * qint;
        return tint;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Distance functions
    ////////////////////////////////////////////////////////////////////////////

    inline double SquaredDistance(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2)
    {
        const double xd = v2.x() - v1.x();
        const double yd = v2.y() - v1.y();
        return ((xd * xd) + (yd * yd));
    }

    inline double Distance(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2)
    {
        return sqrt(SquaredDistance(v1, v2));
    }

    inline double SquaredDistance(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
    {
        const double xd = v2.x() - v1.x();
        const double yd = v2.y() - v1.y();
        const double zd = v2.z() - v1.z();
        return ((xd * xd) + (yd * yd) + (zd * zd));
    }

    inline double Distance(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
    {
        return sqrt(SquaredDistance(v1, v2));
    }

    inline double SquaredDistance(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2)
    {
        assert(v1.size() == v2.size());
        return (v2 - v1).squaredNorm();
    }

    inline double Distance(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2)
    {
        return (v2 - v1).norm();
    }

    // From here: https://chrischoy.github.io/research/measuring-rotation/
    // This assumes that the incomming quaternions are normalized
    inline double Distance(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2)
    {
        const double dq = std::abs((q1.w() * q2.w()) + (q1.x() * q2.x()) + (q1.y() * q2.y()) + (q1.z() * q2.z()));
        if (dq < (1.0 - std::numeric_limits<double>::epsilon()))
        {
            return acos(2.0 * (dq * dq) - 1.0);
        }
        else
        {
            return 0.0;
        }
    }

    // From here: http://www.boris-belousov.net/2016/12/01/quat-dist/#rotation-matrices
    // Returns the minimum angular rotation needed to align r1 and r2
    // Assumes that r1 and r2 are proper rotation matrices
    inline double Distance(const Eigen::Matrix3d& r1, const Eigen::Matrix3d& r2)
    {
        const auto delta = r1 * r2.transpose();
        const auto tr = delta.trace();
        return acos((tr - 1.0) / 2.0);
    }

    inline double Distance(const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2, const double alpha = 0.5)
    {
        assert(alpha >= 0.0);
        assert(alpha <= 1.0);
        const Eigen::Vector3d v1 = t1.translation();
        const Eigen::Quaterniond q1(t1.rotation());
        const Eigen::Vector3d v2 = t2.translation();
        const Eigen::Quaterniond q2(t2.rotation());
        const double vdist = Distance(v1, v2) * (1.0 - alpha);
        const double qdist = Distance(q1, q2) * (alpha);
        return vdist + qdist;
    }


    inline double CalculateTotalDistance(const EigenHelpers::VectorVector3d& points)
    {
        double distance = 0;

        for (size_t idx = 1; idx < points.size(); ++idx)
        {
            const double delta = (points[idx] - points[idx - 1]).norm();
            distance += delta;
        }

        return distance;
    }

    inline std::vector<double> CalculateIndividualDistances(const EigenHelpers::VectorVector3d& points)
    {
        std::vector<double> distances(points.size());

        if (points.size() > 0)
        {
            distances[0] = 0.0;
            for (size_t idx = 1; idx < points.size(); ++idx)
            {
                distances[idx] = (points[idx] - points[idx - 1]).norm();
            }
        }

        return distances;
    }

    inline std::vector<double> CalculateCumulativeDistances(const EigenHelpers::VectorVector3d& points)
    {
        std::vector<double> distances(points.size());

        if (points.size() > 0)
        {
            distances[0] = 0.0;
            for (size_t idx = 1; idx < points.size(); ++idx)
            {
                const double delta = (points[idx] - points[idx - 1]).norm();
                distances[idx] = distances[idx - 1] + delta;
            }
        }

        return distances;
    }

    /**
     * @brief Computes the squared distance between each point in a given set
     *
     * @param set The set of points to compute distances on, arranged with each point as a column
     *
     * @return The distances between each pair of nodes
     */
    template<typename ScalarType, int VectorLength, int NumVectors>
    inline Eigen::Matrix<double, NumVectors, NumVectors> CalculateSquaredDistanceMatrix(const Eigen::Matrix<ScalarType, VectorLength, NumVectors>& set)
    {
        assert (set.cols() > 0);
        const ssize_t num_vectors = set.cols(); // TODO: if NumVectors != Eigen::Dynamic, this is known at compile time

        Eigen::MatrixXd squared_dist(num_vectors, num_vectors);
        #ifdef ENABLE_PARALLEL_DISTANCE_MATRIX
            #pragma omp parallel for
        #endif
        for (ssize_t i = 0; i < num_vectors; i++)
        {
            squared_dist(i, i) = 0.0;
            for (ssize_t j = i + 1; j < num_vectors; j++)
            {
                const double sq_dist = (set.col(i) - set.col(j)).squaredNorm();
                squared_dist(i, j) = sq_dist;
                squared_dist(j, i) = sq_dist;
            }
        }

        return squared_dist;
    }

    /**
     * @brief Computes the distance between each point in a given set
     *
     * @param set The set of points to compute distances on, arranged with each point as a column
     *
     * @return The distances between each pair of nodes
     */
    template<typename ScalarType, int VectorLength, int NumVectors>
    inline Eigen::Matrix<double, NumVectors, NumVectors> CalculateDistanceMatrix(const Eigen::Matrix<ScalarType, VectorLength, NumVectors>& set)
    {
        return CalculateSquaredDistanceMatrix(set).cwiseSqrt();
    }

    /**
     * @brief Computes the squared distance between a given point, and every point in a set
     *
     * @param set The set of points to compute distances on, arranged with each point as a column
     * @param point The point to measure the distance to
     *
     * @return The distances between each pair of nodes
     */
    template<typename ScalarType, int VectorLength, int NumVectors>
    inline Eigen::Matrix<double, NumVectors, 1> CalculateSquaredDistanceToSet(const Eigen::Matrix<ScalarType, VectorLength, NumVectors>& set, const Eigen::Matrix<ScalarType, VectorLength, 1>& point)
    {
        return (set.colwise() - point).colwise().squaredNorm();
    }

    /**
     * @brief Computes the distance between a given point, and every point in a set
     *
     * @param set The set of points to compute distances on, arranged with each point as a column
     * @param point The point to measure the distance to
     *
     * @return The distances between each pair of nodes
     */
    template<typename ScalarType, int VectorLength, int NumVectors>
    inline Eigen::Matrix<double, NumVectors, 1> CalculateDistanceToSet(const Eigen::Matrix<ScalarType, VectorLength, NumVectors>& set, const Eigen::Matrix<ScalarType, VectorLength, 1>& point)
    {
        return CalculateSquaredDistanceToSet(set, point).cwiseSqrt();
    }

    /**
     * @brief Finds the closest point in the set to a given point
     *
     * @param set The set of points to compute distances on, arranged with each point as a column
     * @param point The point to measure the distance to
     *
     * @return The index of the point in the set
     */
    template<typename ScalarType, int VectorLength, int NumVectors>
    inline ssize_t ClosestPointInSet(const Eigen::Matrix<ScalarType, VectorLength, NumVectors>& set, const Eigen::Matrix<ScalarType, VectorLength, 1>& point)
    {
        assert (set.cols() > 0);
        ssize_t min_ind = 0;
        const Eigen::VectorXd squared_dist = CalculateSquaredDistanceToSet(set, point);
        squared_dist.minCoeff(&min_ind);
        return min_ind;
    }

    /**
     * @brief Calculates the squared distance between every point in A and every point in B.
     *
     * @param A (D x M) matrix of points
     * @param B (D x N) matrix of points
     *
     * @return (M X N) matrix of distances between points
     */
    template<typename DerivedA, typename DerivedB>
    inline Eigen::Matrix<double, Eigen::MatrixBase<DerivedA>::ColsAtCompileTime, Eigen::MatrixBase<DerivedB>::ColsAtCompileTime>
    SquaredDistancesBetweenPointSets(const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedB>& B)
    {
        using namespace Eigen;
        Matrix<double, MatrixBase<DerivedA>::ColsAtCompileTime, MatrixBase<DerivedB>::ColsAtCompileTime> distances_sq;
        if (MatrixBase<DerivedA>::ColsAtCompileTime == Dynamic ||
            MatrixBase<DerivedB>::ColsAtCompileTime == Dynamic)
        {
            distances_sq.resize(A.cols(), B.cols());
        }

        for (ssize_t i = 0; i < B.cols(); ++i)
        {
            distances_sq.col(i) = (A.colwise() - B.col(i)).colwise().squaredNorm();
        }

        return distances_sq;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Conversion functions
    ////////////////////////////////////////////////////////////////////////////

    inline Eigen::Quaterniond QuaternionFromRPY(const double R, const double P, const double Y)
    {
        const Eigen::AngleAxisd roll(R, Eigen::Vector3d::UnitX());
        const Eigen::AngleAxisd pitch(P, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd yaw(Y, Eigen::Vector3d::UnitZ());
        const Eigen::Quaterniond quat(roll * pitch * yaw);
        return quat;
    }

    /* URDF RPY IS ACTUALLY APPLIED Y*P*R */
    inline Eigen::Quaterniond QuaternionFromUrdfRPY(const double R, const double P, const double Y)
    {
        const Eigen::AngleAxisd roll(R, Eigen::Vector3d::UnitX());
        const Eigen::AngleAxisd pitch(P, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd yaw(Y, Eigen::Vector3d::UnitZ());
        const Eigen::Quaterniond quat(yaw * pitch * roll);
        return quat;
    }

    // Returns XYZ Euler angles
    inline Eigen::Vector3d EulerAnglesFromRotationMatrix(const Eigen::Matrix3d& rot_matrix)
    {
        const Eigen::Vector3d euler_angles = rot_matrix.eulerAngles(0, 1, 2); // Use XYZ angles
        return euler_angles;
    }

    // Returns XYZ Euler angles
    inline Eigen::Vector3d EulerAnglesFromQuaternion(const Eigen::Quaterniond& quat)
    {
        return EulerAnglesFromRotationMatrix(quat.toRotationMatrix());
    }

    // Returns XYZ Euler angles
    inline Eigen::Vector3d EulerAnglesFromIsometry3d(const Eigen::Isometry3d& trans)
    {
        return EulerAnglesFromRotationMatrix(trans.rotation());
    }

    inline Eigen::Isometry3d TransformFromRPY(const double x, const double y, const double z, const double roll, const double pitch, const double yaw)
    {
        const Eigen::Isometry3d transform = Eigen::Translation3d(x, y, z) * QuaternionFromRPY(roll, pitch, yaw);
        return transform;
    }

    inline Eigen::Isometry3d TransformFromRPY(const Eigen::Vector3d& translation, const Eigen::Vector3d& rotation)
    {
        const Eigen::Isometry3d transform = (Eigen::Translation3d)translation * QuaternionFromRPY(rotation.x(), rotation.y(), rotation.z());
        return transform;
    }

    inline Eigen::Isometry3d TransformFromRPY(const Eigen::VectorXd& components)
    {
        assert(components.size() == 6);
        const Eigen::Isometry3d transform = Eigen::Translation3d(components(0), components(1), components(2)) * QuaternionFromRPY(components(3), components(4), components(5));
        return transform;
    }

    inline Eigen::VectorXd TransformToRPY(const Eigen::Isometry3d& transform)
    {
        Eigen::VectorXd components = Eigen::VectorXd::Zero(6);
        const Eigen::Vector3d translation = transform.translation();
        const Eigen::Vector3d rotation = EulerAnglesFromRotationMatrix(transform.rotation());
        components << translation, rotation;
        return components;
    }

    inline Eigen::Vector3d StdVectorDoubleToEigenVector3d(const std::vector<double>& vector)
    {
        assert(vector.size() == 3 && "std::vector<double> source vector is not 3 elements in size");
        return Eigen::Vector3d(vector[0], vector[1], vector[2]);
    }

    inline Eigen::VectorXd StdVectorDoubleToEigenVectorXd(const std::vector<double>& vector)
    {
        Eigen::VectorXd eigen_vector(vector.size());
        for (size_t idx = 0; idx < vector.size(); idx++)
        {
            const double val = vector[idx];
            eigen_vector((ssize_t)idx) = val;
        }
        return eigen_vector;
    }

    inline std::vector<double> EigenVector3dToStdVectorDouble(const Eigen::Vector3d& point)
    {
        return std::vector<double>{point.x(), point.y(), point.z()};
    }

    inline std::vector<double> EigenVectorXdToStdVectorDouble(const Eigen::VectorXd& eigen_vector)
    {
        std::vector<double> vector((size_t)eigen_vector.size());
        for (size_t idx = 0; idx < (size_t)eigen_vector.size(); idx++)
        {
            const double val = eigen_vector[(ssize_t)idx];
            vector[idx] = val;
        }
        return vector;
    }

    template<typename T, int LENGTH, typename Allocator>
    inline Eigen::Matrix<T, Eigen::Dynamic, 1> VectorEigenVectorToEigenVectorX(const std::vector<Eigen::Matrix<T, LENGTH, 1>, Allocator>& vector_eigen_input)
    {
        assert(vector_eigen_input.size() > 0);

        Eigen::Matrix<T, Eigen::Dynamic, 1> eigen_result;
        eigen_result.resize((ssize_t)vector_eigen_input.size() * vector_eigen_input[0].rows());

        for (size_t idx = 0; idx < vector_eigen_input.size(); idx++)
        {
            eigen_result.segment((ssize_t)idx * LENGTH, LENGTH) = vector_eigen_input[idx];
        }

        return eigen_result;
    }

    template<typename T, int LENGTH>
    inline std::vector<Eigen::Matrix<T, LENGTH, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, LENGTH, 1>>> EigenVectorXToVectorEigenVector(const Eigen::VectorXd& eigen_input)
    // TODO: Why can't I use the more generic version?
//    inline std::vector<Eigen::Matrix<T, LENGTH, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, LENGTH, 1>>> EigenVectorXToVectorEigenVector(const Eigen::Matrix<T, Eigen::Dynamic, 1>& eigen_input)
    {
        assert(eigen_input.rows() % LENGTH == 0);
        size_t num_vectors = eigen_input.rows() / LENGTH;

        std::vector<Eigen::Matrix<T, LENGTH, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, LENGTH, 1>>> vector_eigen_output(num_vectors);

        for (size_t idx = 0; idx < num_vectors; idx++)
        {
            vector_eigen_output[idx] = eigen_input.segment<LENGTH>((ssize_t)idx * LENGTH);
        }

        return vector_eigen_output;
    }

    template<typename T, int LENGTH>
    inline std::vector<Eigen::Matrix<T, LENGTH, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, LENGTH, 1>>> StdVectorXToVectorEigenVector(const std::vector<T>& std_input)
    {
        assert(std_input.size() % LENGTH == 0);
        const size_t num_vectors = std_input.size() / LENGTH;

        std::vector<Eigen::Matrix<T, LENGTH, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, LENGTH, 1>>> vector_eigen_output(num_vectors);

        for (size_t vec_idx = 0; vec_idx < num_vectors; vec_idx++)
        {
            for (size_t inner_idx = 0; inner_idx < LENGTH; ++inner_idx)
            {
                vector_eigen_output[vec_idx](inner_idx) = std_input[vec_idx * LENGTH + inner_idx];
            }
        }

        return vector_eigen_output;
    }

    template<typename T>
    inline std::vector<T> EigenVectorXToStdVector(const Eigen::Matrix<T, Eigen::Dynamic, 1>& eig_vec)
    {
        std::vector<T> std_vec(eig_vec.data(), eig_vec.data() + eig_vec.size());
        return std_vec;
    }

    template<typename T>
    inline Eigen::Matrix<T, Eigen::Dynamic, 1> StdVectorToEigenVectorX(const std::vector<T>& std_vec)
    {
        Eigen::Matrix<T, Eigen::Dynamic, 1> eig_vec(std_vec.size());
        memcpy(eig_vec.data(), std_vec.data(), std_vec.size() * sizeof(T));
        return eig_vec;
    }

    // Takes <x, y, z, w> as is the ROS custom!
    inline Eigen::Quaterniond StdVectorDoubleToEigenQuaterniond(const std::vector<double>& vector)
    {
        if (vector.size() != 4)
        {
            std::cerr << "Quaterniond source vector is not 4 elements in size" << std::endl;
            assert(false);
        }
        Eigen::Quaterniond eigen_quaternion(vector[3], vector[0], vector[1], vector[2]);
        return eigen_quaternion;
    }

    // Returns <x, y, z, w> as is the ROS custom!
    inline std::vector<double> EigenQuaterniondToStdVectorDouble(const Eigen::Quaterniond& quat)
    {
        return std::vector<double>{quat.x(), quat.y(), quat.z(), quat.w()};
    }

    ////////////////////////////////////////////////////////////////////////////
    // Averaging functions
    // Numerically more stable averages taken from http://people.ds.cam.ac.uk/fanf2/hermes/doc/antiforgery/stats.pdf
    ////////////////////////////////////////////////////////////////////////////

    /**
     * This function is really only going to work well for "approximately continuous"
     *  types, i.e. floats and doubles, due to the implementation
     */
    template<typename ScalarType, int Rows, typename Allocator = std::allocator<Eigen::Matrix<ScalarType, Rows, 1>>>
    inline Eigen::Matrix<ScalarType, Rows, 1> AverageEigenVector(
            const std::vector<Eigen::Matrix<ScalarType, Rows, 1>, Allocator>& vectors,
            const std::vector<double>& weights = std::vector<double>())
    {
        // Get the weights
        assert(vectors.size() > 0);
        assert((weights.size() == vectors.size()) || (weights.size() == 0));
        const bool use_weights = (weights.size() != 0);
        // Find the first element with non-zero weight
        size_t starting_idx = 0;
        while (starting_idx < weights.size() && weights[starting_idx] == 0.0)
        {
            starting_idx++;
        }
        // If all weights are zero, result is undefined
        assert(starting_idx < vectors.size());
        // Start the recursive definition with the base case
        Eigen::Matrix<ScalarType, Rows, 1> avg_vector = vectors[starting_idx];
        const double starting_weight = use_weights ? std::abs(weights[starting_idx]) : 1.0;
        assert(starting_weight > 0.0);
        double weights_running_sum = starting_weight;
        // Do the weighted averaging on the rest of the vectors
        for (size_t idx = starting_idx + 1; idx < vectors.size(); ++idx)
        {
            const double weight = use_weights ? std::abs(weights[idx]) : 1.0;
            weights_running_sum += weight;
            const double effective_weight = weight / weights_running_sum;
            const Eigen::Matrix<ScalarType, Rows, 1> prev_avg_vector = avg_vector;
            const Eigen::Matrix<ScalarType, Rows, 1>& current = vectors[idx];
            avg_vector = prev_avg_vector + (effective_weight * (current - prev_avg_vector));
        }
        return avg_vector;
    }

    inline Eigen::Vector3d AverageEigenVector3d(
            const EigenHelpers::VectorVector3d& vectors,
            const std::vector<double>& weights = std::vector<double>())
    {
        return AverageEigenVector(vectors, weights);
    }

    inline Eigen::VectorXd AverageEigenVectorXd(
            const std::vector<Eigen::VectorXd>& vectors,
            const std::vector<double>& weights = std::vector<double>())
    {
        return AverageEigenVector(vectors, weights);
    }

    /**
     * Implementation of method described in (http://stackoverflow.com/a/27410865)
     * See paper at (http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf) for full explanation
     */
    inline Eigen::Quaterniond AverageEigenQuaterniond(
            const EigenHelpers::VectorQuaterniond& quaternions,
            const std::vector<double>& weights = std::vector<double>())
    {
        // Get the weights
        const bool use_weights = weights.size() == quaternions.size() ? true : false;
        assert(quaternions.size() > 0);
        assert((weights.size() == quaternions.size()) || (weights.size() == 0));
        // Shortcut the process if there is only 1 quaternion
        if (quaternions.size() == 1)
        {
            assert(weights.size() == 0 || weights[0] != 0.0);
            return quaternions[0];
        }
        // Build the averaging matrix
        Eigen::MatrixXd q_matrix(4, quaternions.size());
        for (size_t idx = 0; idx < quaternions.size(); idx++)
        {
            const double weight = use_weights ? std::abs(weights[idx]) : 1.0;
            const Eigen::Quaterniond& q = quaternions[idx];
            q_matrix.col((ssize_t)idx) << weight * q.w(), weight * q.x(), weight * q.y(), weight * q.z();
        }
        // Make the matrix square
        const Eigen::Matrix<double, 4, 4> qqtranspose_matrix = q_matrix * q_matrix.transpose();
        // Compute the eigenvectors and eigenvalues of the qqtranspose matrix
        const Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>> solver(qqtranspose_matrix);
        const Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>>::EigenvalueType eigen_values = solver.eigenvalues();
        const Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>>::EigenvectorsType eigen_vectors = solver.eigenvectors();
        // Extract the eigenvector corresponding to the largest eigenvalue
        double max_eigenvalue = -INFINITY;
        int64_t max_eigenvector_index = -1;
        for (size_t idx = 0; idx < 4; idx++)
        {
            const double current_eigenvalue = eigen_values((long)idx).real();
            if (current_eigenvalue > max_eigenvalue)
            {
                max_eigenvalue = current_eigenvalue;
                max_eigenvector_index = (int64_t)idx;
            }
        }
        assert(max_eigenvector_index >= 0);
        // Note that these are already normalized!
        const Eigen::Vector4cd best_eigenvector = eigen_vectors.col((long)max_eigenvector_index);
        // Convert back into a quaternion
        const Eigen::Quaterniond average_q(best_eigenvector(0).real(), best_eigenvector(1).real(), best_eigenvector(2).real(), best_eigenvector(3).real());
        return average_q;
    }

    inline Eigen::Isometry3d AverageEigenIsometry3d(
            const EigenHelpers::VectorIsometry3d& transforms,
            const std::vector<double>& weights = std::vector<double>())
    {
        assert(transforms.size() > 0);
        assert((weights.size() == transforms.size()) || (weights.size() == 0));
        // Shortcut the process if there is only 1 transform
        if (transforms.size() == 1)
        {
            assert(weights.size() == 0 || weights[0] != 0.0);
            return transforms[0];
        }
        // Extract components
        EigenHelpers::VectorVector3d translations(transforms.size());
        EigenHelpers::VectorQuaterniond rotations(transforms.size());
        for (size_t idx = 0; idx < transforms.size(); idx++)
        {
            translations[idx] = transforms[idx].translation();
            rotations[idx] = Eigen::Quaterniond(transforms[idx].rotation());
        }
        // Average
        const Eigen::Vector3d average_translation = AverageEigenVector(translations, weights);
        const Eigen::Quaterniond average_rotation = AverageEigenQuaterniond(rotations, weights);
        // Make the average transform
        const Eigen::Isometry3d average_transform = (Eigen::Translation3d)average_translation * average_rotation;
        return average_transform;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Projection/Rejection functions
    ////////////////////////////////////////////////////////////////////////////

    // Projects vector_to_project onto base_vector and returns the portion that is parallel to base_vector
    template <typename DerivedB, typename DerivedV>
    inline Eigen::Matrix<typename DerivedB::Scalar, Eigen::Dynamic, 1> VectorProjection(
            const Eigen::MatrixBase<DerivedB>& base_vector,
            const Eigen::MatrixBase<DerivedV>& vector_to_project)
    {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedB);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedV);
        EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(DerivedB, DerivedV)
        static_assert(std::is_same<typename DerivedB::Scalar, typename DerivedV::Scalar>::value,
                      "base_vector and vector_to_project must have the same data type");
        // Perform projection
        const typename DerivedB::Scalar b_squared_norm = base_vector.squaredNorm();
        if (b_squared_norm > 0)
        {
            return (base_vector.dot(vector_to_project) / b_squared_norm) * base_vector;
        }
        else
        {
            return Eigen::Matrix<typename DerivedB::Scalar, Eigen::Dynamic, 1>::Zero(base_vector.rows());
        }
    }

    // Projects vector_to_project onto base_vector and returns the portion that is perpendicular to base_vector
    template <typename DerivedB, typename DerivedV>
    inline Eigen::Matrix<typename DerivedB::Scalar, Eigen::Dynamic, 1> VectorRejection(
            const Eigen::MatrixBase<DerivedB>& base_vector,
            const Eigen::MatrixBase<DerivedV>& vector_to_reject)
    {
        // Rejection is defined in relation to projection
        return vector_to_reject - VectorProjection(base_vector, vector_to_reject);
    }

    // Intended only for planes in 3-D, not hyperplanes (see Hyperplane class)
    template <typename DerivedB1, typename DerivedB2, typename DerivedV>
    inline Eigen::Vector3d VectorProjectionToPlane(
            const Eigen::MatrixBase<DerivedB1>& plane_vector1,
            const Eigen::MatrixBase<DerivedB2>& plane_vector2,
            const Eigen::MatrixBase<DerivedV>& vector)
    {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedB1);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedB2);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedV);
        EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(DerivedB1, DerivedV);
        EIGEN_STATIC_ASSERT_SAME_VECTOR_SIZE(DerivedB2, DerivedV);

        const Eigen::Vector3d unit_plane_vector1 = plane_vector1.normalized();
        const Eigen::Vector3d unit_plane_vector2 = plane_vector2.normalized();

        // Error/numerical problems check input
        const double plane_vector_dot_product_mag = std::abs(unit_plane_vector1.dot(unit_plane_vector2));
        if (IsApprox(plane_vector_dot_product_mag , 1.0, 1e-10))
        {
            throw_arc_exception(std::invalid_argument, "Plane vectors do not define a valid plane");
        }

        // Get the normal to the plane, then reject any component of the vector that is parallel
        const Eigen::Vector3d normal = unit_plane_vector1.cross(unit_plane_vector2);
        return VectorRejection(normal, vector);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Geometry functions
    ////////////////////////////////////////////////////////////////////////////

    /**
     * @brief DistanceToLine
     * Math taken from http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
     * x = x0
     * point_on_line = x1
     * unit_vector = x2 - x1 / |x2 - x1|
     * @param point_on_line
     * @param unit_vector
     * @param x
     * @return The distance to the line, and the displacement along the line
     */
    inline std::pair<double, double> DistanceToLine(
            const Eigen::Vector3d& point_on_line,
            const Eigen::Vector3d& unit_vector,
            const Eigen::Vector3d& x)
    {
        // Ensure that our input data is valid
        const auto real_unit_vector = unit_vector.normalized();
        if (!CloseEnough(unit_vector.norm(), 1.0, 1e-13))
        {
            std::cerr << "[Distance to line]: unit vector was not normalized: "
                      << unit_vector.transpose() << " Norm: " << unit_vector.norm() << std::endl;
        }

        const auto delta = x - point_on_line;
        const double displacement_along_line = real_unit_vector.dot(delta);
        const auto x_projected_onto_line = point_on_line + real_unit_vector * displacement_along_line;
        const double distance_to_line = (x_projected_onto_line - x).norm();

        // A simple neccescary (but not sufficient) check to look for math errors
        assert(IsApprox(distance_to_line * distance_to_line +
                        displacement_along_line * displacement_along_line, delta.squaredNorm(), 1e-10));

        return std::make_pair(distance_to_line, displacement_along_line);
    }

    // We want to constrain all vectors "r" to lie within a specified angle of the cone direction.
    // I.e. cone_direction.transpose() * r / norm(r) >= cos(angle)
    // or cone_direction.transpose() * r_normalized >= min_normalized_dot_product
    // It is assumed that cone_direction is already normalized
    // Returns the normal vectors that point out of a pyramid approximation of the cone
    inline VectorVector3d ConvertConeToPyramid(
            const Eigen::Vector3d& cone_direction,
            const double min_normalized_dot_product)
    {
        // Build a vector that is garunteed to be perpendicular to cone_direction, and non-zero
        auto tmp = VectorRejection(cone_direction, Eigen::Vector3d::UnitX());
        tmp += VectorRejection(cone_direction, Eigen::Vector3d::UnitY());
        tmp += VectorRejection(cone_direction, Eigen::Vector3d::UnitZ());

        assert(tmp.norm() > 1e-6);
        tmp.normalize();

        const Eigen::Vector3d p1 = tmp;
        const Eigen::Vector3d p2 = cone_direction.cross(p1).normalized();
        const Eigen::Vector3d p3 = -p1;
        const Eigen::Vector3d p4 = -p2;

        const double theta_max = std::acos(min_normalized_dot_product);
        const double dist = std::tan(theta_max);

        const Eigen::Vector3d ray1 = cone_direction + dist * p1;
        const Eigen::Vector3d ray2 = cone_direction + dist * p2;
        const Eigen::Vector3d ray3 = cone_direction + dist * p3;
        const Eigen::Vector3d ray4 = cone_direction + dist * p4;

        VectorVector3d normals(4);
        normals[0] = -ray1.cross(ray2).normalized();
        normals[1] = -ray2.cross(ray3).normalized();
        normals[2] = -ray3.cross(ray4).normalized();
        normals[3] = -ray4.cross(ray1).normalized();

        return normals;
    }

    ////////////////////////////////////////////////////////////////////////////
    // (Weighted) dot product, norm, and angle functions
    ////////////////////////////////////////////////////////////////////////////

    // Returns the (non-negative) angle defined by the vectors (b - a), and (b - c)
    template <typename DerivedA, typename DerivedB, typename DerivedC>
    inline double AngleDefinedByPoints(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedB>& b, const Eigen::MatrixBase<DerivedC>& c)
    {
        // Check for potential numerical problems
        if (a.isApprox(b) || (b.isApprox(c)))
        {
            std::cerr << "Warning: Potential numerical stability problems in AngleDefinedByPoints\n";
        }

        // Do the actual math here
        const auto vec1 = (a - b).normalized();
        const auto vec2 = (c - b).normalized();
        const double cosine_raw = vec1.dot(vec2);
        const double cosine = std::max(-1.0, std::min(cosine_raw, 1.0));
        return std::acos(cosine);
    }

    inline double WeightedDotProduct(const Eigen::VectorXd& vec1, const Eigen::VectorXd& vec2, const Eigen::VectorXd& weights)
    {
        return vec1.cwiseProduct(weights).dot(vec2);
    }

    inline double WeightedSquaredNorm(const Eigen::VectorXd& vec, const Eigen::VectorXd weights)
    {
        return WeightedDotProduct(vec, vec, weights);
    }

    inline double WeightedNorm(const Eigen::VectorXd& vec, const Eigen::VectorXd& weights)
    {
        return std::sqrt(WeightedSquaredNorm(vec, weights));
    }

    inline double WeightedCosineAngleBetweenVectors(const Eigen::VectorXd& vec1, const Eigen::VectorXd& vec2, const Eigen::VectorXd& weights)
    {
        const double vec1_norm = WeightedNorm(vec1, weights);
        const double vec2_norm = WeightedNorm(vec2, weights);
        assert(vec1_norm > 0 && vec2_norm > 0);
        const double result = WeightedDotProduct(vec1, vec2, weights) / (vec1_norm * vec2_norm);
        return std::max(-1.0, std::min(result, 1.0));
    }

    inline double WeightedAngleBetweenVectors(const Eigen::VectorXd& vec1, const Eigen::VectorXd& vec2, const Eigen::VectorXd& weights)
    {
        return std::acos(WeightedCosineAngleBetweenVectors(vec1, vec2, weights));
    }

    ////////////////////////////////////////////////////////////////////////////
    // Other auxiliary functions
    ////////////////////////////////////////////////////////////////////////////

    class Hyperplane
    {
    protected:

        Eigen::VectorXd plane_origin_;
        Eigen::VectorXd plane_normal_;

    public:

        Hyperplane(const Eigen::VectorXd& origin, const Eigen::VectorXd& normal)
        {
            assert(origin.size() == normal.size());
            plane_origin_ = origin;
            plane_normal_ = normal;
        }

        Hyperplane() {}

        size_t GetDimensionality() const
        {
            return (size_t)plane_origin_.size();
        }

        const Eigen::VectorXd& GetOrigin() const
        {
            return plane_origin_;
        }

        const Eigen::VectorXd& GetNormal() const
        {
            return plane_normal_;
        }

        double GetNormedDotProduct(const Eigen::VectorXd& point) const
        {
            assert(point.size() == plane_origin_.size());
            const Eigen::VectorXd check_vector = point - plane_origin_;
            const Eigen::VectorXd check_vector_normed = EigenHelpers::SafeNormal(check_vector);
            const double dot_product = check_vector_normed.dot(plane_normal_);
            return dot_product;
        }

        double GetRawDotProduct(const Eigen::VectorXd& point) const
        {
            assert(point.size() == plane_origin_.size());
            const Eigen::VectorXd check_vector = point - plane_origin_;
            const double dot_product = check_vector.dot(plane_normal_);
            return dot_product;
        }

        Eigen::VectorXd RejectVectorOntoPlane(const Eigen::VectorXd& vector) const
        {
            return VectorProjection(plane_normal_, vector);
        }

        double GetSquaredDistanceToPlane(const Eigen::VectorXd& point) const
        {
            const Eigen::VectorXd origin_to_point_vector = point - plane_origin_;
            return VectorProjection(plane_normal_, origin_to_point_vector).squaredNorm();
        }

        double GetDistanceToPlane(const Eigen::VectorXd& point) const
        {
            const Eigen::VectorXd origin_to_point_vector = point - plane_origin_;
            return VectorProjection(plane_normal_, origin_to_point_vector).norm();
        }

        Eigen::VectorXd ProjectVectorOntoPlane(const Eigen::VectorXd& vector) const
        {
            return VectorRejection(plane_normal_, vector);
        }

        Eigen::VectorXd ProjectPointOntoPlane(const Eigen::VectorXd& point) const
        {
            const Eigen::VectorXd origin_to_point_vector = point - plane_origin_;
            const Eigen::VectorXd projected_to_point_vector = VectorRejection(plane_normal_, origin_to_point_vector);
            const Eigen::VectorXd projected_point = plane_origin_ + projected_to_point_vector;
            return projected_point;
        }
    };

    /*
     * Returns a pair of <centroid point, normal vector> defining the plane
     */
    inline Hyperplane FitPlaneToPoints(const std::vector<Eigen::VectorXd>& points)
    {
        // Subtract out the centroid
        const Eigen::VectorXd centroid = EigenHelpers::AverageEigenVectorXd(points);
        Eigen::MatrixXd centered_points(centroid.size(), points.size());
        for (size_t idx = 0; idx < points.size(); idx++)
        {
            const Eigen::VectorXd& current_point = points[idx];
            centered_points.block(0, (ssize_t)idx, centroid.size(), 1) = (current_point - centroid);
        }
        // Compute SVD of the centered points
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered_points, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // Get results of SVD
        const Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType& singular_values = svd.singularValues();
        const Eigen::JacobiSVD<Eigen::MatrixXd>::MatrixUType& u_matrix = svd.matrixU();
        // Get the left singular vector corresponding to the minimum singular value
        double minimum_singular_value = INFINITY;
        ssize_t best_singular_value_index = -1;
        for (ssize_t idx = 0; idx < singular_values.size(); idx++)
        {
            const std::complex<double> current_singular_value = singular_values(idx);
            if (current_singular_value.real() < minimum_singular_value)
            {
                minimum_singular_value = current_singular_value.real();
                best_singular_value_index = idx;
            }
        }
        assert(best_singular_value_index >= 0);
        // The corresponding left singular vector is the normal vector of the best-fit plane
        const Eigen::VectorXd best_left_singular_vector = u_matrix.col(best_singular_value_index);
        const Eigen::VectorXd normal_vector = EigenHelpers::SafeNormal(best_left_singular_vector);
        return Hyperplane(centroid, normal_vector);
    }

    inline double SuggestedRcond()
    {
        return 0.001;
    }

    // Derived from code by Yohann Solaro ( http://listengine.tuxfamily.org/lists.tuxfamily.org/eigen/2010/01/msg00187.html )
    // see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method
    inline Eigen::MatrixXd Pinv(const Eigen::MatrixXd& b, const double rcond, const bool enable_flip=true)
    {
        bool flip = false;
        Eigen::MatrixXd a;
        if (enable_flip && (b.rows() < b.cols()))
        {
            a = b.transpose();
            flip = true;
        }
        else
        {
            a = b;
        }
        // SVD
        Eigen::JacobiSVD<Eigen::MatrixXd> svdA;
        svdA.compute(a, Eigen::ComputeFullU | Eigen::ComputeThinV);
        Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType vSingular = svdA.singularValues();
        // Build a diagonal matrix with the Inverted Singular values
        // The pseudo inverted singular matrix is easy to compute :
        // is formed by replacing every nonzero entry by its reciprocal (inversing).
        Eigen::VectorXd vPseudoInvertedSingular(svdA.matrixV().cols());
        for (int iRow = 0; iRow < vSingular.rows(); iRow++)
        {
            if (std::abs(vSingular(iRow)) <= rcond) // Todo : Put epsilon in parameter
            {
                vPseudoInvertedSingular(iRow)= 0.0;
            }
            else
            {
                vPseudoInvertedSingular(iRow) = 1.0 / vSingular(iRow);
            }
        }
        // A little optimization here
        const Eigen::MatrixXd mAdjointU = svdA.matrixU().adjoint().block(0, 0, vSingular.rows(), svdA.matrixU().adjoint().cols());
        // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wconversion"
        // Pseudo-Inversion : V * S * U'
        const Eigen::MatrixXd a_pinv = (svdA.matrixV() * vPseudoInvertedSingular.asDiagonal()) * mAdjointU;
        #pragma GCC diagnostic pop
        // Flip back if need be
        if (flip)
        {
            return a_pinv.transpose();
        }
        else
        {
            return a_pinv;
        }
    }

    /**
     * @brief WeightedLeastSquaresSolver Solves the minimization problem min || Ax - b ||^2 for x, using weights w in the norm
     *                                   If the problem is ill-conditioned, adds in a damping factor. This is equivalent to
     *                                   solving A^T * diag(W) * A * x = A^T * diag(W) * b for x.
     * @param A size M x N with M > N
     * @param b size M x 1
     * @param w size M x 1
     * @param damping_threshold The smallest singular value we allow in A^T * W * A before we apply damping
     * @param damping_value The damping value we apply to the main diagonal of A^T * W * A if we exceed the threshold
     * @return size N x 1
     */
    inline Eigen::VectorXd WeightedLeastSquaresSolver(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const Eigen::VectorXd& w, const double damping_threshold, const double damping_value)
    {
        // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wconversion"
        Eigen::MatrixXd lhs = A.transpose() * w.asDiagonal() * A;
        #pragma GCC diagnostic pop
        const double minimum_singular_value = lhs.jacobiSvd().singularValues().minCoeff();

        if (minimum_singular_value < damping_threshold)
        {
            lhs += damping_value * Eigen::MatrixXd::Identity(lhs.rows(), lhs.cols());
        }

        // With the damping we can assume that the left side is positive definite, so use LLT to solve this
        return lhs.llt().solve(A.transpose() * w.cwiseProduct(b));
    }

    inline Eigen::VectorXd UnderdeterminedSolver(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const double damping_threshold, const double damping_value)
    {
        assert(A.cols() > A.rows());
        // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wconversion"
        Eigen::MatrixXd damped = A * A.transpose();
        #pragma GCC diagnostic pop
        const double minimum_singular_value = damped.jacobiSvd().singularValues().minCoeff();

        if (minimum_singular_value < damping_threshold)
        {
            damped += damping_value * Eigen::MatrixXd::Identity(damped.rows(), damped.cols());
        }

        // With the damping we can assume that what we are inverting is positive definite, so use LLT to solve this
        return A.transpose() * damped.llt().solve(b);
    }
}


#endif // EIGEN_HELPERS_HPP
