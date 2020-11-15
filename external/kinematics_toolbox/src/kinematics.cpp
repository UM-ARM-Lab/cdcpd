#include "kinematics_toolbox/kinematics.h"

using namespace kinematics;

////////////////////////////////////////////////////////////////////////////////
// Basic Transforms
////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix4d kinematics::rotX(const double theta)
{
    Eigen::Matrix4d R;
    R << 1,                0,                0, 0,
         0,  std::cos(theta), -std::sin(theta), 0,
         0,  std::sin(theta),  std::cos(theta), 0,
         0,                0,                0, 1;
    return R;
}

Eigen::Matrix4d kinematics::rotY(const double theta)
{
    Eigen::Matrix4d R;
    R <<  std::cos(theta), 0, std::sin(theta), 0,
                        0, 1,               0, 0,
         -std::sin(theta), 0, std::cos(theta), 0,
                        0, 0,               0, 1;
    return R;
}

Eigen::Matrix4d kinematics::rotZ(const double theta)
{
    Eigen::Matrix4d R;
    R << std::cos(theta), -std::sin(theta), 0, 0,
         std::sin(theta),  std::cos(theta), 0, 0,
                       0,                0, 1, 0,
                       0,                0, 0, 1;
    return R;
}

Eigen::Matrix4d kinematics::translate(const Eigen::Vector3d& p)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 1>(0, 3) = p;
    return T;
}

Eigen::Matrix4d kinematics::transX(const double x)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = x;
    return T;
}

Eigen::Matrix4d kinematics::transY(const double y)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(1, 3) = y;
    return T;
}

Eigen::Matrix4d kinematics::transZ(const double z)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(2, 3) = z;
    return T;
}

////////////////////////////////////////////////////////////////////////////////
// Skew and Unskew for 3x3 matrices
////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix3d kinematics::skew(const Eigen::Vector3d& w)
{
    Eigen::Matrix3d w_hat;

    w_hat <<     0, -w(2),  w(1),
              w(2),     0, -w(0),
             -w(1),  w(0),     0;

    return w_hat;
}

Eigen::Vector3d kinematics::unskew(const Eigen::Matrix3d& w_hat)
{
    Eigen::Vector3d w;
    Eigen::Matrix3d w_hat_sym;
    w_hat_sym = (w_hat - w_hat.transpose()) / 2;

    w << w_hat_sym(2, 1), w_hat_sym(0, 2), w_hat_sym(1, 0);

    return w;
}


////////////////////////////////////////////////////////////////////////////////
// Twist create, cacluting twists, twist hat and unhat
////////////////////////////////////////////////////////////////////////////////

Vector6d kinematics::createTwist(const Eigen::Vector3d& omega, const Eigen::Vector3d& q)
{
    Vector6d xi;

    if (omega(0) == 0 && omega(1) == 0 && omega(2) == 0)
    {
        xi.segment<3>(0) = q;
        xi.segment<3>(3) = omega;
    }
    else
    {
        xi.segment<3>(0) = -omega.cross(q);
        xi.segment<3>(3) = omega;
    }

    return xi;
}

std::vector<Vector6d> kinematics::createTwist(const std::vector<Eigen::Vector3d>& omega,
                                               const std::vector<Eigen::Vector3d>& q)
{
    std::vector<Vector6d> xi(omega.size());

    for (size_t i = 0; i < omega.size(); i++)
    {
        xi[i] = createTwist(omega[i], q[i]);
    }

    return xi;
}


std::vector<Vector6d> kinematics::calculateTwists(const Eigen::Matrix4d& g_base,
                                                   const std::vector<Eigen::Vector3d>& omega0,
                                                   const std::vector<Eigen::Vector3d>& q0)
{
    // TODO: is this the same thing as adj(g_base)*twist?
    std::vector<Vector6d> xi(q0.size());

    for (size_t i = 0; i < q0.size(); i++)
    {
        Eigen::Vector4d omega;
        Eigen::Vector4d q;

        omega << omega0[i], 0;
        q     << q0[i], 1;

        omega = g_base*omega;
        q     = g_base*q;

        xi[i] = kinematics::createTwist(omega.segment<3>(0), q.segment<3>(0));
    }

    return xi;
}

Eigen::Matrix4d kinematics::twistHat(const Vector6d& xi)
{
    Eigen::Matrix4d xi_hat = Eigen::Matrix4d::Zero();

    Eigen::Vector3d v = xi.segment<3>(0);
    Eigen::Vector3d w = xi.segment<3>(3);
    Eigen::Matrix3d w_hat = skew(w);

    xi_hat.block< 3, 3>(0, 0) = w_hat;
    xi_hat.block< 3, 1>(0, 3) = v;

    return xi_hat;
}

Vector6d kinematics::twistUnhat(const Eigen::Matrix4d& xi_hat)
{
    Vector6d xi;

    Eigen::Vector3d v = xi_hat.block< 3, 1>(0, 3);
    Eigen::Matrix3d w_hat = xi_hat.block< 3, 3>(0, 0);
    Eigen::Vector3d w = unskew(w_hat);

    xi.segment<3>(0) = v;
    xi.segment<3>(3) = w;

    return xi;
}

////////////////////////////////////////////////////////////////////////////////
// Adjoints and twist exponentials
////////////////////////////////////////////////////////////////////////////////

Matrix6d kinematics::adj(const Eigen::Matrix4d& g)
{
    const Eigen::Matrix3d R = g.block< 3, 3>(0, 0);
    const Eigen::Vector3d p = g.block< 3, 1>(0, 3);
    const Eigen::Matrix3d p_hat = skew(p);

    Matrix6d adj_g;

    adj_g.block< 3, 3>(0, 0) = R;
    adj_g.block< 3, 3>(0, 3) = p_hat*R;
    adj_g.block< 3, 3>(3, 0) = Eigen::Matrix3d::Zero(3, 3);
    adj_g.block< 3, 3>(3, 3) = R;

    return adj_g;
}

/*Matrix6d kinematics::adjinv(const Eigen::Matrix4d& g)
{
    Eigen::Matrix3d R = g.block< 3, 3>(0, 0);
    Eigen::Vector3d p = g.block< 3, 1>(0, 3);
    Eigen::Matrix3d p_hat = skew(p);

    Matrix6d adjinv_g;

    adjinv_g.block<3, 3>(0, 0) = R.transpose();
    adjinv_g.block<3, 3>(0, 3) = -R.transpose()*p_hat;
    adjinv_g.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero(3, 3);
    adjinv_g.block<3, 3>(3, 3) = R.transpose();

    return adjinv_g;
}*/

Eigen::Matrix3d kinematics::expmExact(const Eigen::Matrix3d& w_hat, const double theta)
{
    // w_hat should be normalized before calling this function
    assert(std::abs(unskew(w_hat).norm() - 1) < 1e-10);

    Eigen::Matrix3d expM;
    expM = Eigen::Matrix3d::Identity()
           + w_hat * std::sin(theta)
           + w_hat * w_hat * (1 - std::cos(theta));

    return expM;
}

Eigen::Matrix4d kinematics::expTwist(const Vector6d& xi, double theta)
{
    Eigen::Vector3d v = xi.segment<3>(0);
    Eigen::Vector3d w = xi.segment<3>(3);

    Eigen::Matrix4d expT = Eigen::Matrix4d::Identity();

    if (w(0) == 0 && w(1) == 0 && w(2) == 0)
    {
        expT.block<3, 1>(0, 3) = v * theta;
    }
    else
    {
        double norm_w = w.norm();

        // rescale to make norm(w) == 1
        theta *= norm_w;
        w /= norm_w;
        v /= norm_w;

        Eigen::Matrix3d w_hat = skew(w);
        Eigen::Matrix3d exp_w_hat_theta = expmExact(w_hat, theta);

        expT.block<3, 3>(0, 0) = exp_w_hat_theta;
        expT.block<3, 1>(0, 3) =
                (Eigen::Matrix3d::Identity() - exp_w_hat_theta) * w.cross(v)
                + w * w.transpose() * v * theta;
    }

    return expT;
}

Eigen::Isometry3d kinematics::expTwistIsometry3d(const Vector6d& xi, double theta)
{
    // TODO: this better
    const Eigen::Matrix4d tmp = expTwist(xi, theta);
    Eigen::Isometry3d expT;
    expT = tmp.matrix();
    return expT;
}

Eigen::Matrix4d kinematics::expTwist(const std::vector<Vector6d>& xi,
                                     const std::vector<double>& theta)
{
    Eigen::Matrix4d g = Eigen::Matrix4d::Identity();

    for (size_t i = 0; i < theta.size(); i++)
    {
        g = g * expTwist(xi[i], theta[i]);
    }

    return g;
}

Eigen::Isometry3d kinematics::expTwistIsometry3d(const std::vector<Vector6d>& xi,
                                             const std::vector<double>& theta)
{
    Eigen::Isometry3d g = Eigen::Isometry3d::Identity();

    for (size_t i = 0; i < theta.size(); i++)
    {
        g = g * expTwistIsometry3d(xi[i], theta[i]);
    }

    return g;
}

VectorIsometry3d kinematics::applyTwist(const VectorIsometry3d& starting_pose,
                                      const VectorVector6d& xi,
                                      std::vector<double> theta)
{
    const size_t num_poses = starting_pose.size();
    if (theta.size() == 0)
    {
        theta.resize(num_poses, 1.0);
    }

    assert(num_poses == xi.size());
    assert(num_poses == theta.size());

    VectorIsometry3d result(num_poses);
    for (size_t pose_ind = 0; pose_ind < num_poses; pose_ind++)
    {
        result[pose_ind] = starting_pose[pose_ind] *
                expTwistIsometry3d(xi[pose_ind], theta[pose_ind]);
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////
// Jacobians
////////////////////////////////////////////////////////////////////////////////

Matrix6Xd kinematics::spatialJacobian(const std::vector<Vector6d>& xi,
                                      const std::vector<double>& theta)
{
    const size_t num_theta = theta.size();
    Matrix6Xd J_s(6, num_theta);

    Eigen::Matrix4d g = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d expT;

    for(size_t i = 0; i < num_theta; i++)
    {
        if (i != 0)
        {
            expT = expTwist(xi[i - 1], theta[i - 1]);
            g = g * expT;
            J_s.block<6, 1>(0, (ssize_t)i) = adj(g) * xi[i];
        }
        else
        {
            J_s.block<6, 1>(0, (ssize_t)i) = xi[i];
        }
    }

    return J_s;
}

Matrix6Xd kinematics::bodyJacobian(const std::vector<Vector6d>& xi,
                                   const std::vector<double>& theta,
                                   const Eigen::Matrix4d& g_zero)
{
    const ssize_t num_theta = (ssize_t)theta.size();
    Matrix6Xd J_b(6, num_theta);

    Eigen::Matrix4d g = g_zero;

    Eigen::Matrix4d expT;

    for(ssize_t i = num_theta - 1; i >= 0; i--)
    {
        expT = expTwist(xi[(size_t)i], theta[(size_t)i]);
        g = expT * g;
        J_b.block<6,1>(0, i) = adj(g.inverse()) * xi[(size_t)i];
    }

    return J_b;
}

Eigen::MatrixXd kinematics::dampedPinv6Xd(const kinematics::Matrix6Xd& J,
                                          const std::vector<double>& theta,
                                          const double theta_limit,
                                          const double limit_threshold,
                                          const double manipubility_threshold,
                                          const double damping_ratio)
{
    const size_t num_joints = theta.size();

    // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wconversion"
    const kinematics::Matrix6d JJtranspose = J * J.transpose();
    #pragma GCC diagnostic pop
    const kinematics::Matrix6d W_x = kinematics::Matrix6d::Identity();
    Eigen::MatrixXd W_q = Eigen::MatrixXd::Identity((ssize_t)num_joints, (ssize_t)num_joints);

    // Determine least-squares damping and weights, from:
    // "Robust Inverse Kinematics Using Damped Least Squares
    // with Dynamic Weighting"  NASA 1994
    for(size_t i = 1; i < num_joints; i++)
    {
        double theta_to_limit = theta_limit - std::abs(theta[i]);

        if (theta_to_limit < limit_threshold)
        {
            W_q((ssize_t)i, (ssize_t)i) = 0.1 + 0.9 * theta_to_limit / limit_threshold;
        }
    }

    // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wconversion"
    const kinematics::Matrix6Xd J_w = W_x * J * W_q;
    #pragma GCC diagnostic pop

    // find the damping ratio
    // Based on Manipulability, in 'Prior Work' of above paper
    double manipubility = JJtranspose.determinant();
    double damping = 0;
    if (manipubility < manipubility_threshold)
    {
        damping = damping_ratio * std::pow(1 - manipubility / manipubility_threshold, 2);
    }

    const kinematics::Matrix6d tmp = JJtranspose + damping * kinematics::Matrix6d::Identity();
    const Eigen::MatrixXd J_inv = J_w.transpose() * tmp.inverse();

    return W_q * J_inv * W_x;
}

Eigen::MatrixXd kinematics::dampedPinvXd(const Eigen::MatrixXd& J,
                                         const std::vector<double>& theta,
                                         const double theta_limit,
                                         const double limit_threshold,
                                         const double manipubility_threshold,
                                         const double damping_ratio)
{
    const ssize_t num_joints = (ssize_t)theta.size();
    const ssize_t num_velocities = J.rows();

    // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wconversion"
    const kinematics::Matrix6d JJtranspose = J * J.transpose();
    #pragma GCC diagnostic pop
    const kinematics::Matrix6d W_x = kinematics::Matrix6d::Identity();
    Eigen::MatrixXd W_q = Eigen::MatrixXd::Identity(num_joints, num_joints);

    // Determine least-squares damping and weights, from:
    // "Robust Inverse Kinematics Using Damped Least Squares
    // with Dynamic Weighting"  NASA 1994
    for(ssize_t i = 1; i < num_joints; i++)
    {
        double theta_to_limit = theta_limit - std::abs(theta[(size_t)i]);

        if (theta_to_limit < limit_threshold)
        {
            W_q(i, i) = 0.1 + 0.9 * theta_to_limit / limit_threshold;
        }
    }

    // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wconversion"
    const Eigen::MatrixXd J_w = W_x * J * W_q;
    #pragma GCC diagnostic pop

    // find the damping ratio
    // Based on Manipulability, in 'Prior Work' of above paper
    double manipubility = JJtranspose.determinant();
    double damping = 0;
    if (manipubility < manipubility_threshold)
    {
        damping = damping_ratio * std::pow(1 - manipubility / manipubility_threshold, 2);
    }

    const Eigen::MatrixXd tmp = JJtranspose + damping * Eigen::MatrixXd::Identity(num_velocities, num_velocities);
    // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wconversion"
    const Eigen::MatrixXd J_inv = J_w.transpose() * tmp.inverse();
    #pragma GCC diagnostic pop

    return W_q * J_inv * W_x;
}

////////////////////////////////////////////////////////////////////////////////
// Other
////////////////////////////////////////////////////////////////////////////////

Vector6d kinematics::calculateError(const Eigen::Isometry3d& g_current,
                                    const Eigen::Isometry3d& g_desired)
{
    Vector6d xi;

    Eigen::Isometry3d g_diff = g_current.inverse() * g_desired;

    xi = twistUnhat(g_diff.matrix().log());

    return xi;
}

Vector6d kinematics::calculateError(const Eigen::Matrix4d& g_current,
                                    const Eigen::Matrix4d& g_desired)
{
    Vector6d xi;

    // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wconversion"
    const Eigen::Matrix4d g_diff = g_current.inverse() * g_desired;
    #pragma GCC diagnostic pop

    xi = twistUnhat(g_diff.log());

    return xi;
}

Vector6d kinematics::calculateVelocity(const Eigen::Isometry3d& g_current,
                                       const Eigen::Isometry3d& g_next,
                                       const double dt)
{
    Vector6d xi;

    const Eigen::Isometry3d g_diff = g_current.inverse() * g_next;

    xi = twistUnhat(g_diff.matrix().log());

    return xi/dt;
}

VectorVector6d kinematics::calculateVelocities(const VectorIsometry3d& g_trajectory,
                                               const double dt)
{
    assert(g_trajectory.size() >= 2);

    VectorVector6d xi(g_trajectory.size() - 1);

    for (size_t i = 0; i < xi.size() ; i++)
    {
        xi[i] = calculateVelocity(g_trajectory[i], g_trajectory[i + 1], dt);
    }

    return xi;
}

Eigen::VectorXd kinematics::calulateDeltas(const VectorIsometry3d& g)
{
    assert(g.size() >= 1);

    Eigen::VectorXd delta(6 * (ssize_t)g.size());

    for (long ind = 0; ind < (long)g.size(); ind++)
    {
        delta.segment<6>(ind * 6) = twistUnhat(g[(size_t)ind].matrix().log());
    }

    return delta;
}
