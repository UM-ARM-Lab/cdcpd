#ifndef GUROBI_SOLVERS_H
#define GUROBI_SOLVERS_H

#include <Eigen/Dense>
#include <arc_utilities/eigen_typedefs.hpp>

namespace smmap
{
    Eigen::VectorXd minSquaredNorm_MaxXNormConstraint(
            const Eigen::MatrixXd& A,
            const Eigen::VectorXd& b,
            const double max_x_norm);

    Eigen::VectorXd minSquaredNorm_MaxXNormConstraint(
            const Eigen::MatrixXd& A,
            const Eigen::VectorXd& b,
            const Eigen::VectorXd& weights,
            const double max_x_norm);

    Eigen::VectorXd minSquaredNorm_MaxXNormConstraint_LinearConstraints(
            const Eigen::MatrixXd& A,
            const Eigen::VectorXd& b,
            const Eigen::VectorXd& weights,
            const double max_x_norm,
            const std::vector<Eigen::RowVectorXd>& linear_constraint_linear_terms,
            const std::vector<double>& linear_constraint_affine_terms,
            const Eigen::VectorXd& x_lower_bound = Eigen::VectorXd(0),
            const Eigen::VectorXd& x_upper_bound = Eigen::VectorXd(0));

    Eigen::VectorXd minSquaredNorm_SE3VelocityConstraints(
            const Eigen::MatrixXd& A,
            const Eigen::VectorXd& b,
            const Eigen::VectorXd& weights,
            const double max_se3_velocity);

    Eigen::VectorXd minSquaredNorm_SE3VelocityConstraints_LinearConstraints(
            const Eigen::MatrixXd& A,
            const Eigen::VectorXd& b,
            const Eigen::VectorXd& weights,
            const double max_se3_velocity,
            const std::vector<Eigen::RowVectorXd>& linear_constraint_linear_terms,
            const std::vector<double>& linear_constraint_affine_terms);

    Eigen::VectorXd minSquaredNorm_SE3VelocityConstraints_QuadraticConstraints(
            const Eigen::MatrixXd& A,
            const Eigen::VectorXd& b,
            const Eigen::VectorXd& weights,
            const Eigen::MatrixXd& J,
            const double max_se3_velocity,
            const std::vector<Eigen::RowVectorXd>& linear_constraint_linear_terms,
            const std::vector<double>& linear_constraint_affine_terms,
            const std::vector<Eigen::MatrixXd>& quadratic_constraint_quadratic_terms,
            const std::vector<Eigen::RowVectorXd>& quadratic_constraint_linear_terms,
            const std::vector<double>& quadratic_constraint_affine_terms,
            const Eigen::VectorXd& x_lower_bound = Eigen::VectorXd(0),
            const Eigen::VectorXd& x_upper_bound = Eigen::VectorXd(0));

    Eigen::VectorXd minXNorm_LinearConstraints(
            const std::vector<Eigen::RowVectorXd>& linear_constraint_linear_terms,
            const std::vector<double>& linear_constraint_affine_terms,
            const Eigen::VectorXd& x_weights,
            const Eigen::VectorXd& x_lower_bound = Eigen::VectorXd(0),
            const Eigen::VectorXd& x_upper_bound = Eigen::VectorXd(0));

    std::pair<Eigen::VectorXd, double> minimizeConstraintViolations(
            const ssize_t num_vars,
            const std::vector<Eigen::RowVectorXd>& linear_constraint_linear_terms,
            const std::vector<double>& linear_constraint_affine_terms,
            const double max_x_norm,
            const Eigen::VectorXd& x_lower_bound = Eigen::VectorXd(0),
            const Eigen::VectorXd& x_upper_bound = Eigen::VectorXd(0),
            const double constraint_lower_bound = -1e3,
            const double constraint_upper_bound = 1e3);

    Eigen::VectorXd findClosestValidPoint(
            const Eigen::VectorXd& starting_point,
            const std::vector<Eigen::RowVectorXd>& linear_constraint_linear_terms,
            const std::vector<double>& linear_constraint_affine_terms,
            const double max_x_norm,
            const Eigen::VectorXd& x_lower_bound = Eigen::VectorXd(0),
            const Eigen::VectorXd& x_upper_bound = Eigen::VectorXd(0));

    Eigen::VectorXd findClosestValidPoint(
            const Eigen::VectorXd& starting_point,
            const Eigen::MatrixXd& J,
            const double max_se3_velocity,
            const std::vector<Eigen::RowVectorXd>& linear_constraint_linear_terms,
            const std::vector<double>& linear_constraint_affine_terms,
            const std::vector<Eigen::MatrixXd>& quadratic_constraint_quadratic_terms,
            const std::vector<Eigen::RowVectorXd>& quadratic_constraint_linear_terms,
            const std::vector<double>& quadratic_constraint_affine_terms,
            const Eigen::VectorXd& x_lower_bound = Eigen::VectorXd(0),
            const Eigen::VectorXd& x_upper_bound = Eigen::VectorXd(0));

    EigenHelpers::VectorVector3d denoiseWithDistanceConstraints(
            const EigenHelpers::VectorVector3d& observations,
            const Eigen::VectorXd& observation_strength,
            const Eigen::MatrixXd& distance_sq_constraints,
            const double variable_bound = 1000.0);
}

#endif // GUROBI_SOLVERS_H
