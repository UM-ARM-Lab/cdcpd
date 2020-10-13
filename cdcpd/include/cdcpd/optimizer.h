#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <algorithm>

#include <Eigen/Dense>
#include <gurobi_c++.h>
#include "cdcpd.h"


////////////////////////////////////////////////////////////////
// Internally used static objects
////////////////////////////////////////////////////////////////

// std::tuple<Eigen::Matrix3Xf, Eigen::Matrix3Xf>
// nearest_points_and_normal(const Eigen::Matrix3Xf& last_template);

// std::tuple<Eigen::Matrix3Xf, Eigen::Matrix3Xf>
//    nearest_points_line_segments(const Eigen::Matrix3Xf& last_template, const Eigen::Matrix2Xi& E);

void Wsolver(const Eigen::MatrixXf& P, const Eigen::Matrix3Xf& X, const Eigen::Matrix3Xf& Y, const Eigen::MatrixXf& G, const Eigen::MatrixXf& L, const double sigma2, const double alpha, const double lambda, Eigen::MatrixXf& W);

class Optimizer
{
public:
    Optimizer(const Eigen::Matrix3Xf _init_temp, const Eigen::Matrix3Xf _last_temp, const float _stretch_lambda);

    Eigen::Matrix3Xf operator()(const Eigen::Matrix3Xf& Y,
                                const Eigen::Matrix2Xi& E,
                                const std::vector<CDCPD::FixedPoint>& fixed_points,
                                const bool self_intersection = true,
                                const bool interation_constrain = true);
private:
    bool all_constraints_satisfiable(const std::vector<CDCPD::FixedPoint>& fixed_points) const;

    Eigen::Matrix3Xf initial_template;
    Eigen::Matrix3Xf last_template;
    float stretch_lambda;
};

#endif