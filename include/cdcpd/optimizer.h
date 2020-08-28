#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <algorithm>

#include <Eigen/Dense>
#include <gurobi_c++.h>
#include "cdcpd.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/AABB_tree.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel             K;
typedef K::Point_3                                                      Point_3;
typedef CGAL::Surface_mesh<Point_3>                                     Mesh;


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
    #ifdef SHAPE_COMP
    Optimizer(const Eigen::Matrix3Xf _init_temp, const Eigen::Matrix3Xf _last_temp, const float _stretch_lambda, const obsParam& obstacle_param);
    #else
    Optimizer(const Eigen::Matrix3Xf _init_temp, const Eigen::Matrix3Xf _last_temp, const float _stretch_lambda);
    #endif

    Eigen::Matrix3Xf operator()(const Eigen::Matrix3Xf& Y,
                                const Eigen::Matrix2Xi& E,
                                const std::vector<CDCPD::FixedPoint>& fixed_points,
                                const bool self_intersection = true,
                                const bool interation_constrain = true);
private:
    bool all_constraints_satisfiable(const std::vector<CDCPD::FixedPoint>& fixed_points) const;
    #ifdef SHAPE_COMP
	Mesh initObstacle(obsParam obs_param);
    #endif
	std::tuple<Eigen::Matrix3Xf, Eigen::Matrix3Xf> nearest_points_and_normal(const Eigen::Matrix3Xf& last_template);
	
    Eigen::Matrix3Xf initial_template;
    Eigen::Matrix3Xf last_template;
    float stretch_lambda;
    #ifdef SHAPE_COMP
	const Eigen::Matrix3Xf obs_mesh;
	const Eigen::Matrix3Xf obs_normal;
	const Mesh mesh;
    #endif
};

#endif
