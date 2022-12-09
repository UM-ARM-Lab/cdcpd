#pragma once

#include <Eigen/Dense>
#include <ros/console.h>

#include "cdcpd/tracking_map.h"

using Eigen::Matrix3Xf;
using Eigen::MatrixXf;
using Eigen::RowVectorXf;
using Eigen::VectorXf;

class CPDInterface
{
public:
    CPDInterface(std::string const log_name_base, double const tolerance, int const max_iterations,
        double const initial_sigma_scale, double const w, double const alpha, double const beta,
        double const zeta, double const start_lambda, MatrixXf const m_lle);
    virtual ~CPDInterface(){}

    // TODO(Dylan): Clean up passing in the tracking_map. This is passing in way too much information.
    virtual Matrix3Xf operator()(const Matrix3Xf &X, const Matrix3Xf &Y, const Matrix3Xf &Y_pred,
        const Eigen::VectorXf &Y_emit_prior, TrackingMap const& tracking_map) = 0;

    virtual MatrixXf calculate_gaussian_kernel() = 0;

    MatrixXf calculate_P_matrix(const Matrix3Xf &X, const Matrix3Xf &Y, const Matrix3Xf &Y_pred,
        const Eigen::VectorXf &Y_emit_prior, Matrix3Xf const& TY, double const sigma2);

    double initial_sigma2(MatrixXf const& X, MatrixXf const& Y);

protected:
    std::string const log_name_;
    double const tolerance_;
    int const max_iterations_;
    double const initial_sigma_scale_;
    double const w_;
    double const alpha_;
    double const beta_;
    double const zeta_;
    double const start_lambda_;

    // TODO(Dylan): Delete this and instead use the LLE weights set in each deformable object
    // configuration's LLE weights.
    // double_const
    MatrixXf m_lle_;

    // I don't love that I have to put this here to calculate the Gaussian kernel matrix. Maybe
    // refactor?
    Matrix3Xf original_template_;
};

class CPD : public CPDInterface
{
public:
    CPD(std::string const log_name_base, double const tolerance, int const max_iterations,
        double const initial_sigma_scale, double const w, double const alpha, double const beta,
        double const zeta, double const start_lambda, MatrixXf const m_lle);
    ~CPD(){}

    Matrix3Xf operator()(const Matrix3Xf &X, const Matrix3Xf &Y, const Matrix3Xf &Y_pred,
        const Eigen::VectorXf &Y_emit_prior, TrackingMap const& tracking_map) override;

    MatrixXf calculate_gaussian_kernel() override;
};

class CPDMultiTemplate : public CPDInterface
{
public:
    CPDMultiTemplate(std::string const log_name_base, double const tolerance, int const max_iterations,
        double const initial_sigma_scale, double const w, double const alpha, double const beta,
        double const zeta, double const start_lambda, MatrixXf const m_lle);
    ~CPDMultiTemplate(){}

    Matrix3Xf operator()(const Matrix3Xf &X, const Matrix3Xf &Y, const Matrix3Xf &Y_pred,
        const Eigen::VectorXf &Y_emit_prior, TrackingMap const& tracking_map) override;

    MatrixXf calculate_gaussian_kernel() override;

protected:
    MatrixXf calculate_mahalanobis_matrix();
};
