#pragma once

#include <Eigen/Dense>
#include <ros/console.h>

#include "cdcpd/tracking_map.h"

using Eigen::Matrix3Xf;
using Eigen::MatrixXf;
using Eigen::RowVectorXf;
using Eigen::VectorXf;
using Eigen::MatrixXi;
using Eigen::MatrixXd;

typedef std::vector<std::shared_ptr<MatrixXf> > SyntheticGaussianCentroidsVector;

class CPDInterface
{
public:
    CPDInterface(std::string const log_name_base, double const tolerance, int const max_iterations,
        double const initial_sigma_scale, double const w, double const alpha, double const beta,
        double const zeta, double const start_lambda, MatrixXf const m_lle);
    virtual ~CPDInterface(){}

    // TODO(Dylan): Clean up passing in the tracking_map. This is passing in way too much information.
    virtual Matrix3Xf operator()(const Eigen::Ref<const Matrix3Xf>& X,
        const Eigen::Ref<const Matrix3Xf>& Y, const Eigen::Ref<const Matrix3Xf>& Y_pred,
        const Eigen::Ref<const VectorXf>& Y_emit_prior, TrackingMap const& tracking_map) = 0;

    MatrixXf calculate_P_matrix(const Eigen::Ref<const Matrix3Xf>& X,
        const Eigen::Ref<const VectorXf>& Y_emit_prior, const Eigen::Ref<const Matrix3Xf>&  TY,
        double const sigma2);

    double initial_sigma2(const Eigen::Ref<const MatrixXf>&  X,
        const Eigen::Ref<const MatrixXf>&  Y);

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

    // Number of Gaussians considered in the CPD iteration.
    int M_;

    // Number of masked points in the CPD iteration.
    int N_;

    // Dimension of the CPD points.
    int D_;
};

class CPD : public CPDInterface
{
public:
    CPD(std::string const log_name_base, double const tolerance, int const max_iterations,
        double const initial_sigma_scale, double const w, double const alpha, double const beta,
        double const zeta, double const start_lambda, MatrixXf const m_lle);
    ~CPD(){}

    Matrix3Xf operator()(const Eigen::Ref<const Matrix3Xf>& X,
        const Eigen::Ref<const Matrix3Xf>& Y, const Eigen::Ref<const Matrix3Xf>& Y_pred,
        const Eigen::Ref<const VectorXf>& Y_emit_prior, TrackingMap const& tracking_map) override;

protected:
    MatrixXf calculate_gaussian_kernel();
};

// Performs an augmented version of CPD, designed for multi-template tracking.
// NOTE: There's a lot of functions that return pointers to matrices that can get confusing.
//       the reason for this is so that we create the matrices on the heap instead of the stack
//       which avoids expensive matrix copies every time we call a function that creates a matrix.
class CPDMultiTemplate : public CPDInterface
{
public:
    CPDMultiTemplate(std::string const log_name_base, double const tolerance, int const max_iterations,
        double const initial_sigma_scale, double const w, double const alpha, double const beta,
        double const zeta, double const start_lambda, MatrixXf const m_lle);
    ~CPDMultiTemplate(){}

    Matrix3Xf operator()(const Eigen::Ref<const Matrix3Xf>& X,
        const Eigen::Ref<const Matrix3Xf>& Y, const Eigen::Ref<const Matrix3Xf>& Y_pred,
        const Eigen::Ref<const VectorXf>& Y_emit_prior, TrackingMap const& tracking_map) override;

protected:
    // Calculates the Gaussian kernel (G in paper).
    // NOTE: This is slightly modified for multi-template tracking such that the kernel values for
    // Gaussians between templates is set to zero.
    std::shared_ptr<MatrixXf> calculate_gaussian_kernel(TrackingMap const& tracking_map);

    // Computes the pointwise association prior
    MatrixXf calculate_association_prior(const Eigen::Ref<const Matrix3Xf>& X,
        const Eigen::Ref<const Matrix3Xf>& TY, TrackingMap const& tracking_map,
        double const sigma2);

    // Computes the Mahalanobis distance of a point from the given Gaussian.
    // NOTE: Using Eigen::Ref means that this function will accept either Eigen::Block or
    // Eigen::Matrix types.
    float mahalanobis_distance(const Eigen::Ref<const VectorXf>& gaussian_centroid,
        const Eigen::Ref<const MatrixXf>& covariance_inverse,
        const Eigen::Ref<const VectorXf>& pt);

    // Finds the closest Gaussians (returns the index) to the given points for each template.
    std::shared_ptr<MatrixXi> find_pointwise_closest_gaussians(
        const Eigen::Ref<const MatrixXf>& d_M, TrackingMap const& tracking_map);

    // Finds nearest neighboring Gaussian to points given the closest Gaussians for each template.
    std::shared_ptr<MatrixXi> find_pointwise_second_closest_gaussians(
        const Eigen::Ref<const MatrixXf>& d_M, TrackingMap const& tracking_map,
        const Eigen::Ref<const MatrixXi>& closest_gaussians);

    // Computes the projection of a vector from start->pt onto start->end with clipping if the
    // projection extends past the vector from start->end
    Eigen::VectorXf project_point_onto_line(const Eigen::Ref<const VectorXf>& line_start,
        const Eigen::Ref<const VectorXf>& line_end, const Eigen::Ref<const VectorXf>& pt);

    // Returns vector of length num_templates of pointers to matrices of shape (D_, num_points)
    SyntheticGaussianCentroidsVector find_synthetic_gaussian_centroids(
        const Eigen::Ref<const Matrix3Xf>& X, const Eigen::Ref<const Matrix3Xf>& TY,
        const Eigen::Ref<const MatrixXi>& closest_gaussians,
        const Eigen::Ref<const MatrixXi>& second_closest_gaussians);

    std::shared_ptr<MatrixXf> calculate_mahalanobis_matrix(const Eigen::Ref<const Matrix3Xf>& X,
        const Eigen::Ref<const Matrix3Xf>& TY, double const sigma2);

    // Calculates the distance of all masked segmentation points (X) to each template.
    std::shared_ptr<MatrixXf> calculate_point_to_template_distance(
        const Eigen::Ref<const Matrix3Xf>& X,
        SyntheticGaussianCentroidsVector const& synthetic_centroids, double const sigma2);

    // NOTE: This is a very small matrix (3, 3), so we don't worry about the copy constructor
    // overhead.
    Eigen::MatrixXf get_covariance_inverse(float const sigma2);

};
