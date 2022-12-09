#include "cdcpd/cpd.h"

CPDInterface::CPDInterface(std::string const log_name_base, double const tolerance,
    int const max_iterations, double const initial_sigma_scale, double const w, double const alpha,
    double const beta, double const zeta, double const start_lambda, MatrixXf const m_lle)
    : log_name_(log_name_base + ".cpd")
    , tolerance_(tolerance)
    , max_iterations_(max_iterations)
    , initial_sigma_scale_(initial_sigma_scale)
    , w_(w)
    , alpha_(alpha)
    , beta_(beta)
    , zeta_(zeta)
    , start_lambda_(start_lambda)
    , m_lle_(m_lle)
{}

MatrixXf CPDInterface::calculate_P_matrix(const Matrix3Xf &X, const Matrix3Xf &Y,
    const Matrix3Xf &Y_pred, const Eigen::VectorXf &Y_emit_prior, Matrix3Xf const& TY,
    double const sigma2)
{
    int const N = X.cols();
    int const M = Y.cols();
    int const D = Y.rows();

    MatrixXf P(M, N);
    for (int i = 0; i < M; ++i)
    {
        for (int j = 0; j < N; ++j)
        {
            P(i, j) = (X.col(j) - TY.col(i)).squaredNorm();
        }
    }

    float c = std::pow(2 * M_PI * sigma2, static_cast<double>(D) / 2);
    c *= w_ / (1 - w_);
    c *= static_cast<double>(M) / N;

    P = (-P / (2 * sigma2)).array().exp().matrix();
    P.array().colwise() *= Y_emit_prior.array();

    RowVectorXf den = P.colwise().sum();
    den.array() += c;

    P = P.array().rowwise() / den.array();

    return P;
}

double CPDInterface::initial_sigma2(MatrixXf const& X, MatrixXf const& Y)
{
    // X: (3, N) matrix, X^t in Algorithm 1
    // Y: (3, M) matrix, Y^(t-1) in Algorithm 1
    // Implement Line 2 of Algorithm 1
    double total_error = 0.0;
    assert(X.rows() == Y.rows());
    for (int i = 0; i < X.cols(); ++i)
    {
        for (int j = 0; j < Y.cols(); ++j)
        {
            total_error += (X.col(i) - Y.col(j)).squaredNorm();
        }
    }
    return total_error / (X.cols() * Y.cols() * X.rows());
}

CPD::CPD(std::string const log_name_base, double const tolerance, int const max_iterations,
    double const initial_sigma_scale, double const w, double const alpha, double const beta,
    double const zeta, double const start_lambda, MatrixXf const m_lle)
    : CPDInterface(log_name_base, tolerance, max_iterations, initial_sigma_scale, w, alpha, beta,
        zeta, start_lambda, m_lle)
{}

Matrix3Xf CPD::operator()(const Matrix3Xf &X, const Matrix3Xf &Y, const Matrix3Xf &Y_pred,
                     const Eigen::VectorXf &Y_emit_prior, TrackingMap const& tracking_map)
{
    // X: (3, M) matrix of downsampled, masked point cloud
    // Y: (3, M) matrix Y^t (Y in IV.A) in the paper
    // Y_pred: (3, M) matrix of predicted tracked point locations.
    // Y_emit_prior: vector with a probability per tracked point that that Gaussian would generate
    //     samples at this time step. Generated from the visibility prior.

    // G: (M, M) Guassian kernel matrix
    {
        // Set the original template to calculate the Gaussian kernel matrix.
        bool const use_initial_tracking = true;
        original_template_ =
            tracking_map.form_vertices_cloud(use_initial_tracking)->getMatrixXfMap().topRows(3);
    }
    MatrixXf G = calculate_gaussian_kernel();

    // TY: Y^(t) in Algorithm 1
    Matrix3Xf TY = Y;
    double sigma2 = initial_sigma2(X, TY) * initial_sigma_scale_;

    int iterations = 0;
    double error = tolerance_ + 1;  // loop runs the first time

    while (iterations <= max_iterations_ && error > tolerance_)
    {
        double qprev = sigma2;
        // Expectation step
        int const M = Y.cols();
        int const D = Y.rows();

        // P: P in Line 5 in Algorithm 1 (mentioned after Eq. (18))
        // Calculate Eq. (9) (Line 5 in Algorithm 1)
        // NOTE: Eq. (9) misses M in the denominator

        MatrixXf const P = calculate_P_matrix(X, Y, Y_pred, Y_emit_prior, TY, sigma2);

        // Fast Gaussian Transformation to calculate Pt1, P1, PX
        MatrixXf PX = (P * X.transpose()).transpose();

        // // Maximization step
        VectorXf Pt1 = P.colwise().sum();
        VectorXf P1 = P.rowwise().sum();
        float Np = P1.sum();

        // NOTE: lambda means gamma here
        // Corresponding to Eq. (18) in the paper
        auto const lambda = start_lambda_;
        MatrixXf p1d = P1.asDiagonal();

        MatrixXf A = (P1.asDiagonal() * G)
                     + alpha_ * sigma2 * MatrixXf::Identity(M, M)
                     + sigma2 * lambda * (m_lle_ * G)
                     + zeta_ * G;

        MatrixXf B = PX.transpose()
                     - (p1d + sigma2 * lambda * m_lle_) * Y.transpose()
                     + zeta_ * (Y_pred.transpose() - Y.transpose());

        MatrixXf W = (A).householderQr().solve(B);

        TY = Y + (G * W).transpose();

        // Corresponding to Eq. (19) in the paper
        VectorXf xPxtemp = (X.array() * X.array()).colwise().sum();
        double xPx = Pt1.dot(xPxtemp);
        VectorXf yPytemp = (TY.array() * TY.array()).colwise().sum();
        double yPy = P1.dot(yPytemp);
        double trPXY = (TY.array() * PX.array()).sum();
        sigma2 = (xPx - 2 * trPXY + yPy) / (Np * static_cast<double>(D));

        if (sigma2 <= 0)
        {
            sigma2 = tolerance_ / 10;
        }

        error = std::abs(sigma2 - qprev);
        iterations++;
    }

    ROS_DEBUG_STREAM_NAMED(log_name_, "cpd error: " << error << " itr: " << iterations);

    return TY;
}

MatrixXf CPD::calculate_gaussian_kernel()
{
    // Y: (3, M) matrix, corresponding to Y^(t-1) in Eq. 13.5 (Y^t in VI.A)
    // beta: beta in Eq. 13.5 (between 13 and 14)
    Matrix3Xf const& Y = original_template_;
    MatrixXf diff(Y.cols(), Y.cols());
    diff.setZero();
    for (int i = 0; i < Y.cols(); ++i)
    {
        for (int j = 0; j < Y.cols(); ++j)
        {
            diff(i, j) = (Y.col(i) - Y.col(j)).squaredNorm();
        }
    }
    // ???: beta should be beta^2
    MatrixXf kernel = (-diff / (2 * beta_ * beta_)).array().exp();
    return kernel;
}

CPDMultiTemplate::CPDMultiTemplate(std::string const log_name_base, double const tolerance,
    int const max_iterations, double const initial_sigma_scale, double const w, double const alpha,
    double const beta, double const zeta, double const start_lambda, MatrixXf const m_lle)
    : CPDInterface(log_name_base, tolerance, max_iterations, initial_sigma_scale, w, alpha, beta,
        zeta, start_lambda, m_lle)
{}

Matrix3Xf CPDMultiTemplate::operator()(const Matrix3Xf &X, const Matrix3Xf &Y,
    const Matrix3Xf &Y_pred, const Eigen::VectorXf &Y_emit_prior, TrackingMap const& tracking_map)
{
    // X: (3, M) matrix of downsampled, masked point cloud
    // Y: (3, M) matrix Y^t (Y in IV.A) in the paper
    // Y_pred: (3, M) matrix of predicted tracked point locations.
    // Y_emit_prior: vector with a probability per tracked point that that Gaussian would generate
    //     samples at this time step. Generated from the visibility prior.

    // G: (M, M) Guassian kernel matrix
    // MatrixXf G = gaussian_kernel(original_template, beta);  // Y, beta);
    MatrixXf G = calculate_gaussian_kernel();

    // TY: Y^(t) in Algorithm 1
    Matrix3Xf TY = Y;
    double sigma2 = initial_sigma2(X, TY) * initial_sigma_scale_;

    int iterations = 0;
    double error = tolerance_ + 1;  // loop runs the first time

    while (iterations <= max_iterations_ && error > tolerance_) {
        double qprev = sigma2;
        // Expectation step
        int N = X.cols();
        int M = Y.cols();
        int D = Y.rows();

        // P: P in Line 5 in Algorithm 1 (mentioned after Eq. (18))
        // Calculate Eq. (9) (Line 5 in Algorithm 1)
        // NOTE: Eq. (9) misses M in the denominator

        MatrixXf P = calculate_P_matrix();
        MatrixXf d_M_full(M, N);

        // Fast Gaussian Transformation to calculate Pt1, P1, PX
        MatrixXf PX = (P * X.transpose()).transpose();

        // // Maximization step
        VectorXf Pt1 = P.colwise().sum();
        VectorXf P1 = P.rowwise().sum();
        float Np = P1.sum();

        // NOTE: lambda means gamma here
        // Corresponding to Eq. (18) in the paper
        auto const lambda = start_lambda_;
        MatrixXf p1d = P1.asDiagonal();

        MatrixXf A = (P1.asDiagonal() * G) + alpha_ * sigma2 * MatrixXf::Identity(M, M)
        + sigma2 * lambda * (m_lle_ * G) + zeta_ * G;

        MatrixXf B = PX.transpose() - (p1d + sigma2 * lambda * m_lle_) * Y.transpose()
        + zeta_ * (Y_pred.transpose() - Y.transpose());

        MatrixXf W = (A).householderQr().solve(B);

        TY = Y + (G * W).transpose();

        // Corresponding to Eq. (19) in the paper
        VectorXf xPxtemp = (X.array() * X.array()).colwise().sum();
        double xPx = Pt1.dot(xPxtemp);
        VectorXf yPytemp = (TY.array() * TY.array()).colwise().sum();
        double yPy = P1.dot(yPytemp);
        double trPXY = (TY.array() * PX.array()).sum();
        sigma2 = (xPx - 2 * trPXY + yPy) / (Np * static_cast<double>(D));

        if (sigma2 <= 0) {
        sigma2 = tolerance_ / 10;
        }

        error = std::abs(sigma2 - qprev);
        iterations++;
    }

    ROS_DEBUG_STREAM_NAMED(log_name_, "cpd error: " << error << " itr: " << iterations);
    ROS_DEBUG_STREAM_NAMED(log_name_, "cpd std dev: " << std::pow(sigma2, 0.5));

    return TY;
}

MatrixXf CPDMultiTemplate::calculate_gaussian_kernel()
{

}

MatrixXf CPDMultiTemplate::calculate_mahalanobis_matrix()
{
// //     // TODO: Implement Mahalanobis distance metric here.

// //     // Get the Mahalanobis distance of each point to each Gaussian
// //     double const sigma2_inverse = 1.0 / sigma2;
// //     Eigen::MatrixXf covariance_inverse = Eigen::MatrixXf(3, 3);
// //     covariance_inverse << sigma2_inverse,            0.0,            0.0,
// //                                     0.0, sigma2_inverse,            0.0,
// //                                     0.0,            0.0, sigma2_inverse;
// //     for (int m = 0; m < M; ++m)
// //     {
// //         auto const& y_m = TY.col(m);
// //         for (int i = 0; i < N; ++i)
// //         {
// //         auto const& x_i = X.col(i);
// //         d_M_full(m, i) = mahalanobis_distance(y_m, covariance_inverse, x_i);
// //         }
// //     }

// //     // Find the closest Gaussian to each point (by Mahalanobis distance) for each template.
// //     // NOTE: Use the connectivity graph added to the deformable object tracking now.
// //     int const num_templates = 2;  // Prototyping with 2 templates while I figure out how to pass
// //     // in a dynamic amount of templates
// //     int const num_gaussians_in_template_0 = 15;
// //     Eigen::MatrixXi pointwise_closest_gaussians = Eigen::MatrixXi::Zero(num_templates, N);
// //     for (int pt_idx = 0; pt_idx < N; ++pt_idx)
// //     {
// //         float val_min = 1e15;
// //         int idx_min = -1;
// //         auto const& dists = d_M_full.col(pt_idx);

// //         // I wonder if I can do the Gaussian-template assignments really stupidly at first where
// //         // the first 15 Gaussians belong to template 0 and the last 15 belong to template 1?
// //         // Find the closest Gaussian for template 0.
// //         for (int m_idx = 0; m_idx < num_gaussians_in_template_0; ++m_idx)
// //         {
// //         float const distance = dists(m_idx);
// //         if (distance < val_min)
// //         {
// //             // Update the minimum distance Gaussian
// //             val_min = distance;
// //             idx_min = m_idx;
// //         }
// //         }
// //         pointwise_closest_gaussians(0, pt_idx) = idx_min;

// //         // Find the closest Gaussian for template 1.
// //         val_min = 1e15;
// //         idx_min = -1;
// //         for (int m_idx = num_gaussians_in_template_0; m_idx < M; ++m_idx)
// //         {
// //         float const distance = dists(m_idx);
// //         if (distance < val_min)
// //         {
// //             // Update the minimum distance Gaussian
// //             val_min = distance;
// //             idx_min = m_idx;
// //         }
// //         }
// //         pointwise_closest_gaussians(1, pt_idx) = idx_min;
// //     }

// //     // Find the closest Gaussian to the point that is a neighbor to the closest Gaussian in the
// //     // connectivity graph.
// //     // for (int pt_idx = 0; pt_idx < N; ++pt_idx)
// //     // {
// //     //   // int const gaussian_idx = pointwise_closest_gaussians(0, pt_idx);

// //     // }


// //     // Compute the projection of each point onto the line formed between the two closest Gaussians
// //     // for each template.


// //     // Form the synthetic Gaussian based on the projection of the point onto the line formed by
// //     // the two closest Gaussians.

// //     // Compute the sum of distances.

// //     // Find the association prior for each point and Gaussian pair based on the point's distance
// //     // to this template compared to the sum of distances.
// //     // e^(-this_distance / distance_sum)


// //     // Multiply P by the association prior for each point.
}