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
