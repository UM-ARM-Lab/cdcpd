#include "cdcpd/cpd.h"

#include <iostream>

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
    , has_point_assignments_been_set_(false)
{
    // // std::cout << "past CPDInterface initializer list\n";
}

MatrixXf CPDInterface::get_point_assignments()
{
    if (has_point_assignments_been_set_)
    {
        return point_assignments_;
    }
    else
    {
        // This represents essentially no information on the point-to-template association.
        return MatrixXf::Ones(M_, N_);
    }
}

void CPDInterface::set_point_assignments(const Ref<const MatrixXf>& assignments)
{
    has_point_assignments_been_set_ = true;
    point_assignments_ = assignments;
}

MatrixXf CPDInterface::calculate_P_matrix(const Ref<const Matrix3Xf>& X,
    const Ref<const VectorXf>& Y_emit_prior, const Ref<const Matrix3Xf>&  TY,
    double const sigma2)
{
    // std::cout << "calculate_P_matrix\n\t1";
    MatrixXf P(M_, N_);
    for (int i = 0; i < M_; ++i)
    {
        for (int j = 0; j < N_; ++j)
        {
            P(i, j) = (X.col(j) - TY.col(i)).squaredNorm();
        }
    }
    // std::cout << "2";

    // auto association_prior = get_point_assignments();

    float c = std::pow(2 * M_PI * sigma2, static_cast<double>(D_) / 2);
    c *= w_ / (1 - w_);
    c *= static_cast<double>(M_) / N_;
    // std::cout << 3;

    // P = P.cwiseProduct(association_prior);

    P = (-P / (2 * sigma2)).array().exp().matrix();
    P.array().colwise() *= Y_emit_prior.array();
    // std::cout << 4;

    RowVectorXf den = P.colwise().sum();
    den.array() += c;

    // std::cout << 5;

    P = P.array().rowwise() / den.array();

    // std::cout << "6\n";

    return P;
}

double CPDInterface::initial_sigma2(const Ref<const MatrixXf>&  X, const Ref<const MatrixXf>&  Y)
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

Matrix3Xf CPD::operator()(const Ref<const Matrix3Xf>& X,
    const Ref<const Matrix3Xf>& Y, const Ref<const Matrix3Xf>& Y_pred,
    const Ref<const VectorXf>& Y_emit_prior, TrackingMap const& tracking_map)
{
    // X: (3, M) matrix of downsampled, masked point cloud
    // Y: (3, M) matrix Y^t (Y in IV.A) in the paper
    // Y_pred: (3, M) matrix of predicted tracked point locations.
    // Y_emit_prior: vector with a probability per tracked point that that Gaussian would generate
    //     samples at this time step. Generated from the visibility prior.

    M_ = Y.cols();
    N_ = X.cols();
    D_ = Y.rows();

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

        MatrixXf const P = calculate_P_matrix(X, Y_emit_prior, TY, sigma2);

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

CPDMultiTemplate::CPDMultiTemplate(std::string const log_name_base, double const tolerance, int const max_iterations,
    double const initial_sigma_scale, double const w, double const alpha, double const beta,
    double const zeta, double const start_lambda, MatrixXf const m_lle)
    : CPDInterface(log_name_base, tolerance, max_iterations, initial_sigma_scale, w, alpha, beta,
        zeta, start_lambda, m_lle)
{}

std::shared_ptr<MatrixXf> CPDMultiTemplate::calculate_gaussian_kernel(
    TrackingMap const& tracking_map)
{
    std::vector<TemplateVertexAssignment> const vertex_assignments =
        tracking_map.get_vertex_assignments();

    auto kernel_ptr = std::make_shared<MatrixXf>(M_, M_);
    MatrixXf& kernel = *kernel_ptr;
    kernel.setZero();

    // We only assign the kernel non-zero values where the index of the Gaussians belong to the same
    // template.
    for (TemplateVertexAssignment const& assignment : vertex_assignments)
    {
        // Build the matrix containing the pointwise squared norm for this template.
        std::shared_ptr<DeformableObjectConfiguration> def_obj_config =
            tracking_map.tracking_map.at(assignment.template_id);
        Matrix3Xf const& Y_initial = def_obj_config->initial_.getVertices();
        int const num_gaussians = assignment.idx_end - assignment.idx_start;
        MatrixXf diff = MatrixXf::Zero(num_gaussians, num_gaussians);

        for (size_t i = 0; i < num_gaussians; ++i)
        {
            for (size_t j = 0; j < num_gaussians; ++j)
            {
                float this_diff = (Y_initial.col(i) - Y_initial.col(j)).squaredNorm();
                // std::cout << "This diff: " << this_diff << std::endl;
                diff(i, j) = this_diff;
            }
        }

        // Set the portion of the Gaussian kernel matrix to the value calculated from the squared
        // norms.
        kernel.block(assignment.idx_start, assignment.idx_start, num_gaussians, num_gaussians)
            = (-diff / (2.0F * beta_ * beta_)).array().exp();
    }

    // std::cout << "Full Gaussian RBF Kernel:\n" << kernel << std::endl;

    return kernel_ptr;
}

CPDMultiTemplateMahalanobis::CPDMultiTemplateMahalanobis(std::string const log_name_base, double const tolerance,
    int const max_iterations, double const initial_sigma_scale, double const w, double const alpha,
    double const beta, double const zeta, double const start_lambda, MatrixXf const m_lle)
    : CPDMultiTemplate(log_name_base, tolerance, max_iterations, initial_sigma_scale, w, alpha, beta,
        zeta, start_lambda, m_lle)
{}

Matrix3Xf CPDMultiTemplateMahalanobis::operator()(const Ref<const Matrix3Xf>& X,
    const Ref<const Matrix3Xf>& Y, const Ref<const Matrix3Xf>& Y_pred,
    const Ref<const VectorXf>& Y_emit_prior, TrackingMap const& tracking_map)
{
    // X: (3, M) matrix of downsampled, masked point cloud
    // Y: (3, M) matrix Y^t (Y in IV.A) in the paper
    // Y_pred: (3, M) matrix of predicted tracked point locations.
    // Y_emit_prior: vector with a probability per tracked point that that Gaussian would generate
    //     samples at this time step. Generated from the visibility prior.
    N_ = X.cols();
    M_ = Y.cols();
    D_ = Y.rows();

    // G: (M, M) Guassian kernel matrix
    // MatrixXf G = gaussian_kernel(original_template, beta);  // Y, beta);
    std::shared_ptr<MatrixXf> G_ptr = calculate_gaussian_kernel(tracking_map);
    MatrixXf const& G = *G_ptr;

    // TY: Y^(t) in Algorithm 1
    Matrix3Xf TY = Y;
    double sigma2 = initial_sigma2(X, TY) * initial_sigma_scale_;

    int iterations = 0;
    double error = tolerance_ + 1;  // loop runs the first time

    while (iterations <= max_iterations_ && error > tolerance_) {
        double qprev = sigma2;
        // Expectation step

        // P: P in Line 5 in Algorithm 1 (mentioned after Eq. (18))
        // Calculate Eq. (9) (Line 5 in Algorithm 1)
        // NOTE: Eq. (9) misses M in the denominator

        MatrixXf P_naive = calculate_P_matrix(X, Y_emit_prior, TY, sigma2);
        MatrixXf association_prior = calculate_association_prior(X, TY, tracking_map, sigma2);

        // Multiply P by the association prior for each point.
        MatrixXf P = P_naive * association_prior;

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

        MatrixXf A = (P1.asDiagonal() * G) + alpha_ * sigma2 * MatrixXf::Identity(M_, M_)
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
        sigma2 = (xPx - 2 * trPXY + yPy) / (Np * static_cast<double>(D_));

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

MatrixXf CPDMultiTemplateMahalanobis::calculate_association_prior(const Ref<const Matrix3Xf>& X,
    const Ref<const Matrix3Xf>& TY, TrackingMap const& tracking_map, double const sigma2)
{
    std::shared_ptr<MatrixXf> d_M_ptr = calculate_mahalanobis_matrix(X, TY, sigma2);

    // Find the closest Gaussian to a point (by Mahalanobis distance), then find the neighbor of the
    // closest Gaussian that has the minimum distance to the point.
    auto const closest_gaussians = find_pointwise_closest_gaussians(*d_M_ptr, tracking_map);
    auto const second_closest_guassians =
        find_pointwise_second_closest_gaussians(*d_M_ptr, tracking_map, *closest_gaussians);

    // From the two closest Gaussians, calculate the synthetic Gaussian for each point for each
    // template.
    SyntheticGaussianCentroidsVector const synthetic_gaussians =
        find_synthetic_gaussian_centroids(X, TY, *closest_gaussians, *second_closest_guassians);

    // Calculate the Mahalanobis distance between each point and the associated synthetic Gaussian
    // for each template.
    // Return type should be a matrix with shape (num_templates, points)
    std::shared_ptr<MatrixXf> point_to_template_dists =
        calculate_point_to_template_distance(X, synthetic_gaussians, sigma2);

    // Compute the pointwise sum of synthetic Gaussian distances across all templates.
    // Simple eigen colwise sum.
    Eigen::RowVectorXf const pointwise_dist_sum = point_to_template_dists->colwise().sum();

    // Compute the distance score for each point/template combination.
    // Of shape (num_templates, num_points).
    MatrixXf dist_scores = (point_to_template_dists->array().rowwise() / (-1.0F * pointwise_dist_sum.array())).exp().matrix();
    // {
    //     // Ensure that the operation actually does assignment. I'm don't know if this is okay.
    //     dist_scores.array().rowwise() /= (-1.0F * pointwise_dist_sum.array());
    // }

    // dist_scores = dist_scores.array().exp().matrix();
    // MatrixXf const dist_scores = (point_to_template_dists->rowwise() / pointwise_dist_sum).array().exp().matrix();

    // Now assign the distance scores to the appropriate Gaussian indices.
    MatrixXf association_prior(M_, N_);
    std::vector<TemplateVertexAssignment> const vertex_assignments =
        tracking_map.get_vertex_assignments();
    for (size_t template_idx = 0; template_idx < vertex_assignments.size(); ++template_idx)
    {
        TemplateVertexAssignment const& assignment = vertex_assignments.at(template_idx);
        Eigen::RowVectorXf const& template_scores = dist_scores.row(template_idx);

        // Calculate the number of Gaussians in this template for using Eigen block expression
        // assignment
        int const num_gaussians = assignment.idx_end - assignment.idx_start;

        // for (int pt_idx = 0; pt_idx < N_; ++pt_idx)
        // {
        //     // Get the distance metric scores for all Gaussians associated with this template for
        //     // this point.
        //     auto score = dist_scores.col(pt_idx);
        //     association_prior.block(assignment.idx_start, pt_idx, num_gaussians, 1) = score;
        // }

        // Create a block (a smaller matrix view) of the association prior that represents all of
        // the Gaussians in this template and all points.
        auto template_block = association_prior.block(assignment.idx_start, 0, num_gaussians, N_);

        // Assign the same score to each row of the association prior since all Gaussians in the
        // template receive the same score for a given point.
        template_block.rowwise() = template_scores;
    }
    return association_prior;
}

float CPDMultiTemplateMahalanobis::mahalanobis_distance(
    const Ref<const VectorXf>& gaussian_centroid,
    const Ref<const MatrixXf>& covariance_inverse,
    const Ref<const VectorXf>& pt)
{
    auto const diff = pt - gaussian_centroid;
    // Using sum here as a means to convert 1x1 Eigen matrix to a double.
    float const d_M = (diff.transpose() * covariance_inverse * diff).sum();

    return d_M;
}

std::shared_ptr<MatrixXi> CPDMultiTemplateMahalanobis::find_pointwise_closest_gaussians(
    const Ref<const MatrixXf>&  d_M, TrackingMap const& tracking_map)
{
    // Find the closest Gaussian to each point (by Mahalanobis distance) for each template.
    std::vector<TemplateVertexAssignment> vertex_assignments =
        tracking_map.get_vertex_assignments();
    int const num_templates = vertex_assignments.size();

    // Pre-allocate the returned matrix so we don't incur extra overhead of dynamically expanding
    // the matrices.
    auto pointwise_closest_gaussians_ptr = std::shared_ptr<MatrixXi>(
        new MatrixXi(MatrixXi::Zero(num_templates, N_)) );
    Eigen::MatrixXi& pointwise_closest_gaussians = *pointwise_closest_gaussians_ptr;

    for (int pt_idx = 0; pt_idx < N_; ++pt_idx)
    {
        auto const& dists = d_M.col(pt_idx);

        // Though we don't use a reference for this Eigen column block, the values that we assign to
        // it within the loop are still saved in the original pointwise_closest_gaussians.
        auto pts_closest_gaussians = pointwise_closest_gaussians.col(pt_idx);

        // Iterate through each template's vertices to find the minimum distance Gaussian for this
        // template/point combination.
        int template_idx = 0;
        for (auto const& assignment : vertex_assignments)
        {
            float val_min = 1e15;
            int idx_min = -1;
            for (int m_idx = assignment.idx_start; m_idx < assignment.idx_end; ++m_idx)
            {
                float const distance = dists(m_idx);
                if (distance < val_min)
                {
                    // Update the minimum distance Gaussian
                    val_min = distance;
                    idx_min = m_idx;
                }
            }
            pts_closest_gaussians(template_idx) = idx_min;
            ++template_idx;
        }
    }

    return pointwise_closest_gaussians_ptr;
}

std::shared_ptr<MatrixXi> CPDMultiTemplateMahalanobis::find_pointwise_second_closest_gaussians(
    const Ref<const MatrixXf>& d_M, TrackingMap const& tracking_map,
    const Ref<const MatrixXi>& closest_gaussians)
{
    // Find the closest Gaussian to the point that is a neighbor to the closest Gaussian in the
    // connectivity graph.
    std::vector<int> const template_order = tracking_map.get_def_obj_id_iteration_order();
    std::vector<std::shared_ptr<ConnectivityGraph> > graphs = tracking_map.get_tracked_graphs();
    int const num_templates = graphs.size();

    // Pre-allocate the returned matrix so we don't incur extra overhead of dynamically expanding
    // the matrices.
    auto closest_neighboring_gaussians_ptr = std::shared_ptr<MatrixXi>(
        new MatrixXi(MatrixXi::Zero(num_templates, N_)) );
    Eigen::MatrixXi& closest_neighboring_gaussians = *closest_neighboring_gaussians_ptr;

    // Find the neighbor of the closest Gaussian to each point that's closest to the point for each
    // template.
    for (int template_idx = 0; template_idx < num_templates; ++template_idx)
    {
        // Get the connectivity graph for this template.
        ConnectivityGraph const& connectivity_graph = *graphs.at(template_idx);
        NodeMap const& node_map = connectivity_graph.get_node_map();

        // Define some intermediate variables for faster access to data.
        auto const pts_closest_gaussians = closest_gaussians.row(template_idx);
        auto pts_closest_gaussian_neighbor = closest_neighboring_gaussians.row(template_idx);

        for (int pt_idx = 0; pt_idx < N_; ++pt_idx)
        {
            auto const pts_d_M = d_M.col(pt_idx);
            int const closest_gaussian_idx = pts_closest_gaussians(pt_idx);

            auto const closest_gaussian_node = node_map.at(closest_gaussian_idx);
            NodeMap const& neighbors = closest_gaussian_node->get_neighbors();

            // Search the node's neighbors for the next closest Gaussian.
            float dist_min = 1e15;
            int idx_min = -1;
            for (auto const& neighbor : neighbors)
            {
                int const neighbor_idx = neighbor.first;
                float const neighbor_dist = pts_d_M(neighbor_idx);
                if (neighbor_dist < dist_min)
                {
                    dist_min = neighbor_dist;
                    idx_min = neighbor_idx;
                }
            }

            pts_closest_gaussian_neighbor(pt_idx) = idx_min;
        }
    }
    return closest_neighboring_gaussians_ptr;
}

Eigen::VectorXf CPDMultiTemplateMahalanobis::project_point_onto_line(
    const Ref<const VectorXf>& line_start, const Ref<const VectorXf>& line_end,
    const Ref<const VectorXf>& pt)
{
    auto const edge_line = line_end - line_start;
    auto const pt_line = pt - line_start;
    Eigen::VectorXf projection = (edge_line.dot(pt_line) / edge_line.dot(edge_line)) * edge_line;

    if (projection.norm() > edge_line.norm())
    {
        // This means the point extends past the edge line, so we just set the synthetic Gaussian
        // centroid to be the closest centroid.
        projection = line_end;
    }
    else
    {
        // Otherwise, we add back the origin of this projection to get the coordinates.
        projection += line_start;
    }
    return projection;
}

SyntheticGaussianCentroidsVector CPDMultiTemplateMahalanobis::find_synthetic_gaussian_centroids(
    const Ref<const Matrix3Xf>& X, const Ref<const Matrix3Xf>& TY,
    const Ref<const MatrixXi>& closest_gaussians,
    const Ref<const MatrixXi>& second_closest_gaussians)
{
    SyntheticGaussianCentroidsVector synthetic_gaussian_centroids_all_templates;

    // Compute the projection of each point onto the line formed between the two closest Gaussians
    // for each template.
    for (int template_idx = 0; template_idx < closest_gaussians.rows(); ++template_idx)
    {
        auto projections_ptr = std::make_shared<MatrixXf>(MatrixXf::Zero(D_, N_));
        MatrixXf& projections = *projections_ptr;
        for (int pt_idx = 0; pt_idx < N_; ++pt_idx)
        {
            int const idx_closest = closest_gaussians(template_idx, pt_idx);
            int const idx_second_closest = second_closest_gaussians(template_idx, pt_idx);

            auto const& pt = X.col(pt_idx);
            auto const& cent_closest = TY.col(idx_closest);
            auto const& cent_second_closest = TY.col(idx_second_closest);

            // The arguments may appear to be backwards at first, but using the second closest
            // centroid as the origin of the projection lends itself to easy clipping if the
            // projection extends past the line formed between the two Gaussians.
            projections.col(pt_idx) =
                project_point_onto_line(cent_second_closest, cent_closest, pt);
        }
        synthetic_gaussian_centroids_all_templates.push_back(projections_ptr);
    }

    // TODO: If we decide to use different variances for each Gaussian, then the following steps
    // will become necessary.
    // 1. From the projection, we can calculate the distance of the synthetic Gaussian centroid to
    //    the closest and second closest tracked Gaussians.
    // 2. Calculate the variance of the synthetic Gaussian for this template by computing average of
    //    variances weighted by Mahalanobis distance to the two Gaussians.
    // 3. Form the synthetic Gaussian based on the projection of the point onto the line formed by
    //    the two closest Gaussians.
    return synthetic_gaussian_centroids_all_templates;
}

std::shared_ptr<MatrixXf> CPDMultiTemplateMahalanobis::calculate_mahalanobis_matrix(
    const Ref<const Matrix3Xf>&  X, const Ref<const Matrix3Xf>&  TY,
    double const sigma2)
{
    // Creating a matrix on the heap so we don't have to worry about expensive matrix copy
    // constructors when returning from functions.
    auto d_M_ptr = std::shared_ptr<MatrixXf>( new MatrixXf(MatrixXf::Zero(M_, N_)) );
    MatrixXf& d_M = *d_M_ptr;

    // Get the Mahalanobis distance of each point to each Gaussian
    Eigen::MatrixXf covariance_inverse = get_covariance_inverse(sigma2);
    for (int m = 0; m < M_; ++m)
    {
        auto const& y_m = TY.col(m);
        for (int i = 0; i < N_; ++i)
        {
            auto const& x_i = X.col(i);
            d_M(m, i) = mahalanobis_distance(y_m, covariance_inverse, x_i);
        }
    }

    return d_M_ptr;
}

std::shared_ptr<MatrixXf> CPDMultiTemplateMahalanobis::calculate_point_to_template_distance(
    const Ref<const Matrix3Xf>&  X,
    SyntheticGaussianCentroidsVector const& synthetic_centroids, double const sigma2)
{
    int const num_templates = synthetic_centroids.size();
    auto pt_to_template_dists_ptr = std::make_shared<MatrixXf>(MatrixXf::Zero(num_templates, N_));
    MatrixXf& pt_to_template_dists = *pt_to_template_dists_ptr;
    MatrixXf cov_inv = get_covariance_inverse(sigma2);

    for (int template_idx = 0; template_idx < num_templates; ++template_idx)
    {
        MatrixXf const& synthetic_centroids_this_template = *synthetic_centroids.at(template_idx);
        for (int pt_idx = 0; pt_idx < N_; ++pt_idx)
        {
            auto const& centroid = synthetic_centroids_this_template.col(pt_idx);
            auto const& pt = X.col(pt_idx);
            pt_to_template_dists(template_idx, pt_idx) =
                mahalanobis_distance(centroid, cov_inv, pt);
        }
    }
    return pt_to_template_dists_ptr;
}

Eigen::MatrixXf CPDMultiTemplateMahalanobis::get_covariance_inverse(float const sigma2)
{
    float const sigma2_inverse = 1.0 / sigma2;
    Eigen::MatrixXf covariance_inverse = Eigen::MatrixXf(3, 3);
    covariance_inverse << sigma2_inverse,            0.0,            0.0,
                                    0.0, sigma2_inverse,            0.0,
                                    0.0,            0.0, sigma2_inverse;
    return covariance_inverse;
}

CPDMultiTemplateExternalPointAssignment::CPDMultiTemplateExternalPointAssignment(
    std::string const log_name_base, double const tolerance,
    int const max_iterations, double const initial_sigma_scale, double const w,
    double const alpha, double const beta, double const zeta, double const start_lambda,
    MatrixXf const m_lle)
    : CPDMultiTemplate(log_name_base, tolerance, max_iterations, initial_sigma_scale, w, alpha,
        beta, zeta, start_lambda, m_lle)
{
    // std::cout << "Past CPDMultiTemplateExternalPointAssignment initializer list" << std::endl;
}

Matrix3Xf CPDMultiTemplateExternalPointAssignment::operator()(const Ref<const Matrix3Xf>& X,
    const Ref<const Matrix3Xf>& Y, const Ref<const Matrix3Xf>& Y_pred,
    const Ref<const VectorXf>& Y_emit_prior, TrackingMap const& tracking_map)
{
    // X: (3, M) matrix of downsampled, masked point cloud
    // Y: (3, M) matrix Y^t (Y in IV.A) in the paper
    // Y_pred: (3, M) matrix of predicted tracked point locations.
    // Y_emit_prior: vector with a probability per tracked point that that Gaussian would generate
    //     samples at this time step. Generated from the visibility prior.
    // std::cout << "CPD operator()\n";
    N_ = X.cols();
    M_ = Y.cols();
    D_ = Y.rows();

    // std::cout << "X (rows, cols): (" << X.rows() << ", " << X.cols() << ")\n";
    // std::cout << "Y (rows, cols): (" << Y.rows() << ", " << Y.cols() << ")\n";

    // G: (M, M) Guassian kernel matrix
    // MatrixXf G = gaussian_kernel(original_template, beta);  // Y, beta);
    // std::cout << "G kernel calc\n";
    std::shared_ptr<MatrixXf> G_ptr = calculate_gaussian_kernel(tracking_map);
    MatrixXf const& G = *G_ptr;

    // TY: Y^(t) in Algorithm 1
    Matrix3Xf TY = Y;
    double sigma2 = initial_sigma2(X, TY) * initial_sigma_scale_;

    int iterations = 0;
    double error = tolerance_ + 1;  // loop runs the first time

    // This class allows external point assignment so we take the point assignment out of the loop.
    // std::cout << "pre assoc prior\n";
    MatrixXf const association_prior = get_point_assignments();
    // std::cout << "association_prior (rows, cols): (" << association_prior.rows() << ", " << association_prior.cols() << ")\n";
    // std::cout << "post\n";
    // std::cout << "X:\n" << X << std::endl;
    // std::cout << "Y_emit_prior:\n" << Y_emit_prior << std::endl;

    while (iterations <= max_iterations_ && error > tolerance_) {\
        std::cout << "sigma2: " << sigma2 << std::endl;
        double qprev = sigma2;
        // Expectation step

        // P: P in Line 5 in Algorithm 1 (mentioned after Eq. (18))
        // Calculate Eq. (9) (Line 5 in Algorithm 1)
        // NOTE: Eq. (9) misses M in the denominator

        MatrixXf P_naive = calculate_P_matrix(X, Y_emit_prior, TY, sigma2);
        // MatrixXf P = calculate_P_matrix(X, Y_emit_prior, TY, sigma2);
        // std::cout << "P_naive:\n" << P_naive << std::endl;
        // std::cout << "P_naive (rows, cols): (" << P_naive.rows() << ", " << P_naive.cols() << ")\n";

        // Multiply P by the association prior for each point.
        // std::cout << "here 1\n";
        MatrixXf P = P_naive.cwiseProduct(association_prior);
        // std::cout << "P with association prior:\n" << P << std::endl;
        // std::cout << "P (rows, cols): (" << P.rows() << ", " << P.cols() << ")\n";
        // std::cout << "2\n";

        // Fast Gaussian Transformation to calculate Pt1, P1, PX
        MatrixXf PX = (P * X.transpose()).transpose();
        // std::cout << "3\n";

        // Maximization step
        VectorXf Pt1 = P.colwise().sum();
        VectorXf P1 = P.rowwise().sum();
        float Np = P1.sum();

        // NOTE: lambda means gamma here
        // Corresponding to Eq. (18) in the paper
        // std::cout << "4\n";
        auto const lambda = start_lambda_;
        MatrixXf p1d = P1.asDiagonal();

        MatrixXf A = (P1.asDiagonal() * G) + alpha_ * sigma2 * MatrixXf::Identity(M_, M_)
        + sigma2 * lambda * (m_lle_ * G) + zeta_ * G;

        MatrixXf B = PX.transpose() - (p1d + sigma2 * lambda * m_lle_) * Y.transpose()
        + zeta_ * (Y_pred.transpose() - Y.transpose());

        MatrixXf W = (A).householderQr().solve(B);

        TY = Y + (G * W).transpose();

        // std::cout << "5\n";

        // Corresponding to Eq. (19) in the paper
        VectorXf xPxtemp = (X.array() * X.array()).colwise().sum();
        // std::cout << "5.1\n";
        // std::cout << "Pt1 (rows, cols): (" << Pt1.rows() << ", " << Pt1.cols() << ")\n";
        // std::cout << "xPxtemp (rows, cols): (" << xPxtemp.rows() << ", " << xPxtemp.cols() << ")\n";
        double xPx = Pt1.dot(xPxtemp);
        // std::cout << "5.2\n";
        VectorXf yPytemp = (TY.array() * TY.array()).colwise().sum();
        // std::cout << "5.3\n";
        double yPy = P1.dot(yPytemp);
        // std::cout << "5.4\n";
        double trPXY = (TY.array() * PX.array()).sum();
        // std::cout << "5.5\n";
        sigma2 = (xPx - 2 * trPXY + yPy) / (Np * static_cast<double>(D_));

        // std::cout << "6\n";

        if (sigma2 <= 0) {
        sigma2 = tolerance_ / 10;
        }

        error = std::abs(sigma2 - qprev);
        iterations++;
        // std::cout << std::endl;
    }

    ROS_DEBUG_STREAM_NAMED(log_name_, "cpd error: " << error << " itr: " << iterations);
    ROS_DEBUG_STREAM_NAMED(log_name_, "cpd std dev: " << std::pow(sigma2, 0.5));

    return TY;
}