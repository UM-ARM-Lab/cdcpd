#include "cdcpd/optimizer.h"

using Eigen::MatrixXf;
using Eigen::Matrix3Xf;
using Eigen::Matrix3Xd;
using Eigen::MatrixXi;

// Builds the quadratic term ||point_a - point_b||^2
// This is equivalent to [point_a' point_b'] * Q * [point_a' point_b']'
// where Q is [ I, -I
//             -I,  I]
static GRBQuadExpr buildDifferencingQuadraticTerm(GRBVar* point_a, GRBVar* point_b, const size_t num_vars_per_point)
{
    GRBQuadExpr expr;

    // Build the main diagonal
    const std::vector<double> main_diag(num_vars_per_point, 1.0);
    expr.addTerms(main_diag.data(), point_a, point_a, (int)num_vars_per_point);
    expr.addTerms(main_diag.data(), point_b, point_b, (int)num_vars_per_point);

    // Build the off diagonal - use -2 instead of -1 because the off diagonal terms are the same
    const std::vector<double> off_diagonal(num_vars_per_point, -2.0);
    expr.addTerms(off_diagonal.data(), point_a, point_b, (int)num_vars_per_point);

    return expr;
}

static GRBEnv& getGRBEnv()
{
    static GRBEnv env;
    return env;
}

// TODO do setup here
Optimizer::Optimizer(const Eigen::Matrix3Xf _init_temp, const float _stretch_lambda)
    : initial_template(_init_temp), stretch_lambda(_stretch_lambda)
{
}

// TODO change everything to Matrix3Xfs

MatrixXf Optimizer::operator()(const Matrix3Xf& Y, const MatrixXi& E)
{
    assert(E.rows() == 2);
    Matrix3Xf Y_opt(Y.rows(), Y.cols());
    GRBVar* vars = nullptr;
    try
    {
        const size_t num_vectors = (size_t) Y.cols();
        const size_t num_vars = 3 * num_vectors;

        GRBEnv& env = getGRBEnv();

        // Disables logging to file and logging to console (with a 0 as the value of the flag)
        env.set(GRB_IntParam_OutputFlag, 0);
        GRBModel model(env);
        model.set("ScaleFlag", "2");

        // Add the vars to the model
        {
            // Note that variable bound is important, without a bound, Gurobi defaults to 0, which is clearly unwanted
            const std::vector<double> lb(num_vars, -GRB_INFINITY);
            const std::vector<double> ub(num_vars, GRB_INFINITY);
            vars = model.addVars(lb.data(), ub.data(), nullptr, nullptr, nullptr, (int) num_vars);
            model.update();
        }

        // Add the edge constraints
        {
            for (size_t i = 0; i < E.cols(); ++i)
            {
                std::cout << "constraint: " << stretch_lambda * stretch_lambda * (initial_template.col(E(0, i)) - initial_template.col(E(1, i))).squaredNorm() << std::endl;
                model.addQConstr(
                            buildDifferencingQuadraticTerm(&vars[E(0, i) * 3], &vars[E(1, i) * 3], 3),
                            GRB_LESS_EQUAL,
                            stretch_lambda * stretch_lambda * (initial_template.col(E(0, i)) - initial_template.col(E(1, i))).squaredNorm(),
                            "edge_" + std::to_string(E(0, i)) + "_to_" + std::to_string(E(1, i)));
            }
            model.update();
        }

        // Build the objective function
        {
            Matrix3Xd Y_copy = Y.cast<double>(); // TODO is this exactly what we want?
            GRBQuadExpr objective_fn(0);
            for (size_t i = 0; i < num_vectors; ++i)
            {
                const auto expr0 = vars[i * 3 + 0] - Y_copy(0, i);
                const auto expr1 = vars[i * 3 + 1] - Y_copy(1, i);
                const auto expr2 = vars[i * 3 + 2] - Y_copy(2, i);
                objective_fn += expr0 * expr0;
                objective_fn += expr1 * expr1;
                objective_fn += expr2 * expr2;
            }
            model.setObjective(objective_fn, GRB_MINIMIZE);
            model.update();
        }

        // Find the optimal solution, and extract it
        {
            model.optimize();
            if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
            {
                std::cout << "Y" << std::endl;
                std::cout << Y << std::endl;
                for (size_t i = 0; i < num_vectors; i++)
                {
                    Y_opt(0, i) = vars[i * 3 + 0].get(GRB_DoubleAttr_X);
                    Y_opt(1, i) = vars[i * 3 + 1].get(GRB_DoubleAttr_X);
                    Y_opt(2, i) = vars[i * 3 + 2].get(GRB_DoubleAttr_X);
                }
            }
            else
            {
                std::cout << "Status: " << model.get(GRB_IntAttr_Status) << std::endl;
                exit(-1);
            }
        }
    }
    catch(GRBException& e)
    {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }
    catch(...)
    {
        std::cout << "Exception during optimization" << std::endl;
    }

    delete[] vars;
    return Y_opt;
}
