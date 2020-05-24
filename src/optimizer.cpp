#include "cdcpd/optimizer.h"

using Eigen::Matrix3Xf;
using Eigen::Matrix3Xd;
using Eigen::Matrix2Xi;
using Eigen::Vector3f;

#include <iostream>
using std::cout;
using std::endl;

/* interaction_cylinder.bag
Vector3f cylinder_orien(0.004483963943558, 0.121338945834278, 0.130282864480891);
Vector3f cylinder_center(-0.043180266753345, 0.038185108108776, 0.968493909342117);
float cylinder_radius = 0.036371412988240;
float cylinder_height = 0.178092308891112;
 */

/* interaction_cylinder_2.bag
Vector3f cylinder_orien(-0.008885936014668, 0.101494242992091, 0.115133360576856);
Vector3f cylinder_center(0.145124522395497, -0.152708792314512, 1.095150852162702);
float cylinder_radius = 0.033137245873063;
float cylinder_height = 0.153739168519654;
 */

// interaction_cylinder_4.bag
Vector3f cylinder_orien(-0.001324255947704, 0.058704082788457, 0.128014310218385);
Vector3f cylinder_center(-0.001783838376740, -0.202407765852103, 1.255950979292225);
float cylinder_radius = 0.05;
float cylinder_height = 0.21;

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

std::tuple<Matrix3Xf, Matrix3Xf>
        nearest_points_and_normal(const Matrix3Xf& last_template) {
    // find of the nearest points and corresponding normal vector on the cylinder
    Matrix3Xf nearestPts(3, last_template.cols());
    Matrix3Xf normalVecs(3, last_template.cols());
    for (int i = 0; i < last_template.cols(); i++) {
        Vector3f pt;
        pt << last_template.col(i);
        Vector3f unitVecH = cylinder_orien/cylinder_orien.norm();
        Vector3f unitVecR = (pt - cylinder_center) - ((pt - cylinder_center).transpose()*unitVecH)*unitVecH;
        unitVecR = unitVecR/unitVecR.norm();
        float h = unitVecH.transpose()*(pt-cylinder_center);
        float r = unitVecR.transpose()*(pt-cylinder_center);
        Vector3f nearestPt;
        Vector3f normalVec;
        if (h > cylinder_height/2 && r >= cylinder_radius) {
            nearestPt = unitVecR*cylinder_radius + unitVecH*cylinder_height/2 + cylinder_center;
            normalVec = unitVecR + unitVecH;
        }
        else if (h < -cylinder_height/2 && r >= cylinder_radius) {
            nearestPt = unitVecR*cylinder_radius - unitVecH*cylinder_height/2 + cylinder_center;
            normalVec = unitVecR - cylinder_orien/cylinder_orien.norm();
        }
        else if (h > cylinder_height/2 && r < cylinder_radius) {
            nearestPt = r*unitVecR + cylinder_height/2*unitVecH + cylinder_center;
            normalVec = unitVecH;
        }
        else if (h < -cylinder_height/2 && r < cylinder_radius) {
            nearestPt = r*unitVecR - cylinder_height/2*unitVecH + cylinder_center;
            normalVec = -unitVecH;
        }
        else if (h <= cylinder_height/2 && h >= -cylinder_height/2 && r < cylinder_radius) {
            if (cylinder_height/2 - h < cylinder_radius - r) {
                nearestPt = r*unitVecR + cylinder_height/2*unitVecH + cylinder_center;
                normalVec = unitVecH;
            }
            else if (h + cylinder_height/2 < cylinder_radius - r) {
                nearestPt = r*unitVecR + cylinder_height/2*unitVecH + cylinder_center;
                normalVec = -unitVecH;
            } else {
                nearestPt = cylinder_radius*unitVecR + h*unitVecH + cylinder_center;
                normalVec = unitVecR;
            }
        }
        else if (h <= cylinder_height/2 && h >= -cylinder_height/2 && r >= cylinder_radius) {
            nearestPt = cylinder_radius*unitVecR + h*unitVecH + cylinder_center;
            normalVec = unitVecR;
        }
        normalVec = normalVec/normalVec.norm();
        for (int j = 0; j < 3; ++j) {
            nearestPts(j, i) = nearestPt(j);
            normalVecs(j, i) = normalVec(j);
        }
    }
    return {nearestPts, normalVecs};
}


// TODO do setup here
Optimizer::Optimizer(const Eigen::Matrix3Xf _init_temp, const Eigen::Matrix3Xf _last_temp, const float _stretch_lambda)
    : initial_template(_init_temp), last_template(_last_temp), stretch_lambda(_stretch_lambda)
{
}

Matrix3Xf Optimizer::operator()(const Matrix3Xf& Y, const Matrix2Xi& E, const std::vector<CDCPD::FixedPoint>& fixed_points)
{
    // Y: Y^t in Eq. (21)
    // E: E in Eq. (21)
    Matrix3Xf Y_opt(Y.rows(), Y.cols());
    GRBVar* vars = nullptr;
    auto [nearestPts, normalVecs] = nearest_points_and_normal(last_template);
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
        // ???: why multiply stretch_lambda twice
        {
            for (size_t i = 0; i < E.cols(); ++i)
            {
                model.addQConstr(
                            buildDifferencingQuadraticTerm(&vars[E(0, i) * 3], &vars[E(1, i) * 3], 3),
                            GRB_LESS_EQUAL,
                            stretch_lambda * stretch_lambda * (initial_template.col(E(0, i)) - initial_template.col(E(1, i))).squaredNorm(),
                            "upper_edge_" + std::to_string(E(0, i)) + "_to_" + std::to_string(E(1, i)));
            }
            model.update();
        }


        // Add interaction constraints
        {
            for (size_t i = 0; i < num_vectors; ++i) {
                model.addConstr(
                        (vars[i*3 + 0] - nearestPts(0, i))*normalVecs(0, i) +
                                (vars[i*3 + 1] - nearestPts(1, i))*normalVecs(1, i) +
                                (vars[i*3 + 2] - nearestPts(2, i))*normalVecs(2, i) >= 0, "interaction constrain for point " +std::to_string(i));
            }
        }


        Matrix3Xd Y_copy = Y.cast<double>(); // TODO is this exactly what we want?

        // Next, add the fixed point constraints that we might have.
        // TODO make this more
        // First, make sure that the constraints can be satisfied
        GRBQuadExpr gripper_objective_fn(0);
        if (all_constraints_satisfiable(fixed_points))
        {
            // If that's possible, we'll require that all constraints are equal
            for (const auto& fixed_point : fixed_points)
            {
                model.addConstr(vars[3 * fixed_point.template_index + 0], GRB_EQUAL, fixed_point.position(0), "fixed_point");
                model.addConstr(vars[3 * fixed_point.template_index + 1], GRB_EQUAL, fixed_point.position(1), "fixed_point");
                model.addConstr(vars[3 * fixed_point.template_index + 2], GRB_EQUAL, fixed_point.position(2), "fixed_point");
            }
        }
        else
        {
            cout << "Gripper constraint cannot be satisfied." << endl;
            for (const auto& fixed_point : fixed_points)
            {
                const auto expr0 = vars[fixed_point.template_index + 0] - Y_copy(0, fixed_point.template_index);
                const auto expr1 = vars[fixed_point.template_index + 1] - Y_copy(1, fixed_point.template_index);
                const auto expr2 = vars[fixed_point.template_index + 2] - Y_copy(2, fixed_point.template_index);
                gripper_objective_fn += 100.0 * (expr0 * expr0 + expr1 * expr1 + expr2 * expr2);
            }
        }

        // Build the objective function
        {
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
                // std::cout << "Y" << std::endl;
                // std::cout << Y << std::endl;
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

bool Optimizer::all_constraints_satisfiable(const std::vector<CDCPD::FixedPoint>& fixed_points) const
{
    for (auto first_elem = fixed_points.cbegin(); first_elem != fixed_points.cend(); ++first_elem)
    {
        for (auto second_elem = first_elem + 1; second_elem != fixed_points.cend(); ++second_elem)
        {
            float current_distance = (first_elem->position - second_elem->position).squaredNorm();
            float original_distance = (initial_template.col(first_elem->template_index) - initial_template.col(second_elem->template_index)).squaredNorm();
            cout << "current_distance " << current_distance << endl;
            cout << "original_distance " << original_distance << endl;
            if (current_distance > original_distance * stretch_lambda * stretch_lambda)
            {
                return false;
            }
        }
    }
    return true;
}
