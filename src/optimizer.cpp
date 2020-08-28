#include "cdcpd/optimizer.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel             K;
typedef K::FT                                                           FT;
typedef K::Point_3                                                      Point_3;
typedef CGAL::Surface_mesh<Point_3>                                     Mesh;

namespace PMP = CGAL::Polygon_mesh_processing;

typedef PMP::Face_location<Mesh, FT>                                    Face_location;

using Eigen::Matrix3Xf;
using Eigen::MatrixXf;
using Eigen::Matrix3Xd;
using Eigen::Matrix2Xi;
using Eigen::Vector3f;
using Eigen::VectorXf;

#include <iostream>
using std::cout;
using std::endl;
using std::min;
using std::max;

#ifdef CYL1
// interaction_cylinder.bag
Vector3f cylinder_orien(0.004483963943558, 0.121338945834278, 0.130282864480891);
Vector3f cylinder_center(-0.043180266753345, 0.038185108108776, 0.968493909342117);
float cylinder_radius = 0.036371412988240;
float cylinder_height = 0.178092308891112;
#endif

#ifdef CYL2
// interaction_cylinder_2.bag
Vector3f cylinder_orien(-0.008885936014668, 0.101494242992091, 0.115133360576856);
Vector3f cylinder_center(0.145124522395497, -0.152708792314512, 1.095150852162702);
float cylinder_radius = 0.033137245873063;
float cylinder_height = 0.153739168519654;
#endif

#ifdef CYL4
// interaction_cylinder_4.bag
Vector3f cylinder_orien(-0.001324255947704, 0.058704082788457, 0.128014310218385);
Vector3f cylinder_center(-0.001783838376740, -0.202407765852103, 1.255950979292225);
float cylinder_radius = 0.05;
float cylinder_height = 0.21;
#endif

#ifdef CYL5
// interaction_cylinder_5.bag
Vector3f cylinder_orien(-0.013265312948576, 0.271214729597447, -0.068290358018994);
Vector3f cylinder_center(-0.007203971514259, -0.282011643023486, 1.351697407251410);
float cylinder_radius = 0.05;
float cylinder_height = 0.21;
#endif

#ifdef CYL6
// interaction_cylinder_6.bag
Vector3f cylinder_orien(-0.001063927061630, 0.262452937850508, -0.062874506695239);
Vector3f cylinder_center(-0.025889295027034, -0.020591825574503, 1.200787565152055);
float cylinder_radius = 0.05;
float cylinder_height = 0.21;
#endif

#ifdef CYL7
// interaction_cylinder_7.bag
Vector3f cylinder_orien(-0.000711277105401, 0.271567425774266, -0.074522999332804 );
Vector3f cylinder_center(-0.028109420928789, 0.024601311519531, 1.344487500578158);
float cylinder_radius = 0.05;
float cylinder_height = 0.21;
#endif

#ifdef CYL8
// interaction_cylinder_8.bag
Vector3f cylinder_orien(-0.000598250974917, 0.063758412010692, 0.160749191566454);
Vector3f cylinder_center(-0.239953252695972, -0.326861315788172, 1.459887097878595);
float cylinder_radius = 0.05;
float cylinder_height = 0.21;
#endif

#ifdef CYL9
// interaction_cylinder_9.bag
Vector3f cylinder_orien(-0.000598250974917, 0.063758412010692, 0.160749191566454);
Vector3f cylinder_center(-0.239953252695972, -0.28, 1.459887097878595);
float cylinder_radius = 0.05;
float cylinder_height = 0.21;
#endif

#ifdef CYL_CLOTH1
// interaction_cloth1.bag
Vector3f cylinder_orien(0.009859699816896, 0.043357983138650, 0.170135231471733);
Vector3f cylinder_center(-0.114121248950204, -0.180876677250917, 1.384255148567173);
float cylinder_radius = 0.05;
float cylinder_height = 0.21;
#endif

#ifdef CYL_CLOTH3
// interaction_cloth3.bag
Vector3f cylinder_orien(0.009859699816896, 0.043357983138650, 0.170135231471733);
Vector3f cylinder_center(-0.134121248950204, -0.110876677250917, 1.384255148567173);
float cylinder_radius = 0.05;
float cylinder_height = 0.21;
#endif

#ifdef CYL_CLOTH4
// interaction_cloth3.bag
Vector3f cylinder_orien(0.009859699816896, 0.043357983138650, 0.170135231471733);
Vector3f cylinder_center(-0.134121248950204, -0.110876677250917, 1.384255148567173);
float cylinder_radius = 0.05;
float cylinder_height = 0.21;
#endif



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

#ifdef CYLINDER_INTER
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
#endif

#ifdef SHAPE_COMP

static Vector3f Pt3toVec(const Point_3 pt)
{
	return Vector3f(float(pt.x()), float(pt.y()), float(pt.z()));
}

std::tuple<Matrix3Xf, Matrix3Xf>
        Optimizer::nearest_points_and_normal(const Matrix3Xf& last_template)
{
	Matrix3Xf nearestPts(3, last_template.cols());
    Matrix3Xf normalVecs(3, last_template.cols());

	for(int pt_ind = 0; pt_ind < last_template.cols(); pt_ind++)
	{
		Point_3 pt(last_template(0, pt_ind),
				   last_template(1, pt_ind),
				   last_template(2, pt_ind));
		Face_location query_location = PMP::locate(pt, mesh);
		Point_3 nearestPt = PMP::construct_point(query_location, mesh);
		nearestPts.col(pt_ind) = Pt3toVec(nearestPt);

		double w[3];
		for(int i = 0; i < 3; i++){
			w[i] = query_location.second[i];
		}

		MatrixXf verts_of_face(3, 3);
		verts_of_face.col(0) = Pt3toVec(mesh.point(source(halfedge(query_location.first,mesh),mesh)));
		verts_of_face.col(1) = Pt3toVec(mesh.point(target(halfedge(query_location.first,mesh),mesh)));
		verts_of_face.col(2) = Pt3toVec(mesh.point(target(next(halfedge(query_location.first,mesh),mesh),mesh)));
		
		Vector3f normalVec(0.0, 0.0, 0.0);
		for (int i = 0; i < 3; i++) {
			for (int mesh_ind = 0; mesh_ind < obs_mesh.cols(); mesh_ind++)
			{
				if (verts_of_face.col(i).isApprox(obs_mesh.col(mesh_ind))) {
					normalVec = normalVec + obs_normal.col(mesh_ind) * w[i];
				}
			}
		}
		
		normalVecs.col(pt_ind) = normalVec;
	}
}
#endif

std::tuple<MatrixXf, MatrixXf>
    nearest_points_line_segments(const Matrix3Xf& last_template, const Matrix2Xi& E) {
    // find the nearest points on the line segments
    // refer to the website https://math.stackexchange.com/questions/846054/closest-points-on-two-line-segments
    MatrixXf startPts(4, E.cols()*E.cols()); // Matrix: 3 * E^2: startPts.col(E*cols()*i + j) is the nearest point on edge i w.r.t. j
    MatrixXf endPts(4, E.cols()*E.cols()); // Matrix: 3 * E^2: endPts.col(E*cols()*i + j) is the nearest point on edge j w.r.t. i
    for (int i = 0; i < E.cols(); ++i)
    {
        Vector3f P1 = last_template.col(E(0, i));
        Vector3f P2 = last_template.col(E(1, i));
        // cout << "P1:" << endl;
        // cout << P1 << endl << endl;
        // cout << "P2:" << endl;
        // cout << P2 << endl << endl;
        for (int j = 0; j < E.cols(); ++j)
        {
            Vector3f P3 = last_template.col(E(0, j));
            Vector3f P4 = last_template.col(E(1, j));
            
            // cout << "P3:" << endl;
            // cout << P3 << endl << endl;
            // cout << "P4:" << endl;
            // cout << P4 << endl << endl;

            float R21 = (P2-P1).squaredNorm();
            float R22 = (P4-P3).squaredNorm();
            float D4321 = (P4-P3).dot(P2-P1);
            float D3121 = (P3-P1).dot(P2-P1);
            float D4331 = (P4-P3).dot(P3-P1);

            // cout << "original s:" << (-D4321*D4331+D3121*R22)/(R21*R22-D4321*D4321) << endl;
            // cout << "original t:" << (D4321*D3121-D4331*R21)/(R21*R22-D4321*D4321) << endl;

            float s;
            float t;
            
            if (R21*R22-D4321*D4321 != 0)
            {
                s = min(max((-D4321*D4331+D3121*R22)/(R21*R22-D4321*D4321), 0.0f), 1.0f);
                t = min(max((D4321*D3121-D4331*R21)/(R21*R22-D4321*D4321), 0.0f), 1.0f);
            } else {
                // means P1 P2 P3 P4 are on the same line
                float P13 = (P3 - P1).squaredNorm();
                s = 0; t = 0;
                float P14 = (P4 - P1).squaredNorm();
                if (P14 < P13) {
                    s = 0; t = 1;
                }
                float P23 = (P3 - P2).squaredNorm();
                if (P23 < P14 && P23 < P13)
                {
                    s = 1; t = 0;
                }
                float P24 = (P4 - P2).squaredNorm();
                if (P24 < P23 && P24 < P14 && P24 < P13) {
                    s = 1; t = 1;
                }
            }
            // cout << "s: " << s << endl;
            // cout << "t: " << t << endl;

            for (int dim = 0; dim < 3; ++dim)
            {
                startPts(dim, E.cols()*i+j) = (1-s)*P1(dim)+s*P2(dim);
                endPts(dim, E.cols()*i+j) = (1-t)*P3(dim)+t*P4(dim);
            }
            startPts(3, E.cols()*i+j) = s;
            endPts(3, E.cols()*i+j) = t;
        }
    }
    return {startPts, endPts};
}

void Wsolver(const MatrixXf& P, const Matrix3Xf& X, const Matrix3Xf& Y, const MatrixXf& G, const MatrixXf& L, const double sigma2, const double alpha, const double lambda, MatrixXf& W) {
    try {
    GRBEnv& env = getGRBEnv();
    env.set(GRB_IntParam_OutputFlag, 0);
    GRBModel model(env);
    model.set("ScaleFlag", "2");
    model.set("NonConvex", "2");

    GRBVar* vars = nullptr;

    const ssize_t num_vars = 3 * X.cols();
        
    // Note that variable bound is important, without a bound, Gurobi defaults to 0, which is clearly unwanted
    const std::vector<double> lb(num_vars, -GRB_INFINITY);
    const std::vector<double> ub(num_vars, GRB_INFINITY);
    vars = model.addVars(lb.data(), ub.data(), nullptr, nullptr, nullptr, (int) num_vars);
    model.update();

    GRBQuadExpr objective_fn(0);
    for (ssize_t m = 0; m < Y.cols(); ++m)
    {
        for (ssize_t n = 0; n < X.cols(); ++n)
        {
            GRBLinExpr GWx(0);
            GRBLinExpr GWy(0);
            GRBLinExpr GWz(0);
            for (ssize_t i = 0; i < Y.cols(); ++i)
            {
                GWx += G(m, i)*vars[i * 3 + 0];
                GWy += G(m, i)*vars[i * 3 + 1];
                GWz += G(m, i)*vars[i * 3 + 2];
            }
            auto diffx = X(0, n) - (Y(0, m) + GWx);
            auto diffy = X(1, n) - (Y(1, m) + GWy);
            auto diffz = X(2, n) - (Y(2, m) + GWz);
            objective_fn += (P(m, n) /sigma2) * (diffx * diffx + diffy * diffy + diffz * diffz);
        }
    }

    for (ssize_t d = 0; d < 3; ++d)
    {
        for (ssize_t m = 0; m < Y.cols(); ++m)
        {
            GRBLinExpr WTG(0);
            for (ssize_t i = 0; i < Y.cols(); ++i)
            {
                WTG += G(m, i)*vars[i * 3 + d];
            }
            auto WTGW = WTG * vars[m * 3 + d];
            objective_fn += (alpha/2)*WTGW;
        }
    }

    for (int m = 0; m < Y.cols(); ++m)
    {
        GRBLinExpr Tmx(0);
        GRBLinExpr Tmy(0);
        GRBLinExpr Tmz(0);
        GRBLinExpr Tix(0);
        GRBLinExpr Tiy(0);
        GRBLinExpr Tiz(0);
        for (ssize_t i = 0; i < Y.cols(); ++i)
        {
            Tmx += G(m, i)*vars[i * 3 + 0];
            Tmy += G(m, i)*vars[i * 3 + 1];
            Tmz += G(m, i)*vars[i * 3 + 2];
        }
        Tmx += Y(0, m);
        Tmy += Y(1, m);
        Tmz += Y(2, m);
        for (ssize_t i = 0; i < L.cols(); ++i)
        {
            if (L(m, i) < 0.00000001 && L(m, i) > -0.00000001) {
                for (ssize_t j = 0; j < Y.cols(); ++j)
                {
                    Tix += L(m, i)*G(m, j)*vars[j * 3 + 0];
                    Tiy += L(m, i)*G(m, j)*vars[j * 3 + 1];
                    Tiz += L(m, i)*G(m, j)*vars[j * 3 + 2];
                }
                Tix += L(m, i)*Y(0, i);
                Tiy += L(m, i)*Y(1, i);
                Tiz += L(m, i)*Y(2, i);
            }
        }
        auto diffx = Tmx - Tix;
        auto diffy = Tmy - Tiy;
        auto diffz = Tmz - Tiz;
        objective_fn += (lambda/2) * (diffx * diffx + diffy * diffy + diffz * diffz);
    }
    model.setObjective(objective_fn, GRB_MINIMIZE);
    model.update();

    // Find the optimal solution, and extract it
    {
        model.optimize();
        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
        {
            // std::cout << "Y" << std::endl;
            // std::cout << Y << std::endl;
            for (ssize_t i = 0; i < Y.cols(); i++)
            {
                W(0, i) = vars[i * 3 + 0].get(GRB_DoubleAttr_X);
                W(1, i) = vars[i * 3 + 1].get(GRB_DoubleAttr_X);
                W(2, i) = vars[i * 3 + 2].get(GRB_DoubleAttr_X);
            }
        }
        else
        {
            std::cout << "Status: " << model.get(GRB_IntAttr_Status) << std::endl;
            exit(-1);
        }
    }
    delete[] vars;
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
}

#ifdef SHAPE_COMP
Optimizer::Optimizer(const Eigen::Matrix3Xf _init_temp, const Eigen::Matrix3Xf _last_temp, const float _stretch_lambda, const obsParam& obstacle_param)
    : initial_template(_init_temp),
      last_template(_last_temp),
      stretch_lambda(_stretch_lambda),
      obs_mesh(obstacle_param.verts),
      obs_normal(obstacle_param.normals),
      mesh(initObstacle(obstacle_param))
{
    
}
#else
Optimizer::Optimizer(const Eigen::Matrix3Xf _init_temp, const Eigen::Matrix3Xf _last_temp, const float _stretch_lambda)
    : initial_template(_init_temp),
      last_template(_last_temp),
      stretch_lambda(_stretch_lambda)
{
    
}
#endif

Matrix3Xf Optimizer::operator()(const Matrix3Xf& Y, const Matrix2Xi& E, const std::vector<CDCPD::FixedPoint>& fixed_points, const bool self_intersection, const bool interation_constrain)
{
    // Y: Y^t in Eq. (21)
    // E: E in Eq. (21)

    Matrix3Xf Y_opt(Y.rows(), Y.cols());
    GRBVar* vars = nullptr;
    try
    {
        const ssize_t num_vectors = Y.cols();
        const ssize_t num_vars = 3 * num_vectors;

        GRBEnv& env = getGRBEnv();

        // Disables logging to file and logging to console (with a 0 as the value of the flag)
        env.set(GRB_IntParam_OutputFlag, 0);
        GRBModel model(env);
        model.set("ScaleFlag", "2");
        // model.set("OutputFlag", "1");

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
            for (ssize_t i = 0; i < E.cols(); ++i)
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


        if (interation_constrain)
        {
#ifdef SHAPE_COMP
            auto [nearestPts, normalVecs] = nearest_points_and_normal(last_template);
            cout << "added interaction constrain" << endl;
            for (ssize_t i = 0; i < num_vectors; ++i) {
                model.addConstr(
                        (vars[i*3 + 0] - nearestPts(0, i))*normalVecs(0, i) +
                        (vars[i*3 + 1] - nearestPts(1, i))*normalVecs(1, i) +
                        (vars[i*3 + 2] - nearestPts(2, i))*normalVecs(2, i) >= 0, "interaction constrain for point " +std::to_string(i));
            }
#endif
        }

        if (self_intersection)
        {
            cout << "added self intersection constrain" << endl;
            auto [startPts, endPts] = nearest_points_line_segments(last_template, E);
            for (int row = 0; row < E.cols(); ++row)
            {
                Vector3f P1 = last_template.col(E(0, row));
                Vector3f P2 = last_template.col(E(1, row));
                for (int col = 0; col < E.cols(); ++col)
                {
                    float s = startPts(3, row*E.cols() + col);
                    float t = endPts(3, row*E.cols() + col);
                    Vector3f P3 = last_template.col(E(0, col));
                    Vector3f P4 = last_template.col(E(1, col));
                    float l = (endPts.col(row*E.cols() + col).topRows(3) - startPts.col(row*E.cols() + col).topRows(3)).norm();
                    if (!P1.isApprox(P3) && !P1.isApprox(P4) && !P2.isApprox(P3) && !P2.isApprox(P4) && l <= 0.02) {
                        // model.addConstr((vars[E(0, col)*3 + 0] - startPts(0, row*E.cols() + col))*(endPts(0, row*E.cols() + col) - startPts(0, row*E.cols() + col)) +
                        //                 (vars[E(0, col)*3 + 1] - startPts(1, row*E.cols() + col))*(endPts(1, row*E.cols() + col) - startPts(1, row*E.cols() + col)) +
                        //                 (vars[E(0, col)*3 + 2] - startPts(2, row*E.cols() + col))*(endPts(2, row*E.cols() + col) - startPts(2, row*E.cols() + col)) >= 0);
                        // model.addConstr((vars[E(1, col)*3 + 0] - startPts(0, row*E.cols() + col))*(endPts(0, row*E.cols() + col) - startPts(0, row*E.cols() + col)) +
                        //                 (vars[E(1, col)*3 + 1] - startPts(1, row*E.cols() + col))*(endPts(1, row*E.cols() + col) - startPts(1, row*E.cols() + col)) +
                        //                 (vars[E(1, col)*3 + 2] - startPts(2, row*E.cols() + col))*(endPts(2, row*E.cols() + col) - startPts(2, row*E.cols() + col)) >= 0);
                        model.addConstr(((vars[E(0, col)*3 + 0]*(1-t) + vars[E(1, col)*3 + 0]*t) - (vars[E(0, row)*3 + 0]*(1-s) + vars[E(1, row)*3 + 0]*s))
                                            *(endPts(0, row*E.cols() + col) - startPts(0, row*E.cols() + col)) +
                                        ((vars[E(0, col)*3 + 1]*(1-t) + vars[E(1, col)*3 + 1]*t) - (vars[E(0, row)*3 + 1]*(1-s) + vars[E(1, row)*3 + 1]*s))
                                            *(endPts(1, row*E.cols() + col) - startPts(1, row*E.cols() + col)) +
                                        ((vars[E(0, col)*3 + 2]*(1-t) + vars[E(1, col)*3 + 2]*t) - (vars[E(0, row)*3 + 2]*(1-s) + vars[E(1, row)*3 + 2]*s))
                                            *(endPts(2, row*E.cols() + col) - startPts(2, row*E.cols() + col)) >= 0.01 * l);
                    }
                }
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
            for (ssize_t i = 0; i < num_vectors; ++i)
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
                for (ssize_t i = 0; i < num_vectors; i++)
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
            // cout << "current_distance " << current_distance << endl;
            // cout << "original_distance " << original_distance << endl;
            if (current_distance > original_distance * stretch_lambda * stretch_lambda)
            {
                return false;
            }
        }
    }
    return true;
}

#ifdef SHAPE_COMP
Mesh Optimizer::initObstacle(obsParam obs_param)
{
	Mesh mesh;
	for(int face_ind = 0; face_ind < obs_param.faces.cols(); face_ind++)
	{
		std::vector<Mesh::Vertex_index> indices;
		for(int i = 0; i < 3; i++)
		{
			int pt_ind = int(obs_param.faces(i, face_ind));
			indices.push_back(mesh.add_vertex(Point_3(obs_param.verts(0, pt_ind),
												      obs_param.verts(1, pt_ind),
												      obs_param.verts(2, pt_ind))));
		}
		mesh.add_face(indices[0], indices[1], indices[2]);
	}
	return mesh;
}
#endif

