#include <Eigen/Dense>
#include <gurobi_c++.h>

////////////////////////////////////////////////////////////////
// Internally used static objects
////////////////////////////////////////////////////////////////

class Optimizer
{
public:
    Optimizer(const Eigen::Matrix3Xf _init_temp, const float _stretch_lambda);

    Eigen::MatrixXf operator()(const Eigen::Matrix3Xf& Y, const Eigen::MatrixXi& E);
private:
    Eigen::Matrix3Xf initial_template;
    float stretch_lambda;
};
