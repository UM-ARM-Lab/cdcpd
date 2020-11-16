#include "smmap/mlp_classifier.h"
#include <arc_utilities/ros_helpers.hpp>

using namespace smmap;

inline static std::string getModelFilename(ros::NodeHandle& nh)
{
    return ROSHelpers::GetParamRequired<std::string>(nh, "mlp/model_file", __func__);
}

MLPClassifier::MLPClassifier(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph)
    : Classifier(nh, ph, "mlp")
    , model_(torch::jit::load(getModelFilename(*ph_)))
    , threshold_(ROSHelpers::GetParamRequired<double>(*ph_, "mlp/threshold", __func__))
    , scaler_(nh_, ph_)
{
    // Ensure that the model's forward module has already been initialized
    // https://github.com/pytorch/pytorch/issues/23920#issuecomment-519355570
    (void)model_.get_method("forward");
}

double MLPClassifier::predict_impl(Eigen::VectorXd const& vec) const
{
    Eigen::VectorXd const scaled_vec = scaler_(vec);
    auto vec_torch = torch::empty({num_features_});
    for (int idx = 0; idx < num_features_; ++idx)
    {
        vec_torch[idx] = scaled_vec[idx];
    }
    std::vector<torch::jit::IValue> const query(1, vec_torch);
    // Our particular forward is threadsafe per
    // https://github.com/pytorch/pytorch/issues/23920#issuecomment-519355570
    auto const output = const_cast<MLPClassifier*>(this)->model_.forward(query).toTensor().item().toFloat();
    auto const mistake = (output > threshold_) ? 1.0 : -1.0;
    return mistake;
}

void MLPClassifier::addData_impl(Eigen::MatrixXd const& features, std::vector<double> const& labels)
{
    throw_arc_exception(std::runtime_error, "Not implemented");
}
