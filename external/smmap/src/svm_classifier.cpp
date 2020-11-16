#include "smmap/svm_classifier.h"
#include <stdlib.h>
#include <arc_utilities/ros_helpers.hpp>

using namespace smmap;

std::once_flag SVMClassifier::init_instance_flag_;
svm_model* SVMClassifier::model_;

SVMClassifier::SVMClassifier(std::shared_ptr<ros::NodeHandle> nh,
                             std::shared_ptr<ros::NodeHandle> ph)
    : Classifier(nh, ph, "svm")
    , scaler_(nh_, ph_)
{
    std::call_once(init_instance_flag_, SVMClassifier::Initialize, this);
}

void SVMClassifier::Initialize(SVMClassifier *svm)
{
    const auto filename = ROSHelpers::GetParamRequired<std::string>(*svm->ph_, "svm/model_file", __func__);
    model_ = svm_load_model(filename.c_str());
    assert(model_->nr_class = 2);
    assert(model_->l >= 1);
    assert(model_->nSV != nullptr);
    assert(model_->SV != nullptr);
    assert(svm->num_features_ > 0);
    assert(svm->num_features_ == model_->SV[0].dim - 1);
}

void SVMClassifier::Deinitialize()
{
    svm_free_and_destroy_model(&model_);
}

double SVMClassifier::predict_impl(Eigen::VectorXd const& vec) const
{
    assert(static_cast<int>(vec.rows()) == num_features_);

    Eigen::VectorXd const scaled_vec = scaler_(vec);

    svm_node query;
    query.dim = num_features_ + 1;
    query.values = (double*)(malloc(model_->SV[0].dim * sizeof(double)));
    query.values[0] = 0.0;
    memcpy(&query.values[1], scaled_vec.data(), num_features_ * sizeof(double));
    auto const mistake = svm_predict(model_, &query);
    free(query.values);
    return mistake;
}

void SVMClassifier::addData_impl(Eigen::MatrixXd const& features, std::vector<double> const& labels)
{
    throw_arc_exception(std::runtime_error, "Not implemented");
}
