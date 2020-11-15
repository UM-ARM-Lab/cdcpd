#ifndef MLP_CLASSIFIER_H
#define MLP_CLASSIFIER_H

#include "smmap/classifier.h"
#include <torch/script.h>
#include "smmap/min_max_transformer.hpp"

namespace smmap
{
    class MLPClassifier : public Classifier
    {
    public:
        MLPClassifier(std::shared_ptr<ros::NodeHandle> nh,
                      std::shared_ptr<ros::NodeHandle> ph);

    private:
        virtual double predict_impl(Eigen::VectorXd const& vec) const override final;
        virtual void addData_impl(Eigen::MatrixXd const& features, std::vector<double> const& labels) override final;

        torch::jit::script::Module model_;
        double const threshold_;
        MinMaxTransformer scaler_;
    };
}

#endif // MLP_CLASSIFIER_H
