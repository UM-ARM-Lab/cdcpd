#ifndef SVM_CLASSIFIER_H
#define SVM_CLASSIFIER_H

#include "smmap/classifier.h"
#include <svm/svm.h>
#include <mutex>
#include "smmap/min_max_transformer.hpp"

namespace smmap
{
    class SVMClassifier : public Classifier
    {
    public:
        SVMClassifier(std::shared_ptr<ros::NodeHandle> nh,
                      std::shared_ptr<ros::NodeHandle> ph);

    private:
        virtual double predict_impl(Eigen::VectorXd const& vec) const override final;
        virtual void addData_impl(Eigen::MatrixXd const& features, std::vector<double> const& labels) override final;

        static void Initialize(SVMClassifier* svm);
        static void Deinitialize();
        static std::once_flag init_instance_flag_;
        static svm_model* model_;

        MinMaxTransformer scaler_;
    };
}

#endif // SVM_CLASSIFIER_H
