#include "smmap/classifier.h"
#include "smmap/no_classifier.h"
#include "smmap/knn_classifier.h"
#include "smmap/svm_classifier.h"
#include "smmap/mlp_classifier.h"
#include <deformable_manipulation_experiment_params/ros_params.hpp>

using namespace smmap;

Classifier::Ptr Classifier::MakeClassifier(
        std::shared_ptr<ros::NodeHandle> nh,
        std::shared_ptr<ros::NodeHandle> ph)
{
    ClassifierType const type = GetClassifierType(*ph);
    switch (type)
    {
        case None:
            return std::make_shared<NoClassifier>(nh, ph);

        case kNN:
            return std::make_shared<kNNClassifier>(nh, ph);

        case SVM:
            return std::make_shared<SVMClassifier>(nh, ph);

        case MLP:
            return std::make_shared<MLPClassifier>(nh, ph);

        case VOXNET:
            #pragma warning "Voxnet classifier hack addition to classification framework"
            return nullptr;

        default:
            ROS_FATAL("Invalid classifier type in MakeClassifier(), this should not be possible");
            throw_arc_exception(std::invalid_argument, "Invalid classifier type in MakeClassifier(), this should not be possible");
            return nullptr;
    }
}
