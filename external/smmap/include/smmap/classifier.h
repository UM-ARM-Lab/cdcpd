#ifndef SMMAP_CLASSIFIER_H
#define SMMAP_CLASSIFIER_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <arc_utilities/ros_helpers.hpp>

namespace smmap
{
    class Classifier
    {
    protected:
        std::shared_ptr<ros::NodeHandle> const nh_;
        std::shared_ptr<ros::NodeHandle> const ph_;

    public:
        typedef std::shared_ptr<Classifier> Ptr;

        std::string const name_;
        int const num_features_;
        double const accuracy_;

        static Ptr MakeClassifier(std::shared_ptr<ros::NodeHandle> nh,
                                  std::shared_ptr<ros::NodeHandle> ph);

        Classifier(std::shared_ptr<ros::NodeHandle> nh,
                   std::shared_ptr<ros::NodeHandle> ph,
                   const std::string& name)
            : nh_(nh)
            , ph_(ph)
            , name_(name)
            , num_features_(ROSHelpers::GetParamRequired<int>(*ph_, "classifier/dim", __func__))
            , accuracy_(ROSHelpers::GetParamRequired<double>(*ph_, name + "/accuracy", __func__))
        {}

        double predict(Eigen::VectorXd const& vec) const
        {
            assert(vec.rows() == num_features_);
            return predict_impl(vec);
        }

        void addData(Eigen::MatrixXd const& features,
                     std::vector<double> const& labels)
        {
            assert(features.rows() == num_features_);
            assert(features.cols() == static_cast<ssize_t>(labels.size()));
            if (labels.size() == 0)
            {
                return;
            }
            addData_impl(features, labels);
        }

    protected:
        ~Classifier() {}

    private:
        virtual double predict_impl(Eigen::VectorXd const& vec) const = 0;
        virtual void addData_impl(Eigen::MatrixXd const& features, std::vector<double> const& labels) = 0;
    };
}

#endif // SMMAP_CLASSIFIER_H
