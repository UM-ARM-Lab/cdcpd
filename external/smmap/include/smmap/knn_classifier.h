#ifndef KNN_CLASSIFIER_H
#define KNN_CLASSIFIER_H

#include "smmap/classifier.h"
#include <flann/flann.hpp>
#include <mutex>
#include "smmap/min_max_transformer.hpp"

namespace smmap
{
    class kNNClassifier : public Classifier
    {
    public:
        kNNClassifier(std::shared_ptr<ros::NodeHandle> nh,
                      std::shared_ptr<ros::NodeHandle> ph);

    private:
        virtual double predict_impl(Eigen::VectorXd const& vec) const override final;
        virtual void addData_impl(Eigen::MatrixXd const& features, std::vector<double> const& labels) override final;

        // https://www.modernescpp.com/index.php/thread-safe-initialization-of-data
        static void Initialize(kNNClassifier* const knn);
        static std::once_flag init_instance_flag_;

        void convertScaledDataToUnscaled(const ssize_t start_idx);
        void convertUnscaledDataToScaled(const ssize_t start_idx);

        static Eigen::MatrixXd unscaled_raw_data_;
        static Eigen::MatrixXd scaled_raw_data_;
        static std::vector<double> labels_;
        static flann::KDTreeSingleIndex<flann::L2<double>> nn_index_;

        MinMaxTransformer scaler_;
    };
}

#endif // NN_CLASSIFIER_H
