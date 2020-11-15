#include "smmap/knn_classifier.h"
#include <algorithm>
#include <fstream>
#include <arc_utilities/ros_helpers.hpp>

using namespace smmap;

std::once_flag kNNClassifier::init_instance_flag_;
Eigen::MatrixXd kNNClassifier::unscaled_raw_data_;
Eigen::MatrixXd kNNClassifier::scaled_raw_data_;
std::vector<double> kNNClassifier::labels_;
flann::KDTreeSingleIndex<flann::L2<double>> kNNClassifier::nn_index_;

static inline size_t GetLineCount(const std::string& filename)
{
    size_t num_lines = 0;
    std::ifstream in(filename);
    std::string unused;
    while (std::getline(in, unused))
    {
       ++num_lines;
    }
    return num_lines;
}

kNNClassifier::kNNClassifier(std::shared_ptr<ros::NodeHandle> nh,
                             std::shared_ptr<ros::NodeHandle> ph)
    : Classifier(nh, ph, "kNN")
    , scaler_(nh_, ph_)
{
    std::call_once(init_instance_flag_, kNNClassifier::Initialize, this);
}

void kNNClassifier::Initialize(kNNClassifier* const knn)
{
    auto const filename = ROSHelpers::GetParamRequired<std::string>(*knn->ph_, "kNN/data_file", __func__);
    auto const is_scaled_data = (filename.substr(filename.length() - 3) == "ssv");
    auto const line_count = GetLineCount(filename);
    ROS_INFO_STREAM_COND_NAMED(is_scaled_data, "kNN", "Parsing " << line_count << " scaled examples from " << filename);
    ROS_INFO_STREAM_COND_NAMED(!is_scaled_data, "kNN", "Parsing " << line_count << " un-scaled examples from " << filename);

    unscaled_raw_data_.resize(knn->num_features_, line_count);
    scaled_raw_data_.resize(knn->num_features_, line_count);
    labels_.resize(line_count);

    // Keep track of the number of lines parsed in order to catch parsing/data errors
    auto file = std::ifstream(filename);
    for (size_t line_num = 0; line_num < line_count; ++line_num)
    {
        std::string line;
        // TODO: replace this error parsing with execption style things:
        // https://stackoverflow.com/questions/17337602/how-to-get-error-message-when-ifstream-open-fails
        if (!std::getline(file, line))
        {
            std::stringstream ss;
            ss << "Error parsing " << filename << ": " << std::strerror(errno);
            ROS_ERROR_NAMED("kNN", ss.str().c_str());
            throw_arc_exception(std::runtime_error, ss.str());
        }

        // Replace commas with spaces to enable us to treat everything as a space seperated file
        std::replace(line.begin(), line.end(), ',', ' ');

        std::stringstream iss(line);
        double label;
        iss >> label;
        labels_[line_num] = label;

        Eigen::VectorXd features(knn->num_features_);
        for (int i = 0; i < knn->num_features_; ++i)
        {
            iss >> features(i);
        }

        if (is_scaled_data)
        {
            scaled_raw_data_.col(line_num) = features;
        }
        else
        {
            unscaled_raw_data_.col(line_num) = features;
        }
    }

    if (is_scaled_data)
    {
        knn->convertScaledDataToUnscaled(0);
    }
    else
    {
        // TODO: validate the scaler against the data
        knn->convertUnscaledDataToScaled(0);
    }

    // Only build the index if we have some data to work with
    if (labels_.size() > 0)
    {
        flann::Matrix<double> data(scaled_raw_data_.data(), labels_.size(), knn->num_features_);
        nn_index_.buildIndex(data);
    }
}

void kNNClassifier::convertScaledDataToUnscaled(const ssize_t start_idx)
{
    unscaled_raw_data_.conservativeResizeLike(scaled_raw_data_);
    for (ssize_t idx = start_idx; idx < unscaled_raw_data_.cols(); ++idx)
    {
        unscaled_raw_data_.col(idx) = scaler_.inverse(scaled_raw_data_.col(idx));
    }
}

void kNNClassifier::convertUnscaledDataToScaled(const ssize_t start_idx)
{
    scaled_raw_data_.conservativeResizeLike(unscaled_raw_data_);
    for (ssize_t idx = start_idx; idx < scaled_raw_data_.cols(); ++idx)
    {
        scaled_raw_data_.col(idx) = scaler_(unscaled_raw_data_.col(idx));
    }
}

double kNNClassifier::predict_impl(const Eigen::VectorXd& vec) const
{
    // If we don't have any data yet, then return "no mistake"
    if (labels_.size() == 0)
    {
        return -1.0;
    }

    Eigen::VectorXd scaled_vec = scaler_(vec);
    const flann::Matrix<double> query(scaled_vec.data(), 1, num_features_);

    const size_t knn = 1;
    std::vector<std::vector<size_t>> indices(query.rows, std::vector<size_t>(knn, -1));
    std::vector<std::vector<double>> dists(query.rows, std::vector<double>(knn, std::numeric_limits<double>::infinity()));

    const float eps = 0.0;
    const flann::SearchParams params(flann::flann_checks_t::FLANN_CHECKS_UNLIMITED, eps);
    nn_index_.knnSearch(query, indices, dists, knn, params);
    const auto mistake = labels_[indices[0][0]];
    return mistake;
}

void kNNClassifier::addData_impl(Eigen::MatrixXd const& features, std::vector<double> const& labels)
{
    assert(static_cast<ssize_t>(labels_.size()) == unscaled_raw_data_.cols());
    // Note that this function assumes that the scaler is already consistent with the existing data
    const auto num_starting_examples = unscaled_raw_data_.cols();
    const auto num_new_examples = features.cols();
    // Add the new data to our data cache
    unscaled_raw_data_.conservativeResize(Eigen::NoChange, num_starting_examples + num_new_examples);
    unscaled_raw_data_.rightCols(num_new_examples) = features;
    // Update the scaler with the new data and re-create the scaled data
    scaler_.addDataExamples(features);
    convertUnscaledDataToScaled(0);
    // Aggreate the new labels
    labels_.insert(labels_.end(), labels.begin(), labels.end());
    // Update the knn index with the new scaled data
    flann::Matrix<double> data(scaled_raw_data_.data(), labels_.size(), num_features_);
    nn_index_.buildIndex(data);
}
