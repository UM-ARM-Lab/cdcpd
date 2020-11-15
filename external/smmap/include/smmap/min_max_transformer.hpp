#ifndef MIN_MAX_TRANSFORMER_HPP
#define MIN_MAX_TRANSFORMER_HPP

#include <arc_utilities/ros_helpers.hpp>
#include <Eigen/Dense>

namespace smmap
{
    class MinMaxTransformer
    {
    public:
        MinMaxTransformer(std::shared_ptr<ros::NodeHandle> nh,
                          std::shared_ptr<ros::NodeHandle> ph)
            : feature_min_(-1.0)
            , feature_max_(1.0)
        {
            (void)nh;
            const int vec_len = ROSHelpers::GetParamRequired<int>(*ph, "classifier/dim", __func__);
            const auto scale = ROSHelpers::GetVectorRequired<double>(*ph, "classifier/transformer/scale", __func__);
            const auto offset = ROSHelpers::GetVectorRequired<double>(*ph, "classifier/transformer/offset", __func__);
            assert(static_cast<int>(scale.size()) == vec_len);
            assert(static_cast<int>(offset.size()) == vec_len);

            scale_ = Eigen::VectorXd::Map(scale.data(), vec_len);
            offset_ = Eigen::VectorXd::Map(offset.data(), vec_len);
            calcDataRangeFromScaleOffset();
        }

        Eigen::VectorXd operator()(const Eigen::VectorXd& x) const
        {
            return x.cwiseProduct(scale_) + offset_;
        }

        Eigen::VectorXd inverse(const Eigen::VectorXd& sx) const
        {
            return (sx - offset_).cwiseQuotient(scale_);
        }

        // Sets the scalar parameters based only on the new data
        // Data is assumed to be D x N, where D is the data dimension, and N is the number of examples
        void setDataExamples(const Eigen::MatrixXd& data)
        {
            assert(data.rows() == scale_.rows());
            data_min_ = data.rowwise().minCoeff();
            data_max_ = data.rowwise().maxCoeff();
            data_range_ = data_max_ - data_min_;

            assert(data_range_.rows() == scale_.rows());
            calcScaleOffsetFromDataRange();
        }

        // Updates the scalar parameters based on the new data and the old data_min_ and data_max_
        // Data is assumed to be D x N, where D is the data dimension, and N is the number of examples
        void addDataExamples(const Eigen::MatrixXd& data)
        {
            assert(data.rows() == scale_.rows());
            const Eigen::VectorXd new_data_min = data.rowwise().minCoeff();
            Eigen::MatrixX2d min(data_range_.rows(), 2);
            min << new_data_min, data_min_;
            data_min_ = min.rowwise().minCoeff();

            const Eigen::VectorXd new_data_max = data.rowwise().maxCoeff();
            Eigen::MatrixX2d max(data_range_.rows(), 2);
            max << new_data_max, data_max_;
            data_max_ = max.rowwise().maxCoeff();

            data_range_ = data_max_ - data_min_;

            assert(data_range_.rows() == scale_.rows());
            calcScaleOffsetFromDataRange();
        }

    private:
        Eigen::VectorXd scale_;
        Eigen::VectorXd offset_;

        // Used only to specify scale_ and offset_
        const double feature_min_;
        const double feature_max_;
        Eigen::VectorXd data_min_;
        Eigen::VectorXd data_max_;
        Eigen::VectorXd data_range_;

        void calcDataRangeFromScaleOffset()
        {
            data_min_ = (feature_min_ - offset_.array()) * scale_.array().inverse();
            data_max_ = (feature_max_ - feature_min_) * scale_.cwiseInverse() + data_min_;
            data_range_ = data_max_ - data_min_;

//            std::cout << "Calculating data from scale and offset:\n"
//                      << "scale     = [" << scale_.transpose() << "]';\n"
//                      << "offset    = [" << offset_.transpose() << "]';\n"
//                      << "dmin      = [" << data_min_.transpose() << "]';\n"
//                      << "dmax      = [" << data_max_.transpose() << "]';\n"
//                      << "drange    = [" << data_range_.transpose() << "]';\n"
//                      << "fmin      = " << feature_min_ << ";\n"
//                      << "fmax      = " << feature_max_ << ";\n" << std::endl;

            assert((data_range_.array() > 0.0).all());
        }

        void calcScaleOffsetFromDataRange()
        {
            scale_ = (feature_max_ - feature_min_) * data_range_.cwiseInverse();
            offset_ = feature_min_ - data_min_.cwiseProduct(scale_).array();

//            std::cout << "Calculating scale and offset from data:\n"
//                      << "scale     = [" << scale_.transpose() << "]';\n"
//                      << "offset    = [" << offset_.transpose() << "]';\n"
//                      << "dmin      = [" << data_min_.transpose() << "]';\n"
//                      << "dmax      = [" << data_max_.transpose() << "]';\n"
//                      << "drange    = [" << data_range_.transpose() << "]';\n"
//                      << "fmin      = " << feature_min_ << ";\n"
//                      << "fmax      = " << feature_max_ << ";\n" << std::endl;

            assert((scale_.array() > 0.0).all());
        }
    };
}

#endif
