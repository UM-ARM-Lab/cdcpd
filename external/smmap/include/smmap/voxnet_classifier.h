#ifndef VOXNET_CLASSIFIER_H
#define VOXNET_CLASSIFIER_H

#include <ros/ros.h>
#include <sdf_tools/collision_map.hpp>
#include <torch/script.h>
#include "smmap/quinlan_rubber_band.h"

namespace smmap
{
    class VoxnetClassifier
    {
    protected:
        std::shared_ptr<ros::NodeHandle> const nh_;
        std::shared_ptr<ros::NodeHandle> const ph_;
        sdf_tools::SignedDistanceField::ConstPtr const sdf_;

    public:
        std::string const name_;
        double const accuracy_;
    private:
        torch::jit::script::Module model_;
        double const threshold_;
        int64_t const n_cells_;

    public:
        struct Features
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            sdf_tools::CollisionMapGrid local_environment;
            sdf_tools::CollisionMapGrid pre_band;
            sdf_tools::CollisionMapGrid post_band;
        };

        VoxnetClassifier(std::shared_ptr<ros::NodeHandle> nh,
                         std::shared_ptr<ros::NodeHandle> ph,
                         sdf_tools::SignedDistanceField::ConstPtr sdf);

        Features features(const RubberBand& initial_band,
                          const RubberBand& default_prediction) const;

        double predict(const RubberBand& initial_band,
                       const RubberBand& default_prediction) const;
    };
}

#endif // VOXNET_CLASSIFIER_H
