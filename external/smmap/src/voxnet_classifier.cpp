#include "smmap/voxnet_classifier.h"
#include <arc_utilities/ros_helpers.hpp>

using namespace Eigen;

namespace smmap
{
    inline static std::string getModelFilename(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<std::string>(nh, "voxnet/model_file", __func__);
    }

    VoxnetClassifier::VoxnetClassifier(std::shared_ptr<ros::NodeHandle> nh,
                                       std::shared_ptr<ros::NodeHandle> ph,
                                       sdf_tools::SignedDistanceField::ConstPtr sdf)
        : nh_(nh)
        , ph_(ph)
        , sdf_(sdf)
        , name_("voxnet")
        , accuracy_(ROSHelpers::GetParamRequired<double>(*ph_, "voxnet/accuracy", __func__))
        , model_(torch::jit::load(getModelFilename(*ph_)))
        , threshold_(ROSHelpers::GetParamRequired<double>(*ph_, "voxnet/threshold", __func__))
        , n_cells_(32)
    {}


    VoxnetClassifier::Features VoxnetClassifier::features(
            const RubberBand& initial_band,
            const RubberBand& default_prediction) const
    {
        const auto max_band_length = initial_band.maxSafeLength();
        const auto local_env_resolution = max_band_length / static_cast<double>(n_cells_);

        const auto starting_gripper_positions = initial_band.getEndpoints();
        const auto ending_gripper_positions = default_prediction.getEndpoints();
        Vector3d trans = (starting_gripper_positions.first +
                          starting_gripper_positions.second +
                          ending_gripper_positions.first +
                          ending_gripper_positions.second) / 4;
        trans -= Vector3d(max_band_length, max_band_length, max_band_length) / 2;
        const Isometry3d origin = Isometry3d::Identity() * Translation3d(trans);

        // The local environment is centered at the centroid of the pre/post left/right gripper positions
        // and has a fixed size that captures all possible bands by being as large as possible.
        sdf_tools::CollisionMapGrid local_environment(origin,
                                                      sdf_->GetFrame(),
                                                      local_env_resolution,
                                                      n_cells_,
                                                      n_cells_,
                                                      n_cells_,
                                                      sdf_tools::COLLISION_CELL(0.0),
                                                      sdf_tools::COLLISION_CELL(0.0));
        sdf_tools::CollisionMapGrid band_pre(origin,
                                             sdf_->GetFrame(),
                                             local_env_resolution,
                                             n_cells_,
                                             n_cells_,
                                             n_cells_,
                                             sdf_tools::COLLISION_CELL(0.0),
                                             sdf_tools::COLLISION_CELL(0.0));
        sdf_tools::CollisionMapGrid band_post(origin,
                                              sdf_->GetFrame(),
                                              local_env_resolution,
                                              n_cells_,
                                              n_cells_,
                                              n_cells_,
                                              sdf_tools::COLLISION_CELL(0.0),
                                              sdf_tools::COLLISION_CELL(0.0));

        // iterate through the cells in the local environment occupancy grid, and for each cell:
        // compute the world location of the cell, look-up that point in the SDF, check if it's occupied
        // and set the value in the local environment occupancy grid.
        for (int64_t x_idx = 0; x_idx < n_cells_; ++x_idx)
        {
            for (int64_t y_idx = 0; y_idx < n_cells_; ++y_idx)
            {
                for (int64_t z_idx = 0; z_idx < n_cells_; ++z_idx)
                {
                    const auto location = local_environment.GridIndexToLocation(x_idx, y_idx, z_idx);
                    const auto sdf_val = sdf_->GetImmutable4d(location);
                    const auto sdf_occupancy = sdf_val.first > 0.f ? 0.f : 1.f;
                    local_environment.SetValue(x_idx, y_idx, z_idx, sdf_tools::COLLISION_CELL(sdf_occupancy));
                }
            }
        }

        for (auto const& point : initial_band.upsampleBand())
        {
            band_pre.SetValue(point(0), point(1), point(2), sdf_tools::COLLISION_CELL(1.0));
        }

        for (auto const& point : default_prediction.upsampleBand())
        {
            band_post.SetValue(point(0), point(1), point(2), sdf_tools::COLLISION_CELL(1.0));
        }

        Features voxnet_features;
        voxnet_features.local_environment = local_environment;
        voxnet_features.post_band = band_post;
        voxnet_features.pre_band = band_pre;
        return voxnet_features;
    }

    double VoxnetClassifier::predict(
            const RubberBand& initial_band,
            const RubberBand& default_prediction) const
    {
        const auto input = features(initial_band, default_prediction);

//        const auto options =
//                torch::TensorOptions()
//                    .dtype(torch::kFloat32)
//                    .layout(torch::kStrided)
//                    .device(torch::kCPU)
//                    .requires_grad(false);

        auto torch_tensor = torch::empty({1, 3, n_cells_, n_cells_, n_cells_});
        // at::from_blob(data, sizes, options);

        // TODO: fill out this tensor directly rather than first filling CollisionMapGrids?
        auto accessor = torch_tensor.accessor<float, 5>();
        for (int64_t x_idx = 0; x_idx < n_cells_; ++x_idx)
        {
            for (int64_t y_idx = 0; y_idx < n_cells_; ++y_idx)
            {
                for (int64_t z_idx = 0; z_idx < n_cells_; ++z_idx)
                {
                    accessor[0][0][x_idx][y_idx][z_idx]
                            = input.local_environment.GetImmutable(x_idx, y_idx, z_idx).first.occupancy;
                    accessor[0][1][x_idx][y_idx][z_idx]
                            = input.pre_band.GetImmutable(x_idx, y_idx, z_idx).first.occupancy;
                    accessor[0][2][x_idx][y_idx][z_idx]
                            = input.post_band.GetImmutable(x_idx, y_idx, z_idx).first.occupancy;
                }
            }
        }

        std::vector<torch::jit::IValue> const query(1, torch_tensor.to(torch::kCUDA));
        // Our particular forward is threadsafe per
        // https://github.com/pytorch/pytorch/issues/23920#issuecomment-519355570
        auto const output = const_cast<VoxnetClassifier*>(this)->model_.forward(query).toTensor().item().toFloat();
        auto const mistake = (output > threshold_) ? 1.0 : -1.0;
        return mistake;
    }
}
