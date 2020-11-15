#include <torch/script.h>
#include <ros/ros.h>
#include <sdf_tools/collision_map.hpp>

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
    auto frame = "anything";
    auto local_env_resolution = 0.025;
    auto n_cells = 32l;

    sdf_tools::CollisionMapGrid local_environment(origin,
                                                  frame,
                                                  local_env_resolution,
                                                  n_cells,
                                                  n_cells,
                                                  n_cells,
                                                  sdf_tools::COLLISION_CELL(0.0),
                                                  sdf_tools::COLLISION_CELL(0.0));
    sdf_tools::CollisionMapGrid band_pre(origin,
                                         frame,
                                         local_env_resolution,
                                         n_cells,
                                         n_cells,
                                         n_cells,
                                         sdf_tools::COLLISION_CELL(0.0),
                                         sdf_tools::COLLISION_CELL(0.0));
    sdf_tools::CollisionMapGrid band_post(origin,
                                          frame,
                                          local_env_resolution,
                                          n_cells,
                                          n_cells,
                                          n_cells,
                                          sdf_tools::COLLISION_CELL(0.0),
                                          sdf_tools::COLLISION_CELL(0.0));

    torch::jit::script::Module model = torch::jit::load(
                "/home/dmcconac/Dropbox/catkin_ws/src/smmap/logs/rope_hooks_simple/rope_voxnet_multichannel_b64_cuda.pt");

    auto torch_tensor = torch::empty({1, 3, n_cells, n_cells, n_cells});
    // at::from_blob(data, sizes, options);

    // TODO: fill out this tensor directly rather than first filling CollisionMapGrids?
    auto accessor = torch_tensor.accessor<float, 5>();
    for (int64_t x_idx = 0; x_idx < n_cells; ++x_idx)
    {
        for (int64_t y_idx = 0; y_idx < n_cells; ++y_idx)
        {
            for (int64_t z_idx = 0; z_idx < n_cells; ++z_idx)
            {
                accessor[0][0][x_idx][y_idx][z_idx]
                        = local_environment.GetImmutable(x_idx, y_idx, z_idx).first.occupancy;
                accessor[0][1][x_idx][y_idx][z_idx]
                        = band_pre.GetImmutable(x_idx, y_idx, z_idx).first.occupancy;
                accessor[0][2][x_idx][y_idx][z_idx]
                        = band_post.GetImmutable(x_idx, y_idx, z_idx).first.occupancy;
            }
        }
    }

    std::vector<torch::jit::IValue> const query(1, torch_tensor.to(torch::kCUDA));
    float const output = model.forward(query).toTensor().item().toFloat();
    auto const mistake = (output > 0.5) ? 1.0 : -1.0;

    std::cout << "output: " << output << "    mistake: " << mistake << std::endl;
    return EXIT_SUCCESS;
}
