#include <ros/ros.h>
#include <arc_utilities/zlib_helpers.hpp>
#include <arc_utilities/math_helpers.hpp>
#include <arc_utilities/serialization_ros.hpp>
#include <smmap_utilities/visualization_tools.h>
#include <deformable_manipulation_msgs/TestRobotPathsFeedback.h>
#include "smmap/robot_interface.h"
#include "smmap/band_rrt.h"
#include "smmap/conversions.h"
#include "smmap/trajectory.hpp"

using namespace EigenHelpers;
using namespace smmap;
namespace dmm = deformable_manipulation_msgs;

std::vector<Eigen::VectorXd> getJointInfo()
{
    const Eigen::VectorXd lower_limits = Eigen::VectorXd::Constant(6, -std::numeric_limits<double>::max());
    const Eigen::VectorXd upper_limits = Eigen::VectorXd::Constant(6, std::numeric_limits<double>::max());
    const Eigen::VectorXd weights = Eigen::VectorXd::Constant(6, 1.0);
    return {lower_limits, upper_limits, weights};
}

class Tester
{
private:
    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<ros::NodeHandle> ph_;
    Visualizer::Ptr vis_;
    RobotInterface::Ptr robot_;
    TaskSpecification::Ptr task_specification_;
    DijkstrasCoverageTask::Ptr dijkstras_task_;
    WorldState task_initial_world_state_;
    RubberBand::Ptr initial_band_;

    void initializeBand()
    {
        assert(task_initial_world_state_.all_grippers_single_pose_.size() == 2);
        // Extract the maximum distance between the grippers
        // This assumes that the starting position of the grippers is at the maximum "unstretched" distance
        const auto& grippers_starting_poses = task_initial_world_state_.all_grippers_single_pose_;
        const double max_calced_band_length =
                (grippers_starting_poses[0].translation() - grippers_starting_poses[1].translation()).norm()
                * dijkstras_task_->maxStretchFactor();
        ROS_ERROR_STREAM_COND_NAMED(!CloseEnough(max_calced_band_length, dijkstras_task_->maxBandLength(), 1e-3),
                                    "task_framework",
                                    "Calc'd max band distance is: " << max_calced_band_length <<
                                    " but the ros param saved distance is " << dijkstras_task_->maxBandLength() <<
                                    ". Double check the stored value in the roslaunch file.");

        // Find the shortest path through the object, between the grippers, while follow nodes of the object.
        // Used to determine the starting position of the rubber band at each timestep
        const auto neighbour_fn = [&] (const ssize_t& node)
        {
            return dijkstras_task_->getNodeNeighbours(node);
        };

        // Create the initial rubber band
        const double resampled_band_max_pointwise_dist = dijkstras_task_->work_space_grid_.minStepDimension() / 2.0;
        const size_t upsampled_band_num_points = GetRRTBandMaxPoints(*ph_);

        initial_band_ = std::make_shared<RubberBand>(
                    nh_,
                    ph_,
                    vis_,
                    dijkstras_task_->sdf_,
                    dijkstras_task_->work_space_grid_,
                    neighbour_fn,
                    task_initial_world_state_,
                    resampled_band_max_pointwise_dist,
                    upsampled_band_num_points,
                    dijkstras_task_->maxBandLength());
    }

public:
    Tester(std::shared_ptr<ros::NodeHandle> nh,
           std::shared_ptr<ros::NodeHandle> ph)
        : nh_(nh)
        , ph_(ph)
        , vis_(std::make_shared<Visualizer>(nh_, ph_, true))
        , robot_(std::make_shared<RobotInterface>(nh_, ph_))
        , task_specification_(TaskSpecification::MakeTaskSpecification(nh_, ph_, vis_))
        , dijkstras_task_(std::dynamic_pointer_cast<DijkstrasCoverageTask>(task_specification_)) // If possible, this will be done, if not, it will be NULL (nullptr?)
    {
        robot_->setCallbackFunctions(
                    nullptr,
                    nullptr,
                    nullptr,
                    &getJointInfo,
                    nullptr,
                    nullptr,
                    nullptr,
                    nullptr,
                    nullptr,
                    nullptr,
                    nullptr,
                    nullptr,
                    nullptr);
        task_initial_world_state_ = robot_->start();
        initializeBand();
    }

    bool test(std::string const& world_state_file,
              std::string const& rrt_path_file,
              std::string const& test_result_file) const
    {
        auto const world_state = WorldState::Deserialize(ZlibHelpers::LoadFromFileAndDecompress(world_state_file), 0).first;
        auto const rrt_path = BandRRT::LoadPath(rrt_path_file, *initial_band_);
        auto const test_result = arc_utilities::RosMessageDeserializationWrapper<dmm::TestRobotPathsFeedback>(
                    ZlibHelpers::LoadFromFileAndDecompress(test_result_file), 0).first.test_result;
        auto const test_waypoint_indices = [&] ()
        {
            // It is assumed that the robot starts where the path is at idx 0, so trim that element from the planned path
            RRTPath const commanded_path(rrt_path.begin() + 1, rrt_path.end());
            assert(commanded_path.size() > 0 && "If this is false, it probably means that plan_start == plan_goal");
            auto const robot_path = RRTPathToGrippersPoseTrajectory(commanded_path);
            auto const interp_result = robot_->interpolateGrippersTrajectory(robot_path);
            return interp_result.second;
        }();

        auto const traj_gen_result = ToTrajectory(world_state, rrt_path, test_result, test_waypoint_indices);
        return traj_gen_result.second;
    }

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "to_trajectory_tester");
    auto nh = std::make_shared<ros::NodeHandle>();
    auto ph = std::make_shared<ros::NodeHandle>("~");
    auto const tester = Tester(nh, ph);
    auto const world_state_file = ROSHelpers::GetParam<std::string>(*ph, "world_state_file", "/tmp/to_trajectory_testing/world_state.compressed");
    auto const rrt_path_file = ROSHelpers::GetParam<std::string>(*ph, "rrt_path_file", "/tmp/to_trajectory_testing/rrt_path.compressed");
    auto const test_result_file = ROSHelpers::GetParam<std::string>(*ph, "test_result_file", "/tmp/to_trajectory_testing/test_result_file.compressed");
    auto const path_complete = tester.test(world_state_file, rrt_path_file, test_result_file);
    ROS_INFO_STREAM("Data indicates that the path was completed successfully: " << path_complete);
    return EXIT_SUCCESS;
}
