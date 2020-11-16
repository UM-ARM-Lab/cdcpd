#include "smmap/task_framework.h"

std::vector<Eigen::VectorXd> getJointInfo()
{
    const Eigen::VectorXd lower_limits = Eigen::VectorXd::Constant(6, -std::numeric_limits<double>::max());
    const Eigen::VectorXd upper_limits = Eigen::VectorXd::Constant(6, std::numeric_limits<double>::max());
    const Eigen::VectorXd weights = Eigen::VectorXd::Constant(6, 1.0);
    return {lower_limits, upper_limits, weights};
}

int main(int argc, char* argv[])
{
    using namespace smmap;

    // Read in all ROS parameters
    ros::init(argc, argv, "smmap_planner_node", ros::init_options::NoSigintHandler);

    auto nh = std::make_shared<ros::NodeHandle>();
    auto ph = std::make_shared<ros::NodeHandle>("~");

    ROS_INFO("Creating utility objects");
    auto robot = std::make_shared<RobotInterface>(nh, ph);
    robot->setCallbackFunctions(
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
    auto vis = std::make_shared<Visualizer>(nh, ph, true);
    auto task_specification(TaskSpecification::MakeTaskSpecification(nh, ph, vis));

    ROS_INFO("Creating and executing planner");
    TaskFramework planner(nh, ph, robot, vis, task_specification);
    planner.execute();

    ROS_INFO("Disposing planner...");

    return EXIT_SUCCESS;
}
