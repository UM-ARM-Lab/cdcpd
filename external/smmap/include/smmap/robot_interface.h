#ifndef ROBOT_INTERFACE_HPP
#define ROBOT_INTERFACE_HPP

#include <thread>
#include <functional>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <deformable_manipulation_msgs/messages.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <smmap_utilities/grippers.h>

#include <smmap_utilities/task_function_pointer_types.h>

namespace smmap
{
    class RobotInterface
    {
    public:
        typedef std::shared_ptr<RobotInterface> Ptr;
        typedef std::shared_ptr<const RobotInterface> ConstPtr;

        RobotInterface(
                std::shared_ptr<ros::NodeHandle> nh,
                std::shared_ptr<ros::NodeHandle> ph);
        ~RobotInterface();

        WorldState start();

        bool ok() const;
        void shutdown();
        void reset();

        const std::vector<GripperData>& getGrippersData() const;

        AllGrippersSinglePose getGrippersPoses();

        // This function assumes only 2 grippers, and it is called before the grippers are moved by sendGrippersPoses
        double getGrippersInitialDistance();

        std::pair<WorldState, std::vector<WorldState>> commandRobotMotion(
                const AllGrippersSinglePose& target_grippers_poses,
                const Eigen::VectorXd& target_robot_configuration,
                const bool robot_configuration_valid);

        bool testRobotMotion(
                const std::vector<AllGrippersSinglePose>& test_grippers_poses,
                const std::vector<Eigen::VectorXd>& test_robot_configurations,
                const bool robot_configuration_valid,
                const TestRobotMotionFeedbackCallback& feedback_callback);

        bool generateTransitionData(
                const std::vector<deformable_manipulation_msgs::TransitionTest>& tests,
                const std::vector<std::string>& filenames,
                const GenerateTransitionDataFeedbackCallback& feedback_callback,
                const bool wait_for_feedback);

        bool testRobotPaths(
                const std::vector<AllGrippersPoseTrajectory>& test_paths,
                const std::vector<std::string>& filenames,
                const TestRobotPathsFeedbackCallback& feedback_callback,
                const bool return_microsteps,
                const bool wait_for_feedback);

        std::pair<WorldState, std::vector<WorldState>> testRobotMotionMicrosteps(
                const EigenHelpers::VectorIsometry3d& starting_rope_configuration,
                const AllGrippersSinglePose& starting_grippers_poses,
                const AllGrippersSinglePose& target_grippers_poses,
                const int num_substeps);

        std::vector<CollisionData> checkGripperCollision(
                const AllGrippersSinglePose& grippers_pose);

        void resetRandomSeeds(const unsigned long seed, const unsigned long num_discards);
        void lockEnvironment() const;
        void unlockEnvironment() const;

        const Eigen::VectorXd& getJointLowerLimits() const;
        const Eigen::VectorXd& getJointUpperLimits() const;
        const Eigen::VectorXd& getJointWeights() const;

        void setActiveDOFValues(const Eigen::VectorXd& robot_configuration) const;

        // TODO: Change this name to something else, it is confusing with getGrippersPoses()
        AllGrippersSinglePose getGrippersPosesFunctionPointer() const;

        // This a Jacobian between the movement of the grippers (in the gripper body frame)
        // and the movement of the robot's DOF
        Eigen::MatrixXd getGrippersJacobian() const;

        // This looks up the points of interest as reporeted by the external robot (i.e. OpenRAVE)
        // then querrys Bullet for the data needed to do collision avoidance, and querrys OpenRAVE
        // for the Jacobian of the movement of the point relative to the robot DOF movement.
        //
        // This includes the grippers.
        std::vector<std::pair<CollisionData, Eigen::Matrix3Xd>> getPointsOfInterestCollisionData();


        Eigen::VectorXd mapGripperMotionToRobotMotion(
                const AllGrippersSinglePoseDelta& grippers_delta) const;

        bool checkRobotCollision() const;

        std::vector<Eigen::VectorXd> getCloseIkSolutions(
                const AllGrippersSinglePose& target_poses,
                const double max_gripper_distance) const;

        std::vector<Eigen::VectorXd> getCloseIkSolutions(
                const std::vector<std::string>& gripper_names,
                const AllGrippersSinglePose& target_poses,
                const double max_gripper_distance) const;

        std::pair<bool, Eigen::VectorXd> getGeneralIkSolution(
                const Eigen::VectorXd& starting_config,
                const AllGrippersSinglePose& target_poses) const;

        std::pair<bool, Eigen::VectorXd> getGeneralIkSolution(
                const Eigen::VectorXd& starting_config,
                const std::vector<std::string>& gripper_names,
                const AllGrippersSinglePose& target_poses) const;

        bool testPathForCollision(const std::vector<Eigen::VectorXd>& path) const;

        void setCallbackFunctions(
                const std::function<void(const size_t, const size_t)>& reset_random_seeds_fn,
                const std::function<void()>& lock_env_fn,
                const std::function<void()>& unlock_env_fn,
                const std::function<std::vector<Eigen::VectorXd>            ()>& get_robot_joint_info_fn,
                const std::function<void                                    (const Eigen::VectorXd& configuration)> set_active_dof_values_fn,
                const std::function<AllGrippersSinglePose  ()>& get_ee_poses_fn,
                const std::function<Eigen::MatrixXd                         ()>& get_grippers_jacobian_fn,
                const std::function<std::vector<Eigen::Vector3d>            ()>& get_collision_points_of_interest_fn,
                const std::function<std::vector<Eigen::MatrixXd>            ()>& get_collision_points_of_interest_jacobians_fn,
                const std::function<bool                                    ()>& full_robot_collision_check_fn,
                const std::function<std::vector<Eigen::VectorXd>            (const std::vector<std::string>& gripper_names, const AllGrippersSinglePose& target_poses, const double max_gripper_distance)>& close_ik_solutions_fn,
                const std::function<std::pair<bool, Eigen::VectorXd>        (const Eigen::VectorXd& starting_config, const std::vector<std::string>& gripper_names, const AllGrippersSinglePose& target_poses)>& general_ik_solution_fn,
                const std::function<bool                                    (const std::vector<Eigen::VectorXd>& path)>& test_path_for_collision_fn);

        // Defaults the timespace to "latest available", indicted by ros::Time(0)
        Eigen::Vector3d transformToFrame(
                const Eigen::Vector3d& point,
                const std::string& source_frame,
                const std::string& target_frame,
                const ros::Time& time = ros::Time(0)) const;

        const Eigen::Isometry3d& getWorldToTaskFrameTf() const;

        // Returns the pose vector in the direction of target, with the the maximum change allowed from start
        // Second element is true if target was reached, false otherwise
        std::pair<AllGrippersSinglePose, bool> clampGrippersMovement(
                const AllGrippersSinglePose& start,
                const AllGrippersSinglePose& target) const;

        // Returns the pose vector in the direction of target, with the the maximum change allowed from start
        // Second element is true if target was reached, false otherwise
        std::pair<Eigen::VectorXd, bool> clampFullRobotMovement(
                const Eigen::VectorXd& start,
                const Eigen::VectorXd& target) const;

        // Returns the interpolated trajectory (as defined by repeated applications of clampGrippersMovement)
        // and the indices that correspond to the original waypoints
        std::pair<AllGrippersPoseTrajectory, std::vector<size_t>> interpolateGrippersTrajectory(
                const AllGrippersPoseTrajectory& waypoints) const;

        // Returns the interpolated trajectory (as defined by repeated applications of clampFullRobotMovement)
        // and the indices that correspond to the original waypoints
        std::pair<std::vector<Eigen::VectorXd>, std::vector<size_t>> interpolateFullRobotTrajectory(
                const std::vector<Eigen::VectorXd>& waypoints) const;

    private:
        ////////////////////////////////////////////////////////////////////
        // ROS objects and helpers
        ////////////////////////////////////////////////////////////////////

        const std::shared_ptr<ros::NodeHandle> nh_;
        const std::shared_ptr<ros::NodeHandle> ph_;

    public:
        const std::string bullet_frame_name_;
        const std::string world_frame_name_; // Frame that all incomming data must be in
    private:
        tf2_ros::Buffer tf_buffer_;
        const tf2_ros::TransformListener tf_listener_;
        Eigen::Isometry3d world_to_bullet_tf_;

        const std::vector<GripperData> grippers_data_;
        GripperCollisionChecker gripper_collision_checker_;
        ros::ServiceClient execute_gripper_movement_client_;
        actionlib::SimpleActionClient<deformable_manipulation_msgs::TestRobotMotionAction> test_grippers_poses_client_;
        actionlib::SimpleActionClient<deformable_manipulation_msgs::GenerateTransitionDataAction> generate_transition_data_client_;
        actionlib::SimpleActionClient<deformable_manipulation_msgs::TestRobotPathsAction> test_grippers_paths_client_;

    // TODO: comments, and placement, and stuff
    public:
        const double dt_;
        const double max_gripper_velocity_norm_;
        const double max_dof_velocity_norm_;
        const double min_controller_distance_to_obstacles_;

    private:
        Eigen::VectorXd joint_lower_limits_;
        Eigen::VectorXd joint_upper_limits_;
        Eigen::VectorXd joint_weights_;

    private:
        std::thread spin_thread_;

        // Function pointers that allow for generic(ish) external robots, without explicit inheritance
        std::function<void(const unsigned long, const unsigned long)> reset_random_seeds_fn_;
        std::function<void()> lock_env_fn_;
        std::function<void()> unlock_env_fn_;

        std::function<void                                      (const Eigen::VectorXd& configuration)> set_active_dof_values_fn_;
        std::function<AllGrippersSinglePose    ()> get_ee_poses_fn_;
        std::function<Eigen::MatrixXd                           ()> get_grippers_jacobian_fn_;
        std::function<std::vector<Eigen::Vector3d>              ()> get_collision_points_of_interest_fn_;
        std::function<std::vector<Eigen::MatrixXd>              ()> get_collision_points_of_interest_jacobians_fn_;
        std::function<bool                                      ()> full_robot_collision_check_fn_;
        std::function<std::vector<Eigen::VectorXd>              (const std::vector<std::string>& gripper_names, const AllGrippersSinglePose& target_poses, const double max_gripper_distance)> close_ik_solutions_fn_;
        std::function<std::pair<bool, Eigen::VectorXd>          (const Eigen::VectorXd& starting_config, const std::vector<std::string>& gripper_names, const AllGrippersSinglePose& target_poses)> general_ik_solution_fn_;
        std::function<bool                                      (const std::vector<Eigen::VectorXd>& path)> test_path_for_collision_fn_;

        std::pair<WorldState, std::vector<WorldState>> commandRobotMotion_impl(
                const deformable_manipulation_msgs::ExecuteRobotMotionRequest& movement);

        deformable_manipulation_msgs::ExecuteRobotMotionRequest noOpGripperMovement();
        deformable_manipulation_msgs::ExecuteRobotMotionRequest toRosMovementRequest(
                const AllGrippersSinglePose& grippers_poses,
                const Eigen::VectorXd& robot_configuration,
                const bool robot_configuration_valid) const;

        ////////////////////////////////////////////////////////////////////
        // Testing specific gripper movements
        ////////////////////////////////////////////////////////////////////

        // used by both action servers
        volatile size_t feedback_counter_;
        std::vector<bool> feedback_recieved_;

        void internalTestPoseFeedbackCallback(
                const deformable_manipulation_msgs::TestRobotMotionActionFeedbackConstPtr& feedback,
                const TestRobotMotionFeedbackCallback& feedback_callback);

        bool testRobotMotion_impl(
                const deformable_manipulation_msgs::TestRobotMotionGoal& goal,
                const TestRobotMotionFeedbackCallback& feedback_callback);

        deformable_manipulation_msgs::TestRobotMotionGoal toRosTestPosesGoal(
                const std::vector<AllGrippersSinglePose>& grippers_poses,
                const std::vector<Eigen::VectorXd>& robot_configurations,
                const bool robot_configurations_valid) const;

        ////////////////////////////////////////////////////////////////////
        // Transition testing framework
        ////////////////////////////////////////////////////////////////////

        void internalGenerateTransitionDataFeedbackCallback(
                const deformable_manipulation_msgs::GenerateTransitionDataActionFeedbackConstPtr& feedback,
                const GenerateTransitionDataFeedbackCallback& feedback_callback);

        bool generateTransitionData_impl(
                const deformable_manipulation_msgs::GenerateTransitionDataGoal& goal,
                const GenerateTransitionDataFeedbackCallback& feedback_callback,
                const bool wait_for_feedback);

    public:
        deformable_manipulation_msgs::TransitionTest toRosTransitionTest(
                const EigenHelpers::VectorIsometry3d& starting_rope_node_transforms,
                const AllGrippersSinglePose& starting_gripper_poses,
                const AllGrippersPoseTrajectory& path_to_start_of_test,
                const AllGrippersSinglePose& final_gripper_targets) const;

    private:

        ////////////////////////////////////////////////////////////////////
        // Path testing framework
        ////////////////////////////////////////////////////////////////////

        void internalTestRobotPathsFeedbackCallback(
                const deformable_manipulation_msgs::TestRobotPathsActionFeedbackConstPtr& feedback,
                const TestRobotPathsFeedbackCallback& feedback_callback);

        bool testRobotPaths_impl(
                const deformable_manipulation_msgs::TestRobotPathsGoal& goal,
                const TestRobotPathsFeedbackCallback& feedback_callback,
                const bool wait_for_feedback);

        deformable_manipulation_msgs::RobotPathTest toRosTestRobotPath(
                const AllGrippersPoseTrajectory& grippers_pose_traj,
                const bool return_microsteps) const;

        ////////////////////////////////////////////////////////////////////
        // Single motion framework
        ////////////////////////////////////////////////////////////////////

        std::pair<WorldState, std::vector<WorldState>> testRobotMotionMicrosteps_impl(
                const deformable_manipulation_msgs::TestRobotMotionMicrostepsRequest& request);

        deformable_manipulation_msgs::TestRobotMotionMicrostepsRequest toRosMicrostepsRequest(
                const EigenHelpers::VectorIsometry3d& starting_rope_configuration,
                const AllGrippersSinglePose& starting_grippers_poses,
                const AllGrippersSinglePose& target_grippers_poses,
                const int num_substeps) const;
    };
}

#endif // ROBOT_INTERFACE_HPP
