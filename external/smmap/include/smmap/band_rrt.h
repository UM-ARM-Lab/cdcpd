#pragma once

#include <random>
#include <arc_utilities/eigen_typedefs.hpp>
#include <smmap_utilities/visualization_tools.h>

#include <flann/flann.hpp>

#include "smmap/transition_estimation.h"
#include "smmap/quinlan_rubber_band.h"
#include "smmap/robot_interface.h"

namespace flann
{
    /**
     * Squared Euclidean distance functor, optimized version
     */
    template<class T>
    struct L2_weighted
    {
        typedef bool is_kdtree_distance;

        typedef T ElementType;
        typedef T ResultType;
        typedef Eigen::Matrix<ElementType, Eigen::Dynamic, 1> VectorX;

        L2_weighted(const Eigen::VectorXd& dof_weights)
        {
            dof_weights_.resizeLike(dof_weights);
            dof_weights2_.resizeLike(dof_weights);
            for (int i = 0; i < dof_weights.rows(); ++i)
            {
                dof_weights_(i) = (ElementType)dof_weights(i);
                dof_weights2_(i) = (ElementType)(dof_weights(i) * dof_weights(i));
            }
        }

        /**
         *  Compute the squared Euclidean distance between two vectors.
         *
         *	The computation of squared root at the end is omitted for
         *	efficiency.
         */
        template <typename Iterator1, typename Iterator2>
        ResultType operator()(Iterator1 a, Iterator2 b, size_t size, ResultType worst_dist = -1) const
        {
            (void)worst_dist;
            const Eigen::Map<const VectorX> a_vec(a, size);
            const Eigen::Map<const VectorX> b_vec(b, size);
            auto delta = (a_vec - b_vec).cwiseProduct(dof_weights_);
            return delta.squaredNorm();
        }

        /**
         *	Partial euclidean distance, using just one dimension. This is used by the
         *	kd-tree when computing partial distances while traversing the tree.
         *
         *	Squared root is omitted for efficiency.
         */
        template <typename U, typename V>
        inline ResultType accum_dist(const U& a, const V& b, int ind) const
        {
            return (a-b) * (a-b) * dof_weights2_(ind);
        }

    private:
        VectorX dof_weights_;
        VectorX dof_weights2_;
    };
}

namespace smmap
{
    class RRTNode;
    typedef Eigen::aligned_allocator<RRTNode> RRTAllocator;
    // An RRT path is a single unbroken chain of nodes, with no splits, with each node having a single child
    typedef std::vector<RRTNode, RRTAllocator> RRTPath;
    // An RRTTree can be any arbitrary set of nodes, so long as each node has a single parent.
    // Notably a node can have multiple children via splits or otherwise
    typedef std::vector<RRTNode, RRTAllocator> RRTTree;
    typedef std::vector<std::pair<RRTPath, std::vector<size_t>>> RRTPolicy;

    AllGrippersPoseTrajectory RRTPathToGrippersPoseTrajectory(const RRTPath& path);

    typedef PairGripperPoses RRTGrippersRepresentation;
    typedef Eigen::VectorXd RRTRobotRepresentation;
    typedef flann::KDTreeSingleIndex<flann::L2_weighted<float>> NNIndexType;

    class RRTNode
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        RRTNode();

        RRTNode(const RRTGrippersRepresentation& grippers_poses,
                const RRTRobotRepresentation& robot_configuration,
                const RubberBand::Ptr& band);

        RRTNode(const RRTGrippersRepresentation& grippers_poses,
                const RRTRobotRepresentation& robot_configuration,
                const RubberBand::Ptr& band,
                const double cost_to_come,
                const double p_reachability,
                const double p_transition,
                const int64_t parent_index,
                const int64_t state_index,
                const int64_t transition_index,
                const int64_t split_index);

        RRTNode(const RRTGrippersRepresentation& grippers_poses,
                const RRTRobotRepresentation& robot_configuration,
                const RubberBand::Ptr& band,
                const double cost_to_come,
                const double p_reachability,
                const double p_transition,
                const double p_goal_reachable,
                const int64_t parent_index,
                const int64_t state_index,
                const int64_t transition_index,
                const int64_t split_index,
                const std::vector<int64_t>& child_indices,
                const bool initialized,
                const bool already_extended_towards_backwards_tree,
                const bool blacklisted_from_nn_search);

        bool initialized() const;

        const RRTGrippersRepresentation& grippers() const;
        const RRTRobotRepresentation& robotConfiguration() const;
        const RubberBand::Ptr& band() const;
        double costToCome() const;

        double pReachability() const;
        double pTransition() const;

        double getpGoalReachable() const;
        void setpGoalReachable(const double p_goal_reachable);

        int64_t parentIndex() const;
        int64_t stateIndex() const;
        int64_t transitionIndex() const;
        int64_t splitIndex() const;

        const std::vector<int64_t>& childIndices() const;
        void clearChildIndices();
        void addChildIndex(const int64_t child_index);
        void removeChildIndex(const int64_t child_index);

        std::string print() const;

        bool operator==(const RRTNode& other) const;

        uint64_t serialize(std::vector<uint8_t>& buffer) const;
        static uint64_t Serialize(const RRTNode& config, std::vector<uint8_t>& buffer);
        static std::pair<RRTNode, uint64_t> Deserialize(const std::vector<uint8_t>& buffer, const uint64_t current, const RubberBand& starting_band);

    public:
        // Made public for convinience

        // Note that when planning in gripper pose space, the robot_configuration_ is exactly the position of both grippers
        RRTGrippersRepresentation grippers_poses_;
        RRTRobotRepresentation robot_configuration_;
        RubberBand::Ptr band_;
        double cost_to_come_;

        // Probabilities of various types
        double p_reachability_;     // Probability of reaching this node from the root of the tree
        double p_transition_;       // Probability of reaching this node from the parent
        double p_goal_reachable_;   // Probability of reaching the goal, assuming we are at this node

        // Book keeping
        int64_t parent_index_;
        int64_t state_index_;           // Monotonically strictly increasing
        int64_t transition_index_;      // Monotonically increasing, and <= state_index_
        int64_t split_index_;           // Is negative if this node is not the immediate result of a split

        std::vector<int64_t> child_indices_;

        bool initialized_;

    public:
        bool already_extended_towards_goal_set_;
        bool blacklisted_from_nn_search_;
    };

    std::ostream& operator<<(std::ostream& out, const RRTNode& node);

    class RRTDistance
    {
        public:
            static const RRTRobotRepresentation& GetJointWeights();
            static void SetJointWeights(const RRTRobotRepresentation joint_weights);

            static double DistanceSquared(const RRTGrippersRepresentation& c1, const RRTGrippersRepresentation& c2);
            static double Distance(const RRTGrippersRepresentation& c1, const RRTGrippersRepresentation& c2);
            static double DistanceSquared(const RRTRobotRepresentation& r1, const RRTRobotRepresentation& r2);
            static double Distance(const RRTRobotRepresentation& r1, const RRTRobotRepresentation& r2);

            static double GrippersPathDistance(const RRTTree& path, const size_t start_index, const size_t end_index);
            static double RobotPathDistance(const RRTTree& path, const size_t start_index, const size_t end_index);

        private:
            static RRTRobotRepresentation joint_weights_;
    };

    class BandRRT
    {
    public:
        typedef std::shared_ptr<BandRRT> Ptr;

        static constexpr double NN_BLACKLIST_DISTANCE = std::numeric_limits<double>::max() - 1e10;
        static constexpr double GRIPPER_TRANSLATION_IS_APPROX_DIST = 0.001;

        // Topic names used for publishing visualization data
        static constexpr char RRT_BLACKLISTED_GOAL_BANDS_NS[]   = "rrt_blacklisted_goal_bands";
        static constexpr char RRT_GOAL_TESTING_NS[]             = "rrt_goal_testing";

        static constexpr char RRT_FORWARD_TREE_GRIPPER_A_NS[]   = "rrt_forward_tree_gripper_a";
        static constexpr char RRT_FORWARD_TREE_GRIPPER_B_NS[]   = "rrt_forward_tree_gripper_b";
        static constexpr char RRT_BACKWARD_TREE_GRIPPER_A_NS[]  = "rrt_backward_tree_gripper_a";
        static constexpr char RRT_BACKWARD_TREE_GRIPPER_B_NS[]  = "rrt_backward_tree_gripper_b";
        static constexpr char RRT_TREE_BAND_NS[]                = "rrt_tree_band";

        static constexpr char RRT_SAMPLE_NS[]                   = "rrt_sample";
        static constexpr char RRT_FORWARD_PROP_START_NS[]       = "rrt_forward_prop_start";

        static constexpr char RRT_PATH_GRIPPER_A_NS[]           = "rrt_path_gripper_a";
        static constexpr char RRT_PATH_GRIPPER_B_NS[]           = "rrt_path_gripper_b";
        static constexpr char RRT_PATH_RUBBER_BAND_NS[]         = "rrt_path_rubber_band";

        static constexpr char RRT_SMOOTHING_GRIPPER_A_NS[]      = "rrt_smoothing_gripper_a";
        static constexpr char RRT_SMOOTHING_GRIPPER_B_NS[]      = "rrt_smoothing_gripper_b";

        struct WorldParams
        {
        public:
            RobotInterface::ConstPtr robot_;
            bool planning_for_whole_robot_;
            sdf_tools::SignedDistanceField::ConstPtr sdf_;
            XYZGrid work_space_grid_;
            TransitionEstimation::Ptr transition_estimator_;
            std::shared_ptr<std::mt19937_64> generator_;
        };

        struct PlanningParams
        {
        public:
            size_t forward_tree_extend_iterations_;
            size_t backward_tree_extend_iterations_;
            bool use_brute_force_nn_;
            size_t kd_tree_grow_threshold_;
            double best_near_radius2_;
            double goal_bias_;
            double feasibility_distancescale_factor_;
        };

        struct SmoothingParams
        {
        public:
            int64_t max_shortcut_index_distance_;
            uint32_t max_smoothing_iterations_;
            double smoothing_band_dist_threshold_;
        };

        struct TaskParams
        {
        public:
            Eigen::Isometry3d task_aligned_frame_transform_;
            Eigen::Vector3d task_aligned_lower_limits_;
            Eigen::Vector3d task_aligned_upper_limits_;

            double max_gripper_step_size_;
            double max_robot_dof_step_size_;
            double min_robot_dof_step_size_;
            double max_gripper_rotation_;
            double goal_reach_radius_;
            double gripper_min_distance_to_obstacles_;

            double band_distance2_scaling_factor_;
            size_t band_max_points_;
        };

        BandRRT(std::shared_ptr<ros::NodeHandle> nh,
                std::shared_ptr<ros::NodeHandle> ph,
                const WorldParams& world_params,
                const PlanningParams& planning_params,
                const SmoothingParams& smoothing_params,
                const TaskParams& task_params,
                const RubberBand::ConstPtr& template_band,
                Visualizer::Ptr vis,
                const bool visualization_enabled);

        //////// Planning functions //////////////////////////////////////////////////////////

        RRTPolicy plan(
                const RRTNode& start,
                const RRTGrippersRepresentation& grippers_goal_poses,
                const std::chrono::duration<double>& time_limit);

        const std::vector<RubberBand::ConstPtr>& getBlacklist() const;
        void addBandToBlacklist(const RubberBand& band);
        void clearBlacklist();

        // returns {planning_statistics_, smoothing_statistics_}
        typedef std::pair<std::map<std::string, double>, std::map<std::string, double>> PlanningSmoothingStatistics;
        PlanningSmoothingStatistics getStatistics() const;

        //////// Policy extraction functions /////////////////////////////////////////////////

        static bool CheckTreeLinkage(
                const RRTTree& tree);
        
        static bool CheckPathLinkage(
                const RRTPath& path);

        static std::vector<Eigen::VectorXd> ConvertRRTPathToRobotPath(
                const RRTTree& path);

        static std::vector<std::pair<RRTPath, std::vector<size_t>>> ExtractSolutionPolicy(const RRTTree& tree);

        ///////////////////////////////////////////////////////////////////////////////////////
        // Visualization and other debugging tools
        ///////////////////////////////////////////////////////////////////////////////////////

        std::vector<Visualizer::NamespaceId> visualizeTree(
                const RRTTree& tree,
                const size_t start_idx,
                const std::string ns_a,
                const std::string ns_b,
                const std::string ns_band,
                const int id_a,
                const int id_b,
                const int id_band,
                const std_msgs::ColorRGBA& color_a,
                const std_msgs::ColorRGBA& color_b,
                const std_msgs::ColorRGBA& color_band,
                const bool draw_band = false) const;
        std::vector<Visualizer::NamespaceId> visualizeBothTrees() const;
        void deleteTreeVisualizations() const;
        std::vector<Visualizer::NamespaceId> visualizePath(const RRTPath& path, const std::string& ns_prefix, const int32_t id, const bool draw_band = false) const;
        std::vector<Visualizer::NamespaceId> visualizePolicy(const RRTPolicy& policy, const bool draw_band = false) const;
        std::vector<Visualizer::NamespaceId> visualizeBlacklist() const;

        void storeTree(const RRTTree& tree, std::string file_path = "") const;
        RRTTree loadStoredTree(std::string file_path = "") const;
        bool useStoredTree() const;

        static void SavePath(const RRTPath& path, const std::string& filename);
        static RRTPath LoadPath(const std::string& filename, const RubberBand& template_band);

        void storePolicy(const RRTPolicy& policy, const std::string& file_path) const;
        RRTPolicy loadPolicy(const std::string& file_path) const;

    private:
        ///////////////////////////////////////////////////////////////////////////////////////
        // Helper functions and data for internal rrt planning algorithm
        //  - Order is roughly the order that they are used internally
        ///////////////////////////////////////////////////////////////////////////////////////

        void planningMainLoop();

        //////// Sampling functions //////////////////////////////////////////////////////////

        std::vector<uint8_t> sample_history_buffer_;
        RRTNode configSampling(const bool sample_band);
        // Used for timing purposes
        // https://stackoverflow.com/questions/37786547/enforcing-statement-order-in-c
        RRTGrippersRepresentation posPairSampling_internal();
        RRTRobotRepresentation robotConfigPairSampling_internal();
        EigenHelpers::VectorVector3d bandSampling_internal();

        //////// Nearest neighbour functions /////////////////////////////////////////////////

        // Used for timing purposes
        // https://stackoverflow.com/questions/37786547/enforcing-statement-order-in-c
        int64_t nearestNeighbour(
                const bool use_forward_tree,
                const RRTNode& config);

        std::pair<int64_t, double> nearestNeighbourRobotSpace(
                const bool use_forward_tree,
                const RRTNode& config);

        int64_t nearestBestNeighbourFullSpace(
                const RRTNode& config);

        void rebuildNNIndex(
                std::shared_ptr<NNIndexType> index,
                std::vector<float>& nn_raw_data,
                std::vector<size_t>& nn_data_idx_to_tree_idx,
                const RRTTree& tree,
                size_t& new_data_start_idx,
                const bool force_rebuild);

        //////// Tree extensions functions ///////////////////////////////////////////////////

        size_t connectForwardTree(const int64_t forward_tree_start_idx, const RRTNode& target, const bool is_random);
        size_t connectTreeToGrippersGoalSet(const int64_t last_node_idx_in_forward_tree_branch);

        size_t forwardPropogationFunction(
                RRTTree& tree_to_extend,
                const int64_t& nearest_neighbor_idx,
                const RRTNode& target,
                const bool allow_mistakes,
                const bool visualization_enabled_locally);

        // Returns possible bands, and our confidence in each
        std::vector<std::pair<RubberBand::Ptr, double>> forwardPropogateBand(
                const RubberBand::ConstPtr& starting_band,
                const RRTGrippersRepresentation& next_grippers_poses,
                const bool allow_mistakes);

        //////// Goal check and node blacklist management functions //////////////////////////

        void checkNewStatesForGoal(const ssize_t num_nodes);
        bool goalReached(const RRTNode& node);
        bool isBandFirstOrderVisibileToBlacklist(const RubberBand& test_band);
        bool isBandFirstOrderVisibileToBlacklist_impl(const RubberBand& test_band) const;

        void goalReachedCallback(const int64_t node_idx);
        bool isRootOfGoalBranch(const int64_t node_idx) const;
        void blacklistGoalBranch(const int64_t root_idx);
        void updatePGoalReachable(const int64_t node_idx);

        //////// Shortcut smoothing functions ////////////////////////////////////////////////

        void shortcutSmoothPolicy(
                RRTPolicy& policy,
                const bool visualization_enabled_locally);

        void shortcutSmoothPath(
                RRTPath& path,
                const bool maintain_goal_reach_invariant,
//                const std::vector<QuinlanRubberBand::ConstPtr>& transition_targets,
                const bool visualization_enabled_locally,
                const int32_t visualization_idx);

        std::pair<bool, RRTTree> forwardSimulateGrippersPath(
                const RRTTree& path,
                const size_t start_index,
                RubberBand rubber_band);

    private:
        const std::shared_ptr<ros::NodeHandle> nh_;
        const std::shared_ptr<ros::NodeHandle> ph_;
        const RobotInterface::ConstPtr robot_;
        const bool planning_for_whole_robot_;
        const sdf_tools::SignedDistanceField::ConstPtr sdf_;
        const XYZGrid work_space_grid_;
        const TransitionEstimation::Ptr transition_estimator_;
        const RubberBand template_band_;
        const std::shared_ptr<std::mt19937_64> generator_;
        std::uniform_real_distribution<double> uniform_unit_distribution_;

    private:
        const Eigen::Isometry3d task_aligned_frame_transform_;
        const Eigen::Isometry3d task_aligned_frame_inverse_transform_;
        const Eigen::Vector3d task_aligned_lower_limits_;
        const Eigen::Vector3d task_aligned_upper_limits_;

        const RRTRobotRepresentation robot_joint_lower_limits_;
        const RRTRobotRepresentation robot_joint_upper_limits_;
        const RRTRobotRepresentation robot_joint_weights_;
        const ssize_t total_dof_;

        const double max_gripper_step_size_;
        const double max_robot_dof_step_size_;
        const double min_robot_dof_step_size_;
        const double max_gripper_rotation_;
        const double goal_reach_radius_;
        const double gripper_min_distance_to_obstacles_;

        // Used for double layer NN check
        const double band_distance2_scaling_factor_;
        const size_t band_max_points_;
        const double band_max_dist2_;

        const size_t forward_tree_extend_iterations_;
        const size_t backward_tree_extend_iterations_;
        const bool use_brute_force_nn_;
        const size_t kd_tree_grow_threshold_;
        const double best_near_radius2_;
        const double goal_bias_;

        const int64_t max_shortcut_index_distance_;
        const uint32_t max_smoothing_iterations_;
        const double smoothing_band_dist_threshold_;
        std::uniform_int_distribution<int> uniform_shortcut_smoothing_int_distribution_;

        // Set/updated on each call of "rrtPlan"
        RubberBand::Ptr starting_band_;
        RRTGrippersRepresentation starting_grippers_poses_;
        RRTRobotRepresentation starting_robot_configuration_;

        std::vector<RubberBand::ConstPtr> blacklisted_goal_rubber_bands_;
        double max_grippers_distance_;
        std::chrono::duration<double> time_limit_;
        RRTGrippersRepresentation grippers_goal_poses_;

        // Counters to track which node/transition/split we are on
        int64_t next_state_index_;
        int64_t next_transition_index_;
        int64_t next_split_index_;

        // Trees and nearest neighbour data structures
        RRTTree forward_tree_;
        RRTTree grippers_goal_set_; // Note that the band portion of the backward tree is invalid
        std::vector<size_t> forward_nn_data_idx_to_tree_idx_;
        std::vector<size_t> goal_set_nn_data_idx_to_tree_idx_;
        std::vector<float> forward_nn_raw_data_;
        std::vector<float> goal_set_nn_raw_data_;
        std::shared_ptr<NNIndexType> forward_nn_index_;
        std::shared_ptr<NNIndexType> goal_set_nn_index_;
        size_t forward_next_idx_to_add_to_nn_dataset_;
        size_t goal_set_next_idx_to_add_to_nn_dataset_;


        // Planning, Uncertainty, and Smoothing statistics
        std::map<std::string, double> planning_statistics_;
        std::map<std::string, double> smoothing_statistics_;
        double total_sampling_time_;
        double total_nearest_neighbour_index_building_time_;
        double total_nearest_neighbour_index_searching_time_;
        double total_nearest_neighbour_linear_searching_time_;
        double total_nearest_neighbour_radius_searching_time_;
        double total_nearest_neighbour_best_searching_time_;
        double total_nearest_neighbour_time_;
        double total_forward_kinematics_time_;
        double total_projection_time_;
        double total_collision_check_time_;
        double total_band_forward_propogation_time_;
        double total_first_order_vis_propogation_time_;
        double total_everything_included_forward_propogation_time_;

        size_t forward_random_samples_useful_;
        size_t forward_random_samples_useless_;
        size_t backward_random_samples_useful_;
        size_t backward_random_samples_useless_;
        size_t forward_connection_attempts_useful_;
        size_t forward_connection_attempts_useless_;
        size_t forward_connections_made_;
        size_t backward_connection_attempts_useful_;
        size_t backward_connection_attempts_useless_;
        size_t backward_connections_made_;

        // The backwards tree has no splits, if we're even using it
        size_t forward_transition_id_;
        size_t forward_split_id_;

        // Success and timeout tracking
//        bool path_found_;
//        int64_t goal_idx_in_forward_tree_;
        std::chrono::time_point<std::chrono::steady_clock> start_time_;

        // Visualization
        const Visualizer::Ptr vis_;
        const bool visualization_enabled_globally_;
    public:
        const std_msgs::ColorRGBA gripper_a_forward_tree_color_;
        const std_msgs::ColorRGBA gripper_b_forward_tree_color_;
        const std_msgs::ColorRGBA gripper_a_backward_tree_color_;
        const std_msgs::ColorRGBA gripper_b_backward_tree_color_;
        const std_msgs::ColorRGBA band_tree_color_;
    private:
        // Used in the forward propagation function
        int32_t tree_marker_id_;
        size_t forward_tree_next_visualized_node_;
        size_t backward_tree_next_visualized_node_;
    };
}
