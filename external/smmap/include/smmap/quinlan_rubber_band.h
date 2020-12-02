#ifndef QUINLAN_RUBBER_BAND
#define QUINLAN_RUBBER_BAND

#include <arc_utilities/eigen_typedefs.hpp>
#include <sdf_tools/sdf.hpp>
#include <deformable_manipulation_experiment_params/xyzgrid.h>
#include <smmap_utilities/visualization_tools.h>
#include <smmap_utilities/grippers.h>
#include <smmap_utilities/trajectory.hpp>

namespace smmap
{
    class QuinlanRubberBand
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef std::shared_ptr<QuinlanRubberBand> Ptr;
        typedef std::shared_ptr<const QuinlanRubberBand> ConstPtr;

        static std::atomic<bool> ENABLE_BAND_DEBUGGING_;
        // For any of the below to be meaningful, ENABLE_BAND_DEBUGGING must be true
        static std::atomic<bool> ENABLE_INTERPOLATE_DEBUGGING_;
        static std::atomic<bool> ENABLE_REMOVE_DEBUGGING_;
        static std::atomic<bool> ENABLE_SMOOTHING_DEBUGGING_;

        static ObjectPointSet AggregateBandPoints(
                const std::vector<QuinlanRubberBand>& bands);
        static ObjectPointSet AggregateBandPoints(
                const std::vector<QuinlanRubberBand::Ptr>& bands);
        static ObjectPointSet AggregateBandPoints(
                const std::vector<QuinlanRubberBand::ConstPtr>& bands);

        static ObjectPointSet PointsFromBandPointsAndGripperTargets(
                const EigenHelpers::VectorVector3d& starting_points,
                const PairGripperPositions& grippers_targets,
                const size_t num_gripper_steps);
        static ObjectPointSet PointsFromBandAndGrippers(
                const QuinlanRubberBand& band,
                const PairGripperPositions& grippers_start,
                const PairGripperPositions& grippers_end,
                const size_t num_gripper_steps);

        static constexpr auto SMOOTHING_CLOSE_ENGOUGH_DIST = 1e-4;
        static constexpr auto MAX_DELTA_SCALE_FACTOR = 0.9;
        static constexpr auto MIN_OVERLAP_SCALE_FACTOR = 0.05;
        static constexpr auto MIN_DIST_TO_OBSTACLES_SCALE_FACTOR = 0.1;
        static constexpr auto NODE_REMOVAL_OVERLAP_FACTOR = 1.2;
        static constexpr auto BACKTRACKING_THRESHOLD = 0.1;
        static constexpr auto SMOOTHING_ITERATIONS = 100;

        static QuinlanRubberBand::Ptr BandFromNodeTransformsAndGrippers(
                const EigenHelpers::VectorIsometry3d& node_transforms,
                const PairGripperPositions& grippers_position,
                const QuinlanRubberBand& template_band);

        static QuinlanRubberBand::Ptr BandFromWorldState(
                const WorldState& world_state,
                const QuinlanRubberBand& template_band);

        QuinlanRubberBand(
                std::shared_ptr<ros::NodeHandle> nh,
                std::shared_ptr<ros::NodeHandle> ph,
                const Visualizer::Ptr vis,
                const sdf_tools::SignedDistanceField::ConstPtr& sdf,
                const XYZGrid& work_space_grid,
                const std::function<std::vector<ssize_t>(const ssize_t node)>& node_neighbours_fn,
                const WorldState& world_state,
                const double resample_max_pointwise_dist,
                const size_t upsample_num_points,
                const double max_safe_band_length);

        QuinlanRubberBand& operator=(const QuinlanRubberBand& other);

        [[gnu::warn_unused_result]] bool setPointsWithoutSmoothing(const EigenHelpers::VectorVector3d& points);
        [[gnu::warn_unused_result]] bool setPointsWithoutSmoothing(const ObjectPointSet& points);

        [[gnu::warn_unused_result]] bool setPointsAndSmooth(const EigenHelpers::VectorVector3d& points);
        [[gnu::warn_unused_result]] bool setPointsAndSmooth(const ObjectPointSet& points);

        [[gnu::warn_unused_result]] bool resetBand(const WorldState& world_state);
        [[gnu::warn_unused_result]] bool resetBand(const ObjectPointSet& object_config,
                                                   const PairGripperPositions& gripper_positions);

        void overridePoints(const EigenHelpers::VectorVector3d& points);

        const EigenHelpers::VectorVector3d& forwardPropagate(
                const PairGripperPositions& gripper_positions,
                bool verbose);

        const EigenHelpers::VectorVector3d& getVectorRepresentation() const;
        // Not threadsafe: https://www.justsoftwaresolutions.co.uk/cplusplus/const-and-thread-safety.htm
        const EigenHelpers::VectorVector3d& resampleBand() const;
        // Not threadsafe: https://www.justsoftwaresolutions.co.uk/cplusplus/const-and-thread-safety.htm
        const EigenHelpers::VectorVector3d& upsampleBand() const;
        const Eigen::VectorXd& upsampleBandSingleVector() const;

        Pair3dPositions getEndpoints() const;

        double maxSafeLength() const;
        double totalLength() const;
        bool isOverstretched() const;

        std::vector<Visualizer::NamespaceId> visualize(
                const std::string& marker_name,
                const std_msgs::ColorRGBA& safe_color,
                const std_msgs::ColorRGBA& overstretched_color,
                const int32_t id,
                const bool visualization_enabled = true) const;

        std::vector<Visualizer::NamespaceId> visualize(
                const EigenHelpers::VectorVector3d& test_band,
                const std::string& marker_name,
                const std_msgs::ColorRGBA& safe_color,
                const std_msgs::ColorRGBA& overstretched_color,
                const int32_t id,
                const bool visualization_enabled = true) const;

        std::vector<Visualizer::NamespaceId> visualizeWithBubbles(
                const std::string& marker_name,
                const std_msgs::ColorRGBA& safe_color,
                const std_msgs::ColorRGBA& overstretched_color,
                const int32_t id,
                const bool visualization_enabled = true) const;

        std::vector<Visualizer::NamespaceId> visualizeWithBubbles(
                const EigenHelpers::VectorVector3d& test_band,
                const std::string& marker_name,
                const std_msgs::ColorRGBA& safe_color,
                const std_msgs::ColorRGBA& overstretched_color,
                const int32_t id,
                const bool visualization_enabled = true) const;

        static std::vector<Visualizer::NamespaceId> VisualizeBandSurface(
                const Visualizer::Ptr& vis,
                const ObjectPointSet& band_surface,
                const size_t num_bands,
                const std_msgs::ColorRGBA& start_color,
                const std_msgs::ColorRGBA& end_color,
                const std::string& ns,
                const int32_t id = 1);

        static std::vector<Visualizer::NamespaceId> VisualizeBandSurface(
                const Visualizer::Ptr& vis,
                const std::vector<QuinlanRubberBand>& bands,
                const std_msgs::ColorRGBA& start_color,
                const std_msgs::ColorRGBA& end_color,
                const std::string& ns,
                const int32_t id = 1);

        static std::vector<Visualizer::NamespaceId> VisualizeBandSurface(
                const Visualizer::Ptr& vis,
                const std::vector<QuinlanRubberBand::Ptr>& bands,
                const std_msgs::ColorRGBA& start_color,
                const std_msgs::ColorRGBA& end_color,
                const std::string& ns,
                const int32_t id = 1);

        static std::vector<Visualizer::NamespaceId> VisualizeBandSurface(
                const Visualizer::Ptr& vis,
                const std::vector<QuinlanRubberBand::ConstPtr>& bands,
                const std_msgs::ColorRGBA& start_color,
                const std_msgs::ColorRGBA& end_color,
                const std::string& ns,
                const int32_t id = 1);

        uint64_t serialize(std::vector<uint8_t>& buffer) const;
        uint64_t deserialize(const std::vector<uint8_t>& buffer, const uint64_t current);
        static uint64_t Serialize(const QuinlanRubberBand::ConstPtr& band, std::vector<uint8_t>& buffer);
        static std::pair<QuinlanRubberBand::Ptr, uint64_t> Deserialize(
                const std::vector<uint8_t>& buffer,
                const uint64_t current,
                const QuinlanRubberBand& template_band);

        bool operator==(const QuinlanRubberBand& other) const;
        bool operator!=(const QuinlanRubberBand& other) const;

        double distanceSq(const EigenHelpers::VectorVector3d& other) const;
        double distance(const EigenHelpers::VectorVector3d& other) const;
        double distanceSq(const QuinlanRubberBand& other) const;
        double distance(const QuinlanRubberBand& other) const;
        static double DistanceSq(const QuinlanRubberBand& b1, const QuinlanRubberBand& b2);
        static double Distance(const QuinlanRubberBand& b1, const QuinlanRubberBand& b2);

    private:
        const std::shared_ptr<ros::NodeHandle> nh_;
        const std::shared_ptr<ros::NodeHandle> ph_;
        const sdf_tools::SignedDistanceField::ConstPtr sdf_;
        const XYZGrid work_space_grid_;
        const Visualizer::Ptr vis_;

        const std::vector<ssize_t> path_between_grippers_through_object_;
        EigenHelpers::VectorVector3d band_;
        // Not threadsafe: https://www.justsoftwaresolutions.co.uk/cplusplus/const-and-thread-safety.htm
        mutable EigenHelpers::VectorVector3d resampled_band_; // is cleared every time band_ is updated
        const double resample_max_pointwise_dist_;
        mutable EigenHelpers::VectorVector3d upsampled_band_;  // is cleared every time band_ is updated
        mutable Eigen::VectorXd upsampled_band_single_vector_; // is updated every time upsampled_band_ is updated
        const size_t upsample_num_points_;


        const double max_safe_band_length_;
        const double min_overlap_distance_;
        const double min_distance_to_obstacle_;
        const double node_removal_overlap_factor_;
        const double backtrack_threshold_;
        const size_t smoothing_iterations_;

        Eigen::Vector3d projectToValidBubble(const Eigen::Vector3d& location) const;
        double getBubbleSize(const Eigen::Vector3d& location) const;
        bool sufficientOverlap(
                const double bubble_size_a,
                const double bubble_size_b,
                const double distance) const;
        bool bandIsValid() const;
        bool bandIsValid(const EigenHelpers::VectorVector3d& test_band) const;
        bool bandIsValidWithVisualization() const;
        bool bandIsValidWithVisualization(const EigenHelpers::VectorVector3d& test_band) const;

        [[gnu::warn_unused_result]] bool interpolateBetweenPoints(
                EigenHelpers::VectorVector3d& point_buffer,
                const Eigen::Vector3d& target) const;
        [[gnu::warn_unused_result]] bool interpolateBandPoints();
        void removeExtraBandPoints(const bool verbose);
        [[gnu::warn_unused_result]] bool smoothBandPoints(const bool verbose);

        void printBandData(const EigenHelpers::VectorVector3d& test_band) const;

        void storeBand() const;
        void loadStoredBand();
        bool useStoredBand() const;
    };

    typedef QuinlanRubberBand RubberBand;
}

#endif //QUINLAN_RUBBER_BAND
