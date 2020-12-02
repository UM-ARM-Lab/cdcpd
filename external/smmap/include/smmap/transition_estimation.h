#ifndef TRANSITION_ESTIMATION_H
#define TRANSITION_ESTIMATION_H

#include <arc_utilities/maybe.hpp>
#include <smmap_utilities/visualization_tools.h>
#include <smmap_utilities/trajectory.hpp>
#include "smmap/quinlan_rubber_band.h"
#include "smmap/task_specification.h"
#include "smmap/classifier.h"
#include "smmap/voxnet_classifier.h"

namespace smmap
{
    class TransitionEstimation
    {
    public:
        typedef std::shared_ptr<TransitionEstimation> Ptr;
        typedef std::shared_ptr<const TransitionEstimation> ConstPtr;

        struct State
        {
        public:
            ObjectPointSet deform_config_;
            RubberBand::Ptr rubber_band_;
            RubberBand::Ptr planned_rubber_band_;
            EigenHelpers::VectorIsometry3d rope_node_transforms_;

            uint64_t serialize(std::vector<uint8_t>& buffer) const;

            uint64_t deserialize(
                    const std::vector<uint8_t>& buffer,
                    const uint64_t current);

            static uint64_t Serialize(
                    const State& state,
                    std::vector<uint8_t>& buffer);

            static std::pair<State, uint64_t> Deserialize(
                    const std::vector<uint8_t>& buffer,
                    const uint64_t current,
                    const RubberBand& template_band);

            bool operator==(const State& other) const;
            bool operator!=(const State& other) const;
        };

        struct StateTransition
        {
        public:
            State starting_state_;
            State ending_state_;
            // This is the target position of the grippers.
            // In practice this data is duplicated in the endpoints of the band,
            // but this is being kept to keep everything in the
            // "state, action, next state" framework
            PairGripperPositions starting_gripper_positions_;
            PairGripperPositions ending_gripper_positions_;
            std::vector<WorldState> microstep_state_history_;
            std::vector<RubberBand::Ptr> microstep_band_history_;

            uint64_t serialize(std::vector<uint8_t>& buffer) const;

            uint64_t deserialize(
                    const std::vector<uint8_t>& buffer,
                    const uint64_t current);

            static uint64_t Serialize(
                    const StateTransition& state_transition,
                    std::vector<uint8_t>& buffer);

            static std::pair<StateTransition, uint64_t> Deserialize(
                    const std::vector<uint8_t>& buffer,
                    const uint64_t current,
                    const RubberBand& template_band);

            bool operator==(const StateTransition& other) const;
            bool operator!=(const StateTransition& other) const;

            std::string toString() const;
        };

        typedef std::pair<State, std::vector<WorldState>> StateMicrostepsPair;
        typedef std::vector<StateMicrostepsPair> StateTrajectory;

        ////////////////////////////////////////////////////////////////////////
        // Constructor
        ////////////////////////////////////////////////////////////////////////

        TransitionEstimation(
                std::shared_ptr<ros::NodeHandle> nh,
                std::shared_ptr<ros::NodeHandle> ph,
                std::shared_ptr<std::mt19937_64> generator,
                const sdf_tools::SignedDistanceField::ConstPtr& sdf,
                const XYZGrid work_space_grid,
                const Visualizer::Ptr& vis,
                const RubberBand& template_band);

        ////////////////////////////////////////////////////////////////////////
        // Helper functions - used externally and internally
        ////////////////////////////////////////////////////////////////////////

        // Assumes the vectors have already been appropriately discretized/resampled
        bool checkFirstOrderHomotopyPoints(
                const EigenHelpers::VectorVector3d& b1,
                const EigenHelpers::VectorVector3d& b2) const;
        bool checkMatchedStarightLineHomotopyPoints(
                const EigenHelpers::VectorVector3d& b1,
                const EigenHelpers::VectorVector3d& b2) const;
        // Resamples the bands appropriately, then calls the above
        bool checkFirstOrderHomotopy(
                const RubberBand& b1,
                const RubberBand& b2) const;

        std::vector<RubberBand::Ptr> reduceMicrostepsToBands(
                const std::vector<WorldState>& microsteps) const;

        ////////////////////////////////////////////////////////////////////////
        // Learning Transitions
        ////////////////////////////////////////////////////////////////////////

        Maybe::Maybe<StateTransition> findMostRecentBadTransition(
                const StateTrajectory& trajectory,
                const bool visualize = false) const;

        void learnTransition(const StateTransition& transition);

        std::vector<RubberBand> extractBandSurface(const StateTransition& transition) const;

        ////////////////////////////////////////////////////////////////////////
        // Using transitions
        ////////////////////////////////////////////////////////////////////////

        const std::vector<StateTransition>& transitions() const;

        struct TransitionAdaptationResult
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            // Results
            RubberBand::Ptr default_next_band_;
            RubberBand::Ptr result_;

            // Intermediary calculations
            ObjectPointSet target_points_to_match_;
            ObjectPointSet template_points_to_align_;
            Eigen::Isometry3d invariant_transform_;
            ObjectPointSet template_planned_band_aligned_to_target_;
            ObjectPointSet next_band_points_to_smooth_;
            ObjectPointSet transformed_band_surface_points_;
            std::vector<RubberBand::Ptr> tightened_transformed_bands_;
            ObjectPointSet tightened_transformed_bands_surface_;
            std::vector<bool> foh_values_;

            // "Distance" evaluations
            double template_misalignment_dist_;
            bool default_band_foh_result_;
            double default_band_dist_;
            double band_tighten_delta_;
            int num_foh_changes_;

            uint64_t serialize(std::vector<uint8_t>& buffer) const;

            static uint64_t Serialize(
                    const TransitionAdaptationResult& adaptation_result,
                    std::vector<uint8_t>& buffer);

            static std::pair<TransitionAdaptationResult, uint64_t> Deserialize(
                    const std::vector<uint8_t>& buffer,
                    const uint64_t current,
                    const RubberBand& template_band);

            bool operator==(const TransitionAdaptationResult& other) const;

            bool operator!=(const TransitionAdaptationResult& other) const;
        };

        TransitionAdaptationResult generateTransition(
                const StateTransition& stored_trans,
                const RubberBand& test_band_start,
                const PairGripperPositions& ending_gripper_positions) const;

        std::string classifierName() const;
        // Returns a string representation of slice_type, normalize_lengths, etc.
        std::string featuresUsed() const;

        Eigen::VectorXd transitionFeatures(const RubberBand& initial_band,
                                           const RubberBand& default_prediction,
                                           const bool verbose = false) const;

        // Returns vector of potential outcomes of the action, and a relative
        // confidence from 0 (not likely) to 1 (input data exactly matched a
        // stored transition) in the possibility that the transition is possible.
        // I.e.; confidence 1 does not mean that the transition will happen, but
        // rather that it *could* happen.
        std::vector<std::pair<RubberBand::Ptr, double>> estimateTransitions(
                const RubberBand& test_band_start,
                const PairGripperPositions& ending_gripper_positions,
                const bool allow_mistakes,
                const bool verbose = false);

        void addExperienceToClassifier(const StateTrajectory& trajectory);

        void resetStatistics();
        double classifierTime() const;
        size_t numBandWeirdness() const;
        size_t numBandSafe() const;
        size_t numBandOverstretch() const;
        size_t numNoMistakes() const;
        size_t numMistakes() const;
        size_t numAcceptedMistakes() const;

        ////////////////////////////////////////////////////////////////////////
        // Visualizing transitions
        ////////////////////////////////////////////////////////////////////////

        void visualizeTransition(
                const StateTransition& transition,
                const int32_t id = 1,
                const std::string& ns_prefix = "") const;

        static void VisualizeTransition(
                const Visualizer::Ptr& vis,
                const StateTransition& transition,
                const int32_t id = 1,
                const std::string& ns_prefix = "");

        void visualizeLearnedTransitions(
                const std::string& ns_prefix = "all_") const;

        void clearVisualizations() const;

        // Namespaces used for publishing visualization data
        static constexpr char MDP_PRE_STATE_NS[]        = "mdp_pre_state";
        static constexpr char MDP_TESTING_STATE_NS[]    = "mdp_testing_state";
        static constexpr char MDP_POST_STATE_NS[]       = "mdp_post_state";

        ////////////////////////////////////////////////////////////////////////
        // Saving and loading helpers
        ////////////////////////////////////////////////////////////////////////

        void saveStateTransition(const StateTransition& state, const std::string& filename) const;
        StateTransition loadStateTransition(const std::string& filename) const;

        void saveTrajectory(const StateTrajectory& trajectory, const std::string& filename) const;
        StateTrajectory loadTrajectory(const std::string& filename) const;

        void saveAdaptationResult(const TransitionAdaptationResult& result, const std::string& filename) const;
        TransitionAdaptationResult loadAdaptationResult(const std::string& filename) const;

    private:

        const std::shared_ptr<ros::NodeHandle> nh_;
        const std::shared_ptr<ros::NodeHandle> ph_;

        const sdf_tools::SignedDistanceField::ConstPtr sdf_;
        const XYZGrid work_space_grid_;
        const Visualizer::Ptr vis_;
        std::vector<StateTransition> learned_transitions_;
        std::vector<std::vector<RubberBand>> learned_band_surfaces_;

        const double default_propogation_confidence_;
#if 0
        const double default_band_dist_threshold_;
        const double confidence_threshold_;
        const double template_misalignment_scale_factor_;
        const double band_tighten_scale_factor_;
        const double homotopy_changes_scale_factor_;
#endif

        ////////////////////////////////////////////////////////////////////////
        // Default transition mistake estimation
        ////////////////////////////////////////////////////////////////////////

        const double mistake_dist_thresh_;
        const bool normalize_lengths_;
        const bool normalize_connected_components_;

        #warning "Voxnet classifier hack addition to classification framework"
        std::shared_ptr<VoxnetClassifier> voxnet_classifier_;

        Classifier::Ptr const transition_mistake_classifier_;
        double const accept_scale_factor_;
        #warning "Voxnet classifier hack addition to classification framework"
        double accept_mistake_rate_;
        std::uniform_real_distribution<double> accept_transition_distribution_;
        std::shared_ptr<std::mt19937_64> generator_;
        double classifier_time_;
        size_t num_band_weirdness_;
        size_t num_band_safe_;
        size_t num_band_overstretch_;
        size_t num_no_mistake_;
        size_t num_mistake_;
        size_t num_accepted_mistake_;

        ////////////////////////////////////////////////////////////////////////
        // Saving and loading learned transitions
        ////////////////////////////////////////////////////////////////////////

        bool useStoredTransitions() const;
        void storeTransitions() const;
        void loadSavedTransitions();
        const RubberBand template_band_;
    };

    std::ostream& operator<<(
            std::ostream& out,
            const TransitionEstimation::StateTransition& t);
}

#endif
