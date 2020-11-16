#ifndef TASK_SPECIFICATION_IMPLEMENTIONS_H
#define TASK_SPECIFICATION_IMPLEMENTIONS_H

#include <smmap_utilities/neighbours.h>
#include "smmap/task_specification.h"
#include "smmap/point_reflector.hpp"

namespace smmap
{
    /**
     * @brief The ClothColabFolding class
     */
    class ClothColabFolding : public TaskSpecification
    {
        public:
            ClothColabFolding(
                    std::shared_ptr<ros::NodeHandle> nh,
                    std::shared_ptr<ros::NodeHandle> ph,
                    Visualizer::Ptr vis);

        private:
            virtual double calculateError_impl(
                    const WorldState& world_state) override final;

            virtual ObjectDeltaAndWeight calculateObjectErrorCorrectionDelta_impl(
                    const WorldState& world_state) override final;

            virtual std::vector<ssize_t> getNodeNeighbours_impl(const ssize_t node) const override final;

            virtual bool taskDone_impl(
                    const WorldState& world_state) override final;

            const Grid4Neighbours neighbours_;

        private:
            const PointReflector point_reflector_;
            static PointReflector CreatePointReflector(ros::NodeHandle& nh);

            const std::map<long, long> mirror_map_;
            static std::map<long, long> CreateMirrorMap(ros::NodeHandle& nh, const PointReflector& point_reflector);
    };

    /**
     * @brief The RopeDirectCoverage class
     */
    class RopeDirectCoverage : public DirectCoverageTask
    {
        public:
            RopeDirectCoverage(
                    std::shared_ptr<ros::NodeHandle> nh,
                    std::shared_ptr<ros::NodeHandle> ph,
                    Visualizer::Ptr vis);

        private:
            virtual std::vector<ssize_t> getNodeNeighbours_impl(const ssize_t node) const override final;

            virtual bool taskDone_impl(
                    const WorldState& world_state) override final;

            const LineNeighbours neighbours_;
    };

    /**
     * @brief The ClothDirectCoverage class
     */
    class ClothDirectCoverage : public DirectCoverageTask
    {
        public:
            ClothDirectCoverage(
                    std::shared_ptr<ros::NodeHandle> nh,
                    std::shared_ptr<ros::NodeHandle> ph,
                    Visualizer::Ptr vis);

        private:
            virtual std::vector<ssize_t> getNodeNeighbours_impl(const ssize_t node) const override final;

            virtual bool taskDone_impl(
                    const WorldState& world_state) override final;

            const Grid4Neighbours neighbours_;
    };

    /**
     * @brief The RopeDistanceBasedCorrespondences class. Uses the Dijkstra's
     * field to determine distances and directions to manipulate objects.
     * Correspondences are determined based on a dynamic "what's nearest right
     * now basis.
     */
    class RopeDistanceBasedCorrespondences : public DistanceBasedCorrespondencesTask
    {
        public:
            RopeDistanceBasedCorrespondences(
                    std::shared_ptr<ros::NodeHandle> nh,
                    std::shared_ptr<ros::NodeHandle> ph,
                    Visualizer::Ptr vis);

        private:
            virtual std::vector<ssize_t> getNodeNeighbours_impl(const ssize_t node) const override final;

            virtual bool taskDone_impl(
                    const WorldState& world_state) override final;

            const LineNeighbours neighbours_;
    };

    /**
     * @brief The ClothDistanceBasedCorrespondences class. Uses the Dijkstra's
     * field to determine distances and directions to manipulate objects.
     * Correspondences are determined based on a dynamic "what's nearest right
     * now basis.
     */
    class ClothDistanceBasedCorrespondences : public DistanceBasedCorrespondencesTask
    {
        public:
            ClothDistanceBasedCorrespondences(
                    std::shared_ptr<ros::NodeHandle> nh,
                    std::shared_ptr<ros::NodeHandle> ph,
                    Visualizer::Ptr vis);

        private:
            virtual std::vector<ssize_t> getNodeNeighbours_impl(const ssize_t node) const override final;

            virtual bool taskDone_impl(
                    const WorldState& world_state) override final;

            const Grid4Neighbours neighbours_;
    };

    /**
     * @brief The RopeFixedCorrespondences class. Uses the Dijkstra's
     * field to determine distances and directions to manipulate objects.
     * Correspondences are determined based on a fixed apriori assignment.
     */
    class RopeFixedCorrespondences : public FixedCorrespondencesTask
    {
        public:
            RopeFixedCorrespondences(
                    std::shared_ptr<ros::NodeHandle> nh,
                    std::shared_ptr<ros::NodeHandle> ph,
                    Visualizer::Ptr vis);

        private:
            virtual std::vector<ssize_t> getNodeNeighbours_impl(const ssize_t node) const override final;

            virtual bool taskDone_impl(
                    const WorldState& world_state) override final;

            const LineNeighbours neighbours_;
    };

    /**
     * @brief The ClothFixedCorrespondences class. Uses the Dijkstra's
     * field to determine distances and directions to manipulate objects.
     * Correspondences are determined based on a fixed apriori assignment.
     */
    class ClothFixedCorrespondences : public FixedCorrespondencesTask
    {
        public:
            ClothFixedCorrespondences(
                    std::shared_ptr<ros::NodeHandle> nh,
                    std::shared_ptr<ros::NodeHandle> ph,
                    Visualizer::Ptr vis);

        private:
            virtual std::vector<ssize_t> getNodeNeighbours_impl(const ssize_t node) const override final;

            virtual bool taskDone_impl(
                    const WorldState& world_state) override final;

            // TODO: Not clear that this will be the case moving forward - are all nodes in a grid?
            const Grid4Neighbours neighbours_;
    };
}

#endif // TASK_SPECIFICATION_IMPLEMENTIONS_H
