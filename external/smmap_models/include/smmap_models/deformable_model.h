#ifndef DEFORMABLE_MODEL_H
#define DEFORMABLE_MODEL_H

#include <atomic>
#include <arc_utilities/log.hpp>
#include <smmap_utilities/grippers.h>

#include <smmap_utilities/task_function_pointer_types.h>

namespace smmap
{
    class DeformableModel
    {
        public:
            typedef std::shared_ptr<DeformableModel> Ptr;
            typedef std::shared_ptr<const DeformableModel> ConstPtr;

            DeformableModel(std::shared_ptr<ros::NodeHandle> nh);

            ////////////////////////////////////////////////////////////////////
            // Virtual functions that define the interface
            ////////////////////////////////////////////////////////////////////

            void updateModel(const WorldState& previous, const WorldState& next);

            ObjectPointSet getObjectDelta(
                    const WorldState& world_state,
                    const AllGrippersSinglePoseDelta& grippers_pose_delta) const;

            ////////////////////////////////////////////////////////////////////
            // Update/Set function for static members
            ////////////////////////////////////////////////////////////////////

            static void SetGrippersData(
                    const std::vector<GripperData>& grippers_data);

            static const std::vector<GripperData>& GetGrippersData();

            static void SetCallbackFunctions(
                    const GripperCollisionCheckFunction& gripper_collision_check_fn);


        protected:

            ////////////////////////////////////////////////////////////////////
            // Destructor that prevents "delete pointer to base object"
            ////////////////////////////////////////////////////////////////////

            ~DeformableModel() {}

            ////////////////////////////////////////////////////////////////////
            // Static data
            ////////////////////////////////////////////////////////////////////

            static std::atomic<bool> grippers_data_initialized_;
            static std::vector<GripperData> grippers_data_;

            static std::atomic_bool function_pointers_initialized_;
            static GripperCollisionCheckFunction gripper_collision_check_fn_;

        private:
            virtual void updateModel_impl(const WorldState& previous, const WorldState& next) = 0;

            virtual ObjectPointSet getObjectDelta_impl(
                    const WorldState& world_state,
                    const AllGrippersSinglePoseDelta& grippers_pose_delta) const = 0;

            ////////////////////////////////////////////////////////////////////
            // Logging
            ////////////////////////////////////////////////////////////////////

//            const std::shared_ptr<Log::Log> computation_time_log_;
    };
}

#endif // DEFORMABLE_MODEL_H
