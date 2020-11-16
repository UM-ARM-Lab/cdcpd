#include <stdlib.h>
#include <vector>
#include <string>
#include <stdexcept>
#include <functional>
#include <chrono>
#include <random>
#include <memory>
#include <Eigen/Dense>
#include <arc_utilities/serialization.hpp>

#ifndef SIMPLE_RRT_PLANNER_HPP
#define SIMPLE_RRT_PLANNER_HPP

namespace simple_rrt_planner
{
    template<typename T, typename Allocator = std::allocator<T>>
    class SimpleRRTPlannerState
    {
    protected:

        T value_;
        std::vector<int64_t> child_indices_;
        int64_t parent_index_;
        bool initialized_;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        static uint64_t Serialize(const SimpleRRTPlannerState<T, Allocator>& state, std::vector<uint8_t>& buffer, const std::function<uint64_t(const T&, std::vector<uint8_t>&)>& value_serializer)
        {
            return state.SerializeSelf(buffer, value_serializer);
        }

        static std::pair<SimpleRRTPlannerState<T, Allocator>, uint64_t> Deserialize(const std::vector<uint8_t>& buffer, const uint64_t current, const std::function<std::pair<T, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
        {
            SimpleRRTPlannerState<T, Allocator> temp_state;
            const uint64_t bytes_read = temp_state.DeserializeSelf(buffer, current, value_deserializer);
            return std::make_pair(temp_state, bytes_read);
        }

        SimpleRRTPlannerState()
            : parent_index_(-1)
            , initialized_(false)
        {
            child_indices_.clear();
        }

        SimpleRRTPlannerState(const T& value, const int64_t parent_index, const std::vector<int64_t>& child_indices)
            : value_(value)
            , child_indices_(child_indices)
            , parent_index_(parent_index)
            , initialized_(true)
        {}

        SimpleRRTPlannerState(const T& value, const int64_t parent_index)
            : value_(value)
            , parent_index_(parent_index)
            , initialized_(true)
        {
            child_indices_.clear();
        }

        explicit SimpleRRTPlannerState(const T& value)
            : value_(value)
            , parent_index_(-1)
            , initialized_(true)
        {
            child_indices_.clear();
        }

        uint64_t SerializeSelf(std::vector<uint8_t>& buffer, const std::function<uint64_t(const T&, std::vector<uint8_t>&)>& value_serializer) const
        {
            const uint64_t start_buffer_size = buffer.size();
            // Serialize the initialized
            arc_utilities::SerializeFixedSizePOD<uint8_t>((uint8_t)initialized_, buffer);
            // Serialize the parent index
            arc_utilities::SerializeFixedSizePOD<int64_t>(parent_index_, buffer);
            // Serialize the child indices
            arc_utilities::SerializeVector<int64_t>(child_indices_, buffer, arc_utilities::SerializeFixedSizePOD<int64_t>);
            // Serialize the value
            value_serializer(value_, buffer);
            // Figure out how many bytes were written
            const uint64_t end_buffer_size = buffer.size();
            const uint64_t bytes_written = end_buffer_size - start_buffer_size;
            return bytes_written;
        }

        uint64_t DeserializeSelf(const std::vector<uint8_t>& buffer, const uint64_t current, const std::function<std::pair<T, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& value_deserializer)
        {
            uint64_t current_position = current;
            // Deserialize the initialized
            const std::pair<uint8_t, uint64_t> initialized_deserialized = arc_utilities::DeserializeFixedSizePOD<uint8_t>(buffer, current_position);
            initialized_ = (bool)initialized_deserialized.first;
            current_position += initialized_deserialized.second;
            // Deserialize the parent index
            const std::pair<int64_t, uint64_t> parent_index_deserialized = arc_utilities::DeserializeFixedSizePOD<int64_t>(buffer, current_position);
            parent_index_ = parent_index_deserialized.first;
            current_position += parent_index_deserialized.second;
            // Deserialize the child indices
            const std::pair<std::vector<int64_t>, uint64_t> child_indices_deserialized = arc_utilities::DeserializeVector<int64_t>(buffer, current_position, arc_utilities::DeserializeFixedSizePOD<int64_t>);
            child_indices_ = child_indices_deserialized.first;
            current_position += child_indices_deserialized.second;
            // Deserialize the value
            const std::pair<T, uint64_t> value_deserialized = value_deserializer(buffer, current_position);
            value_ = value_deserialized.first;
            current_position += value_deserialized.second;
            // Figure out how many bytes were read
            const uint64_t bytes_read = current_position - current;
            return bytes_read;
        }

        bool IsInitialized() const
        {
            return initialized_;
        }

        const T& GetValueImmutable() const
        {
            return value_;
        }

        T& GetValueMutable()
        {
            return value_;
        }

        int64_t GetParentIndex() const
        {
            return parent_index_;
        }

        void SetParentIndex(const int64_t parent_index)
        {
            parent_index_ = parent_index;
        }

        const std::vector<int64_t>& GetChildIndices() const
        {
            return child_indices_;
        }

        void ClearChildIndicies()
        {
            child_indices_.clear();
        }

        void AddChildIndex(const int64_t child_index)
        {
            for (size_t idx = 0; idx < child_indices_.size(); idx++)
            {
                if (child_indices_[idx] == child_index)
                {
                    return;
                }
            }
            child_indices_.push_back(child_index);
        }

        void RemoveChildIndex(const int64_t child_index)
        {
            std::vector<int64_t> new_child_indices;
            for (size_t idx = 0; idx < child_indices_.size(); idx++)
            {
                if (child_indices_[idx] != child_index)
                {
                    new_child_indices.push_back(child_indices_[idx]);
                }
            }
            child_indices_ = new_child_indices;
        }
    };

    class SimpleHybridRRTPlanner
    {
    private:

        SimpleHybridRRTPlanner() {}

    public:

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * start - starting configuration
         * goal - target configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of the goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * goal_bias - in (0, 1), selects the probability that the new sampled state is the goal state
         * time_limit - limit, in seconds, for the runtime of the planner
         * rng - a random number generator matching the interface of the generators provided by std::random
         *
         * Returns:
         * std::pair<path, statistics>
         * path - vector of states corresponding to the planned path
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        template<typename RNG, typename T, typename Allocator=std::allocator<T>>
        static std::pair<std::vector<T, Allocator>, std::map<std::string, double>> Plan(
                const T& start,
                const T& goal,
                const std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&,const T&)>& nearest_neighbor_fn,
                const std::function<bool(const T&)>& goal_reached_fn,
                const std::function<T(void)>& state_sampling_fn,
                const std::function<std::vector<std::pair<T, int64_t>>(const T&, const T&)>& forward_propagation_fn,
                const double goal_bias,
                const std::chrono::duration<double>& time_limit,
                RNG& rng)
        {
            std::uniform_real_distribution<double> goal_bias_distribution(0.0, 1.0);
            const std::function<T(void)> sampling_function = [&](void) { return ((goal_bias_distribution(rng) > goal_bias) ? state_sampling_fn() : goal); };
            return Plan(start, nearest_neighbor_fn, goal_reached_fn, sampling_function, forward_propagation_fn, time_limit);
        }

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * start - starting configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * goal_sampling_fn - returns a goal state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * goal_bias - in (0, 1), selects the probability that the new sampled state is a goal state
         * time_limit - limit, in seconds, for the runtime of the planner
         * rng - a random number generator matching the interface of the generators provided by std::random
         *
         * Returns:
         * std::pair<path, statistics>
         * path - vector of states corresponding to the planned path
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        template<typename RNG, typename T, typename Allocator=std::allocator<T>>
        static std::pair<std::vector<T, Allocator>, std::map<std::string, double>> Plan(
                const T& start,
                const std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn,
                const std::function<bool(const T&)>& goal_reached_fn,
                const std::function<T(void)>& state_sampling_fn,
                const std::function<T(void)>& goal_sampling_fn,
                const std::function<std::vector<std::pair<T, int64_t>>(const T&, const T&)>& forward_propagation_fn,
                const double goal_bias,
                const std::chrono::duration<double>& time_limit,
                RNG& rng)
        {
            std::uniform_real_distribution<double> goal_bias_distribution(0.0, 1.0);
            const std::function<T(void)> sampling_function = [&](void) { return ((goal_bias_distribution(rng) > goal_bias) ? state_sampling_fn() : goal_sampling_fn()); };
            return Plan(start, nearest_neighbor_fn, goal_reached_fn, sampling_function, forward_propagation_fn, time_limit);
        }

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * start - starting configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * time_limit - limit, in seconds, for the runtime of the planner
         *
         * Returns:
         * std::pair<path, statistics>
         * path - vector of states corresponding to the planned path
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        template<typename T, typename Allocator=std::allocator<T>>
        static std::pair<std::vector<T, Allocator>, std::map<std::string, double>> Plan(
                const T& start,
                const std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn,
                const std::function<bool(const T&)>& goal_reached_fn,
                const std::function<T(void)>& sampling_fn,
                const std::function<std::vector<std::pair<T, int64_t>>(const T&, const T&)>& forward_propagation_fn,
                const std::chrono::duration<double>& time_limit)
        {
            std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
            const std::function<bool(void)> termination_check_fn = [&](void) { return (((std::chrono::time_point<std::chrono::steady_clock>)std::chrono::steady_clock::now() - start_time) > time_limit); };
            return Plan(start, nearest_neighbor_fn, goal_reached_fn, sampling_fn, forward_propagation_fn, termination_check_fn);
        }

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * start - starting configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * termination_check_fn - returns if the planner should terminate (for example, if it has exceeded time/space limits)
         *
         * Returns:
         * std::pair<path, statistics>
         * path - vector of states corresponding to the planned path
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        template<typename T, typename Allocator=std::allocator<T>>
        static std::pair<std::vector<T, Allocator>, std::map<std::string, double>> Plan(
                const T& start,
                const std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn,
                const std::function<bool(const T&)>& goal_reached_fn,
                const std::function<T(void)>& sampling_fn,
                const std::function<std::vector<std::pair<T, int64_t>>(const T&, const T&)>& forward_propagation_fn,
                const std::function<bool(void)>& termination_check_fn)
        {
            // Define a couple lambdas to let us use the generic multi-path planner as if it were a single-path planner
            bool solution_found = false;
            const std::function<bool(const T&)> real_goal_found_fn = [&](const T& state) { if (goal_reached_fn(state)) { solution_found = true; return true; } else {return false;} };
            const std::function<bool(void)> real_termination_check_fn = [&](void) { if (!solution_found) { return termination_check_fn(); } else {return true;} };
            const std::function<void(std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t)> dummy_goal_reached_callback_fn = [](std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t) {;};
            // Call the planner
            std::pair<std::vector<std::vector<T, Allocator>>, std::map<std::string, double>> planning_result = PlanMultiPath(start, nearest_neighbor_fn, real_goal_found_fn, dummy_goal_reached_callback_fn, sampling_fn, forward_propagation_fn, real_termination_check_fn);
            // Put together the return
            std::vector<T, Allocator> planned_path;
            if (planning_result.first.size() > 0)
            {
                planned_path = planning_result.first[0];
            }
            return std::pair<std::vector<T, Allocator>, std::map<std::string, double>>(planned_path, planning_result.second);
        }

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * nodes - a mutable vector of planner states, used internally to store the planner tree.
         *          This is provided to allow external use of the tree during and after planning.
         * start - starting configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * termination_check_fn - returns if the planner should terminate (for example, if it has exceeded time/space limits)
         *
         * Returns:
         * std::pair<path, statistics>
         * path - vector of states corresponding to the planned path
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        template<typename T, typename Allocator=std::allocator<T>>
        static std::pair<std::vector<T, Allocator>, std::map<std::string, double>> Plan(
                std::vector<SimpleRRTPlannerState<T, Allocator>>& nodes,
                const T& start,
                const std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn,
                const std::function<bool(const T&)>& goal_reached_fn,
                const std::function<T(void)>& sampling_fn,
                const std::function<std::vector<std::pair<T, int64_t>>(const T&, const T&)>& forward_propagation_fn,
                const std::function<bool(void)>& termination_check_fn)
        {
            // Define a couple lambdas to let us use the generic multi-path planner as if it were a single-path planner
            bool solution_found = false;
            const std::function<bool(const T&)> real_goal_found_fn = [&](const T& state) { if (goal_reached_fn(state)) { solution_found = true; return true; } else {return false;} };
            const std::function<bool(void)> real_termination_check_fn = [&](void) { if (!solution_found) { return termination_check_fn(); } else {return true;} };
            const std::function<void(std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t)> dummy_goal_reached_callback_fn = [](std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t) {;};
            // Call the planner
            std::pair<std::vector<std::vector<T, Allocator>>, std::map<std::string, double>> planning_result = PlanMultiPath(nodes, start, nearest_neighbor_fn, real_goal_found_fn, dummy_goal_reached_callback_fn, sampling_fn, forward_propagation_fn, real_termination_check_fn);
            // Put together the return
            std::vector<T, Allocator> planned_path;
            if (planning_result.first.size() > 0)
            {
                planned_path = planning_result.first[0];
            }
            return std::pair<std::vector<T, Allocator>, std::map<std::string, double>>(planned_path, planning_result.second);
        }

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * start - starting configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * termination_check_fn - returns if the planner should terminate (for example, if it has exceeded time/space limits)
         *
         * Returns:
         * std::pair<paths, statistics>
         * paths - vector of vector of states corresponding to the planned path(s)
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        template<typename T, typename Allocator=std::allocator<T>>
        static std::pair<std::vector<std::vector<T, Allocator>>, std::map<std::string, double>> PlanMultiPath(
                const T& start,
                const std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn,
                const std::function<bool(const T&)>& goal_reached_fn,
                const std::function<void(std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t)>& goal_reached_callback_fn,
                const std::function<T(void)>& sampling_fn,
                const std::function<std::vector<std::pair<T, int64_t>>(const T&, const T&)>& forward_propagation_fn,
                const std::function<bool(void)>& termination_check_fn)
        {
            // Keep track of states
            std::vector<SimpleRRTPlannerState<T, Allocator>> nodes;
            return PlanMultiPath(nodes, start, nearest_neighbor_fn, goal_reached_fn, goal_reached_callback_fn, sampling_fn, forward_propagation_fn, termination_check_fn);
        }

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * nodes - a mutable vector of planner states, used internally to store the planner tree.
         *          This is provided to allow external use of the tree during and after planning.
         * start - starting configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * termination_check_fn - returns if the planner should terminate (for example, if it has exceeded time/space limits)
         *
         * Returns:
         * std::pair<paths, statistics>
         * paths - vector of vector of states corresponding to the planned path(s)
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        template<typename T, typename Allocator=std::allocator<T>>
        static std::pair<std::vector<std::vector<T, Allocator>>, std::map<std::string, double>> PlanMultiPath(
                std::vector<SimpleRRTPlannerState<T, Allocator>>& nodes,
                const T& start,
                const std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn,
                const std::function<bool(const T&)>& goal_reached_fn,
                const std::function<void(std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t)>& goal_reached_callback_fn,
                const std::function<T(void)>& sampling_fn,
                const std::function<std::vector<std::pair<T, int64_t>>(const T&, const T&)>& forward_propagation_fn,
                const std::function<bool(void)>& termination_check_fn)
        {
            // Clear the tree we've been given
            nodes.clear();
            // Add the start state
            nodes.emplace_back(SimpleRRTPlannerState<T, Allocator>(start));
            // Call the planner
            return PlanMultiPath(nodes, nearest_neighbor_fn, goal_reached_fn, goal_reached_callback_fn, sampling_fn, forward_propagation_fn, termination_check_fn);
        }

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * nodes - a mutable vector of planner states, used internally to store the planner tree.
         *          This is provided to allow external use of the tree during and after planning.
         *          This contains either a SINGLE start state, or the tree resulting from previous planning.
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * termination_check_fn - returns if the planner should terminate (for example, if it has exceeded time/space limits)
         *
         * Returns:
         * std::pair<paths, statistics>
         * paths - vector of vector of states corresponding to the planned path(s)
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        template<typename T, typename Allocator=std::allocator<T>>
        static std::pair<std::vector<std::vector<T, Allocator>>, std::map<std::string, double>> PlanMultiPath(
                std::vector<SimpleRRTPlannerState<T, Allocator>>& nodes,
                const std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn,
                const std::function<bool(const T&)>& goal_reached_fn,
                const std::function<void(std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t)>& goal_reached_callback_fn,
                const std::function<T(void)>& sampling_fn,
                const std::function<std::vector<std::pair<T, int64_t>>(const T&, const T&)>& forward_propagation_fn,
                const std::function<bool(void)>& termination_check_fn)
        {
            // Make a dummy state added function
            const std::function<void(std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t)> dummy_state_added_fn = [] (std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t) { ; };
            // Call the planner
            return PlanMultiPath(nodes, nearest_neighbor_fn, dummy_state_added_fn, goal_reached_fn, goal_reached_callback_fn, sampling_fn, forward_propagation_fn, termination_check_fn);
        }

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * nodes - a mutable vector of planner states, used internally to store the planner tree.
         *          This is provided to allow external use of the tree during and after planning.
         *          This contains either a SINGLE start state, or the tree resulting from previous planning.
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * state_added_fn - callback function that takes (parent, child) for each extension
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * termination_check_fn - returns if the planner should terminate (for example, if it has exceeded time/space limits)
         *
         * Returns:
         * std::pair<paths, statistics>
         * paths - vector of vector of states corresponding to the planned path(s)
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        template<typename T, typename Allocator=std::allocator<T>>
        static std::pair<std::vector<std::vector<T, Allocator>>, std::map<std::string, double>> PlanMultiPath(
                std::vector<SimpleRRTPlannerState<StateType, StateAllocator>>& tree,
                const std::function<int64_t(const std::vector<SimpleRRTPlannerState<StateType, StateAllocator>>&, const SampleType&)>& nearest_neighbor_fn,
                const std::function<void(std::vector<SimpleRRTPlannerState<StateType, StateAllocator>>&, const int64_t)>& state_added_fn,
                const std::function<bool(const StateType&)>& goal_reached_fn,
                const std::function<void(std::vector<SimpleRRTPlannerState<StateType, StateAllocator>>&, const int64_t)>& goal_reached_callback_fn,
                const std::function<SampleType(void)>& sampling_fn,
                const std::function<std::vector<std::pair<StateType, int64_t>>(const StateType&, const SampleType&)>& forward_propagation_fn,
                const std::function<bool(void)>& termination_check_fn)
        {
            // Make sure we've been given a start state
            if (tree.empty())
            {
                throw std::invalid_argument("Must be called with at least one node in tree");
            }
            // Make sure the tree is properly linked
            if(CheckTreeLinkage(tree) == false)
            {
                throw std::invalid_argument("Provided tree has invalid linkage");
            }
            // Keep track of statistics
            std::map<std::string, double> statistics;
            statistics["total_samples"] = 0.0;
            statistics["successful_samples"] = 0.0;
            statistics["failed_samples"] = 0.0;
            // Storage for the goal states we reach
            std::vector<int64_t> goal_state_indices;
            // Safety check before doing real work
            for (size_t idx = 0; idx < tree.size(); idx++)
            {
                if (goal_reached_fn(tree[idx].GetValueImmutable()))
                {
                    std::cerr << "Starting node " << idx << " meets goal conditions, adding to goal states" << std::endl;
                    goal_state_indices.push_back((int64_t)idx);
                    goal_reached_callback_fn(tree, (int64_t)idx);
                }
            }
            // Update the start time
            const std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
            // Plan
            while (!termination_check_fn())
            {
                // Sample a random goal
                const StateType random_target = sampling_fn();
                // Get the nearest neighbor
                const int64_t nearest_neighbor_index = nearest_neighbor_fn(nodes, random_target);
                if (unlikely(nearest_neighbor_index < 0))
                {
                    break;
                }
                const T& nearest_neighbor = nodes.at(nearest_neighbor_index).GetValueImmutable();
                // Forward propagate towards the goal
                std::vector<std::pair<StateType, int64_t>> propagated = forward_propagation_fn(nearest_neighbor, random_target);
                if (!propagated.empty())
                {
                    statistics["total_samples"] += 1.0;
                    statistics["successful_samples"] += 1.0;
                    for (size_t idx = 0; idx < propagated.size(); idx++)
                    {
                        const std::pair<StateType, int64_t>& current_propagation = propagated[idx];
                        // Determine the parent index of the new state
                        // This process deserves some explanation
                        // The "current relative parent index" is the index of the parent, relative to the list of propagated nodes.
                        // A negative value means the nearest neighbor in the tree, zero means the first propagated node, and so on.
                        // NOTE - the relative parent index *must* be lower than the index in the list of prograted nodes
                        // i.e. the first node must have a negative value, and so on.
                        const int64_t& current_relative_parent_index = current_propagation.second;
                        int64_t node_parent_index = nearest_neighbor_index;
                        if (current_relative_parent_index >= 0)
                        {
                            const int64_t current_relative_index = (int64_t)idx;
                            if (current_relative_parent_index >= current_relative_index)
                            {
                                throw std::invalid_argument("Linkage with relative parent index >= current relative index is invalid");
                            }
                            const int64_t current_relative_offset = current_relative_parent_index - current_relative_index;
                            const int64_t current_nodes_size = (int64_t)tree.size();
                            node_parent_index = current_nodes_size + current_relative_offset; // Offset is negative!
                        }
                        else
                        {
                            node_parent_index = nearest_neighbor_index; // Negative relative parent index means our parent index is the nearest neighbor index
                        }
                        // Build the new state
                        const T& current_propagated = current_propagation.first;
                        const SimpleRRTPlannerState<T, Allocator> new_state(current_propagated, node_parent_index);
                        // Add the state to the tree
                        nodes.push_back(new_state);
                        const int64_t new_node_index = (int64_t)nodes.size() - 1;
                        nodes[node_parent_index].AddChildIndex(new_node_index);
                        // Call the state added callback
                        state_added_fn(tree, new_node_index);
                        // Check if we've reached the goal
                        if (goal_reached_fn(tree[new_node_index].GetValueImmutable()))
                        {
                            goal_state_indices.push_back(new_node_index);
                            goal_reached_callback_fn(tree, new_node_index);
                        }
                    }
                }
                else
                {
                    statistics["total_samples"] += 1.0;
                    statistics["failed_samples"] += 1.0;
                }
            }
            // Put together the results
            // Make sure the tree is properly linked
            if(CheckTreeLinkage(tree) == false)
            {
                throw std::runtime_error("Tree linkage was corrupted during planning");
            }
            const std::vector<std::vector<StateType, StateAllocator>> planned_paths = ExtractSolutionPaths(tree, goal_state_indices);
            const std::chrono::time_point<std::chrono::steady_clock> cur_time = std::chrono::steady_clock::now();
            const std::chrono::duration<double> planning_time(cur_time - start_time);
            statistics["planning_time"] = planning_time.count();
            statistics["total_states"] = tree.size();
            statistics["solutions"] = (double)planned_paths.size();
            return std::pair<std::vector<std::vector<StateType, StateAllocator>>, std::map<std::string, double>>(planned_paths, statistics);
        }

        /* Checks the planner tree to make sure the parent-child linkages are correct
         */
        template<typename T, typename Allocator=std::allocator<T>>
        static bool CheckTreeLinkage(const std::vector<SimpleRRTPlannerState<T, Allocator>>& nodes)
        {
            // Step through each state in the tree. Make sure that the linkage to the parent and child states are correct
            for (size_t current_index = 0; current_index < nodes.size(); current_index++)
            {
                // For every state, make sure all the parent<->child linkages are valid
                const SimpleRRTPlannerState<T, Allocator>& current_state = nodes[current_index];
                if (!current_state.IsInitialized())
                {
                    std::cerr << "Tree contains uninitialized node(s) " << current_index << std::endl;
                    return false;
                }
                // Check the linkage to the parent state
                const int64_t parent_index = current_state.GetParentIndex();
                if ((parent_index >= 0) && (parent_index < (int64_t)nodes.size()))
                {
                    if (parent_index != (int64_t)current_index)
                    {
                        const SimpleRRTPlannerState<T, Allocator>& parent_state = nodes[parent_index];
                        if (!parent_state.IsInitialized())
                        {
                            std::cerr << "Tree contains uninitialized node(s) " << parent_index << std::endl;
                            return false;
                        }
                        // Make sure the corresponding parent contains the current node in the list of child indices
                        const std::vector<int64_t>& parent_child_indices = parent_state.GetChildIndices();
                        auto index_found = std::find(parent_child_indices.begin(), parent_child_indices.end(), (int64_t)current_index);
                        if (index_found == parent_child_indices.end())
                        {
                            std::cerr << "Parent state " << parent_index << " does not contain child index for current node " << current_index << std::endl;
                            return false;
                        }
                    }
                    else
                    {
                        std::cerr << "Invalid parent index " << parent_index << " for state " << current_index << " [Indices can't be the same]" << std::endl;
                        return false;
                    }
                }
                else if (parent_index < -1)
                {
                    std::cerr << "Invalid parent index " << parent_index << " for state " << current_index << std::endl;
                    return false;
                }
                // Check the linkage to the child states
                const std::vector<int64_t>& current_child_indices = current_state.GetChildIndices();
                for (size_t idx = 0; idx < current_child_indices.size(); idx++)
                {
                    // Get the current child index
                    const int64_t current_child_index = current_child_indices[idx];
                    if ((current_child_index > 0) && (current_child_index < (int64_t)nodes.size()))
                    {
                        if (current_child_index != (int64_t)current_index)
                        {
                            const SimpleRRTPlannerState<T, Allocator>& child_state = nodes[current_child_index];
                            if (!child_state.IsInitialized())
                            {
                                std::cerr << "Tree contains uninitialized node(s) " << current_child_index << std::endl;
                                return false;
                            }
                            // Make sure the child node points to us as the parent index
                            const int64_t child_parent_index = child_state.GetParentIndex();
                            if (child_parent_index != (int64_t)current_index)
                            {
                                std::cerr << "Parent index " << child_parent_index << " for current child state " << current_child_index << " does not match index " << current_index << " for current node " << std::endl;
                                return false;
                            }
                        }
                        else
                        {
                            std::cerr << "Invalid child index " << current_child_index << " for state " << current_index << " [Indices can't be the same]" << std::endl;
                            return false;
                        }
                    }
                    else
                    {
                        std::cerr << "Invalid child index " << current_child_index << " for state " << current_index << std::endl;
                        return false;
                    }
                }
            }
            return true;
        }

        /* Extracts all the solution paths corresponding to the provided goal states
         */
        template<typename T, typename Allocator=std::allocator<T>>
        static std::vector<std::vector<T, Allocator>> ExtractSolutionPaths(
                const std::vector<SimpleRRTPlannerState<T, Allocator>>& nodes,
                const std::vector<int64_t>& goal_state_indices)
        {
            std::vector<std::vector<T, Allocator>> solution_paths;
            for (size_t idx = 0; idx < goal_state_indices.size(); idx++)
            {
                std::vector<T, Allocator> solution_path = ExtractSolutionPath(nodes, goal_state_indices[idx]);
                solution_paths.push_back(solution_path);
            }
            return solution_paths;
        }

        /* Extracts a single solution path corresponding to the provided goal state
         */
        template<typename T, typename Allocator=std::allocator<T>>
        static std::vector<T, Allocator> ExtractSolutionPath(
                const std::vector<SimpleRRTPlannerState<T, Allocator>>& nodes,
                const int64_t goal_state_index)
        {
            std::vector<T, Allocator> solution_path;
            const SimpleRRTPlannerState<T, Allocator>& goal_state = nodes[goal_state_index];
            solution_path.push_back(goal_state.GetValueImmutable());
            int64_t parent_index = goal_state.GetParentIndex();
            while (parent_index >= 0)
            {
                const SimpleRRTPlannerState<T, Allocator>& parent_state = nodes.at(parent_index);
                const T& parent = parent_state.GetValueImmutable();
                solution_path.push_back(parent);
                parent_index = parent_state.GetParentIndex();
            }
            // Put it in the right order
            std::reverse(solution_path.begin(), solution_path.end());
            return solution_path;
        }
    };

    class SimpleHybridBiRRTPlanner
    {
    private:

        SimpleHybridBiRRTPlanner() {}

    public:

        template<typename RNG, typename T, typename Allocator=std::allocator<T>>
        static std::pair<std::vector<T, Allocator>, std::map<std::string, double>> Plan(
                const T& start,
                const T& goal,
                const std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&,const T&)>& nearest_neighbor_fn,
                const std::function<bool(const T&, const T&)>& states_connected_fn,
                const std::function<T(void)>& state_sampling_fn,
                const std::function<std::vector<std::pair<T, int64_t>>(const T&, const T&)>& forward_propagation_fn,
                const double tree_sampling_bias,
                const double p_switch_tree,
                const std::chrono::duration<double>& time_limit,
                RNG& rng)
        {
            const std::function<void(std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t)> dummy_state_added_fn =
                    [] (std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t) { ; };
            const std::function<void(std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t, std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t, const bool)> dummy_goal_bridge_callback_fn =
                    [] (std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t, std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t, const bool) { ; };
            const std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
            const std::function<bool(void)> termination_check_fn =
                    [&](void) { return (((std::chrono::time_point<std::chrono::steady_clock>)std::chrono::steady_clock::now() - start_time) > time_limit); };
            bool solution_found = false;
            const std::function<bool(const T&, const T&)> real_states_connected_fn =
                    [&](const T& state1, const T& state2) { if (states_connected_fn(state1, state2)) { solution_found = true; return true; } else {return false;} };
            const std::function<bool(void)> real_termination_check_fn =
                    [&](void) { if (!solution_found) { return termination_check_fn(); } else {return true;} };
            const std::pair<std::vector<std::vector<T, Allocator>>, std::map<std::string, double>> planning_result =
                    PlanMultiPath(
                        start,
                        goal,
                        nearest_neighbor_fn,
                        dummy_state_added_fn,
                        real_states_connected_fn,
                        dummy_goal_bridge_callback_fn,
                        state_sampling_fn,
                        forward_propagation_fn,
                        tree_sampling_bias,
                        p_switch_tree,
                        real_termination_check_fn,
                        rng);
            // Put together the return
            std::vector<T, Allocator> planned_path;
            if (planning_result.first.size() > 0)
            {
                planned_path = planning_result.first[0];
            }
            return std::pair<std::vector<T, Allocator>, std::map<std::string, double>>(planned_path, planning_result.second);
        }

        template<typename RNG, typename T, typename Allocator=std::allocator<T>>
        static std::pair<std::vector<std::vector<T, Allocator>>, std::map<std::string, double>> PlanMultiPath(
                const T& start,
                const T& goal,
                const std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&,const T&)>& nearest_neighbor_fn,
                const std::function<bool(const T&, const T&)>& states_connected_fn,
                const std::function<T(void)>& state_sampling_fn,
                const std::function<std::vector<std::pair<T, int64_t>>(const T&, const T&)>& forward_propagation_fn,
                const double tree_sampling_bias,
                const double p_switch_tree,
                const std::function<bool(void)>& termination_check_fn,
                RNG& rng)
        {
            const std::function<void(std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t)> dummy_state_added_fn =
                    [] (std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t) { ; };
            const std::function<void(std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t, std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t, const bool)> dummy_goal_bridge_callback_fn =
                    [] (std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t, std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t, const bool) { ; };
            return PlanMultiPath(
                        start,
                        goal,
                        nearest_neighbor_fn,
                        dummy_state_added_fn,
                        states_connected_fn,
                        dummy_goal_bridge_callback_fn,
                        state_sampling_fn,
                        forward_propagation_fn,
                        tree_sampling_bias,
                        p_switch_tree,
                        termination_check_fn,
                        rng);
        }

        /* Template-based bidirectional RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * start - starting configuration
         * goal - target configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * states_connected_fn - return if a the given states meet conditions to connect the trees (for example, within a radius)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * tree_sampling_bias - in (0, 1), selects the probability that the new sampled state is a state in the other tree
         * time_limit - limit, in seconds, for the runtime of the planner
         * rng - a random number generator matching the interface of the generators provided by std::random
         *
         * Returns:
         * std::pair<path, statistics>
         * path - vector of states corresponding to the planned path
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        template<typename T, typename Allocator=std::allocator<T>>
        static std::pair<std::vector<std::vector<T>>, std::map<std::string, double>> PlanMultiPath(
                const T& start,
                const T& goal,
                const std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&,const T&)>& nearest_neighbor_fn,
                const std::function<void(std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t)>& state_added_fn,
                const std::function<bool(const T&, const T&)>& states_connected_fn,
                const std::function<void(std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t, std::vector<SimpleRRTPlannerState<T, Allocator>>&, const int64_t, const bool)>& goal_bridge_callback_fn,
                const std::function<T(void)>& state_sampling_fn,
                const std::function<std::vector<std::pair<T, int64_t>>(const T&, const T&)>& forward_propagation_fn,
                const double tree_sampling_bias,
                const double p_switch_tree,
                const std::function<bool(void)>& termination_check_fn,
                RNG& rng)
        {
            // Initialize the trees
            std::vector<SimpleRRTPlannerState<T, Allocator>> start_tree;
            start_tree.emplace_back(SimpleRRTPlannerState<T, Allocator>(start));
            std::vector<SimpleRRTPlannerState<T, Allocator>> goal_tree;
            goal_tree.emplace_back(SimpleRRTPlannerState<T, Allocator>(goal));
            return PlanMultiPath(
                        start_tree,
                        goal_tree,
                        nearest_neighbor_fn,
                        state_added_fn,
                        states_connected_fn,
                        goal_bridge_callback_fn,
                        state_sampling_fn,
                        forward_propagation_fn,
                        tree_sampling_bias,
                        p_switch_tree,
                        termination_check_fn,
                        rng);
        }

        template<typename RNG, typename StateType, typename Allocator=std::allocator<StateType>>
        static std::pair<std::vector<std::vector<StateType, Allocator>>, std::map<std::string, double>> PlanMultiPath(
            std::vector<SimpleRRTPlannerState<StateType, Allocator>>& start_tree,
            std::vector<SimpleRRTPlannerState<StateType, Allocator>>& goal_tree,
            const std::function<int64_t(const std::vector<SimpleRRTPlannerState<StateType, Allocator>>&,const StateType&)>& nearest_neighbor_fn,
            const std::function<void(std::vector<SimpleRRTPlannerState<StateType, Allocator>>&, const int64_t)>& state_added_fn,
            const std::function<bool(const StateType&, const StateType&)>& states_connected_fn,
            const std::function<void(std::vector<SimpleRRTPlannerState<StateType, Allocator>>&, const int64_t, std::vector<SimpleRRTPlannerState<StateType, Allocator>>&, const int64_t, const bool)>& goal_bridge_callback_fn,
            const std::function<StateType(void)>& state_sampling_fn,
            const std::function<std::vector<std::pair<StateType, int64_t>>(const StateType&, const StateType&)>& forward_propagation_fn,
            const double tree_sampling_bias,
            const double p_switch_tree,
            const std::function<bool(void)>& termination_check_fn,
            RNG& rng)
        {
            if (start_tree.empty())
            {
                throw std::invalid_argument("Must be called with at least one node in start tree");
            }
            if (goal_tree.empty())
            {
                throw std::invalid_argument("Must be called with at least one node in goal tree");
            }
            if (SimpleHybridRRTPlanner::CheckTreeLinkage(start_tree) == false)
            {
                throw std::invalid_argument("Provided start tree has invalid linkage");
            }
            if (SimpleHybridRRTPlanner::CheckTreeLinkage(goal_tree) == false)
            {
                throw std::invalid_argument("Provided goal tree has invalid linkage");
            }
            // Keep track of the "goal bridges" between the trees
            std::vector<std::pair<int64_t, int64_t>> goal_bridges;
            // Keep track of the active treee
            bool start_tree_active = true;
            // Distribution to control sampling type
            std::uniform_real_distribution<double> unit_real_distribution(0.0, 1.0);
            // Keep track of statistics
            std::map<std::string, double> statistics;
            statistics["total_samples"] = 0.0;
            statistics["successful_samples"] = 0.0;
            statistics["failed_samples"] = 0.0;
            statistics["active_tree_swaps"] = 0.0;
            // Safety check before doing real work
            for (size_t start_tree_idx = 0; start_tree_idx < start_tree.size(); start_tree_idx++)
            {
                for (size_t goal_tree_idx = 0; goal_tree_idx < goal_tree.size(); goal_tree_idx++)
                {
                    if (states_connected_fn(start_tree[start_tree_idx].GetValueImmutable(), goal_tree[goal_tree_idx].GetValueImmutable()))
                    {
                        std::cerr << "Starting pair (" << start_tree_idx << ", " << goal_tree_idx << ") meets goal conditions, adding to goal states" << std::endl;
                        goal_bridges.push_back(std::pair<int64_t, int64_t>((int64_t)start_tree_idx, (int64_t)goal_tree_idx));
                        goal_bridge_callback_fn(start_tree, start_tree_idx, goal_tree, goal_tree_idx, true);
                    }
                }
            }
            // Update the start time
            std::chrono::time_point<std::chrono::steady_clock> start_time = std::chrono::steady_clock::now();
            // Plan
            while (!termination_check_fn())
            {
                // Get the current active/target trees
                std::vector<SimpleRRTPlannerState<StateType, Allocator>>& active_tree = (start_tree_active) ? start_tree : goal_tree;
                std::vector<SimpleRRTPlannerState<StateType, Allocator>>& target_tree = (start_tree_active) ? goal_tree : start_tree;
                // Select our sampling type
                const bool sample_from_tree = (unit_real_distribution(rng) <= tree_sampling_bias);
                int64_t target_tree_node_index = -1;
                if (sample_from_tree)
                {
                    std::uniform_int_distribution<int64_t> tree_sampling_distribution(0, (int64_t)target_tree.size() - 1);
                    target_tree_node_index = tree_sampling_distribution(rng);
                }
                // Sample a target state
                const StateType target_state = (sample_from_tree) ? target_tree.at(target_tree_node_index).GetValueImmutable() : state_sampling_fn();
                // Get the nearest neighbor
                const int64_t nearest_neighbor_index = nearest_neighbor_fn(active_tree, target_state);
                if (unlikely(nearest_neighbor_index < 0))
                {
                    break;
                }
                const StateType& nearest_neighbor = active_tree.at(nearest_neighbor_index).GetValueImmutable();
                // Forward propagate towards the goal
                std::vector<std::pair<StateType, int64_t>> propagated = forward_propagation_fn(nearest_neighbor, target_state);
                if (!propagated.empty())
                {
                    statistics["total_samples"] += 1.0;
                    statistics["successful_samples"] += 1.0;
                    for (size_t idx = 0; idx < propagated.size(); idx++)
                    {
                        const std::pair<StateType, int64_t>& current_propagation = propagated[idx];
                        // Determine the parent index of the new state
                        // This process deserves some explanation
                        // The "current relative parent index" is the index of the parent, relative to the list of propagated nodes.
                        // A negative value means the nearest neighbor in the tree, zero means the first propagated node, and so on.
                        // NOTE - the relative parent index *must* be lower than the index in the list of prograted nodes
                        // i.e. the first node must have a negative value, and so on.
                        const int64_t& current_relative_parent_index = current_propagation.second;
                        int64_t node_parent_index = nearest_neighbor_index;
                        if (current_relative_parent_index >= 0)
                        {
                            const int64_t current_relative_index = (int64_t)idx;
                            if (current_relative_parent_index >= current_relative_index)
                            {
                                throw std::invalid_argument("Linkage with relative parent index >= current relative index is invalid");
                            }
                            const int64_t current_relative_offset = current_relative_parent_index - current_relative_index;
                            const int64_t current_nodes_size = (int64_t)active_tree.size();
                            node_parent_index = current_nodes_size + current_relative_offset; // Offset is negative!
                        }
                        else
                        {
                            node_parent_index = nearest_neighbor_index; // Negative relative parent index means our parent index is the nearest neighbor index
                        }
                        // Build the new state
                        const StateType& current_propagated = current_propagation.first;
                        SimpleRRTPlannerState<StateType, Allocator> new_state(current_propagated, node_parent_index);
                        // Add the state to the tree
                        active_tree.push_back(new_state);
                        const int64_t new_node_index = (int64_t)active_tree.size() - 1;
                        active_tree[node_parent_index].AddChildIndex(new_node_index);
                        // Call the state added callback
                        state_added_fn(active_tree, new_node_index);
                        // If we sampled from the other tree
                        if (sample_from_tree)
                        {
                            // Check if we have connected the trees
                            if (states_connected_fn(active_tree[new_node_index].GetValueImmutable(), target_state))
                            {
                                if (start_tree_active)
                                {
                                    goal_bridges.push_back(std::pair<int64_t, int64_t>(new_node_index, target_tree_node_index));
                                    goal_bridge_callback_fn(active_tree, new_node_index, target_tree, target_tree_node_index, start_tree_active);
                                }
                                else
                                {
                                    goal_bridges.push_back(std::pair<int64_t, int64_t>(target_tree_node_index, new_node_index));
                                    goal_bridge_callback_fn(target_tree, target_tree_node_index, active_tree, new_node_index, start_tree_active);
                                }
                            }
                        }
                    }
                }
                else
                {
                    statistics["total_samples"] += 1.0;
                    statistics["failed_samples"] += 1.0;
                }
                // Decide if we should switch the active tree
                if (unit_real_distribution(rng) <= p_switch_tree)
                {
                    start_tree_active = !start_tree_active;
                    statistics["active_tree_swaps"] += 1.0;
                }
            }
            // Put together the results
            if (SimpleHybridRRTPlanner::CheckTreeLinkage(start_tree) == false)
            {
                throw std::runtime_error("Start tree linkage was corrupted during planning");
            }
            if (SimpleHybridRRTPlanner::CheckTreeLinkage(goal_tree) == false)
            {
                throw std::runtime_error("Goal tree linkage was corrupted during planning");
            }
            std::vector<std::vector<StateType, Allocator>> planned_paths;
            // Extract the solution paths
            for (size_t goal_bridge_idx = 0; goal_bridge_idx < goal_bridges.size(); goal_bridge_idx++)
            {
                const std::pair<int64_t, int64_t>& goal_bridge = goal_bridges[goal_bridge_idx];
                // Extract the portion in the start tree
                std::vector<StateType, Allocator> start_path = SimpleHybridRRTPlanner::ExtractSolutionPath(start_tree, goal_bridge.first);
                // Extract the portion in the goal tree
                std::vector<StateType, Allocator> goal_path = SimpleHybridRRTPlanner::ExtractSolutionPath(goal_tree, goal_bridge.second);
                // Reverse the goal tree part
                std::reverse(goal_path.begin(), goal_path.end());
                // Combine
                start_path.insert(start_path.end(), goal_path.begin(), goal_path.end());
                planned_paths.push_back(start_path);
            }
            const std::chrono::time_point<std::chrono::steady_clock> cur_time = std::chrono::steady_clock::now();
            const std::chrono::duration<double> planning_time(cur_time - start_time);
            statistics["planning_time"] = planning_time.count();
            statistics["total_states"] = (double)(start_tree.size() + goal_tree.size());
            statistics["solutions"] = (double)planned_paths.size();
            return std::pair<std::vector<std::vector<StateType, Allocator>>, std::map<std::string, double>>(planned_paths, statistics);
        }
    };

}

#endif // SIMPLE_RRT_PLANNER
