#include "victor_hardware_interface/minimal_arm_wrapper_interface.hpp"

// ROS
#include <ros/callback_queue.h>

// ROS message headers
#include "victor_hardware_interface/ControlModeParameters.h"
#include "victor_hardware_interface/MotionCommand.h"
#include "victor_hardware_interface/MotionStatus.h"
#include "victor_hardware_interface/Robotiq3FingerCommand.h"
#include "victor_hardware_interface/Robotiq3FingerStatus.h"
#include "victor_hardware_interface/SetControlMode.h"
#include "victor_hardware_interface/GetControlMode.h"

// LCM
#include "victor_hardware_interface/iiwa_hardware_interface.hpp"
#include "victor_hardware_interface/robotiq_3finger_hardware_interface.hpp"

namespace victor_hardware_interface
{
    inline ControlModeParameters mergeControlModeParameters(
            const ControlModeParameters& active_control_mode,
            const ControlModeParameters& new_control_mode);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Helpers to test if two messages are equivalent
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    bool jvqEqual(const JointValueQuantity& jvq1, const JointValueQuantity& jvq2)
    {
        if (jvq1.joint_1 != jvq2.joint_1)
        {
            return false;
        }
        else if (jvq1.joint_2 != jvq2.joint_2)
        {
            return false;
        }
        else if (jvq1.joint_3 != jvq2.joint_3)
        {
            return false;
        }
        else if (jvq1.joint_4 != jvq2.joint_4)
        {
            return false;
        }
        else if (jvq1.joint_5 != jvq2.joint_5)
        {
            return false;
        }
        else if (jvq1.joint_6 != jvq2.joint_6)
        {
            return false;
        }
        else if (jvq1.joint_7 != jvq2.joint_7)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    bool cvqEqual(const CartesianValueQuantity& cvq1, const CartesianValueQuantity& cvq2)
    {
        if (cvq1.x != cvq2.x)
        {
            return false;
        }
        else if (cvq1.y != cvq2.y)
        {
            return false;
        }
        else if (cvq1.z != cvq2.z)
        {
            return false;
        }
        else if (cvq1.a != cvq2.a)
        {
            return false;
        }
        else if (cvq1.b != cvq2.b)
        {
            return false;
        }
        else if (cvq1.c != cvq2.c)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    bool jointPexpEqual(const JointPathExecutionParameters& pexp1, const JointPathExecutionParameters& pexp2)
    {
        if (pexp1.joint_relative_acceleration != pexp2.joint_relative_acceleration)
        {
            return false;
        }
        else if (pexp1.joint_relative_velocity != pexp2.joint_relative_velocity)
        {
            return false;
        }
        else if (pexp1.override_joint_acceleration != pexp2.override_joint_acceleration)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    bool cartesianPexpEqual(const CartesianPathExecutionParameters& pexp1, const CartesianPathExecutionParameters& pexp2)
    {
        if (cvqEqual(pexp1.max_velocity, pexp2.max_velocity) == false)
        {
            return false;
        }
        else if (cvqEqual(pexp1.max_acceleration, pexp2.max_acceleration) == false)
        {
            return false;
        }
        else if (pexp1.max_nullspace_velocity != pexp2.max_nullspace_velocity)
        {
            return false;
        }
        else if (pexp1.max_nullspace_acceleration != pexp2.max_nullspace_acceleration)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    bool controlModeParamsEqual(const ControlModeParameters& params1, const ControlModeParameters& params2)
    {
        // Control mode parameter
        const bool cm_equal    = (params1.control_mode.mode == params2.control_mode.mode);

        // Path Execuition mode parameters
        const bool jpexp_equal = jointPexpEqual(params1.joint_path_execution_params, params2.joint_path_execution_params);
        const bool cpexp_equal = cartesianPexpEqual(params1.cartesian_path_execution_params, params2.cartesian_path_execution_params);

        // Joint Impedance mode parameters
        const bool jd_equal    = jvqEqual(params1.joint_impedance_params.joint_damping, params2.joint_impedance_params.joint_damping);
        const bool js_equal    = jvqEqual(params1.joint_impedance_params.joint_stiffness, params2.joint_impedance_params.joint_stiffness);

        // Cartesian Impedance mode parameters
        const bool cd_equal    = cvqEqual(params1.cartesian_impedance_params.cartesian_damping, params2.cartesian_impedance_params.cartesian_damping);
        const bool nd_equal    = (params1.cartesian_impedance_params.nullspace_damping == params2.cartesian_impedance_params.nullspace_damping);
        const bool cs_equal    = cvqEqual(params1.cartesian_impedance_params.cartesian_stiffness, params2.cartesian_impedance_params.cartesian_stiffness);
        const bool ns_equal    = (params1.cartesian_impedance_params.nullspace_stiffness == params2.cartesian_impedance_params.nullspace_stiffness);

        // Cartesian control mode limits parameters
        const bool mpd_equal   = cvqEqual(params1.cartesian_control_mode_limits.max_path_deviation, params2.cartesian_control_mode_limits.max_path_deviation);
        const bool mcv_equal   = cvqEqual(params1.cartesian_control_mode_limits.max_cartesian_velocity, params2.cartesian_control_mode_limits.max_cartesian_velocity);
        const bool mcf_equal   = cvqEqual(params1.cartesian_control_mode_limits.max_control_force, params2.cartesian_control_mode_limits.max_control_force);
        const bool smcf_equal  = (params1.cartesian_control_mode_limits.stop_on_max_control_force == params2.cartesian_control_mode_limits.stop_on_max_control_force);

        if (cm_equal &&
            jpexp_equal && cpexp_equal &&
            jd_equal && js_equal &&
            cd_equal && nd_equal && cs_equal && ns_equal &&
            mpd_equal && mcv_equal && mcf_equal && smcf_equal)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Constructor and single function called externally
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    MinimalArmWrapperInterface::MinimalArmWrapperInterface(
            ros::NodeHandle& nh,
            const std::shared_ptr<lcm::LCM>& send_lcm_ptr,
            const std::shared_ptr<lcm::LCM>& recv_lcm_ptr,
            const std::string& cartesian_control_frame,
            const double set_control_mode_timeout,
            // ROS Topics
            const std::string& motion_command_topic,
            const std::string& motion_status_topic,
            const std::string& control_mode_status_topic,
            const std::string& get_control_mode_service,
            const std::string& set_control_mode_service,
            const std::string& gripper_command_topic,
            const std::string& gripper_status_topic,
            // LCM channels
            const std::string& motion_command_channel,
            const std::string& motion_status_channel,
            const std::string& control_mode_command_channel,
            const std::string& control_mode_status_channel,
            const std::string& gripper_command_channel,
            const std::string& gripper_status_channel)
        : nh_(nh)
        , cartesian_control_frame_(cartesian_control_frame)
        , set_control_mode_timeout_(set_control_mode_timeout)
        , send_lcm_ptr_(send_lcm_ptr)
        , recv_lcm_ptr_(recv_lcm_ptr)
    {
        // Verify that the control frame is a valid ROS TF name (or at least not empty)
        // TODO: Are there any other limits on TF frame names other than "non-empty"? see Issue #28
        if (cartesian_control_frame_ == "")
        {
            throw std::invalid_argument("Cartesian control frame [""] is not valid");
        }

        // Set up IIWA LCM interface
        const auto motion_status_callback_fn = [&] (const MotionStatus& motion_status)
        {
            return motionStatusLCMCallback(motion_status);
        };
        const auto control_mode_status_callback_fn = [&] (const ControlModeParameters& control_mode_status)
        {
            return controlModeStatusLCMCallback(control_mode_status);
        };
        iiwa_ptr_ = std::unique_ptr<IIWAHardwareInterface>(new IIWAHardwareInterface(
                        send_lcm_ptr_, recv_lcm_ptr_,
                        motion_command_channel, motion_status_channel, motion_status_callback_fn,
                        control_mode_command_channel, control_mode_status_channel, control_mode_status_callback_fn));

        // Set up Robotiq LCM interface
        const auto gripper_status_callback_fn = [&] (const Robotiq3FingerStatus& gripper_status)
        {
            return gripperStatusLCMCallback(gripper_status);
        };
        robotiq_ptr_ = std::unique_ptr<Robotiq3FingerHardwareInterface>(
                    new Robotiq3FingerHardwareInterface(
                        send_lcm_ptr_, recv_lcm_ptr_,
                        gripper_command_channel, gripper_status_channel, gripper_status_callback_fn));

        // Set up ROS interfaces
        nh_.setCallbackQueue(&ros_callback_queue_);
        motion_status_pub_ = nh_.advertise<MotionStatus>(motion_status_topic, 1, false);
        control_mode_status_pub_ = nh_.advertise<ControlModeParameters>(control_mode_status_topic, 1, false);
        gripper_status_pub_ = nh_.advertise<Robotiq3FingerStatus>(gripper_status_topic, 1, false);
        motion_command_sub_ = nh_.subscribe(motion_command_topic, 1, &MinimalArmWrapperInterface::motionCommandROSCallback, this);
        gripper_command_sub_ = nh_.subscribe(gripper_command_topic, 1, &MinimalArmWrapperInterface::gripperCommandROSCallback, this);;
        set_control_mode_server_ = nh_.advertiseService(set_control_mode_service, &MinimalArmWrapperInterface::setControlModeCallback, this);
        get_control_mode_server_ = nh_.advertiseService(get_control_mode_service, &MinimalArmWrapperInterface::getControlModeCallback, this);
    }

    // The one function called externally
    void MinimalArmWrapperInterface::spin()
    {
        // Start ROS thread - this must happen *after* the LCM objects have been initialized as they use iiwa_ptr_ and
        // robotiq_ptr_, so we do so here instead of in the constructor
        ROS_INFO_NAMED(ros::this_node::getName(), "Starting ROS spin loop.");
        auto ros_callback_thread = std::thread(std::bind(&MinimalArmWrapperInterface::rosSpinThread, this));

        ROS_INFO_NAMED(ros::this_node::getName(), "Starting LCM spin loop.");
        bool lcm_ok = true;
        // Continue to loop so long as both ROS and LCM have no un-recoverable errors or SIGINT style interruptions
        while (ros::ok() && lcm_ok)
        {
            const int ret = recv_lcm_ptr_->handleTimeout(LCM_HANDLE_TIMEOUT);
            if (ret < 0)
            {
                lcm_ok = false;
                ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), "LCM error: " << ret);
            }
        }

        ros::shutdown();
        ros_callback_thread.join();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Internal helper function used to process ROS callbacks
    void MinimalArmWrapperInterface::rosSpinThread()
    {
        const ros::WallDuration timeout(ROS_SPIN_PERIOD);
        while (nh_.ok())
        {
            ros_callback_queue_.callAvailable(timeout);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Get/Set Control Mode functionality
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //// Static helpers to check parameters for being in valid ranges for the control mode /////////////////////////

    std::pair<bool, std::string> MinimalArmWrapperInterface::validateJointPathExecutionParams(
            const JointPathExecutionParameters& params)
    {
        bool valid = true;
        std::string message;
        if (params.joint_relative_velocity <= 0.0 || params.joint_relative_velocity > 1.0)
        {
            valid = false;
            message += "+Invalid joint relative velocity";
        }
        if (params.joint_relative_acceleration <= 0.0 || params.joint_relative_acceleration > 1.0)
        {
            valid = false;
            message += "+Invalid joint relative acceleration";
        }
        if (params.override_joint_acceleration < 0.0 || params.override_joint_acceleration > 10.0)
        {
            valid = false;
            message += "+Invalid override joint acceleration";
        }
        return std::make_pair(valid, message);
    }

    std::pair<bool, std::string> MinimalArmWrapperInterface::validateCartesianPathExecutionParams(
            const CartesianPathExecutionParameters& params)
    {
        bool valid = true;
        std::string message;

        // Velocities, mm/s, rad/s, and 1/s respectively
        if (params.max_velocity.x <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF X max velocity";
        }
        if (params.max_velocity.y <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Y max velocity";
        }
        if (params.max_velocity.z <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Z max velocity";
        }
        if (params.max_velocity.a <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF A max velocity";
        }
        if (params.max_velocity.b <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF B max velocity";
        }
        if (params.max_velocity.c <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF C max velocity";
        }
        if (params.max_nullspace_velocity <= 0.0)
        {
            valid = false;
            message += "+Invalid nullspace max velocity";
        }

        // Accelerations, mm/s^2, rad/s^2, and 1/s^2 respectively
        if (params.max_acceleration.x <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF X max acceleration";
        }
        if (params.max_acceleration.y <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Y max acceleration";
        }
        if (params.max_acceleration.z <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Z max acceleration";
        }
        if (params.max_acceleration.a <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF A max acceleration";
        }
        if (params.max_acceleration.b <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF B max acceleration";
        }
        if (params.max_acceleration.c <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF C max acceleration";
        }
        if (params.max_nullspace_acceleration <= 0.0)
        {
            valid = false;
            message += "+Invalid nullspace max acceleration";
        }

        return std::make_pair(valid, message);
    }

    std::pair<bool, std::string> MinimalArmWrapperInterface::validateJointImpedanceParams(
            const JointImpedanceParameters& params)
    {
        bool valid = true;
        std::string message;

        // Joint damping - unitless
        if (params.joint_damping.joint_1 < 0.0 || params.joint_damping.joint_1 > 1.0)
        {
            valid = false;
            message += "+Invalid joint 1 damping";
        }
        if (params.joint_damping.joint_2 < 0.0 || params.joint_damping.joint_2 > 1.0)
        {
            valid = false;
            message += "+Invalid joint 2 damping";
        }
        if (params.joint_damping.joint_3 < 0.0 || params.joint_damping.joint_3 > 1.0)
        {
            valid = false;
            message += "+Invalid joint 3 damping";
        }
        if (params.joint_damping.joint_4 < 0.0 || params.joint_damping.joint_4 > 1.0)
        {
            valid = false;
            message += "+Invalid joint 4 damping";
        }
        if (params.joint_damping.joint_5 < 0.0 || params.joint_damping.joint_5 > 1.0)
        {
            valid = false;
            message += "+Invalid joint 5 damping";
        }
        if (params.joint_damping.joint_6 < 0.0 || params.joint_damping.joint_6 > 1.0)
        {
            valid = false;
            message += "+Invalid joint 6 damping";
        }
        if (params.joint_damping.joint_7 < 0.0 || params.joint_damping.joint_7 > 1.0)
        {
            valid = false;
            message += "+Invalid joint 7 damping";
        }

        // Joint stiffness - Nm/rad
        if (params.joint_stiffness.joint_1 < 0.0)
        {
            valid = false;
            message += "+Invalid joint 1 stiffness";
        }
        if (params.joint_stiffness.joint_2 < 0.0)
        {
            valid = false;
            message += "+Invalid joint 2 stiffness";
        }
        if (params.joint_stiffness.joint_3 < 0.0)
        {
            valid = false;
            message += "+Invalid joint 3 stiffness";
        }
        if (params.joint_stiffness.joint_4 < 0.0)
        {
            valid = false;
            message += "+Invalid joint 4 stiffness";
        }
        if (params.joint_stiffness.joint_5 < 0.0)
        {
            valid = false;
            message += "+Invalid joint 5 stiffness";
        }
        if (params.joint_stiffness.joint_6 < 0.0)
        {
            valid = false;
            message += "+Invalid joint 6 stiffness";
        }
        if (params.joint_stiffness.joint_7 < 0.0)
        {
            valid = false;
            message += "+Invalid joint 7 stiffness";
        }

        return std::make_pair(valid, message);
    }

    std::pair<bool, std::string> MinimalArmWrapperInterface::validateCartesianImpedanceParams(
            const CartesianImpedanceParameters& params)
    {
        bool valid = true;
        std::string message;

        // Damping - unitless
        if (params.cartesian_damping.x < 0.1 || params.cartesian_damping.x > 1.0)
        {
            valid = false;
            message += "+Invalid DoF X damping";
        }
        if (params.cartesian_damping.y < 0.1 || params.cartesian_damping.y > 1.0)
        {
            valid = false;
            message += "+Invalid DoF Y damping";
        }
        if (params.cartesian_damping.z < 0.1 || params.cartesian_damping.z > 1.0)
        {
            valid = false;
            message += "+Invalid DoF Z damping";
        }
        if (params.cartesian_damping.a < 0.1 || params.cartesian_damping.a > 1.0)
        {
            valid = false;
            message += "+Invalid DoF A damping";
        }
        if (params.cartesian_damping.b < 0.1 || params.cartesian_damping.b > 1.0)
        {
            valid = false;
            message += "+Invalid DoF B damping";
        }
        if (params.cartesian_damping.c < 0.1 || params.cartesian_damping.c > 1.0)
        {
            valid = false;
            message += "+Invalid DoF C damping";
        }
        if (params.nullspace_damping < 0.3 || params.nullspace_damping > 1.0)
        {
            valid = false;
            message += "+Invalid nullspace damping";
        }

        // Stiffness - units N/m, Nm/rad, no idea for nullspace
        if (params.cartesian_stiffness.x < 0.0 || params.cartesian_stiffness.x > 5000.0)
        {
            valid = false;
            message += "+Invalid DoF X stiffness";
        }
        if (params.cartesian_stiffness.y < 0.0 || params.cartesian_stiffness.y > 5000.0)
        {
            valid = false;
            message += "+Invalid DoF Y stiffness";
        }
        if (params.cartesian_stiffness.z < 0.0 || params.cartesian_stiffness.z > 5000.0)
        {
            valid = false;
            message += "+Invalid DoF Z stiffness";
        }
        // TODO: original values set by Calder were < 0.1 and > 300.0 - documentation states < 0.0 and > 300.0; why was it set to 0.1?
        if (params.cartesian_stiffness.a < 0.0 || params.cartesian_stiffness.a > 300.0)
        {
            valid = false;
            message += "+Invalid DoF A stiffness";
        }
        if (params.cartesian_stiffness.b < 0.0 || params.cartesian_stiffness.b > 300.0)
        {
            valid = false;
            message += "+Invalid DoF B stiffness";
        }
        if (params.cartesian_stiffness.c < 0.0 || params.cartesian_stiffness.c > 300.0)
        {
            valid = false;
            message += "+Invalid DoF C stiffness";
        }
        if (params.nullspace_stiffness < 0.0)
        {
            valid = false;
            message += "+Invalid nullspace stiffness";
        }

        return std::make_pair(valid, message);
    }

    std::pair<bool, std::string> MinimalArmWrapperInterface::validateCartesianControlModeLimits(
            const CartesianControlModeLimits& params)
    {
        bool valid = true;
        std::string message;

        // Path deviation
        if (params.max_path_deviation.x <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF X max path deviation";
        }
        if (params.max_path_deviation.y <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Y max path deviation";
        }
        if (params.max_path_deviation.z <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Z max path deviation";
        }
        if (params.max_path_deviation.a <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF A max path deviation";
        }
        if (params.max_path_deviation.b <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF B max path deviation";
        }
        if (params.max_path_deviation.c <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF C max path deviation";
        }

        // Cartesian velocity
        if (params.max_cartesian_velocity.x <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF X max cartesian velocity";
        }
        if (params.max_cartesian_velocity.y <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Y max cartesian velocity";
        }
        if (params.max_cartesian_velocity.z <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Z max cartesian velocity";
        }
        if (params.max_cartesian_velocity.a <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF A max cartesian velocity";
        }
        if (params.max_cartesian_velocity.b <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF B max cartesian velocity";
        }
        if (params.max_cartesian_velocity.c <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF C max cartesian velocity";
        }

        // Cartesian force
        if (params.max_control_force.x <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF X max control force";
        }
        if (params.max_control_force.y <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Y max control force";
        }
        if (params.max_control_force.z <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF Z max control force";
        }
        if (params.max_control_force.a <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF A max control force";
        }
        if (params.max_control_force.b <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF B max control force";
        }
        if (params.max_control_force.c <= 0.0)
        {
            valid = false;
            message += "+Invalid DoF C max control force";
        }

        return std::make_pair(valid, message);
    }

    std::pair<bool, std::string> MinimalArmWrapperInterface::validateControlMode(const ControlModeParameters& params)
    {
        bool valid = true;
        std::string message;

        // Check the control mode itself
        if (params.control_mode.mode != ControlMode::JOINT_POSITION &&
            params.control_mode.mode != ControlMode::JOINT_IMPEDANCE &&
            params.control_mode.mode != ControlMode::CARTESIAN_POSE &&
            params.control_mode.mode != ControlMode::CARTESIAN_IMPEDANCE)
        {
            valid = false;
            message += "+Invalid control mode";
        }

        // Check each part of the control mode
        const auto valid_joint_impedance_params          = validateJointImpedanceParams(params.joint_impedance_params);
        const auto valid_cartesian_impedance_params      = validateCartesianImpedanceParams(params.cartesian_impedance_params);
        const auto valid_cartesian_control_mode_limits   = validateCartesianControlModeLimits(params.cartesian_control_mode_limits);
        const auto valid_joint_path_execution_params     = validateJointPathExecutionParams(params.joint_path_execution_params);
        const auto valid_cartesian_path_execution_params = validateCartesianPathExecutionParams(params.cartesian_path_execution_params);

        // Aggregate the results
        valid &= valid_joint_impedance_params.first &&
                 valid_cartesian_impedance_params.first &&
                 valid_cartesian_control_mode_limits.first &&
                 valid_joint_path_execution_params.first &&
                 valid_cartesian_path_execution_params.first;

        message += valid_joint_impedance_params.second +
                   valid_cartesian_impedance_params.second +
                   valid_cartesian_control_mode_limits.second +
                   valid_joint_path_execution_params.second +
                   valid_cartesian_path_execution_params.second;

        return std::make_pair(valid, message);
    }

    //// ROS Callbacks to get and set the control mode as service calls ////////////////////////////////////////////

    bool MinimalArmWrapperInterface::setControlModeCallback(
            SetControlMode::Request& req,
            SetControlMode::Response& res)
    {
        Maybe::Maybe<ControlModeParameters> local_active_control_mode_copy;
        {
            std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
            local_active_control_mode_copy = active_control_mode_;
        }
        if (local_active_control_mode_copy.Valid())
        {
            const auto merged_command = mergeControlModeParameters(local_active_control_mode_copy.GetImmutable(), req.new_control_mode);
            const auto validity_check = validateControlMode(merged_command);
            if (validity_check.first)
            {
                iiwa_ptr_->SendControlModeCommandMessage(merged_command);

                // Loop waiting for a matching control mode to be parsed
                bool control_mode_matches = false;

                const std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
                std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();

                do
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    end_time = std::chrono::steady_clock::now();
                    std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
                    control_mode_matches = controlModeParamsEqual(merged_command, active_control_mode_.GetImmutable());
                }
                while (!control_mode_matches && std::chrono::duration<double>(end_time - start_time).count() < set_control_mode_timeout_);

                // Check the results of the timeout
                if (control_mode_matches)
                {
                    res.success = true;
                    res.message = "Control mode set successfully";
                }
                else
                {
                    res.success = false;
                    res.message = "Control mode could not be set in Sunrise within the timeout window of " + std::to_string(set_control_mode_timeout_);
                }
            }
            else
            {
                res.success = false;
                res.message = validity_check.second;
            }
        }
        else
        {
            res.success = false;
            res.message = "No initial control mode available from the controller";
        }
        return true;
    }

    bool MinimalArmWrapperInterface::getControlModeCallback(
            GetControlMode::Request& req,
            GetControlMode::Response& res)
    {
        UNUSED(req);
        std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
        res.has_active_control_mode = active_control_mode_.Valid();
        if (res.has_active_control_mode)
        {
            res.active_control_mode = active_control_mode_.GetImmutable();
        }
        return true;
    }

    /*
     * Callback function used by the LCM subsystem when a control_mode_status message is received. Caches the value
     * in active_control_mode_ for use by setControlModeCallback(...) and getControlModeCallback(...), as well as publishes
     * the message on the correct ROS topic
     */
    void MinimalArmWrapperInterface::controlModeStatusLCMCallback(
            const ControlModeParameters& control_mode_status)
    {
        {
            std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
            if (active_control_mode_.Valid() == false)
            {
                ROS_INFO_NAMED(ros::this_node::getName(), "Initializing active_control_mode for the first time");
            }
            active_control_mode_ = control_mode_status;
        }
        control_mode_status_pub_.publish(control_mode_status);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Arm movement/control and feedback functionality
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //// Static helpers to check parameters for being in valid ranges for the motion command ///////////////////////

    std::pair<bool, std::string> MinimalArmWrapperInterface::validateJointPositions(
            const JointValueQuantity& positions)
    {
        //TODO: The function is not implemented yet; it should check that the commanded joint positions are within the joint limits of the arm
        UNUSED(positions);
        return std::make_pair(true, std::string(""));
    }

    std::pair<bool, std::string> MinimalArmWrapperInterface::validateCartesianPose(
            const geometry_msgs::Pose& pose,
            const std::string& frame) const
    {
        bool valid = true;
        std::string message;

        // Check to make sure the frame is correct
        if (frame != cartesian_control_frame_)
        {
            valid = false;
            message += "+Commanded cartesian pose has the wrong frame, " + frame + " given, " + cartesian_control_frame_ + " expected";
        }

        // Check to make sure the quaternion is well formed
        {
            const double quat_squared_norm = (pose.orientation.w * pose.orientation.w)
                                             + (pose.orientation.x * pose.orientation.x)
                                             + (pose.orientation.y * pose.orientation.y)
                                             + (pose.orientation.z * pose.orientation.z);
            const double error = std::fabs(1.0 - quat_squared_norm);
            if (error > 1e-6)
            {
                valid = false;
                message += "+Commanded cartesian pose quaternion is not normalized, squared norm = " + std::to_string(quat_squared_norm);
            }
        }
        return std::make_pair(valid, message);
    }

    std::pair<bool, std::string>MinimalArmWrapperInterface::validateMotionCommand(
            const MotionCommand& command) const
    {
        Maybe::Maybe<ControlModeParameters> active_control_mode_cached;
        {
            std::lock_guard<std::mutex> lock(control_mode_status_mutex_);
            active_control_mode_cached = active_control_mode_;
        }
        if (active_control_mode_cached.Valid())
        {
            const uint8_t active_control_mode = active_control_mode_cached.GetImmutable().control_mode.mode;
            const uint8_t command_motion_mode = command.control_mode.mode;

            // Note that this assumes the two messages use the same enums for each item, this is asserted in the constructor
            if (active_control_mode != command_motion_mode)
            {
                return std::make_pair(false, std::string("Active control mode does not match commanded control mode"));
            }

            switch (command_motion_mode)
            {
                case ControlMode::JOINT_POSITION:
                case ControlMode::JOINT_IMPEDANCE:
                    return validateJointPositions(command.joint_position);

                case ControlMode::CARTESIAN_POSE:
                case ControlMode::CARTESIAN_IMPEDANCE:
                    return validateCartesianPose(command.cartesian_pose, command.header.frame_id);

                default:
                    return std::make_pair(false, std::string("Invalid commanded control mode. This should not be possible"));;
            }
        }
        else
        {
            return std::make_pair(false, std::string("No active control mode, cannot command motion"));
        }
    }

    /*
     * ROS callback to parse a arm motion command, and pass it along to the LCM subsystem
     */
    void MinimalArmWrapperInterface::motionCommandROSCallback(const MotionCommand& command)
    {
        const auto validity_check_results = validateMotionCommand(command);
        if (validity_check_results.first)
        {
            iiwa_ptr_->SendMotionCommandMessage(command);
        }
        else
        {
            ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), "Arm motion command failed validity checks: " << validity_check_results.second);
        }
    }

    /*
     * LCM callback used by the LCM subsystem when a LCM status message is recieved. Republishes the motion status
     * on the correct ROS topic
     */
    void MinimalArmWrapperInterface::motionStatusLCMCallback(const MotionStatus& motion_status)
    {
        motion_status_pub_.publish(motion_status);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Gripper movement/control and feedback functionality
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //// Static helpers to check parameters for being in valid ranges for the gripper command //////////////////////

    std::pair<bool, std::string> MinimalArmWrapperInterface::validateFingerCommand(
            const Robotiq3FingerActuatorCommand& command)
    {
        bool valid = true;
        std::string message;
        if (command.position > 1.0 || command.position < 0.0)
        {
            valid = false;
            message += "+Invalid finger position";
        }
        
        if (command.force > 1.0 || command.force < 0.0)
        {
            valid = false;
            message += "+Invalid finger force";
        }
        
        if (command.speed > 1.0 || command.speed < 0.0)
        {
            valid = false;
            message += "+Invalid finger speed";
        }
        
        return std::make_pair(valid, message);
    }

    std::pair<bool, std::string> MinimalArmWrapperInterface::validateGripperCommand(
            const Robotiq3FingerCommand& command)
    {
        const auto ac = validateFingerCommand(command.finger_a_command);
        const auto bc = validateFingerCommand(command.finger_b_command);
        const auto cc = validateFingerCommand(command.finger_c_command);
        const auto sc = validateFingerCommand(command.scissor_command);

        const bool valid = ac.first && bc.first && cc.first && sc.first;
        const std::string message = ac.second + bc.second + cc.second + sc.second;
        return std::make_pair(valid, message);
    }

    /*
     * ROS callback to parse a gripper motion command, and pass it along to the LCM subsystem
     */
    void MinimalArmWrapperInterface::gripperCommandROSCallback(
            const Robotiq3FingerCommand& command)
    {
        const auto validity_check_results = validateGripperCommand(command);
        if (validity_check_results.first)
        {
            robotiq_ptr_->sendCommandMessage(command);
        }
        else
        {
            ROS_WARN_STREAM_NAMED(ros::this_node::getName(), "Gripper command failed validity checks: " << validity_check_results.second);
        }
    }

    /*
     * LCM callback used by the LCM subsystem when a LCM status message is recieved. Republishes the motion status
     * on the correct ROS topic
     */
    void MinimalArmWrapperInterface::gripperStatusLCMCallback(
            const Robotiq3FingerStatus& gripper_status)
    {
        gripper_status_pub_.publish(gripper_status);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Internal helpers
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    inline bool jointPathExecutionParamsIsDefault(const JointPathExecutionParameters& params)
    {
        return (params.joint_relative_velocity == 0 && 
                params.joint_relative_acceleration == 0 && 
                params.override_joint_acceleration == 0);
    }

    inline bool cartesianPathExecutionParamsIsDefault(const CartesianPathExecutionParameters& params)
    {
        return (params.max_velocity.x == 0 && params.max_velocity.y == 0 && 
                params.max_velocity.z == 0 && params.max_velocity.a == 0 && 
                params.max_velocity.b == 0 && params.max_velocity.c == 0 &&
                params.max_acceleration.x == 0 && params.max_acceleration.y == 0 &&
                params.max_acceleration.z == 0 && params.max_acceleration.a == 0 && 
                params.max_acceleration.b == 0 && params.max_acceleration.c == 0 &&
                params.max_nullspace_velocity == 0 && params.max_nullspace_acceleration == 0);
    }

    inline bool jointImpedanceParamsIsDefault(const JointImpedanceParameters& params)
    {
        return (params.joint_stiffness.joint_1 == 0 && params.joint_stiffness.joint_2 == 0 && 
                params.joint_stiffness.joint_3 == 0 && params.joint_stiffness.joint_4 == 0 &&
                params.joint_stiffness.joint_5 == 0 && params.joint_stiffness.joint_6 == 0 && 
                params.joint_stiffness.joint_7 == 0 &&
                params.joint_damping.joint_1 == 0 && params.joint_damping.joint_2 == 0 && 
                params.joint_damping.joint_3 == 0 && params.joint_damping.joint_4 == 0 &&
                params.joint_damping.joint_5 == 0 && params.joint_damping.joint_6 == 0 && 
                params.joint_damping.joint_7 == 0);
    }

    inline bool cartesianImpedanceParamsIsDefault(const CartesianImpedanceParameters& params)
    {
        return (params.cartesian_stiffness.x == 0 && params.cartesian_stiffness.y == 0 && 
                params.cartesian_stiffness.z == 0 && params.cartesian_stiffness.a == 0 && 
                params.cartesian_stiffness.b == 0 && params.cartesian_stiffness.c == 0 &&
                params.cartesian_damping.x == 0 && params.cartesian_damping.y == 0 && 
                params.cartesian_damping.z == 0 && params.cartesian_damping.a == 0 && 
                params.cartesian_damping.b == 0 && params.cartesian_damping.c == 0 &&
                params.nullspace_stiffness == 0 && params.nullspace_damping == 0);
    }

    inline bool cartesianControlModeLimitsIsDefault(const CartesianControlModeLimits& params)
    {
        return (params.max_path_deviation.x == 0 && params.max_path_deviation.y == 0 && 
                params.max_path_deviation.z == 0 && params.max_path_deviation.a == 0 && 
                params.max_path_deviation.b == 0 && params.max_path_deviation.c == 0 &&
                params.max_cartesian_velocity.x == 0 && params.max_cartesian_velocity.y == 0 && 
                params.max_cartesian_velocity.z == 0 && params.max_cartesian_velocity.a == 0 && 
                params.max_cartesian_velocity.b == 0 && params.max_cartesian_velocity.c == 0 &&
                params.max_control_force.x == 0 && params.max_control_force.y == 0 && 
                params.max_control_force.z == 0 && params.max_control_force.a == 0 && 
                params.max_control_force.b == 0 && params.max_control_force.c == 0 &&
                params.stop_on_max_control_force == false);
    }

    inline ControlModeParameters mergeControlModeParameters(
            const ControlModeParameters& active_control_mode,
            const ControlModeParameters& new_control_mode)
    {
        /***************************************************************************************************************
        This function is a helper function for the callback function of setting a new control mode(setControlModeCallBack).
        It copies the parameters of the old control mode to the new one, and updates relevant parameters with theparameters
        of the new control mode.
        
        Parameters updated in each control mode:
        JOINT_POSITION: joint_path_execution_params
        CARTESIAN_POSE: cartesian_path_execution_params
        JOINT_IMPEDANCE: joint_impedance_params, joint_path_execution_params
        CARTESIAN_IMPEDANCE: cartesian_impedance_params, cartesian_control_mode_limits, cartesian_path_execution_params
        ***************************************************************************************************************/

        ControlModeParameters merged_control_mode;
        // Copy the old over
        merged_control_mode.joint_path_execution_params = active_control_mode.joint_path_execution_params;
        merged_control_mode.joint_impedance_params = active_control_mode.joint_impedance_params;
        merged_control_mode.cartesian_impedance_params = active_control_mode.cartesian_impedance_params;
        merged_control_mode.cartesian_control_mode_limits = active_control_mode.cartesian_control_mode_limits;
        merged_control_mode.cartesian_path_execution_params = active_control_mode.cartesian_path_execution_params;
        // Copy manadatory members
        merged_control_mode.control_mode = new_control_mode.control_mode;
        // Copy mode-dependant members
        switch (new_control_mode.control_mode.mode)
        {
            case ControlMode::JOINT_IMPEDANCE:
                merged_control_mode.joint_path_execution_params = new_control_mode.joint_path_execution_params;
                merged_control_mode.joint_impedance_params = new_control_mode.joint_impedance_params;

                if (!cartesianImpedanceParamsIsDefault(new_control_mode.cartesian_impedance_params))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian impedance parameters are specified but ignored in JOINT_IMPEDANCE mode.");
                }
                if (!cartesianControlModeLimitsIsDefault(new_control_mode.cartesian_control_mode_limits))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian control mode limits are specified but ignored in JOINT_IMPEDANCE mode.");
                }
                if (!cartesianPathExecutionParamsIsDefault(new_control_mode.cartesian_path_execution_params))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian path execution parameters are specified but ignored in JOINT_IMPEDANCE mode.");
                }

                break;

            case ControlMode::CARTESIAN_IMPEDANCE:
                merged_control_mode.cartesian_impedance_params = new_control_mode.cartesian_impedance_params;
                merged_control_mode.cartesian_control_mode_limits = new_control_mode.cartesian_control_mode_limits;
                merged_control_mode.cartesian_path_execution_params = new_control_mode.cartesian_path_execution_params;

                if (!jointPathExecutionParamsIsDefault(new_control_mode.joint_path_execution_params))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "The joint path execution parameters are specified but ignored in CASRTESIAN_IMPEDANCE mode.");
                }
                if (!jointImpedanceParamsIsDefault(new_control_mode.joint_impedance_params))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "The joint impedance parameters are specified but ignored in CASRTESIAN_IMPEDANCE mode.");
                }

                break;

            case ControlMode::JOINT_POSITION:
                // From the new
                merged_control_mode.joint_path_execution_params = new_control_mode.joint_path_execution_params;
                
                if (!jointImpedanceParamsIsDefault(new_control_mode.joint_impedance_params))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "The joint impedance parameters are specified but ignored in JOINT_POSITION mode.");
                }
                if (!cartesianImpedanceParamsIsDefault(new_control_mode.cartesian_impedance_params))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian impedance parameters are specified but ignored in JOINT_POSITION mode.");
                }
                if (!cartesianControlModeLimitsIsDefault(new_control_mode.cartesian_control_mode_limits))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian control mode limits are specified but ignored in JOINT_POSITION mode.");
                }
                if (!cartesianPathExecutionParamsIsDefault(new_control_mode.cartesian_path_execution_params))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian path execution parameters are specified but ignored in JOINT_POSITION mode.");
                }

                break;

            case ControlMode::CARTESIAN_POSE:
                // From the new
                merged_control_mode.cartesian_path_execution_params = new_control_mode.cartesian_path_execution_params;

                if (!jointPathExecutionParamsIsDefault(new_control_mode.joint_path_execution_params))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "The joint path execution parameters are specified but ignored in CARTESIAN_POSE mode.");
                }
                if (!jointImpedanceParamsIsDefault(new_control_mode.joint_impedance_params))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "The joint impedance parameters are specified but ignored in CARTESIAN_POSE mode.");
                }
                if (!cartesianImpedanceParamsIsDefault(new_control_mode.cartesian_impedance_params))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian impedance parameters are specified but ignored in CARTESIAN_POSE mode.");
                }
                if (!cartesianControlModeLimitsIsDefault(new_control_mode.cartesian_control_mode_limits))
                {
                    ROS_WARN_NAMED(ros::this_node::getName(), "The cartesian control mode limits are specified but ignored in CARTESIAN_POSE mode.");
                }

                break;

            default:
                ROS_INFO_STREAM_NAMED(ros::this_node::getName(), "Invalid control mode: " << new_control_mode.control_mode << ".");
                assert(false);

                break;

        }

        return merged_control_mode;
    }
}
