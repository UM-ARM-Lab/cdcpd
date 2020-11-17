#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/math_helpers.hpp>
#include <victor_hardware_interface/iiwa_hardware_interface.hpp>
#include <victor_hardware_interface/robotiq_3finger_hardware_interface.hpp>
// ROS message headers
#include <victor_hardware_interface/ControlModeParameters.h>
#include <victor_hardware_interface/MotionCommand.h>
#include <victor_hardware_interface/MotionStatus.h>
#include <victor_hardware_interface/Robotiq3FingerCommand.h>
#include <victor_hardware_interface/Robotiq3FingerStatus.h>
// ROS
#include <ros/ros.h>
// LCM
#include <lcm/lcm-cpp.hpp>

#define RTT_CHECK_COUNT 10000

class LoopbackTester
{
protected:

    ros::NodeHandle nh_;
    std::shared_ptr<lcm::LCM> send_lcm_ptr_;
    std::shared_ptr<lcm::LCM> recv_lcm_ptr_;
    std::unique_ptr<victor_hardware_interface::IIWAHardwareInterface> iiwa_ptr_;
    std::unique_ptr<victor_hardware_interface::Robotiq3FingerHardwareInterface> robotiq_ptr_;

    std::vector<victor_hardware_interface::MotionCommand> motion_command_queue_;
    std::vector<victor_hardware_interface::MotionStatus> motion_status_queue_;
    std::vector<victor_hardware_interface::ControlModeParameters> control_mode_command_queue_;
    std::vector<victor_hardware_interface::ControlModeParameters> control_mode_status_queue_;
    std::vector<victor_hardware_interface::Robotiq3FingerCommand> gripper_command_queue_;
    std::vector<victor_hardware_interface::Robotiq3FingerStatus> gripper_status_queue_;

    std::vector<double> motion_command_rtt_;
    std::vector<double> control_mode_command_rtt_;
    std::vector<double> gripper_command_rtt_;

public:

    LoopbackTester(
            ros::NodeHandle& nh,
            const std::shared_ptr<lcm::LCM>& send_lcm_ptr,
            const std::shared_ptr<lcm::LCM>& recv_lcm_ptr,
            const std::string& motion_command_channel,
            const std::string& motion_status_channel,
            const std::string& control_mode_command_channel,
            const std::string& control_mode_status_channel,
            const std::string& gripper_command_channel,
            const std::string& gripper_status_channel)
        : nh_(nh), send_lcm_ptr_(send_lcm_ptr)
        , recv_lcm_ptr_(recv_lcm_ptr)
    {
        motion_command_rtt_.reserve(RTT_CHECK_COUNT);
        control_mode_command_rtt_.reserve(RTT_CHECK_COUNT);
        gripper_command_rtt_.reserve(RTT_CHECK_COUNT);
        //
        const auto motion_status_callback_fn = [&] (const victor_hardware_interface::MotionStatus& motion_status) { return MotionStatusCallback(motion_status); };
        const auto control_mode_status_callback_fn = [&] (const victor_hardware_interface::ControlModeParameters& control_mode_status) { return ControlModeStatusCallback(control_mode_status); };
        iiwa_ptr_ = std::unique_ptr<victor_hardware_interface::IIWAHardwareInterface>(new victor_hardware_interface::IIWAHardwareInterface(send_lcm_ptr_, recv_lcm_ptr_, motion_command_channel, motion_status_channel, motion_status_callback_fn, control_mode_command_channel, control_mode_status_channel, control_mode_status_callback_fn));
        const auto gripper_status_callback_fn = [&] (const victor_hardware_interface::Robotiq3FingerStatus& gripper_status) { return GripperStatusCallback(gripper_status); };
        robotiq_ptr_ = std::unique_ptr<victor_hardware_interface::Robotiq3FingerHardwareInterface>(new victor_hardware_interface::Robotiq3FingerHardwareInterface(send_lcm_ptr_, recv_lcm_ptr_, gripper_command_channel, gripper_status_channel, gripper_status_callback_fn));
    }

    void MotionStatusCallback(const victor_hardware_interface::MotionStatus& motion_status)
    {
        ROS_DEBUG_STREAM_NAMED(ros::this_node::getName(), "Got motion status " << motion_status);
        motion_status_queue_.push_back(motion_status);
    }

    void ControlModeStatusCallback(const victor_hardware_interface::ControlModeParameters& control_mode_status)
    {
        ROS_DEBUG_STREAM_NAMED(ros::this_node::getName(), "Got control mode status " << control_mode_status);
        control_mode_status_queue_.push_back(control_mode_status);
    }

    void GripperStatusCallback(const victor_hardware_interface::Robotiq3FingerStatus& gripper_status)
    {
        ROS_DEBUG_STREAM_NAMED(ros::this_node::getName(), "Got gripper status " << gripper_status);
        gripper_status_queue_.push_back(gripper_status);
    }

    static inline bool JVQMatch(const victor_hardware_interface::JointValueQuantity& jvq1, const victor_hardware_interface::JointValueQuantity& jvq2)
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

    static inline bool CVQMatch(const victor_hardware_interface::CartesianValueQuantity& cvq1, const victor_hardware_interface::CartesianValueQuantity& cvq2)
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

    static inline bool CartPoseMatch(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
    {
        if (pose1.position.x != pose2.position.x)
        {
            return false;
        }
        else if (pose1.position.y != pose2.position.y)
        {
            return false;
        }
        else if (pose1.position.z != pose2.position.z)
        {
            return false;
        }
        else if (pose1.orientation.w != pose2.orientation.w)
        {
            return false;
        }
        else if (pose1.orientation.x != pose2.orientation.x)
        {
            return false;
        }
        else if (pose1.orientation.y != pose2.orientation.y)
        {
            return false;
        }
        else if (pose1.orientation.z != pose2.orientation.z)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    static inline bool JointPExPMatch(const victor_hardware_interface::JointPathExecutionParameters& pexp1, const victor_hardware_interface::JointPathExecutionParameters& pexp2)
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

    static inline bool CartesianPExPMatch(const victor_hardware_interface::CartesianPathExecutionParameters& pexp1, const victor_hardware_interface::CartesianPathExecutionParameters& pexp2)
    {
        if (CVQMatch(pexp1.max_velocity, pexp2.max_velocity) == false)
        {
            return false;
        }
        else if (CVQMatch(pexp1.max_acceleration, pexp2.max_acceleration) == false)
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

    static inline victor_hardware_interface::JointValueQuantity MakeJVQ(const double j1, const double j2, const double j3, const double j4, const double j5, const double j6, const double j7)
    {
        victor_hardware_interface::JointValueQuantity jvq;
        jvq.joint_1 = j1;
        jvq.joint_2 = j2;
        jvq.joint_3 = j3;
        jvq.joint_4 = j4;
        jvq.joint_5 = j5;
        jvq.joint_6 = j6;
        jvq.joint_7 = j7;
        return jvq;
    }

    static inline victor_hardware_interface::CartesianValueQuantity MakeCVQ(const double x, const double y, const double z, const double a, const double b, const double c)
    {
        victor_hardware_interface::CartesianValueQuantity cvq;
        cvq.x = x;
        cvq.y = y;
        cvq.z = z;
        cvq.a = a;
        cvq.b = b;
        cvq.c = c;
        return cvq;
    }

    static inline victor_hardware_interface::JointPathExecutionParameters MakeJointPExP(const double jra, const double jrv, const double oja)
    {
        victor_hardware_interface::JointPathExecutionParameters pexp;
        pexp.joint_relative_acceleration = jra;
        pexp.joint_relative_velocity = jrv;
        pexp.override_joint_acceleration = oja;
        return pexp;
    }

    static inline victor_hardware_interface::Robotiq3FingerActuatorCommand MakeRobotiqActuatorCommand(const double p, const double s, const double f)
    {
        victor_hardware_interface::Robotiq3FingerActuatorCommand command;
        command.position = p;
        command.speed = s;
        command.force = f;
        return command;
    }

    void SendMotionCommand()
    {
        victor_hardware_interface::MotionCommand command;
        command.joint_position = MakeJVQ(0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875);
        command.joint_velocity = MakeJVQ(0.875, 0.75, 0.625, 0.5, 0.375, 0.25, 0.125);
        command.cartesian_pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(Eigen::Isometry3d::Identity());
        command.control_mode.mode = victor_hardware_interface::ControlMode::JOINT_POSITION;
        command.header.stamp = ros::Time::now();
        const bool sent = iiwa_ptr_->SendMotionCommandMessage(command);
        UNUSED(sent);
        assert(sent);
        motion_command_queue_.push_back(command);
    }

    bool CheckMotionCommandAndStatusMatch(const victor_hardware_interface::MotionCommand& command, const victor_hardware_interface::MotionStatus& status) const
    {
        const bool jpmatch = JVQMatch(command.joint_position, status.measured_joint_position);
        const bool jvmatch = JVQMatch(command.joint_velocity, status.measured_joint_velocity);
        const bool cpmatch = CartPoseMatch(command.cartesian_pose, status.measured_cartesian_pose);
        const bool ctmatch = (command.control_mode.mode == status.active_control_mode.mode);
        if (jpmatch && jvmatch && cpmatch && ctmatch)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void SendControlModeCommand()
    {
        victor_hardware_interface::ControlModeParameters command;
        command.joint_impedance_params.joint_damping = MakeJVQ(0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875);
        command.joint_impedance_params.joint_stiffness = MakeJVQ(0.875, 0.75, 0.625, 0.5, 0.375, 0.25, 0.125);
        command.cartesian_impedance_params.cartesian_damping = MakeCVQ(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        command.cartesian_impedance_params.cartesian_stiffness = MakeCVQ(7.0, 6.0, 5.0, 4.0, 3.0, 2.0);
        command.cartesian_impedance_params.nullspace_damping = 7.0;
        command.cartesian_impedance_params.nullspace_stiffness = 1.0;
        command.joint_path_execution_params = MakeJointPExP(1.0, 2.0, 3.0);
        command.cartesian_path_execution_params.max_velocity = MakeCVQ(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        command.cartesian_path_execution_params.max_acceleration = MakeCVQ(7.0, 6.0, 5.0, 4.0, 3.0, 2.0);
        command.cartesian_path_execution_params.max_nullspace_velocity = 7.0;
        command.cartesian_path_execution_params.max_nullspace_acceleration = 1.0;
        command.control_mode.mode = victor_hardware_interface::ControlMode::JOINT_POSITION;
        command.header.stamp = ros::Time::now();
        const bool sent = iiwa_ptr_->SendControlModeCommandMessage(command);
        UNUSED(sent);
        assert(sent);
        control_mode_command_queue_.push_back(command);
    }

    bool CheckControlModeCommandAndStatusMatch(const victor_hardware_interface::ControlModeParameters& command, const victor_hardware_interface::ControlModeParameters& status) const
    {
        const bool cdmatch = CVQMatch(command.cartesian_impedance_params.cartesian_damping, status.cartesian_impedance_params.cartesian_damping);
        const bool ndmatch = (command.cartesian_impedance_params.nullspace_damping == status.cartesian_impedance_params.nullspace_damping);
        const bool csmatch = CVQMatch(command.cartesian_impedance_params.cartesian_stiffness, status.cartesian_impedance_params.cartesian_stiffness);
        const bool nsmatch = (command.cartesian_impedance_params.nullspace_stiffness == status.cartesian_impedance_params.nullspace_stiffness);
        const bool jdmatch = JVQMatch(command.joint_impedance_params.joint_damping, status.joint_impedance_params.joint_damping);
        const bool jsmatch = JVQMatch(command.joint_impedance_params.joint_stiffness, status.joint_impedance_params.joint_stiffness);
        const bool jpexpmatch = JointPExPMatch(command.joint_path_execution_params, status.joint_path_execution_params);
        const bool cpexpmatch = CartesianPExPMatch(command.cartesian_path_execution_params, status.cartesian_path_execution_params);
        const bool cmmatch = (command.control_mode.mode == status.control_mode.mode);
        if (cdmatch && ndmatch && csmatch && nsmatch && jdmatch && jsmatch && jpexpmatch && cpexpmatch && cmmatch)
        {
            return true;
        }
        else
        {
            return true;
        }
    }

    void SendGripperCommand()
    {
        victor_hardware_interface::Robotiq3FingerCommand command;
        command.finger_a_command = MakeRobotiqActuatorCommand(0.0, 0.125, 0.25);
        command.finger_b_command = MakeRobotiqActuatorCommand(0.125, 0.25, 0.375);
        command.finger_c_command = MakeRobotiqActuatorCommand(0.25, 0.375, 0.5);
        command.scissor_command = MakeRobotiqActuatorCommand(0.375, 0.5, 0.625);
        command.header.stamp = ros::Time::now();
        command.finger_a_command.header.stamp = command.header.stamp;
        command.finger_b_command.header.stamp = command.header.stamp;
        command.finger_c_command.header.stamp = command.header.stamp;
        command.scissor_command.header.stamp = command.header.stamp;
        const bool sent = robotiq_ptr_->sendCommandMessage(command);
        UNUSED(sent);
        assert(sent);
        gripper_command_queue_.push_back(command);
    }

    static inline bool FingerActuatorCommandStatusTestMatch(const victor_hardware_interface::Robotiq3FingerActuatorCommand& command, const victor_hardware_interface::Robotiq3FingerActuatorStatus& status)
    {
        if (command.position != status.position_request)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    bool CheckGripperCommandAndStatusMatch(const victor_hardware_interface::Robotiq3FingerCommand& command, const victor_hardware_interface::Robotiq3FingerStatus& status) const
    {
        const bool famatch = FingerActuatorCommandStatusTestMatch(command.finger_a_command, status.finger_a_status);
        const bool fbmatch = FingerActuatorCommandStatusTestMatch(command.finger_b_command, status.finger_b_status);
        const bool fcmatch = FingerActuatorCommandStatusTestMatch(command.finger_c_command, status.finger_c_status);
        const bool smatch = FingerActuatorCommandStatusTestMatch(command.scissor_command, status.scissor_status);
        if (famatch && fbmatch && fcmatch && smatch)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    inline double GetRTT(const ros::Time& start, const ros::Time& end)
    {
        const ros::Duration rtt = end - start;
        return rtt.toSec();
    }

    void CheckMotionCommandAndStatus()
    {
        assert(motion_command_queue_.size() == 1);
        assert(motion_status_queue_.size() == 1);
        const bool match = CheckMotionCommandAndStatusMatch(motion_command_queue_.front(), motion_status_queue_.front());
        UNUSED(match);
        assert(match);
        motion_command_rtt_.push_back(GetRTT(motion_command_queue_.front().header.stamp, ros::Time::now()));
        motion_command_queue_.clear();
        motion_status_queue_.clear();
    }

    void CheckControlModeCommandAndStatus()
    {
        assert(control_mode_command_queue_.size() == 1);
        assert(control_mode_status_queue_.size() == 1);
        const bool match = CheckControlModeCommandAndStatusMatch(control_mode_command_queue_.front(), control_mode_status_queue_.front());
        UNUSED(match);
        assert(match);
        control_mode_command_rtt_.push_back(GetRTT(control_mode_command_queue_.front().header.stamp, ros::Time::now()));
        control_mode_command_queue_.clear();
        control_mode_status_queue_.clear();
    }

    void CheckGripperCommandAndStatus()
    {
        assert(gripper_command_queue_.size() == 1);
        assert(gripper_status_queue_.size() == 1);
        const bool match = CheckGripperCommandAndStatusMatch(gripper_command_queue_.front(), gripper_status_queue_.front());
        UNUSED(match);
        assert(match);
        gripper_command_rtt_.push_back(GetRTT(gripper_command_queue_.front().header.stamp, ros::Time::now()));
        gripper_command_queue_.clear();
        gripper_status_queue_.clear();
    }

    void Loop()
    {
        bool rtt_gathering = true;
        bool lcm_ok = true;
        while (ros::ok() && lcm_ok && rtt_gathering)
        {
            if (motion_command_queue_.empty() && motion_status_queue_.empty())
            {
                SendMotionCommand();
            }
            else if (!motion_command_queue_.empty() && !motion_status_queue_.empty())
            {
                CheckMotionCommandAndStatus();
            }
            else if (motion_command_queue_.empty() && !motion_status_queue_.empty())
            {
                ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), "Motion command queue " << motion_command_queue_.size() << " Motion status queue " << motion_status_queue_.size());
                break;
            }
            else
            {
                ; // Waiting for a response
            }
            if (control_mode_command_queue_.empty() && control_mode_status_queue_.empty())
            {
                SendControlModeCommand();
            }
            else if (!control_mode_command_queue_.empty() && !control_mode_status_queue_.empty())
            {
                CheckControlModeCommandAndStatus();
            }
            else if (control_mode_command_queue_.empty() && !control_mode_status_queue_.empty())
            {
                ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), "Control mode command queue " << control_mode_command_queue_.size() << " Control mode status queue " << control_mode_status_queue_.size());
                break;
            }
            else
            {
                ; // Waiting for a response
            }
            if (gripper_command_queue_.empty() && gripper_status_queue_.empty())
            {
                SendGripperCommand();
            }
            else if (!gripper_command_queue_.empty() && !gripper_status_queue_.empty())
            {
                CheckGripperCommandAndStatus();
            }
            else if (gripper_command_queue_.empty() && !gripper_status_queue_.empty())
            {
                ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), "Gripper command queue " << gripper_command_queue_.size() << " Gripper status queue " << gripper_status_queue_.size());
                break;
            }
            else
            {
                ; // Waiting for a response
            }
            // Run LCM callbacks
            bool lcm_running = true;
            while (lcm_running)
            {
                const int ret = recv_lcm_ptr_->handleTimeout(1);
                if (ret > 0)
                {
                    lcm_running = true;
                }
                else if (ret == 0)
                {
                    lcm_running = false;
                }
                else
                {
                    lcm_running = false;
                    lcm_ok = false;
                    ROS_ERROR_STREAM_NAMED(ros::this_node::getName(), "LCM error " << ret << " stopping loop");
                }
            }
            // Run ROS callbacks
            ros::spinOnce();
            if (motion_command_rtt_.size() >= RTT_CHECK_COUNT && control_mode_command_rtt_.size() >= RTT_CHECK_COUNT && gripper_command_rtt_.size() >= RTT_CHECK_COUNT)
            {
                rtt_gathering = false;
            }
        }
        // Show RTT stats
        const auto minmax_motion_command_rtt = std::minmax_element(motion_command_rtt_.begin(), motion_command_rtt_.end());
        const double min_motion_command_rtt = *minmax_motion_command_rtt.first;
        const double max_motion_command_rtt = *minmax_motion_command_rtt.second;
        const double avg_motion_command_rtt = EigenHelpers::AverageStdVectorDouble(motion_command_rtt_);
        const double stddev_motion_command_rtt = EigenHelpers::ComputeStdDevStdVectorDouble(motion_command_rtt_, avg_motion_command_rtt);
        ROS_INFO_NAMED(ros::this_node::getName(), "Motion command RTT: %f (min) %f (max) %f (avg) %f (stddev)", min_motion_command_rtt, max_motion_command_rtt, avg_motion_command_rtt, stddev_motion_command_rtt);
        const auto minmax_control_mode_command_rtt = std::minmax_element(control_mode_command_rtt_.begin(), control_mode_command_rtt_.end());
        const double min_control_mode_command_rtt = *minmax_control_mode_command_rtt.first;
        const double max_control_mode_command_rtt = *minmax_control_mode_command_rtt.second;
        const double avg_control_mode_command_rtt = EigenHelpers::AverageStdVectorDouble(control_mode_command_rtt_);
        const double stddev_control_mode_command_rtt = EigenHelpers::ComputeStdDevStdVectorDouble(control_mode_command_rtt_, avg_control_mode_command_rtt);
        ROS_INFO_NAMED(ros::this_node::getName(), "Control mode command RTT: %f (min) %f (max) %f (avg) %f (stddev)", min_control_mode_command_rtt, max_control_mode_command_rtt, avg_control_mode_command_rtt, stddev_control_mode_command_rtt);
        const auto minmax_gripper_command_rtt = std::minmax_element(gripper_command_rtt_.begin(), gripper_command_rtt_.end());
        const double min_gripper_command_rtt = *minmax_gripper_command_rtt.first;
        const double max_gripper_command_rtt = *minmax_gripper_command_rtt.second;
        const double avg_gripper_command_rtt = EigenHelpers::AverageStdVectorDouble(gripper_command_rtt_);
        const double stddev_gripper_command_rtt = EigenHelpers::ComputeStdDevStdVectorDouble(gripper_command_rtt_, avg_gripper_command_rtt);
        ROS_INFO_NAMED(ros::this_node::getName(), "Gripper command RTT: %f (min) %f (max) %f (avg) %f (stddev)", min_gripper_command_rtt, max_gripper_command_rtt, avg_gripper_command_rtt, stddev_gripper_command_rtt);
    }
};

int main(int argc, char** argv)
{
    // Default parameter values
    const std::string DEFAULT_SEND_LCM_URL("udp://10.10.10.11:30000");
    const std::string DEFAULT_RECV_LCM_URL("udp://10.10.10.99:30001");
    const std::string DEFAULT_MOTION_COMMAND_CHANNEL("motion_command");
    const std::string DEFAULT_MOTION_STATUS_CHANNEL("motion_status");
    const std::string DEFAULT_CONTROL_MODE_COMMAND_CHANNEL("control_mode_command");
    const std::string DEFAULT_CONTROL_MODE_STATUS_CHANNEL("control_mode_status");
    const std::string DEFAULT_GRIPPER_COMMAND_CHANNEL("gripper_command");
    const std::string DEFAULT_GRIPPER_STATUS_CHANNEL("gripper_status");
    // Start ROS
    ros::init(argc, argv, "loopback_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    const std::string send_lcm_url = nhp.param(std::string("send_lcm_url"), DEFAULT_SEND_LCM_URL);
    const std::string recv_lcm_url = nhp.param(std::string("recv_lcm_url"), DEFAULT_RECV_LCM_URL);
    const std::string motion_command_channel = nhp.param(std::string("motion_command_channel"), DEFAULT_MOTION_COMMAND_CHANNEL);
    const std::string motion_status_channel = nhp.param(std::string("motion_status_channel"), DEFAULT_MOTION_STATUS_CHANNEL);
    const std::string control_mode_command_channel = nhp.param(std::string("control_mode_command_channel"), DEFAULT_CONTROL_MODE_COMMAND_CHANNEL);
    const std::string control_mode_status_channel = nhp.param(std::string("control_mode_status_channel"), DEFAULT_CONTROL_MODE_STATUS_CHANNEL);
    const std::string gripper_command_channel = nhp.param(std::string("gripper_command_channel"), DEFAULT_GRIPPER_COMMAND_CHANNEL);
    const std::string gripper_status_channel = nhp.param(std::string("gripper_status_channel"), DEFAULT_GRIPPER_STATUS_CHANNEL);
    // Start LCM
    if (send_lcm_url == recv_lcm_url)
    {
        std::shared_ptr<lcm::LCM> lcm_ptr(new lcm::LCM(send_lcm_url));
        ROS_INFO("Starting Loopback Test Node with shared send/receive LCM...");
        LoopbackTester tester(nh, lcm_ptr, lcm_ptr, motion_command_channel, motion_status_channel, control_mode_command_channel, control_mode_status_channel, gripper_command_channel, gripper_status_channel);
        tester.Loop();
        return 0;
    }
    else
    {
        std::shared_ptr<lcm::LCM> send_lcm_ptr(new lcm::LCM(send_lcm_url));
        std::shared_ptr<lcm::LCM> recv_lcm_ptr(new lcm::LCM(recv_lcm_url));
        ROS_INFO("Starting Loopback Test Node with separate send and receive LCM...");
        LoopbackTester tester(nh, send_lcm_ptr, recv_lcm_ptr, motion_command_channel, motion_status_channel, control_mode_command_channel, control_mode_status_channel, gripper_command_channel, gripper_status_channel);
        tester.Loop();
        return 0;
    }
}
