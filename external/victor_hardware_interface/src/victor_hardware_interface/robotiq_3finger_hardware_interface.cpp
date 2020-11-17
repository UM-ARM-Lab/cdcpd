#include <victor_hardware_interface/robotiq_3finger_hardware_interface.hpp>

namespace victor_hardware_interface
{
    /////////////////////////////////////////////////////////////////////////////////
    // Robotiq3FingerHardwardInterface class implementation
    /////////////////////////////////////////////////////////////////////////////////

    Robotiq3FingerHardwareInterface::Robotiq3FingerHardwareInterface(
            const std::shared_ptr<lcm::LCM>& send_lcm_ptr,
            const std::shared_ptr<lcm::LCM>& recv_lcm_ptr,
            const std::string& command_channel_name,
            const std::string& status_channel_name,
            const std::function<void(const Robotiq3FingerStatus&)>& status_callback_fn)
        : send_lcm_ptr_(send_lcm_ptr), recv_lcm_ptr_(recv_lcm_ptr)
        , command_channel_name_(command_channel_name)
        , status_channel_name_(status_channel_name)
        , status_callback_fn_(status_callback_fn)
    {
        // Check lcm is valid to communicating
        if (send_lcm_ptr_->good() != true)
        {
            throw std::invalid_argument("Send LCM interface is not good");
        }
        if (recv_lcm_ptr_->good() != true)
        {
            throw std::invalid_argument("Receive LCM interface is not good");
        }
        recv_lcm_ptr_->subscribe(status_channel_name_,
                                 &Robotiq3FingerHardwareInterface::internalStatusLCMCallback,
                                 this);
    }

    bool Robotiq3FingerHardwareInterface::sendCommandMessage(const Robotiq3FingerCommand& command)
    {
        const robotiq_3finger_command lcm_command = commandRosToLcm(command);
        const int ret = send_lcm_ptr_->publish(command_channel_name_, &lcm_command);
        if (ret == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void Robotiq3FingerHardwareInterface::internalStatusLCMCallback(const lcm::ReceiveBuffer* buffer,
                                                                    const std::string& channel,
                                                                    const robotiq_3finger_status* status_msg)
    {
        UNUSED(buffer);
        UNUSED(channel);
        const Robotiq3FingerStatus ros_status = statusLcmToRos(*status_msg);
        status_callback_fn_(ros_status);
    }

    /////////////////////////////////////////////////////////////////////////////////
    // Ros and LCM convert helper functions
    /////////////////////////////////////////////////////////////////////////////////

    robotiq_3finger_actuator_command fingerCommandRosToLcm(const Robotiq3FingerActuatorCommand& finger_command)
    {
        robotiq_3finger_actuator_command lcm_command;

        // "Magic" number 0.0 and 1 are the bounds representing percentage value
        lcm_command.position = arc_helpers::ClampValueAndWarn(finger_command.position, 0.0, 1.0);
        lcm_command.speed = arc_helpers::ClampValueAndWarn(finger_command.speed, 0.0, 1.0);
        lcm_command.force = arc_helpers::ClampValueAndWarn(finger_command.force, 0.0, 1.0);
        lcm_command.timestamp = finger_command.header.stamp.toSec();
        return lcm_command;
    }

    Robotiq3FingerActuatorStatus fingerStatusLcmToRos(const robotiq_3finger_actuator_status& finger_status)
    {
        Robotiq3FingerActuatorStatus ros_status;
        ros_status.position = finger_status.position;
        ros_status.position_request = finger_status.position_request;
        ros_status.current = finger_status.current;
        ros_status.header.stamp = ros::Time(finger_status.timestamp);
        return ros_status;
    }

    Robotiq3FingerObjectStatus objectStatusLcmToRos(const robotiq_3finger_object_status& object_status)
    {
        Robotiq3FingerObjectStatus ros_status;
        ros_status.status = (uint8_t)object_status.status;
        ros_status.header.stamp = ros::Time(object_status.timestamp);
        return ros_status;
    }

    Robotiq3FingerStatus statusLcmToRos(const robotiq_3finger_status& status)
    {
        Robotiq3FingerStatus ros_status;
        ros_status.finger_a_status = fingerStatusLcmToRos(status.finger_a_status);
        ros_status.finger_b_status = fingerStatusLcmToRos(status.finger_b_status);
        ros_status.finger_c_status = fingerStatusLcmToRos(status.finger_c_status);
        ros_status.scissor_status = fingerStatusLcmToRos(status.scissor_status);
        ros_status.finger_a_object_status = objectStatusLcmToRos(status.finger_a_object_status);
        ros_status.finger_b_object_status = objectStatusLcmToRos(status.finger_b_object_status);
        ros_status.finger_c_object_status = objectStatusLcmToRos(status.finger_c_object_status);
        ros_status.scissor_object_status = objectStatusLcmToRos(status.scissor_object_status);
        ros_status.gripper_action_status = (uint8_t)status.gripper_action_status;
        ros_status.gripper_system_status = (uint8_t)status.gripper_system_status;
        ros_status.gripper_motion_status = (uint8_t)status.gripper_motion_status;
        ros_status.gripper_fault_status = (uint8_t)status.gripper_fault_status;
        ros_status.initialization_status = (uint8_t)status.initialization_status;
        ros_status.header.stamp = ros::Time(status.timestamp);
        return ros_status;
    }

    robotiq_3finger_command commandRosToLcm(const Robotiq3FingerCommand& command)
    {
        robotiq_3finger_command lcm_command;
        lcm_command.finger_a_command = fingerCommandRosToLcm(command.finger_a_command);
        lcm_command.finger_b_command = fingerCommandRosToLcm(command.finger_b_command);
        lcm_command.finger_c_command = fingerCommandRosToLcm(command.finger_c_command);
        lcm_command.scissor_command = fingerCommandRosToLcm(command.scissor_command);
        lcm_command.timestamp = command.header.stamp.toSec();
        return lcm_command;
    }

}
