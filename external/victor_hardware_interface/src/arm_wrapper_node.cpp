#include <string>
#include <iostream>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

#include "victor_hardware_interface/minimal_arm_wrapper_interface.hpp"

int main(int argc, char** argv)
{
    // Default ROS params
    const std::string DEFAULT_CARTESIAN_POSE_FRAME = "base";
    const double DEFAULT_SET_CONTROL_MODE_TIMEOUT = 2.5;  // seconds

    // Default ROS topic / service names
    const std::string DEFAULT_MOTION_COMMAND_TOPIC("motion_command");
    const std::string DEFAULT_MOTION_STATUS_TOPIC("motion_status");
    const std::string DEFAULT_CONTROL_MODE_STATUS_TOPIC("control_mode_status");
    const std::string DEFAULT_SET_CONTROL_MODE_SERVICE("set_control_mode_service");
    const std::string DEFAULT_GET_CONTROL_MODE_SERVICE("get_control_mode_service");
    const std::string DEFAULT_GRIPPER_COMMAND_TOPIC("gripper_command");
    const std::string DEFAULT_GRIPPER_STATUS_TOPIC("gripper_status");

    // Default LCM parameters
    const std::string DEFAULT_SEND_LCM_URL("udp://10.10.10.11:30000");
    const std::string DEFAULT_RECV_LCM_URL("udp://10.10.10.100:30001");
    const std::string DEFAULT_MOTION_COMMAND_CHANNEL("motion_command");
    const std::string DEFAULT_MOTION_STATUS_CHANNEL("motion_status");
    const std::string DEFAULT_CONTROL_MODE_COMMAND_CHANNEL("control_mode_command");
    const std::string DEFAULT_CONTROL_MODE_STATUS_CHANNEL("control_mode_status");
    const std::string DEFAULT_GRIPPER_COMMAND_CHANNEL("gripper_command");
    const std::string DEFAULT_GRIPPER_STATUS_CHANNEL("gripper_status");

    // Start ROS
    ros::init(argc, argv, "right_arm_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    // Get params
    const std::string cartesian_pose_frame  = nhp.param("cartesian_pose_frame", DEFAULT_CARTESIAN_POSE_FRAME);
    const double set_control_mode_timeout   = nhp.param("set_control_mode_timeout", DEFAULT_SET_CONTROL_MODE_TIMEOUT);

    // Get topic & service names
    const std::string motion_command_topic      = nhp.param("motion_command_topic", DEFAULT_MOTION_COMMAND_TOPIC);
    const std::string motion_status_topic       = nhp.param("motion_status_topic", DEFAULT_MOTION_STATUS_TOPIC);
    const std::string control_mode_status_topic = nhp.param("control_mode_status_topic", DEFAULT_CONTROL_MODE_STATUS_TOPIC);
    const std::string get_control_mode_service  = nhp.param("get_control_mode_service", DEFAULT_GET_CONTROL_MODE_SERVICE);
    const std::string set_control_mode_service  = nhp.param("set_control_mode_service", DEFAULT_SET_CONTROL_MODE_SERVICE);
    const std::string gripper_command_topic     = nhp.param("gripper_command_topic", DEFAULT_GRIPPER_COMMAND_TOPIC);
    const std::string gripper_status_topic      = nhp.param("gripper_status_topic", DEFAULT_GRIPPER_STATUS_TOPIC);

    // Get LCM params
    const std::string send_lcm_url                  = nhp.param("send_lcm_url", DEFAULT_SEND_LCM_URL);
    const std::string recv_lcm_url                  = nhp.param("recv_lcm_url", DEFAULT_RECV_LCM_URL);
    const std::string motion_command_channel        = nhp.param("motion_command_channel", DEFAULT_MOTION_COMMAND_CHANNEL);
    const std::string motion_status_channel         = nhp.param("motion_status_channel", DEFAULT_MOTION_STATUS_CHANNEL);
    const std::string control_mode_command_channel  = nhp.param("control_mode_command_channel", DEFAULT_CONTROL_MODE_COMMAND_CHANNEL);
    const std::string control_mode_status_channel   = nhp.param("control_mode_status_channel", DEFAULT_CONTROL_MODE_STATUS_CHANNEL);
    const std::string gripper_command_channel       = nhp.param("gripper_command_channel", DEFAULT_GRIPPER_COMMAND_CHANNEL);
    const std::string gripper_status_channel        = nhp.param("gripper_status_channel", DEFAULT_GRIPPER_STATUS_CHANNEL);

    std::shared_ptr<lcm::LCM> send_lcm_ptr = nullptr;
    std::shared_ptr<lcm::LCM> recv_lcm_ptr = nullptr;
    // Create only one lcm object if the send and recieve addresses are the same (used for udpm protocol)
    if (send_lcm_url == recv_lcm_url)
    {
        ROS_INFO_NAMED(ros::this_node::getName(), "Starting with shared send/receive LCM [%s]...", send_lcm_url.c_str());
        send_lcm_ptr = std::make_shared<lcm::LCM>(send_lcm_url);
        recv_lcm_ptr = send_lcm_ptr;
    }
    // Create seperate lcm objects if the send and recieve addresses are different (used for udp and tcp protocols)
    else
    {
        ROS_INFO_NAMED(ros::this_node::getName(), "Starting with separate send [%s] and receive [%s] LCM...", send_lcm_url.c_str(), recv_lcm_url.c_str());
        send_lcm_ptr = std::make_shared<lcm::LCM>(send_lcm_url);
        recv_lcm_ptr = std::make_shared<lcm::LCM>(recv_lcm_url);
    }

    // Start ROS and LCM bridge itself
    victor_hardware_interface::MinimalArmWrapperInterface interface(
                nh, send_lcm_ptr, recv_lcm_ptr,
                cartesian_pose_frame,
                set_control_mode_timeout,
                // ROS Topics
                motion_command_topic, motion_status_topic,
                control_mode_status_topic, get_control_mode_service, set_control_mode_service,
                gripper_command_topic, gripper_status_topic,
                // LCM Channels
                motion_command_channel, motion_status_channel,
                control_mode_command_channel, control_mode_status_channel,
                gripper_command_channel, gripper_status_channel);

    // Loop forever, or until there an error or termination signal (SIGINT, etc.)
    interface.spin();
    return 0;
}
