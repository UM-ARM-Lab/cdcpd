#! /usr/bin/env python


# Run this script to be able to manually push the robot to any desired position
#
# Puts the robot in impedance mode, but updates the setpoint position to be the measured position
# Lowpass filter and thresholding prevent drift
# 


import numpy as np
import time

import rospy
from victor_hardware_interface import victor_utils as vu
from victor_hardware_interface.msg import *
from threading import Lock
from arc_utilities import ros_helpers as rh


joint_names = ['joint_' + str(i) for i in range(1, 8)]

hard_limits_deg = np.array([170.0,  120.0,  170.0,  120.0,  170.0,  120.0,  175.0])
safety_deg = 30.0  # Stay far away from joint hard limits

joint_lower = -(hard_limits_deg - safety_deg) * np.pi/180
joint_upper =  (hard_limits_deg - safety_deg) * np.pi/180


class ManualMotion:

    def __init__(self, arm_name):
        self.sub = rospy.Subscriber(arm_name + "/motion_status", MotionStatus, self.callback_update)
        self.pub = rospy.Publisher(arm_name + "/motion_command", MotionCommand, queue_size=10)

        self.threshold = 0.05
        self.arrive_threshold = 0.02
        
        self.low_pass_tau = 0.01  # seconds
        self.prev_time = None
        self.lock = Lock()
        self.follow = False
        self.last_pos = None

    def callback_update(self, msg):
        with self.lock:
            self.run_lowpass(msg)

    def run_lowpass(self, msg):
        
        if self.prev_time is None:
            self.prev_time = time.time()

        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        if dt > 1:
            print("Incredibly long time between messages. Something is weird")
            print("dt: {}".format(dt))
            assert False

        # Update our internal copy and calculate error for later thresholding
        meas = np.array([getattr(msg.measured_joint_position,  joint) for joint in joint_names])
        cmd =  np.array([getattr(msg.commanded_joint_position, joint) for joint in joint_names])
        err = np.linalg.norm(meas - cmd)
        self.last_pos = meas


        if err > self.threshold:
            # Robot is far enough away from setpoint that we assume someone is pushing it
            self.follow = True

        if err < self.arrive_threshold:
            # Robot is close enough to setpoint that we go into "hold position" mode
            self.follow = False

        if not self.follow:
            # Just hold position
            return

        # Stay away from hard limits
        meas = meas.clip(joint_lower, joint_upper)

        # Low pass filter to avoid jerky motions
        alpha = dt / (self.low_pass_tau + dt)
        new_cmd = (1.0 - alpha) * cmd + alpha * meas

        # Send the actual command
        cmd_msg = MotionCommand()
        cmd_msg.control_mode.mode = ControlMode.JOINT_IMPEDANCE
        for idx in range(len(joint_names)):
            setattr(cmd_msg.joint_position, joint_names[idx], new_cmd[idx])
        self.pub.publish(cmd_msg)
        
        
def print_joints(left, right):
    """Print nicely to the terminal so joint values can be copied"""
    rospy.loginfo("Joint angles are: ")
    vec_to_rad_str = lambda vec: '[' + ', '.join([str(np.round(rad, 3)) for rad in vec]) + ']'
    if (left is not None) and (left.last_pos is not None):
        rospy.loginfo("Left: " + vec_to_rad_str(left.last_pos))
    if (right is not None) and (right.last_pos is not None):
        rospy.loginfo("Right: " + vec_to_rad_str(right.last_pos))


if __name__ == "__main__":
    rospy.init_node("manual_motion")
    

    control_mode_params = vu.get_joint_impedance_params(vu.Stiffness.MEDIUM)

    use_left_arm = rospy.get_param("~use_left_arm", True)
    use_right_arm = rospy.get_param("~use_right_arm", True)



    if(use_left_arm):
        print("initializing left arm ...")
        sys.stdout.flush()
        l_cm = rh.Listener("left_arm/control_mode_status", ControlModeParameters)
        cur_mode = l_cm.get(block_until_data=True)
        control_mode_params.joint_path_execution_params.joint_relative_velocity = cur_mode.joint_path_execution_params.joint_relative_velocity
        control_mode_params.joint_path_execution_params.joint_relative_acceleration = cur_mode.joint_path_execution_params.joint_relative_acceleration
        
        result = vu.send_new_control_mode("left_arm", control_mode_params)
        while not result.success:
            result = vu.send_new_control_mode("left_arm", control_mode_params)
        print("done")
    else:
        print("not using left arm")    

    if(use_right_arm):
        print("initializing right arm ...")
        r_cm = rh.Listener("right_arm/control_mode_status", ControlModeParameters)
        cur_mode = r_cm.get(block_until_data=True)
        control_mode_params.joint_path_execution_params.joint_relative_velocity = cur_mode.joint_path_execution_params.joint_relative_velocity
        control_mode_params.joint_path_execution_params.joint_relative_acceleration = cur_mode.joint_path_execution_params.joint_relative_acceleration

        sys.stdout.flush()
        result = vu.send_new_control_mode("right_arm", control_mode_params)
        while not result.success:
            result = vu.send_new_control_mode("right_arm", control_mode_params)
        print("done")
    else:
        print("not using right arm")    


    left = None
    right = None
    if(use_left_arm):
        left = ManualMotion("left_arm")
    if(use_right_arm):
        right = ManualMotion("right_arm")

    

    rospy.on_shutdown(lambda: print_joints(left, right))

    print("Running")
    rospy.spin()
