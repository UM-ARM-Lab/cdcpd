#!/usr/bin/env python

#####################################################
#                                                   #
#   Copyright (c) 2017, UM-ARM-LAB                  #
#                                                   #
#   Regression test for arm control modes           #
#                                                   #
#####################################################

import rospy
import victor_hardware_interface.msg
import victor_hardware_interface.srv


class ControlModeTester(object):

    def __init__(self):
        print("Setting up control mode & motion command...")
        rospy.init_node("control_mode_tester")
        self.set_control_mode_server = rospy.ServiceProxy("right_arm/set_control_mode_service",
                                                          victor_hardware_interface.srv.SetControlMode)
        self.set_control_mode_server.wait_for_service()
        self.get_control_mode_server = rospy.ServiceProxy("right_arm/get_control_mode_service",
                                                          victor_hardware_interface.srv.GetControlMode)
        self.get_control_mode_server.wait_for_service()
        self.motion_command_pub = rospy.Publisher("right_arm/motion_command",
                                                  victor_hardware_interface.msg.MotionCommand,
                                                  queue_size=1)
        print("...Finished setting up services & publishers")

    def joint_position_test(self):
        raw_input("Run joint position test - Press ENTER to continue...")
        control_mode_command = victor_hardware_interface.msg.ControlModeParameters()
        control_mode_command.control_mode.mode = victor_hardware_interface.msg.ControlMode.JOINT_POSITION
        control_mode_command.joint_path_execution_params.joint_relative_velocity = 0.5
        control_mode_command.joint_path_execution_params.joint_relative_acceleration = 0.5
        control_mode_command.joint_path_execution_params.override_joint_acceleration = 0.0
        request = victor_hardware_interface.srv.SetControlModeRequest()
        request.new_control_mode = control_mode_command
        response = self.set_control_mode_server.call(request)
        print("SetControlMode response: " + str(response))
        assert(response.success is True)
        print("...Joint position test complete")

    def joint_impedance_test(self):
        raw_input("Run joint impedance test - Press ENTER to continue...")
        control_mode_command = victor_hardware_interface.msg.ControlModeParameters()
        control_mode_command.control_mode.mode = victor_hardware_interface.msg.ControlMode.JOINT_IMPEDANCE
        control_mode_command.joint_path_execution_params.joint_relative_velocity = 0.5
        control_mode_command.joint_path_execution_params.joint_relative_acceleration = 0.5
        control_mode_command.joint_path_execution_params.override_joint_acceleration = 0.0
        control_mode_command.joint_impedance_params.joint_damping.joint_1 = 0.7
        control_mode_command.joint_impedance_params.joint_damping.joint_2 = 0.7
        control_mode_command.joint_impedance_params.joint_damping.joint_3 = 0.7
        control_mode_command.joint_impedance_params.joint_damping.joint_4 = 0.7
        control_mode_command.joint_impedance_params.joint_damping.joint_5 = 0.7
        control_mode_command.joint_impedance_params.joint_damping.joint_6 = 0.7
        control_mode_command.joint_impedance_params.joint_damping.joint_7 = 0.7
        control_mode_command.joint_impedance_params.joint_stiffness.joint_1 = 1.0
        control_mode_command.joint_impedance_params.joint_stiffness.joint_2 = 1.0
        control_mode_command.joint_impedance_params.joint_stiffness.joint_3 = 1.0
        control_mode_command.joint_impedance_params.joint_stiffness.joint_4 = 1.0
        control_mode_command.joint_impedance_params.joint_stiffness.joint_5 = 1.0
        control_mode_command.joint_impedance_params.joint_stiffness.joint_6 = 1.0
        control_mode_command.joint_impedance_params.joint_stiffness.joint_7 = 1.0
        request = victor_hardware_interface.srv.SetControlModeRequest()
        request.new_control_mode = control_mode_command
        response = self.set_control_mode_server.call(request)
        print("SetControlMode response: " + str(response))
        assert (response.success is True)
        print("...Joint impedance test complete")

    def cartesian_pose_test(self):
        raw_input("Run cartesian pose test - Press ENTER to continue...")
        control_mode_command = victor_hardware_interface.msg.ControlModeParameters()
        control_mode_command.control_mode.mode = victor_hardware_interface.msg.ControlMode.CARTESIAN_POSE
        control_mode_command.cartesian_path_execution_params.max_velocity.x = 0.5
        control_mode_command.cartesian_path_execution_params.max_velocity.y = 0.5
        control_mode_command.cartesian_path_execution_params.max_velocity.z = 0.5
        control_mode_command.cartesian_path_execution_params.max_velocity.a = 0.5
        control_mode_command.cartesian_path_execution_params.max_velocity.b = 0.5
        control_mode_command.cartesian_path_execution_params.max_velocity.c = 0.5
        control_mode_command.cartesian_path_execution_params.max_nullspace_velocity = 0.5
        control_mode_command.cartesian_path_execution_params.max_acceleration.x = 0.5
        control_mode_command.cartesian_path_execution_params.max_acceleration.y = 0.5
        control_mode_command.cartesian_path_execution_params.max_acceleration.z = 0.5
        control_mode_command.cartesian_path_execution_params.max_acceleration.a = 0.5
        control_mode_command.cartesian_path_execution_params.max_acceleration.b = 0.5
        control_mode_command.cartesian_path_execution_params.max_acceleration.c = 0.5
        control_mode_command.cartesian_path_execution_params.max_nullspace_acceleration = 0.5
        request = victor_hardware_interface.srv.SetControlModeRequest()
        request.new_control_mode = control_mode_command
        response = self.set_control_mode_server.call(request)
        print("SetControlMode response: " + str(response))
        assert (response.success is True)
        print("...Cartesian pose test complete")

    def cartesian_impedance_test(self):
        raw_input("Run cartesian impedance test - Press ENTER to continue...")
        control_mode_command = victor_hardware_interface.msg.ControlModeParameters()
        control_mode_command.control_mode.mode = victor_hardware_interface.msg.ControlMode.CARTESIAN_IMPEDANCE
        control_mode_command.cartesian_path_execution_params.max_velocity.x = 10.0
        control_mode_command.cartesian_path_execution_params.max_velocity.y = 10.0
        control_mode_command.cartesian_path_execution_params.max_velocity.z = 10.0
        control_mode_command.cartesian_path_execution_params.max_velocity.a = 10.0
        control_mode_command.cartesian_path_execution_params.max_velocity.b = 10.0
        control_mode_command.cartesian_path_execution_params.max_velocity.c = 10.0
        control_mode_command.cartesian_path_execution_params.max_nullspace_velocity = 25.0
        control_mode_command.cartesian_path_execution_params.max_acceleration.x = 10.0
        control_mode_command.cartesian_path_execution_params.max_acceleration.y = 10.0
        control_mode_command.cartesian_path_execution_params.max_acceleration.z = 10.0
        control_mode_command.cartesian_path_execution_params.max_acceleration.a = 10.0
        control_mode_command.cartesian_path_execution_params.max_acceleration.b = 10.0
        control_mode_command.cartesian_path_execution_params.max_acceleration.c = 10.0
        control_mode_command.cartesian_path_execution_params.max_nullspace_acceleration = 10.0
        control_mode_command.cartesian_impedance_params.cartesian_damping.x = 0.7
        control_mode_command.cartesian_impedance_params.cartesian_damping.y = 0.7
        control_mode_command.cartesian_impedance_params.cartesian_damping.z = 0.7
        control_mode_command.cartesian_impedance_params.cartesian_damping.a = 0.7
        control_mode_command.cartesian_impedance_params.cartesian_damping.b = 0.7
        control_mode_command.cartesian_impedance_params.cartesian_damping.c = 0.7
        control_mode_command.cartesian_impedance_params.nullspace_damping = 0.7
        control_mode_command.cartesian_impedance_params.cartesian_stiffness.x = 10.0
        control_mode_command.cartesian_impedance_params.cartesian_stiffness.y = 10.0
        control_mode_command.cartesian_impedance_params.cartesian_stiffness.z = 10.0
        control_mode_command.cartesian_impedance_params.cartesian_stiffness.a = 10.0
        control_mode_command.cartesian_impedance_params.cartesian_stiffness.b = 10.0
        control_mode_command.cartesian_impedance_params.cartesian_stiffness.c = 10.0
        control_mode_command.cartesian_impedance_params.nullspace_stiffness = 10.0
        control_mode_command.cartesian_control_mode_limits.max_path_deviation.x = 10000000.0
        control_mode_command.cartesian_control_mode_limits.max_path_deviation.y = 10000000.0
        control_mode_command.cartesian_control_mode_limits.max_path_deviation.z = 10000000.0
        control_mode_command.cartesian_control_mode_limits.max_path_deviation.a = 10000000.0
        control_mode_command.cartesian_control_mode_limits.max_path_deviation.b = 10000000.0
        control_mode_command.cartesian_control_mode_limits.max_path_deviation.c = 10000000.0
        control_mode_command.cartesian_control_mode_limits.max_cartesian_velocity.x = 1000.0
        control_mode_command.cartesian_control_mode_limits.max_cartesian_velocity.y = 1000.0
        control_mode_command.cartesian_control_mode_limits.max_cartesian_velocity.z = 1000.0
        control_mode_command.cartesian_control_mode_limits.max_cartesian_velocity.a = 1000.0
        control_mode_command.cartesian_control_mode_limits.max_cartesian_velocity.b = 1000.0
        control_mode_command.cartesian_control_mode_limits.max_cartesian_velocity.c = 1000.0
        control_mode_command.cartesian_control_mode_limits.max_control_force.x = 10.0
        control_mode_command.cartesian_control_mode_limits.max_control_force.y = 10.0
        control_mode_command.cartesian_control_mode_limits.max_control_force.z = 10.0
        control_mode_command.cartesian_control_mode_limits.max_control_force.a = 10.0
        control_mode_command.cartesian_control_mode_limits.max_control_force.b = 10.0
        control_mode_command.cartesian_control_mode_limits.max_control_force.c = 10.0
        control_mode_command.cartesian_control_mode_limits.stop_on_max_control_force = False
        request = victor_hardware_interface.srv.SetControlModeRequest()
        request.new_control_mode = control_mode_command
        response = self.set_control_mode_server.call(request)
        print("SetControlMode response: " + str(response))
        assert (response.success is True)
        print("...Cartesian impedance test complete")

    def run_all_tests(self):
        print("Starting arm control regression tests...")
        raw_input("Is the arm safely away from obstacles? Press ENTER to continue...")
        self.joint_position_test()
        self.joint_impedance_test()
        self.cartesian_pose_test()
        self.cartesian_impedance_test()
        print("...Finished arm control regression tests")


if __name__ == '__main__':
    tester = ControlModeTester()
    tester.run_all_tests()
