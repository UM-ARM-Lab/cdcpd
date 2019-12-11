#!/usr/bin/env python
import rospy
import sys
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import *
from cdcpd.ros_wrappers import get_ros_param
from cdcpd.tf_utils import TF2Wrapper


class GripperTransforms():
    def __init__(self):
        self.tf_wrapper = TF2Wrapper()
        gripper0_name = get_ros_param(param_name="gripper0_name", default='left')
        gripper1_name = get_ros_param(param_name="gripper1_name", default='right')
        self.left_gripper_tf_name = self.tf_wrapper.GripperTFName(arm_name=gripper0_name)
        self.right_gripper_tf_name = self.tf_wrapper.GripperTFName(arm_name=gripper1_name)

        self.pub_left = rospy.Publisher("cdcpd/left_gripper_prior", TransformStamped, queue_size=10)
        self.pub_right = rospy.Publisher("cdcpd/right_gripper_prior", TransformStamped, queue_size=10)

        self.use_victor = get_ros_param(param_name="use_victor", default=True)
        self.use_val = get_ros_param(param_name="use_val", default=False)
        assert(self.use_victor ^ self.use_val)

        self.target_frame = get_ros_param(param_name="~kinect_frame", default='kinect2_victor_head_rgb_optical_frame')

    def getGripperTransform(self, gripper_name, stamp):
        if self.use_victor:
            palm = self.tf_wrapper.get_transform_ros(parent=self.target_frame, child=gripper_name + "_palm_surface")
            fingertipA = self.tf_wrapper.get_transform_ros(parent=self.target_frame, child=gripper_name + "_fingerA_dist")
            fingertipB = self.tf_wrapper.get_transform_ros(parent=self.target_frame, child=gripper_name + "_fingerB_dist")
            fingertipC = self.tf_wrapper.get_transform_ros(parent=self.target_frame, child=gripper_name + "_fingerC_dist")

            tf = TransformStamped()
            tf.header.frame_id = self.target_frame
            tf.header.stamp = stamp
            tf.child_frame_id = gripper_name + "_fingertip_average"
            tf.transform.translation.x = (fingertipA.transform.translation.x +
                                          fingertipB.transform.translation.x +
                                          fingertipC.transform.translation.x) / 3.0
            tf.transform.translation.y = (fingertipA.transform.translation.y +
                                          fingertipB.transform.translation.y +
                                          fingertipC.transform.translation.y) / 3.0
            tf.transform.translation.z = (fingertipA.transform.translation.z +
                                          fingertipB.transform.translation.z +
                                          fingertipC.transform.translation.z) / 3.0
            tf.transform.rotation = palm.transform.rotation
            return tf

        elif self.use_val:
            fingertip_transform = self.tf_wrapper.get_transform_ros(parent=self.target_frame, child=gripper_name)
            return fingertip_transform

        else:
            assert(False)

    def spin(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            stamp = rospy.get_rostime()
            left_gripper = self.getGripperTransform(gripper_name=self.left_gripper_tf_name, stamp=stamp)
            right_gripper = self.getGripperTransform(gripper_name=self.right_gripper_tf_name, stamp=stamp)
            self.pub_left.publish(left_gripper)
            self.pub_right.publish(right_gripper)
            rospy.loginfo_throttle(2.0, "Gripper prior spinning ...")
            rate.sleep()


def main():
    rospy.init_node('gripper_tf_node')
    use_gripper_prior = get_ros_param(param_name="cdcpd_node/use_gripper_prior", default=False)
    if use_gripper_prior:
        gt = GripperTransforms()
        gt.spin()
    else:
        rospy.logwarn("use_gripper_prior set to ", use_gripper_prior, ". Exiting")


if __name__ == "__main__":
    main()

