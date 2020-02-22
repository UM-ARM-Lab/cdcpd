#!/usr/bin/env python
import rospy
import sys
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import *
from cdcpd.ros_wrappers import get_ros_param
from cdcpd.tf_utils import TF2Wrapper


class Tracker():
    def __init__(self, tf_wrapper, frame_name):
        self.tf_wrapper = tf_wrapper
        self.frame_name = frame_name
        self.pub = rospy.Publisher("cdcpd/" + self.frame_name, TransformStamped, queue_size=1)

    def publish(self, target_frame):
        current = self.tf_wrapper.get_transform_ros(parent=target_frame, child=self.frame_name)
        self.pub.publish(current)


class GripperTransforms():
    def __init__(self):
        self.tf_wrapper = TF2Wrapper()
        self.tracked_frames = ["victor_left_tool"]
        self.trackers = {name: Tracker(self.tf_wrapper, name) for name in self.tracked_frames}
        self.target_frame = get_ros_param(param_name="~kinect_frame", default='kinect2_victor_head_rgb_optical_frame')

    def spin(self):
        rospy.loginfo("Gripper prior spinning ...")
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            for _, tracker in self.trackers.items():
                tracker.publish(self.target_frame)
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
