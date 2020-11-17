#! /usr/bin/env python

#Suuuuuper basic grasp status node
#publishes a message indicitating if the right gripper is
#"open": all the way open
#"closed_partial": partially closed
#"closed_empty": All the way closed (so no object is being grasped)
#
# super hacky now. Only works in scissors mode

import rospy
import time
from victor_hardware_interface.msg import Robotiq3FingerStatus, GraspStatus
from std_msgs.msg import String



def right_gripper_status_callback(finger_status, pub):
    a_pos = finger_status.finger_a_status.position
    b_pos = finger_status.finger_b_status.position
    c_pos = finger_status.finger_c_status.position

    openness = "intermediate"

    closed_limit = .43
    open_limit = .03
    if a_pos > 0.40 and b_pos > closed_limit and c_pos > closed_limit:
        openness = "closed"
    elif a_pos < open_limit and b_pos < open_limit and c_pos < open_limit:
        openness = "open"

    s = GraspStatus()
    s.openness = openness
    s.in_motion = finger_status.gripper_motion_status == 1
    pub.publish(s)
    # print a_pos, b_pos, c_pos

def left_gripper_status_callback(finger_status, pub):
    a_pos = finger_status.finger_a_status.position
    b_pos = finger_status.finger_b_status.position
    c_pos = finger_status.finger_c_status.position

    openness = "intermediate"

    closed_limit = .43
    open_limit = .03
    if a_pos > 0.40 and b_pos > closed_limit and c_pos > closed_limit:
        openness = "closed"
    elif a_pos < open_limit and b_pos < open_limit and c_pos < open_limit:
        openness = "open"

    s = GraspStatus()
    s.openness = openness
    s.in_motion = finger_status.gripper_motion_status == 1
    pub.publish(s)


def main():
    rospy.init_node('gripper_status_node')

    right_grasp_status_publisher = rospy.Publisher('/right_arm/grasp_status', GraspStatus, queue_size=1)
    right_gripper_status_subscriber = rospy.Subscriber('/right_arm/gripper_status', Robotiq3FingerStatus, right_gripper_status_callback, right_grasp_status_publisher)

    left_grasp_status_publisher = rospy.Publisher('/left_arm/grasp_status', GraspStatus, queue_size=1)
    left_gripper_status_subscriber = rospy.Subscriber('/left_arm/gripper_status', Robotiq3FingerStatus, left_gripper_status_callback, left_grasp_status_publisher)
    

    rospy.spin()

if __name__ == "__main__":
    main()

