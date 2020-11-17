#!/usr/bin/env python

#####################################################
#                                                   #
#   Copyright (c) 2017, UM-ARM-LAB                  #
#                                                   #
#   Arm Wrench Republisher                          #
#                                                   #
#####################################################

# Listens to robot feedback, extracts and republishes just the wrench data
# Useful for plotting directly in rviz, or otherwise.



import rospy
from victor_hardware_interface.msg import MotionStatus
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3

hand_to_frame = {'left':  'victor_left_gripper_palm_surface',
                 'right': 'victor_right_gripper_palm_surface'}

hand_wrench_pub = {'left':  rospy.Publisher('left_gripper/wrench', WrenchStamped, queue_size=1),
                   'right': rospy.Publisher('right_gripper/wrench', WrenchStamped, queue_size=1)}


def publishWrench(data, hand):
    wr = data.estimated_external_wrench
    wrench_msg = Wrench()
    wrench_msg.force = Vector3(wr.x, wr.y, wr.z)
    wrench_msg.torque = Vector3(wr.c, wr.b, wr.a)

    wrench_stamped_msg = WrenchStamped()
    wrench_stamped_msg.wrench = wrench_msg
    wrench_stamped_msg.header.stamp = rospy.Time(0)
    wrench_stamped_msg.header.frame_id = hand_to_frame[hand]

    hand_wrench_pub[hand].publish(wrench_stamped_msg)

def listener():
    rospy.Subscriber('/left_arm/motion_status', MotionStatus, publishWrench, 'left')
    rospy.Subscriber('/right_arm/motion_status', MotionStatus, publishWrench, 'right')
    rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node('wrench_republisher')
    rospy.loginfo('Wrench republisher started')
    listener()
