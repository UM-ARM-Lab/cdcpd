#!/usr/bin/env python3
import time
start_time = time.time()
import sys
import rospy
import pickle
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import *
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
import ros_numpy
from cdcpd import CDCPDParams, ConstrainedDeformableCPD
from cpd import CPDParams
from optimizer import EdgeConstrainedOptimizer
from optimizer import DistanceConstrainedOptimizer
from geometry_utils import build_line
from geometry_utils import build_rectangle
from cv_utils import chroma_key_rope
from cv_utils import chroma_key_mflag_lab
from prior import UniformPrior
from prior import ThresholdVisibilityPrior
from failure_recovery import SmoothFreeSpaceCost
from geometry_msgs.msg import TransformStamped
from copy import deepcopy
import time
from threading import Lock
from ros_wrappers import Listener
from ros_wrappers import get_ros_param
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib import cm

class Tracker:
    def __init__(self, object_name):
        """
        Tracker is a node which combines CDCPD tracking with the information from
        grippers and also uses failure recovery for rope or cloth.
        Parameters:
            object_name(str):       "rope" or "cloth" depending on the object you need to detect
        """
        self.kinect_intrinsics = np.array([1068.842896477257, 0.0, 950.2974736758024, 0.0, 1066.0150152835104, 537.097974092338, 0.0, 0.0, 1.0],
            dtype=np.float32).reshape((3, 3))
        self.kinect_intrinsics[:2] /= 2.0
        self.listener_left = None
        self.listener_right = None
        self.cost_estimator = None
        self.count = 0
        self.use_pickle = get_ros_param(param_name="~use_pickle", default=True)
        self.use_gripper_prior = get_ros_param(param_name="~use_gripper_prior", default=False)
        self.visualize_violations = get_ros_param(param_name="~visualize_violations", default=False)
        if(self.use_pickle):
            self.input_data = pickle.load(open("/home/deformtrack/examples/data/2019-06-27 13-13-10.pk", "rb"))
        if(object_name=="rope"):
            self.template_verts, self.template_edges = build_line(1.0, get_ros_param(param_name="rope_num_links", default=50))
            self.key_func = chroma_key_rope
        elif(object_name=="cloth"):
            self.template_verts, self.template_edges = build_rectangle(width=get_ros_param(param_name="cloth_y_size", default=0.45),
                height=get_ros_param(param_name="cloth_x_size", default=0.32), width_num_node=get_ros_param(param_name="cloth_num_control_points_y", default=23),
                height_num_node=get_ros_param(param_name="cloth_num_control_points_x", default=17))
            self.key_func = chroma_key_mflag_lab

        if(self.use_gripper_prior):
            self.prior = UniformPrior()
            self.optimizer = EdgeConstrainedOptimizer(template=self.template_verts, edges=self.template_edges,
             use_gripper_prior = self.use_gripper_prior, visualize_violations = self.visualize_violations)
            self.listener_left = Listener(topic_name="/left_gripper/prior", topic_type=TransformStamped)
            self.listener_right = Listener(topic_name="/right_gripper/prior", topic_type=TransformStamped)
        else:
            self.prior = ThresholdVisibilityPrior(self.kinect_intrinsics)
            self.optimizer = EdgeConstrainedOptimizer(template=self.template_verts, edges=self.template_edges, 
                use_gripper_prior = self.use_gripper_prior, visualize_violations = self.visualize_violations)

        self.cpd_params = CPDParams()

        if(get_ros_param(param_name="~use_failure_recovery", default=False)):
            self.cost_estimator = SmoothFreeSpaceCost(self.kinect_intrinsics)
            self.cdcpd_params = CDCPDParams(prior=self.prior,
                               optimizer=self.optimizer,
                               use_recovery=True,
                               visualize_violations = self.visualize_violations,
                               recovery_cost_estimator=self.cost_estimator,
                               recovery_cost_threshold=0.1)
        else:
            self.cdcpd_params = CDCPDParams(prior=self.prior, optimizer=self.optimizer,down_sample_size=150)

        self.cdcpd = ConstrainedDeformableCPD(template=self.template_verts,
                                         cdcpd_params=self.cdcpd_params)
        
        if(self.use_gripper_prior):
            self.gripper_prior_idx = [get_ros_param(param_name="right_gripper_attached_node_idx", default=0), get_ros_param(param_name="left_gripper_attached_node_idx", default=49)]

        # initialize ROS publisher
        self.pub = rospy.Publisher("/cdcpd_tracker/points", PointCloud2, queue_size=10)
        self.vis_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        # print("--- %s seconds ---" % (time.time() - start_time))
        self.listen()

    def listen(self):
        self.sub = rospy.Subscriber(get_ros_param(param_name="~PointCloud_topic", default="/kinect2_victor_head/qhd/points"), PointCloud2, self.callback, queue_size=1)
    
    def display_violations_1(self, points, msg):
        #visualize violations
        # marker_msg = MarkerArray()
        marker_temp = Marker()
        marker_temp.header = msg.header
        # marker_temp.header.stamp = rospy.Time.now()
        # marker_temp.header.frame_id = "kinect2_victor_head_rgb_optical_frame"
        marker_temp.ns = "stretch"
        marker_temp.id = 0
        marker_temp.type = Marker.LINE_LIST
        marker_temp.action = Marker.ADD
        marker_temp.pose.position.x = 0
        marker_temp.pose.position.y = 0
        marker_temp.pose.position.z = 0
        marker_temp.pose.orientation.x = 0.0
        marker_temp.pose.orientation.y = 0.0
        marker_temp.pose.orientation.z = 0.0
        marker_temp.pose.orientation.w = 1.0
        marker_temp.scale.x = 0.01
        marker_temp.scale.y = 0.01
        marker_temp.scale.z = 0.01
        marker_temp.color.a = 1.0 # Don't forget to set the alpha!
        marker_temp.color.r = 0.0
        marker_temp.color.g = 1.0
        marker_temp.color.b = 0.0
        marker_temp.points = []
        for i in range(points.shape[1]):
            # print(i) 
            marker_temp.points.append(ros_numpy.msgify(Point, points[0][i]))
            marker_temp.points.append(ros_numpy.msgify(Point, points[1][i]))
            # marker_msg.markers.append(marker_temp) 
        
        self.vis_pub.publish( marker_temp )

    def display_violations_2(self, points, msg):
        marker_temp = Marker()
        marker_temp.header = msg.header
        # marker_temp.header.stamp = rospy.Time.now()
        # marker_temp.header.frame_id = "kinect2_victor_head_rgb_optical_frame"
        marker_temp.ns = "crossing"
        marker_temp.id = 0
        marker_temp.type = Marker.LINE_LIST
        marker_temp.action = Marker.ADD
        marker_temp.pose.position.x = 0
        marker_temp.pose.position.y = 0
        marker_temp.pose.position.z = 0
        marker_temp.pose.orientation.x = 0.0
        marker_temp.pose.orientation.y = 0.0
        marker_temp.pose.orientation.z = 0.0
        marker_temp.pose.orientation.w = 1.0
        marker_temp.scale.x = 0.01
        marker_temp.scale.y = 0.01
        marker_temp.scale.z = 0.01
        marker_temp.color.a = 1.0 # Don't forget to set the alpha!
        marker_temp.color.r = 1.0
        marker_temp.color.g = 0.0
        marker_temp.color.b = 0.0
        marker_temp.points = []
        for i in range(points.shape[0]):
            marker_temp.points.append(ros_numpy.msgify(Point, points[i][0]))
            marker_temp.points.append(ros_numpy.msgify(Point, points[i][1]))
        
        self.vis_pub.publish( marker_temp )

    def callback(self, msg: PointCloud2):
        # converting ROS message to dense numpy array
        # print("1"+"--- %s seconds ---" % (time.time() - start_time))
        data = ros_numpy.numpify(msg)

        if(self.use_gripper_prior):
            left_data = self.listener_left.get()
            right_data = self.listener_right.get()
        if(self.use_pickle):
            try:
                color_img = self.input_data['colour_img'][self.count]
                point_cloud_img = self.input_data['point_cloud'][self.count]
            except IndexError:
                sys.exit()

        else:
            arr = ros_numpy.point_cloud2.split_rgb_field(data)
            point_cloud_img = structured_to_unstructured(arr[['x', 'y', 'z']])
            color_img = structured_to_unstructured(arr[['r', 'g', 'b']])

        mask_img = self.key_func(point_cloud_img, color_img)

        if(self.use_gripper_prior):
            left_gripper = [left_data.transform.translation.x,left_data.transform.translation.y,left_data.transform.translation.z]
            right_gripper = [right_data.transform.translation.x,right_data.transform.translation.y,right_data.transform.translation.z]
            prior_pos = np.array([left_gripper, right_gripper])
            self.optimizer.set_prior(prior_pos=prior_pos, prior_idx=self.gripper_prior_idx)

        # invoke tracker
        tracking_result, violate_points_1, violate_points_2 = self.cdcpd.step(point_cloud=point_cloud_img,
                                     mask=mask_img, 
                                     cpd_param=self.cpd_params)
    
        # print("4"+"--- %s seconds ---" % (time.time() - start_time))
        # converting tracking result to ROS message
        if tracking_result.dtype is not np.float32:
            tracking_result = tracking_result.astype(np.float32)
        if(self.visualize_violations):
            if violate_points_1.dtype is not np.float32:
                violate_points_1 = violate_points_1.astype(np.float32)
            if violate_points_2.dtype is not np.float32:
                violate_points_2 = violate_points_2.astype(np.float32)
        
        out_struct_arr = unstructured_to_structured(tracking_result, names=['x', 'y', 'z'])
        pub_msg = ros_numpy.msgify(PointCloud2, out_struct_arr)
        pub_msg.header = msg.header
        # print("5"+"--- %s seconds ---" % (time.time() - start_time))
        self.pub.publish(pub_msg)
        # print("6"+"--- %s seconds ---" % (time.time() - start_time))
        if(self.visualize_violations):
            self.display_violations_1(violate_points_1, msg)
            self.display_violations_2(violate_points_2, msg)
        self.count+=1


def main():
    rospy.init_node('cdcpd_node')
    tracker = Tracker(object_name = get_ros_param(param_name="deformable_type", default="rope"))
    rospy.spin()

main()
