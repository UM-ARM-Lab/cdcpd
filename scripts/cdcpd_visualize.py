#!/home/dmcconac/miniconda3/envs/catkin_cdcpd/bin/python3

import time
import sys
import rospy
import pickle
import numpy as np
import ros_numpy
import IPython
from std_msgs.msg import *
from copy import deepcopy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud, PointCloud2
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import *
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
from cdcpd.cdcpd_solver import CDCPDParams, ConstrainedDeformableCPD
from cdcpd.cpd import CPDParams
from cdcpd.optimizer import EdgeConstrainedOptimizer
from cdcpd.geometry_utils import build_line
from cdcpd.geometry_utils import build_rectangle
from cdcpd.cv_utils import chroma_key_rope
from cdcpd.cv_utils import chroma_key_mflag_lab
from cdcpd.prior import UniformPrior
from cdcpd.prior import ThresholdVisibilityPrior
from cdcpd.failure_recovery import SmoothFreeSpaceCost
from cdcpd.ros_wrappers import Listener
from cdcpd.ros_wrappers import get_ros_param
from gurobipy import GurobiError
import matplotlib.pyplot as plt
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
        self.kinect_intrinsics = np.array([[1068.842896477257, 0.0, 950.2974736758024],
                                           [0.0, 1066.0150152835104, 537.097974092338],
                                           [0.0, 0.0, 1.0]], dtype=np.float32)
        self.kinect_intrinsics[:2] /= 2.0  # TODO: why are we dividing by 2?
        self.listener_left = None
        self.listener_right = None
        self.cost_estimator = None
        self.object_name = object_name
        self.tracking_result = None

        self.use_pickle = get_ros_param(param_name="~use_pickle", default=False)
        self.use_gripper_prior = get_ros_param(param_name="~use_gripper_prior", default=False)
        self.use_passingthru_constraint = get_ros_param(param_name="~use_passingthru_constraint", default=False)
        self.visualize_violations = get_ros_param(param_name="~visualize_violations", default=False)

        if self.use_pickle:
            self.input_data = pickle.load(open("/home/deformtrack/examples/data/flickertest_cloth2.pk", "rb"))

        if self.object_name == "rope":
            rope_num_links = get_ros_param(param_name="rope_num_links", default=10)
            rope_length = get_ros_param(param_name="rope_length", default=18.0 * 0.0254)
            self.template_rows = rope_num_links
            self.template_cols = 1
            self.template_verts, self.template_edges = build_line(rope_length, rope_num_links)
            self.key_func = chroma_key_rope
        elif self.object_name == "cloth":
            self.template_rows = get_ros_param(param_name="cloth_num_control_points_x", default=20)
            self.template_cols = get_ros_param(param_name="cloth_num_control_points_y", default=27)
            self.template_verts, self.template_edges = build_rectangle(
                    width=get_ros_param(param_name="cloth_x_size", default=0.32),
                    height=get_ros_param(param_name="cloth_y_size", default=0.45),
                    width_num_node=self.template_rows,
                    height_num_node=self.template_cols)
            self.key_func = chroma_key_mflag_lab

        stretch_factor = get_ros_param(param_name="smmap_planner_node/task/max_stretch_factor", default=1.0)
        self.prior = ThresholdVisibilityPrior(self.kinect_intrinsics)
        self.optimizer = EdgeConstrainedOptimizer(
            template=self.template_verts,
            edges=self.template_edges,
            stretch_coefficient=stretch_factor,
            use_gripper_prior=self.use_gripper_prior,
            use_passingthru_constraint=self.use_passingthru_constraint,
            visualize_violations=self.visualize_violations)

        if self.use_gripper_prior:
            print("Using gripper prior")
            self.listener_left = Listener(topic_name="cdcpd/left_gripper_prior", topic_type=TransformStamped)
            self.listener_right = Listener(topic_name="cdcpd/right_gripper_prior", topic_type=TransformStamped)
            self.gripper_prior_idx = [get_ros_param(param_name="right_gripper_attached_node_idx", default=0),
                                      get_ros_param(param_name="left_gripper_attached_node_idx", default=49)]

        self.cpd_params = CPDParams()
        self.down_sample_size = get_ros_param("~down_sample_size", default=300)
        self.cdcpd_params = CDCPDParams(
            prior=self.prior,
            optimizer=self.optimizer,
            template_rows=self.template_rows,
            template_cols=self.template_cols,
            down_sample_size=self.down_sample_size,
            visualize_violations=self.visualize_violations)

        if get_ros_param(param_name="~use_failure_recovery", default=False):
            self.cost_estimator = SmoothFreeSpaceCost(self.kinect_intrinsics)
            self.cdcpd_params.use_recovery = True
            self.cdcpd_params.recovery_cost_estimator = self.cost_estimator
            self.cdcpd_params.recovery_cost_threshold = 0.1

        self.cdcpd = ConstrainedDeformableCPD(template=self.template_verts,
                                              cdcpd_params=self.cdcpd_params)

        # initialize ROS publishers and subscribers - data flows in the order the publishers and subscribers are listed
        point_cloud_topic = get_ros_param(param_name="~input_cloud_topic", default="kinect2_victor_head/qhd/points")
        self.point_cloud_sub = Listener(topic_name=point_cloud_topic, topic_type=PointCloud2)
        self.pub_points = rospy.Publisher("cdcpd/new_point_cloud", PointCloud2, queue_size=10)
        self.sub_sample = Listener(topic_name="cdcpd/mask_down_sampled_points", topic_type=PointCloud2)
        self.pub_filter = rospy.Publisher("cdcpd/mask_filtered_points", PointCloud2, queue_size=10)
        self.pub_tracker = rospy.Publisher("cdcpd/tracker_points", PointCloud2, queue_size=10)
        self.vis_pub = rospy.Publisher("cdcpd/visualization_marker", Marker, queue_size=10)

        # print("--- %s seconds ---" % (time.time() - start_time))

    def display_violations_1(self, points, header):
        # visualize violations
        marker = Marker()
        marker.header = deepcopy(header)
        marker.ns = "stretch"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = []
        for i in range(points.shape[1]):
            marker.points.append(ros_numpy.msgify(Point, points[0][i]))
            marker.points.append(ros_numpy.msgify(Point, points[1][i]))

        self.vis_pub.publish(marker)

    def display_violations_2(self, points, header):
        marker = Marker()
        marker.header = deepcopy(header)
        marker.ns = "crossing"
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = []
        for i in range(points.shape[0]):
            marker.points.append(ros_numpy.msgify(Point, points[i][0]))
            marker.points.append(ros_numpy.msgify(Point, points[i][1]))

        self.vis_pub.publish(marker)

    def display_cube(self, header, lower_bound, upper_bound):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.ns = "filter_bounds"
        marker.id = 2
        marker.action = Marker.ADD
        marker.header = deepcopy(header)
        # markers.header.frame_id = "table_surface"
        marker.scale.x = 0.2 + upper_bound[0] - lower_bound[0]
        marker.scale.y = 0.2 + upper_bound[1] - lower_bound[1]
        marker.scale.z = 0.2 + upper_bound[2] - lower_bound[2]
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.1
        marker.pose.position.x = (lower_bound[0] + upper_bound[0])/2
        marker.pose.position.y = (lower_bound[1] + upper_bound[1])/2
        marker.pose.position.z = (lower_bound[2] + upper_bound[2])/2
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        self.vis_pub.publish(marker)

    def cdcpd_main(self):
        # converting ROS message to dense numpy array
        # print("1"+"--- %s seconds ---" % (time.time() - start_time))

        msg = self.point_cloud_sub.get()
        data = ros_numpy.numpify(msg)
        # table_data = self.listener_table.get()
        # Used to disable the transform if we are so inclined
        # table_data.data = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        if self.use_gripper_prior:
            left_data = self.listener_left.get()
            right_data = self.listener_right.get()
        if self.use_pickle:
            try:
                color_img = self.input_data['colour_img'][self.count]
                point_cloud_img = self.input_data['point_cloud'][self.count]
                plt.imshow(color_img)

                plt.draw()
                plt.pause(0.0001)
                plt.clf()
            except IndexError:
                rospy.signal_shutdown('Reached the end of file')
        else:
            arr = ros_numpy.point_cloud2.split_rgb_field(data)
            point_cloud_img = structured_to_unstructured(arr[['x', 'y', 'z']])
            color_img = structured_to_unstructured(arr[['r', 'g', 'b']])

        if self.tracking_result is None:
            upper_bound = 5 * np.ones((3,))
            lower_bound = -5 * np.ones((3,))
        else:
            upper_bound = self.tracking_result.max(axis=0)
            lower_bound = self.tracking_result.min(axis=0)
            self.display_cube(msg.header, lower_bound, upper_bound)

        mask_img, point_cloud_img = self.key_func(point_cloud_img, color_img, lower_bound, upper_bound)
        filtered_points = point_cloud_img[mask_img]

        # if point_cloud_img.dtype is not np.float32:
        #     point_cloud_img = point_cloud_img.astype(np.float32)
        # out_struct_arr = unstructured_to_structured(point_cloud_img, names=['x', 'y', 'z'])
        # pub_points_msg = ros_numpy.msgify(PointCloud2, out_struct_arr)
        # pub_points_msg.header = msg.header
        # # pub_points_msg.header.frame_id ='table_surface'
        # self.pub_points.publish(pub_points_msg)
        # self.pub_points.publish(pub_points_msg)
        # self.pub_points.publish(pub_points_msg)
        # self.pub_points.publish(pub_points_msg)
        # self.pub_points.publish(pub_points_msg)
        # self.pub_points.publish(pub_points_msg)
        # self.pub_points.publish(pub_points_msg)

        if filtered_points.dtype is not np.float32:
            filtered_points = filtered_points.astype(np.float32)
        out_struct_arr = unstructured_to_structured(filtered_points, names=['x', 'y', 'z'])
        pub_filter_msg = ros_numpy.msgify(PointCloud2, out_struct_arr)
        pub_filter_msg.header = msg.header
        self.pub_filter.publish(pub_filter_msg)
        # X = self.template[:,0]
        # Y = self.template[:,1]
        # Z = self.template[:,2]

        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.scatter(X,Y,Z)
        # plt.show()

        if len(filtered_points) <= len(self.template_verts):
            raise ValueError("Not enough point in masked point cloud.")
            IPython.embed()
        msg_sample = self.sub_sample.get()
        data_sample = ros_numpy.numpify(msg_sample)
        down_sampled_points = structured_to_unstructured(data_sample[['x', 'y', 'z']])

        if self.use_gripper_prior:
            left_gripper = [left_data.transform.translation.x, left_data.transform.translation.y, left_data.transform.translation.z]
            right_gripper = [right_data.transform.translation.x, right_data.transform.translation.y, right_data.transform.translation.z]
            prior_pos = np.array([left_gripper, right_gripper])
            self.optimizer.set_prior(prior_pos=prior_pos, prior_idx=self.gripper_prior_idx)

        # invoke tracker
        self.tracking_result, violate_points_1, violate_points_2 = self.cdcpd.step(
                                     point_cloud=point_cloud_img,
                                     down_sampled_points=down_sampled_points,
                                     mask=mask_img,
                                     cpd_param=self.cpd_params)

        # print("4"+"--- %s seconds ---" % (time.time() - start_time))
        # converting tracking result to ROS message
        if self.tracking_result.dtype is not np.float32:
            self.tracking_result = self.tracking_result.astype(np.float32)

        # Ensure that the result is in the same data order as expected by SMMAP
        reshaped1 = np.reshape(self.tracking_result, (self.template_rows, self.template_cols, 3))
        transposed1 = reshaped1.transpose(1, 0, 2)
        self.tracking_result = np.reshape(transposed1, np.shape(self.template_verts))

        out_struct_arr = unstructured_to_structured(self.tracking_result, names=['x', 'y', 'z'])
        pub_msg = ros_numpy.msgify(PointCloud2, out_struct_arr)
        pub_msg.header = msg.header

        # print("5"+"--- %s seconds ---" % (time.time() - start_time))
        self.pub_tracker.publish(pub_msg)

        # print("6"+"--- %s seconds ---" % (time.time() - start_time))
        if self.visualize_violations:
            if violate_points_1.dtype is not np.float32:
                violate_points_1 = violate_points_1.astype(np.float32)
            if violate_points_2.dtype is not np.float32:
                violate_points_2 = violate_points_2.astype(np.float32)
            self.display_violations_1(violate_points_1, msg.header)
            self.display_violations_2(violate_points_2, msg.header)
        # temp = input("Prompt: ")


def main():
    rospy.init_node('cdcpd_node')
    object_type = get_ros_param(param_name="deformable_type", default="rope")
    tracker = Tracker(object_name=object_type)
    while not rospy.is_shutdown():
        try:
            tracker.cdcpd_main()
        except (ValueError, AttributeError, GurobiError) as ex:
            rospy.logwarn("Error: {0}".format(ex))


if __name__ == "__main__":
    main()
