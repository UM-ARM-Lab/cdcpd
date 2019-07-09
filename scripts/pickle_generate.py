#!/usr/bin/env python3
import pickle
import rospy
import time
from sensor_msgs.msg import PointCloud2
import cdcpd
import cv_utils
import ros_numpy
from ros_wrappers import get_ros_param
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured

file_name_suffix = time.strftime("%Y-%m-%d %H-%M-%S", time.localtime())

class Generator:
    def __init__(self):
        self.point_cloud_img = []
        self.color_img = []
        self.sub = rospy.Subscriber("/kinect2_victor_head/qhd/points", PointCloud2, self.callback, queue_size=1)
    
    def callback(self, msg: PointCloud2):
        # converting ROS message to dense numpy array
        # print("1"+"--- %s seconds ---" % (time.time() - start_time))
        data = ros_numpy.numpify(msg)
        arr = ros_numpy.point_cloud2.split_rgb_field(data)
        point_cloud = structured_to_unstructured(arr[['x', 'y', 'z']])
        self.point_cloud_img.append(point_cloud)
        color = structured_to_unstructured(arr[['r', 'g', 'b']])
        self.color_img.append(color)

    def store(self):
    	input_data = {"colour_img" : self.color_img, "point_cloud" : self.point_cloud_img}
    	pickle.dump(input_data, open("/home/deformtrack/examples/data/"+file_name_suffix+".pk", "wb"))
        
        
def main():
    rospy.init_node('pickle')
    gen = Generator()
    rospy.spin()
    gen.store()

main()