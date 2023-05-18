from abc import ABC, abstractmethod
from enum import Enum
from typing import Dict, Union
from pathlib import Path

import rosbag
import ros_numpy
import numpy as np
import torch
from cv_bridge import CvBridge
import open3d as o3d

DATA_NAME_TO_TOPIC = {
    "cpd_output": "/cdcpd/template",
    "downsampled_point_cloud": "/cdcpd/downsampled",
    "masked_point_cloud": "/cdcpd/masked",
    "gurobi_output": "/cdcpd/output"
}
# TOPIC_TO_DATA_NAME = {v: k for k, v in DATA_NAME_TO_TOPIC.items()}

CVBRIDGE = CvBridge()

class BagDataContainer:
    """Base class for containers of CDCPD bag data"""

    def __init__(self, topic: str, device: str="cpu", dtype: torch.dtype=torch.double):
        self.topic = topic
        self.times = []
        self.data = []
        self.device = device
        self.dtype = dtype

    def tensorfy(self):
        self._tensorfy_data()
        self._tensorfy_times()

    def read_msg(self, msg):
        self._read_msg_data(msg)
        self._read_msg_timestamp(msg)

    @abstractmethod
    def _read_msg_data(self, msg):
        pass

    @abstractmethod
    def _stack_data(self):
        """Stacks data list into single numpy array"""
        pass

    @abstractmethod
    def _tensorfy_data(self):
        pass

    def _read_msg_timestamp(self, msg):
        self.times.append(msg.header.stamp.to_nsec())

    def _tensorfy_times(self):
        self.times = torch.tensor(self.times, device=self.device, dtype=torch.long)

class BagDataContainerCloud(BagDataContainer):
    """CDCPD bag point cloud container"""

    def __init__(self, topic: str, device: str="cpu", dtype: torch.dtype=torch.double):
        super().__init__(topic, device, dtype)

    def visualize_frame(self, frame_idx: int):
        """Visualizes the point could at the given frame_idx using open3d"""
        dat = self.data[frame_idx]
        pcd = o3d.geometry.PointCloud()
        # Have to transpose points as open3d is expected (num_points, 3) shape
        pcd.points = o3d.utility.Vector3dVector(dat['xyz'].T)
        if 'rgb' in dat.keys():
            # open3d expects color values to be floats between [0, 1]
            rgb_floats = dat['rgb'].astype(float) / 255.
            pcd.colors = o3d.utility.Vector3dVector(rgb_floats.T)

        o3d.visualization.draw_geometries([pcd])

    def _read_msg_data(self, msg):
        # Determine if the point cloud has RGB fields.
        is_rgb_cloud = False
        for f in msg.fields:
            if f.name == 'rgb':
                is_rgb_cloud = True

        if not is_rgb_cloud:
            xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
            # xyz_array = torch.tensor(xyz_array, dtype=self.dtype, device=self.device)
            self.data.append({"xyz": xyz_array.T})
        else:
            xyzrgb_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            # self.data.append(xyzrgb_array)
            x = xyzrgb_array['x']
            y = xyzrgb_array['y']
            z = xyzrgb_array['z']
            rgb = xyzrgb_array['rgb']

            # Find valid indices (where XYZ not nan).
            valid_idxs = ~np.isnan(x)

            xyz_array = np.stack((x[valid_idxs], y[valid_idxs], z[valid_idxs]), axis=0)


            # This is some evil bit-shifty work that's aggregated from several internet sources.
            # It seems to work with the Kinect2 bagdata but we'll see if it works with other data.
            # xyz_array = np.zeros((xyzrgb_array.shape))
            # We have to view the array as np.int32 so we can do bit shifting to recover original uint8
            # RGB values.
            rgb_valid = rgb[valid_idxs].view(np.int32)

            red_vals = (rgb_valid & 0x00FF0000) >> 16
            green_vals = (rgb_valid & 0x0000FF00) >> 8
            blue_vals = (rgb_valid & 0x000000FF)

            rgb_array = np.stack((red_vals, green_vals, blue_vals), axis=0)

            self.data.append({
                "xyz": xyz_array,
                "rgb": rgb_array.astype(np.uint8)
            })

    def _stack_data(self):
        """Since point clouds can be ragged, we don't stack the data into a larger array"""
        pass

    def _tensorfy_data(self):
        for i in range(len(self.data)):
            for key, val in self.data[i].items():
                if key == 'rgb':
                    dtype = torch.long
                else:
                    dtype = self.dtype

                self.data[i][key] = torch.from_numpy(val).to(dtype=dtype, device=self.device)


class BagDataContainerImage(BagDataContainer):
    """CDCPD bag image container"""

    def __init__(self, topic: str, device: str="cpu", dtype: torch.dtype=torch.double):
        super().__init__(topic, device, dtype)

    def _read_msg_data(self, msg):
        img = CVBRIDGE.imgmsg_to_cv2(msg).astype(float)
        # img = torch.tensor(img, dtype=self.dtype, device=self.device)
        self.data.append(img)

    def _stack_data(self):
        self.data = np.stack(self.data, axis=0)

    def _tensorfy_data(self):
        self.data = torch.from_numpy(self.data).to(device=self.device, dtype=self.dtype)

class BagDataContainerIntrinsic(BagDataContainer):
    """CDCPD bag camera intrinsic container"""

    def __init__(self, topic: str, device: str="cpu", dtype: torch.dtype=torch.double):
        super().__init__(topic, device, dtype)

    def _read_msg_data(self, msg):
        intrinsic = np.array(msg.P).reshape(3, 4)
        # intrinsic = torch.tensor(intrinsic, dtype=self.dtype, device=self.device)
        self.data.append(intrinsic)

    def _stack_data(self):
        self.data = np.stack(self.data, axis=0)

    def _tensorfy_data(self):
        self.data = torch.from_numpy(self.data).to(device=self.device, dtype=self.dtype)

class BagDataContainerGripperConfig(BagDataContainer):
    """CDCPD bag gripper configuration container"""

    def __init__(self, topic: str, device: str="cpu", dtype: torch.dtype=torch.double):
        super().__init__(topic, device, dtype)

    def _read_msg_data(self, msg):
        # TODO: Should likely find a better way to read this.
        gripper_data_0 = np.array(msg.data.data[:16]).reshape(4, 4)
        gripper_data_1 = np.array(msg.data.data[16:]).reshape(4, 4)

        gripper_configs = np.stack((gripper_data_0, gripper_data_1), axis=0)
        # gripper_configs = torch.tensor(gripper_configs, device=self.device, dtype=self.dtype)

        self.data.append(gripper_configs)

    def _stack_data(self):
        self.data = np.stack(self.data, axis=0)

    def _tensorfy_data(self):
        self.data = torch.from_numpy(self.data).to(device=self.device, dtype=self.dtype)

class BagData:
    """Reads CDCPD data stored in ROS bags"""

    def __init__(self, bag_filepath: str, rgb_topic: str=None, depth_topic: str=None,
                 mask_topic: str=None, intrinsic_topic: str=None, raw_clouds_topic: str=None,
                 segmented_clouds_topic: str=None, downsampled_clouds_topic: str=None,
                 gurobi_output_topic: str=None, gripper_configs_topic: str=None):
        """Reads CDCPD data stored in ROS bags based on given topics"""
        self.bag_filepath = bag_filepath
        self.rgb_imgs = BagDataContainerImage(rgb_topic)
        self.depth_imgs = BagDataContainerImage(depth_topic)
        self.mask_imgs = BagDataContainerImage(mask_topic)
        self.camera_intrinsics = BagDataContainerIntrinsic(intrinsic_topic)
        self.raw_clouds = BagDataContainerCloud(raw_clouds_topic)
        self.segmented_clouds = BagDataContainerCloud(segmented_clouds_topic)
        self.downsampled_segmented_clouds = BagDataContainerCloud(downsampled_clouds_topic)
        self.gurobi_output = BagDataContainerCloud(gurobi_output_topic)
        self.gripper_configs = BagDataContainerGripperConfig(gripper_configs_topic)

        # Internal storage to make things easier.
        self.__data_list = [
            self.rgb_imgs,
            self.depth_imgs,
            self.mask_imgs,
            self.camera_intrinsics,
            self.raw_clouds,
            self.segmented_clouds,
            self.downsampled_segmented_clouds,
            self.gurobi_output,
            self.gripper_configs
        ]
        # Trim the data list based on if a topic was given for the data.
        # self.__data_list = [d for d in self.__data_list if d.topic is not None]

        self.__topic_to_data_mapping = {d.topic: d for d in self.__data_list}

        self._read()

    def get_all_topics(self):
        return [d.topic for d in self.__data_list if d.topic is not None]

    def _read(self):
        all_topics = self.get_all_topics()
        print("Reading data from topics:", all_topics)
        with rosbag.Bag(self.bag_filepath, 'r') as bag:
            for topic, msg, _ in bag.read_messages(topics=all_topics):
                data_container = self.__topic_to_data_mapping[topic]
                data_container.read_msg(msg)

        # Print what was read in.
        for d in self.__data_list:
            if d.topic is None:
                continue
            print(f"Read {len(d.data)} messages from topic '{d.topic}'", flush=True)

        self._trim_bagdata_to_same_length()
        self._stack_data()

    def tensorfy(self):
        # raise NotImplementedError("Getting crazy tensor conversion hangs when doing this.")
        for d in self.__data_list:
            if d.topic is None:
                continue
            print("Tensorfying:", d.topic, flush=True)
            d.tensorfy()

    def _trim_bagdata_to_same_length(self):
        """Trims data such that they're all the same length

        This should probably be improved based on time matching versus just cutting off the ending
        data.
        """
        list_lengths = [len(d.data) for d in self.__data_list if d.topic is not None]
        min_len = min(list_lengths)
        print("Trimming data to length =", min_len)
        for d in self.__data_list:
            if d.topic is None:
                continue

            # Should probably do more sophisticated matching of timestamps here.
            d.data = d.data[:min_len]
            d.times = d.times[:min_len]

    def _stack_data(self):
        """Stacks data in all containers into single arrays (except point clouds)"""
        for d in self.__data_list:
            if d.topic is None:
                continue
            d._stack_data()


#-----------------------------------------------
# Old functional methods to read bag data
#-----------------------------------------------

def read_cdcpd_data(bagfile_path, name_to_topic_mapping=DATA_NAME_TO_TOPIC):
    """Reads data from CDCPD bagfiles from topics in `DATA_NAME_TO_TOPIC`"""
    topic_to_name_mapping = {v: k for k, v in name_to_topic_mapping.items()}
    bag_data = {key: {"times": [], "cloud": []} for key in name_to_topic_mapping.keys()}

    with rosbag.Bag(bagfile_path, 'r') as bag:
        for topic, msg, _ in bag.read_messages(topics=list(name_to_topic_mapping.values())):
            t = msg.header.stamp

            xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

            data_name = topic_to_name_mapping[topic]
            bag_data[data_name]["times"].append(t.to_nsec())
            bag_data[data_name]["cloud"].append(xyz_array.T)

    return bag_data

def trim_bagdata_to_same_length(bag_data):
    """Chop off the data according to the size of the smallest list

    This is since publishers don't necessarily stop at the same time
    """
    list_lengths = [len(d["times"]) for d in bag_data.values()]
    list_lengths = [l for l in list_lengths if l != 0]
    min_len = min(list_lengths)

    print("Initial CDCPD Output Lengths:")
    for name, d in bag_data.items():
        print('\t', name, len(d["times"]))

    for k, d in bag_data.items():
        if len(d["times"]) > 0:
            times_trunc = d["times"][:min_len]
            cloud_trunc = d["cloud"][:min_len]

            bag_data[k]["times"] = times_trunc
            bag_data[k]["cloud"] = cloud_trunc

    print("CDCPD Output Lengths After Truncation:")
    for name, d in bag_data.items():
        print('\t', name, len(d["times"]))

    return bag_data

def convert_bagdata_to_tensors(bag_data: Dict, device: str, dtype: torch.dtype) -> Dict:
    """Converts all numpy arrays/lists in bag_data to `torch.Tensor`s"""
    for k, d in bag_data.items():
        # Converting even len == 0 lists to tensors just for a unified data structure.
        bag_data[k]["times"] = torch.tensor(bag_data[k]["times"], device=device, dtype=dtype)
        bag_data[k]["cloud"] = [
            torch.tensor(c, device=device, dtype=dtype) for c in bag_data[k]["cloud"]
        ]

def read_sensor_frame_id(bagfile_path):
    """Reads the sensor frame ID (for RVIZ display) of the first available message"""
    with rosbag.Bag(bagfile_path, 'r') as bag:
        # The time of the first message.
        t_init = None
        for topic, msg, _ in bag.read_messages(topics=list(DATA_NAME_TO_TOPIC.values())):
            frame_id = msg.header.frame_id
            break
    return frame_id
