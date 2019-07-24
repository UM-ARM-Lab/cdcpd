import numpy as np
import cv2
from .cv_utils import project_image_space


class Prior:
    """
    Base class for prior estimators.
    """
    def set_point_cloud(self, point_cloud, mask):
        """
        Set the point cloud being used for estimation.
        :param point_cloud: (H, W, 3) numpy array for point cloud image received from Kinect
        :param mask: (H, W) bool mask for object of interest. True means pixel belong to object.
        :return:
        """
        pass

    def run(self, verts):
        """
        Estimates prior probability of vertices to be seen.
        :param verts: (M, 3) numpy array of float
        :return: (M,) numpy array of float, between 0 and 1
        """
        pass


class UniformPrior(Prior):
    """
    Generates uniform distribution for visibility probability
    """
    def run(self, verts):
        prior = np.ones(verts.shape[0], dtype=verts.dtype)
        prior /= verts.shape[0]
        return prior


class ThresholdVisibilityPrior(Prior):
    """
    Generates visibility of nodes based on how far away it is from point cloud
    and how far away it is from object mask.
    """
    def __init__(self, intrinsic_mat, k=1e1, saturation_threshold=0.01):
        """
        Constructor
        :param intrinsic_mat: (3, 3) numpy array for camera intrinsics matrix
        :param k: controls flatness of exp(-k*x). The larger the more tolerant.
        :param saturation_threshold: Above this threshold, probability will be filled as 1.
        """

        self.intrinsic_mat = intrinsic_mat
        self.k = k
        self.saturation_threshold = saturation_threshold
        self.point_cloud = None
        self.mask = None

    def set_point_cloud(self, point_cloud, mask):
        self.point_cloud = point_cloud
        self.mask = mask

    def run(self, verts):
        projected = project_image_space(verts, self.intrinsic_mat)
        projected[:, 0] = np.clip(projected[:, 0], 0, self.point_cloud.shape[1])
        projected[:, 1] = np.clip(projected[:, 1], 0, self.point_cloud.shape[0])

        coords = projected[:, :2].astype(np.int64)
        coords[:, 0] = np.clip(coords[:, 0], 0, self.point_cloud.shape[1] - 1)
        coords[:, 1] = np.clip(coords[:, 1], 1, self.point_cloud.shape[0] - 1)

        depth = projected[:, 2]
        image_depth = self.point_cloud[coords[:, 1], coords[:, 0], 2]

        depth_diff = np.clip(depth - image_depth, 0, np.inf)
        # fill nan with positive value to prevent numerical problem
        depth_diff[np.isnan(depth_diff)] = 0.02

        dist_img = cv2.distanceTransform(
            np.logical_not(self.mask).astype(np.uint8), distanceType=cv2.DIST_L2, maskSize=5)
        dist_to_mask = dist_img[coords[:, 1], coords[:, 0]]

        score = dist_to_mask * depth_diff
        prob = np.exp(-self.k * score)
        return prob
