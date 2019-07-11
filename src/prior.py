import numpy as np
import cv2
from cv_utils import project_image_space
import matplotlib.pyplot as plt

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
    def __init__(self, intrinsic_mat, k=1e1):
        """
        Constructor
        :param intrinsic_mat: (3, 3) numpy array for camera intrinsics matrix
        :param k: controls flatness of exp(-k*x). The larger the more tolerant.
        """

        self.intrinsic_mat = intrinsic_mat
        self.k = k
        self.point_cloud = None
        self.mask = None

    def set_point_cloud(self, point_cloud, mask):
        self.point_cloud = point_cloud
        self.mask = mask

    def run(self, verts):
        """
        :return: The probability that a point in verts generated *any* point in self.mask
                 Note that this probability is not normalized, as CDCPD does not need it
                 normalized in order to reach the correct result (minimization does not
                 care about scale factors)
        """
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
        depth_factor = np.maximum(depth_diff, 0.0)

        dist_img = cv2.distanceTransform(
            np.logical_not(self.mask).astype(np.uint8), distanceType=cv2.DIST_L2, maskSize=5)
        cv2.normalize(dist_img, dist_img, 0, 1.0, cv2.NORM_MINMAX)
        dist_to_mask = dist_img[coords[:, 1], coords[:, 0]]

        score = dist_to_mask * depth_factor
        prob = np.exp(-self.k * score)
        # plt.subplot(3,1,1)
        # plt.plot(prob)

        # plt.subplot(3,1,2)
        # plt.plot(depth_factor)
        # plt.subplot(3,1,3)
        # plt.plot(dist_to_mask)
        # plt.draw()
        # plt.pause(0.01)
        # plt.clf()
        return prob
