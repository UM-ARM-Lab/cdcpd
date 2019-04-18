import numpy as np
import cv2
from cdcpd.pcl_features import vfh
from .cv_utils import project_image_space


class KnnLibrary:
    def __init__(self, sample_size=1500):
        self.sample_size = sample_size

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)  # or pass empty dictionary
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        self.features = []
        self.templates = []

    def num_features(self):
        return len(self.features)

    def down_sampled_feature(self, filtered_points, feature_func=vfh):
        if filtered_points.shape[0] > self.sample_size:
            rand_idx = np.random.randint(0, filtered_points.shape[0], size=sample_size, dtype=np.uint32)
            filtered_points = filtered_points[rand_idx]
        feature = feature_func(filtered_points)
        return feature

    def knn_idx_match(self, target_feature, k):
        data = np.array(self.features)
        query = target_feature[np.newaxis, :]

        matches = self.flann.knnMatch(query, data, k=min(k, data.shape[0]))
        return [match.trainIdx for match in matches[0]]

    def add_template(self, filtered_points, template):
        feature = self.down_sampled_feature(filtered_points)
        self.features.append(feature)
        self.templates.append(template)

    def query_template(self, filtered_points, k=8):
        feature = self.down_sampled_feature(filtered_points)
        match_idxs = self.knn_idx_match(feature, k=k)
        return [self.templates[i] for i in match_idxs]


class SmoothFreeSpaceCost:
    def __init__(self, camera_intrinsic, k=1e2):
        self.camera_intrinsic = camera_intrinsic
        self.k = k
        self.point_cloud = None
        self.mask = None

    def set_point_cloud(self, point_cloud, mask):
        self.point_cloud = point_cloud
        self.mask = mask

    def run(self, verts):
        projected = project_image_space(verts, self.camera_intrinsic)
        coords = projected[:, :2].astype(np.int64)
        coords[:, 0] = np.clip(coords[:, 0], 0, self.point_cloud.shape[1] - 1)
        coords[:, 1] = np.clip(coords[:, 1], 1, self.point_cloud.shape[0] - 1)

        depth = projected[:, 2]
        image_depth = self.point_cloud[coords[:, 1], coords[:, 0], 2]
        depth_diff = np.clip(image_depth - depth, 0, np.inf)

        dist_img = cv2.distanceTransform(np.logical_not(self.mask).astype(np.uint8), distanceType=cv2.DIST_L2, maskSize=5)
        dist_to_mask = dist_img[coords[:, 1], coords[:, 0]]

        score = 1 - np.exp(-self.k * dist_to_mask * depth_diff)
        cost = np.nanmean(score)
        return cost
