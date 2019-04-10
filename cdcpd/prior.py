import numpy as np

def project_image_space(points, intrinsic_mat):
    projected = points @ intrinsic_mat.T
    # projected = points @ np.linalg.inv(intrinsic_mat)
    projected[:, 0] /= projected[:, 2]
    projected[:, 1] /= projected[:, 2]
    return projected


def visibility_prior(verts, points, mask, intrinsic_mat, threshold=2.0):
    """

    :param verts: nodes of the template
    :param points: point cloud image
    :param mask: binary mask
    :return:
    """
    projected = project_image_space(verts, intrinsic_mat)
    projected[:, 0] = np.clip(projected[:, 0], 0, points.shape[1])
    projected[:, 1] = np.clip(projected[:, 1], 0, points.shape[0])

    coords = projected[:, :2].astype(np.int64)
    coords[:, 0] = np.clip(coords[:, 0], 0, points.shape[1] - 1)
    coords[:, 1] = np.clip(coords[:, 1], 1, points.shape[0] - 1)

    depth = projected[:, 2]
    image_depth = points[coords[:, 1], coords[:, 0], 2]


    depth_diff = np.clip(depth - image_depth, 0, np.inf)
    depth_diff[np.isnan(depth_diff)] = 0.02
    # cost = np.nanmean(depth_diff)

    dist_img = cv2.distanceTransform(np.logical_not(mask).astype(np.uint8), distanceType=cv2.DIST_L2, maskSize=5)
    dist_to_mask = dist_img[coords[:, 1], coords[:, 0]]

    score = dist_to_mask * depth_diff
    # prob = np.clip(score, 0, threshold) / threshold
    # prob = 1 - prob
    k = 1e1
    # print(k*score)
    prob = np.exp(-k*score)
    # print(prob)

    if prob.sum() < 0.01:
        prob[:] = 1
    return prob