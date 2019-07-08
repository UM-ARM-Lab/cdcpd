import cv2
import numpy as np
import numexpr as ne
import matplotlib.pyplot as plt


def chroma_key_rope(points, colors, table, offset):
    hsv_img = cv2.cvtColor(colors.astype(np.float32) / 255.0, code=cv2.COLOR_RGB2HSV)
    hsv_img[:, :, 0] /= 360.0
    h, s, v = np.transpose(hsv_img, axes=[2, 0, 1])
    points_z = points[:, :, 0]
    mask = ne.evaluate("(h > 0.85) & (s > 0.5) & ~(points_z != points_z)")
    return mask, points

def chroma_key_mflag_lab(points, colors, table, offset):
    table = np.reshape(table, (4,4))
    sh = points.shape
    points = points.transpose(2,0,1).reshape(points.shape[2], -1)
    temp = np.ones((1,points.shape[1]))
    points = np.append(points, temp, axis=0)
    point_cloud = np.matmul(table, points)
    point_cloud = point_cloud[:3, :]
    point_cloud = point_cloud.transpose(1,0)
    point_cloud = point_cloud.reshape(sh)
    hsv_img = cv2.cvtColor(colors.astype(np.float32) / 255.0, code=cv2.COLOR_RGB2HSV)
    hsv_img[:, :, 0] /= 360.0
    h, s, v = np.transpose(hsv_img, axes=[2, 0, 1])
    x, y, z = np.transpose(point_cloud, axes=[2, 0, 1])
    x = x - offset[0]
    y = y - offset[1]
    z = z - offset[2]
    points_z = point_cloud[:,:,2]
    #mask = ne.evaluate(
    #    """((((0.43 < h) & (h < 0.63)) | ((0.2 < h) & (h < 0.35))) & \
    #     (0.22 < s) & (s < 0.7) & (((0.03 < v) & (v < 0.3)) | ((0.22 < v) & (v < 0.65))) \
    #     | ((0.13 < h) & (h < 0.19) & (0.2 < s) & (s < 0.9))) \
    #     & ~(points_z != points_z) &\
    #      ~((0.115 < v) & (v < 0.16)) & ((x < 0.3) & (x > -0.3) & (y < 0.5) & (y > -0.5) & ((z - 0.15) < 0.15) & ((z - 0.15) > -0.2)) """)

    # NaN filter, and box filter around current estimate of deformable object position
    xyz_mask = ne.evaluate("""~(points_z != points_z) \
            & ((x < 0.25) & (x > -0.25) & (y < 0.3) & (y > -0.3) & (z < 0.05) & (z > -0.05))""")

    # Filter out the yellow M, while avoiding the white table
    m_logo_mask = ne.evaluate("""(v > 0.5) & (v < 0.65)""")

    # Filter out the blue background, while avoiding????
    m_background_mask = ne.evaluate("""(v < 0.3)""")

    #Filter red hue from green background
    m_hue_mask = ne.evaluate("""(h < 1.0) & (h > 0.9) """)

    # plt.imshow(mask)
    # plt.show()

    mask = ne.evaluate("""xyz_mask & m_hue_mask""")
    return mask, point_cloud


def project_image_space(points, intrinsic_mat):
    projected = points @ intrinsic_mat.T #@ is equivalent to matmul
    # projected = points @ np.linalg.inv(intrinsic_mat)
    projected[:, 0] /= projected[:, 2]
    projected[:, 1] /= projected[:, 2]
    return projected
