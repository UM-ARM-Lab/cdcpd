import cv2
import numpy as np
import numexpr as ne
import matplotlib.pyplot as plt


def chroma_key_rope(points, colors, lower_bound, upper_bound):
    hsv_img = cv2.cvtColor(colors.astype(np.float32) / 255.0, code=cv2.COLOR_RGB2HSV)
    hsv_img[:, :, 0] /= 360.0
    h, s, v = np.transpose(hsv_img, axes=[2, 0, 1])
    x, y, z = np.transpose(points, axes=[2, 0, 1])
    x_lower = lower_bound[0] - x
    y_lower = lower_bound[1] - y
    z_lower = lower_bound[2] - z
    x_upper = x - upper_bound[0]
    y_upper = y - upper_bound[1]
    z_upper = z - upper_bound[2]
    points_z = points[:, :, 0]
     # NaN filter, and box filter around current estimate of deformable object position
    xyz_mask = ne.evaluate("""~(points_z != points_z) \
            & ((x_lower < 0.1) & (x_upper < 0.1) & (y_lower < 0.1) & (y_upper < 0.1) & (z_lower < 0.1) & (z_upper < 0.1))""")
    #Filter red rope
    red_rope_mask = ne.evaluate("(h > 0.85) & (s > 0.5)")
    mask = ne.evaluate("""xyz_mask & red_rope_mask""")
    return mask

def chroma_key_mflag_lab(points, colors, lower_bound, upper_bound):
    hsv_img = cv2.cvtColor(colors.astype(np.float32) / 255.0, code=cv2.COLOR_RGB2HSV)
    hsv_img[:, :, 0] /= 360.0
    h, s, v = np.transpose(hsv_img, axes=[2, 0, 1])
    x, y, z = np.transpose(points, axes=[2, 0, 1])
    x_lower = lower_bound[0] - x
    y_lower = lower_bound[1] - y
    z_lower = lower_bound[2] - z
    x_upper = x - upper_bound[0]
    y_upper = y - upper_bound[1]
    z_upper = z - upper_bound[2]
    points_z = points[:,:,2]

    # NaN filter, and box filter around current estimate of deformable object position
    xyz_mask = ne.evaluate("""~(points_z != points_z) \
            & ((x_lower < 0.1) & (x_upper < 0.1) & (y_lower < 0.1) & (y_upper < 0.1) & (z_lower < 0.1) & (z_upper < 0.1))""")

    # Filter out the yellow M, while avoiding the white table
    m_logo_mask = ne.evaluate("""(v > 0.5) & (v < 0.65)""")

    # Filter out the blue background, while avoiding????
    m_background_mask = ne.evaluate("""(v < 0.3)""")

    #Filter red hue from green background
    m_hue_mask = ne.evaluate("""(h < 1.0) & (h > 0.9) """)

    # plt.imshow(mask)
    # plt.show()

    mask = ne.evaluate("""xyz_mask & m_hue_mask""")
    return mask, points


def project_image_space(points, intrinsic_mat):
    projected = points @ intrinsic_mat.T #@ is equivalent to matmul
    # projected = points @ np.linalg.inv(intrinsic_mat)
    projected[:, 0] /= projected[:, 2]
    projected[:, 1] /= projected[:, 2]
    return projected
