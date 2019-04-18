import numpy as np
import pickle
from cdcpd.cdcpd import CDCPDParams, ConstrainedDeformableCPD
from cdcpd.cpd import CPDParams
from cdcpd.optimizer import DistanceConstrainedOptimizer
from cdcpd.geometry_utils import build_rectangle
from cdcpd.cv_utils import chroma_key_mflag_home
from cdcpd.visualization import simple_visualize
from cdcpd.prior import ThresholdVisibilityPrior
from cdcpd.failure_recovery import SmoothFreeSpaceCost


def main():
    file_path = "data/mflag_fold.pk"
    input_pack = pickle.load(open(file_path, 'rb'), encoding='bytes')
    color_imgs = input_pack["color_img"]
    point_imgs = input_pack["point_img"]
    camera_intrinsic = input_pack["camera_intrinsic"]

    template_verts, template_edges = build_rectangle(0.32, 0.45, grid_size=0.03)
    key_func = chroma_key_mflag_home

    prior = ThresholdVisibilityPrior(camera_intrinsic)
    optimizer = DistanceConstrainedOptimizer(template=template_verts, edges=template_edges)
    cost_estimator = SmoothFreeSpaceCost(camera_intrinsic)

    cpd_params = CPDParams(beta=1.0)
    cdcpd_params = CDCPDParams(prior=prior,
                               optimizer=optimizer,
                               use_recovery=True,
                               recovery_cost_estimator=cost_estimator,
                               recovery_cost_threshold=0.1)
    cdcpd = ConstrainedDeformableCPD(template=template_verts,
                                     cdcpd_params=cdcpd_params)

    tracking_result_history = []
    for i in range(len(point_imgs)):
        point_cloud_img = point_imgs[i]
        color_img = color_imgs[i]
        mask_img = key_func(point_cloud_img, color_img)

        prior.set_point_cloud(point_cloud_img, mask_img)

        tracking_result = cdcpd.step(point_cloud=point_cloud_img,
                                     mask=mask_img,
                                     cpd_param=cpd_params)

        tracking_result_history.append(tracking_result)

    simple_visualize(input_arr=list(zip(point_imgs, color_imgs)),
                     tracking_result_history=tracking_result_history,
                     template_edges=template_edges,
                     key_func=key_func,
                     filter_point_cloud=False,
                     target_fps=10)

main()
