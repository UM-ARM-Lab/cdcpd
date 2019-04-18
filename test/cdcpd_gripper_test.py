import numpy as np
import pickle
from cdcpd.cdcpd import CDCPDParams, ConstrainedDeformableCPD
from cdcpd.cpd import CPDParams
from cdcpd.prior import UniformPrior
from cdcpd.optimizer import PriorConstrainedOptimizer
from cdcpd.geometry_utils import build_line
from cdcpd.cv_utils import chroma_key_rope
from cdcpd.visualization import simple_visualize


class InputPack:
    def __init__(self, input_arr):
        self.input_arr = input_arr

    def __len__(self):
        return len(self.input_arr)

    def __getitem__(self, i):
        return self.input_arr[i][:2]


def main():
    file_path = "data/gripper_touch.pk"
    input_arr = pickle.load(open(file_path, 'rb'), encoding='bytes')
    template_verts, template_edges = build_line(1.0, 50)
    key_func = chroma_key_rope

    cpd_params = CPDParams()

    prior = UniformPrior()
    optimizer = PriorConstrainedOptimizer(template=template_verts, edges=template_edges)
    cdcpd_params = CDCPDParams(prior=prior, optimizer=optimizer)

    cdcpd = ConstrainedDeformableCPD(template=template_verts,
                                     cdcpd_params=cdcpd_params)

    tracking_result_history = []
    for i in range(len(input_arr)):
        point_cloud_img, color_img, left_gripper, right_gripper = input_arr[i]
        mask_img = key_func(point_cloud_img, color_img)

        prior_pos = np.array([left_gripper[:3, 3], right_gripper[:3, 3]])
        prior_idx = [0, 49]
        optimizer.set_prior(prior_pos=prior_pos, prior_idx=prior_idx)

        tracking_result = cdcpd.step(point_cloud=point_cloud_img,
                                     mask=mask_img,
                                     cpd_param=cpd_params)

        tracking_result_history.append(tracking_result)

    input_pack = InputPack(input_arr)
    simple_visualize(input_arr=input_pack,
                     tracking_result_history=tracking_result_history,
                     template_edges=template_edges,
                     key_func=key_func,
                     filter_point_cloud=False,
                     target_fps=10)


main()
