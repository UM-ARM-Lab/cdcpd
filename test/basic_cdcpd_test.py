import pickle
from cdcpd.cdcpd import CDCPDParams, ConstrainedDeformableCPD
from cdcpd.cpd import CPDParams
from cdcpd.optimizer import DistanceConstrainedOptimizer
from cdcpd.geometry_utils import build_line
from cdcpd.cv_utils import chroma_key_rope
from cdcpd.visualization import simple_visualize


def main():
    file_path = "data/rope_fast.pk"
    file_path = "data/rope_simple.pk"
    input_arr = pickle.load(open(file_path, 'rb'), encoding='bytes')
    template_verts, template_edges = build_line(1.0, 50)
    key_func = chroma_key_rope

    cpd_params = CPDParams(beta=4.0)
    cdcpd_params = CDCPDParams()
    optimizer = DistanceConstrainedOptimizer(template=template_verts, edges=template_edges)

    cdcpd = ConstrainedDeformableCPD(template=template_verts,
                                     cdcpd_params=cdcpd_params,
                                     optimizer=optimizer)

    tracking_result_history = []
    for i in range(len(input_arr)):
        point_cloud_img, color_img = input_arr[i]
        mask_img = key_func(point_cloud_img, color_img)

        tracking_result = cdcpd.step(point_cloud=point_cloud_img,
                                     mask=mask_img,
                                     cpd_param=cpd_params)

        tracking_result_history.append(tracking_result)

    simple_visualize(input_arr=input_arr,
                     tracking_result_history=tracking_result_history,
                     template_edges=template_edges,
                     key_func=key_func,
                     filter_point_cloud=False,
                     target_fps=10)

main()
