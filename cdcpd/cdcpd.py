import numpy as np
from .cpd import CPDParams, CPD
from .optimizer import Optimizer
from .lle import locally_linear_embedding
import copy


class CDCPDParams:
    def __init__(self,
                 down_sample_size=300,
                 use_lle=True,
                 lle_neighbors=8):
        """
        Parameters for CDCPD
        :param down_sample_size: Target size for point cloud random down-sample
        :param use_lle: Whether uses LLE loss or not for additional regularization
        :param lle_neighbors: k for k-nearest-neighbors number for LLE
        """
        self.down_sample_size = down_sample_size
        self.use_lle = use_lle
        self.lle_neighbors = lle_neighbors


class ConstrainedDeformableCPD:
    def __init__(self, template,
                 cdcpd_params: CDCPDParams):
        """
        Constructor.
        :param template: (M, 3) template for tracking.
        :param cdcpd_params: Type of CDCPDParams
        """
        self.template = template
        self.cdcpd_params = cdcpd_params
        self.M_LLE = None
        if self.cdcpd_params.use_lle:
            self.M_LLE = locally_linear_embedding(
                self.template, n_neighbors=self.cdcpd_params.lle_neighbors).todense()

    def step(self, point_cloud, mask, cpd_param: CPDParams, optimizer: Optimizer):
        """
        Performs one time-step of ConstrainedDeformableCPD.
        :param point_cloud: (H,W,3) point cloud image in camera frame.
        :param mask: (H, W) binary mask for object of interest in point cloud.
        :param cpd_param: CPD parameters used in current time step. Generally, the first time step
        in a tracking sequence should have less regularization (lower beta and lambd) than others.
        :param optimizer: Optimizer used for constrained optimization, subclass of Optimizer.
        :return: (M, 3) tracking result. Same shape as template.
        """
        filtered_points = point_cloud[mask]
        if len(filtered_points) <= len(self.template):
            raise ValueError("Not enough point in masked point cloud.")

        rand_idx = np.random.randint(0, filtered_points.shape[0],
                                     size=self.cdcpd_params.down_sample_size, dtype=np.uint32)
        down_sampled_points = filtered_points[rand_idx]

        curr_cpd_param = copy.deepcopy(cpd_param)
        if self.cdcpd_params.use_lle is True:
            curr_cpd_param.M_LLE = self.M_LLE

        # CPD
        cpd = CPD(down_sampled_points, self.template, curr_cpd_param)
        cpd_result = cpd.run()

        # Optimization
        optimization_result = optimizer.run(cpd_result)

        # set template for next step
        self.template = optimization_result
        return optimization_result
