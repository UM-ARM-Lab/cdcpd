import numpy as np
from .cpd import CPDParams, CPD
from .optimizer import Optimizer
from .prior import Prior, UniformPrior
from .lle import locally_linear_embedding
from .failure_recovery import KnnLibrary, SmoothFreeSpaceCost
import copy


class CDCPDParams:
    def __init__(self,
                 prior: Prior,
                 optimizer: Optimizer,
                 down_sample_size=300,
                 use_lle=True,
                 lle_neighbors=8,
                 use_recovery=False,
                 recovery_cost_estimator=None,
                 recovery_knn_k=12,
                 recovery_cost_threshold=1.0):
        """
        Parameters for CDCPD
        :param down_sample_size: Target size for point cloud random down-sample
        :param use_lle: Whether uses LLE loss or not for additional regularization
        :param lle_neighbors: k for k-nearest-neighbors number for LLE
        """
        self.down_sample_size = down_sample_size
        self.prior = prior
        self.optimizer = optimizer
        self.use_lle = use_lle
        self.lle_neighbors = lle_neighbors
        self.use_recovery = use_recovery
        self.recovery_cost_estimator = recovery_cost_estimator
        self.recovery_knn_k = recovery_knn_k
        self.recovery_cost_threshold = recovery_cost_threshold


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
        self.knn_library = None
        if self.cdcpd_params.use_recovery:
            self.knn_library = KnnLibrary()

    def step(self,
             point_cloud,
             mask,
             cpd_param: CPDParams):
        """
        Performs one time-step of ConstrainedDeformableCPD.
        :param point_cloud: (H,W,3) point cloud image in camera frame.
        :param mask: (H, W) binary mask for object of interest in point cloud.
        :param cpd_param: CPD parameters used in current time step. Generally, the first time step
        in a tracking sequence should have less regularization (lower beta and lambd) than others.
        :param optimizer: Optimizer used for constrained optimization, subclass of Optimizer.
        :param prior:
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

        if self.cdcpd_params.prior is not None:
            prior = self.cdcpd_params.prior
            prior.set_point_cloud(point_cloud, mask)
            curr_cpd_param.Y_emit_prior = prior.run(self.template)

        # CPD
        cpd = CPD(down_sampled_points, self.template, curr_cpd_param)
        cpd_result = cpd.run()
        tracking_result = cpd_result

        # Optimization
        if self.cdcpd_params.optimizer is not None:
            optimizer = self.cdcpd_params.optimizer
            optimization_result = optimizer.run(tracking_result)
            tracking_result = optimization_result

        # skipping recovery if not enabled
        if not self.cdcpd_params.use_recovery:
            # set template for next step
            self.template = tracking_result
            return tracking_result

        # Failure recovery
        cost_estimator = self.cdcpd_params.recovery_cost_estimator
        cost_estimator.set_point_cloud(point_cloud, mask)
        tracking_failure_index = cost_estimator.run(tracking_result)

        if tracking_failure_index > self.cdcpd_params.recovery_cost_threshold\
            and self.knn_library.num_features() > self.cdcpd_params.recovery_knn_k:
            min_index = tracking_failure_index
            final_tracking_result = tracking_result
            matched_templates = self.knn_library.query_template(down_sampled_points, k=self.cdcpd_params.recovery_knn_k)
            for template in matched_templates:
                # CPD
                cpd = CPD(down_sampled_points, template, curr_cpd_param)
                cpd_result = cpd.run()
                tracking_result = cpd_result

                # optimization
                if self.cdcpd_params.optimizer is not None:
                    optimizer = self.cdcpd_params.optimizer
                    optimization_result = optimizer.run(tracking_result)
                    tracking_result = optimization_result

                tracking_failure_index = cost_estimator.run(tracking_result)
                if tracking_failure_index < min_index:
                    min_index = tracking_failure_index
                    final_tracking_result = tracking_result
            tracking_result = final_tracking_result
            tracking_failure_index = min_index
        else:
            self.knn_library.add_template(down_sampled_points, tracking_result)

        self.template = tracking_result
        return tracking_result
