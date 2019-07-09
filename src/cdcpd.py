import time
start_time = time.time()
import numpy as np
from cpd import CPDParams, CPD
from optimizer import Optimizer
from prior import Prior
from lle import locally_linear_embedding
from failure_recovery import KnnLibrary
import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

class CDCPDParams:
    def __init__(self,
                 prior: Prior,
                 optimizer: Optimizer,
                 down_sample_size=300,
                 use_lle=True,
                 lle_neighbors=8,
                 use_recovery=False,
                 visualize_violations = False,
                 recovery_cost_estimator=None,
                 recovery_knn_k=12,
                 recovery_featrue_sample_size=1500,
                 recovery_cost_threshold=0.5):
        """
        Parameters for CDCPD
        :param prior: Reference to prior estimator, which estimates
         the prior emission probability for gaussian centroids. subclass of Prior
        :param optimizer: Reference to optimizer used for constrained optimization, subclass of Optimizer.
        :param down_sample_size: Target size for point cloud random down-sample for CPD
        :param use_lle: Whether uses LLE loss or not for additional regularization
        :param lle_neighbors: k for k-nearest-neighbors number for LLE
        :param use_recovery: Whether uses tracking failure recovery or not.
        :param recovery_cost_estimator: Reference to a cost estimator, estimates likelihood of current tracking
        result to be correct. The higher the cost, the lower the likelihood.
        :param recovery_knn_k: k for k-nearest-neighbors of feature library query
        :param recovery_featrue_sample_size: Target size for point cloud random down-sample for feature extractor
        :param recovery_cost_threshold: Threshold for recovery_cost to be deemed as tracking failure.
        """

        self.down_sample_size = down_sample_size
        self.prior = prior
        self.optimizer = optimizer
        self.use_lle = use_lle
        self.lle_neighbors = lle_neighbors
        self.use_recovery = use_recovery
        self.visualize_violations = visualize_violations
        self.recovery_cost_estimator = recovery_cost_estimator
        self.recovery_knn_k = recovery_knn_k
        self.recovery_featrue_sample_size = recovery_featrue_sample_size
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
        #X = self.template[:,0]
        #Y = self.template[:,1]
        #Z = self.template[:,2]

        #fig = plt.figure()
        #ax = fig.add_subplot(111)
        #ax.scatter(X,Y)
        #ax.axis('equal')
        #plt.xlabel('X')
        #plt.ylabel('Y')
        #plt.show()

        self.iteration = 0
        #self.prev_template = None
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
             down_sampled_points,
             mask,
             cpd_param: CPDParams):
        """
        Performs one time-step of ConstrainedDeformableCPD.
        :param point_cloud: (H,W,3) point cloud image in camera frame.
        :param mask: (H, W) binary mask for object of interest in point cloud.
        :param cpd_param: CPD parameters used in current time step. Generally, the first time step
        in a tracking sequence should have less regularization (lower beta and lambd) than others.
        :return: (M, 3) tracking result. Same shape as template.
        """
        # print("11"+"--- %s seconds ---" % (time.time() ))
        curr_cpd_param = copy.deepcopy(cpd_param)
        # print("12"+"--- %s seconds ---" % (time.time() - start_time))
        if self.cdcpd_params.use_lle is True:
            curr_cpd_param.M_LLE = self.M_LLE

        if self.cdcpd_params.prior is not None:
            prior = self.cdcpd_params.prior
            prior.set_point_cloud(point_cloud, mask)
            curr_cpd_param.Y_emit_prior = prior.run(self.template)
        # print("13"+"--- %s seconds ---" % (time.time() ))
        # CPD

        cpd = CPD(down_sampled_points, self.template, curr_cpd_param)
        cpd_result = cpd.run()
        tracking_result = cpd_result
        # print("14"+"--- %s seconds ---" % (time.time() - start_time))
        # Optimization
        if self.cdcpd_params.optimizer is not None:
            optimizer = self.cdcpd_params.optimizer
            optimization_result, violate_points_1, violate_points_2 = optimizer.run(tracking_result, self.template, self.iteration)
            tracking_result = optimization_result.astype(self.template.dtype)
            if(self.cdcpd_params.visualize_violations):
                violate_points_1 = violate_points_1.astype(self.template.dtype)
                violate_points_2 = violate_points_2.astype(self.template.dtype)

        # print("15"+"--- %s seconds ---" % (time.time() - start_time))
        # skipping recovery if not enabled
        if not self.cdcpd_params.use_recovery:
            # set template  for next step
            self.template = tracking_result
            self.iteration+=1
            return tracking_result, violate_points_1, violate_points_2

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
                    optimization_result, violate_points_1, violate_points_2 = optimizer.run(tracking_result, self.template, self.iteration)
                    tracking_result = optimization_result

                tracking_failure_index = cost_estimator.run(tracking_result)
                if tracking_failure_index < min_index:
                    min_index = tracking_failure_index
                    final_tracking_result = tracking_result
            tracking_result = final_tracking_result
            tracking_failure_index = min_index
        else:
            self.knn_library.add_template(down_sampled_points, tracking_result)

        #self.prev_template = template
        self.iteration+=1
        self.template = tracking_result
        return tracking_result, violate_points_1, violate_points_2
