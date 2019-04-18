import numpy as np
import gurobipy
import cdcpd.gurobi_utils as grb_utils


class Optimizer:
    def run(self, verts):
        """
        Optimization method called by CDCPD.
        :param verts: (M, 3) vertices to be optimized. Generally use output of CPD.
        :return: (M, 3) optimization result. Same shape as input.
        """
        return verts


def edge_squared_distances(points, edges):
    diff = points[edges[:, 0]] - points[edges[:, 1]]
    sqr_dist = np.sum(np.square(diff), axis=1)
    return sqr_dist


class DistanceConstrainedOptimizer(Optimizer):
    """
    Performs constrained optimization that optimizes MSE between output and verts,
    subject to constraint that edges length in output is within stretch_coefficient
    of original distance.
    """
    def __init__(self, template, edges, stretch_coefficient=1.0):
        """
        Constructor.
        :param template: (M, 3) template whose edge length used as reference.
        :param edges: (E, 2) integer vertices index list, represent edges in template.
        :param stretch_coefficient: Maximum ratio of out output edge length and
         reference edge length.
        """
        self.template = template
        self.edges = edges
        self.stretch_coefficient = stretch_coefficient

    def run(self, verts):
        model = gurobipy.Model()
        model.setParam('OutputFlag', False)
        g_verts = grb_utils.create_gurobi_arr(model, verts.shape, name="verts")

        # distance constraint
        rhs = (self.stretch_coefficient ** 2) * edge_squared_distances(self.template, self.edges)
        lhs = edge_squared_distances(g_verts, self.edges)
        grb_utils.add_constraints(model, lhs, "<=", rhs, name="edge")

        # objective function
        g_objective = np.sum(np.square(g_verts - verts))
        model.setObjective(g_objective, gurobipy.GRB.MINIMIZE)
        model.update()
        model.optimize()

        verts_result = grb_utils.get_value(g_verts)
        return verts_result


class PriorConstrainedOptimizer(Optimizer):
    """
    Performs constrained optimization that optimizes MSE between output and verts,
    subject to constraint that edges length in output is within stretch_coefficient
    of original distance. It also constraints some specified nodes to be certain
     position known a priori.
    """
    def __init__(self, template, edges, stretch_coefficient=1.0):
        """
        Constructor.
        :param template: (M, 3) template whose edge length used as reference.
        :param edges: (E, 2) integer vertices index list, represent edges in template.
        :param stretch_coefficient: Maximum ratio of out output edge length and
         reference edge length.
        """
        self.template = template
        self.edges = edges
        self.stretch_coefficient = stretch_coefficient
        self.prior_pos = np.zeros(0)
        self.prior_idx = []

    def set_prior(self, prior_pos, prior_idx):
        """
        Set prior correspondence to the optimizer. Should be called in every frame.
        Sets linear constraint prior_pose[i] == verts[prior_idx[i]]
        :param prior_pos: (K, 3) float numpy array representing prior position of nodes.
        :param prior_idx: (K,) int array/list representing corresponding index for the constraint.
        :return:
        """
        assert(len(prior_pos) == len(prior_idx))
        self.prior_pos = prior_pos
        self.prior_idx = prior_idx

    def run(self, verts):
        model = gurobipy.Model()
        model.setParam('OutputFlag', False)
        g_verts = grb_utils.create_gurobi_arr(model, verts.shape, name="verts")

        # distance constraint
        rhs = (self.stretch_coefficient ** 2) * edge_squared_distances(self.template, self.edges)
        lhs = edge_squared_distances(g_verts, self.edges)
        grb_utils.add_constraints(model, lhs, "<=", rhs, name="edge")

        # prior pos constraint
        lhs = g_verts[self.prior_idx]
        rhs = self.prior_pos
        grb_utils.add_constraints(model, lhs, "==", rhs, name="prior")

        # objective function
        g_objective = np.sum(np.square(g_verts - verts))
        model.setObjective(g_objective, gurobipy.GRB.MINIMIZE)
        model.update()
        model.optimize()

        verts_result = grb_utils.get_value(g_verts)
        return verts_result

