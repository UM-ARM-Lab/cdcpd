import numpy as np
import gurobipy
from itertools import product


class Optimizer:
    def run(self, verts):
        """
        Optimization method called by CDCPD.
        :param verts: (M, 3) vertices to be optimized. Generally use output of CPD.
        :return: (M, 3) optimization result. Same shape as input.
        """
        return verts


class DistanceConstrainedOptimizer(Optimizer):
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
        target = verts
        constrain_consts = np.square(
            self.stretch_coefficient * np.linalg.norm(
                self.template[self.edges[:, 0]] - self.template[self.edges[:, 1]], axis=1))
        # constrained optimization
        model = gurobipy.Model()
        model.setParam('OutputFlag', False)
        template_vars = model.addVars(*verts.shape, lb=-gurobipy.GRB.INFINITY, name='template')
        for i in range(self.edges.shape[0]):
            left, right = self.edges[i]
            diff = [template_vars[left, j] - template_vars[right, j] for j in range(verts.shape[1])]
            qexpr = gurobipy.quicksum([diff[j] * diff[j] for j in range(len(diff))])
            model.addQConstr(qexpr, gurobipy.GRB.LESS_EQUAL, rhs=constrain_consts[i], name='edge{}'.format(i))
        diff = [template_vars[i, j] - target[i, j] for i, j in
                product(range(verts.shape[0]), range(verts.shape[1]))]
        objective_expr = gurobipy.quicksum([diff[i] * diff[i] for i in range(len(diff))])
        model.setObjective(objective_expr, gurobipy.GRB.MINIMIZE)
        model.update()
        model.optimize()
        optimize_result = np.empty(verts.shape, dtype=verts.dtype)
        for i in range(verts.shape[0]):
            for j in range(verts.shape[1]):
                optimize_result[i, j] = template_vars[i, j].x
        return optimize_result


class PriorConstrainedOptimizer(Optimizer):
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
        target = verts
        constrain_consts = np.square(
            self.stretch_coefficient * np.linalg.norm(
                self.template[self.edges[:, 0]] - self.template[self.edges[:, 1]], axis=1))
        # constrained optimization
        model = gurobipy.Model()
        model.setParam('OutputFlag', False)
        template_vars = model.addVars(*verts.shape, lb=-gurobipy.GRB.INFINITY, name='template')
        for i in range(self.edges.shape[0]):
            left, right = self.edges[i]
            diff = [template_vars[left, j] - template_vars[right, j] for j in range(verts.shape[1])]
            qexpr = gurobipy.quicksum([diff[j] * diff[j] for j in range(len(diff))])
            model.addQConstr(qexpr, gurobipy.GRB.LESS_EQUAL, rhs=constrain_consts[i], name='edge{}'.format(i))

        for i in range(len(self.prior_idx)):
            idx = self.prior_idx[i]
            assert (0 <= idx < verts.shape[0])
            model.addConstrs((template_vars[idx, j] == self.prior_pos[i, j] for j in range(verts.shape[1])))

        diff = [template_vars[i, j] - target[i, j] for i, j in
                product(range(verts.shape[0]), range(verts.shape[1]))]
        objective_expr = gurobipy.quicksum([diff[i] * diff[i] for i in range(len(diff))])
        model.setObjective(objective_expr, gurobipy.GRB.MINIMIZE)
        model.update()
        model.optimize()
        optimize_result = np.empty(verts.shape, dtype=verts.dtype)
        for i in range(verts.shape[0]):
            for j in range(verts.shape[1]):
                optimize_result[i, j] = template_vars[i, j].x
        return optimize_result
