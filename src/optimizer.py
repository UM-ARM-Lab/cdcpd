import numpy as np
import gurobipy
import gurobi_utils as grb_utils
import scipy
from optimize_eqn import opt_equations
from violations import detect_violation
# from scipy.linalg import norm


class Optimizer:
    def run(self, verts):
        """
        Optimization method called by CDCPD.
        :param verts: (M, 3) vertices to be optimized. Generally use output of CPD.
        :return: (M, 3) optimization result. Same shape as input.
        """
        return verts
def squared_norm(points):
    sqr_dist = np.sum(np.square(points))
    return sqr_dist

def isNeighbour(i, j):
    if(np.abs(i-j)<=1):
        return True
    else:
        return False

def DistBetween2Segment(p1, p2, p3, p4):
    u = p1 - p2
    v = p3 - p4
    w = p2 - p4
    a = np.dot(u,u)
    b = np.dot(u,v)
    c = np.dot(v,v)
    d = np.dot(u,w)
    e = np.dot(v,w)
    D = a*c - b*b
    sD = D
    tD = D
    case = 1
    comp1 = (u[0]*v[1]-u[1]*v[0])
    # comp2 = abs(u[2]*v[1]-u[1]*v[2])
    # comp3 = abs(u[0]*v[2]-u[2]*v[0])
    SMALL_NUM = 0.00000001
    
    # compute the line parameters of the two closest points
    #if (D < SMALL_NUM): # the lines are almost parallel
    #if(comp1<SMALL_NUM and comp2<SMALL_NUM and comp3<SMALL_NUM):
    if(comp1<SMALL_NUM and comp1>-SMALL_NUM):
        sN = 0.0       #force using point P0 on segment S1
        sD = 1.0       #to prevent possible division by 0.0 later
        tN = e
        tD = c
        case = 2
    else:                # get the closest points on the infinite lines
        sN = (b*e - c*d)
        tN = (a*e - b*d)
        if (sN < 0.0):   # sc < 0 => the s=0 edge is visible       
            sN = 0.0
            tN = e
            tD = c
            case = 2
        elif (sN > sD):# sc > 1 => the s=1 edge is visible
            sN = sD
            tN = e + b
            tD = c
            case = 3

    if (tN < 0.0):            #tc < 0 => the t=0 edge is visible
        tN = 0.0
        # recompute sc for this edge
        if (-d < 0.0):
            case = 5
            sN = 0.0
        elif (-d > a):
            case = 6
            sN = sD
        else:
            case = 4
            sN = -d
            sD = a

    elif (tN > tD):       # tc > 1 => the t=1 edge is visible
        tN = tD
        # recompute sc for this edge
        if ((-d + b) < 0.0):
            case = 8
            sN = 0
        elif ((-d + b) > a):
            case = 9
            sN = sD
        else:
            case = 7
            sN = (-d + b)
            sD = a
    
    # finally do the division to get sc and tc
    if(np.absolute(sN) < SMALL_NUM):
        sc = 0.0
    else:
        sc = sN / sD
    
    if(np.absolute(tN) < SMALL_NUM):
        tc = 0.0
    else:
        tc = tN / tD
    
    # get the difference of the two closest points
    dP = w + (sc * u) - (tc * v)  # = S1(sc) - S2(tc)
    distance = np.linalg.norm(dP)
    # print(distance)
    # print(np.sqrt(distance))
    # print(np.linalg.norm(dP))
    # print(dP)
    return case, distance

def edge_squared_distances(points, edges):
    diff = points[edges[:, 0]] - points[edges[:, 1]]
    sqr_dist = np.sum(np.square(diff), axis=1)
    return sqr_dist

class EdgeConstrainedOptimizer(Optimizer):
    """
    Performs constrained optimization that optimizes MSE between output and verts,
    subject to constraint that edges length in output is within stretch_coefficient
    of original distance. It also constraints some specified nodes to be certain
     position known a priori. It also constraints edges from passing through themselves 
    """
    def __init__(self, template, edges, stretch_coefficient=1.0, use_gripper_prior = False, use_passingthru_constraint = True, visualize_violations = False):
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
        self.visualize_violations =  visualize_violations
        self.use_passingthru_constraint = use_passingthru_constraint 
        self.use_gripper_prior = use_gripper_prior

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

    def run(self, verts, prev_verts, iteration):
        # print(verts.shape)
        model = gurobipy.Model()
        model.setParam('OutputFlag', False)
        model.setParam('ScaleFlag', 2)
        if(self.visualize_violations):
            violate_points_1, violate_points_2 = detect_violation(verts,prev_verts, self.template, self.edges)
        else:
            violate_points_1 = None
            violate_points_2 = None

        g_verts = grb_utils.create_gurobi_arr(model, verts.shape, name="verts")
        # distance constraint
        rhs = (self.stretch_coefficient ** 2) * edge_squared_distances(self.template, self.edges)
        # print(min(edge_squared_distances(prev_verts, self.edges)))
        lhs = edge_squared_distances(g_verts, self.edges)
        grb_utils.add_constraints(model, lhs, "<=", rhs, name="edge")
        # prior pos constraint
        if(self.use_gripper_prior):
            lhs = g_verts[self.prior_idx]
            rhs = self.prior_pos
            grb_utils.add_constraints(model, lhs, "==", rhs, name="prior")
        
        #object passing through itself constraint
        if(self.use_passingthru_constraint):
            d_max = 0.1 # change
            d_min = 0.0001 # change
            if(iteration != 0):
                lhs = []
                rhs = []
                dist = np.empty((g_verts.shape[0], g_verts.shape[0]))
                for i in range(g_verts.shape[0]-1):
                    for j in range(i+1,g_verts.shape[0]-1):
                        if(isNeighbour(i,j)==False):
                            case,diff = DistBetween2Segment(prev_verts[self.edges[i, 0]], prev_verts[self.edges[i, 1]], prev_verts[self.edges[j, 0]], prev_verts[self.edges[j, 1]])
                            dist[i,j]=diff
                            if( diff<d_max and diff>0.00000001):
                                delta = [g_verts[i][0] - prev_verts[i][0], g_verts[i][1] - prev_verts[i][1], g_verts[i][2] - prev_verts[i][2], 
                                 g_verts[i+1][0] - prev_verts[i+1][0], g_verts[i+1][1] - prev_verts[i+1][1], g_verts[i+1][2] - prev_verts[i+1][2], 
                                 g_verts[j][0] - prev_verts[j][0], g_verts[j][1] - prev_verts[j][1], g_verts[j][2] - prev_verts[j][2], 
                                 g_verts[j+1][0] - prev_verts[j+1][0], g_verts[j+1][1] - prev_verts[j+1][1], g_verts[j+1][2] - prev_verts[j+1][2]]
                                derivative = opt_equations(prev_verts[i], prev_verts[i+1], prev_verts[j], prev_verts[j+1], case)
                                
                                temp = np.dot(derivative, delta)
                                lhs.append(temp)
                                rhs.append(d_min - diff)
                        
                
                if(len(rhs) != 0):
                    grb_utils.add_constraints(model, np.asarray(lhs), ">=" , np.asarray(rhs) , name="collision")
        # objective function
        g_objective = np.sum(np.square(g_verts - verts))
        model.setObjective(g_objective, gurobipy.GRB.MINIMIZE)
        model.update()
        model.optimize()
        try:
            verts_result = grb_utils.get_value(g_verts)
        except:
            import IPython
            IPython.embed()
            raise KeyboardInterrupt
        return verts_result, violate_points_1, violate_points_2

