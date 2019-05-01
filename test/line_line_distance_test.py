import numpy as np
from cdcpd.geometry_utils import build_line
from cdcpd import gurobi_utils as grb_utils
import gurobipy


# linear interpolate with factor t
def lerp(a, b, t):
    return (1 - t) * a + t * b

# get line segment project parameter
def proj_lseg_t(p, a, b):
    ab = b - a
    t = np.dot(p - a, ab) / np.dot(ab, ab)
    return t

# project p to vector a->b
def proj_vec(p, a, b):
    t = proj_lseg_t(p, a, b)
    return lerp(a, b, t)


# project point P to plane defined by normal n0->n1
def proj_plane(P, n0, n1):
    D = len(P)
    v_p = P - n0
    v_n = n1 - n0
    v_p_proj_n = proj_vec(v_p, np.zeros(D), v_n)
    v_p_plane_proj = v_p - v_p_proj_n
    result = v_p_plane_proj + n0
    return result


# define plane with normal a->b
def dist_vec_seg_to_seg(a, b, c, d):
    c_proj = proj_plane(c, a, b)
    d_proj = proj_plane(d, a, b)
    t_cd = proj_lseg_t(a, c_proj, d_proj)
    # if not 0 < t_cd < 1:
    #     return None

    cd_cross = lerp(c, d, t_cd)
    t_ab = proj_lseg_t(cd_cross, a, b)
    # if not 0 < t_ab < 1:
    #     return None

    cd_proj_cross = lerp(c_proj, d_proj, t_cd)
    dist_vec = cd_proj_cross - a
    return dist_vec


# project point p to plane defined by normal n0->n1 to p'
# return p* = p' * norm(n1-n0)^2
def proj_plane_n(p, n0, n1):
    n = n0 - n1
    n_norm_sqr = np.dot(n, n)

    p_star = p * n_norm_sqr - np.dot(p - n0, n) * n
    return p_star


# compute vector of shortest distance between a->b and c->d: v
# return v* = v * sigma
# where sigma = norm(b-a)^2 * norm(d*-c*)^2
# where d* and c* are proj_plane_n(d, a, b)
def dist_vec_seg_to_seg_n(a, b, c, d):
    c_star = proj_plane_n(c, a, b)
    d_star = proj_plane_n(d, a, b)

    cd_star = d_star - c_star
    cd_star_norm_sqr = np.dot(cd_star, cd_star)

    ab = b - a
    ab_norm_sqr = np.dot(ab, ab)

    v_star = c_star * cd_star_norm_sqr + np.dot(a * ab_norm_sqr - c_star, cd_star) * cd_star
    sigma = cd_star_norm_sqr * ab_norm_sqr
    return v_star, sigma


def test_gurobi():
    A = np.array([0, 0, 0])
    B = np.array([1, 0, 0])

    C = np.array([0.5, -0.5, 1.0])
    D = np.array([0.5, 0.5, 1.0])

    model = gurobipy.Model()
    model.setParam('OutputFlag', False)
    g_A = grb_utils.create_gurobi_arr(model, A.shape, name="A")
    g_B = grb_utils.create_gurobi_arr(model, B.shape, name="B")
    g_C = grb_utils.create_gurobi_arr(model, C.shape, name="C")
    g_D = grb_utils.create_gurobi_arr(model, D.shape, name="D")
    model.update()

    v_star, sigma = dist_vec_seg_to_seg_n(g_A, g_B, g_C, g_D)
    v_n = v_star / sigma


def test1():
    A = np.array([0, 0, 0])
    B = np.array([1, 0, 0])

    C = np.array([0.5, -0.5, 1.0])
    D = np.array([0.5, 0.5, 1.0])

    print("len AB", np.linalg.norm(A - B))
    print("len CD", np.linalg.norm(C - D))

    v = dist_vec_seg_to_seg(A, B, C, D)

    v_star, sigma = dist_vec_seg_to_seg_n(A, B, C, D)
    v_n = v_star / sigma

    print("v:", v)
    print("v_n:", v_n)
    assert(np.allclose(v, v_n))


def test2():
    A = np.array([0, 0, 0])
    B = np.array([0.5, 0, 0])

    C = np.array([0.5, -0.5, 1.0])
    D = np.array([0.5, 0.5, 1.0])

    print("len AB", np.linalg.norm(A - B))
    print("len CD", np.linalg.norm(C - D))

    v = dist_vec_seg_to_seg(A, B, C, D)

    v_star, sigma = dist_vec_seg_to_seg_n(A, B, C, D)
    v_n = v_star / sigma

    print("v:", v)
    print("v_n:", v_n)
    assert (np.allclose(v, v_n))

def test3():
    A = np.array([0, 0, 0])
    B = np.array([0.5, 0, 0.5])

    C = np.array([0.5, -0.5, 1.0])
    D = np.array([0.5, 0.5, 1.0])

    print("len AB", np.linalg.norm(A - B))
    print("len CD", np.linalg.norm(C - D))

    v = dist_vec_seg_to_seg(A, B, C, D)

    v_star, sigma = dist_vec_seg_to_seg_n(A, B, C, D)
    v_n = v_star / sigma

    print("v:", v)
    print("v_n:", v_n)
    assert (np.allclose(v, v_n))


def test4():
    A = np.array([0, 0, 0])
    B = np.array([0.5, 0, 0.5])

    C = np.array([0.5, -0.5, 1.0])
    D = np.array([0.5, 0.5, 1.0])

    print("len AB", np.linalg.norm(A - B))
    print("len CD", np.linalg.norm(C - D))

    v = dist_vec_seg_to_seg(A, B, C, D)

    v_star, sigma = dist_vec_seg_to_seg_n(A, B, C, D)
    v_n = v_star / sigma

    print("v:", v)
    print("v_n:", v_n)
    assert (np.allclose(v, v_n))


test_gurobi()
