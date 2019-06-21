#!/usr/bin/env python3
import numpy as np
import gurobipy
import cdcpd
import gurobi_utils as grb_utils
from optimize_eqn import opt_equations

def squared_norm(points):
    sqr_dist = np.sum(np.square(points))
    return sqr_dist

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
    else:                # get the closest points on the infinite lines
        sN = (b*e - c*d)
        tN = (a*e - b*d)
        if (sN < 0.0):   # sc < 0 => the s=0 edge is visible       
            sN = 0.0
            tN = e
            tD = c
        elif (sN > sD):# sc > 1 => the s=1 edge is visible
            sN = sD
            tN = e + b
            tD = c

    if (tN < 0.0):            #tc < 0 => the t=0 edge is visible
        tN = 0.0
        # recompute sc for this edge
        if (-d < 0.0):
            sN = 0.0
        elif (-d > a):
            sN = sD
        else:
            sN = -d
            sD = a

    elif (tN > tD):       # tc > 1 => the t=1 edge is visible
        tN = tD
        # recompute sc for this edge
        if ((-d + b) < 0.0):
            sN = 0
        elif ((-d + b) > a):
            sN = sD
        else:
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
    distance = squared_norm(dP)
    # print(distance)
    # print(np.sqrt(distance))
    # print(np.linalg.norm(dP))
    # print(dP)
    return sc,tc,distance

def test_gurobi():
    A = np.empty([4,3], dtype=float)
    A[0] = np.array([0, 0, 0])
    A[1] = np.array([0.5, 0, 0])

    A[2] = np.array([0.5, -0.5, 0.0])
    A[3] = np.array([0.5, 0.5, 0.0])

    model = gurobipy.Model()
    model.setParam('OutputFlag', True)
    g_A = grb_utils.create_gurobi_arr(model, A.shape, name="A")
    # g_B = grb_utils.create_gurobi_arr(model, B.shape, name="B")
    # g_C = grb_utils.create_gurobi_arr(model, C.shape, name="C")
    # g_D = grb_utils.create_gurobi_arr(model, D.shape, name="D")

    #object passing through itself constraint
    d_min = 0.01 # change
    # lhs = np.empty(A.shape[0], dtype=float)
    # rhs = np.full(lhs.shape, d_min)
    #print((verts.shape))
    delta = [g_A[0][0]-A[0][0], g_A[0][1]-A[0][1], g_A[0][2]-A[0][2], g_A[1][0]-A[1][0], g_A[1][1]-A[1][1], g_A[1][2]-A[1][2], g_A[2][0]-A[2][0], g_A[2][1]-A[2][1], g_A[2][2]-A[2][2], g_A[3][0]-A[3][0], g_A[3][1]-A[3][1], g_A[3][2]-A[3][2]]
    Sc,Tc,diff = DistBetween2Segment(A[0],A[1],A[2],A[3])
    print(Sc, Tc, diff)
    derivative = opt_equations(A[0],A[1],A[2],A[3], Sc, Tc)
    print(derivative)
    lhs=  derivative * delta
    rhs = np.full(12,d_min - diff)


    grb_utils.add_constraints(model, lhs, ">=" , rhs , name="collision")

    # objective function
    g_objective = np.sum(np.square(g_A - A))
    model.setObjective(g_objective, gurobipy.GRB.MINIMIZE)
    model.update()
    model.optimize()

    verts_result = grb_utils.get_value(g_A)
    # print(verts_result)
    # print("end")
    print(verts_result)


test_gurobi()