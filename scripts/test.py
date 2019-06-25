#!/usr/bin/env python3
import numpy as np
import gurobipy
import cdcpd
import gurobi_utils as grb_utils
from optimize_eqn import opt_equations


verts = [[-3.39996848e-01, -1.65605015e-01,  8.03134434e-01],
 [-3.20870004e-01, -1.46675857e-01,  8.14395886e-01],
 [-3.02549563e-01, -1.27228013e-01,  8.23723601e-01],
 [-2.85027442e-01, -1.07381011e-01,  8.31107227e-01],
 [-2.68264484e-01, -8.72746867e-02,  8.36611235e-01],
 [-2.52235916e-01, -6.70449621e-02,  8.40277286e-01],
 [-2.36827568e-01, -4.69739131e-02,  8.42305501e-01],
 [-2.21996213e-01, -2.72607875e-02,  8.42758666e-01],
 [-2.07608614e-01, -8.22973054e-03,  8.41793687e-01],
 [-1.93554257e-01,  9.69112884e-03,  8.39691428e-01],
 [-1.79630998e-01,  2.59480112e-02,  8.36756103e-01],
 [-1.65793379e-01,  3.95894537e-02,  8.33463753e-01],
 [-1.53768896e-01,  4.86371000e-02,  8.30728583e-01],
 [-1.42699868e-01,  5.47166331e-02,  8.28569910e-01],
 [-1.31657872e-01,  5.87427695e-02,  8.26852635e-01],
 [-1.20611688e-01,  6.07476812e-02,  8.25737821e-01],
 [-1.09472906e-01,  6.09221291e-02,  8.25244642e-01],
 [-9.81845362e-02,  5.94674228e-02,  8.25366740e-01],
 [-8.67051372e-02,  5.65902002e-02,  8.26117870e-01],
 [-7.50163034e-02,  5.25175337e-02,  8.27394978e-01],
 [-6.30847789e-02,  4.74145629e-02,  8.29225161e-01],
 [-5.09405551e-02,  4.15519881e-02,  8.31457489e-01],
 [-3.85766249e-02,  3.51191967e-02,  8.34033197e-01],
 [-2.59761257e-02,  2.82995846e-02,  8.36887549e-01],
 [-1.31957859e-02,  2.12951817e-02,  8.39924146e-01],
 [-2.41227588e-04,  1.42674223e-02,  8.42988346e-01],
 [ 1.28759580e-02,  7.35661943e-03,  8.46136944e-01],
 [ 2.61075760e-02,  7.42459395e-04 , 8.49162611e-01],
 [ 3.93913401e-02, -5.47410914e-03 , 8.51985073e-01],
 [ 5.27795403e-02, -1.12207420e-02 , 8.54689414e-01],
 [ 6.61664875e-02, -1.64155517e-02 , 8.57114861e-01],
 [ 7.95793719e-02, -2.09590132e-02 , 8.59243314e-01],
 [ 9.29575760e-02, -2.48338826e-02 , 8.61021867e-01],
 [ 1.06267429e-01, -2.80171701e-02 , 8.62444703e-01],
 [ 1.19504300e-01, -3.04999297e-02 , 8.63495500e-01],
 [ 1.32603645e-01, -3.22505745e-02 , 8.64114828e-01],
 [ 1.45559905e-01, -3.32770453e-02 , 8.64300144e-01],
 [ 1.58337390e-01, -3.36213101e-02 , 8.64055379e-01],
 [ 1.70882203e-01, -3.32396361e-02 , 8.63287855e-01],
 [ 1.83180309e-01, -3.22331880e-02 , 8.62062719e-01],
 [ 1.95183389e-01, -3.05217089e-02 , 8.60250042e-01],
 [ 2.06818380e-01, -2.82096480e-02 , 8.57835851e-01],
 [ 2.18062950e-01, -2.52131128e-02 , 8.54728007e-01],
 [ 2.28863170e-01, -2.15727631e-02 , 8.50894672e-01],
 [ 2.39169568e-01, -1.72603154e-02 , 8.46208580e-01],
 [ 2.48891655e-01, -1.22588411e-02 , 8.40617656e-01],
 [ 2.57937918e-01, -6.52747550e-03 , 8.33868948e-01],
 [ 2.66310352e-01, -6.70889483e-05 , 8.26017113e-01],
 [ 2.73871212e-01,  7.25658398e-03 , 8.16793587e-01],
 [ 2.80539342e-01,  1.55056805e-02 , 8.05997643e-01]]

prev_verts = [[-0.3069172,  -0.13498428,  0.8223339 ],
 [-0.29581264, -0.11791176,  0.82363915],
 [-0.2843045 , -0.10110784,  0.824935  ],
 [-0.2723433 , -0.08462281,  0.8262248 ],
 [-0.25989595, -0.06850092,  0.82750523],
 [-0.24692911, -0.05279321,  0.8287776 ],
 [-0.2333813 , -0.03758365,  0.8300491 ],
 [-0.2191837 , -0.02297978,  0.8313318 ],
 [-0.2042667 , -0.00911227,  0.8326225 ],
 [-0.18854912,  0.00383964,  0.8339219 ],
 [-0.17194556,  0.01561393,  0.8352464 ],
 [-0.15457097,  0.025646  ,  0.8365935 ],
 [-0.1390019 ,  0.03257447,  0.83775425],
 [-0.12448306,  0.03754407,  0.83882403],
 [-0.11001585,  0.04117291,  0.8399018 ],
 [-0.09565817,  0.04350045,  0.8409722 ],
 [-0.0814248 ,  0.04461793,  0.8420265 ],
 [-0.06733082,  0.04461847,  0.84306246],
 [-0.05336206,  0.04358648,  0.84410954],
 [-0.03954219,  0.04162669,  0.84514415],
 [-0.02587559,  0.03884108,  0.8461532 ],
 [-0.0123593 ,  0.03532413,  0.84715265],
 [ 0.00099975,  0.03117493,  0.8481318 ],
 [ 0.0142012 ,  0.02649858,  0.84909326],
 [ 0.02723608,  0.02138453,  0.85003513],
 [ 0.04008888,  0.01596374,  0.8508961 ],
 [ 0.05277503,  0.01030552,  0.8517407 ],
 [ 0.06527667,  0.00453763 , 0.85250324],
 [ 0.07757583, -0.00123269 , 0.853157  ],
 [ 0.08970691, -0.00693906 , 0.8537758 ],
 [ 0.10163598, -0.01244821 , 0.8542564 ],
 [ 0.11337633, -0.01769103 , 0.8546498 ],
 [ 0.12491664, -0.02255446 , 0.8548863 ],
 [ 0.13625771, -0.02695675 , 0.8549882 ],
 [ 0.14739488, -0.03081724 , 0.8549127 ],
 [ 0.15832943, -0.03402374 , 0.8546525 ],
 [ 0.16907078, -0.03652411 , 0.8542069 ],
 [ 0.17960495, -0.03822627 , 0.853531  ],
 [ 0.18993908, -0.03905651 , 0.8526248 ],
 [ 0.20006834, -0.03895139 , 0.85144335],
 [ 0.21001236, -0.03785602 , 0.8500157 ],
 [ 0.21974511, -0.03569389 , 0.84825027],
 [ 0.22929002, -0.03242708 , 0.84619594],
 [ 0.23865132, -0.027991   , 0.8437971 ],
 [ 0.24783307, -0.02236344 , 0.8410579 ],
 [ 0.25685266, -0.01550213 , 0.8379782 ],
 [ 0.26567137, -0.007356   , 0.8344588 ],
 [ 0.27434218,  0.00205155 , 0.83056974],
 [ 0.28286648,  0.01273714 , 0.82630384],
 [ 0.2912281 ,  0.02474173 , 0.8215804 ]]
edges = [[ 0 , 1],
 [ 1,  2],
 [ 2 , 3],
 [ 3,  4],
 [ 4 , 5],
 [ 5,  6],
 [ 6,  7],
 [ 7,  8],
 [ 8,  9],
 [ 9, 10],
 [10, 11],
 [11, 12],
 [12, 13],
 [13, 14],
 [14, 15],
 [15, 16],
 [16, 17],
 [17, 18],
 [18, 19],
 [19, 20],
 [20, 21],
 [21, 22],
 [22 ,23],
 [23, 24],
 [24, 25],
 [25, 26],
 [26, 27],
 [27, 28],
 [28, 29],
 [29, 30],
 [30, 31],
 [31, 32],
 [32, 33],
 [33, 34],
 [34, 35],
 [35, 36],
 [36, 37],
 [37, 38],
 [38, 39],
 [39, 40],
 [40, 41],
 [41, 42],
 [42, 43],
 [43, 44],
 [44, 45],
 [45, 46],
 [46, 47],
 [47, 48],
 [48, 49],]

iteration = 1
def squared_norm(points):
    sqr_dist = np.sum(np.square(points))
    return sqr_dist

def edge_squared_distances(points, edges):
    diff = points[edges[:, 0]] - points[edges[:, 1]]
    sqr_dist = np.sum(np.square(diff), axis=1)
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

def test_optimizer(verts, prev_verts, edges):
	model = gurobipy.Model()
	#model.setParam('OutputFlag', False)
	model.setParam('ScaleFlag', 2)
	verts = np.asarray(verts)
	prev_verts = np.asarray(prev_verts)
	g_verts = grb_utils.create_gurobi_arr(model, verts.shape, name="verts")
	# distance constraint
	rhs = (1 ** 2) * edge_squared_distances(prev_verts, np.asarray(edges))
	lhs = edge_squared_distances(g_verts, np.asarray(edges))
	grb_utils.add_constraints(model, lhs, "<=", rhs, name="edge")

	#object passing through itself constraint
	d_max = 0.2 # change
	d_min = 0.001 # change
	if(iteration != 0):
	    lhs = []
	    rhs = []
	    dist = np.empty((g_verts.shape[0], g_verts.shape[0]))
	    for i in range(g_verts.shape[0]-1):
	        for j in range(i+1,g_verts.shape[0]-1):
	            Sc,Tc,diff = DistBetween2Segment(prev_verts[i], prev_verts[i+1], prev_verts[j], prev_verts[j+1])
	            dist[i,j]=diff
	            print(diff)
	            if( diff<d_max and diff!=0):
	                # delta = [g_verts[self.edges[i, 0]][0] - prev_verts[self.edges[i, 0]][0], g_verts[self.edges[i, 0]][1] - prev_verts[self.edges[i, 0]][1], g_verts[self.edges[i, 0]][2] - prev_verts[self.edges[i, 0]][2], 
	                #  g_verts[self.edges[i, 1]][0] - prev_verts[self.edges[i, 1]][0], g_verts[self.edges[i, 1]][1] - prev_verts[self.edges[i, 1]][1], g_verts[self.edges[i, 1]][2] - prev_verts[self.edges[i, 1]][2], 
	                #  g_verts[self.edges[j, 0]][0] - prev_verts[self.edges[j,0]][0], g_verts[self.edges[j, 0]][1] - prev_verts[self.edges[j,0]][1], g_verts[self.edges[j, 0]][2] - prev_verts[self.edges[j,0]][2], 
	                #  g_verts[self.edges[j, 1]][0] - prev_verts[self.edges[j,1]][0], g_verts[self.edges[j, 1]][1] - prev_verts[self.edges[j,1]][1], g_verts[self.edges[j, 1]][2] - prev_verts[self.edges[j,1]][2]]
	                delta = [g_verts[i][0] - prev_verts[i][0], g_verts[i][1] - prev_verts[i][1], g_verts[i][2] - prev_verts[i][2], 
	                 g_verts[i+1][0] - prev_verts[i+1][0], g_verts[i+1][1] - prev_verts[i+1][1], g_verts[i+1][2] - prev_verts[i+1][2], 
	                 g_verts[j][0] - prev_verts[j][0], g_verts[j][1] - prev_verts[j][1], g_verts[j][2] - prev_verts[j][2], 
	                 g_verts[j+1][0] - prev_verts[j+1][0], g_verts[j+1][1] - prev_verts[j+1][1], g_verts[j+1][2] - prev_verts[j+1][2]]
	                # derivative = opt_equations(prev_verts[self.edges[i, 0]], prev_verts[self.edges[i, 1]], prev_verts[self.edges[j, 0]], prev_verts[self.edges[j, 1]], Sc, Tc)
	                derivative = opt_equations(prev_verts[i], prev_verts[i+1], prev_verts[j], prev_verts[j+1], Sc, Tc)
	                #print(derivative)
	                #print(diff, derivative)
	                temp = np.dot(derivative, delta)
	                lhs.append(temp)
	                rhs.append(d_min - diff)
	            
	            # delta = [g_verts[self.edges[i, 0]] - prev_verts[self.edges[i, 0]], g_verts[self.edges[i, 1]] - prev_verts[self.edges[i, 1]], g_verts[self.edges[j, 0]] - prev_verts[self.edges[j,0]], g_verts[self.edges[j, 1]] - prev_verts[self.edges[j,1]]]
	            # derivative = opt_equations(prev_verts[self.edges[i, 0]], prev_verts[self.edges[i, 1]], prev_verts[self.edges[j, 0]], prev_verts[self.edges[j, 1]])
	            # lhs[i,j]= derivative * delta
	            # diff = DistBetween2Segment(prev_verts[self.edges[i, 0]],prev_verts[self.edges[i, 1]],prev_verts[self.edges[j, 0]],prev_verts[self.edges[j, 1]])
	            # rhs[i,j] = d_min - diff
	    #print(lhs)
	    #print(rhs)
	    if(len(rhs) != 0):
	        grb_utils.add_constraints(model, np.asarray(lhs), ">=" , np.asarray(rhs) , name="collision")
	    #print(dist)

	# objective function
	g_objective = np.sum(np.square(g_verts - verts))
	model.setObjective(g_objective, gurobipy.GRB.MINIMIZE)
	model.update()
	model.optimize()
	# print(grb_utils.get_value(g_objective))
	verts_result = grb_utils.get_value(g_verts)
	print(verts_result)
	# print("end")
	return verts_result

test_optimizer(verts, prev_verts, edges)
