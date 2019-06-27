#!/usr/bin/env python3
import numpy as np
import gurobipy
import cdcpd
import gurobi_utils as grb_utils
from optimize_eqn import opt_equations


verts = [[ 0.10715991,  0.04673988,  0.80860759],
       [ 0.09638239,  0.01433933,  0.81983173],
       [ 0.08617341, -0.01759939,  0.83085142],
       [ 0.07782873, -0.0542619 ,  0.84353298],
       [ 0.06686021, -0.0774444 ,  0.85133865],
       [ 0.05289089, -0.08822634,  0.85543967],
       [ 0.04637606, -0.12401531,  0.86811271],
       [ 0.03562015, -0.13741997,  0.87270725],
       [ 0.0248786 , -0.1413001 ,  0.87320879],
       [ 0.01468532, -0.14773981,  0.87562408],
       [ 0.00357272, -0.14664085,  0.87595227],
       [-0.00820486, -0.13195502,  0.87113355],
       [-0.01745581, -0.10721477,  0.86086335],
       [-0.02305258, -0.09941736,  0.85730128],
       [-0.02878705, -0.09312096,  0.85570456],
       [-0.03524446, -0.08421447,  0.85446885],
       [-0.04015845, -0.06663123,  0.84921755],
       [-0.04169592, -0.03363333,  0.83572083],
       [-0.03347042, -0.0436415 ,  0.83657182],
       [-0.02979077, -0.07286137,  0.84963182],
       [-0.03598519, -0.06596182,  0.85352851],
       [-0.04282322, -0.01328143,  0.83862685],
       [-0.03340218,  0.03100616,  0.81762132],
       [-0.01035984, -0.01179704,  0.826102  ],
       [-0.00166537, -0.06197893,  0.84648769],
       [-0.00956164, -0.0242219 ,  0.84116302],
       [-0.00439734,  0.04134061,  0.81730814],
       [ 0.01767483,  0.04656646,  0.80910828],
       [ 0.04330447,  0.01360924,  0.81378065],
       [ 0.04192999,  0.03358617,  0.8143961 ],
       [ 0.05483742,  0.07409452,  0.80050367],
       [ 0.07551139,  0.07664875,  0.79726915],
       [ 0.09787113,  0.0644173 ,  0.79847118],
       [ 0.10994615,  0.07539072,  0.79767696],
       [ 0.12614477,  0.07059715,  0.80021128],
       [ 0.14983799,  0.04895134,  0.80346403],
       [ 0.16857508,  0.00464203,  0.81560652],
       [ 0.18039214, -0.04256719,  0.83159597],
       [ 0.18869711, -0.09296316,  0.84949368],
       [ 0.19966908, -0.12754858,  0.86138094],
       [ 0.2097545 , -0.121052  ,  0.86174037],
       [ 0.21820921, -0.14896334,  0.87206294],
       [ 0.23307842, -0.17789114,  0.87864879],
       [ 0.24600141, -0.15242748,  0.87122104],
       [ 0.25934295, -0.11600969,  0.86038443],
       [ 0.27538173, -0.07380787,  0.84651199],
       [ 0.29097631, -0.03616908,  0.83405798],
       [ 0.30768158,  0.00909728,  0.81887261],
       [ 0.32403388,  0.05319622,  0.80403907],
       [ 0.3124735 ,  0.0396488 ,  0.8087228 ]]

prev_verts = [[-0.34377405, -0.33031026,  0.9278386 ],
       [-0.3263928 , -0.32014287,  0.9249202 ],
       [-0.30906639, -0.31003934,  0.92196596],
       [-0.29113135, -0.30068174,  0.9192748 ],
       [-0.27464664, -0.2895189 ,  0.91576207],
       [-0.2593275 , -0.27651927,  0.9125789 ],
       [-0.24122125, -0.267361  ,  0.9103944 ],
       [-0.22545305, -0.2551129 ,  0.9068141 ],
       [-0.21077898, -0.24186993,  0.90173674],
       [-0.19544971, -0.22885841,  0.89824384],
       [-0.18084057, -0.21492821,  0.8952414 ],
       [-0.16798946, -0.19961706,  0.89112914],
       [-0.15702881, -0.18399456,  0.88389766],
       [-0.14309183, -0.17012313,  0.8784354 ],
       [-0.12847269, -0.15627009,  0.8751388 ],
       [-0.11387897, -0.14208335,  0.87363803],
       [-0.10035085, -0.1271662 ,  0.87032723],
       [-0.08988503, -0.11159595,  0.8622943 ],
       [-0.07337007, -0.1019052 ,  0.8552348 ],
       [-0.05437598, -0.09480536,  0.85753894],
       [-0.03930293, -0.08228814,  0.86324996],
       [-0.03024845, -0.06401387,  0.8625017 ],
       [-0.0233805 , -0.0501565 ,  0.8491864 ],
       [-0.00565448, -0.0494107 ,  0.83910054],
       [ 0.01425894, -0.04849402,  0.84347177],
       [ 0.02558225, -0.03295129,  0.8503053 ],
       [ 0.03193247, -0.01519542,  0.84250176],
       [ 0.04593866, -0.00740468,  0.82986754],
       [ 0.06325684, -0.00670256,  0.8190934 ],
       [ 0.07793314,  0.0053124 ,  0.8266251 ],
       [ 0.09129542,  0.01992236,  0.8216763 ],
       [ 0.10909576,  0.02755744,  0.8152463 ],
       [ 0.12801644,  0.03185325,  0.80891806],
       [ 0.14625542,  0.04090825,  0.81027406],
       [ 0.16592996,  0.04628979,  0.8096061 ],
       [ 0.18514465,  0.04683311,  0.802751  ],
       [ 0.2047815 ,  0.04242228,  0.79937005],
       [ 0.22448874,  0.0372933 ,  0.8007156 ],
       [ 0.24367481,  0.0310424 ,  0.8037671 ],
       [ 0.26374963,  0.02757846,  0.80498976],
       [ 0.28351188,  0.03205861,  0.8074131 ],
       [ 0.3035415 ,  0.02889915,  0.8097148 ],
       [ 0.3230263 ,  0.02440747,  0.8062649 ],
       [ 0.34244183,  0.03069315,  0.8061259 ],
       [ 0.36127096,  0.03856179,  0.8059291 ],
       [ 0.37981704,  0.04684576,  0.8039517 ],
       [ 0.3989821 ,  0.05365665,  0.8022766 ],
       [ 0.41773614,  0.06139682,  0.8000704 ],
       [ 0.43681917,  0.06834923,  0.7980717 ],
       [ 0.41643342,  0.06742887,  0.79832375]]

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

def isNeighbour(i, j):
    if(np.abs(i-j)<=1):
        return True
    else:
        return False

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

def test_gurobi():
    A = np.empty([4,3], dtype=float)
    A[0] = np.array([0, 0, 0])
    A[1] = np.array([0.5, 0, 0])

    A[2] = np.array([0.5, -0.5, 0.0])
    A[3] = np.array([0.5, 0.5, 0.0])

    model = gurobipy.Model()
    model.setParam('OutputFlag', False)
    model.setParam('ScaleFlag', 2)
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
	d_min = 0.01 # change
	count = 0
	if(iteration != 0):
	    lhs = []
	    rhs = []
	    dist = np.empty((g_verts.shape[0], g_verts.shape[0]))
	    for i in range(g_verts.shape[0]-1):
	        for j in range(i+1,g_verts.shape[0]-1):
	        	if(isNeighbour(i,j)==False):
	        		case,diff = DistBetween2Segment(prev_verts[i], prev_verts[i+1], prev_verts[j], prev_verts[j+1])
	        		dist[i,j]=diff
	        		if( diff<d_max and diff>0.00000001):
	        			delta = [g_verts[i][0] - prev_verts[i][0], g_verts[i][1] - prev_verts[i][1], g_verts[i][2] - prev_verts[i][2], 
	        			 g_verts[i+1][0] - prev_verts[i+1][0], g_verts[i+1][1] - prev_verts[i+1][1], g_verts[i+1][2] - prev_verts[i+1][2], 
	        			 g_verts[j][0] - prev_verts[j][0], g_verts[j][1] - prev_verts[j][1], g_verts[j][2] - prev_verts[j][2], 
	        			 g_verts[j+1][0] - prev_verts[j+1][0], g_verts[j+1][1] - prev_verts[j+1][1], g_verts[j+1][2] - prev_verts[j+1][2]]
	        			derivative = opt_equations(prev_verts[i], prev_verts[i+1], prev_verts[j], prev_verts[j+1], case)
	        			temp = np.dot(derivative, delta)
	        			count+=1
	        			
	        			lhs.append(temp)
	        			rhs.append(d_min - diff) 
			        
	    if(len(rhs) != 0):
	        grb_utils.add_constraints(model, np.asarray(lhs), ">=" , np.asarray(rhs) , name="collision")
	    
	# objective function
	g_objective = np.sum(np.square(g_verts - verts))
	model.setObjective(g_objective, gurobipy.GRB.MINIMIZE)
	model.update()
	model.optimize()
	print(count)
	# print(grb_utils.get_value(g_objective))
	verts_result = grb_utils.get_value(g_verts)
	# print("end")
	return verts_result

test_optimizer(verts, prev_verts, edges)
