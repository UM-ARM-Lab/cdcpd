import numpy as np
import scipy
from optimize_eqn import opt_equations
from geometry_msgs.msg import Point
import ros_numpy

def edge_squared_distances(points, edges):
    diff = points[edges[:, 0]] - points[edges[:, 1]]
    sqr_dist = np.sum(np.square(diff), axis=1)
    return sqr_dist

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

def isNeighbour(i, j):
    if(np.abs(i-j)<=1):
        return True
    else:
        return False

def detect_violation(verts,prev_verts, template, edges, stretch_coefficient = 1):
	rhs = (stretch_coefficient ** 2) * edge_squared_distances(verts, edges)
	initial = edge_squared_distances(template, edges)
	points = edges[rhs>initial]
	points_1 = [verts[points[:,0]], verts[points[:,1]]]
	# print(points)

	d_max = 0.01 # change
	d_min = 0.000001 # change
	lhs = []
	rhs = []
	points = []
	dist = np.empty((verts.shape[0], verts.shape[0]))
	for i in range(verts.shape[0]-1):
	    for j in range(i+1,verts.shape[0]-1):
	    	if(isNeighbour(i,j)==False):
	    		case,diff = DistBetween2Segment(prev_verts[edges[i, 0]], prev_verts[edges[i, 1]], prev_verts[edges[j, 0]], prev_verts[edges[j, 1]])
	    		dist[i,j]=diff
	    		#print(diff)
	    		if( diff<d_max and diff>0.00000001):
	    		    delta = [verts[i][0] - prev_verts[i][0], verts[i][1] - prev_verts[i][1], verts[i][2] - prev_verts[i][2], 
	    		     verts[i+1][0] - prev_verts[i+1][0], verts[i+1][1] - prev_verts[i+1][1], verts[i+1][2] - prev_verts[i+1][2], 
	    		     verts[j][0] - prev_verts[j][0], verts[j][1] - prev_verts[j][1], verts[j][2] - prev_verts[j][2], 
	    		     verts[j+1][0] - prev_verts[j+1][0], verts[j+1][1] - prev_verts[j+1][1], verts[j+1][2] - prev_verts[j+1][2]]
	    		    # derivative = opt_equations(prev_verts[self.edges[i, 0]], prev_verts[self.edges[i, 1]], prev_verts[self.edges[j, 0]], prev_verts[self.edges[j, 1]], Sc, Tc)
	    		    derivative = opt_equations(prev_verts[i], prev_verts[i+1], prev_verts[j], prev_verts[j+1], case)
	    		    #print(derivative)
	    		    #print(diff, derivative)
	    		    temp = np.dot(derivative, delta)
	    		    if(temp<(d_min-diff)):
	    		    	print(diff , i, j)
	    		    	points.append([verts[i], verts[i+1]])
	    		    	points.append([verts[j], verts[j+1]])
	        
	return np.asarray(points_1), np.asarray(points) 


