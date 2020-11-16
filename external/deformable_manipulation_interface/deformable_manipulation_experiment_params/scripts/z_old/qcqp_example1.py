from cvxopt import matrix, solvers, normal, setseed, max
from cvxopt.blas import nrm2
from qcqprel import *
from qcqp import *

"""
Problem

Minimize   (Ax)'(Ax) + b' x + c
s.t.             x' x <= 1
           (Bx)'(Bx)  <= d

We solve this program with 3 different methods:
0. SDP Relaxation of QCQP
1. QCQP
2. coneqp
"""
n=9
m=5

Id=matrix(0., (n,n))
Id[::n+1]=1.

setseed(2)
A=normal(m,n)
A2=A.T*A

B=normal(m+1,n)
B2=B.T*B

b=normal(n,1)

c=0.
d=2.

# 0. SDP Relaxation of QCQP
relP0={'P0': A2, 'b0': b, 'c0': c}
relG0={'P': [Id, B2], 'b': [None, None], 'c': [ -1. , -d ],
       'Peq': [], 'beq': [], 'ceq': []}
sol0=qcqprel(relP0, relG0)

# 1. QCQP
A0={'A0': A, 'b0': None, 'c0': -b, 'd0': -c}
G0={'A': [Id, B], 'b': [None, None], 'c': [ None, None ], 'd': [1., d]}
sol1=qcqp(A0, G0)

# 2. coneqp
P=A2*2
q=b
G=matrix([matrix(0., (1,n)), -Id, 
          matrix(0., (1,n)), -B ])
h=matrix(0., ((n+1)+B.size[0]+1, 1))
h[0]=1.
h[n+1]=d**0.5
dims={'l': 0, 'q': [n+1, B.size[0]+1], 's': []}
sol2=solvers.coneqp(P,q, G, h, dims)

# print some output
x2=sol2['x']
x1=sol1['QCQPx']
x0=sol0['RQCQPx']
X0=sol0['RQCQPX']
print "#"*40
print " "*12, "Diagnostics"
print 
print "max of |X0-x0*x0'|"
print " "*4, max(abs( X0 - x0*x0.T))
print " || xj || "
print " "*4, "x0:", nrm2(x0)
print " "*4, "x1:", nrm2(x1)
print " "*4, "x2:", nrm2(x2)
print "value of (Ax)'(Ax)+b'x  (xB)'(xB)"
obj0=x0.T*A2*x0+b.T*x0
obj1=x1.T*A2*x1+b.T*x1
obj2=x2.T*A2*x2+b.T*x2
con0=x0.T*B2*x0
con1=x1.T*B2*x1
con2=x2.T*B2*x2
print " "*4, "x0:", obj0[0], con0[0]
print " "*4, "x1:", obj1[0], con1[0]
print " "*4, "x2:", obj2[0], con2[0]
