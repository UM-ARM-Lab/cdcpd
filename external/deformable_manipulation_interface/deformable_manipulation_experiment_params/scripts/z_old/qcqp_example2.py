from cvxopt import matrix, solvers, normal, setseed, max
from cvxopt.blas import nrm2
from qcqprel import *

"""
Problem

Minimize   (Ax+b)'(Ax+b) + c' x + d
s.t.        x' x <= 1
	    x[0]*x[1]= 0.05
We solve this program with SDP Relaxation of QCQP
"""
n=9
m=5

Id=matrix(0., (n,n))
Id[::n+1]=1.

setseed(2)
A=normal(m,n)
A2=A.T*A

b=normal(m,1)

c=normal(n,1)
d=2.

Z=matrix(0., (n,n))
Z[1,0]=1.
beq0=-0.05 * 2
            
# 0. SDP Relaxation of QCQP
relP0={'P0': A2, 'b0': 2*A.T*b+c, 'c0': d+b.T*b}
relG0={'P': [Id], 'b': [None], 'c': [ -1. ],
       'Peq': [Z], 'beq': [None], 'ceq': [beq0]}
sol0=qcqprel(relP0, relG0)

# print some output
x0=sol0['RQCQPx']
X0=sol0['RQCQPX']
print "#"*40
print " "*12, "Diagnostics"
print 
print "max of |X0-x0*x0'|"
print " "*4, max(abs( X0 - x0*x0.T))
print " || xj || "
print " "*4, "x0:", nrm2(x0)

print "value of objective"
obj0=(A*x0 + b ).T*(A*x0 + b ) + c.T*x0 + d
print " "*4, "x0: ", obj0[0]
print "Constraint check"
print " "*4, "x0[0]*x0[1]: ", x0[0]*x0[1]
