from cvxopt import matrix, solvers, normal, setseed, max
from cvxopt.blas import nrm2
from qcqprel import *
from qcqp import *

"""
Problem

minimize (A0 x + b0)' (A0 x + b0) - c0' x - d0

s.t.     (Ak x + bk)' (Ak x + bk) - ck' x - dk <= 0

          A x = b

We solve this program with 3 different methods:
0. SDP Relaxation of QCQP
1. QCQP
2. coneqp
"""

m = 3
n = 2
Id=matrix(0., (n,n))
Id[::n+1]=1.

J = matrix(0., (m, n))
J[0,0] = 1.
J[1,1] = 5.

pdot = matrix(10., (m, 1))

print J
print pdot

# # 0. SDP Relaxation of QCQP
# relP0={'P0': A.T*A, 'b0': -pdot, 'c0': None}
# relG0={'P': [Id], 'b': [None], 'c': [ -1.],
#        'Peq': [], 'beq': [], 'ceq': []}
# sol0=qcqprel(relP0, relG0)

A0={'A0': J, 'b0': -pdot, 'c0': None, 'd0': None}
G0={'A': [Id], 'b': [None], 'c': [None], 'd': [1.]}
sol1=qcqp(A0, G0)
qdot0=sol1['QCQPx']

print qdot0