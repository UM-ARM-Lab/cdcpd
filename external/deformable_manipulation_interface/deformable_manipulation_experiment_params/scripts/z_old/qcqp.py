from cvxopt import matrix, solvers, sparse, spmatrix
#
# Copyright 2009 Jeffery Kline
# 
# qcqp is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# qcqp is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""
sol = qcqp(P, G, A, b)

P is dictionary with keys

'A0' is matrix
'b0' is vector
'c0' is vector
'd0' is scalar

G is dictionary with keys

'A' is array of matrices
'b' is array of vectors
'c' is array of vectors
'd' is array of scalars

All arrays are expected to have the same length

qcqp attempts to solve the problem:

minimize (A0 x + b0)' (A0 x + b0) - c0' x - d0

s.t.     (Ak x + bk)' (Ak x + bk) - ck' x - dk <= 0

          A x = b

Returns a dictionary with keys corresponding to keys returned by
cvxopt.solvers.sdp and with additionals key
'QCQPx'

 'QCQPx' is the sought-after soution

There is redundancy in the returned dictionary:
'x'[:-1]=='QCQPx' 

QCQP converts the problem to a semidef problem, it does not try to 
solve a second-order cone program
"""

def qcqp(P, G, A = None, b = None):
    """
    [f0, F0] = augA(A, b=None, c=None, d=None )

    A is mxn,
    b is mx1
    c is nx1
    d is scalar

    if b,c,d are None, default to 0
    
    augA returns sparse matrices F0 and F where
    F0.size is [m+1,m+1]
    
    F.size  is [ (m+1)^2,n]
    """

    solvers.options['show_progress'] = False

    def augA(A, b = None, c = None, d = None):
        m = A.size[0]
        n = A.size[1]
        if b is None:
            b = matrix(0., (m,1))
        if c is None:
            c = matrix(0., (n,1))
        if d is None:
            d = 0.
        f0=spmatrix([], [], [], (m+1, m+1))
        f0[::m+2]=1.
        f0[-1,:m]=b.T
        f0[-1,-1]=d
        F=spmatrix([],[],[], ((m+1)**2, n))
        for j in range(n):
            fk=matrix(0., (m+1, m+1))
            fk[-1,:m]=A[:,j].T
            fk[-1,-1]=c[j]
            F[:,j]=fk[:]
        return f0, F

    """
    Get the objective function
    """

    A0=P['A0']
    b0=P['b0']
    c0=P['c0']
    d0=P['d0']

    """
    Build array of inequality constraints
    Start with objective inequality
    """
    [f0, F0]=augA(A0, b0, c0, d0)
    zz=spmatrix([],[],[], (F0.size[0],1))
    FF0=sparse([[F0], [zz]])
    FF0[-1,-1]=1.
    Fk=[-FF0]
    fk=[ f0 ]
    neq=len(G['A'])

    """
    Build array of inequality constraints
    Step through remaining inequalities
    """
    for j in range(neq):
        [f0, FF0]=augA(G['A'][j], G['b'][j], G['c'][j], G['d'][j])
        zz=spmatrix([],[],[], (FF0.size[0],1))
        FFk=sparse([[FF0], [zz]])
        Fk.append(-FFk)
        fk.append(f0)
    c=matrix(0., (A0.size[1]+1, 1))
    c[-1]=1
    Gl=None
    hl=None
    """
    Solve and return
    """
    if A is not None:
        A=matrix([[A], [matrix(0., (A.size[0], 1))]])
    sol=solvers.sdp(c, Gl, hl, Fk, fk, A, b)
    if sol['x'] is not None:
        sol['QCQPx']=sol['x'][:-1]
    else:
        sol['QCQPx']=None

    return sol
