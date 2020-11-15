from cvxopt import matrix, solvers, sparse
#
# Copyright 2009 Jeffery Kline
# 
# qcqprel is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# qcqprel is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""
sol = qcqprel(P, G, r)

P is dictionary with keys

'P0' is a symmetric matrix
'b0' is vector
'c0' is scalar

G is dictionary with keys

'P' is array of symmetric matrices
'b' is array of vectors
'c' is array of scalars

'Peq' is array of symmetric matrices
'beq' is array of vectors
'ceq' is array of scalars

r is scalar, expected to be >=0

r is optional 'trace weight'. The larger 'r' is, the more emphasis is
given to 'trace(X)' being small. The emphasis is linear in 'r'.

The arrays 'P', 'b' and 'c' expected to have length equal to N>=0.
The arrays 'Peq', 'beq' and 'ceq' expected to have length equal to M>=0

Entries in 'b*' and 'c*' may be 'None'

QCQPREL attempts to find a good lower bound on the problem

minimize x.T P_0 x + b_0.T x + c_0

subj to  x.T P_k x + b_k.T x + c_k <= 0,         0<=k < N

         x.T Peq_k x + beq_k.T x + ceq_k = 0,    0<=k < M

Returns a dictionary with keys corresponding to keys returned by
cvxopt.solvers.sdp and with additional keys 'RQCQPx', 'RQCQPX'

Let 'X':=RQCQPX and 'x':=RQCQPx. If qcqprel returns a non-null answer,
then it is guaranteed that X >= x*x.T

qcqprel is the 'relaxation' of 'QCQP', it relaxes the constraint

X == x*x.T
to
X >= x*x.T
"""

def qcqprel(P, G, r=0.0 ):
    """
    min x' Q0 x + a0'x
    st
    x Qi x + ai x  + bi <=0
    Input Q size nxn
    a size nx1
    b is scalar
    output c size (n+1)*(n+2)/2 x 1

    x Q x + a'x +  b <= 0
    is converted to
    <q,X> <= 0
    where q is returned by this function 
    """
    def augQ(Q, a=None, b=None):
        n  =Q.size[0]
        if Q is None:
            Q=matrix(0., (n,n))
        if a is None:
            a=matrix(0., (n,1))
        if b is None:
            b=0.
            
        Q0 =matrix([[b, a*0.5], [a.T*0.5, Q]])
        jdx=0
        q=matrix(0., ((n+1)*(n+2)/2, 1) )
        for j in range(n+1):
            idx=matrix(range(j,n+1))
            tmp=Q0[idx, j]
            tmp[1:]*=2
            q[jdx+idx]=tmp
            jdx+=len(idx)-1
        return q

    """
    n is integer, size of matrix we want nnd
    returns the sparse 'sdp' matrix Q that
    sets up the constraint
    Force the matrix
    [ Y ] >= H
    
    where H is some nxn matrix, (not specified here)
    """
    def sdpmat(n=None):
        m=n*(n+1)/2
        Q =matrix(0., (n**2,m))
        J=0
        for j in range(n):
            for k in range(j,n):
                Q[k+j*n, J]=-1.
                J+=1
        Q=sparse(Q)
        return Q


    """
    Get the objective
    """
    n=P['P0'].size[0]
    I=matrix(0., (n,n))
    I[::n+1]=r
    
    Q0 =augQ(P['P0']+I, P['b0'], P['c0'])

    """
    Equality: force the [0,0] entry of solution to have
    value 1.
    """
    A=matrix(0., (1,Q0.size[0]))
    A[0]=1.
    b=matrix(1., (1,1))
    """
    Build the inequalities
    """
    Gl=matrix(0., (0, Q0.size[0]))
    for j in range( len( G['P'] ) ):
        Qk=augQ(G['P'][j], G['b'][j], G['c'][j])
        Gl=matrix([Gl, Qk.T])
    hl=matrix(0., (Gl.size[0], 1))
    """
    Build the equalities
    """
    for j in range( len( G['Peq'] ) ):
        Qk=augQ(G['Peq'][j], G['beq'][j], G['ceq'][j])
        A=matrix([A, Qk.T])
        b=matrix([b, 0.])
    
    """
    Get the 'semidef' matrix
    """
    G0=sdpmat(n+1)
    h0=matrix(0., (n+1,n+1))
    sol=solvers.sdp(Q0, Gl, hl, [G0],[h0], A, b)
    x=sol['x']
    """
    Build the matrices 'x' and 'X'
    """
    if x is not None:
        X=x[1:n+1]
        Y=x[n+1:]
        YY=matrix(0., (n,n))
        J=0
        for j in range(n):
            for k in range(j,n):
                YY[j,k]=Y[J]
                YY[k,j]=Y[J]
                J+=1
        sol['RQCQPx']=+sol['x'][1:n+1]
        sol['RQCQPX']=+YY
    else:
        sol['RQCQPx']=None
        sol['RQCQPX']=None
    return sol
