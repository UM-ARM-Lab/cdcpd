import numpy as np
from scipy.linalg import solve
from scipy.sparse import eye, csr_matrix

from sklearn.utils import check_array
from sklearn.utils.validation import FLOAT_DTYPES
from sklearn.neighbors import NearestNeighbors


def barycenter_weights(X, Z, reg=1e-3):
    """Compute barycenter weights of X from Y along the first axis

    We estimate the weights to assign to each point in Y[i] to recover
    the point X[i]. The barycenter weights sum to 1.

    Parameters
    ----------
    X : array-like, shape (n_samples, n_dim)

    Z : array-like, shape (n_samples, n_neighbors, n_dim)

    reg : float, optional
        amount of regularization to add for the problem to be
        well-posed in the case of n_neighbors > n_dim

    Returns
    -------
    B : array-like, shape (n_samples, n_neighbors)

    Notes
    -----
    See developers note for more information.
    """
    X = check_array(X, dtype=FLOAT_DTYPES)
    Z = check_array(Z, dtype=FLOAT_DTYPES, allow_nd=True)

    n_samples, n_neighbors = X.shape[0], Z.shape[1]
    B = np.empty((n_samples, n_neighbors), dtype=X.dtype)
    v = np.ones(n_neighbors, dtype=X.dtype)

    # this might raise a LinalgError if G is singular and has trace
    # zero
    for i, A in enumerate(Z.transpose(0, 2, 1)):
        C = A.T - X[i]  # broadcasting
        G = np.dot(C, C.T)
        trace = np.trace(G)
        if trace > 0:
            R = reg * trace
        else:
            R = reg
        G.flat[::Z.shape[1] + 1] += R
        w = solve(G, v, sym_pos=True)
        B[i, :] = w / np.sum(w)
    return B


def barycenter_kneighbors_graph(X, n_neighbors, reg=1e-3, n_jobs=None):
    """Computes the barycenter weighted graph of k-Neighbors for points in X

    Parameters
    ----------
    X : {array-like, NearestNeighbors}
        Sample data, shape = (n_samples, n_features), in the form of a
        numpy array or a NearestNeighbors object.

    n_neighbors : int
        Number of neighbors for each sample.

    reg : float, optional
        Amount of regularization when solving the least-squares
        problem. Only relevant if mode='barycenter'. If None, use the
        default.

    n_jobs : int or None, optional (default=None)
        The number of parallel jobs to run for neighbors search.
        ``None`` means 1 unless in a :obj:`joblib.parallel_backend` context.
        ``-1`` means using all processors. See :term:`Glossary <n_jobs>`
        for more details.

    Returns
    -------
    A : sparse matrix in CSR format, shape = [n_samples, n_samples]
        A[i, j] is assigned the weight of edge that connects i to j.

    See also
    --------
    sklearn.neighbors.kneighbors_graph
    sklearn.neighbors.radius_neighbors_graph
    """
    knn = NearestNeighbors(n_neighbors + 1, n_jobs=n_jobs).fit(X)
    X = knn._fit_X
    n_samples = X.shape[0]
    ind = knn.kneighbors(X, return_distance=False)[:, 1:]
    data = barycenter_weights(X, X[ind], reg=reg)
    indptr = np.arange(0, n_samples * n_neighbors + 1, n_neighbors)
    return csr_matrix((data.ravel(), ind.ravel(), indptr),
                      shape=(n_samples, n_samples))


def locally_linear_embedding(
        X, n_neighbors, reg=1e-3, eigen_solver='auto', n_jobs=None):
    """Perform a Locally Linear Embedding analysis on the data.

    Read more in the :ref:`User Guide <locally_linear_embedding>`.

    Parameters
    ----------
    X : {array-like, NearestNeighbors}
        Sample data, shape = (n_samples, n_features), in the form of a
        numpy array or a NearestNeighbors object.

    n_neighbors : integer
        number of neighbors to consider for each point.

    reg : float
        regularization constant, multiplies the trace of the local covariance
        matrix of the distances.

    eigen_solver : string, {'auto', 'arpack', 'dense'}
        auto : algorithm will attempt to choose the best method for input data

        arpack : use arnoldi iteration in shift-invert mode.
                    For this method, M may be a dense matrix, sparse matrix,
                    or general linear operator.
                    Warning: ARPACK can be unstable for some problems.  It is
                    best to try several random seeds in order to check results.

        dense  : use standard dense matrix operations for the eigenvalue
                    decomposition.  For this method, M must be an array
                    or matrix type.  This method should be avoided for
                    large problems.

    n_jobs : int or None, optional (default=None)
        The number of parallel jobs to run for neighbors search.
        ``None`` means 1 unless in a :obj:`joblib.parallel_backend` context.
        ``-1`` means using all processors. See :term:`Glossary <n_jobs>`
        for more details.

    Returns
    -------
    Y : array-like, shape [n_samples, n_components]
        Embedding vectors.

    squared_error : float
        Reconstruction error for the embedding vectors. Equivalent to
        ``norm(Y - W Y, 'fro')**2``, where W are the reconstruction weights.

    References
    ----------

    .. [1] `Roweis, S. & Saul, L. Nonlinear dimensionality reduction
        by locally linear embedding.  Science 290:2323 (2000).`
    .. [2] `Donoho, D. & Grimes, C. Hessian eigenmaps: Locally
        linear embedding techniques for high-dimensional data.
        Proc Natl Acad Sci U S A.  100:5591 (2003).`
    .. [3] `Zhang, Z. & Wang, J. MLLE: Modified Locally Linear
        Embedding Using Multiple Weights.`
        http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.70.382
    .. [4] `Zhang, Z. & Zha, H. Principal manifolds and nonlinear
        dimensionality reduction via tangent space alignment.
        Journal of Shanghai Univ.  8:406 (2004)`
    """
    if eigen_solver not in ('auto', 'arpack', 'dense'):
        raise ValueError("unrecognized eigen_solver '%s'" % eigen_solver)

    nbrs = NearestNeighbors(n_neighbors=n_neighbors + 1, n_jobs=n_jobs)
    nbrs.fit(X)
    X = nbrs._fit_X

    N, d_in = X.shape

    if n_neighbors >= N:
        raise ValueError(
            "Expected n_neighbors <= n_samples, "
            " but n_samples = %d, n_neighbors = %d" %
            (N, n_neighbors)
        )

    if n_neighbors <= 0:
        raise ValueError("n_neighbors must be positive")

    M_sparse = (eigen_solver != 'dense')

    W = barycenter_kneighbors_graph(
        nbrs, n_neighbors=n_neighbors, reg=reg, n_jobs=n_jobs)

    # we'll compute M = (I-W)'(I-W)
    # depending on the solver, we'll do this differently
    if M_sparse:
        M = eye(*W.shape, format=W.format) - W
        # M = (M.T * M).tocsr()
        M = (M.T * M)
    else:
        M = (W.T * W - W.T - W).toarray()
        M.flat[::M.shape[0] + 1] += 1  # W = W - I = W - I

    return M

