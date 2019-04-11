import numpy as np


class CPDParams:
    def __init__(self,
                 alpha=3.0,
                 beta=1.0,
                 w=0.1,
                 tolerance=1e-4,
                 max_iter=100,
                 init_sigma_scale=1/8,
                 lambd=1.0,
                 lambd_annealing_factor=0.6,
                 Y_emit_prior = None,
                 M_LLE=None):
        """
        Parameters for our version of Coherent Point Drift.
        :param alpha: Trade-off parameter between fitting and CPD
         regularization. Higher means more regularization.
        :param beta: Width for Gaussian kernels used in CPD regularization.
        :param w: Prior probability for outliers. Higher w result in higher
         robustness against noise, but lowers accuracy.
        :param tolerance: Convergence tolerance for CPD.
        :param max_iter: Maximum number of iterations for CPD.
        :param init_sigma_scale: Scaling on estimated variance. Used to
         accelerate convergence.
        :param lambd: Trade-off parameter for LLE regularization.
        Higher means more regularization.
        :param lambd_annealing_factor: Exponential decay rate for lambd. Used to
        accelerate convergence.
        :param Y_emit_prior: (M,) Prior probability for a gaussian centroid to generate observation.
        :param M_LLE: (M, M) LLE adjacency matrix. Should be pre-computed.
        occlusion.
        """
        self.alpha = alpha
        self.beta = beta
        self.w = w
        self.tolerance = tolerance
        self.max_iter = max_iter
        self.init_sigma_scale = init_sigma_scale
        self.lambd = lambd
        self.lambd_annealing_factor = lambd_annealing_factor
        self.Y_emit_prior = Y_emit_prior
        self.M_LLE = M_LLE


class CPD:
    def __init__(self, X, Y, params: CPDParams):
        """
        :param X: (N, 3) Target point cloud.
        :param Y: (M, 3) Template vertices.
        :param params: Type of CPDParams.
        """
        self.X = X
        self.Y = Y
        self.params = params

        N, D = X.shape
        M, _ = Y.shape
        self.W = np.zeros((M, D))
        self.G = self._gaussian_kernel(Y)
        self.TY = self.Y + self.G @ self.W
        self.P = np.zeros((M, N))

        self.sigma2 = self._initialize_sigma2(self.X, self.TY) * self.params.init_sigma_scale
        self.callback = None

    def run(self):
        """
        Run CPD until convergence.
        :return: (M, 3) Resulting vertices.
        """

        err = self.params.tolerance + 1
        for iter_num in range(self.params.max_iter):
            if err <= self.params.tolerance:
                break

            qprev = self.sigma2

            self._expectation_iter(iter_num)
            self._maximization_iter(iter_num)

            if self.sigma2 <= 0:
                self.sigma2 = self.params.tolerance / 10
            err = np.abs(self.sigma2 - qprev)

            if callable(self.callback):
                kwargs = {
                    'iteration': iter_num,
                    'error': err,
                    'X': self.X,
                    'Y': self.TY,
                    'W': self.W,
                    'P': self.P
                }
                self.callback(**kwargs)
        return self.TY

    def _gaussian_kernel(self, Y):
        (M, D) = Y.shape
        XX = np.reshape(Y, (1, M, D))
        YY = np.reshape(Y, (M, 1, D))
        XX = np.tile(XX, (M, 1, 1))
        YY = np.tile(YY, (1, M, 1))
        diff = XX - YY
        diff = np.multiply(diff, diff)
        diff = np.sum(diff, 2)
        return np.exp(-diff / (2 * self.params.beta))

    def _initialize_sigma2(self, X, Y):
        (N, D) = X.shape
        (M, _) = Y.shape
        XX = np.reshape(X, (1, N, D))
        YY = np.reshape(Y, (M, 1, D))
        XX = np.tile(XX, (M, 1, 1))
        YY = np.tile(YY, (1, N, 1))
        diff = XX - YY
        err  = np.multiply(diff, diff)
        return np.sum(err) / (D * M * N)

    def _expectation_iter(self, iter_num):
        N, D = self.X.shape
        M, _ = self.Y.shape

        P = np.zeros((M, N))
        for i in range(M):
            diff = self.X - np.tile(self.TY[i, :], (N, 1))
            diff = np.multiply(diff, diff)
            P[i, :] = P[i, :] + np.sum(diff, axis=1)

        c = (2 * np.pi * self.sigma2) ** (D / 2)
        c = c * self.params.w / (1 - self.params.w)
        c = c * M / N

        P = np.exp(-P / (2 * self.sigma2))
        if self.params.Y_emit_prior is not None:
            P *= self.params.Y_emit_prior[:, np.newaxis]

        den = np.sum(P, axis=0)
        den = np.tile(den, (M, 1))
        den[den == 0] = np.finfo(float).eps
        den += c

        self.P = np.divide(P, den)

    def _maximization_iter(self, iter_num):
        N, D = self.X.shape
        M, _ = self.Y.shape

        P = self.P
        Pt1 = np.sum(P, axis=0)
        P1 = np.sum(P, axis=1)
        Np = np.sum(P1)

        # solve for W
        if self.params.M_LLE is not None:
            # anneling lambd
            lambd = self.params.lambd * (self.params.lambd_annealing_factor ** (iter_num + 1))

            A = np.diag(P1) @ self.G\
                + self.params.alpha * self.sigma2 * np.eye(M)\
                + self.sigma2 * lambd * (self.params.M_LLE @ self.G)
            B = P @ self.X - (np.diag(P1) + self.sigma2 * lambd * self.params.M_LLE) @ self.Y
            self.W = np.asarray(np.linalg.solve(A, B))
        else:
            A = np.dot(np.diag(P1), self.G) + self.params.alpha * self.sigma2 * np.eye(M)
            B = np.dot(P, self.X) - np.dot(np.diag(P1), self.Y)
            self.W = np.linalg.solve(A, B)

        self.TY = self.Y + np.dot(self.G, self.W)

        xPx = np.dot(np.transpose(Pt1), np.sum(np.multiply(self.X, self.X), axis=1))
        yPy = np.dot(np.transpose(P1), np.sum(np.multiply(self.TY, self.TY), axis=1))
        trPXY = np.sum(np.multiply(self.TY, np.dot(P, self.X)))
        self.sigma2 = (xPx - 2 * trPXY + yPy) / (Np * D)
