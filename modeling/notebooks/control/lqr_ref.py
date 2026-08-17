"""lqr_ref.py -- vehicle-agnostic Python reference for the continuous-time LQR
synthesis by the matrix-sign function.

This is the single source of the Python LQR synthesis used across the modeling
notebooks. It mirrors the hand-written C++ in ``libs/control/lqr.hpp``
one-for-one (same Hamiltonian, same scaled-Newton matrix sign, same stable-
subspace extraction and gain formula), and is pinned to it by the conformance
cell in ``control/lqr.ipynb`` and the dependency-free
``libs/control/bind/lqr_conformance.py``.

It is deliberately *generic*: it knows nothing about vehicles or references. The
caller supplies the plant ``(A, B)`` and the weights ``(Q, R)`` of the cost
integral ``x'Q x + u'R u`` and gets back the optimal gain ``K`` (control law
``u = -K x``) and the Riccati solution ``X``. Model-specific pieces (which error
dynamics ``A, B`` and which weights ``Q, R`` a given vehicle uses) live with the
model, in the ``model/`` notebooks -- never here. See AGENTS.md golden rule 10.
"""
import numpy as np


def matrix_sign(Z, max_iters=100, tol=1e-13):
    """Matrix sign function of ``Z`` by the scaled Newton iteration

        Z <- 1/2 ( c Z + (c Z)^-1 ),   c = sqrt(||Z^-1||_F / ||Z||_F),

    which converges quadratically to ``sign(Z)`` whenever ``Z`` has no eigenvalue
    on the imaginary axis. The scaling ``c`` (Higham) only speeds the first few
    iterations; it tends to 1 near convergence. Identical to the loop in the
    runtime ``CDS::control::lqr``."""
    Z = np.array(Z, dtype=float)
    for _ in range(max_iters):
        Zinv = np.linalg.inv(Z)
        nZ, nZi = np.linalg.norm(Z, 'fro'), np.linalg.norm(Zinv, 'fro')
        c = np.sqrt(nZi / nZ) if nZ > 0 and nZi > 0 else 1.0
        Znew = 0.5 * (c * Z + Zinv / c)
        if np.linalg.norm(Znew - Z, 'fro') <= tol * (np.linalg.norm(Znew, 'fro') + 1.0):
            return Znew
        Z = Znew
    raise RuntimeError("matrix sign iteration did not converge")


def hamiltonian(A, B, Q, R):
    """The Hamiltonian matrix  H = [[A, -G], [-Q, -A']]  with  G = B R^-1 B',
    whose stable invariant subspace carries the stabilising Riccati solution."""
    A, B, Q, R = map(lambda M: np.asarray(M, float), (A, B, Q, R))
    G = B @ np.linalg.solve(R, B.T)
    n = A.shape[0]
    H = np.zeros((2 * n, 2 * n))
    H[:n, :n] = A
    H[:n, n:] = -G
    H[n:, :n] = -Q
    H[n:, n:] = -A.T
    return H


def care(A, B, Q, R, **kw):
    """Stabilising solution ``X`` (symmetric, positive-semidefinite) of the
    continuous algebraic Riccati equation

        A'X + X A - X B R^-1 B' X + Q = 0.

    The stable invariant subspace of the Hamiltonian ``H`` is the eigenspace of
    ``sign(H)`` for the eigenvalue -1, i.e. ``null(sign(H) + I)``. Writing its
    basis as columns ``[I; X]``, ``X`` solves the overdetermined
    ``[S12; S22+I] X = -[S11+I; S21]`` in the least-squares sense."""
    A, B, Q, R = map(lambda M: np.asarray(M, float), (A, B, Q, R))
    n = A.shape[0]
    S = matrix_sign(hamiltonian(A, B, Q, R), **kw)
    S11, S12, S21, S22 = S[:n, :n], S[:n, n:], S[n:, :n], S[n:, n:]
    U = np.vstack([S12, S22 + np.eye(n)])
    W = np.vstack([S11 + np.eye(n), S21])
    X, *_ = np.linalg.lstsq(U, -W, rcond=None)
    return 0.5 * (X + X.T)                     # symmetrise (the CARE solution is symmetric)


def lqr(A, B, Q, R, **kw):
    """Optimal LQR gain ``K = R^-1 B' X`` (control law ``u = -K x``) and the
    Riccati solution ``X``, for the cost integral ``x'Q x + u'R u``."""
    A, B, Q, R = map(lambda M: np.asarray(M, float), (A, B, Q, R))
    X = care(A, B, Q, R, **kw)
    K = np.linalg.solve(R, B.T @ X)
    return K, X


def care_residual(A, B, Q, R, X):
    """The CARE left-hand side ``A'X + XA - X B R^-1 B' X + Q`` at ``X`` (``~0``
    for the true solution) -- an implementation-independent optimality check."""
    A, B, Q, R, X = map(lambda M: np.asarray(M, float), (A, B, Q, R, X))
    G = B @ np.linalg.solve(R, B.T)
    return A.T @ X + X @ A - X @ G @ X + Q
