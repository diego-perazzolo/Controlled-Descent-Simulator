#!/usr/bin/env python3
# =============================================================================
# Controlled Descent Simulator
# =============================================================================
# lqr_conformance.py -- certifies the C++ continuous-time LQR synthesis
# (libs/control/lqr.hpp, exposed by lqr_bench.cpp) against an independent Python
# oracle, with no third-party dependencies (ctypes + stdlib only).
#
# It does NOT re-run a Riccati / sign-function solver. Instead it takes the gain
# K the C++ side returns and certifies it is a genuine LQR optimum by an
# independent computation: it forms the closed loop A_cl = A - B K, solves the
# Lyapunov equation  A_cl' X + X A_cl = -(Q + K' R K)  for the closed-loop cost
# matrix X (a plain linear solve), and checks the LQR stationarity condition
#   || K - R^-1 B' X ||  ~= 0.
# This is rigorous: a stabilising K with K = R^-1 B' X necessarily makes X solve
# the CARE (substitute and the cross terms cancel), so it is the stabilising
# Riccati solution and K is optimal. Positive-definiteness of X (checked by
# Cholesky) witnesses that A_cl is Hurwitz, i.e. K actually stabilises the plant.
#
# The full numpy/scipy side-by-side derivation lives in
# modeling/notebooks/control/lqr.ipynb; this script is the dependency-free,
# CI-able optimality certificate.
#
# Usage:  python3 lqr_conformance.py <build-dir-containing-liblqr_bench>
# =============================================================================
import ctypes
import os
import sys

# ---- synthetic benchmark: three coupled 2nd-order channels [p_i, v_i] --------
# Two channels are open-loop UNSTABLE (negative damping) so the optimal gain is
# non-trivial and must actively stabilise. Deterministic; no physical meaning.
OMEGA = [1.3, 0.8, 1.7]
ZETA  = [-0.10, 0.05, -0.08]     # negative -> unstable channel
GAIN  = [1.0, 1.2, 0.9]
KAPPA = 0.4
NX, NU = 6, 3


def build_system():
    A = [[0.0]*NX for _ in range(NX)]
    B = [[0.0]*NU for _ in range(NX)]
    for i in range(3):
        pi, vi, ni = 2*i, 2*i + 1, 2*((i + 1) % 3)
        A[pi][vi] = 1.0
        A[vi][pi] = -OMEGA[i]**2 - KAPPA
        A[vi][vi] = -2.0*ZETA[i]*OMEGA[i]
        A[vi][ni] += KAPPA
        B[vi][i]  = GAIN[i]
    Q = [[0.0]*NX for _ in range(NX)]
    for i in range(3):
        Q[2*i][2*i] = 3.0        # position weight
        Q[2*i+1][2*i+1] = 0.5    # velocity weight
    R = [[0.0]*NU for _ in range(NU)]
    for a, r in enumerate((0.08, 0.12, 0.10)):
        R[a][a] = r
    return A, B, Q, R


# ---- tiny stdlib linear algebra (list-of-lists matrices) --------------------
def matmul(A, B):
    n, k, m = len(A), len(B), len(B[0])
    return [[sum(A[i][p]*B[p][j] for p in range(k)) for j in range(m)] for i in range(n)]

def transpose(A):
    return [[A[i][j] for i in range(len(A))] for j in range(len(A[0]))]

def matsub(A, B):
    return [[A[i][j] - B[i][j] for j in range(len(A[0]))] for i in range(len(A))]

def matadd(A, B):
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]

def maxabs(A):
    return max(abs(A[i][j]) for i in range(len(A)) for j in range(len(A[0])))

def solve_linear(M, b):
    """Solve M x = b (M square) by Gaussian elimination with partial pivoting."""
    n = len(M)
    a = [row[:] + [b[i]] for i, row in enumerate(M)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[piv][col]) < 1e-300:
            raise ZeroDivisionError("singular system")
        a[col], a[piv] = a[piv], a[col]
        d = a[col][col]
        a[col] = [v / d for v in a[col]]
        for r in range(n):
            if r == col:
                continue
            f = a[r][col]
            if f != 0.0:
                a[r] = [a[r][j] - f*a[col][j] for j in range(n + 1)]
    return [a[i][n] for i in range(n)]

def inv(A):
    n = len(A)
    cols = [solve_linear(A, [1.0 if i == c else 0.0 for i in range(n)]) for c in range(n)]
    return transpose(cols)

def cholesky_ok(A, eps=1e-9):
    """True iff A is symmetric positive definite (Cholesky succeeds)."""
    n = len(A)
    L = [[0.0]*n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            s = A[i][j] - sum(L[i][k]*L[j][k] for k in range(j))
            if i == j:
                if s <= eps:
                    return False
                L[i][j] = s**0.5
            else:
                L[i][j] = s / L[j][j]
    return True


def solve_lyapunov(Acl, W):
    """Solve Acl' X + X Acl = -W for X (symmetric), by vectorising the linear map."""
    n = len(Acl)
    AclT = transpose(Acl)
    # operator L(X) = Acl' X + X Acl, built column by column over the basis E_pq
    cols = []
    for p in range(n):
        for q in range(n):
            E = [[0.0]*n for _ in range(n)]
            E[p][q] = 1.0
            LE = matadd(matmul(AclT, E), matmul(E, Acl))
            cols.append([LE[i][j] for i in range(n) for j in range(n)])
    Mop = transpose(cols)                       # (n*n) x (n*n)
    rhs = [-W[i][j] for i in range(n) for j in range(n)]
    x = solve_linear(Mop, rhs)
    X = [[x[i*n + j] for j in range(n)] for i in range(n)]
    return [[0.5*(X[i][j] + X[j][i]) for j in range(n)] for i in range(n)]   # symmetrise


def load_lib(build_dir):
    for name in ("liblqr_bench.dylib", "liblqr_bench.so"):
        p = os.path.join(build_dir, name)
        if os.path.exists(p):
            return ctypes.CDLL(p)
    raise SystemExit(f"shared lib not found in {build_dir} (build with the bind CMake first)")


def main():
    build_dir = sys.argv[1] if len(sys.argv) > 1 else "build-ilqr-bind"
    lib = load_lib(build_dir)
    lib.lqr_bench_dims.argtypes = [ctypes.POINTER(ctypes.c_int)]*2
    lib.lqr_bench_solve.argtypes = [ctypes.POINTER(ctypes.c_double)]*5
    lib.lqr_bench_solve.restype = ctypes.c_int

    nx = ctypes.c_int(); nu = ctypes.c_int()
    lib.lqr_bench_dims(nx, nu)
    assert (nx.value, nu.value) == (NX, NU), (nx.value, nu.value)
    print(f"dims from C++: NX={nx.value} NU={nu.value}")

    A, B, Q, R = build_system()

    def flat(M):
        return [M[i][j] for i in range(len(M)) for j in range(len(M[0]))]
    cA = (ctypes.c_double*(NX*NX))(*flat(A))
    cB = (ctypes.c_double*(NX*NU))(*flat(B))
    cQ = (ctypes.c_double*(NX*NX))(*flat(Q))
    cR = (ctypes.c_double*(NU*NU))(*flat(R))
    cK = (ctypes.c_double*(NU*NX))()
    rc = lib.lqr_bench_solve(cA, cB, cQ, cR, cK)
    if rc != 0:
        print("C++ LQR solver reported an error"); return 1
    K = [[cK[a*NX + j] for j in range(NX)] for a in range(NU)]

    # closed loop and its cost matrix X (Lyapunov)
    BK = matmul(B, K)
    Acl = matsub(A, BK)
    KtRK = matmul(transpose(K), matmul(R, K))
    W = [[Q[i][j] + KtRK[i][j] for j in range(NX)] for i in range(NX)]
    X = solve_lyapunov(Acl, W)

    # stationarity residual: K should equal R^-1 B' X
    Kopt = matmul(inv(R), matmul(transpose(B), X))
    rho = maxabs(matsub(K, Kopt))
    pd = cholesky_ok(X)

    print(f"stationarity residual  ||K - R^-1 B' X||_max = {rho:.2e}")
    print(f"closed-loop cost X positive-definite (Hurwitz witness): {'yes' if pd else 'NO'}")

    ok = (rho < 1e-7) and pd
    print("LQR CONFORMANCE PASSED" if ok else "LQR CONFORMANCE FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
