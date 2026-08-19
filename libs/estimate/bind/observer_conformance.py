#!/usr/bin/env python3
# =============================================================================
# Controlled Descent Simulator
# =============================================================================
# observer_conformance.py -- certifies the C++ continuous-time observer synthesis
# (libs/estimate/observer.hpp, exposed by observer_bench.cpp) against an
# independent Python oracle, with no third-party dependencies (ctypes + stdlib).
#
# It does NOT re-run a Riccati / sign-function solver. It takes the gain L the
# C++ side returns and certifies it is a genuine steady-state (Kalman-Bucy)
# optimum by an independent computation -- the exact DUAL of the LQR certificate.
# It forms the error dynamics A_cl = A - L C, solves the Lyapunov equation
#   A_cl P + P A_cl' = -(Qw + L Rv L')
# for the stationary error covariance P (a plain linear solve), and checks the
# filter stationarity condition
#   || L - P C' Rv^-1 ||  ~= 0.
# This is rigorous: a detectabilising L with L = P C' Rv^-1 necessarily makes P
# solve the filtering CARE (substitute and the cross terms cancel), so it is the
# stabilising Riccati solution and L is optimal. Positive-definiteness of P
# (checked by Cholesky) witnesses that A_cl is Hurwitz, i.e. L actually renders
# the plant detectable.
#
# Usage:  python3 observer_conformance.py <build-dir-containing-libobserver_bench>
# =============================================================================
import ctypes
import os
import sys

# ---- synthetic benchmark: three coupled 2nd-order channels [p_i, v_i] --------
# Two channels are open-loop UNSTABLE (negative damping) so the optimal gain is
# non-trivial and must actively make the error dynamics stable. We measure the
# three POSITIONS (the dual of driving the three velocities in the LQR bench).
# Deterministic; no physical meaning.
OMEGA = [1.3, 0.8, 1.7]
ZETA  = [-0.10, 0.05, -0.08]     # negative -> unstable channel
KAPPA = 0.4
NX, NY = 6, 3


def build_system():
    A = [[0.0]*NX for _ in range(NX)]
    for i in range(3):
        pi, vi, ni = 2*i, 2*i + 1, 2*((i + 1) % 3)
        A[pi][vi] = 1.0
        A[vi][pi] = -OMEGA[i]**2 - KAPPA
        A[vi][vi] = -2.0*ZETA[i]*OMEGA[i]
        A[vi][ni] += KAPPA
    # Measure the position of each channel.
    C = [[0.0]*NX for _ in range(NY)]
    for i in range(3):
        C[i][2*i] = 1.0
    # Process-noise covariance: excite the velocity (force) states.
    Qw = [[0.0]*NX for _ in range(NX)]
    for i in range(3):
        Qw[2*i][2*i] = 0.5        # position process noise
        Qw[2*i+1][2*i+1] = 3.0    # velocity process noise
    # Measurement-noise covariance per sensor.
    Rv = [[0.0]*NY for _ in range(NY)]
    for a, r in enumerate((0.08, 0.12, 0.10)):
        Rv[a][a] = r
    return A, C, Qw, Rv


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
    """Solve Acl P + P Acl' = -W for P (symmetric), by vectorising the linear map."""
    n = len(Acl)
    AclT = transpose(Acl)
    # operator L(P) = Acl P + P Acl', built column by column over the basis E_pq
    cols = []
    for p in range(n):
        for q in range(n):
            E = [[0.0]*n for _ in range(n)]
            E[p][q] = 1.0
            LE = matadd(matmul(Acl, E), matmul(E, AclT))
            cols.append([LE[i][j] for i in range(n) for j in range(n)])
    Mop = transpose(cols)                       # (n*n) x (n*n)
    rhs = [-W[i][j] for i in range(n) for j in range(n)]
    x = solve_linear(Mop, rhs)
    P = [[x[i*n + j] for j in range(n)] for i in range(n)]
    return [[0.5*(P[i][j] + P[j][i]) for j in range(n)] for i in range(n)]   # symmetrise


def load_lib(build_dir):
    for name in ("libobserver_bench.dylib", "libobserver_bench.so"):
        p = os.path.join(build_dir, name)
        if os.path.exists(p):
            return ctypes.CDLL(p)
    raise SystemExit(f"shared lib not found in {build_dir} (build with the bind CMake first)")


def main():
    build_dir = sys.argv[1] if len(sys.argv) > 1 else "build-observer-bind"
    lib = load_lib(build_dir)
    lib.observer_bench_dims.argtypes = [ctypes.POINTER(ctypes.c_int)]*2
    lib.observer_bench_solve.argtypes = [ctypes.POINTER(ctypes.c_double)]*5
    lib.observer_bench_solve.restype = ctypes.c_int

    nx = ctypes.c_int(); ny = ctypes.c_int()
    lib.observer_bench_dims(nx, ny)
    assert (nx.value, ny.value) == (NX, NY), (nx.value, ny.value)
    print(f"dims from C++: NX={nx.value} NY={ny.value}")

    A, C, Qw, Rv = build_system()

    def flat(M):
        return [M[i][j] for i in range(len(M)) for j in range(len(M[0]))]
    cA  = (ctypes.c_double*(NX*NX))(*flat(A))
    cC  = (ctypes.c_double*(NY*NX))(*flat(C))
    cQw = (ctypes.c_double*(NX*NX))(*flat(Qw))
    cRv = (ctypes.c_double*(NY*NY))(*flat(Rv))
    cL  = (ctypes.c_double*(NX*NY))()
    rc = lib.observer_bench_solve(cA, cC, cQw, cRv, cL)
    if rc != 0:
        print("C++ observer solver reported an error"); return 1
    L = [[cL[i*NY + a] for a in range(NY)] for i in range(NX)]

    # error dynamics and its stationary covariance P (Lyapunov)
    LC = matmul(L, C)
    Acl = matsub(A, LC)
    LRvLt = matmul(L, matmul(Rv, transpose(L)))
    W = [[Qw[i][j] + LRvLt[i][j] for j in range(NX)] for i in range(NX)]
    P = solve_lyapunov(Acl, W)

    # stationarity residual: L should equal P C' Rv^-1
    Lopt = matmul(P, matmul(transpose(C), inv(Rv)))
    rho = maxabs(matsub(L, Lopt))
    pd = cholesky_ok(P)

    print(f"stationarity residual  ||L - P C' Rv^-1||_max = {rho:.2e}")
    print(f"error-covariance P positive-definite (Hurwitz witness): {'yes' if pd else 'NO'}")

    ok = (rho < 1e-7) and pd
    print("OBSERVER CONFORMANCE PASSED" if ok else "OBSERVER CONFORMANCE FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
