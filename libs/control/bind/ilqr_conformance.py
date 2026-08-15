#!/usr/bin/env python3
# =============================================================================
# Controlled Descent Simulator
# =============================================================================
# ilqr_conformance.py -- certifies the C++ generic iLQR solver
# (libs/control/ilqr.hpp, exposed by ilqr_bench.cpp) against an independent
# Python oracle, with no third-party dependencies (ctypes + stdlib only).
#
# It does NOT re-run a second full iLQR. Instead it takes the command sequence
# the C++ solver returns and certifies it is a genuine constrained optimum of
# the identical benchmark problem: it rolls the sequence out through the same
# dynamics, forms the adjoint (costate) gradient of the total cost w.r.t. the
# controls, and checks the box-projected gradient (the KKT stationarity
# residual) is ~0. The discrete Jacobians used by the adjoint are computed by
# central finite differences of the RK4 step -- an implementation independent of
# the solver's own sensitivity propagation -- and the model derivatives are
# self-checked against finite differences before use.
#
# The full pedagogical Python iLQR (numpy) that mirrors the derivation lives in
# modeling/notebooks/control/ilqr.ipynb; this script is the dependency-free,
# CI-able optimality certificate.
#
# Usage:  python3 ilqr_conformance.py <build-dir-containing-libilqr_bench>
# =============================================================================
import ctypes
import os
import sys

# ---- benchmark constants (must match libs/control/bind/ilqr_bench.cpp) -------
OMEGA = [1.3, 0.8, 1.7]
ZETA  = [0.10, 0.05, 0.15]
GAIN  = [1.0, 1.2, 0.9]
BETA  = [0.20, 0.35, 0.15]
KAPPA = 0.4
QP, QV, RU, WTERM = 3.0, 0.5, 0.08, 15.0
DT = 0.05


def f(x, u):
    d = [0.0] * 6
    for i in range(3):
        p, v, pn = x[2*i], x[2*i + 1], x[2*((i + 1) % 3)]
        d[2*i]     = v
        d[2*i + 1] = (-OMEGA[i]**2 * p - 2*ZETA[i]*OMEGA[i]*v
                      + GAIN[i]*u[i] - BETA[i]*p**3 + KAPPA*(pn - p))
    return d


def jac(x):
    fx = [[0.0]*6 for _ in range(6)]
    fu = [[0.0]*3 for _ in range(6)]
    for i in range(3):
        pi, vi, ni = 2*i, 2*i + 1, 2*((i + 1) % 3)
        p = x[pi]
        fx[pi][vi] = 1.0
        fx[vi][pi] = -OMEGA[i]**2 - 3*BETA[i]*p*p - KAPPA
        fx[vi][vi] = -2*ZETA[i]*OMEGA[i]
        fx[vi][ni] += KAPPA
        fu[vi][i]  = GAIN[i]
    return fx, fu


def rk4(x, u, dt):
    k1 = f(x, u)
    x2 = [x[i] + 0.5*dt*k1[i] for i in range(6)]; k2 = f(x2, u)
    x3 = [x[i] + 0.5*dt*k2[i] for i in range(6)]; k3 = f(x3, u)
    x4 = [x[i] + dt*k3[i]     for i in range(6)]; k4 = f(x4, u)
    return [x[i] + dt/6.0*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) for i in range(6)]


def sensitivity_fd(x, u, dt, eps=1e-6):
    """A = dFd/dx, B = dFd/du by central differences of the RK4 step."""
    A = [[0.0]*6 for _ in range(6)]
    B = [[0.0]*3 for _ in range(6)]
    for j in range(6):
        xp, xm = list(x), list(x); xp[j] += eps; xm[j] -= eps
        fp, fm = rk4(xp, u, dt), rk4(xm, u, dt)
        for i in range(6): A[i][j] = (fp[i] - fm[i]) / (2*eps)
    for a in range(3):
        up, um = list(u), list(u); up[a] += eps; um[a] -= eps
        fp, fm = rk4(x, up, dt), rk4(x, um, dt)
        for i in range(6): B[i][a] = (fp[i] - fm[i]) / (2*eps)
    return A, B


def stage_lx(x): return [ (QP if j % 2 == 0 else QV) * x[j] for j in range(6) ]
def term_lx(x):  return [ WTERM * (QP if j % 2 == 0 else QV) * x[j] for j in range(6) ]


def total_cost(xs, us, N):
    J = 0.0
    for k in range(N):
        x, u = xs[k], us[k]
        s = sum(QP*x[2*i]**2 + QV*x[2*i+1]**2 for i in range(3)) + sum(RU*u[a]**2 for a in range(3))
        J += 0.5*s
    xN = xs[N]
    J += 0.5*WTERM*sum(QP*xN[2*i]**2 + QV*xN[2*i+1]**2 for i in range(3))
    return J


def kkt_residual(xs, us, N, umax):
    """max box-projected component of dJ/du_k over the horizon (~0 at optimum)."""
    lam = term_lx(xs[N])
    worst = 0.0
    for k in range(N - 1, -1, -1):
        A, B = sensitivity_fd(xs[k], us[k], DT)
        for a in range(3):
            g = RU*us[k][a] + sum(B[i][a]*lam[i] for i in range(6))   # dJ/du_k[a]
            uk = us[k][a]
            if uk >= umax - 1e-9 and g < 0: g = 0.0     # active at upper bound
            if uk <= -umax + 1e-9 and g > 0: g = 0.0    # active at lower bound
            worst = max(worst, abs(g))
        lx = stage_lx(xs[k])
        lam = [lx[j] + sum(A[i][j]*lam[i] for i in range(6)) for j in range(6)]
    return worst


def load_lib(build_dir):
    for name in ("libilqr_bench.dylib", "libilqr_bench.so"):
        p = os.path.join(build_dir, name)
        if os.path.exists(p):
            return ctypes.CDLL(p)
    raise SystemExit(f"shared lib not found in {build_dir} (build with the bind CMake first)")


def main():
    build_dir = sys.argv[1] if len(sys.argv) > 1 else "build-ilqr-bind"
    lib = load_lib(build_dir)

    lib.ilqr_bench_dims.argtypes = [ctypes.POINTER(ctypes.c_int)]*3
    lib.ilqr_bench_solve.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_double, ctypes.c_int,
                                     ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
                                     ctypes.POINTER(ctypes.c_double)]
    nx = ctypes.c_int(); nu = ctypes.c_int(); nn = ctypes.c_int()
    lib.ilqr_bench_dims(nx, nu, nn)
    NX, NU, N = nx.value, nu.value, nn.value
    assert (NX, NU) == (6, 3), (NX, NU)
    print(f"dims from C++: NX={NX} NU={NU} N={N}")

    # -- self-check the oracle: analytic jac vs finite differences of f --
    xchk = [0.7, -0.3, -1.1, 0.4, 0.6, -0.2]; uchk = [0.5, -0.7, 0.3]
    fxa, fua = jac(xchk)
    eps = 1e-6; worst = 0.0
    for j in range(6):
        xp, xm = list(xchk), list(xchk); xp[j] += eps; xm[j] -= eps
        fp, fm = f(xp, uchk), f(xm, uchk)
        for i in range(6): worst = max(worst, abs(fxa[i][j] - (fp[i]-fm[i])/(2*eps)))
    print(f"oracle self-check (jac vs FD): max err = {worst:.2e}")
    if worst > 1e-6:
        print("ORACLE SELF-CHECK FAILED"); return 1

    x0 = [0.9, 0.0, -1.1, 0.0, 0.7, 0.0]
    MAXIT = 80
    ok = True
    for label, umax in (("loose box (umax=5.0)", 5.0), ("tight box (umax=0.6)", 0.6)):
        cx0  = (ctypes.c_double*NX)(*x0)
        warm_in  = (ctypes.c_double*(N*NU))(*([0.0]*(N*NU)))
        u0_out   = (ctypes.c_double*NU)()
        warm_out = (ctypes.c_double*(N*NU))()
        lib.ilqr_bench_solve(cx0, ctypes.c_double(umax), ctypes.c_int(MAXIT), warm_in, u0_out, warm_out)

        # reconstruct full converged control sequence: us[0]=u0, us[k]=warm_out[k-1]
        us = [[u0_out[a] for a in range(NU)]]
        for k in range(1, N):
            us.append([warm_out[(k-1)*NU + a] for a in range(NU)])
        # roll out and certify optimality
        xs = [list(x0)]
        for k in range(N): xs.append(rk4(xs[k], us[k], DT))
        boxOK = all(-umax - 1e-9 <= us[k][a] <= umax + 1e-9 for k in range(N) for a in range(NU))
        J = total_cost(xs, us, N)
        kkt = kkt_residual(xs, us, N, umax)
        good = boxOK and kkt < 1e-6   # observed ~1e-9; FD floor ~1e-8, ample margin
        print(f"{label:22s}: cost={J:8.4f}  box={'ok' if boxOK else 'VIOLATED'}  KKT residual={kkt:.2e}  -> {'PASS' if good else 'FAIL'}")
        ok = ok and good

    print("ILQR CONFORMANCE PASSED" if ok else "ILQR CONFORMANCE FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
