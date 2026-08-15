"""ilqr_ref.py -- vehicle-agnostic Python reference for the control-limited
iLQR / DDP solver.

This is the single source of the Python solver used across the modeling
notebooks. It mirrors the hand-written C++ in ``libs/control/ilqr.hpp``
one-for-one (same backward/forward passes, box-QP, Levenberg schedule), and is
pinned to it by the conformance test in ``control/ilqr.ipynb`` and the
dependency-free ``libs/control/bind/ilqr_conformance.py``.

It is deliberately *generic*: it knows nothing about quadrotors, quaternions or
references. The caller supplies

    f(x, u)                 -> continuous state derivative dx/dt          (np.ndarray)
    jac(x, u)               -> (fx, fu) continuous Jacobians df/dx, df/du (np.ndarray)
    project(x)              -> state after a post-step projection (e.g. quaternion
                               renorm); pass ``lambda x: x`` if none is needed
    stage_cost(x, u, k)     -> (val, lx, lxx, lu, luu) at stage k
    term_cost(x)            -> (val, lx, lxx) at the horizon end

with per-input actuator bounds ``lo, hi`` (arrays) and step ``dt``. Model-specific
pieces (the tracking cost, the reference sampling) live with the model, in the
``model/`` notebooks -- never here. See AGENTS.md golden rule 10.
"""
import numpy as np


# ---- integration & discrete sensitivity ------------------------------------
def rk4_step(x, u, f, dt):
    k1 = f(x, u); k2 = f(x + 0.5*dt*k1, u)
    k3 = f(x + 0.5*dt*k2, u); k4 = f(x + dt*k3, u)
    return x + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)


def sensitivity(x, u, f, jac, dt):
    """Discrete Jacobians A = dF/dx, B = dF/du of one RK4 step, by the chain rule
    (reuses the continuous jac). Identical to the runtime ``rk4_jacobians``."""
    I = np.eye(x.size)
    k1 = f(x, u); x2 = x + 0.5*dt*k1
    k2 = f(x2, u); x3 = x + 0.5*dt*k2
    k3 = f(x3, u); x4 = x + dt*k3
    A1, B1 = jac(x, u)
    J2fx, J2fu = jac(x2, u); A2 = J2fx @ (I + 0.5*dt*A1); B2 = J2fx @ (0.5*dt*B1) + J2fu
    J3fx, J3fu = jac(x3, u); A3 = J3fx @ (I + 0.5*dt*A2); B3 = J3fx @ (0.5*dt*B2) + J3fu
    J4fx, J4fu = jac(x4, u); A4 = J4fx @ (I + dt*A3);     B4 = J4fx @ (dt*B3)     + J4fu
    A = I + (dt/6.0)*(A1 + 2*A2 + 2*A3 + A4)
    B =     (dt/6.0)*(B1 + 2*B2 + 2*B3 + B4)
    return A, B


def fd_sensitivity(x, u, f, dt, eps=1e-6):
    """A, B by central finite differences of the RK4 step -- an independent check
    of :func:`sensitivity` for acid tests."""
    nx, nu = x.size, u.size
    A = np.zeros((nx, nx)); B = np.zeros((nx, nu))
    for j in range(nx):
        e = np.zeros(nx); e[j] = eps
        A[:, j] = (rk4_step(x+e, u, f, dt) - rk4_step(x-e, u, f, dt)) / (2*eps)
    for a in range(nu):
        e = np.zeros(nu); e[a] = eps
        B[:, a] = (rk4_step(x, u+e, f, dt) - rk4_step(x, u-e, f, dt)) / (2*eps)
    return A, B


# ---- box-constrained QP (projected Newton) ---------------------------------
def box_qp(H, g, lo, hi, maxiter=60):
    """min 1/2 z'Hz + g'z  s.t.  lo <= z <= hi. Returns (argmin, free-mask)."""
    x = np.clip(np.zeros(len(g)), lo, hi)
    free = np.ones(len(x), bool)
    for _ in range(maxiter):
        grad = g + H @ x
        clamped = ((x <= lo) & (grad > 0)) | ((x >= hi) & (grad < 0))
        free = ~clamped
        if not free.any():
            break
        Hf = H[np.ix_(free, free)]
        gf = (g + H @ (x * clamped))[free]        # hold clamped coords fixed
        try:
            Lc = np.linalg.cholesky(Hf + 1e-9*np.eye(free.sum()))
        except np.linalg.LinAlgError:
            break
        xf = np.linalg.solve(Lc.T, np.linalg.solve(Lc, -gf))
        step = np.zeros(len(x)); step[free] = xf - x[free]
        if np.max(np.abs(step)) < 1e-10:
            break
        a, c0 = 1.0, 0.5*x@H@x + g@x               # projected backtracking
        for _ in range(20):
            xn = np.clip(x + a*step, lo, hi)
            if 0.5*xn@H@xn + g@xn <= c0:
                x = xn; break
            a *= 0.5
        else:
            x = np.clip(x + a*step, lo, hi)
    return x, free


# ---- passes & driver --------------------------------------------------------
def rollout(x0, us, f, project, dt):
    xs = [np.asarray(x0, float)]
    for k in range(len(us)):
        xs.append(project(rk4_step(xs[k], us[k], f, dt)))
    return xs


def traj_cost(xs, us, stage_cost, term_cost):
    J = sum(stage_cost(xs[k], us[k], k)[0] for k in range(len(us)))
    return J + term_cost(xs[-1])[0]


def backward_pass(xs, us, f, jac, stage_cost, term_cost, lo, hi, dt, mu):
    """One backward sweep. Returns the feedforward steps kff and feedback gains Kg."""
    N = len(us); nx = xs[0].size; nu = us[0].size
    _, Vx, Vxx = term_cost(xs[N])
    kff = [None]*N; Kg = [None]*N
    for k in range(N-1, -1, -1):
        A, B = sensitivity(xs[k], us[k], f, jac, dt)
        _, lx, lxx, lu, luu = stage_cost(xs[k], us[k], k)
        Qx  = lx + A.T @ Vx
        Qu  = lu + B.T @ Vx
        Qxx = lxx + A.T @ Vxx @ A
        Quu = luu + B.T @ Vxx @ B + mu*np.eye(nu)
        Qux = B.T @ Vxx @ A
        k_i, free = box_qp(Quu, Qu, lo - us[k], hi - us[k])
        K_i = np.zeros((nu, nx))
        if free.any():
            K_i[free, :] = -np.linalg.solve(Quu[np.ix_(free, free)], Qux[free, :])
        kff[k], Kg[k] = k_i, K_i
        Vx  = Qx + K_i.T @ Quu @ k_i + K_i.T @ Qu + Qux.T @ k_i
        Vxx = Qxx + K_i.T @ Quu @ K_i + K_i.T @ Qux + Qux.T @ K_i
        Vxx = 0.5*(Vxx + Vxx.T)
    return kff, Kg


def forward_pass(xs, us, kff, Kg, f, project, stage_cost, term_cost, lo, hi, dt, J0):
    for a in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03, 0.015, 0.007, 0.0):
        xn = [xs[0]]; un = []
        for k in range(len(us)):
            du = a*kff[k] + Kg[k] @ (xn[k] - xs[k])
            uk = np.clip(us[k] + du, lo, hi); un.append(uk)
            xn.append(project(rk4_step(xn[k], uk, f, dt)))
        Jn = traj_cost(xn, un, stage_cost, term_cost)
        if Jn < J0:
            return xn, un, Jn, a
    return xs, us, J0, 0.0


def ilqr(x0, f, jac, project, stage_cost, term_cost, lo, hi, dt, us_init, iters=60, tol=1e-6):
    """Full solve from a warm start ``us_init`` (a list of N command guesses).
    Returns (xs, us, hist). Use tol=0.0 to run a fixed number of iterations
    (as the C++ does), or tol>0 to stop on a relative-cost plateau."""
    us = [np.array(u, float) for u in us_init]
    xs = rollout(x0, us, f, project, dt)
    J = traj_cost(xs, us, stage_cost, term_cost)
    mu = 1e-3; hist = [J]
    for _ in range(iters):
        kff, Kg = backward_pass(xs, us, f, jac, stage_cost, term_cost, lo, hi, dt, mu)
        xs2, us2, J2, a = forward_pass(xs, us, kff, Kg, f, project, stage_cost, term_cost, lo, hi, dt, J)
        if a > 0:
            rel = (J - J2) / max(J, 1e-9)
            xs, us, J = xs2, us2, J2
            mu = max(mu*0.7, 1e-6); hist.append(J)
            if rel < tol:
                break
        else:
            mu *= 4.0; hist.append(J)
            if mu > 1e3:
                break
    return xs, us, hist


def ilqr_solve(x0, f, jac, project, stage_cost, term_cost, lo, hi, dt, max_iters, warm):
    """Receding-horizon wrapper: one fixed-iteration solve, returns the first
    command and shifts ``warm`` (an (N, nu) array) in place for the next tick.
    Mirrors the C++ ``CDS::control::solve`` entry point."""
    us_init = [warm[k].copy() for k in range(len(warm))]
    xs, us, _ = ilqr(x0, f, jac, project, stage_cost, term_cost, lo, hi, dt, us_init,
                     iters=max_iters, tol=0.0)
    u0 = np.clip(us[0], lo, hi)
    for k in range(len(warm) - 1):
        warm[k] = us[k+1]
    warm[-1] = us[-1]
    J = traj_cost(xs, us, stage_cost, term_cost)
    return u0, xs, us, J
