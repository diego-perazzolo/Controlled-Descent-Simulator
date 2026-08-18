# Physics & Control Reference

The full state-space, forces and controller details for every vehicle model.

All models share the same core machinery: a 6-DOF rigid body, an RK4 integrator
(`libs/integrate`), and a controller that runs each tick before the physics step.
The dynamics are **generated as C++** from the Jupyter notebooks (`modeling/`).
The controllers are **hand-written C++** (`libs/control`) validated against a
Python reference: the LQR gain is synthesised at model init from generated
error-dynamics constants (a matrix-sign Riccati solve), and the MPC solver
re-optimizes each tick. How the code is generated is documented in
[build.md](build.md); this page is about *what* the models are.

---

## Degrees of freedom

6-DOF rigid body for every model — 3 translational + 3 rotational:

- **Translational**: X, Y, Z position and velocity in the inertial frame.
- **Rotational**: the Rocket uses Euler angles; both QuadRotor variants use a
  unit quaternion (no gimbal singularities, cheaper to integrate).

---

## State vectors

**Rocket** — 12 physical states + 4 tracking-error integrators:

```
x, y, z                       — position (m)
alpha, beta, psi              — Euler angles (rad)
x_dot, y_dot, z_dot           — linear velocity (m/s)
alpha_dot, beta_dot, psi_dot  — angular rates (rad/s)
IntX, IntY, IntZ, IntPsi      — tracking-error integrators
```

**QuadRotor (FF + LQR)** — 13 physical states + 4 tracking-error integrators:

```
x, y, z                   — position (m)
qw, qx, qy, qz            — attitude quaternion
vx, vy, vz                — linear velocity (m/s)
wx, wy, wz                — body angular rates (rad/s)
IntX, IntY, IntZ, IntPsi  — tracking-error integrators
```

**QuadRotor (MPC)** — the same 13 physical states **without** the 4 integrators:
the receding-horizon MPC needs no integral action, so the augmented state is
dropped. This model has non-zero steady state error in presence of external disturbances.

---

## Forces and torques

- **Actuation**
  - *Rocket*: main thrust + 3 control torques.
  - *QuadRotor*: 4 per-rotor thrusts, mapped to collective thrust + 3 torques
    through the QuadX allocation.
- **Gravity**: `F_g = m·g` along −Z.
- **Aerodynamic drag**: parametric, with a lateral coefficient `c` and an axial
  coefficient `cz`.
- **External perturbations**: a user-injected force vector `(fX, fY, fZ)`, pushed
  live from the frontend force buttons.

---

## Integration

Runge-Kutta 4 (RK4). The step `dt` is the
**measured** wall-clock time elapsed on the backend tick thread (wall-anchored
simulation time, clamped after stalls), not the nominal tick period — so a busy
or slow machine slows the simulation down rather than desyncing it.

---

## Controllers

### LQR + feedforward (Rocket, QuadRotor)

Acts on the tracking error (position + yaw, with the error integrators),
plus a feedforward term on all actuators (differential flatness for the
QuadRotor). Actuator saturation is applied on the final command.

The feedback gain is **synthesised at run time** by the hand-written
`CDS::control::lqr` (continuous-time LQR via the matrix-sign function,
`libs/control/lqr.hpp`) from the model's *frozen* error dynamics `A_e, B_e` and
cost weights `Q, R` — constants the notebooks export alongside the dynamics (the
linearisation stays fixed at the nominal operating point). Because the gain is
recomputed from `Q, R`, the weights are **runtime-tunable**: `SetWeights`
re-synthesises the gain while the physical parameters and the linearisation are
untouched. A construction-time *bridge* check certifies the runtime gain
reproduces the notebook's baked reference gain (`libs/control/lqr_tuner.hpp`).

### Nonlinear MPC (QuadRotor)

A **control-limited MPC**: an iLQR/DDP solver re-optimizes the 4 motor thrusts
over a receding horizon each control step, with the actuator box enforced
*inside* the optimization (not clipped afterwards).

Every hand-written controller ships a **C++↔Python conformance certificate**: a
stdlib-only script certifies the C++ result against an independent Python oracle
on a synthetic benchmark — for iLQR, a ~0 box-projected KKT residual (the returned
command sequence is a constrained optimum); for LQR, a ~0 Lyapunov-stationarity
residual (the returned gain is the optimum). The verification commands are in
[AGENTS.md](../AGENTS.md#verification-commands).

The MPC only re-solves at its control cadence and holds the last command in
between, which is why its per-tick cost is *bimodal* — see
[benchmark.md](benchmark.md).

---

## Screenshots

| Charts view | 3D view | Physics params | Trajectory params |
|:-----------:|:-------:|:--------------:|:-----------------:|
| ![Charts](screenshot-charts.png) | ![3D](screenshot-3d.png) | ![Params](screenshot-params-physics.png) | ![Params](screenshot-params-trajectory.png) |
