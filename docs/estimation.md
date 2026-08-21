# Estimation & Sensors

The optional **state estimator** and **sensor model** that sit between the plant
and the controller. Two complementary, opt-in modules — one *removes* disturbance,
the other *adds* it — both configurable live from the frontend.

```
 true state → [ SENSOR MODEL ] → measurement → [ OBSERVER ] → estimate → [ CONTROLLER ]
                +noise/bias/drop                 filters / estimates d̂
```

Both are **off by default**: with an identity sensor and the observer disabled,
every model behaves exactly as documented in [models.md](models.md). The generic
numerics live under `libs/` (domain-agnostic, header-only); each model supplies
only its physics. This mirrors the controller split (see
[models.md](models.md#controllers)): generic solver in `libs/`, model-specific
pieces in `core/Models`.

---

## The disturbance observer

A **translational** estimator (`libs/estimate/trans_disturbance_observer.hpp`)
carrying the state `[r(3), v(3), d(3)]`:

```
r_dot = v                     kinematics (exact)
v_dot = a_known + Bd·d        a_known: model's force-free acceleration
d_dot = 0                     constant-disturbance model — the integrating state
```

- `a_known` is read **from the generated model** each tick (`Dynamics(x, u, 0)` —
  the acceleration under thrust + gravity + drag with *no* external force, exactly
  the MPC's prediction assumption).
- `Bd = ∂v̇/∂F_ext` (how an external force enters the acceleration) is also read
  **from the generated model**, by finite-differencing `Dynamics` w.r.t. the
  external force at model init — so the physics derives from the notebook-exported
  C++, never from hand code.
- The measurement is the position `r`.

The steady-state gain `L` is synthesised as the **exact dual of the LQR** problem
(`L = lqr(A', C', Qw, Rv)'`), reusing the certified matrix-sign Riccati solver in
`libs/control/lqr.hpp` (`libs/estimate/observer.hpp`). It ships its own
C++↔Python conformance certificate, the dual of the LQR one.

### Two roles, by controller

The observer plays a **different role depending on whether the controller already
integrates**:

- **MPC models** (no integrators → offset in steady state). The observer's job is
  **offset-free tracking**: the disturbance estimate `d̂` is fed to the MPC as the
  predicted external force (`predForce = d̂`), so the controller anticipates the
  force and the steady-state tracking error goes to zero. `d̂` is the missing
  integral action, done in a principled (Kalman) way.

- **FF-LQR models** (already have 4 error integrators → already offset-free). The
  observer's job is **sensor robustness**: its filtered position/velocity estimate
  `x̂` feeds the LQR *feedback* in place of the true state, so a noisy or dropped
  sensor degrades gracefully. `d̂` is **not** fed forward here (the integrators
  reject constant disturbances themselves); it only keeps `x̂` accurate under an
  unmodeled force. Note the integral states live inside the generated dynamics and
  keep using the true state, so this is a **partial** integration — offset-free
  stays with the integrators.

A dropped sensor axis is handled as **predict-only**: the observer stops
correcting that axis and coasts on the model. This graceful coast is the
observer's distinctive value-add.

---

## The sensor model

A per-axis measurement corruptor (`libs/sensor/sensor_model.hpp`) emulating a
sensor suite. Each of the 3 position axes carries:

- **enable** — a disabled axis withholds its measurement (see below);
- **bias** — a constant additive offset;
- **noise std** — the standard deviation of additive zero-mean Gaussian noise.

The noise is **deterministic** (a seeded splitmix64 PRNG with Box-Muller), so runs
are reproducible across platforms. A fresh sensor is the identity (enabled, zero
bias, zero noise) — the feature is opt-in.

### Sensors bite with or without the observer

The sensor is **independent of the observer**:

- **Observer on** — the corrupted measurement feeds the observer, which *filters*
  it (position/velocity) and coasts on a dropped axis.
- **Observer off** — the controller reads the corrupted measurement **directly**
  (unfiltered), so noise and bias bite anyway. This exposes the *problem* a
  sensor causes; turning the observer on shows the *solution*.

A disabled axis **without** an observer passes the true value through (there is no
estimator to coast on, so it simply injects nothing — coasting is the observer's
value-add). With the observer on, a disabled axis is the predict-only coast.

---

## Tuning the observer

The observer is a Kalman-Bucy filter, and **it only helps if its noise model
matches reality**. A mistuned filter can be *worse* than no filter — this is the
one pitfall to know. The covariances are exposed as runtime knobs (group
`Observer`):

| knob | meaning | how to set it |
|------|---------|---------------|
| **measurement noise** | assumed sensor-noise variance (`Rv`) | set to ≈ the sensor's noise **variance** (σ²) |
| **process noise: disturbance** | how fast `d̂` may change | the responsiveness knob — higher tracks the disturbance faster (shorter transient) but is noisier |
| **process noise: velocity** | model uncertainty on acceleration | raise if the model's acceleration is unreliable |
| **process noise: position** | model uncertainty on `ṙ = v` | keep small (the kinematics are exact) |

Changing any covariance **re-synthesises the gain** (off the tick path).

**The mistuning pitfall.** The default *measurement noise* assumes a near-perfect
sensor. If you dial a large sensor *noise std* but leave *measurement noise* small,
the filter over-trusts the sensor: `x̂` chases the noise and the disturbance state
`d̂` wanders, injecting spurious forces — tracking gets *worse* than with the
observer off (the plant's own inertia low-passes raw noise more gently). The fix
is to raise *measurement noise* to match: set it to ≈ (noise std)². The
`offset_free_model_test` demonstrates this — at a 1 m sensor noise the error is
~0.62 m with the default and ~0.08 m once matched.

**The transient / steady-state trade-off.** Because `d̂` is an integrating state,
the observer trades transient performance for steady-state accuracy: under a step
disturbance the vehicle first drifts (while `d̂` ramps up) and `d̂` may overshoot,
so the *transient* error can briefly exceed the observer-off case — which just
settles quickly to a persistent offset. Raise *process noise: disturbance* to
speed `d̂` up and shorten the transient. (The Rocket MPC defaults to a larger value
than the QuadRotor because its heavy mass makes the disturbance weakly observed.)

---

## Configuration

Both modules ride the existing **controller-parameter manifest** — no new wire
protocol. The observer and sensor knobs appear as extra manifest groups
(`Observer`, `Sensor x/y/z`) alongside the controller tuning, and are set through
the same `SetControllerParam` command (see [api.md](api.md)). In the frontend they
render in the **Params** view under an *Estimator & sensors* section: the enable
flags as checkboxes, the rest as numeric fields.

---

## Verification

Native, self-contained tests (full command list in
[AGENTS.md](../AGENTS.md#verification-commands)):

```bash
./build-observer-test/observer_test    # observer synthesis (dual LQR): known-answer gains
./build-observer-test/tdo_test         # translational observer: d̂ recovery + dropped-axis coast
./build-sensor-test/sensor_model_test  # corruptor: passthrough + bias + disable + noise stats
./build-mpc-model-test/offset_free_model_test   # MPC: offset-free + sensor injection + tuning
./build-mpc-model-test/lqr_observer_model_test  # FF-LQR: clean baseline + noise filtering + coast
```

---

## Limitations & future work

- **Offset-free in prediction, not in target.** The MPC feeds `d̂` to the
  prediction but does not recompute the steady-state *target*. Holding position
  under a lateral force needs a steady tilt, which the upright attitude reference
  penalises, so a residual set by the controller's cost trade-off remains (tiny
  for the soft-attitude QuadRotor, larger for the stiff-attitude Rocket). Full
  offset-free would recompute the attitude target from `d̂`.
- **Fixed-rate, no accelerometer.** The gain is steady-state (rate-invariant) and
  measures position only. Variable-rate / multi-rate measurements and an
  accelerometer (which is an *input* driving the propagation, whose **bias** — not
  the acceleration itself — would be a new estimator state) need a time-varying
  Kalman / EKF with bias states. Velocity and angular-rate sensors, by contrast,
  fit the current direct-injection path.
