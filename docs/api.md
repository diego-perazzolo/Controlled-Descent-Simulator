# Core API

The external communication API (`apps/common/exported_cpp/ext_comm.hpp`) is
the boundary between the frontend and the core. It is exposed to JavaScript
one-to-one via the embind bindings (`exported_cpp/bindings.cpp`) and is
identical for every app: `wasm-only` implements it by calling straight into
the core (`ext_comm.cpp`), `ws-served` by marshalling each call over
WebSocket to `cds_server` (`exported_cpp/ext_comm_ws.cpp`).

The API is **described declaratively in `apps/common/ext_api.py`** — structs,
bindings, wire protocol, client marshalling and server dispatch are all
generated from it into the `exported_cpp/` folders by
`apps/common/gen_ext.py` (the app builds re-run it automatically when the
description changes). To add a command: add its structs and a `Cmd` entry
there, regenerate (or just build), then implement the adapter function in
the hand-written `apps/common/ext_comm.cpp`.

Error convention: functions returning `bool` return `true` on error.

```cpp
/* Initialize the Rocket model (FF_LQR_01), returns true on error */
bool ext_initRocket_FFLQR01(ext_initRocketParams params);       /* JS: ext_rocketInit */

/* Initialize the QuadRotor model (FF_LQR_01), returns true on error */
bool ext_initQuadRotor_FFLQR01(ext_initQuadRotorParams params); /* JS: ext_quadRotorInit */

/* Advance one integration step */
ext_stepRet ext_step(ext_stepParams params);

/* Get a point at time instant t along the trajectory */
ext_trajectoryPoint ext_trajectory_get_point(ext_coord_t t);

/* Add a trajectory Polynomial 4th order, returns true on error */
bool ext_trajectory_append_poly4(ext_trajectoryPoly4Params_t params);

/* Add a trajectory Point, returns true on error */
bool ext_trajectory_append_point(ext_trajectoryPointParams_t params);

/* Remove last trajectory item, returns true on error */
bool ext_trajectory_remove_last_item(void);
```

## Key types

Defined in `apps/common/exported_cpp/ext_defs.hpp`; all fields are
`ext_coord_t` (float). Structs at this boundary are POD — no STL containers.

```cpp
ext_rocketParams               { mass_Kg, inertiaX/Y/Z_Kgm2, c, cz }
ext_rocketActuatorLimits       { fZ_max/min, Tx_max/min, Ty_max/min }
ext_quadRotorParams            { mass_Kg, inertiaX/Y/Z_Kgm2, c, cz, motorThrustCoefficient,
                                motorTorqueCoefficient, distanceBtwMotorAndCoM, motorMomentOfInertia }
ext_quadRotorActuatorLimits    { motor_max_thrust, motor_min_thrust }
ext_trajectoryPoly4Params_t    { initialPos/Vel + initialYaw/YawRate, finalPos/Vel/Acc + finalYaw/YawRate/YawAcc, time_s }
ext_trajectoryPointParams_t    { finalPos, finalYaw, time_s }
ext_userForce                  { fX, fY, fZ }
ext_fullState                  { x, y, z, x_dot, y_dot, z_dot,
                                roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot }
ext_setpointError              { xErr, yErr, zErr, yawErr }
```

## JS-side usage

The frontend imports the module from `build/simulator.js` and calls the API
synchronously:

```js
import createSimulator from '../build/simulator.js';

const sim = await createSimulator();
sim.ext_rocketInit({ rocketPar: {...}, rocketActuatorLimits: {...} });
const { isError, state, err } = sim.ext_step({ timeStep_s: 0.01, userForce: { fX: 0, fY: 0, fZ: 0 } });
```

In the ws-served app the same calls are transparently forwarded to
`cds_server`; the page must then be served with COOP/COEP headers
(`python3 tools/serve.py 8080`) — see [build.md](build.md).
