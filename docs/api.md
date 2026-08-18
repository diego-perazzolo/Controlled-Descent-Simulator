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

The vehicle models this API initializes and drives — their state vectors,
forces and controllers — are described in [`models.md`](models.md).

Error convention: functions returning `bool` return `true` on error.

```cpp
/* Initialize the Rocket model (FF_LQR_01), returns true on error */
bool ext_initRocket_FFLQR01(ext_initRocketParams params);       /* JS: ext_rocketInit */

/* Initialize the QuadRotor model (FF_LQR_01), returns true on error */
bool ext_initQuadRotor_FFLQR01(ext_initQuadRotorParams params); /* JS: ext_quadRotorInit */

/* Initialize the QuadRotor model driven by a nonlinear MPC (MPC_01); reuses the
   QuadRotor params/limits (rotor inertia is unused), returns true on error */
bool ext_initQuadRotor_MPC01(ext_initQuadRotorParams params);   /* JS: ext_quadRotorMpcInit */

/* Set system parameters (tick period, user forces), returns true on error */
bool ext_setSystemParams(ext_systemParams params);

/* Get a snapshot of the simulation: elapsed time, state, tracking errors */
ext_snapshotData ext_getSnapshot(void);

/* Get the plant's last sample: plant-side time, sequence number, state and
   readiness (isReadyToStart: staged, or no staging needed). isAttached
   distinguishes "no plant" from "plant attached, no sample yet" (both report
   isError) */
ext_plantSnapshotData ext_getPlantSnapshot(void);

/* Start the simulation / plant ticking, returns true on error */
bool ext_run(void);

/* Stop the simulation / plant ticking, returns true on error */
bool ext_stop(void);

/* Auto-stage the plant to a hover at (trajectory vertical range +
   safetyAltitude, m) via GUIDED -> arm -> takeoff -> climb; ready when staged.
   Returns true on error (no plant, no trajectory, or already running) */
bool ext_beginStaging(ext_coord_t safetyAltitude);

/* Abort auto-staging (hold in place), returns true on error */
bool ext_stopStaging(void);

/* Get a point at time instant t along the trajectory */
ext_trajectoryPoint ext_trajectory_get_point(ext_coord_t t);

/* Add a trajectory Polynomial 4th order, returns true on error */
bool ext_trajectory_append_poly4(ext_trajectoryPoly4Params_t params);

/* Add a trajectory Point, returns true on error */
bool ext_trajectory_append_point(ext_trajectoryPointParams_t params);

/* Remove last trajectory item, returns true on error */
bool ext_trajectory_remove_last_item(void);

/* --- logger / profiler inspection (libs/log, libs/profile) --- */

/* Drain a batch of recent log lines from the UI buffer. `lines` packs `count`
   newline-separated 'timestamp\tLEVEL\tmodule\ttext' records (timestamp is local
   wall-clock "YYYY-MM-DD HH:MM:SS.uuuuuu", microsecond precision); `dropped`
   counts lines lost to UI-buffer overflow since the last call. Parse on the JS
   side */
ext_logBatch ext_getLogBatch(void);

/* List modules, one record per newline. getLogModules:
   'index\tname\tlevel\tsampleN'; getProfileModules: 'index\tname\tenabled' */
ext_moduleList ext_getLogModules(void);
ext_moduleList ext_getProfileModules(void);

/* Set a log module's runtime level (0=Trace..4=Error, 5=Off) and its sampling
   divisor N (1 = emit all; N>1 = 1 in N per _SAMPLED call site). Returns true
   on error (module index out of range) */
bool ext_setLogLevel(ext_logLevelParams params);

/* Enable or disable profiling for a module, returns true on error */
bool ext_setProfileEnabled(ext_profileEnableParams params);

/* Get the profiler stats table from the latest published snapshot (only scopes
   of enabled modules), one record per newline:
   'module\tscope\tkind\tcount\tmean\tstd\tmin\tmax\tp50\tp95\tp99'. kind is
   'us' (a timed scope, values in microseconds) or 'val' (a value scope, raw) */
ext_profileTable ext_getProfileTable(void);

/* Reset all profiler statistics (clears cold-start outliers), returns true on
   error */
bool ext_resetProfile(void);

/* Toggle the server-side diagnostics files (no-op in the wasm build, which has
   no real filesystem): logFile mirrors the log to a file (cds.log), profileRaw
   streams every raw profiler sample to a CSV (cds_profile_raw.csv) for offline
   analysis. Every file gets a unique, timestamped name and a fresh file is
   opened each time serialization is toggled back on. Returns true on error */
bool ext_setDiagFiles(ext_diagFiles params);

/* Toggle the per-tick data recorders — lossless wide-CSV "black boxes" of the
   active model (state/input/reference/tracking-error) and, separately, the
   active plant (published measurements: time, sequence, state), one row each per
   tick/sample, for offline validation and comparison (server-side only). Enables
   both at once and returns the recorder status (a "model + plant" name summary,
   the enabled flag, and the combined dropped-row count) */
ext_recordStatus ext_setRecording(ext_recordParams params);

/* Get the data recorder status without changing it (poll the dropped-row count
   and the active model+plant names from the frontend) */
ext_recordStatus ext_getRecordStatus(void);

/* Get the active controller's parameter manifest: a self-describing TSV listing
   of its exposed parameters, one record per newline
   ('id\tgroup\tlabel\tflags\tvalue', flags = 'rw' | 'ro'). The frontend builds
   its tuning panel from this (no controller-specific UI). Empty text if no model
   is running or the controller exposes no parameters */
ext_controllerManifest ext_getControllerManifest(void);

/* Set one controller parameter, addressed by its manifest id, to a new value.
   Slow-path, one coefficient at a time (never on the tick). For the LQR models a
   set re-synthesizes the gain; for the MPC it retunes the cost/solver knobs.
   Returns true on error (no model, bad id, read-only or rejected value) */
bool ext_setControllerParam(ext_controllerParamSet params);
```

## Key types

Defined in `apps/common/exported_cpp/ext_defs.hpp`; fields are `ext_coord_t`
(float) unless noted, plus `bool` and fixed `char` buffers. Structs at this
boundary are POD — no STL containers. A `char[N]` field is how variable text
crosses the POD wire: it is bound to JS as a `string` (embind getter/setter)
and carries a newline/tab-delimited blob the frontend parses. Text-blob
responses are why `WS_MAX_MSG_SIZE` is 4096.

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
ext_systemParams               { timestep_seconds, user_forces (ext_userForce) }
ext_snapshotData               { time_seconds, state (ext_fullState),
                                err (ext_setpointError), isError (bool) }
ext_plantSnapshotData          { time_seconds, sequence, state (ext_fullState),
                                isAttached (bool), isReadyToStart (bool),
                                isError (bool) }
ext_logBatch                   { lines (char[3800]), count, dropped }
ext_moduleList                 { list (char[1200]), count }
ext_profileTable               { table (char[3600]), count }
ext_logLevelParams             { module, level, sampleN }
ext_profileEnableParams        { module, enabled (bool) }
ext_diagFiles                  { logFile (bool), profileRaw (bool) }
ext_recordParams               { enabled (bool) }
ext_recordStatus               { modelName (char[64]), active, enabled, droppedRows }
ext_controllerManifest         { text (char[2048]) }
ext_controllerParamSet         { id, value }
```

`ext_recordStatus.modelName` is a "model + plant" summary of the active
recorders. Its `active`/`enabled`/`droppedRows` are `ext_coord_t` (0.0/1.0 flags
and a summed count) rather than `bool`, because a wire struct may not mix a
`char` buffer with a `bool`.

## Protocol version

`WS_PROTOCOL_VERSION` (in the generated `ws_protocol.hpp`) is **not an
incremental number and is never maintained by hand**: it is an 8-bit
fingerprint of the API description, computed by the generator — a signature
answering "were both peers generated from the same `ext_api.py`?" rather
than "which version is this?". Change the description and the byte changes
on both sides automatically; a client and a server carrying different bytes
refuse to talk (explicit error + server log) instead of mis-parsing each
other.

## Adding a command

1. **Describe it in `apps/common/ext_api.py`**: declare any new structs, then
   add a `Cmd` entry. Mind the meaning of the naming fields:
   - `wire` (2nd arg) — PascalCase stem for the wire-level names only
     (`Run` → `reqRun_t`, `WS_MSG_RUN`);
   - `cfn` (3rd arg) — the C++ function name: this is the symbol you will
     implement in `ext_comm.cpp`;
   - `js` (4th arg) — the name the frontend calls (`sim.<js>(...)`).
2. **Regenerate**: run `python3 apps/common/gen_ext.py`, or simply build any
   app — CMake re-runs the generator when the description changes.
3. **Implement the adapter** in the hand-written `apps/common/ext_comm.cpp`:
   the function named `cfn`, converting the ext structs and calling the core.
   Until you do, `wasm-only` and the `ws-served` server fail to link with an
   undefined reference to `cfn` — that is the reminder, not a generator bug.
   (The `ws-served` client links regardless: its implementation of `cfn` is
   the generated marshalling.)

## JS-side usage

The frontend imports the module from `build/simulator.js` and calls the API
synchronously:

```js
import createSimulator from '../build/simulator.js';

const sim = await createSimulator();
sim.ext_rocketInit({ rocketPar: {...}, rocketActuatorLimits: {...} });
sim.ext_setSystemParams({ timestep_seconds: 0.01, user_forces: { fX: 0, fY: 0, fZ: 0 } });

// with a SITL plant attached: auto-stage before the mission, then poll
// readiness from the plant snapshot
sim.ext_beginStaging(5);   // range + 5 m safety margin
const { isReadyToStart } = sim.ext_getPlantSnapshot();

sim.ext_run();
const { isError, time_seconds, state, err } = sim.ext_getSnapshot();
sim.ext_stop();
```

In the ws-served app the same calls are transparently forwarded to
`cds_server`; the page must then be served with COOP/COEP headers
(`python3 tools/serve.py 8080`) — see [build.md](build.md).
