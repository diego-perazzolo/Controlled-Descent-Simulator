# Build & Run

The core is a static library (`cds_core`); what you build is an **app** under
`apps/`. The frontend and the embind API are identical for every app — the
choice is made by pointing CMake at the app you want:

- **`apps/wasm-only`** — the whole simulator (core included) compiled into a
  single WASM module. Runs entirely in the browser, no server.
- **`apps/ws-served`** — the core runs natively inside `cds_server` and is
  served over WebSocket; the browser loads a thin WASM proxy (~22 KB) that
  forwards every `ext_*` call as a synchronous binary RPC.

```
wasm-only:  frontend → embind → ext_comm.cpp → core          (all in browser)
ws-served:  frontend → embind → ext_comm_ws.cpp → WebSocket
                                → cds_server (ws_server → ext_comm.cpp → core)
```

## Prerequisites

| Tool | Version | Notes |
|------|---------|-------|
| [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html) | 3.1.56+ | Provides `emcmake` / `emcc` |
| CMake | 3.15+ | Build system |
| Python 3 | any | Local dev server |

If necessary configure the Emscripten environment with:

```bash
source pathToEmSDK/emsdk_env.sh
```

> Every CMake build directory lives under `build/` (e.g. `build/wasm-only`,
> `build/ws-client`, `build/server`, `build/mpc-model-test`) —
> one folder, all gitignored. Each browser app keeps its **own** build dir
> (separate CMake caches, no conflicts), but the compiled module always lands in
> `build/` top-level (`build/simulator.js`) — the delivery point imported by the
> frontend, coexisting with the build dirs. Whichever browser app was built last
> owns that delivery: switching app is just one `cmake --build`, no reconfigure,
> no deleting.

## wasm-only (full browser)

```bash
emcmake cmake -S apps/wasm-only -B build/wasm-only -DCMAKE_BUILD_TYPE=Debug   # or Release
cmake --build build/wasm-only
```

Then serve the repo root and open the frontend:

```bash
python3 tools/serve.py 8080
```

`http://localhost:8080/frontend/` in the browser.

## ws-served (native core server + thin client)

**1. Build the WASM proxy** (lands in `build/` like wasm-only — the last app
built is the one the frontend runs):

```bash
emcmake cmake -S apps/ws-served/client -B build/ws-client -DCMAKE_BUILD_TYPE=Debug
cmake --build build/ws-client
```

**2. Build and run the native core server** (no emsdk needed — plain CMake):

```bash
cmake -S apps/ws-served/server -B build/server -DCMAKE_BUILD_TYPE=Release
cmake --build build/server
./build/server/cds_server          # listens on ws://0.0.0.0:9002 (port as argv[1])
```

The websocket server optionally attaches a plant, selected by a second argument —
`loopback` or `sitl` — whilst the first argument is the communication port. With
**no** second argument no plant is attached: the server runs a pure, deterministic
simulation (fixed-step tick — a plant would make the tick wall-clock real-time):

```bash
./build/server/cds_server 9002            # no plant: pure deterministic simulation
./build/server/cds_server 9002 loopback   # the echo test double
./build/server/cds_server 9002 sitl       # ArduPilot SITL over MAVLink/UDP
```

**3. Serve the frontend with COOP/COEP headers** (required: the proxy uses
`SharedArrayBuffer` to make the async WebSocket look synchronous to embind):

```bash
python3 tools/serve.py 8080
```

The server URL defaults to `ws://localhost:9002` and can be overridden with
the `?ws=` query parameter, e.g. `http://localhost:8080/frontend/?ws=ws://192.168.1.10:9002`.
A quick end-to-end check is available at `http://localhost:8080/apps/ws-served/test/test_ws_e2e.html`.

To go back to the fully in-browser app: `cmake --build build/wasm-only`.

## Running against ArduPilot SITL

The `sitl` plant speaks MAVLink 2 over UDP to an ArduPilot **Copter** SITL
(use the QuadRotor model in the frontend). Start the server with the plant
selected — it listens, GCS-style, on `0.0.0.0:14550` and learns the vehicle
from the first valid datagram:

```bash
./build/server/cds_server 9002 sitl
```

Point the SITL's MAVLink output at that port. With the ArduPilot dev tools
running natively:

```bash
sim_vehicle.py -v ArduCopter -f quad --out=udp:127.0.0.1:14550
```

Once telemetry flows the plant ghost appears in the 3D view; use the **Plant
bar** to stage the vehicle (auto `GUIDED → arm → takeoff → climb`) and then
run the mission. For the full walkthrough against a SITL in **Docker** —
wiring, stream rates, a co-connected QGroundControl and the staging workflow —
see [`sitl.md`](sitl.md).

> The MAVLink C headers under `plants/sitl/mavlink/` are vendored and pinned;
> see [`plants/sitl/mavlink/VENDORED.md`](../plants/sitl/mavlink/VENDORED.md).
> `plants/sitl/mavlink_pin.hpp` fails the build if a re-vendor drifts the wire
> contract of the messages the plant uses.

## Run Jupyter notebooks in VS Code

Create and activate a python virtual environment.
Navigate with the Terminal to a folder where you want to store the virtual environment - venv -, then:

```bash
# Create virtual environment
python3 -m venv venv

# Activate environment (macOS/zsh)
source venv/bin/activate
```

Register kernel name:

```bash
python -m ipykernel install --user --name=rocket-modeling --display-name="Python (rocket-modeling)"
```

Now it is possible to open the notebook in VS Code, select the previously created python kernel and run the notebook

## Native tests

Standalone native tests (no browser, no emsdk). From the repo root:

```bash
# generic iLQR solver — self-contained (double integrator, no core/quaternions)
cmake -S libs/control/test -B build/ilqr-test -DCMAKE_BUILD_TYPE=Release && cmake --build build/ilqr-test
./build/ilqr-test/ilqr_test

# QuadRotor MPC — the model + solver end to end (Poly4 tracking + gust rejection)
cmake -S core/Models/test -B build/mpc-model-test -DCMAKE_BUILD_TYPE=Release && cmake --build build/mpc-model-test
./build/mpc-model-test/mpc_model_test

# solver C++<->Python conformance (ctypes, no numpy): certifies the C++ result is a
# constrained optimum (box-projected KKT residual ~0) on a synthetic benchmark
cmake -S libs/control/bind -B build/ilqr-bind -DCMAKE_BUILD_TYPE=Release && cmake --build build/ilqr-bind
python3 libs/control/bind/ilqr_conformance.py build/ilqr-bind
```

## Diagnostics: logging, profiling, recording

Three header-only diagnostics facilities (`libs/log`, `libs/profile`,
`libs/record`) ship in the build, all controllable at runtime from the frontend
**Diag** view and the persistent log dock at the bottom of every view:

- **Logger** (`libs/log`) — deferred-format logging with a per-module runtime
  level and "1-in-N" sampling; the stream shows in the collapsible bottom dock
  (on every view) and can be mirrored to a uniquely-named file (server-side).
- **Profiler** (`libs/profile`) — opt-in per-module scope timing with
  mean/percentiles (p50/p95/p99) and a live mean+p95 sparkline per scope;
  optional raw-sample CSV.
- **Recorder** (`libs/record`) — a per-tick "black box" wide CSV of the active
  model and plant (state / input / reference / tracking error), toggled from
  Diag (server-side); every file gets a unique, timestamped name.

Left at their defaults (log level Warn, profiler off, recording off) the runtime
cost is negligible — a per-call-site level/enabled check of ~1–5 ns, and the log
arguments are not even evaluated when the level filters them out.

### Compiling the diagnostics out

For a clean release/measurement build, strip the call sites entirely at compile
time with three macros. Pass them to any build via `CMAKE_CXX_FLAGS` (they
propagate to core, plants and the app):

| macro | effect |
|-------|--------|
| `CDS_LOG_COMPILE_LEVEL=N` | keep only logs at level ≥ N (Trace 0, Debug 1, Info 2, Warn 3, Error 4; 5 = none) |
| `CDS_PROFILE_ENABLED=0` | strip every `CDS_PROFILE` scope |
| `CDS_RECORD_ENABLED=0` | strip every `CDS_RECORD` row |

```bash
# example: keep only Warn/Error logs, remove profiler and recorder entirely
cmake -S apps/ws-served/server -B build/server -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-DCDS_LOG_COMPILE_LEVEL=3 -DCDS_PROFILE_ENABLED=0 -DCDS_RECORD_ENABLED=0"
```

The recorder CSV paths default under `out_data/` and can be overridden with the
`CDS_LOG_FILE`, `CDS_PROFILE_RAW_FILE`, `CDS_RECORD_FILE` and
`CDS_RECORD_PLANT_FILE` environment variables.

### Benchmarks

`bench/` holds the speed benchmarks: `perf_bench` sizes the diagnostics per-call
overhead (ON vs OFF, plus the drain and the compile-out residual via
`perf_bench_off`), and `model_bench` times one physics tick per model. Not a CI
gate — the numbers are timing-dependent; the CI `benchmark` job runs them and
uploads a report artifact. See [benchmark.md](benchmark.md) for the results and
discussion.

```bash
cmake -S bench -B build/bench -DCMAKE_BUILD_TYPE=Release && cmake --build build/bench
./build/bench/perf_bench        # diagnostics per-call cost (features in)
./build/bench/perf_bench_off    # same, features compiled out (~0)
./build/bench/model_bench       # per-model integration cost
python3 bench/report.py build/bench > benchmark-report.md   # full report (gitignored)
```

## GitHub Pages

The repository includes a GitHub Actions workflow ([`.github/workflows/deploy.yml`](../.github/workflows/deploy.yml)) that automatically builds the WASM in Release mode and deploys to GitHub Pages on every push to `main`.

**Cross-origin isolation.** GitHub Pages serves static files and cannot send the
`Cross-Origin-Opener-Policy` / `Cross-Origin-Embedder-Policy` headers, so the
page is not cross-origin isolated and browsers clamp `performance.now()` to
100 µs. Since the tick `dt` is wall-anchored on that clock, the added jitter
destabilises the feed-forward controllers (the quadrotor FF-LQR visibly
oscillates). [`frontend/coi-serviceworker.js`](../frontend/coi-serviceworker.js)
(vendored, MIT) re-injects those headers from a service worker to restore the
high-resolution clock; the first visit performs one transparent reload. It is a
no-op locally, where [`tools/serve.py`](../tools/serve.py) already sends the
headers.

## Repository Structure

The state vectors, forces and controllers of the models under `core/Models/`
are described in [`models.md`](models.md).

```
/
├── core/                                       # The physics core, built as a static library (cds_core)
│   ├── CMakeLists.txt
│   ├── core.hpp / core.cpp                     # C-style public interface (stubs → impl)
│   ├── core_defs.hpp                           # Internal type definitions
│   ├── System/
│   │   └── SystemManager.hpp / .cpp            # System owner: model, trajectory, lock boundary
│   ├── Plant/
│   │   └── BasePlant.hpp / BasePlant.cpp       # Base plant class + mailbox exchange (impls live outside core)
│   ├── Models/
│   │   ├── BaseModel.hpp / BaseModel.cpp       # Abstract model base
│   │   ├── Rocket.hpp / Rocket.cpp             # 6 DOF rocket model (Euler angles)
│   │   ├── QuadRotor.hpp / QuadRotor.cpp       # 6 DOF quadrotor model (quaternion, FF + LQR)
│   │   ├── QuadRotorMPC.hpp / .cpp             # Quadrotor driven by the nonlinear MPC (libs/control solver)
│   │   └── test/mpc_model_test.cpp             # Native model + solver acid test (tracking + gust)
│   └── Trajectory/
│       ├── TrajectoryManager.hpp / .cpp        # trajectory composition
│       ├── Trajectory.hpp / Trajectory.cpp     # base trajectory class
│       ├── Point.hpp / Point.cpp               # waypoint-like
│       └── Poly4.hpp / Poly4.cpp               # 4th order polynomial trajectory
│
├── plants/                                     # Concrete plant implementations (plants → core, one way)
│   ├── CMakeLists.txt                          # cds_plants static library
│   ├── loopback/
│   │   └── LoopbackPlant.hpp / .cpp            # SITL loopback: echoes the reference with period/latency/dropouts
│   ├── sitl/                                   # ArduPilot SITL plant (MAVLink 2 / UDP)
│   │   ├── SitlPlant.hpp / .cpp               # Link session, telemetry decode, Guided setpoints, frame alignment
│   │   ├── UdpTransport.hpp / .cpp            # Minimal UDP endpoint (transport seam; serial link is its sibling)
│   │   ├── mavlink_pin.hpp                    # Sole MAVLink entry point + wire-contract static_asserts (version pin)
│   │   └── mavlink/                           # VENDORED MAVLink C headers — never hand-edit (see VENDORED.md)
│   └── test/
│       ├── CMakeLists.txt                      # Native integration test project
│       ├── driver.cpp                          # Plant machinery test (standalone + SystemManager)
│       └── sitl_driver.cpp                     # SITL plant test vs in-process fake ArduCopter over UDP
│
├── apps/                                       # Deployments of the core; each app has its own CMakeLists
│   ├── common/                                 # Shared by all apps: the ext API "factory"
│   │   ├── ext_api.py                          # Declarative description of the ext API (source of truth)
│   │   ├── gen_ext.py                          # Generator: ext_api.py → the exported_cpp/ folders
│   │   ├── gen_ext.cmake                       # Hooks the generator into the app builds (auto re-run)
│   │   ├── ext_comm.cpp                        # Direct adapter: ext → core function calls (hand-written)
│   │   └── exported_cpp/                       # GENERATED — never hand-edit
│   │       ├── ext_defs.hpp                    # External struct definitions (the ext API boundary)
│   │       ├── ext_comm.hpp                    # ext API contract (implemented by each adapter)
│   │       └── bindings.cpp                    # Emscripten embind bindings
│   ├── wasm-only/
│   │   └── CMakeLists.txt                      # Full in-browser app: core + direct adapter in one WASM
│   └── ws-served/                              # Core on a native server, browser as thin client
│       ├── exported_cpp/                       # GENERATED — never hand-edit
│       │   ├── ws_protocol.hpp                 # Binary wire protocol (client ↔ server)
│       │   ├── ext_comm_ws.cpp                 # WebSocket adapter: ext ↔ ws_protocol marshalling
│       │   └── dispatch.cpp                    # ws_protocol requests → ext communication layer
│       ├── client/
│       │   └── CMakeLists.txt
│       ├── server/
│       │   ├── CMakeLists.txt
│       │   ├── dispatch.hpp
│       │   └── main.cpp                        # cds_server entry point
│       └── test/
│           ├── test_protocol.py                # e2e protocol test against a real cds_server (runs in CI)
│           └── test_ws_e2e.html                # Manual in-browser end-to-end check of the ws-served app
│
├── libs/                                       # In-house infrastructure libraries (apps/core → libs, one way)
│   ├── sync/
│   │   └── TripleBuffer.hpp                    # Wait-free SPSC latest-wins mailbox
│   ├── integrate/
│   │   └── rk4.hpp                             # Generic fixed-control RK4 step (header-only, domain-agnostic)
│   ├── control/                                # Hand-written model-agnostic controllers (AGENTS.md rule 10)
│   │   ├── ilqr.hpp                            # Generic control-limited iLQR/DDP solver (header-only)
│   │   ├── test/ilqr_test.cpp                  # Self-contained solver acid test (double integrator)
│   │   └── bind/                               # C-ABI shim + ilqr_conformance.py (C++↔Python KKT certificate)
│   ├── param/                                  # Generic tunable-parameter registry (domain-agnostic)
│   │   └── param_table.hpp                     # ParamTable: TSV manifest + set-by-id, shared by model/controller/observer/sensor
│   ├── estimate/                               # Hand-written model-agnostic estimators (AGENTS.md rule 10)
│   │   ├── observer.hpp                        # Generic dual-LQR observer (header-only)
│   │   ├── trans_disturbance_observer.hpp      # Reusable offset-free translational disturbance observer
│   │   ├── observer_params.hpp                 # Observer knobs → ParamTable (libs/param)
│   │   ├── test/                               # Self-contained observer + TDO acid tests
│   │   └── bind/                               # C-ABI shim + observer_conformance.py (C++↔Python certificate)
│   ├── sensor/                                 # Measurement corruptor (per-channel noise/bias/enable)
│   │   ├── sensor_model.hpp                    # Per-channel SensorModel (Gaussian noise + bias + dropout)
│   │   ├── sensor_params.hpp                   # Sensor knobs → ParamTable (libs/param) + measuredThrough
│   │   └── test/sensor_model_test.cpp          # Self-contained sensor acid test
│   ├── log/                                    # Deferred-format logger (opt-in, runtime-levelled)
│   │   ├── log.hpp / LogRing.hpp               # Registry + CDS_LOG_* macros; wait-free MPSC ring
│   │   ├── LogSinks.hpp / LogUiSink.hpp        # Console/File sinks; recent-lines UI buffer
│   │   ├── UniqueFile.hpp                      # Timestamped unique output paths (shared by all sinks)
│   │   └── test/log_test.cpp                   # Self-contained logger acid test
│   ├── profile/                                # Opt-in scope profiler (aggregates + percentiles)
│   │   ├── profile.hpp / P2Quantile.hpp        # Registry + CDS_PROFILE; P² streaming quantiles
│   │   ├── ProfileReport.hpp                   # Text/CSV dump of a snapshot
│   │   └── test/profile_test.cpp               # Self-contained profiler acid test
│   ├── record/                                 # Per-tick "black box" data recorder (wide CSV)
│   │   ├── Recorder.hpp                        # Templated Row<T,N>, model/plant slots, drop counter
│   │   └── test/record_test.cpp               # Self-contained recorder acid test
│   └── ws/                                     # Dependency-free WebSocket RPC transport
│       ├── CMakeLists.txt                      # cds_ws_client (emscripten) / cds_ws_server (native)
│       ├── ws_server.hpp / ws_server.cpp       # Minimal RFC 6455 WebSocket RPC server
│       ├── ws_rpc_client.hpp / .cpp            # Synchronous WASM RPC transport (EM_JS + SharedArrayBuffer)
│       └── ws_rpc_client_pre.js                # pre-js: spawns the WebSocket bridge worker
│
├── bench/                                      # Speed benchmarks (not a CI gate; report is an artifact)
│   ├── perf_bench.cpp                          # ns/op of logger/profiler/recorder ON vs OFF + drain
│   ├── model_bench.cpp                         # per-model integration cost (mean/p50/p95/max) — links core
│   ├── report.py                               # assembles the Markdown report artifact from the binaries
│   └── CMakeLists.txt                          # perf_bench / perf_bench_off / model_bench
│
├── tools/
│   └── serve.py                                # Dev server with COOP/COEP headers
│
├── modeling/
│   ├── requirements.txt                        # Python requirements
│   └── notebooks/
│       ├── base_codegen.py                     # Shared C++ code-generation base (used by all model codegens)
│       ├── model/                              # Per-vehicle: dynamics detail -> codegen -> integration demos
│       │   ├── dynamics_rocket_FFLQR01.ipynb   # Rocket dynamics with LQR + FF
│       │   ├── dynamics_quadRotor_FFLQR01.ipynb# QuadRotor dynamics with LQR + flatness FF
│       │   ├── dynamics_quadRotor_MPC01.ipynb  # QuadRotor prediction model for the nonlinear MPC
│       │   └── rocket_codegen.py / quad_codegen.py / mpc_codegen.py   # per-model codegen (derive from base)
│       ├── control/                            # Vehicle-agnostic controller development (mirrors libs/control)
│       │   ├── ilqr.ipynb                      # Generic iLQR derivation + C++ conformance test
│       │   └── ilqr_ref.py                     # Single Python source of the iLQR reference
│       └── exported_cpp/                       # GENERATED — never hand-edit (stays at the notebooks root)
│           ├── ROCKET_FF_LQR_01/               # dynamics_rocket_ff_lqr_01.cpp / .hpp
│           ├── QUADROTOR_FF_LQR_01/            # dynamics_quadrotor_ff_lqr_01.cpp / .hpp
│           └── QUADROTOR_MPC_01/               # dynamics_quadrotor_mpc_01.cpp / .hpp (prediction model only)
│
├── frontend/
│   ├── index.html
│   ├── coi-serviceworker.js                    # Vendored (MIT): restores cross-origin isolation on GitHub Pages
│   └── main.js                                 # Simulation loop, renderers, UI logic
│
├── docs/                                       # Documentation and media
│   ├── build.md                                # This file
│   └── api.md                                  # Core API reference
│
├── .github/
│   └── workflows/
│       ├── ci.yml                              # CI: generator sync, builds, protocol e2e test
│       └── deploy.yml                          # GitHub Pages deployment
│
└── Readme.md
```
