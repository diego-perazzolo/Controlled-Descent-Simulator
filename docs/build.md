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

> Each browser app has its **own build directory** (separate CMake caches, no
> conflicts), but the compiled module always lands in `build/` — the delivery
> point imported by the frontend. Whichever app was built last owns `build/`:
> switching app is just one `cmake --build`, no reconfigure, no deleting.

## wasm-only (full browser)

```bash
emcmake cmake -S apps/wasm-only -B build-wasm-only -DCMAKE_BUILD_TYPE=Debug   # or Release
cmake --build build-wasm-only
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
emcmake cmake -S apps/ws-served/client -B build-ws-client -DCMAKE_BUILD_TYPE=Debug
cmake --build build-ws-client
```

**2. Build and run the native core server** (no emsdk needed — plain CMake):

```bash
cmake -S apps/ws-served/server -B build-server -DCMAKE_BUILD_TYPE=Release
cmake --build build-server
./build-server/cds_server          # listens on ws://0.0.0.0:9002 (port as argv[1])
```

The websocket server attaches a plant, selected by an optional second argument —
`loopback` (default) or `sitl`, whilst the first argument is the communication port:

```bash
./build-server/cds_server 9002 loopback   # default: the echo test double
./build-server/cds_server 9002 sitl       # ArduPilot SITL over MAVLink/UDP
```

**3. Serve the frontend with COOP/COEP headers** (required: the proxy uses
`SharedArrayBuffer` to make the async WebSocket look synchronous to embind):

```bash
python3 tools/serve.py 8080
```

The server URL defaults to `ws://localhost:9002` and can be overridden with
the `?ws=` query parameter, e.g. `http://localhost:8080/frontend/?ws=ws://192.168.1.10:9002`.
A quick end-to-end check is available at `http://localhost:8080/apps/ws-served/test/test_ws_e2e.html`.

To go back to the fully in-browser app: `cmake --build build-wasm-only`.

## Running against ArduPilot SITL

The `sitl` plant speaks MAVLink 2 over UDP to an ArduPilot **Copter** SITL
(use the QuadRotor model in the frontend). Start the server with the plant
selected — it listens, GCS-style, on `0.0.0.0:14550` and learns the vehicle
from the first valid datagram:

```bash
./build-server/cds_server 9002 sitl
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

## GitHub Pages

The repository includes a GitHub Actions workflow ([`.github/workflows/deploy.yml`](../.github/workflows/deploy.yml)) that automatically builds the WASM in Release mode and deploys to GitHub Pages on every push to `main`.

## Repository Structure

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
│   │   └── QuadRotor.hpp / QuadRotor.cpp       # 6 DOF quadrotor model (quaternion)
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
│   └── ws/                                     # Dependency-free WebSocket RPC transport
│       ├── CMakeLists.txt                      # cds_ws_client (emscripten) / cds_ws_server (native)
│       ├── ws_server.hpp / ws_server.cpp       # Minimal RFC 6455 WebSocket RPC server
│       ├── ws_rpc_client.hpp / .cpp            # Synchronous WASM RPC transport (EM_JS + SharedArrayBuffer)
│       └── ws_rpc_client_pre.js                # pre-js: spawns the WebSocket bridge worker
│
├── tools/
│   └── serve.py                                # Dev server with COOP/COEP headers
│
├── modeling/
│   ├── requirements.txt                        # Python requirements
│   └── notebooks/
│       ├── dynamics_rocket_FFLQR01.ipynb       # Rocket dynamics with LQR + FF for trajectory tracking
│       ├── dynamics_quadRotor_FFLQR01.ipynb    # QuadRotor dynamics with LQR + flatness FF
│       ├── base_codegen.py                     # Shared C++ code-generation base
│       ├── rocket_codegen.py                   # Rocket-specific codegen (derives from base)
│       ├── quad_codegen.py                     # QuadRotor-specific codegen (derives from base)
│       └── exported_cpp/
│           ├── ROCKET_FF_LQR_01/
│           │   └── dynamics_rocket_ff_lqr_01.cpp / .hpp        # Generated rocket dynamics + controller
│           └── QUADROTOR_FF_LQR_01/
│               └── dynamics_quadrotor_ff_lqr_01.cpp / .hpp     # Generated quadrotor dynamics + controller
│
├── frontend/
│   ├── index.html
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
