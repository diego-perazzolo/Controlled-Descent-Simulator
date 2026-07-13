# Controlled Descent Simulator

A real-time, interactive simulator of **controlled flight and descent**, built with a C++ physics core compiled to **WebAssembly** and a vanilla JS frontend. Runs entirely in the browser — no server required.

Two vehicle models are available, selectable at runtime:
- **Rocket** — powered descent of a single-stage booster (SpaceX Falcon 9 style)
- **QuadRotor** — quaternion-based 6-DOF quadrotor with differential-flatness feedforward

**[Live Demo](https://diego-perazzolo.github.io/Controlled-Descent-Simulator/frontend/)**

![Rocket demo](docs/demo.gif)

<!-- TODO: add quadrotor demo GIF -->
<!-- ![QuadRotor demo](docs/demo-quadrotor.gif) -->

---

## Project Scope

Simulate the controlled 3D flight of a rocket booster and of a quadrotor, with:
- A **dynamics modeling notebook** per vehicle (Jupyter/SymPy), with C++ code generation
- A **physics core** written in C++20, compiled to `.wasm` via Emscripten
- A **plain HTML/JS frontend** for real-time visualization, parameter configuration, and interactive control
- Full deployment on **GitHub Pages** 

---

## Features

### Frontend

- **Model selector** — switch between Rocket and QuadRotor at runtime; each model has its own parameter panel
- **Charts view** — real-time strip charts for x, y, z position, yaw error and position error magnitude
- **3D view** — Three.js scene with vehicle mesh (rocket: body + nose cone + landing legs; quadrotor: frame + rotors), trajectory trail, orbital camera (orbit / pan / zoom)
- **Params tab** — edit all physical and trajectory parameters at runtime (including yaw setpoints); Apply & Reset re-initializes the core without reloading the page
- **User force buttons** — six hold-to-apply buttons (±X, ±Y, ±Z) inject external perturbation forces into the simulation at every step; force magnitude is configurable
- **Simulation controls** — Start / Stop / Reset
- **Live simulation time** display

### Core (C++ → WebAssembly)

#### Communication Layer (`ext/`)
- `ext_rocketInit(params)` — initializes the Rocket model with parameters and actuator limits
- `ext_quadRotorInit(params)` — initializes the QuadRotor model with parameters and actuator limits
- `ext_step(stepParams)` — advances one integration step; returns full state and tracking errors (position + yaw)
- `ext_trajectory_get_point(timeInstant)` - provides a point along the reference trajectory, used for trajectory preview
- `ext_trajectory_append_poly4(params)` - appends a trajectory of type polynomial 4th order, configured with total time for the maneuver, initial/final position, velocity, acceleration and yaw
- `ext_trajectory_append_point(params)` - appends a trajectory of type point, configured with a final position, a final yaw and the total time needed for the maneuver
- `ext_trajectory_remove_last_item(void)` - removes the last trajectory item from the trajectory list
- Emscripten `embind` bindings expose all structs and functions to JavaScript

#### Physics Engine
- **6 DOF rigid body dynamics** (3 translational + 3 rotational) for both models
- **Rocket** — Euler-angle attitude; augmented state with 4 error integrators (x, y, z, yaw); inputs: main thrust + 3 torques
- **QuadRotor** — quaternion attitude (13 physical states) + 4 error integrators; inputs: 4 per-rotor thrusts (QuadX allocation); differential-flatness feedforward with heading-frame LQR correction
- Forces: thrust, gravity, aerodynamic drag (parametric coefficients `c`, `cz`), user-injected perturbations
- ODE integration: **Runge-Kutta 4 (RK4)**
- Parametric controller (LQR with FF on all actuators, actuator saturation)
- Model dynamics and controller gains are **generated as C++** from the Jupyter notebooks

### Screenshots

| Charts view | 3D view | Physics params view | Trajectory params view |
|:-----------:|:-------:|:-------:|:-------:|
| ![Charts](docs/screenshot-charts.png) | ![3D](docs/screenshot-3d.png) | ![Params](docs/screenshot-params-physics.png) | ![Params](docs/screenshot-params-trajectory.png) |

---

## Software Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  FRONTEND (HTML + JS)                   │
│                                                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐  │
│  │ Charts view │  │   3D view   │  │   Params tab    │  │
│  │  (canvas)   │  │ (Three.js)  │  │  (form + Apply) │  │
│  └──────┬──────┘  └──────┬──────┘  └────────┬────────┘  │
│         └────────────────┼──────────────────┘           │
│              renderers[].update(state, err)             │
└───────────────────────────┬─────────────────────────────┘
                            │  ext_rocketInit() / ext_quadRotorInit(),
                            │  ext_step(), ext_trajectory_...()
                            ▼
┌─────────────────────────────────────────────────────────┐
│                   SIMULATOR (.wasm)                     │
│                                                         │
│  ┌───────────────────────────────────────────────────┐  │
│  │           ext/ — Communication Layer              │  │
│  │   embind bindings · struct conversion · errors    │  │
│  └──────────────────────┬────────────────────────────┘  │
│                         │                               │
│  ┌──────────────────────▼────────────────────────────┐  │
│  │              Core Physics (C++)                   │  │
│  │  ┌──────────┐   ┌────────────┐   ┌────────────┐   │  │
│  │  │  Models  │   │ Controller │   │ Trajectory │   │  │
│  │  │ Rkt/Quad │   │  FF + LQR  │   │  Manager   │   │  │
│  │  └──────────┘   └────────────┘   └────────────┘   │  │
│  └──────────────────────┬────────────────────────────┘  │
│                         │                               │
│  ┌──────────────────────▼────────────────────────────┐  │
│  │                   Dynamics                        │  │
│  │  ┌──────────┐   ┌────────────┐   ┌────────────┐   │  │
│  │  │ Jupyter  │   │ Generated  │   │ Controller │   │  │
│  │  │ notebook │   │    C++     │   │   design   │   │  │
│  │  └──────────┘   └────────────┘   └────────────┘   │  │
│  └───────────────────────────────────────────────────┘  │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## Physics Model

### Degrees of Freedom
6 DOF rigid body (both models):
- **Translational**: X, Y, Z position and velocity in inertial frame
- **Rotational**: Rocket uses Euler angles; QuadRotor uses a unit quaternion

### State

**Rocket** (12 physical + 4 integrators):
```
x, y, z                       — position (m)
alpha, beta, psi              — Euler angles (rad)
x_dot, y_dot, z_dot           — linear velocity (m/s)
alpha_dot, beta_dot, psi_dot  — angular rates (rad/s)
IntX, IntY, IntZ, IntPsi      — tracking-error integrators
```

**QuadRotor** (13 physical + 4 integrators):
```
x, y, z                   — position (m)
qw, qx, qy, qz            — attitude quaternion
vx, vy, vz                — linear velocity (m/s)
wx, wy, wz                — body angular rates (rad/s)
IntX, IntY, IntZ, IntPsi  — tracking-error integrators
```

### Forces and Torques
- **Rocket**: main thrust + 3 control torques; **QuadRotor**: 4 per-rotor thrusts mapped to collective thrust + 3 torques (QuadX allocation)
- Gravity: `F_g = m·g` along −Z
- Aerodynamic drag: lateral coefficient `c`, axial coefficient `cz`
- External perturbations: user-injected force vector `(fX, fY, fZ)`

### Integration
Runge-Kutta 4 (RK4), fixed step `dt`.

### Controller
LQR on tracking error (position + yaw, with error integrators), feedforward on all actuators (differential flatness for the QuadRotor), actuator saturation; all derived in the Jupyter notebooks and exported as C++

---

## Core API

```cpp
// Initialize the Rocket model (FF_LQR_01), returns true on error
bool ext_initRocket_FFLQR01(ext_initRocketParams params);      // JS: ext_rocketInit

// Initialize the QuadRotor model (FF_LQR_01), returns true on error
bool ext_initQuadRotor_FFLQR01(ext_initQuadRotorParams params); // JS: ext_quadRotorInit

// Advance one integration step
ext_stepRet ext_step(ext_stepParams params);

// Get a point at time instant t along the trajectory
ext_trajectoryPoint ext_trajectory_get_point(ext_coord_t t);

/* Add a trajectory Polynomial 4th order, returns true on error */
bool ext_trajectory_append_poly4(ext_trajectoryPoly4Params_t params);

/* Add a trajectory Point, returns true on error */
bool ext_trajectory_append_point(ext_trajectoryPointParams_t params);

/* Remove last trajectory item, returns true on error */
bool ext_trajectory_remove_last_item(void);
```


### Key types
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

---

## Tech Stack

| Layer | Technology |
|---|---|
| Modeling | Python |
| Dynamics | C++20 |
| Physics core | C++20 |
| WASM compilation | Emscripten (`emcc`) |
| JS bindings | Emscripten `embind` |
| Frontend | Vanilla HTML + JS (ES modules) |
| 3D rendering | Three.js |
| Build | CMake |
| Deployment | GitHub Pages |

---

## Repository Structure

```
/
├── core/
│   ├── CMakeLists.txt
│   ├── core.hpp / core.cpp                     # C-style public interface (stubs → impl)
│   ├── core_defs.hpp                           # Internal type definitions
│   ├── Models/
│   │   ├── BaseModel.hpp / BaseModel.cpp       # Abstract model base
│   │   ├── Rocket.hpp / Rocket.cpp             # 6 DOF rocket model (Euler angles)
│   │   └── QuadRotor.hpp / QuadRotor.cpp       # 6 DOF quadrotor model (quaternion)
│   ├── Trajectory/
│   │   ├── TrajectoryManager.hpp / .cpp        # trajectory composition
│   │   ├── Trajectory.hpp / Trajectory.cpp     # base trajectory class
│   │   ├── Point.hpp / Point.cpp               # waypoint-like
│   │   └── Poly4.hpp / Poly4.cpp               # 4th order polynomial trajectory
│   └── ext/
│       ├── ext_defs.hpp                        # External struct definitions
│       ├── ext_comm.hpp / ext_comm.cpp         # Adapter layer (ext ↔ core)
│       └── bindings.cpp                        # Emscripten embind bindings
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
├── .github/
│   └── workflows/
│       └── deploy.yml                          # GitHub Pages CI/CD
│
└── Readme.md
```

---

## Prerequisites

| Tool | Version | Notes |
|------|---------|-------|
| [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html) | 3.1.56+ | Provides `emcmake` / `emcc` |
| CMake | 3.15+ | Build system |
| Python 3 | any | Local dev server |

## Build & Run

### Compile the core (WebAssembly)

If necessary configure the environment with:

```bash
source pathToEmSDK/emsdk_env.sh
```

**Debug** (with source maps for local development):

```bash
emcmake cmake -S core -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

**Release** (optimised, no debug symbols):

```bash
emcmake cmake -S core -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

### Run the frontend locally

From the project root:

```bash
python3 -m http.server 8080
```

Then open `http://localhost:8080/frontend/` in the browser.

### Run Jupyter notebooks in VS Code

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

### GitHub Pages

The repository includes a GitHub Actions workflow ([`.github/workflows/deploy.yml`](.github/workflows/deploy.yml)) that automatically builds the WASM in Release mode and deploys to GitHub Pages on every push to `main`.

---

## Roadmap

- [x] Architecture design
- [x] Communication layer (`ext/`) with embind bindings
- [x] Plain HTML/JS frontend — charts, 3D view, params tab, force buttons
- [x] Three.js 3D scene — rocket mesh + trajectory trail + OrbitControls
- [x] C++ core: 6 DOF model + RK4 integrator
- [x] C++ core: LQR controller
- [x] Customizable trajectory (with yaw setpoints)
- [x] QuadRotor model (quaternion attitude, flatness FF + LQR)
- [x] Shared codegen base class (rocket + quadrotor generators)
- [ ] C++ core: PID controller
- [x] GitHub Pages deployment (CI/CD workflow)
- [x] Quaternion rotation support (QuadRotor)
- [ ] DAE solver (future)

---

## Author

Diego Perazzolo

Co-Authored-By: Claude AI (mainly frontend, docs and VS Code Setup)

---

## License

MIT — see [LICENSE](LICENSE) for details.
