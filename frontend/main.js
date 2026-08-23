import createSimulator from '../build/simulator.js';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// =============================================================================
// Config
// =============================================================================
const DEFAULT_TIMESTEP_S = 0.01;
const TIMESTEP_MIN_S     = 0.0000001;
const TIMESTEP_MAX_S     = 0.5;

// Available dynamic models. The backend exposes a separate init entry point
// per model; the trajectory / run / snapshot API is shared between them.
const MODEL_ROCKET        = 'rocket';
const MODEL_QUADROTOR     = 'quadrotor';
const MODEL_QUADROTOR_MPC = 'quadrotor_mpc';   // same airframe, nonlinear MPC controller
const MODEL_ROCKET_MPC    = 'rocket_mpc';      // same airframe, nonlinear MPC controller
// Each airframe has two variants that share params, mesh, panels and camera and
// differ only in which backend init entry point they call: the quadrotor family
// (FF-LQR vs MPC) and the rocket family (FF-LQR vs MPC).
const isQuadFamily   = (m) => m === MODEL_QUADROTOR || m === MODEL_QUADROTOR_MPC;
const isRocketFamily = (m) => m === MODEL_ROCKET    || m === MODEL_ROCKET_MPC;

// Hardcoded fallback for the very first Poly4 when the trajectory list is
// empty and the user has no rocket state to seed from. Matches the reference
// descent in the design notebook (section 6).
const SEED_INITIAL_POS = { x: -10, y:  20, z: 80 };
const SEED_INITIAL_VEL = { x:   0, y:   5, z: -30 };
// Yaw seeds for the initial Poly4 (rad / rad·s⁻¹). Only meaningful once a
// model that tracks yaw is selected, but always sent to the backend.
const SEED_INITIAL_YAW      = 0;
const SEED_INITIAL_YAW_RATE = 0;

// Default trajectory loaded at boot so a user can press Start immediately
// without having to compose a sequence first. Loaded once, on the first boot;
// not re-injected on Reset / Apply, so a user who explicitly clears the
// sequence stays cleared.
const DEFAULT_TRAJECTORY = [
    {
        kind: 'poly4',
        params: {
            initialPos:     { x: -10, y: 20, z:  80 },
            initialYaw:     0,
            initialVel:     { x:   0, y:  5, z: -30 },
            initialYawRate: 0,
            finalPos:       { x:   0, y:  0, z:   0 },
            finalYaw:       0,
            finalVel:       { x:   0, y:  0, z:   0 },
            finalYawRate:   0,
            finalAcc:       { x:   0, y:  0, z:   0 },
            finalYawAcc:    0,
            time_s: 20,
        },
    },
];

// ext_initRocketParams: { rocketPar, rocketActuatorLimits }
const ROCKET_INIT_PARAMS = {
    rocketPar: {
        mass_Kg:        10.0,
        inertiaX_Kgm2: 10.0 / 3,
        inertiaY_Kgm2: 10.0 / 3,
        inertiaZ_Kgm2: 1,
        c:              1,
        cz:             0.02,
    },
    rocketActuatorLimits: {
        fZ_max: 500.0,
        fZ_min: 0.0,
        Tx_max: 10.0,
        Tx_min: -10.0,
        Ty_max: 10.0,
        Ty_min: -10.0,
        Tz_max: 10.0,
        Tz_min: -10.0,
    },
};

// ext_initQuadRotorParams: { quadRotorPar, quadRotorActuatorLimits }
// Defaults mirror the backend PhysicsParams reference airframe.
// Note: gravity (g) has no field in the binding and is owned by the backend.
const QUADROTOR_INIT_PARAMS = {
    quadRotorPar: {
        mass_Kg:        2.4,
        inertiaX_Kgm2: 0.025,
        inertiaY_Kgm2: 0.025,
        inertiaZ_Kgm2: 0.045,
        c:              0.2,
        cz:             0.3,
        motorThrustCoefficient: 1.0e-5,
        motorTorqueCoefficient: 1.6e-7,
        distanceBtwMotorAndCoM: 0.275,
        motorMomentOfInertia:   3.0e-5,
    },
    quadRotorActuatorLimits: {
        motor_max_thrust: 36.0,
        motor_min_thrust: 0.0,
    },
};

// Map model id -> its default init params, so boot / model-switch can pick.
const DEFAULT_INIT_PARAMS = {
    [MODEL_ROCKET]:        ROCKET_INIT_PARAMS,
    [MODEL_QUADROTOR]:     QUADROTOR_INIT_PARAMS,
    [MODEL_QUADROTOR_MPC]: QUADROTOR_INIT_PARAMS,   // MPC reuses the airframe params
    [MODEL_ROCKET_MPC]:    ROCKET_INIT_PARAMS,      // MPC reuses the airframe params
};

// =============================================================================
// Simulation state
// =============================================================================
let sim        = null;
let renderer3d = null;
let chartsRenderer = null;
let running    = false;
let frameId    = null;
let simTime    = 0;
let stepCount  = 0;
let lastFpsTs  = performance.now();
let fpsCounter = 0;
let fpsDisplay = 0;
let timestep_s = DEFAULT_TIMESTEP_S;
let simRate = 1.0;   // simulation speed multiplier (pure sim only; 1.0 = real-time)
let currentModel = MODEL_ROCKET;   // 'rocket' | 'rocket_mpc' | 'quadrotor' | 'quadrotor_mpc'
let plantAvailable = false;        // plant attached AND publishing fresh samples

// =============================================================================
// Renderers registry — add a renderer here to hook into the sim loop
// Each renderer is { update(state, err, simTime, stepCount, plantState), reset() }
// plantState is null when plant snapshots are unavailable.
// =============================================================================
const renderers = [];

// =============================================================================
// DOM
// =============================================================================
const $ = id => document.getElementById(id);

const ui = {
    simTime:    $('simTime'),
    x:          $('x'),      y:      $('y'),      z:      $('z'),
    x_dot:      $('x_dot'),  y_dot:  $('y_dot'),  z_dot:  $('z_dot'),  v_mag: $('v_mag'),
    roll:       $('roll'),   pitch:  $('pitch'),  yaw:    $('yaw'),
    roll_dot:   $('roll_dot'), pitch_dot: $('pitch_dot'), yaw_dot: $('yaw_dot'),
    xErr:       $('xErr'),   yErr:   $('yErr'),   zErr:   $('zErr'),   yawErr: $('yawErr'), err_mag: $('err_mag'),
    stepCount:  $('stepCount'), dt: $('dt'), fps: $('fps'),
    btnStart:   $('btnStart'), btnStop: $('btnStop'), btnReset: $('btnReset'),
    btnCharts:  $('btnCharts'), btn3d: $('btn3d'), btnParams: $('btnParams'), btnDiag: $('btnDiag'),
    viewCharts: $('view-charts'), view3d: $('view-3d'), viewParams: $('view-params'), viewDiag: $('view-diag'),
    status:     $('statusBar'), error: $('errorMsg'), perfWarn: $('perfWarn'),
    controlsPanel: $('controlsPanel'), btnControlsToggle: $('btnControlsToggle'),
    btnApply:   $('btnApply'),
    // trajectory save / load
    btnTrajSave:   $('btnTrajSave'),
    btnTrajLoad:   $('btnTrajLoad'),
    trajFileInput: $('trajFileInput'),
    // controller parameters (data-driven panel + JSON save / load)
    ctrlParamsInfo: $('ctrlParamsInfo'),
    ctrlParams:     $('ctrlParams'),
    btnCtrlSave:    $('btnCtrlSave'),
    btnCtrlLoad:    $('btnCtrlLoad'),
    ctrlFileInput:  $('ctrlFileInput'),
    // 3D view source toggles
    chkViewTraj:  $('chkViewTraj'),
    chkViewModel: $('chkViewModel'),
    chkViewPlant: $('chkViewPlant'),
    lblViewPlant: $('lblViewPlant'),
    // plant / staging controls
    plantBar:     $('plantBar'),
    plantStatus:  $('plantStatus'),
    stageSafetyAlt: $('stageSafetyAlt'),
    btnBeginStaging: $('btnBeginStaging'),
    btnStopStaging:  $('btnStopStaging'),
    modelSelect: $('modelSelect'),
    // Rocket params panel
    panelRocket: $('panel-rocket'),
    p_mass: $('p_mass'), p_iX: $('p_iX'), p_iY: $('p_iY'), p_iZ: $('p_iZ'),
    p_c:    $('p_c'),    p_cz: $('p_cz'),
    p_fZmax: $('p_fZmax'), p_fZmin: $('p_fZmin'), p_tXmax: $('p_tXmax'), p_tXmin: $('p_tXmin'), p_tYmax: $('p_tYmax'), p_tYmin: $('p_tYmin'), p_tZmax: $('p_tZmax'), p_tZmin: $('p_tZmin'),
    // Quadrotor params panel
    panelQuad: $('panel-quad'),
    q_mass: $('q_mass'), q_iX: $('q_iX'), q_iY: $('q_iY'), q_iZ: $('q_iZ'),
    q_c:    $('q_c'),    q_cz: $('q_cz'),
    q_kThrust: $('q_kThrust'), q_kTorque: $('q_kTorque'), q_dist: $('q_dist'), q_motI: $('q_motI'),
    q_motMax: $('q_motMax'), q_motMin: $('q_motMin'),
    p_dt:    $('p_dt'),
    rateBox: $('rateBox'),
    sldRate: $('sldRate'),
    rateVal: $('rateVal'),
};

// =============================================================================
// User force state — updated by force buttons
// =============================================================================
const userForce = { fX: 0, fY: 0, fZ: 0 };

// Push the current tick period and user forces to the backend. Integration
// runs on the backend tick thread, so these take effect from the next tick.
// Returns the backend error code (truthy = failure).
function sendSystemParams() {
    return sim.ext_setSystemParams({
        timestep_seconds: timestep_s,
        rate: simRate,
        user_forces: { fX: userForce.fX, fY: userForce.fY, fZ: userForce.fZ },
    });
}

function setupForceButtons() {
    document.querySelectorAll('.btn-force').forEach(btn => {
        const axis = btn.dataset.axis;          // 'fX' | 'fY' | 'fZ'
        const sign = parseFloat(btn.dataset.sign);

        const press = () => {
            const mag = parseFloat($('forceMag').value) || 0;
            userForce[axis] = sign * mag;
            btn.classList.add('pressing');
            if (sim) sendSystemParams();
        };
        const release = () => {
            userForce[axis] = 0;
            btn.classList.remove('pressing');
            if (sim) sendSystemParams();
        };

        btn.addEventListener('mousedown',   press);
        btn.addEventListener('touchstart',  press,   { passive: true });
        btn.addEventListener('mouseup',     release);
        btn.addEventListener('mouseleave',  release);
        btn.addEventListener('touchend',    release);
    });
}

// =============================================================================
// Params form helpers
// =============================================================================
function fillRocketForm(p) {
    ui.p_mass.value  = p.rocketPar.mass_Kg;
    ui.p_iX.value    = p.rocketPar.inertiaX_Kgm2;
    ui.p_iY.value    = p.rocketPar.inertiaY_Kgm2;
    ui.p_iZ.value    = p.rocketPar.inertiaZ_Kgm2;
    ui.p_c.value     = p.rocketPar.c;
    ui.p_cz.value    = p.rocketPar.cz;
    ui.p_fZmax.value = p.rocketActuatorLimits.fZ_max;
    ui.p_fZmin.value = p.rocketActuatorLimits.fZ_min;
    ui.p_tXmax.value = p.rocketActuatorLimits.Tx_max;
    ui.p_tXmin.value = p.rocketActuatorLimits.Tx_min;
    ui.p_tYmax.value = p.rocketActuatorLimits.Ty_max;
    ui.p_tYmin.value = p.rocketActuatorLimits.Ty_min;
    ui.p_tZmax.value = p.rocketActuatorLimits.Tz_max;
    ui.p_tZmin.value = p.rocketActuatorLimits.Tz_min;
}

function fillQuadForm(p) {
    ui.q_mass.value    = p.quadRotorPar.mass_Kg;
    ui.q_iX.value      = p.quadRotorPar.inertiaX_Kgm2;
    ui.q_iY.value      = p.quadRotorPar.inertiaY_Kgm2;
    ui.q_iZ.value      = p.quadRotorPar.inertiaZ_Kgm2;
    ui.q_c.value       = p.quadRotorPar.c;
    ui.q_cz.value      = p.quadRotorPar.cz;
    ui.q_kThrust.value = p.quadRotorPar.motorThrustCoefficient;
    ui.q_kTorque.value = p.quadRotorPar.motorTorqueCoefficient;
    ui.q_dist.value    = p.quadRotorPar.distanceBtwMotorAndCoM;
    ui.q_motI.value    = p.quadRotorPar.motorMomentOfInertia;
    ui.q_motMax.value  = p.quadRotorActuatorLimits.motor_max_thrust;
    ui.q_motMin.value  = p.quadRotorActuatorLimits.motor_min_thrust;
}

// Populate the whole params view for a given model's default params, and set
// the shared timestep field.
function fillParamsForm(p) {
    if (p.quadRotorPar) fillQuadForm(p);
    else                fillRocketForm(p);
    ui.p_dt.value = timestep_s;
}

const n = id => parseFloat($(id).value) || 0;

function readRocketForm() {
    return {
        rocketPar: {
            mass_Kg:        n('p_mass'),
            inertiaX_Kgm2: n('p_iX'),
            inertiaY_Kgm2: n('p_iY'),
            inertiaZ_Kgm2: n('p_iZ'),
            c:              n('p_c'),
            cz:             n('p_cz'),
        },
        rocketActuatorLimits: {
            fZ_max: n('p_fZmax'),
            fZ_min: n('p_fZmin'),
            Tx_max: n('p_tXmax'),
            Tx_min: n('p_tXmin'),
            Ty_max: n('p_tYmax'),
            Ty_min: n('p_tYmin'),
            Tz_max: n('p_tZmax'),
            Tz_min: n('p_tZmin'),
        },
    };
}

function readQuadForm() {
    return {
        quadRotorPar: {
            mass_Kg:        n('q_mass'),
            inertiaX_Kgm2: n('q_iX'),
            inertiaY_Kgm2: n('q_iY'),
            inertiaZ_Kgm2: n('q_iZ'),
            c:              n('q_c'),
            cz:             n('q_cz'),
            motorThrustCoefficient: n('q_kThrust'),
            motorTorqueCoefficient: n('q_kTorque'),
            distanceBtwMotorAndCoM: n('q_dist'),
            motorMomentOfInertia:   n('q_motI'),
        },
        quadRotorActuatorLimits: {
            motor_max_thrust: n('q_motMax'),
            motor_min_thrust: n('q_motMin'),
        },
    };
}

// Read the params for whatever model is currently selected, in the shape its
// init entry point expects.
function readParamsForm() {
    return isQuadFamily(currentModel) ? readQuadForm() : readRocketForm();
}

function readTimestep() {
    let v = parseFloat(ui.p_dt.value);
    if (!isFinite(v) || v <= 0) v = DEFAULT_TIMESTEP_S;
    if (v < TIMESTEP_MIN_S) v = TIMESTEP_MIN_S;
    if (v > TIMESTEP_MAX_S) v = TIMESTEP_MAX_S;
    ui.p_dt.value = v;
    return v;
}

// The live tick-period slider is usable only when its "edit" box is checked, a
// model exists, and no plant is attached — a plant paces the tick, so the period
// is locked (the core enforces this too, not just this control). Kept in sync
// with the current period.
// The live simulation-speed (rate) slider is usable once a model exists and no
// plant is attached — a plant forces real-time (the core enforces this too). The
// timestep/granularity itself stays at the model's value (params-form p_dt);
// only the playback speed changes here.
// The speed slider and the user-force buttons are set-up controls: needed now
// and then, in the way the rest of the time. They collapse like the log dock,
// the choice is remembered, and on a phone-sized screen they start collapsed --
// there the primary row alone already fills the width.
const CONTROLS_COLLAPSE_KEY = 'cds.controls.collapsed';
const NARROW_SCREEN_PX = 700;

function setControlsCollapsed(collapsed) {
    ui.controlsPanel.classList.toggle('collapsed', collapsed);
    ui.btnControlsToggle.textContent = collapsed ? 'Controls ▾' : 'Controls ▴';
    ui.btnControlsToggle.setAttribute('aria-expanded', String(!collapsed));
    try { localStorage.setItem(CONTROLS_COLLAPSE_KEY, collapsed ? '1' : '0'); } catch (e) {}
    publishChromeMetrics();   // the bar just changed height: the views resize with it
}

function initControlsPanel() {
    let collapsed = window.innerWidth < NARROW_SCREEN_PX;      // default by screen
    try {
        const stored = localStorage.getItem(CONTROLS_COLLAPSE_KEY);
        if (stored !== null) collapsed = (stored === '1');     // the user decided
    } catch (e) {}
    setControlsCollapsed(collapsed);
    ui.btnControlsToggle.addEventListener('click',
        () => setControlsCollapsed(!ui.controlsPanel.classList.contains('collapsed')));
}

function refreshRateControl() {
    // A plant paces real-time: the speed control is meaningless, so remove it
    // from the bar entirely (the core enforces the lock regardless).
    ui.rateBox.style.display = plantAvailable ? 'none' : '';
    ui.sldRate.disabled      = !sim;
    ui.sldRate.value         = String(simRate);
    ui.rateVal.textContent   = `${simRate.toFixed(2)}×`;
}

ui.sldRate.addEventListener('input', () => {
    const v = parseFloat(ui.sldRate.value);
    if (!isFinite(v) || v <= 0) return;
    simRate = v;
    ui.rateVal.textContent = `${simRate.toFixed(2)}×`;
    if (sim && sendSystemParams()) setError('Rate change rejected (a plant is attached).');
    else                          setError('');
});

const fmt  = (v, d = 4) => v.toFixed(d);
const fmtI = v => Math.round(v).toString();

function setStatus(msg) { ui.status.textContent = msg; }
function setError(msg)  { ui.error.textContent  = msg; }

// Performance banner. Empty message hides it. Unlike the status/error lines it
// is fixed to the viewport, so it reaches the user from the 3D view too.
function setPerfWarning(msg) {
    ui.perfWarn.textContent   = msg;
    ui.perfWarn.style.display = msg ? 'block' : 'none';
}

// Initialise the backend core for the currently-selected model. `params` must
// already be in the shape expected by that model's init entry point:
//   rocket    -> ext_initRocketParams    { rocketPar, rocketActuatorLimits }
//   quadrotor -> ext_initQuadRotorParams { quadRotorPar, quadRotorActuatorLimits }
// Init leaves the backend stopped; the tick period is pushed right after so
// the tick thread is configured before the first Run.
// Returns the backend error code (truthy = failure), matching the old ext_init.
function initBackend(params) {
    let err;
    if (currentModel === MODEL_QUADROTOR)          err = sim.ext_quadRotorInit(params);
    else if (currentModel === MODEL_QUADROTOR_MPC) err = sim.ext_quadRotorMpcInit(params);
    else if (currentModel === MODEL_ROCKET_MPC)    err = sim.ext_rocketMpcInit(params);
    else                                           err = sim.ext_rocketInit(params);
    if (err) return err;
    refreshRateControl();    // enable the live sim-speed control for the new model
    return sendSystemParams();
}

// =============================================================================
// 3D renderer
// =============================================================================
function make3DRenderer() {
    const TRAIL_MAX = 6000;
    const TRAJECTORY_MAX = 6000;
    const trail      = [];
    const plantTrail = [];
    const trajectory = [];

    let scene, camera, renderer, controls;
    let rocketGroup, trailLine, trajectoryLine;   // rocketGroup = active vehicle mesh
    let plantGroup, plantTrailLine;               // plant ghost mesh + its trail
    let initialized = false;
    let animating   = false;
    let visible     = false;

    // Which sources are drawn (driven by the view3d-sources checkboxes)
    const sources = { traj: true, model: true, plant: false };

    function buildRocket() {
        const group = new THREE.Group();

        // Body
        const body = new THREE.Mesh(
            new THREE.CylinderGeometry(0.5, 0.6, 6, 20),
            new THREE.MeshLambertMaterial({ color: 0xdddddd })
        );
        body.position.y = 3;
        group.add(body);

        // Nose cone (red = front/top)
        const nose = new THREE.Mesh(
            new THREE.ConeGeometry(0.5, 2, 20),
            new THREE.MeshLambertMaterial({ color: 0xff3333 })
        );
        nose.position.y = 7;
        group.add(nose);

        // 4 landing legs
        for (let i = 0; i < 4; i++) {
            const angle = (i / 4) * Math.PI * 2;
            const leg   = new THREE.Mesh(
                new THREE.BoxGeometry(0.12, 2, 0.12),
                new THREE.MeshLambertMaterial({ color: 0x888888 })
            );
            leg.position.set(Math.cos(angle) * 0.9, 0.8, Math.sin(angle) * 0.9);
            leg.rotation.z = Math.cos(angle) * 0.35;
            leg.rotation.x = -Math.sin(angle) * 0.35;
            group.add(leg);
        }

        return group;
    }

    function buildQuadrotor() {
        const group = new THREE.Group();

        const armColor   = 0x555555;
        const motorColor = 0x222222;
        const hubColor   = 0xdddddd;

        // Real-size drone (scene units = meters): ~520-class frame,
        // centre->motor arm 0.26 m as on the physical vehicle.
        const ARM_LEN   = 0.26;   // centre -> motor, scene units (m)
        const ARM_THICK = 0.02;
        const MOTOR_R   = 0.014;
        const PROP_R    = 0.115;  // ~9" propeller

        // Central hub
        const hub = new THREE.Mesh(
            new THREE.CylinderGeometry(0.06, 0.06, 0.035, 16),
            new THREE.MeshLambertMaterial({ color: hubColor })
        );
        group.add(hub);

        // Small canopy so orientation is readable
        const canopy = new THREE.Mesh(
            new THREE.SphereGeometry(0.08, 16, 12, 0, Math.PI * 2, 0, Math.PI / 2),
            new THREE.MeshLambertMaterial({ color: 0xaaaaaa })
        );
        canopy.position.y = 0.03;
        group.add(canopy);

        // X-configuration: motors at 45°, so +X/+Z quadrant is "front-right".
        // Front rotors get a distinct colour so heading is visible in flight.
        const diagonals = [
            { ax:  1, az:  1, front: true  },
            { ax: -1, az:  1, front: true  },
            { ax:  1, az: -1, front: false },
            { ax: -1, az: -1, front: false },
        ];

        const arm = ARM_LEN / Math.SQRT2;   // per-axis offset for a 45° arm
        for (const d of diagonals) {
            const mx = d.ax * arm;
            const mz = d.az * arm;

            // Arm (box) from hub to motor
            const armMesh = new THREE.Mesh(
                new THREE.BoxGeometry(ARM_THICK, ARM_THICK, ARM_LEN),
                new THREE.MeshLambertMaterial({ color: armColor })
            );
            armMesh.position.set(mx / 2, 0, mz / 2);
            armMesh.lookAt(new THREE.Vector3(mx, 0, mz));
            group.add(armMesh);

            // Motor pod
            const motor = new THREE.Mesh(
                new THREE.CylinderGeometry(MOTOR_R, MOTOR_R, 0.03, 14),
                new THREE.MeshLambertMaterial({ color: motorColor })
            );
            motor.position.set(mx, 0.025, mz);
            group.add(motor);

            // Propeller disc
            const prop = new THREE.Mesh(
                new THREE.CylinderGeometry(PROP_R, PROP_R, 0.006, 20),
                new THREE.MeshLambertMaterial({
                    color: d.front ? 0xff3333 : 0x33aaff,
                    transparent: true,
                    opacity: 0.55,
                })
            );
            prop.position.set(mx, 0.045, mz);
            group.add(prop);
        }

        return group;
    }

    // Build the mesh for whichever model is currently selected.
    function buildVehicle() {
        return isQuadFamily(currentModel) ? buildQuadrotor() : buildRocket();
    }

    // Ghost version of the vehicle for the plant: same shape, translucent green
    function buildPlantVehicle() {
        const group = buildVehicle();
        group.traverse(obj => {
            if (obj.isMesh) {
                obj.material = obj.material.clone();
                obj.material.color.set(0x33ff88);
                obj.material.transparent = true;
                obj.material.opacity = 0.45;
            }
        });
        return group;
    }

    // Camera preset per model: the quadrotor is real-size (~0.75 m tip-to-tip),
    // so it needs a much closer start and a smaller zoom-in limit than the rocket.
    function applyCameraForModel() {
        if (!camera || !controls) return;
        if (isQuadFamily(currentModel)) {
            camera.position.set(2.5, 1.5, 2.5);
            controls.minDistance = 0.5;
        } else {
            camera.position.set(40, 25, 40);
            controls.minDistance = 5;
        }
        controls.target.set(0, 0, 0);
        controls.update();
    }

    function init() {
        const container = $('view-3d');
        if (!container) return false;

        const W = container.offsetWidth  || 800;
        const H = container.offsetHeight || 500;

        // WebGL renderer
        renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.setSize(W, H);
        renderer.setClearColor(0x0d0d0d);
        container.appendChild(renderer.domElement);

        // Scene
        scene = new THREE.Scene();
        scene.fog = new THREE.Fog(0x0d0d0d, 400, 1200);

        // Camera
        camera = new THREE.PerspectiveCamera(50, W / H, 0.1, 5000);

        // OrbitControls
        controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping  = true;
        controls.dampingFactor  = 0.06;
        controls.maxDistance    = 1000;
        applyCameraForModel();

        // Lights
        scene.add(new THREE.AmbientLight(0xffffff, 0.45));
        const sun = new THREE.DirectionalLight(0xffffff, 1.1);
        sun.position.set(200, 400, 150);
        scene.add(sun);

        // Ground grid
        scene.add(new THREE.GridHelper(600, 60, 0x2a2a2a, 0x1a1a1a));

        // Vehicle (rocket or quadrotor, per current model)
        rocketGroup = buildVehicle();
        scene.add(rocketGroup);

        // Plant ghost vehicle (hidden until enabled and fed by snapshots)
        plantGroup = buildPlantVehicle();
        plantGroup.visible = false;
        scene.add(plantGroup);

        // Trail line. frustumCulled = false: these lines update their geometry
        // every frame without recomputing the bounding sphere, so the cached
        // (stale) sphere makes the culler drop the whole line when zoomed in —
        // never cull them, they are cheap and must stay visible at any zoom.
        trailLine = new THREE.Line(
            new THREE.BufferGeometry(),
            new THREE.LineBasicMaterial({ color: 0x0099ff, transparent: true, opacity: 0.55 })
        );
        trailLine.frustumCulled = false;
        scene.add(trailLine);

        // Plant trail line
        plantTrailLine = new THREE.Line(
            new THREE.BufferGeometry(),
            new THREE.LineBasicMaterial({ color: 0x33ff88, transparent: true, opacity: 0.5 })
        );
        plantTrailLine.frustumCulled = false;
        scene.add(plantTrailLine);

        // Trajectory preview
        trajectoryLine = new THREE.Line(
            new THREE.BufferGeometry(),
            new THREE.LineBasicMaterial({ color: 0xff9900, transparent: true, opacity: 0.55 })
        );
        trajectoryLine.frustumCulled = false;
        scene.add(trajectoryLine);

        // Resize observer
        new ResizeObserver(() => {
            if (!renderer) return;
            const w = container.offsetWidth, h = container.offsetHeight;
            if (!w || !h) return;
            renderer.setSize(w, h);
            camera.aspect = w / h;
            camera.updateProjectionMatrix();
        }).observe(container);

        initialized = true;
        return true;
    }

    function renderLoop() {
        if (!animating) return;
        requestAnimationFrame(renderLoop);
        controls?.update();
        renderer?.render(scene, camera);
    }

    function updateTrail(points, line, x, y, z) {
        points.push(new THREE.Vector3(x, y, z));
        if (points.length > TRAIL_MAX) points.shift();

        const pos = new Float32Array(points.length * 3);
        points.forEach((v, i) => {
            pos[i * 3]     = v.x;
            pos[i * 3 + 1] = v.y;
            pos[i * 3 + 2] = v.z;
        });
        line.geometry.setAttribute(
            'position', new THREE.BufferAttribute(pos, 3)
        );
        line.geometry.setDrawRange(0, points.length);
        line.geometry.attributes.position.needsUpdate = true;
    }

    function pushTrajectoryPoint(x, y, z) {
        trajectory.push(new THREE.Vector3(x, y, z));
        if (trajectory.length > TRAJECTORY_MAX) trajectory.shift();

        const pos = new Float32Array(trajectory.length * 3);
        trajectory.forEach((v, i) => {
            pos[i * 3]     = v.x;
            pos[i * 3 + 1] = v.y;
            pos[i * 3 + 2] = v.z;
        });
        trajectoryLine.geometry.setAttribute(
            'position', new THREE.BufferAttribute(pos, 3)
        );
        trajectoryLine.geometry.setDrawRange(0, trajectory.length);
        trajectoryLine.geometry.attributes.position.needsUpdate = true;
    }

    function generateTrajectoryPreview() {
        if (!sim) return;

        // Sample the live backend trajectory over [0, totalDuration]. The
        // total duration is owned by the trajectory builder so the preview
        // automatically follows append/remove operations.
        const totalDuration = trajectoryBuilder.getTotalDuration();
        if (totalDuration <= 0) return;

        const sampleDt = 0.2;
        for (let t = 0; t <= totalDuration; t += sampleDt) {
            const p = sim.ext_trajectory_get_point(t);
            // sim(x, y, z=up) -> Three.js(x, z, y)
            pushTrajectoryPoint(p.x, p.z, p.y);
        }
    }

    function clearTrajectoryPreview() {
        trajectory.length = 0;
        if (trajectoryLine) trajectoryLine.geometry.setDrawRange(0, 0);
    }

    // sim(x, y, z=up) → Three.js(x, z, y)  [Y is up in Three.js]
    // The world→scene axis swap (y↔z) is a reflection: every world rotation
    // maps to the permuted scene axis with negated angle (world X→scene X,
    // world Y→scene Z, world Z→scene Y), and the composition order must
    // match each model's Euler sequence.
    function poseVehicle(group, state) {
        group.position.set(state.x, state.z, state.y);
        if (isQuadFamily(currentModel)) {
            // Aerospace ZYX: roll about X, pitch about Y, yaw about Z
            // R = Rz(yaw)·Ry(pitch)·Rx(roll) → scene 'YZX' with negated angles.
            group.rotation.set(-state.roll, -state.yaw, -state.pitch, 'YZX');
        } else {
            // Rocket Rm = Ry(alpha)·Rx(beta)·Rz(psi), GetState maps
            // roll=alpha, pitch=beta, yaw=psi → scene 'ZXY' with negated angles.
            group.rotation.set(-state.pitch, -state.yaw, -state.roll, 'ZXY');
        }
    }

    function applySources() {
        if (!initialized) return;
        trajectoryLine.visible = sources.traj;
        rocketGroup.visible    = sources.model;
        trailLine.visible      = sources.model;
        plantTrailLine.visible = sources.plant;
        if (!sources.plant) plantGroup.visible = false;
    }

    return {
        update(state, err, simTime, stepCount, plantState) {
            if (!initialized || !visible) return;

            // Generate preview lazily on first display, or after invalidation.
            if (trajectory.length === 0) {
                generateTrajectoryPreview();
            }

            poseVehicle(rocketGroup, state);
            updateTrail(trail, trailLine, state.x, state.z, state.y);

            // Plant ghost: only when snapshots are flowing
            if (plantState) {
                poseVehicle(plantGroup, plantState);
                updateTrail(plantTrail, plantTrailLine,
                            plantState.x, plantState.z, plantState.y);
            }
            plantGroup.visible = sources.plant && !!plantState;

            applySources();
        },
        // Pose ONLY the plant ghost, leaving the model and charts untouched.
        // Used before Start so the vehicle staging near the origin is visible
        // (and responds to manual commands) while the simulation is idle.
        previewPlant(plantState) {
            if (!initialized || !visible) return;
            if (plantState) {
                poseVehicle(plantGroup, plantState);
                updateTrail(plantTrail, plantTrailLine,
                            plantState.x, plantState.z, plantState.y);
            }
            plantGroup.visible = sources.plant && !!plantState;
            applySources();
        },
        reset() {
            trail.length = 0;
            plantTrail.length = 0;
            if (trailLine) trailLine.geometry.setDrawRange(0, 0);
            if (plantTrailLine) plantTrailLine.geometry.setDrawRange(0, 0);

            for (const g of [rocketGroup, plantGroup]) {
                if (g) {
                    g.position.set(0, 0, 0);
                    g.rotation.set(0, 0, 0);
                }
            }
            if (plantGroup) plantGroup.visible = false;
        },
        // Enable/disable the drawn sources (reference trajectory, model, plant)
        setSources(s) {
            Object.assign(sources, s);
            applySources();
        },
        // Drop the cached trajectory line so the next frame regenerates it
        // from the current backend state. Called by the trajectory builder
        // whenever the sequence changes.
        invalidateTrajectory() {
            clearTrajectoryPreview();
        },
        // Swap the vehicle mesh to match the current model. Safe to call before
        // init (no-op until the scene exists); the correct mesh is then built
        // lazily on first show().
        rebuildVehicle() {
            if (!initialized || !scene) return;
            if (rocketGroup) scene.remove(rocketGroup);
            if (plantGroup)  scene.remove(plantGroup);
            rocketGroup = buildVehicle();
            rocketGroup.position.set(0, 0, 0);
            rocketGroup.rotation.set(0, 0, 0);
            scene.add(rocketGroup);
            plantGroup = buildPlantVehicle();
            plantGroup.visible = false;
            scene.add(plantGroup);
            applySources();
            applyCameraForModel();
        },
        show() {
            visible = true;

            if (!initialized) {
                init();
            }

            if (!animating) { animating = true; renderLoop(); }
        },
        hide() {
            visible   = false;
            animating = false;
        },
    };
}

// =============================================================================
// Charts renderer (canvas 2D, no charting library)
// =============================================================================
// Time-series panels drawn by hand: a ring buffer per series (no per-frame array
// copying), a real simulated-time x axis over a sliding window, labelled y
// ticks, and the commanded reference drawn under the measured value so tracking
// is readable at a glance. The buffers fill on every tick, but nothing is
// rasterised while the charts view is off screen -- on the wasm-only build the
// canvases share a thread with the simulation, so drawing what nobody is looking
// at would slow the run down.
function makeChartsRenderer() {
    const CAP      = 6000;   // ring capacity (samples)
    const WINDOW_S = 12.0;   // simulated seconds shown
    const PAD_L = 46, PAD_R = 10, PAD_T = 10, PAD_B = 18;

    // One panel per canvas. `ref` marks the series that have a commanded value
    // to compare against (|err| is an error magnitude: nothing to track).
    const charts = [
        { id: 'chartX',   key: 'x',   color: '#f80', unit: 'm',   ref: true  },
        { id: 'chartY',   key: 'y',   color: '#0f8', unit: 'm',   ref: true  },
        { id: 'chartZ',   key: 'z',   color: '#0cf', unit: 'm',   ref: true  },
        { id: 'chartYaw', key: 'yaw', color: '#c8f', unit: 'rad', ref: true  },
        { id: 'chartErr', key: 'e',   color: '#fa0', unit: 'm',   ref: false },
    ];

    // Ring buffer: `head` is the next write slot, `n` the samples held.
    const t   = new Float64Array(CAP);
    const val = {}, ref = {};
    for (const c of charts) { val[c.key] = new Float64Array(CAP); ref[c.key] = new Float64Array(CAP); }
    let head = 0, n = 0;

    function push(time, values, references) {
        t[head] = time;
        for (const c of charts) {
            val[c.key][head] = values[c.key];
            ref[c.key][head] = references[c.key] ?? 0;
        }
        head = (head + 1) % CAP;
        if (n < CAP) n++;
    }

    // Indices of the samples inside the visible window, oldest first. Walking
    // the ring backwards from the newest stops as soon as the window is filled,
    // so the cost follows what is drawn, not what is stored.
    function windowIndices() {
        const out = [];
        if (n === 0) return out;
        const newest = (head - 1 + CAP) % CAP;
        const tEnd   = t[newest];
        for (let k = 0; k < n; k++) {
            const i = (newest - k + CAP) % CAP;
            if (tEnd - t[i] > WINDOW_S) break;
            out.push(i);
        }
        out.reverse();
        return out;
    }

    // Match the backing store to the CSS box and the device pixel ratio, so
    // lines stay crisp on a HiDPI screen instead of being upscaled.
    function fitCanvas(canvas) {
        const w = canvas.offsetWidth, h = canvas.offsetHeight;
        if (!w || !h) return null;
        const dpr = Math.min(window.devicePixelRatio || 1, 2);
        const bw = Math.round(w * dpr), bh = Math.round(h * dpr);
        if (canvas.width !== bw || canvas.height !== bh) { canvas.width = bw; canvas.height = bh; }
        const ctx = canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        return { ctx, w, h };
    }

    const fmtAxis = v => (Math.abs(v) >= 100 ? v.toFixed(0)
                        : Math.abs(v) >= 10  ? v.toFixed(1)
                                             : v.toFixed(2));

    function drawChart(chart, idx) {
        const canvas = $(chart.id);
        if (!canvas) return;
        const fit = fitCanvas(canvas);
        if (!fit) return;
        const { ctx, w, h } = fit;

        ctx.clearRect(0, 0, w, h);
        if (idx.length < 2) return;

        const series = val[chart.key];
        const target = ref[chart.key];

        // vertical range over what is actually on screen, value and reference
        let lo = Infinity, hi = -Infinity;
        for (const i of idx) {
            const v = series[i];
            if (v < lo) lo = v;
            if (v > hi) hi = v;
            if (chart.ref) {
                const r = target[i];
                if (r < lo) lo = r;
                if (r > hi) hi = r;
            }
        }
        if (!isFinite(lo) || !isFinite(hi)) return;
        if (hi - lo < 1e-9) { lo -= 0.5; hi += 0.5; }          // flat series: give it room
        const margin = (hi - lo) * 0.08;
        lo -= margin; hi += margin;

        const t0 = t[idx[0]], t1 = t[idx[idx.length - 1]];
        const span = (t1 - t0) || 1;
        const plotW = w - PAD_L - PAD_R, plotH = h - PAD_T - PAD_B;
        const X = time  => PAD_L + ((time - t0) / span) * plotW;
        const Y = value => PAD_T + (1 - (value - lo) / (hi - lo)) * plotH;

        // ---- grid + labels
        ctx.font = '10px monospace';
        ctx.textBaseline = 'middle';
        ctx.strokeStyle = '#1e1e1e';
        ctx.fillStyle   = '#556';
        ctx.lineWidth   = 1;
        for (let g = 0; g <= 2; g++) {
            const value = lo + (hi - lo) * (g / 2);
            const y = Math.round(Y(value)) + 0.5;
            ctx.beginPath(); ctx.moveTo(PAD_L, y); ctx.lineTo(w - PAD_R, y); ctx.stroke();
            ctx.textAlign = 'right';
            ctx.fillText(fmtAxis(value), PAD_L - 6, y);
        }
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';
        for (let g = 0; g <= 2; g++) {
            const time = t0 + span * (g / 2);
            const x = Math.round(X(time)) + 0.5;
            ctx.strokeStyle = '#1a1a1a';
            ctx.beginPath(); ctx.moveTo(x, PAD_T); ctx.lineTo(x, h - PAD_B); ctx.stroke();
            ctx.fillStyle = '#556';
            ctx.fillText(`${time.toFixed(1)}s`, x, h - PAD_B + 4);
        }

        // zero line, when the window straddles it
        if (lo <= 0 && hi >= 0) {
            const y = Math.round(Y(0)) + 0.5;
            ctx.strokeStyle = '#333';
            ctx.beginPath(); ctx.moveTo(PAD_L, y); ctx.lineTo(w - PAD_R, y); ctx.stroke();
        }

        // ---- reference first, so the measured value reads on top of it
        if (chart.ref) {
            ctx.strokeStyle = '#667';
            ctx.lineWidth = 1;
            ctx.setLineDash([4, 3]);
            ctx.beginPath();
            idx.forEach((i, k) => { const px = X(t[i]), py = Y(target[i]); k ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
            ctx.stroke();
            ctx.setLineDash([]);
        }

        ctx.strokeStyle = chart.color;
        ctx.lineWidth   = 1.5;
        ctx.beginPath();
        idx.forEach((i, k) => { const px = X(t[i]), py = Y(series[i]); k ? ctx.lineTo(px, py) : ctx.moveTo(px, py); });
        ctx.stroke();

        // ---- current value, top right
        const last = idx[idx.length - 1];
        ctx.textAlign = 'right';
        ctx.textBaseline = 'top';
        ctx.fillStyle = chart.color;
        ctx.fillText(`${fmtAxis(series[last])} ${chart.unit}`, w - PAD_R, PAD_T);
    }

    // Drawing is skipped unless the charts view is the one on screen: the
    // buffers keep filling regardless, so switching to it shows the history.
    function chartsVisible() {
        return ui.viewCharts && ui.viewCharts.style.display !== 'none' && !document.hidden;
    }

    return {
        update(state, err, simTime) {
            const eMag = Math.sqrt(err.xErr**2 + err.yErr**2 + err.zErr**2);
            // The backend reports the tracking error as (reference - state), so
            // the commanded value comes back by adding it to the measurement.
            push(simTime,
                 { x: state.x, y: state.y, z: state.z, yaw: state.yaw, e: eMag },
                 { x: state.x + err.xErr, y: state.y + err.yErr, z: state.z + err.zErr,
                   yaw: state.yaw + err.yawErr });

            if (!chartsVisible()) return;
            const idx = windowIndices();
            charts.forEach(c => drawChart(c, idx));
        },
        // Redraw on demand (entering the charts view) without adding a sample.
        redraw() {
            if (!chartsVisible()) return;
            const idx = windowIndices();
            charts.forEach(c => drawChart(c, idx));
        },
        reset() {
            head = 0; n = 0;
            charts.forEach(({ id }) => {
                const c = $(id);
                if (c) c.getContext('2d').clearRect(0, 0, c.width, c.height);
            });
        },
    };
}

// =============================================================================
// Trajectory builder
// =============================================================================
//
// Responsibilities:
//   - Owns the JS-side spec list of trajectory items (the "sequence").
//   - Forwards every append/remove to the backend immediately, keeping the
//     UI list and the WASM-side trajectory in lockstep. If the backend
//     rejects an append, the UI list is not modified, so the two views never
//     drift.
//   - Emits change notifications so the rest of the app can: (a) invalidate
//     the 3D trajectory preview, (b) enable/disable the Start button.
//
// Item shape on the JS side:
//   { kind: 'poly4',
//     params: { initialPos:{x,y,z}, initialYaw, initialVel:{x,y,z}, initialYawRate,
//               finalPos:{x,y,z},   finalYaw,   finalVel:{x,y,z},   finalYawRate,
//               finalAcc:{x,y,z},   finalYawAcc, time_s } }
//   { kind: 'point',
//     params: { finalPos:{x,y,z}, finalYaw, time_s } }
//
const trajectoryBuilder = (() => {
    const items = [];
    const onChangeListeners = [];

    // ---- DOM ----
    const dom = {
        list:        $('trajList'),
        count:       $('trajCount'),
        total:       $('trajTotal'),
        btnRemove:   $('btnTrajRemoveLast'),
        btnClear:    $('btnTrajClear'),
        btnAppend:   $('btnTrajAppend'),
        type:        $('traj_type'),
        time:        $('traj_time'),
        // Poly4 inputs
        poly4Box: $('traj_poly4_fields'),
        p0: ['traj_p0_x', 'traj_p0_y', 'traj_p0_z'].map($),
        v0: ['traj_v0_x', 'traj_v0_y', 'traj_v0_z'].map($),
        pF: ['traj_pF_x', 'traj_pF_y', 'traj_pF_z'].map($),
        vF: ['traj_vF_x', 'traj_vF_y', 'traj_vF_z'].map($),
        aF: ['traj_aF_x', 'traj_aF_y', 'traj_aF_z'].map($),
        // Poly4 yaw scalars
        yaw0:     $('traj_yaw0'),
        yawRate0: $('traj_yawRate0'),
        yawF:     $('traj_yawF'),
        yawRateF: $('traj_yawRateF'),
        yawAccF:  $('traj_yawAccF'),
        // Point inputs
        pointBox: $('traj_point_fields'),
        pt: ['traj_pt_x', 'traj_pt_y', 'traj_pt_z'].map($),
        ptYaw: $('traj_pt_yaw'),
    };

    // ---- Helpers ----
    function num(el) {
        const v = parseFloat(el.value);
        return isFinite(v) ? v : 0;
    }
    function readVec3(els)    { return { x: num(els[0]), y: num(els[1]), z: num(els[2]) }; }
    function setVec3(els, v)  { els[0].value = v.x; els[1].value = v.y; els[2].value = v.z; }
    function setScalar(el, v) { if (el) el.value = v; }
    function emitChange()     { onChangeListeners.forEach(fn => fn()); }
    function getTotalDuration() {
        return items.reduce((acc, it) => acc + it.params.time_s, 0);
    }

    function endStateOfLastItem() {
        // What the rocket position/velocity/yaw will be at the end of the last
        // item in the sequence. Used to seed initialPos/initialVel/initialYaw
        // of the next Poly4 so successive segments line up by default.
        if (items.length === 0) {
            return {
                pos: SEED_INITIAL_POS, vel: SEED_INITIAL_VEL,
                yaw: SEED_INITIAL_YAW, yawRate: SEED_INITIAL_YAW_RATE,
            };
        }
        const last = items[items.length - 1];
        if (last.kind === 'poly4') {
            return {
                pos: last.params.finalPos, vel: last.params.finalVel,
                yaw: last.params.finalYaw ?? 0, yawRate: last.params.finalYawRate ?? 0,
            };
        }
        // 'point' has no finalVel/finalYawRate; assume zero at the waypoint.
        return {
            pos: last.params.finalPos, vel: { x: 0, y: 0, z: 0 },
            yaw: last.params.finalYaw ?? 0, yawRate: 0,
        };
    }

    // ---- Rendering ----
    function renderList() {
        if (items.length === 0) {
            dom.list.innerHTML =
                '<div class="empty">No items. Append a Poly4 or Point below to build the sequence.</div>';
        } else {
            const rows = items.map((it, i) => {
                const fp = (it.kind === 'poly4') ? it.params.finalPos : it.params.finalPos;
                return `
                    <tr>
                        <td class="row-idx">${i}</td>
                        <td class="row-type">${it.kind}</td>
                        <td class="row-time">${it.params.time_s.toFixed(2)} s</td>
                        <td class="row-pos">to (${fmt(fp.x,2)}, ${fmt(fp.y,2)}, ${fmt(fp.z,2)})</td>
                    </tr>`;
            }).join('');
            dom.list.innerHTML = `
                <table>
                    <thead><tr>
                        <th>#</th><th>type</th><th style="text-align:right">duration</th><th>final pos</th>
                    </tr></thead>
                    <tbody>${rows}</tbody>
                </table>`;
        }
        dom.count.textContent = items.length.toString();
        dom.total.textContent = getTotalDuration().toFixed(3);

        const empty = items.length === 0;
        dom.btnRemove.disabled = empty;
        dom.btnClear.disabled  = empty;
    }

    function refreshAutofill() {
        // Update the Poly4 form's initialPos/initialVel to match the end of
        // the current sequence. Fields stay editable; this is just a default.
        const end = endStateOfLastItem();
        setVec3(dom.p0, end.pos);
        setVec3(dom.v0, end.vel);
        setScalar(dom.yaw0,     end.yaw);
        setScalar(dom.yawRate0, end.yawRate);
    }

    function setTypeVisibility() {
        const t = dom.type.value;
        dom.poly4Box.style.display = (t === 'poly4') ? '' : 'none';
        dom.pointBox.style.display = (t === 'point') ? '' : 'none';
    }

    // ---- Append / remove ----
    function appendCurrent() {
        const kind = dom.type.value;
        const time_s = parseFloat(dom.time.value);
        if (!isFinite(time_s) || time_s <= 0) {
            setError('Trajectory: time_s must be > 0');
            return false;
        }

        let item, ok;
        if (kind === 'poly4') {
            item = {
                kind: 'poly4',
                params: {
                    initialPos:     readVec3(dom.p0),
                    initialYaw:     num(dom.yaw0),
                    initialVel:     readVec3(dom.v0),
                    initialYawRate: num(dom.yawRate0),
                    finalPos:       readVec3(dom.pF),
                    finalYaw:       num(dom.yawF),
                    finalVel:       readVec3(dom.vF),
                    finalYawRate:   num(dom.yawRateF),
                    finalAcc:       readVec3(dom.aF),
                    finalYawAcc:    num(dom.yawAccF),
                    time_s,
                },
            };
            ok = !sim.ext_trajectory_append_poly4(item.params);
        } else {
            item = {
                kind: 'point',
                params: {
                    finalPos: readVec3(dom.pt),
                    finalYaw: num(dom.ptYaw),
                    time_s,
                },
            };
            ok = !sim.ext_trajectory_append_point(item.params);
        }

        if (!ok) {
            setError(`Trajectory: backend rejected append (${kind})`);
            return false;
        }

        items.push(item);
        setError('');
        renderList();
        refreshAutofill();
        emitChange();
        return true;
    }

    function removeLast() {
        if (items.length === 0) return;
        const ok = !sim.ext_trajectory_remove_last_item();
        if (!ok) {
            setError('Trajectory: backend rejected remove_last_item');
            return;
        }
        items.pop();
        renderList();
        refreshAutofill();
        emitChange();
    }

    function clearAll() {
        // Pop until empty. Keep the JS list in sync only after each backend
        // call succeeds, so a mid-loop failure leaves a consistent state.
        while (items.length > 0) {
            const ok = !sim.ext_trajectory_remove_last_item();
            if (!ok) {
                setError('Trajectory: backend rejected remove_last_item during clear');
                renderList();
                refreshAutofill();
                emitChange();
                return;
            }
            items.pop();
        }
        renderList();
        refreshAutofill();
        emitChange();
    }

    // ---- Replay (after re-init wipes backend state) ----
    function replayToBackend() {
        // After re-init the backend trajectory is assumed to be empty. Push
        // every JS-side item back into it. If any push fails, reset the UI
        // list to whatever the backend actually accepted.
        const surviving = [];
        for (const it of items) {
            const ok = (it.kind === 'poly4')
                ? !sim.ext_trajectory_append_poly4(it.params)
                : !sim.ext_trajectory_append_point(it.params);
            if (!ok) {
                setError(`Trajectory: replay failed at item ${surviving.length}`);
                break;
            }
            surviving.push(it);
        }
        items.length = 0;
        items.push(...surviving);
        renderList();
        refreshAutofill();
        emitChange();
    }

    // ---- Export / replace (JSON save / load) ----
    function getItems() {
        // Deep copy of the JS-side spec list, for serialization. params hold
        // only plain data (numbers and {x,y,z}), so a JSON round-trip clones
        // them safely and drops any accidental references.
        return items.map(it => ({
            kind: it.kind,
            params: JSON.parse(JSON.stringify(it.params)),
        }));
    }

    function replaceItems(newItems) {
        // Replace the whole sequence (used by JSON load). Clear the current
        // backend + JS sequence, then append the incoming items. Item shape is
        // validated at the call site; here we only guard backend rejections and
        // keep the JS list in lockstep with what the backend actually holds.
        while (items.length > 0) {
            if (sim.ext_trajectory_remove_last_item()) {   // true == error
                setError('Trajectory: backend rejected remove during load');
                renderList();
                refreshAutofill();
                emitChange();
                return false;
            }
            items.pop();
        }
        let ok = true;
        for (const it of newItems) {
            const accepted = (it.kind === 'poly4')
                ? !sim.ext_trajectory_append_poly4(it.params)
                : !sim.ext_trajectory_append_point(it.params);
            if (!accepted) {
                setError(`Trajectory: backend rejected item ${items.length} during load`);
                ok = false;
                break;
            }
            items.push(it);
        }
        renderList();
        refreshAutofill();
        emitChange();
        return ok;
    }

    // ---- Load preset (boot only) ----
    function loadPreset(preset) {
        // Append a list of items both to the backend and to the JS spec list,
        // stopping at the first backend rejection. Used to inject a default
        // trajectory at boot so the user can hit Start without composing a
        // sequence by hand. Not used on Reset/Apply: those preserve whatever
        // the user has chosen, including an explicit clear.
        for (const it of preset) {
            const ok = (it.kind === 'poly4')
                ? !sim.ext_trajectory_append_poly4(it.params)
                : !sim.ext_trajectory_append_point(it.params);
            if (!ok) {
                setError(`Trajectory: preset load failed at item ${items.length}`);
                break;
            }
            items.push(it);
        }
        renderList();
        refreshAutofill();
        emitChange();
    }

    // ---- Wiring ----
    dom.type.addEventListener('change', setTypeVisibility);
    dom.btnAppend.addEventListener('click', appendCurrent);
    dom.btnRemove.addEventListener('click', removeLast);
    dom.btnClear.addEventListener('click',  clearAll);

    setTypeVisibility();
    renderList();

    return {
        onChange(fn) { onChangeListeners.push(fn); },
        isEmpty:        () => items.length === 0,
        getTotalDuration,
        replayToBackend,
        loadPreset,
        refreshAutofill,
        getItems,
        replaceItems,
    };
})();

// =============================================================================
// Main panels update
// =============================================================================
function updatePanels(state, err, t, step) {
    ui.simTime.innerHTML = `${t.toFixed(3)} <span>s</span>`;

    ui.x.textContent = fmt(state.x);
    ui.y.textContent = fmt(state.y);
    ui.z.textContent = fmt(state.z);

    ui.x_dot.textContent = fmt(state.x_dot);
    ui.y_dot.textContent = fmt(state.y_dot);
    ui.z_dot.textContent = fmt(state.z_dot);
    const vMag = Math.sqrt(state.x_dot**2 + state.y_dot**2 + state.z_dot**2);
    ui.v_mag.textContent = fmt(vMag);

    ui.roll.textContent      = fmt(state.roll);
    ui.pitch.textContent     = fmt(state.pitch);
    ui.yaw.textContent       = fmt(state.yaw);
    ui.roll_dot.textContent  = fmt(state.roll_dot);
    ui.pitch_dot.textContent = fmt(state.pitch_dot);
    ui.yaw_dot.textContent   = fmt(state.yaw_dot);

    ui.xErr.textContent = fmt(err.xErr);
    ui.yErr.textContent = fmt(err.yErr);
    ui.zErr.textContent = fmt(err.zErr);
    if (ui.yawErr) ui.yawErr.textContent = fmt(err.yawErr ?? 0);
    const eMag = Math.sqrt(err.xErr**2 + err.yErr**2 + err.zErr**2);
    ui.err_mag.textContent = fmt(eMag);

    ui.stepCount.textContent = fmtI(step);
    ui.dt.textContent        = `${(timestep_s * 1000).toFixed(2)} ms`;

    // FPS
    fpsCounter++;
    const now = performance.now();
    if (now - lastFpsTs >= 500) {
        fpsDisplay = Math.round(fpsCounter / ((now - lastFpsTs) / 1000));
        fpsCounter = 0;
        lastFpsTs = now;
    }
    ui.fps.textContent = fpsDisplay;
}

// =============================================================================
// Display loop — integration runs on the backend tick thread; here we only
// poll a snapshot per animation frame and refresh the panels/renderers.
// =============================================================================
// Plant freshness: the plant link lives from attach through the mission, so
// its sequence advances whenever it is publishing (staging OR running). We
// treat it as "available" while the sequence keeps changing, and drop it once
// it has been frozen (link down / detached) for longer than the timeout —
// independent of the frame rate.
let lastPlantSeq   = -1;
let lastPlantFreshMs = 0;
const PLANT_STALE_MS = 1000;

let plantReady = false;        // last isReadyToStart from the plant snapshot
let stagingRequested = false;  // Begin Staging pressed, not yet aborted

function refreshPlantFreshness(psnap) {
    const live = psnap.isAttached && !psnap.isError;
    const now  = performance.now();
    if (live && psnap.sequence !== lastPlantSeq) {
        lastPlantSeq     = psnap.sequence;
        lastPlantFreshMs = now;
    }
    setPlantAvailable(live && (now - lastPlantFreshMs) < PLANT_STALE_MS);

    // reflect the plant readiness in the staging status + Start gating
    const wasReady = plantReady;
    plantReady = plantAvailable && !!psnap.isReadyToStart;
    if (plantReady) {
        ui.plantStatus.textContent = 'Staged — ready';
        ui.plantStatus.className = 'ready';
        // nothing to abort once ready: only aborting an in-progress climb
        ui.btnStopStaging.disabled = true;
    } else if (plantAvailable && stagingRequested) {
        ui.plantStatus.textContent = 'Staging…';
        ui.plantStatus.className = 'staging';
        ui.btnStopStaging.disabled = running;
    } else if (plantAvailable) {
        ui.plantStatus.textContent = 'Not staged';
        ui.plantStatus.className = '';
        ui.btnStopStaging.disabled = true;
    }
    if (plantReady !== wasReady) refreshStartEnabled();
}

// The loop runs continuously from boot: the plant ghost is polled every frame
// so it is visible while staging (before Start), and the model/time/charts are
// driven only while the simulation is actually running.
// A synchronous ext_* call blocks the main thread while the transport spins for
// the response; if one blocks longer than this the server is treated as
// unresponsive and the simulation is auto-stopped (as if Stop were pressed), so
// the UI recovers instead of freezing. Kept just under the transport's own spin
// timeout (libs/ws/ws_rpc_client.cpp): above the worst legitimate RPC latency,
// below anything a user would tolerate as a freeze.
//
// It applies to the ws-served build ONLY. In wasm-only the core is in this same
// module: there is no transport that can stop answering, and a slow call just
// means a heavy model (a long MPC horizon). Auto-stopping there would blame a
// server that does not exist, and would look exactly like "the simulation
// stopped by itself".
const CALL_STALL_MS = 900;
let lastCallStalled = false;

// Run a blocking sim call and record whether it stalled. Because the RPC blocks
// the main thread until it returns, the wall-clock elapsed IS the block time.
function timedCall(fn) {
    const t0 = performance.now();
    const r = fn();
    const elapsed = performance.now() - t0;
    lastCallStalled = usesWsTransport() && elapsed > CALL_STALL_MS;
    if (!lastCallStalled && elapsed > CALL_STALL_MS) {
        // wasm-only: not a transport problem, but worth saying out loud once
        reportSlowCall(elapsed);
    }
    return r;
}

// True when the backend lives behind the WebSocket bridge (ws-served) rather
// than inside this page (wasm-only).
function usesWsTransport() { return !!globalThis.__cdsWs; }

let slowCallWarnedAt = 0;
function reportSlowCall(elapsed) {
    const now = performance.now();
    if (now - slowCallWarnedAt < 5000) return;
    slowCallWarnedAt = now;
    console.warn(`[cds] a backend call blocked this frame for ${Math.round(elapsed)} ms `
               + '(heavy model — e.g. a long MPC horizon). The simulation keeps running.');
}

function handleTransportStall() {
    reportStop('Server not responding (a call timed out) — simulation stopped.');
    stop();   // same effect as pressing Stop: halts the run and frees the UI
}

// Something stopped the run on its own. Say it where it cannot be missed (the
// sticky banner, visible from every view), keep it in the error line, and leave
// a console trace: an auto-stop that only whispers is indistinguishable from the
// simulation "just freezing".
function reportStop(message) {
    setError(message);
    setPerfWarning(message);
    console.error('[cds] ' + message);
}

// =============================================================================
// Chrome metrics
// =============================================================================
// The app bar wraps and the log dock collapses, so their heights are not
// constants: measure them and publish them as CSS variables, which is what the
// full-height views size themselves against (instead of a hard-coded guess).
function publishChromeMetrics() {
    const appBar  = $('appBar');
    const logDock = $('logDock');
    const root    = document.documentElement;
    if (appBar)  root.style.setProperty('--chrome-h', `${Math.round(appBar.getBoundingClientRect().height)}px`);
    if (logDock) root.style.setProperty('--dock-h',   `${Math.round(logDock.getBoundingClientRect().height)}px`);
}

function trackChromeSize() {
    if (typeof ResizeObserver === 'function') {
        const ro = new ResizeObserver(publishChromeMetrics);
        const appBar  = $('appBar');
        const logDock = $('logDock');
        if (appBar)  ro.observe(appBar);
        if (logDock) ro.observe(logDock);
    }
    window.addEventListener('resize', publishChromeMetrics);
    publishChromeMetrics();
}

// =============================================================================
// Speed watchdog
// =============================================================================
// The backend never refuses to run: an over-heavy model just advances the
// simulation far slower than asked, which reads as "nothing is happening".
// Over a window, compare the simulated time delivered against the wall time
// times the requested rate. Pure sim only: a plant paces the run itself.
const RATE_WATCH_WINDOW_S = 2.0;   // observation window
const RATE_WATCH_FLOOR    = 0.25;  // delivered/requested below this warns

let rateWatchArmed  = false;
let rateWatchWall_s = 0;   // wall clock at the window start
let rateWatchSim_s  = 0;   // simulated time at the window start
let rateWatchStale  = false;   // the window saw a hidden tab: not evidence

// A hidden tab throttles rAF — and in the wasm-only build the tick loop rides
// on it — so wall time passes with no simulation. Mark those windows instead of
// guessing from their length: a single tick CAN legitimately take seconds here,
// which is exactly the case worth reporting.
document.addEventListener('visibilitychange', () => {
    if (document.hidden) rateWatchStale = true;
});

function resetRateWatch() {
    rateWatchArmed = false;
    setPerfWarning('');
}

function pollRateWatch() {
    if (!running || plantAvailable) { resetRateWatch(); return; }

    const wallNow_s = performance.now() / 1000;
    if (!rateWatchArmed) {
        rateWatchArmed  = true;
        rateWatchWall_s = wallNow_s;
        rateWatchSim_s  = simTime;
        return;
    }

    const wallElapsed_s = wallNow_s - rateWatchWall_s;
    if (wallElapsed_s < RATE_WATCH_WINDOW_S) return;

    const delivered_s = simTime - rateWatchSim_s;
    const requested_s = wallElapsed_s * simRate;
    rateWatchWall_s = wallNow_s;   // slide the window
    rateWatchSim_s  = simTime;

    // Discontinuities that are not a slow model: a tab that went hidden, and a
    // simulated-time reset. The evidence is void, so drop any standing verdict
    // and observe the next window (already re-based above).
    if (rateWatchStale) { rateWatchStale = false; setPerfWarning(''); return; }
    if (delivered_s < 0 || !(requested_s > 0)) { setPerfWarning(''); return; }

    const ratio = delivered_s / requested_s;
    if (ratio >= RATE_WATCH_FLOOR) { setPerfWarning(''); return; }

    if (delivered_s <= 0) {
        setPerfWarning('Simulation is not advancing — the model is too heavy to run here. '
                     + 'A Debug WASM build cannot drive the MPC models: rebuild in Release.');
    } else {
        setPerfWarning(`Simulation running at ${Math.round(ratio * 100)}% of the requested `
                     + `${simRate.toFixed(2)}× speed (about ${(ratio * simRate).toFixed(2)}× real time) — `
                     + 'the model is too heavy to keep up. Lower the speed or the MPC horizon, '
                     + 'and build in Release.');
    }
}

function loop() {
    frameId = requestAnimationFrame(loop);
    if (!sim) return;

    const psnap = timedCall(() => sim.ext_getPlantSnapshot());
    if (lastCallStalled) { handleTransportStall(); return; }
    refreshPlantFreshness(psnap);
    const plantState = plantAvailable ? psnap.state : null;

    if (running) {
        const snap = timedCall(() => sim.ext_getSnapshot());
        if (lastCallStalled) { handleTransportStall(); return; }
        if (snap.isError) {
            reportStop('The backend refused to report its state (ext_getSnapshot error) — simulation stopped.');
            stop();
            return;
        }
        simTime   = snap.time_seconds;
        stepCount = Math.round(simTime / timestep_s);   // estimated backend ticks
        updatePanels(snap.state, snap.err, simTime, stepCount);
        renderers.forEach(r => r.update(snap.state, snap.err, simTime, stepCount, plantState));
    } else {
        // Idle: keep only the plant ghost live, leaving model and charts as-is.
        renderer3d.previewPlant(plantState);
    }

    // Log stream drains continuously (persistent dock, every view); profiler
    // stats refresh only while the Diag view is up.
    diagnostics.pollLog();
    if (ui.viewDiag.style.display !== 'none') diagnostics.pollStats();

    // Verdict on whether the backend is keeping up with the requested speed
    pollRateWatch();
}

// =============================================================================
// Controls
// =============================================================================
function refreshStartEnabled() {
    // Start is allowed only when:
    //   - sim is not currently running, AND
    //   - the trajectory has at least one item (otherwise the backend has
    //     nothing to track and the run is meaningless), AND
    //   - if a plant is connected, it is ready (staged).
    // Loading a trajectory replays it into the backend, so it is only allowed
    // while stopped; saving is always safe.
    if (ui.btnTrajLoad) ui.btnTrajLoad.disabled = running;
    if (running) {
        ui.btnStart.disabled = true;
        return;
    }
    const plantBlocks = plantAvailable && !plantReady;
    ui.btnStart.disabled = trajectoryBuilder.isEmpty() || plantBlocks;
}

function start() {
    if (trajectoryBuilder.isEmpty()) {
        reportStop('Cannot start: the trajectory sequence is empty.');
        return;
    }
    if (sendSystemParams()) {
        reportStop('Cannot start: ext_setSystemParams failed — is a model initialized?');
        return;
    }
    if (sim.ext_run()) {
        // The usual cause is a backend trajectory that a failed re-init left
        // empty while the JS-side list still looks populated.
        reportStop('Cannot start: ext_run refused — the backend has no model or no trajectory. '
                 + 'Try Reset; if it persists, check the log.');
        return;
    }
    running = true;
    resetRateWatch();        // judge this run on its own window (also clears the banner)
    ui.btnStart.disabled = true;
    ui.btnStop.disabled  = false;
    ui.btnReset.disabled = true;
    ui.btnBeginStaging.disabled = true;   // no staging while the mission runs
    setStatus('Running...');
    setError('');
    refreshStartEnabled();   // gate the trajectory Load button while running
    // the poll loop is already running (started at boot); it now switches to
    // driving the model/time/charts because `running` is set
}

function stop() {
    running = false;
    resetRateWatch();
    // the poll loop keeps running (it still polls the plant ghost while idle)
    if (sim) sim.ext_stop();
    ui.btnStop.disabled  = true;
    ui.btnReset.disabled = false;
    ui.btnBeginStaging.disabled = false;   // staging allowed again once stopped
    // the plant clears its staging on Stop (the vehicle descended and must be
    // re-staged before another mission): reflect that in the UI
    stagingRequested = false;
    ui.btnStopStaging.disabled = true;
    setStatus('Stopped.');
    refreshStartEnabled();
}

function reset() {
    stop();
    timestep_s = readTimestep();
    simTime   = 0;
    stepCount = 0;
    ui.simTime.innerHTML = `0.000 <span>s</span>`;
    renderers.forEach(r => r.reset());
    setStatus('Ready.');
    setError('');
    ui.btnReset.disabled = true;

    // Re-init core. Init wipes the backend trajectory, so the JS-side
    // sequence has to be replayed for the next run to be valid. It also rebuilds
    // the model with default controller parameters, so carry the current tuning
    // across the re-init rather than losing it on every Reset.
    const savedCtrl = snapshotControllerParams();
    const err = initBackend(readParamsForm());
    if (err) {
        // Bailing out here leaves the backend without the trajectory the JS list
        // still shows, and the next Start fails with no obvious cause: say so.
        reportStop('Reset failed to re-initialize the model — the backend has no trajectory, '
                 + 'so Start will refuse. Check the log.');
        return;
    }
    trajectoryBuilder.replayToBackend();
    refreshControllerPanel();          // fresh model (defaults) + current manifest structure
    applyControllerParams(savedCtrl);  // re-apply the user's controller tuning
    refreshControllerPanel();          // reflect the re-applied values

    // 3D preview must be recomputed from the freshly-replayed backend state.
    renderer3d?.invalidateTrajectory?.();

    refreshStartEnabled();
}

ui.btnStart.addEventListener('click', start);
ui.btnStop.addEventListener('click',  stop);
ui.btnReset.addEventListener('click', reset);

// =============================================================================
// 3D view source toggles (reference trajectory / model / plant)
// =============================================================================
function readSourceToggles() {
    return {
        traj:  ui.chkViewTraj.checked,
        model: ui.chkViewModel.checked,
        plant: ui.chkViewPlant.checked && plantAvailable,
    };
}

function applySourceToggles() {
    renderer3d?.setSources?.(readSourceToggles());
}

// Enable/disable the "Plant" checkbox as snapshots (dis)appear. The checked
// state is preserved, so a plant that comes back is shown again without a click.
function setPlantAvailable(avail) {
    if (avail === plantAvailable) return;
    plantAvailable = avail;
    ui.chkViewPlant.disabled = !avail;
    ui.lblViewPlant.classList.toggle('disabled', !avail);
    // a plant just appeared: show its ghost automatically
    if (avail) ui.chkViewPlant.checked = true;
    // reveal the staging controls only while a plant is connected
    ui.plantBar.style.display = avail ? 'flex' : 'none';
    if (!avail) {
        stagingRequested = false;
        plantReady = false;
        ui.btnStopStaging.disabled = true;
        refreshStartEnabled();
    }
    applySourceToggles();
    refreshRateControl();   // lock/unlock the live sim-speed slider with the plant
}

ui.chkViewTraj.addEventListener('change',  applySourceToggles);
ui.chkViewModel.addEventListener('change', applySourceToggles);
ui.chkViewPlant.addEventListener('change', applySourceToggles);

// Staging controls
ui.btnBeginStaging.addEventListener('click', () => {
    const safetyAlt = parseFloat(ui.stageSafetyAlt.value);
    if (!(safetyAlt >= 0)) { setError('Safety altitude must be ≥ 0'); return; }
    if (sim.ext_beginStaging(safetyAlt)) {
        setError('Begin staging failed — need a plant, a non-empty trajectory, and a stopped sim');
        return;
    }
    stagingRequested = true;
    ui.btnStopStaging.disabled = false;
    setError('');
});
ui.btnStopStaging.addEventListener('click', () => {
    sim.ext_stopStaging();
    stagingRequested = false;
    ui.btnStopStaging.disabled = true;
});

// View toggle helpers
// =============================================================================
// Diagnostics (Diag view): logger console + logger/profiler controls. Everything
// here rides the six diagnostics ext commands, whose responses are text blobs
// (newline records, tab fields) parsed on this side — see docs/api.md. The core
// interface stays minimal: the logger is a secondary feature and adds no more.
// =============================================================================
const diagnostics = (() => {
    const LEVELS = ['Trace', 'Debug', 'Info', 'Warn', 'Error', 'Off'];
    const con        = $('logConsole');
    const dropped    = $('logDropped');
    const peek       = $('logPeek');
    const autoscroll = $('logAutoscroll');
    const logTable   = $('logModulesTable'),  logTbody  = logTable.querySelector('tbody'),  logEmpty  = $('logModulesEmpty');
    const profTable  = $('profileModulesTable'), profTbody = profTable.querySelector('tbody'), profEmpty = $('profileModulesEmpty');
    const statsTbody = $('profileStatsTable').querySelector('tbody'), statsEmpty = $('profileStatsEmpty');

    const MAX_LINES = 500;
    // Poll cadences (ms): decoupled from the frame rate so the blocking ext
    // calls (a WebSocket round-trip each, in ws-served) do not stall rendering.
    const LOG_POLL_MS = 120;    // ~8 Hz log stream
    const STATS_POLL_MS = 500;  // 2 Hz profiler stats
    let totalDropped = 0;
    let lastLogPoll = 0;
    let lastStatsPoll = 0;

    // Per-scope time series for the profiler sparklines, accumulated client-side
    // from successive getProfilerTable snapshots (no extra ext traffic). Keyed by
    // "module\tscope"; ~1 min of history at the stats cadence.
    const TREND_LEN = 120;
    const trends = new Map();
    function pushTrend(key, p95) {
        let t = trends.get(key);
        if (!t) { t = { p95: [] }; trends.set(key, t); }
        t.p95.push(p95);
        if (t.p95.length > TREND_LEN) t.p95.shift();
    }
    // Draw a scope's p95 trend (amber) into its row canvas, scaled to the
    // window's peak so the shape is always visible.
    function drawSpark(canvas, t) {
        const ctx = canvas.getContext('2d');
        const W = canvas.width, H = canvas.height;
        ctx.clearRect(0, 0, W, H);
        if (!t || t.p95.length < 2) return;
        let hi = 0;
        for (const v of t.p95) if (v > hi) hi = v;
        if (hi <= 0) hi = 1;
        const n = t.p95.length;
        const x = i => (i / (n - 1)) * (W - 1);
        const y = v => H - 1 - (v / hi) * (H - 2);
        ctx.beginPath();
        for (let i = 0; i < n; i++) { const px = x(i), py = y(t.p95[i]); i ? ctx.lineTo(px, py) : ctx.moveTo(px, py); }
        ctx.strokeStyle = '#fb4'; ctx.lineWidth = 1; ctx.stroke();
    }

    const esc = s => s.replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
    // parse "a\tb\tc\n..." into an array of field arrays
    const rows = blob => blob.split('\n').filter(l => l.length).map(l => l.split('\t'));

    $('btnLogClear').addEventListener('click', () => {
        con.innerHTML = ''; totalDropped = 0; dropped.textContent = ''; peek.textContent = '';
    });

    // persistent log dock: collapse toggle (state remembered). Keep the body
    // padded by the dock's height so the fixed dock never covers content.
    const logDock = $('logDock'), logDockToggle = $('logDockToggle');
    // The dock's height is published as --dock-h; the body padding and the
    // full-height views both derive from it (see the :root block in index.html).
    function padBodyForDock() { publishChromeMetrics(); }
    function setDockCollapsed(collapsed) {
        logDock.classList.toggle('collapsed', collapsed);
        logDockToggle.textContent = collapsed ? 'Expand' : 'Collapse';
        try { localStorage.setItem('cds.logDock.collapsed', collapsed ? '1' : '0'); } catch (e) {}
        padBodyForDock();
    }
    logDockToggle.addEventListener('click',
        () => setDockCollapsed(!logDock.classList.contains('collapsed')));
    let dockCollapsed0 = false;
    try { dockCollapsed0 = localStorage.getItem('cds.logDock.collapsed') === '1'; } catch (e) {}
    setDockCollapsed(dockCollapsed0);
    window.addEventListener('resize', padBodyForDock);

    // profiler reset + server-side file toggles
    $('btnResetProfile').addEventListener('click', () => {
        if (!sim) return;
        sim.ext_resetProfiler();
        trends.clear();          // restart the sparkline history too
        refreshProfileStats();
    });
    const chkRawCsv = $('chkRawCsv'), chkLogFile = $('chkLogFile');
    function pushFileToggles() {
        if (sim) sim.ext_setDiagFiles({ logFile: chkLogFile.checked, profileRaw: chkRawCsv.checked });
    }
    chkRawCsv.addEventListener('change', pushFileToggles);
    chkLogFile.addEventListener('change', pushFileToggles);

    // set the level of EVERY log module at once (keeps each module's sampling N)
    $('logLevelAll').addEventListener('change', (e) => {
        const level = e.target.value;
        if (!sim || level === '') return;
        logTbody.querySelectorAll('tr').forEach(row => {
            const modEl = row.querySelector('[data-mod]');
            if (!modEl) return;
            const n = Math.max(1, Number(row.querySelector('.n').value) || 1);
            sim.ext_setLogLevel({ module: Number(modEl.dataset.mod), level: Number(level), sampleN: n });
        });
        e.target.value = '';      // reset the picker
        refreshLogModules();       // reflect the new per-module levels
    });

    // Server-side file sinks only work when a native server is running (ws-served).
    // In the in-browser build (wasm-only) there is no filesystem, so their toggles
    // are disabled — like the plant checkbox when no plant is connected.
    let fileSinksAvailable = true;
    function setFileSinksAvailable(available) {
        fileSinksAvailable = available;
        for (const el of [chkLogFile, chkRawCsv]) {
            el.disabled = !available;
            const row = el.closest('.file-toggle');
            if (row) row.classList.toggle('unavailable', !available);
        }
        if (!available) {
            chkRecord.disabled = true;
            chkRecord.closest('.file-toggle')?.classList.add('unavailable');
        }
    }

    // data recorder: toggle + live status (active model, dropped rows). The
    // status struct carries flags as numbers (0/1) and modelName as a string.
    const chkRecord = $('chkRecord'), recordStatus = $('recordStatus'), recordDropped = $('recordDropped');
    function applyRecordStatus(s) {
        if (!s) return;
        const active = Number(s.active) >= 1;
        chkRecord.disabled = !active || !fileSinksAvailable;
        chkRecord.checked = Number(s.enabled) >= 1;
        recordStatus.textContent = !active ? 'no model running'
            : chkRecord.checked ? `recording: ${s.modelName}`
            : `idle: ${s.modelName}`;
        recordStatus.classList.toggle('rec-on', active && chkRecord.checked);
        const drop = Number(s.droppedRows) || 0;
        recordDropped.textContent = drop > 0 ? `${drop} rows dropped` : '';
    }
    function refreshRecordStatus() { if (sim) applyRecordStatus(sim.ext_getRecordStatus()); }
    chkRecord.addEventListener('change', () => {
        if (sim) applyRecordStatus(sim.ext_setRecording({ enabled: chkRecord.checked }));
    });

    function appendLog(batch) {
        if (batch.dropped > 0) {
            totalDropped += batch.dropped;
            dropped.textContent = `${totalDropped} dropped`;
        }
        let lines = rows(batch.lines);
        if (!lines.length) return;
        // a flooded batch can be hundreds of lines; only the most recent
        // MAX_LINES can survive the trim, so never build more than that
        if (lines.length > MAX_LINES) lines = lines.slice(-MAX_LINES);
        // build ONE html string for the whole batch: a single parse, instead of
        // createElement + innerHTML per line (that is what froze the UI)
        let html = '';
        for (const [ts, lvl, mod, ...rest] of lines) {
            html += `<span class="ln"><span class="ts">${esc(ts || '')}</span> ` +
                    `<span class="lv lv-${esc(lvl)}">${esc(lvl)}</span>` +
                    `<span class="mod">${esc(mod || '')}</span> ${esc(rest.join('\t'))}</span>`;
        }
        con.insertAdjacentHTML('beforeend', html);
        let over = con.childElementCount - MAX_LINES;
        while (over-- > 0) con.removeChild(con.firstChild);
        if (autoscroll.checked) con.scrollTop = con.scrollHeight;

        // peek: newest line shown in the bar while the dock is collapsed
        const last = lines[lines.length - 1];
        if (last) {
            const [, lvl, mod, ...rest] = last;   // drop the timestamp for the compact peek
            peek.innerHTML = `<span class="lv lv-${esc(lvl)}">${esc(lvl)}</span>` +
                             `<span class="mod">${esc(mod || '')}</span> ${esc(rest.join('\t'))}`;
        }
    }

    function refreshLogModules() {
        const list = rows(sim.ext_getLogModules().list);
        logEmpty.style.display = list.length ? 'none' : '';
        logTbody.innerHTML = list.map(([idx, name, level, sampleN]) => {
            const opts = LEVELS.map((n, i) => `<option value="${i}" ${i === Number(level) ? 'selected' : ''}>${n}</option>`).join('');
            return `<tr><td>${esc(name)}</td>` +
                   `<td><select class="lvl" data-mod="${idx}">${opts}</select></td>` +
                   `<td><input type="number" min="1" step="1" class="n" data-mod="${idx}" value="${esc(sampleN)}"></td></tr>`;
        }).join('');
    }

    function refreshProfileModules() {
        const list = rows(sim.ext_getProfilerModules().list);
        profEmpty.style.display = list.length ? 'none' : '';
        profTbody.innerHTML = list.map(([idx, name, enabled]) =>
            `<tr><td>${esc(name)}</td>` +
            `<td><input type="checkbox" class="en" data-mod="${idx}" ${enabled === '1' ? 'checked' : ''}></td></tr>`
        ).join('');
    }

    function refreshProfileStats() {
        const list = rows(sim.ext_getProfilerTable().table);
        statsEmpty.style.display = list.length ? 'none' : '';
        statsTbody.innerHTML = list.map(([mod, scope, kind, count, mean, std, min, max, p50, p95, p99]) => {
            const key = mod + '\t' + scope;
            pushTrend(key, Number(p95) || 0);
            const num = v => `<td class="num">${esc(v ?? '')}</td>`;
            return `<tr><td>${esc(mod)}</td><td>${esc(scope)}</td><td>${esc(kind)}</td>` +
                   num(count) + num(mean) + num(std) + num(min) + num(max) + num(p50) + num(p95) + num(p99) +
                   `<td><canvas class="spark" width="90" height="22" data-key="${esc(key)}"></canvas></td></tr>`;
        }).join('');
        // rows were just rebuilt: redraw each sparkline from its accumulated series
        statsTbody.querySelectorAll('canvas.spark').forEach(c => drawSpark(c, trends.get(c.dataset.key)));
    }

    // delegated handlers: rows are rebuilt, so bind once on the tables
    logTable.addEventListener('change', (e) => {
        const row = e.target.closest('tr');
        if (!row || !sim) return;
        sim.ext_setLogLevel({
            module: Number(e.target.dataset.mod ?? row.querySelector('[data-mod]').dataset.mod),
            level: Number(row.querySelector('.lvl').value),
            sampleN: Math.max(1, Number(row.querySelector('.n').value) || 1),
        });
    });
    profTable.addEventListener('change', (e) => {
        if (!sim || !e.target.classList.contains('en')) return;
        sim.ext_setProfilerEnabled({ module: Number(e.target.dataset.mod), enabled: e.target.checked });
    });

    return {
        setFileSinksAvailable,
        onShow() {
            if (!sim) return;
            try {
                refreshLogModules();
                refreshProfileModules();
                refreshProfileStats();
                refreshRecordStatus();
            } catch (e) { /* never let the Diag view break the app */ }
        },
        // the log stream drains continuously on every view (the dock is always
        // present); throttled to its own cadence so it never runs at frame rate
        pollLog() {
            if (!sim) return;
            const now = performance.now();
            if (now - lastLogPoll < LOG_POLL_MS) return;
            lastLogPoll = now;
            try {
                const batch = timedCall(() => sim.ext_getLogBatch());
                if (!lastCallStalled) appendLog(batch);
            } catch (e) { /* diagnostics must never wedge the render loop */ }
        },
        // profiler stats + recorder status: only while the Diag view is up
        pollStats() {
            if (!sim) return;
            const now = performance.now();
            if (now - lastStatsPoll < STATS_POLL_MS) return;
            lastStatsPoll = now;
            try {
                refreshProfileStats();
                refreshRecordStatus();
            } catch (e) { /* never wedge the render loop */ }
        },
    };
})();

function showView(name) {
    ui.viewCharts.style.display = name === 'charts' ? 'grid'  : 'none';
    ui.view3d.style.display     = name === '3d'     ? 'block' : 'none';
    ui.viewParams.style.display = name === 'params' ? 'block' : 'none';
    ui.viewDiag.style.display   = name === 'diag'   ? 'block' : 'none';
    ui.btnCharts.classList.toggle('active', name === 'charts');
    ui.btn3d.classList.toggle('active',     name === '3d');
    ui.btnParams.classList.toggle('active', name === 'params');
    ui.btnDiag.classList.toggle('active',   name === 'diag');
    if (name === '3d')     renderer3d.show();
    else                   renderer3d.hide();
    if (name === 'charts') chartsRenderer?.redraw();   // show the buffered history at once
    if (name === 'diag')   diagnostics.onShow();
    if (name === 'params') refreshControllerPanel();
}

ui.btnCharts.addEventListener('click', () => showView('charts'));
ui.btn3d.addEventListener('click',     () => showView('3d'));
ui.btnParams.addEventListener('click', () => showView('params'));
ui.btnDiag.addEventListener('click',   () => showView('diag'));

// =============================================================================
// Model selection (rocket / quadrotor)
// =============================================================================
function applyModelPanelVisibility() {
    const isQuad = isQuadFamily(currentModel);
    if (ui.panelRocket) ui.panelRocket.style.display = isQuad ? 'none' : '';
    if (ui.panelQuad)   ui.panelQuad.style.display   = isQuad ? '' : 'none';
}

// Switch the active dynamic model. Stops the sim, re-inits the core with the
// new model's default params, and replays the (shared) trajectory so the run
// stays valid. The trajectory sequence itself is model-agnostic and preserved.
function switchModel(model) {
    if (!isRocketFamily(model) && !isQuadFamily(model)) return;
    stop();
    currentModel = model;

    // Load that model's default params into its form and show only its panel.
    fillParamsForm(DEFAULT_INIT_PARAMS[model]);
    applyModelPanelVisibility();

    const err = initBackend(readParamsForm());
    if (err) { setError(`init failed switching to ${model}`); return; }

    // Init wipes the backend trajectory; replay the JS-side sequence.
    trajectoryBuilder.replayToBackend();
    renderer3d?.rebuildVehicle?.();
    renderer3d?.invalidateTrajectory?.();
    refreshControllerPanel();   // the new model exposes its own controller params

    renderers.forEach(r => r.reset());
    simTime = 0; stepCount = 0;
    ui.simTime.innerHTML = `0.000 <span>s</span>`;
    setStatus(`Ready — model: ${model}.`);
    setError('');
    ui.btnReset.disabled = true;
    refreshStartEnabled();
}

ui.modelSelect.addEventListener('change', () => switchModel(ui.modelSelect.value));

// Apply rocket / actuator params + timestep (also re-inits the core).
ui.btnApply.addEventListener('click', () => {
    const params = readParamsForm();
    timestep_s = readTimestep();
    stop();
    const savedCtrl = snapshotControllerParams();   // keep the controller tuning across re-init
    const err = initBackend(params);
    if (err) { setError('init failed'); return; }

    // Init wipes the backend trajectory; replay the JS-side sequence.
    trajectoryBuilder.replayToBackend();
    renderer3d?.invalidateTrajectory?.();
    refreshControllerPanel();
    applyControllerParams(savedCtrl);
    refreshControllerPanel();

    renderers.forEach(r => r.reset());
    simTime = 0; stepCount = 0;
    ui.simTime.innerHTML = `0.000 <span>s</span>`;
    setStatus('Ready — params applied.');
    setError('');
    ui.btnReset.disabled = true;
    refreshStartEnabled();
});

// Trajectory changes invalidate the preview and may flip the Start button.
trajectoryBuilder.onChange(() => {
    renderer3d?.invalidateTrajectory?.();
    refreshStartEnabled();
});

// =============================================================================
// Trajectory save / load (JSON)
// =============================================================================
//
// The saved file carries only the trajectory sequence (the JS spec list), which
// is model-agnostic: the same file loads under Rocket or QuadRotor. A small
// header (schema + version) lets the loader reject unrelated JSON and lets the
// format evolve later.
//
const TRAJ_FILE_SCHEMA  = 'cds-trajectory';
const TRAJ_FILE_VERSION = 1;

function isFiniteNum(v) { return typeof v === 'number' && isFinite(v); }
function isVec3(v) {
    return v && typeof v === 'object' &&
        isFiniteNum(v.x) && isFiniteNum(v.y) && isFiniteNum(v.z);
}

function validateTrajItem(it, i) {
    // Returns an error string, or null when the item is well-formed. Mirrors the
    // two item shapes documented on the trajectory builder.
    if (!it || typeof it !== 'object') return `item ${i}: not an object`;
    const p = it.params;
    if (!p || typeof p !== 'object') return `item ${i}: missing params`;
    if (!isFiniteNum(p.time_s) || p.time_s <= 0) return `item ${i}: time_s must be > 0`;
    if (it.kind === 'poly4') {
        for (const k of ['initialPos', 'initialVel', 'finalPos', 'finalVel', 'finalAcc'])
            if (!isVec3(p[k])) return `item ${i}: ${k} must be {x,y,z}`;
        for (const k of ['initialYaw', 'initialYawRate', 'finalYaw', 'finalYawRate', 'finalYawAcc'])
            if (!isFiniteNum(p[k])) return `item ${i}: ${k} must be a number`;
        return null;
    }
    if (it.kind === 'point') {
        if (!isVec3(p.finalPos)) return `item ${i}: finalPos must be {x,y,z}`;
        if (!isFiniteNum(p.finalYaw)) return `item ${i}: finalYaw must be a number`;
        return null;
    }
    return `item ${i}: unknown kind "${it.kind}"`;
}

function parseTrajectoryFile(text) {
    // Returns the normalized item list, or throws Error with a user-facing
    // message. Normalization keeps only the exact fields the embind structs
    // expect, so stray properties in the file can never reach the backend.
    let doc;
    try { doc = JSON.parse(text); }
    catch { throw new Error('not valid JSON'); }
    if (!doc || typeof doc !== 'object' || Array.isArray(doc))
        throw new Error('root must be an object');
    if (doc.schema !== TRAJ_FILE_SCHEMA)
        throw new Error(`unexpected schema (want "${TRAJ_FILE_SCHEMA}")`);
    if (doc.version !== TRAJ_FILE_VERSION)
        throw new Error(`unsupported version ${doc.version} (want ${TRAJ_FILE_VERSION})`);
    if (!Array.isArray(doc.items)) throw new Error('items must be an array');
    if (doc.items.length === 0)    throw new Error('items is empty');
    doc.items.forEach((it, i) => {
        const msg = validateTrajItem(it, i);
        if (msg) throw new Error(msg);
    });
    const vec3 = (v) => ({ x: v.x, y: v.y, z: v.z });
    return doc.items.map((it) => {
        const p = it.params;
        if (it.kind === 'poly4') {
            return { kind: 'poly4', params: {
                initialPos:     vec3(p.initialPos),
                initialYaw:     p.initialYaw,
                initialVel:     vec3(p.initialVel),
                initialYawRate: p.initialYawRate,
                finalPos:       vec3(p.finalPos),
                finalYaw:       p.finalYaw,
                finalVel:       vec3(p.finalVel),
                finalYawRate:   p.finalYawRate,
                finalAcc:       vec3(p.finalAcc),
                finalYawAcc:    p.finalYawAcc,
                time_s:         p.time_s,
            } };
        }
        return { kind: 'point', params: {
            finalPos: vec3(p.finalPos),
            finalYaw: p.finalYaw,
            time_s:   p.time_s,
        } };
    });
}

function timestampSlug() {
    const d = new Date();
    const p = (n) => String(n).padStart(2, '0');
    return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}`
         + `-${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
}

function saveTrajectory() {
    const items = trajectoryBuilder.getItems();
    if (items.length === 0) {
        setError('Nothing to save: trajectory sequence is empty.');
        return;
    }
    const doc = { schema: TRAJ_FILE_SCHEMA, version: TRAJ_FILE_VERSION, items };
    const blob = new Blob([JSON.stringify(doc, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `trajectory-${timestampSlug()}.json`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
    setError('');
}

function loadTrajectoryFromFile(file) {
    if (running) {   // defensive: the button is gated, but never replay live
        setError('Stop the simulation before loading a trajectory.');
        return;
    }
    const reader = new FileReader();
    reader.onload = () => {
        let items;
        // Parse + validate fully before touching the current sequence, so a bad
        // file never destroys the trajectory the user already has.
        try { items = parseTrajectoryFile(String(reader.result)); }
        catch (e) {
            setError(`Trajectory load failed: ${e.message}`);
            return;
        }
        if (!trajectoryBuilder.replaceItems(items)) return;   // replaceItems set the error
        renderer3d?.invalidateTrajectory?.();
        setStatus(`Trajectory loaded (${items.length} item${items.length === 1 ? '' : 's'}).`);
        setError('');
    };
    reader.onerror = () => setError('Trajectory load failed: could not read file.');
    reader.readAsText(file);
}

ui.btnTrajSave.addEventListener('click', saveTrajectory);
ui.btnTrajLoad.addEventListener('click', () => ui.trajFileInput.click());
ui.trajFileInput.addEventListener('change', (e) => {
    const file = e.target.files && e.target.files[0];
    if (file) loadTrajectoryFromFile(file);
    e.target.value = '';   // reset so re-selecting the same file fires 'change'
});

// =============================================================================
// Controller parameters (data-driven panel + JSON save / load)
// =============================================================================
//
// The active model describes its tunable and observed parameters through four
// per-domain manifests -- controller / model / observer / sensor -- each a TSV
// listing (id, group, label, flags, value). The panel is built entirely from the
// concatenation of those manifests (no model-specific UI code); each writable
// field is committed one at a time through the matching domain's setter. The JSON
// file mirrors the trajectory's: a schema+version header, then the writable
// parameters keyed by (group, label) so a file loads onto the matching model.
//
const CTRL_FILE_SCHEMA  = 'cds-controller';
const CTRL_FILE_VERSION = 1;

// The four parameter domains, in panel order. Each has a manifest getter and a
// set-by-id setter; the ids are per-domain (each manifest starts at 0), so a row
// is identified by (domain, id).
const PARAM_DOMAINS = [
    { key: 'controller', get: () => sim.ext_controllerGetManifest().text, set: (a) => sim.ext_controllerSetParam(a) },
    { key: 'model',      get: () => sim.ext_modelGetManifest().text,      set: (a) => sim.ext_modelSetParam(a) },
    { key: 'observer',   get: () => sim.ext_observerGetManifest().text,   set: (a) => sim.ext_observerSetParam(a) },
    { key: 'sensor',     get: () => sim.ext_sensorGetManifest().text,     set: (a) => sim.ext_sensorSetParam(a) },
];

// Commit one parameter to its domain's setter. Returns true on error/reject.
function setParamByDomain(domain, id, value) {
    const d = PARAM_DOMAINS.find(x => x.key === domain);
    return d ? d.set({ id, value }) : true;   // true = error (bool-is-error)
}

let controllerManifest = [];   // [{ domain, id, group, label, writable, value }]

function parseControllerManifest(text, domain) {
    return text.split('\n').filter(l => l.length).map(l => {
        const [id, group, label, flags, value] = l.split('\t');
        return { domain, id: Number(id), group, label, writable: flags === 'rw', value: Number(value) };
    });
}

// A writable knob whose label carries the "(0|1)" convention is a boolean flag
// (observer / sensor enables): render it as a checkbox rather than a number box.
function isBooleanParam(p) { return p.writable && /\(0\|1\)/.test(p.label); }
function paramDisplayLabel(p) { return p.label.replace(/\s*\(0\|1\)\s*/, ''); }

// The observer/sensor domains are demarcated into their own section so they read
// as a sensor panel, not controller tuning.
function isEstimatorGroup(g) { return g === 'Observer' || /^Sensor /.test(g); }

function fetchControllerManifest() {
    if (!sim) return [];
    const out = [];
    for (const d of PARAM_DOMAINS) {
        try { for (const row of parseControllerManifest(d.get(), d.key)) out.push(row); }
        catch { /* a domain a model does not expose returns an empty manifest */ }
    }
    return out;
}

function renderControllerPanel() {
    const box = ui.ctrlParams;
    box.innerHTML = '';
    if (controllerManifest.length === 0) {
        ui.ctrlParamsInfo.textContent = sim
            ? 'This controller exposes no tunable parameters.'
            : 'Initialize a model to load its controller parameters.';
        ui.btnCtrlSave.disabled = true;
        ui.btnCtrlLoad.disabled = !sim;
        return;
    }
    ui.ctrlParamsInfo.textContent = 'Edit a value to retune the controller live. Read-only rows are derived.';
    ui.btnCtrlSave.disabled = false;
    ui.btnCtrlLoad.disabled = false;

    let lastGroup = null;
    let sensorSectionStarted = false;
    for (const p of controllerManifest) {
        if (!sensorSectionStarted && isEstimatorGroup(p.group)) {
            const sec = document.createElement('div');
            sec.className = 'ctrl-section';
            sec.textContent = 'Estimator & sensors';
            box.appendChild(sec);
            sensorSectionStarted = true;
        }
        if (p.group !== lastGroup) {
            const h = document.createElement('div');
            h.className = 'ctrl-group';
            h.textContent = p.group;
            box.appendChild(h);
            lastGroup = p.group;
        }
        const row = document.createElement('div');
        row.className = 'ctrl-row' + (p.writable ? '' : ' ro');
        const lab = document.createElement('label');
        const inp = document.createElement('input');
        inp.dataset.id = String(p.id);
        inp.dataset.domain = p.domain;
        if (isBooleanParam(p)) {
            lab.textContent = paramDisplayLabel(p);
            inp.type = 'checkbox';
            inp.checked = p.value >= 0.5;
            inp.dataset.bool = '1';
            row.appendChild(lab);
            row.appendChild(inp);
            inp.addEventListener('change', onControllerToggleChange);
        } else {
            lab.textContent = p.label;
            inp.type = 'number'; inp.step = 'any'; inp.value = String(p.value);
            row.appendChild(lab);
            row.appendChild(inp);
            if (p.writable) {
                inp.addEventListener('change', onControllerInputChange);
            } else {
                inp.readOnly = true;
                const tag = document.createElement('span');
                tag.className = 'ro-tag'; tag.textContent = 'read-only';
                row.appendChild(tag);
            }
        }
        row.dataset.group = p.group;
        row.dataset.label = p.label;
        box.appendChild(row);
    }
    applySensorDimming();
}

// "prediction only" means that axis publishes nothing, so its bias and noise
// describe a measurement that is not being taken: grey them out (and refuse the
// edit) rather than leave two live boxes that change nothing.
function applySensorDimming() {
    const rows = [...ui.ctrlParams.querySelectorAll('.ctrl-row[data-group]')];
    const offGroups = new Set(rows
        .filter(r => /^prediction only/.test(r.dataset.label) && r.querySelector('input')?.checked)
        .map(r => r.dataset.group));

    for (const r of rows) {
        if (/^prediction only/.test(r.dataset.label)) continue;
        if (!/^Sensor /.test(r.dataset.group)) continue;
        const off = offGroups.has(r.dataset.group);
        r.classList.toggle('dimmed', off);
        const inp = r.querySelector('input');
        if (inp) inp.disabled = off;
    }
}

function refreshControllerPanel() {
    controllerManifest = fetchControllerManifest();
    renderControllerPanel();
}

function syncControllerValues() {
    // Refresh values in place (reflect clamping and updated read-only rows)
    // without rebuilding the DOM, so the field the user just edited keeps focus.
    controllerManifest = fetchControllerManifest();
    const byId = new Map(controllerManifest.map(p => [`${p.domain}:${p.id}`, p.value]));
    ui.ctrlParams.querySelectorAll('input[data-id]').forEach(inp => {
        if (document.activeElement === inp) return;
        const v = byId.get(`${inp.dataset.domain}:${Number(inp.dataset.id)}`);
        if (v === undefined) return;
        if (inp.dataset.bool === '1') inp.checked = v >= 0.5;
        else                          inp.value = String(v);
    });
}

function onControllerInputChange(e) {
    const id = Number(e.target.dataset.id);
    const domain = e.target.dataset.domain;
    const value = Number(e.target.value);
    const label = e.target.previousSibling ? e.target.previousSibling.textContent : `id ${id}`;
    if (!isFiniteNum(value)) { setError(`"${label}" must be a number.`); return; }
    if (setParamByDomain(domain, id, value)) {
        // Rejected: an invalid value (e.g. a negative weight) or the controller
        // could not re-synthesise its gain for these settings. The gain is kept.
        setError(`"${label}" = ${value} rejected — invalid value or the controller could not re-synthesise (gain unchanged). See the Diag log.`);
    } else {
        setError('');
    }
    syncControllerValues();
}

// Boolean knob (observer / sensor enable): commit as 1 / 0. On rejection revert
// the checkbox to its prior state so the UI reflects what the backend accepted.
function onControllerToggleChange(e) {
    const id = Number(e.target.dataset.id);
    const domain = e.target.dataset.domain;
    const value = e.target.checked ? 1 : 0;
    const label = e.target.previousSibling ? e.target.previousSibling.textContent : `id ${id}`;
    if (setParamByDomain(domain, id, value)) {
        setError(`"${label}" toggle rejected. See the Diag log.`);
        e.target.checked = !e.target.checked;
    } else {
        setError('');
    }
    syncControllerValues();
    applySensorDimming();
}

function saveControllerParams() {
    if (controllerManifest.length === 0) { setError('No controller parameters to save.'); return; }
    const params = controllerManifest
        .filter(p => p.writable)
        .map(p => ({ group: p.group, label: p.label, value: p.value }));
    const doc = { schema: CTRL_FILE_SCHEMA, version: CTRL_FILE_VERSION, model: currentModel, params };
    const blob = new Blob([JSON.stringify(doc, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `controller-${currentModel}-${timestampSlug()}.json`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
    setError('');
}

function parseControllerFile(text) {
    let doc;
    try { doc = JSON.parse(text); }
    catch { throw new Error('not valid JSON'); }
    if (!doc || typeof doc !== 'object' || Array.isArray(doc)) throw new Error('root must be an object');
    if (doc.schema !== CTRL_FILE_SCHEMA)   throw new Error(`unexpected schema (want "${CTRL_FILE_SCHEMA}")`);
    if (doc.version !== CTRL_FILE_VERSION)  throw new Error(`unsupported version ${doc.version} (want ${CTRL_FILE_VERSION})`);
    if (!Array.isArray(doc.params))         throw new Error('params must be an array');
    doc.params.forEach((p, i) => {
        if (!p || typeof p !== 'object')                              throw new Error(`param ${i}: not an object`);
        if (typeof p.group !== 'string' || typeof p.label !== 'string') throw new Error(`param ${i}: missing group/label`);
        if (!isFiniteNum(p.value))                                    throw new Error(`param ${i}: value must be a number`);
    });
    return doc.params;
}

// Snapshot the current writable parameters (to carry across a re-init).
function snapshotControllerParams() {
    return controllerManifest.filter(p => p.writable)
        .map(p => ({ group: p.group, label: p.label, value: p.value }));
}

// Apply a list of {group, label, value} onto the current (live) controller,
// matching by (group, label) so entries that don't fit are skipped rather than
// failing the batch. Returns { applied, skipped, rejected }.
function applyControllerParams(params) {
    const rowOf = new Map(controllerManifest.filter(p => p.writable).map(p => [`${p.group}\t${p.label}`, p]));
    let applied = 0, skipped = 0, rejected = 0;
    for (const p of params) {
        const row = rowOf.get(`${p.group}\t${p.label}`);
        if (row === undefined) { skipped++; continue; }
        if (setParamByDomain(row.domain, row.id, p.value)) rejected++; else applied++;
    }
    return { applied, skipped, rejected };
}

function loadControllerParamsFromFile(file) {
    if (!sim || controllerManifest.length === 0) {
        setError('Initialize a model before loading controller parameters.');
        return;
    }
    const reader = new FileReader();
    reader.onload = () => {
        let params;
        try { params = parseControllerFile(String(reader.result)); }
        catch (e) { setError(`Controller load failed: ${e.message}`); return; }
        const { applied, skipped, rejected } = applyControllerParams(params);
        refreshControllerPanel();
        setStatus(`Controller parameters: ${applied} applied`
                  + (skipped ? `, ${skipped} skipped` : '')
                  + (rejected ? `, ${rejected} rejected` : '') + '.');
        setError(rejected ? 'Some parameters were rejected by the controller.' : '');
    };
    reader.onerror = () => setError('Controller load failed: could not read file.');
    reader.readAsText(file);
}

ui.btnCtrlSave.addEventListener('click', saveControllerParams);
ui.btnCtrlLoad.addEventListener('click', () => ui.ctrlFileInput.click());
ui.ctrlFileInput.addEventListener('change', (e) => {
    const file = e.target.files && e.target.files[0];
    if (file) loadControllerParamsFromFile(file);
    e.target.value = '';
});

// =============================================================================
// Boot
// =============================================================================
(async () => {
    try {
        sim = await createSimulator();

        // ws-served spins up the WebSocket bridge (globalThis.__cdsWs) in its
        // transport pre-js; the wasm-only build has no server and no filesystem,
        // so its server-side file-sink toggles are disabled.
        diagnostics.setFileSinksAvailable(!!globalThis.__cdsWs);

        // Register renderers
        chartsRenderer = makeChartsRenderer();
        renderers.push(chartsRenderer);

        initControlsPanel();            // secondary bar: collapsed state + toggle
        trackChromeSize();              // publish --chrome-h / --dock-h for the views
        renderer3d = make3DRenderer();
        renderers.push(renderer3d);
        ui.btn3d.disabled = false;
        showView('3d');                 // home view (the CSS starts on it too)

        // Reflect the default model in the selector and show its params panel.
        currentModel = ui.modelSelect ? ui.modelSelect.value : MODEL_ROCKET;
        applyModelPanelVisibility();

        // The backend outlives page reloads (ws-served): a previous session
        // may have left it running, and init is refused while running.
        sim.ext_stop();

        const err = initBackend(DEFAULT_INIT_PARAMS[currentModel]);
        if (err) {
            setError('init failed');
            return;
        }

        fillParamsForm(DEFAULT_INIT_PARAMS[currentModel]);
        // Inject a default sequence so a fresh user can press Start straight
        // away. Done after re-init so the backend trajectory is empty.
        // loadPreset triggers refreshAutofill internally, so initialPos /
        // initialVel of the form will reflect the end of the default item.
        trajectoryBuilder.loadPreset(DEFAULT_TRAJECTORY);
        setupForceButtons();
        refreshControllerPanel();       // load the initial controller's parameters
        setStatus('Ready.');
        refreshStartEnabled();
        ui.btnReset.disabled = false;   // reset is safe whenever not running

        // Start the continuous poll loop: it keeps the plant ghost live even
        // before the simulation runs, so staging is visible.
        loop();
    } catch (e) {
        setError(`Failed to load WASM: ${e.message}`);
    }
})();