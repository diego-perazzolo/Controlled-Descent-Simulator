import createSimulator from '../build/simulator.js';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// =============================================================================
// Config
// =============================================================================
const TIMESTEP_S = 0.001;

const INIT_PARAMS = {
    rocketPar: {
        mass_Kg:        10.0,
        inertiaX_Kgm2: 10.0 / 3,
        inertiaY_Kgm2: 10.0 / 3,
        inertiaZ_Kgm2: 1,
        c:              10,
        cz:             0.02,
    },
    actuatorLimits: {
        fZ_lim: 700.0,
    },
    trajParams: {
        a0: 0.0,
        a1: 0.0,
        a2: 0.0,
        a3: 0.0,
    },
};

// =============================================================================
// Simulation state
// =============================================================================
let sim        = null;
let renderer3d = null;
let running    = false;
let frameId    = null;
let simTime    = 0;
let stepCount  = 0;
let lastFpsTs  = performance.now();
let fpsCounter = 0;
let fpsDisplay = 0;

// =============================================================================
// Renderers registry — add a renderer here to hook into the sim loop
// Each renderer is { update(state, err, simTime, stepCount), reset() }
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
    xErr:       $('xErr'),   yErr:   $('yErr'),   zErr:   $('zErr'),   err_mag: $('err_mag'),
    stepCount:  $('stepCount'), dt: $('dt'), fps: $('fps'),
    btnStart:   $('btnStart'), btnStop: $('btnStop'), btnReset: $('btnReset'),
    btnCharts:  $('btnCharts'), btn3d: $('btn3d'), btnParams: $('btnParams'),
    viewCharts: $('view-charts'), view3d: $('view-3d'), viewParams: $('view-params'),
    status:     $('statusBar'), error: $('errorMsg'),
    btnApply:   $('btnApply'),
    p_mass: $('p_mass'), p_iX: $('p_iX'), p_iY: $('p_iY'), p_iZ: $('p_iZ'),
    p_c:    $('p_c'),    p_cz: $('p_cz'),
    p_fZlim: $('p_fZlim'),
    p_a0: $('p_a0'), p_a1: $('p_a1'), p_a2: $('p_a2'), p_a3: $('p_a3'),
};

// =============================================================================
// User force state — updated by force buttons
// =============================================================================
const userForce = { fX: 0, fY: 0, fZ: 0 };

function setupForceButtons() {
    document.querySelectorAll('.btn-force').forEach(btn => {
        const axis = btn.dataset.axis;          // 'fX' | 'fY' | 'fZ'
        const sign = parseFloat(btn.dataset.sign);

        const press = () => {
            const mag = parseFloat($('forceMag').value) || 0;
            userForce[axis] = sign * mag;
            btn.classList.add('pressing');
        };
        const release = () => {
            userForce[axis] = 0;
            btn.classList.remove('pressing');
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
function fillParamsForm(p) {
    ui.p_mass.value  = p.rocketPar.mass_Kg;
    ui.p_iX.value    = p.rocketPar.inertiaX_Kgm2;
    ui.p_iY.value    = p.rocketPar.inertiaY_Kgm2;
    ui.p_iZ.value    = p.rocketPar.inertiaZ_Kgm2;
    ui.p_c.value     = p.rocketPar.c;
    ui.p_cz.value    = p.rocketPar.cz;
    ui.p_fZlim.value = p.actuatorLimits.fZ_lim;
    ui.p_a0.value    = p.trajParams.a0;
    ui.p_a1.value    = p.trajParams.a1;
    ui.p_a2.value    = p.trajParams.a2;
    ui.p_a3.value    = p.trajParams.a3;
}

function readParamsForm() {
    const n = id => parseFloat($(id).value) || 0;
    return {
        rocketPar: {
            mass_Kg:        n('p_mass'),
            inertiaX_Kgm2: n('p_iX'),
            inertiaY_Kgm2: n('p_iY'),
            inertiaZ_Kgm2: n('p_iZ'),
            c:              n('p_c'),
            cz:             n('p_cz'),
        },
        actuatorLimits: { fZ_lim: n('p_fZlim') },
        trajParams:     { a0: n('p_a0'), a1: n('p_a1'), a2: n('p_a2'), a3: n('p_a3') },
    };
}

const fmt  = (v, d = 4) => v.toFixed(d);
const fmtI = v => Math.round(v).toString();

function setStatus(msg) { ui.status.textContent = msg; }
function setError(msg)  { ui.error.textContent  = msg; }

// =============================================================================
// 3D renderer
// =============================================================================
function make3DRenderer() {
    const TRAIL_MAX = 600;
    const trail     = [];

    let scene, camera, renderer, controls;
    let rocketGroup, trailLine;
    let initialized = false;
    let animating   = false;
    let visible     = false;

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
            leg.rotation.x = Math.sin(angle) * 0.35;
            group.add(leg);
        }

        return group;
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
        camera.position.set(40, 25, 40);

        // OrbitControls
        controls = new OrbitControls(camera, renderer.domElement);
        controls.enableDamping  = true;
        controls.dampingFactor  = 0.06;
        controls.minDistance    = 5;
        controls.maxDistance    = 1000;

        // Lights
        scene.add(new THREE.AmbientLight(0xffffff, 0.45));
        const sun = new THREE.DirectionalLight(0xffffff, 1.1);
        sun.position.set(200, 400, 150);
        scene.add(sun);

        // Ground grid
        scene.add(new THREE.GridHelper(600, 60, 0x2a2a2a, 0x1a1a1a));

        // Rocket
        rocketGroup = buildRocket();
        scene.add(rocketGroup);

        // Trail line
        trailLine = new THREE.Line(
            new THREE.BufferGeometry(),
            new THREE.LineBasicMaterial({ color: 0x0099ff, transparent: true, opacity: 0.55 })
        );
        scene.add(trailLine);

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

    function updateTrail(x, y, z) {
        trail.push(new THREE.Vector3(x, y, z));
        if (trail.length > TRAIL_MAX) trail.shift();

        const pos = new Float32Array(trail.length * 3);
        trail.forEach((v, i) => {
            pos[i * 3]     = v.x;
            pos[i * 3 + 1] = v.y;
            pos[i * 3 + 2] = v.z;
        });
        trailLine.geometry.setAttribute(
            'position', new THREE.BufferAttribute(pos, 3)
        );
        trailLine.geometry.setDrawRange(0, trail.length);
        trailLine.geometry.attributes.position.needsUpdate = true;
    }

    return {
        update(state) {
            if (!initialized || !visible) return;
            // sim(x, y, z=up) → Three.js(x, z, y)  [Y is up in Three.js]
            rocketGroup.position.set(state.x, state.z, state.y);
            // roll→Z, pitch→X, yaw→Y  (intrinsic, adjust once physics is live)
            rocketGroup.rotation.set(state.pitch, state.yaw, state.roll);
            updateTrail(state.x, state.z, state.y);
        },
        reset() {
            trail.length = 0;
            if (trailLine) trailLine.geometry.setDrawRange(0, 0);
            if (rocketGroup) {
                rocketGroup.position.set(0, 0, 0);
                rocketGroup.rotation.set(0, 0, 0);
            }
        },
        show() {
            visible = true;
            if (!initialized) init();
            if (!animating) { animating = true; renderLoop(); }
        },
        hide() {
            visible   = false;
            animating = false;
        },
    };
}

// =============================================================================
// Canvas renderer
// =============================================================================
function makeUplotRenderer() {
    const MAX_PTS = 300;

    const bufs = { x: [], y: [], z: [], e: [] };

    // One chart per canvas id
    const charts = [
        { id: 'chartX',   key: 'x', color: '#f80', label: 'x (m)'    },
        { id: 'chartY',   key: 'y', color: '#0f8', label: 'y (m)'    },
        { id: 'chartZ',   key: 'z', color: '#0cf', label: 'z (m)'    },
        { id: 'chartErr', key: 'e', color: '#fa0', label: '|err| (m)' },
    ];

    function drawChart({ id, key, color }) {
        const canvas = $(id);
        if (!canvas) return;
        const data = bufs[key];

        const w = canvas.width  = canvas.offsetWidth;
        const h = canvas.height = canvas.offsetHeight;
        const ctx = canvas.getContext('2d');
        ctx.clearRect(0, 0, w, h);
        if (data.length < 2) return;

        const min   = Math.min(...data);
        const max   = Math.max(...data);
        const range = max - min || 1;

        // zero line
        if (min <= 0 && max >= 0) {
            const y0 = h - ((-min) / range) * (h - 4) - 2;
            ctx.strokeStyle = '#2a2a2a';
            ctx.lineWidth = 1;
            ctx.beginPath(); ctx.moveTo(0, y0); ctx.lineTo(w, y0); ctx.stroke();
        }

        ctx.strokeStyle = color;
        ctx.lineWidth   = 1.5;
        ctx.beginPath();
        data.forEach((v, i) => {
            const px = (i / (MAX_PTS - 1)) * w;
            const py = h - ((v - min) / range) * (h - 4) - 2;
            i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
        });
        ctx.stroke();
    }

    return {
        update(state, err) {
            const eMag = Math.sqrt(err.xErr**2 + err.yErr**2 + err.zErr**2);
            bufs.x.push(state.x);
            bufs.y.push(state.y);
            bufs.z.push(state.z);
            bufs.e.push(eMag);
            for (const k of ['x', 'y', 'z', 'e'])
                if (bufs[k].length > MAX_PTS) bufs[k].shift();

            charts.forEach(drawChart);
        },
        reset() {
            for (const k of ['x', 'y', 'z', 'e']) bufs[k] = [];
            charts.forEach(({ id }) => {
                const c = $(id);
                if (c) c.getContext('2d').clearRect(0, 0, c.width, c.height);
            });
        },
    };
}

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
    const eMag = Math.sqrt(err.xErr**2 + err.yErr**2 + err.zErr**2);
    ui.err_mag.textContent = fmt(eMag);

    ui.stepCount.textContent = fmtI(step);
    ui.dt.textContent        = `${(TIMESTEP_S * 1000).toFixed(1)} ms`;

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
// Simulation loop
// =============================================================================
function loop() {
    if (!running) return;

    const stepParams = {
        timeStep_s: TIMESTEP_S,
        userForce: { fX: userForce.fX, fY: userForce.fY, fZ: userForce.fZ },
    };

    const result = sim.ext_step(stepParams);

    if (result.isError) {
        setError('ext_step returned error — simulation stopped');
        stop();
        return;
    }

    simTime += TIMESTEP_S;
    stepCount++;

    updatePanels(result.state, result.err, simTime, stepCount);
    renderers.forEach(r => r.update(result.state, result.err, simTime, stepCount));

    frameId = requestAnimationFrame(loop);
}

// =============================================================================
// Controls
// =============================================================================
function start() {
    running = true;
    ui.btnStart.disabled = true;
    ui.btnStop.disabled  = false;
    ui.btnReset.disabled = true;
    setStatus('Running...');
    setError('');
    loop();
}

function stop() {
    running = false;
    if (frameId) cancelAnimationFrame(frameId);
    ui.btnStart.disabled = false;
    ui.btnStop.disabled  = true;
    ui.btnReset.disabled = false;
    setStatus('Stopped.');
}

function reset() {
    simTime   = 0;
    stepCount = 0;
    ui.simTime.innerHTML = `0.000 <span>s</span>`;
    renderers.forEach(r => r.reset());
    setStatus('Ready.');
    setError('');
    ui.btnReset.disabled = true;

    // Re-init core
    const err = sim.ext_init(INIT_PARAMS);
    if (err) setError('ext_init failed on reset');
}

ui.btnStart.addEventListener('click', start);
ui.btnStop.addEventListener('click',  stop);
ui.btnReset.addEventListener('click', reset);

// View toggle helpers
function showView(name) {
    ui.viewCharts.style.display = name === 'charts' ? 'grid'  : 'none';
    ui.view3d.style.display     = name === '3d'     ? 'block' : 'none';
    ui.viewParams.style.display = name === 'params' ? 'block' : 'none';
    ui.btnCharts.classList.toggle('active', name === 'charts');
    ui.btn3d.classList.toggle('active',     name === '3d');
    ui.btnParams.classList.toggle('active', name === 'params');
    if (name === '3d')     renderer3d.show();
    else                   renderer3d.hide();
}

ui.btnCharts.addEventListener('click', () => showView('charts'));
ui.btn3d.addEventListener('click',     () => showView('3d'));
ui.btnParams.addEventListener('click', () => showView('params'));

// Apply params
ui.btnApply.addEventListener('click', () => {
    const params = readParamsForm();
    stop();
    const err = sim.ext_init(params);
    if (err) { setError('ext_init failed'); return; }
    renderers.forEach(r => r.reset());
    simTime = 0; stepCount = 0;
    ui.simTime.innerHTML = `0.000 <span>s</span>`;
    setStatus('Ready — params applied.');
    setError('');
    ui.btnStart.disabled = false;
    ui.btnReset.disabled = true;
});

// =============================================================================
// Boot
// =============================================================================
(async () => {
    try {
        sim = await createSimulator();

        // Register renderers
        renderers.push(makeUplotRenderer());

        renderer3d = make3DRenderer();
        renderers.push(renderer3d);
        ui.btn3d.disabled = false;

        const err = sim.ext_init(INIT_PARAMS);
        if (err) {
            setError('ext_init failed');
            return;
        }

        fillParamsForm(INIT_PARAMS);
        setupForceButtons();
        setStatus('Ready.');
        ui.btnStart.disabled = false;
    } catch (e) {
        setError(`Failed to load WASM: ${e.message}`);
    }
})();
