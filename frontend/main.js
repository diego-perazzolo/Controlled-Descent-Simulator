import createSimulator from '../build/simulator.js';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// =============================================================================
// Config
// =============================================================================
const DEFAULT_TIMESTEP_S = 0.01;
const TIMESTEP_MIN_S     = 0.0001;
const TIMESTEP_MAX_S     = 0.5;

// Hardcoded fallback for the very first Poly4 when the trajectory list is
// empty and the user has no rocket state to seed from. Matches the reference
// descent in the design notebook (section 6).
const SEED_INITIAL_POS = { x: -50, y:  50, z: 150 };
const SEED_INITIAL_VEL = { x:   0, y:   5, z: -50 };

// Default trajectory loaded at boot so a user can press Start immediately
// without having to compose a sequence first. Loaded once, on the first boot;
// not re-injected on Reset / Apply, so a user who explicitly clears the
// sequence stays cleared.
const DEFAULT_TRAJECTORY = [
    {
        kind: 'poly4',
        params: {
            initialPos: { x: -50, y: 50, z:  80 },
            initialVel: { x:   0, y:  5, z: -50 },
            finalPos:   { x:   0, y:  0, z:   0 },
            finalVel:   { x:   0, y:  0, z:   0 },
            finalAcc:   { x:   0, y:  0, z:   0 },
            time_s: 20,
        },
    },
];

const INIT_PARAMS = {
    rocketPar: {
        mass_Kg:        10.0,
        inertiaX_Kgm2: 10.0 / 3,
        inertiaY_Kgm2: 10.0 / 3,
        inertiaZ_Kgm2: 1,
        c:              1,
        cz:             0.02,
    },
    actuatorLimits: {
        fZ_max: 500.0,
        fZ_min: 0.0,
        Tx_max: 10.0,
        Tx_min: -10.0,
        Ty_max: 10.0,
        Ty_min: -10.0,
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
let timestep_s = DEFAULT_TIMESTEP_S;

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
    p_fZmax: $('p_fZmax'), p_fZmin: $('p_fZmin'), p_tXmax: $('p_tXmax'), p_tXmin: $('p_tXmin'), p_tYmax: $('p_tYmax'), p_tYmin: $('p_tYmin'),
    p_dt:    $('p_dt'),
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
    ui.p_fZmax.value = p.actuatorLimits.fZ_max;
    ui.p_fZmin.value = p.actuatorLimits.fZ_min;
    ui.p_tXmax.value = p.actuatorLimits.Tx_max;
    ui.p_tXmin.value = p.actuatorLimits.Tx_min;
    ui.p_tYmax.value = p.actuatorLimits.Ty_max;
    ui.p_tYmin.value = p.actuatorLimits.Ty_min;
    ui.p_dt.value    = timestep_s;
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
        actuatorLimits: { 
            fZ_max: n('p_fZmax'), 
            fZ_min: n('p_fZmin'), 
            Tx_max: n('p_tXmax'), 
            Tx_min: n('p_tXmin'), 
            Ty_max: n('p_tYmax'), 
            Ty_min: n('p_tYmin'), 
        },
    };
}

function readTimestep() {
    let v = parseFloat(ui.p_dt.value);
    if (!isFinite(v) || v <= 0) v = DEFAULT_TIMESTEP_S;
    if (v < TIMESTEP_MIN_S) v = TIMESTEP_MIN_S;
    if (v > TIMESTEP_MAX_S) v = TIMESTEP_MAX_S;
    ui.p_dt.value = v;
    return v;
}

const fmt  = (v, d = 4) => v.toFixed(d);
const fmtI = v => Math.round(v).toString();

function setStatus(msg) { ui.status.textContent = msg; }
function setError(msg)  { ui.error.textContent  = msg; }

// =============================================================================
// 3D renderer
// =============================================================================
function make3DRenderer() {
    const TRAIL_MAX = 6000;
    const TRAJECTORY_MAX = 6000;
    const trail     = [];
    const trajectory = [];

    let scene, camera, renderer, controls;
    let rocketGroup, trailLine, trajectoryLine;
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
            leg.rotation.x = -Math.sin(angle) * 0.35;
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

        // Trajectory preview
        trajectoryLine = new THREE.Line(
            new THREE.BufferGeometry(),
            new THREE.LineBasicMaterial({ color: 0xff9900, transparent: true, opacity: 0.55 })
        );
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

    return {
        update(state) {
            if (!initialized || !visible) return;

            // Generate preview lazily on first display, or after invalidation.
            if (trajectory.length === 0) {
                generateTrajectoryPreview();
            }

            // sim(x, y, z=up) → Three.js(x, z, y)  [Y is up in Three.js]
            rocketGroup.position.set(state.x, state.z, state.y);
            // roll→Z, pitch→X, yaw→Y  (intrinsic, adjust once physics is live)
            rocketGroup.rotation.set(-state.pitch, -state.yaw, -state.roll);
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
        // Drop the cached trajectory line so the next frame regenerates it
        // from the current backend state. Called by the trajectory builder
        // whenever the sequence changes.
        invalidateTrajectory() {
            clearTrajectoryPreview();
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
// Canvas renderer
// =============================================================================
function makeUplotRenderer() {
    const MAX_PTS = 3000;

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
//     params: { initialPos:{x,y,z}, initialVel:{x,y,z},
//               finalPos:{x,y,z},   finalVel:{x,y,z}, finalAcc:{x,y,z},
//               time_s } }
//   { kind: 'point',
//     params: { finalPos:{x,y,z}, time_s } }
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
        // Point inputs
        pointBox: $('traj_point_fields'),
        pt: ['traj_pt_x', 'traj_pt_y', 'traj_pt_z'].map($),
    };

    // ---- Helpers ----
    function num(el) {
        const v = parseFloat(el.value);
        return isFinite(v) ? v : 0;
    }
    function readVec3(els)    { return { x: num(els[0]), y: num(els[1]), z: num(els[2]) }; }
    function setVec3(els, v)  { els[0].value = v.x; els[1].value = v.y; els[2].value = v.z; }
    function emitChange()     { onChangeListeners.forEach(fn => fn()); }
    function getTotalDuration() {
        return items.reduce((acc, it) => acc + it.params.time_s, 0);
    }

    function endStateOfLastItem() {
        // What the rocket position/velocity will be at the end of the last
        // item in the sequence. Used to seed initialPos/initialVel of the
        // next Poly4 so successive segments line up by default.
        if (items.length === 0) {
            return { pos: SEED_INITIAL_POS, vel: SEED_INITIAL_VEL };
        }
        const last = items[items.length - 1];
        if (last.kind === 'poly4') {
            return { pos: last.params.finalPos, vel: last.params.finalVel };
        }
        // 'point' has no finalVel; assume zero velocity at the waypoint.
        return { pos: last.params.finalPos, vel: { x: 0, y: 0, z: 0 } };
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
                    initialPos: readVec3(dom.p0),
                    initialVel: readVec3(dom.v0),
                    finalPos:   readVec3(dom.pF),
                    finalVel:   readVec3(dom.vF),
                    finalAcc:   readVec3(dom.aF),
                    time_s,
                },
            };
            ok = !sim.ext_trajectory_append_poly4(item.params);
        } else {
            item = {
                kind: 'point',
                params: {
                    finalPos: readVec3(dom.pt),
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

    // ---- Replay (after ext_init wipes backend state) ----
    function replayToBackend() {
        // After ext_init the backend trajectory is assumed to be empty. Push
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
// Simulation loop
// =============================================================================
function loop() {
    if (!running) return;

    const stepParams = {
        timeStep_s: timestep_s,
        userForce: { fX: userForce.fX, fY: userForce.fY, fZ: userForce.fZ },
    };

    const result = sim.ext_step(stepParams);

    if (result.isError) {
        setError('ext_step returned error — simulation stopped');
        stop();
        return;
    }

    simTime += timestep_s;
    stepCount++;

    updatePanels(result.state, result.err, simTime, stepCount);
    renderers.forEach(r => r.update(result.state, result.err, simTime, stepCount));

    frameId = requestAnimationFrame(loop);
}

// =============================================================================
// Controls
// =============================================================================
function refreshStartEnabled() {
    // Start is allowed only when:
    //   - sim is not currently running, AND
    //   - the trajectory has at least one item (otherwise the backend has
    //     nothing to track and the run is meaningless).
    if (running) {
        ui.btnStart.disabled = true;
        return;
    }
    ui.btnStart.disabled = trajectoryBuilder.isEmpty();
}

function start() {
    if (trajectoryBuilder.isEmpty()) {
        setError('Cannot start: trajectory sequence is empty.');
        return;
    }
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
    ui.btnStop.disabled  = true;
    ui.btnReset.disabled = false;
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

    // Re-init core. ext_init wipes the backend trajectory, so the JS-side
    // sequence has to be replayed for the next run to be valid.
    const err = sim.ext_init(readParamsForm());
    if (err) { setError('ext_init failed on reset'); return; }
    trajectoryBuilder.replayToBackend();

    // 3D preview must be recomputed from the freshly-replayed backend state.
    renderer3d?.invalidateTrajectory?.();

    refreshStartEnabled();
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

// Apply rocket / actuator params + timestep (also re-inits the core).
ui.btnApply.addEventListener('click', () => {
    const params = readParamsForm();
    timestep_s = readTimestep();
    stop();
    const err = sim.ext_init(params);
    if (err) { setError('ext_init failed'); return; }

    // ext_init wipes the backend trajectory; replay the JS-side sequence.
    trajectoryBuilder.replayToBackend();
    renderer3d?.invalidateTrajectory?.();

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
        // Inject a default sequence so a fresh user can press Start straight
        // away. Done after ext_init so the backend trajectory is empty.
        // loadPreset triggers refreshAutofill internally, so initialPos /
        // initialVel of the form will reflect the end of the default item.
        trajectoryBuilder.loadPreset(DEFAULT_TRAJECTORY);
        setupForceButtons();
        setStatus('Ready.');
        refreshStartEnabled();
    } catch (e) {
        setError(`Failed to load WASM: ${e.message}`);
    }
})();