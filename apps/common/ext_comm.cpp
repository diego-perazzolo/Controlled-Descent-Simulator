// =============================================================================
// Controlled Descent Simulator
// =============================================================================
//
// Copyright (c) 2026 Diego Perazzolo
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// =============================================================================
// File        : ext_comm.cpp
// Description : Direct implementation of the external communication API:
//               converts ext structs and calls straight into the core.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "ext_comm.hpp"
#include "core.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "log.hpp"
#include "LogUiSink.hpp"
#include "LogSinks.hpp"
#include "profile.hpp"
#include "Recorder.hpp"

/* Immediately return if ret == true */
#define RETURN_IF_TRUE(ret) if(ret) return ret

/* Static functions */
static core_rocketParams_t _convertExtToCore_rocketParams(ext_rocketParams rPar, ext_rocketActuatorLimits aPar)
{
    core_rocketParams_t coreParam  = {};
    coreParam.m  = rPar.mass_Kg;
    coreParam.Ix = rPar.inertiaX_Kgm2;
    coreParam.Iy = rPar.inertiaY_Kgm2;
    coreParam.Iz = rPar.inertiaZ_Kgm2;
    coreParam.g = 9.81;
    coreParam.c = rPar.c;
    coreParam.cz = rPar.cz;
    coreParam.F1_max = aPar.fZ_max;
    coreParam.F1_min = aPar.fZ_min;
    coreParam.T1_max = aPar.Tx_max;
    coreParam.T1_min = aPar.Tx_min;
    coreParam.T2_max = aPar.Ty_max;
    coreParam.T2_min = aPar.Ty_min;
    coreParam.T3_max = aPar.Tz_max;
    coreParam.T3_min = aPar.Tz_min;

    return coreParam;
}

static core_quadRotorParams_t _convertExtToCore_quadRotorParams(ext_quadRotorParams rPar, ext_quadRotorActuatorLimits aPar)
{
    core_quadRotorParams_t coreParam  = {};
    coreParam.m  = rPar.mass_Kg;
    coreParam.Ix = rPar.inertiaX_Kgm2;
    coreParam.Iy = rPar.inertiaY_Kgm2;
    coreParam.Iz = rPar.inertiaZ_Kgm2;
    coreParam.g = 9.81;
    coreParam.c = rPar.c;
    coreParam.cz = rPar.cz;
    coreParam.kT = rPar.motorThrustCoefficient;
    coreParam.kQ = rPar.motorTorqueCoefficient;
    coreParam.L = rPar.distanceBtwMotorAndCoM;
    coreParam.Irot = rPar.motorMomentOfInertia;
    coreParam.Fm_max = aPar.motor_max_thrust;
    coreParam.Fm_min = aPar.motor_min_thrust;

    return coreParam;
}

static core_trajectoryPoly4Params_t _convertExtToCore_trajectoryPoly4Params(ext_trajectoryPoly4Params_t ext)
{
    core_trajectoryPoly4Params_t coreParam = {
        .initialPos = {ext.initialPos.x, ext.initialPos.y, ext.initialPos.z},
        .initialYaw = ext.initialYaw,
        .initialVel = {ext.initialVel.x, ext.initialVel.y, ext.initialVel.z},
        .initialYawRate = ext.initialYawRate,
        .finalPos = {ext.finalPos.x, ext.finalPos.y, ext.finalPos.z},
        .finalYaw = ext.finalYaw,
        .finalVel = {ext.finalVel.x, ext.finalVel.y, ext.finalVel.z},
        .finalYawRate = ext.finalYawRate,
        .finalAcc = {ext.finalAcc.x, ext.finalAcc.y, ext.finalAcc.z},
        .finalYawAcc = ext.finalYawAcc,
        .time_s = ext.time_s
    };

    return coreParam;
}

static core_trajectoryPointParams_t _convertExtToCore_trajectoryPointParams(ext_trajectoryPointParams_t ext)
{
    core_trajectoryPointParams_t coreParam = {
        .finalPos = {ext.finalPos.x, ext.finalPos.y, ext.finalPos.z},
        .finalYaw = ext.finalYaw,
        .time_s = ext.time_s
    };

    return coreParam;
}

static ext_trajectoryPoint _convertExtToCore_trajectoryPoint(Vec3& point)
{
    ext_trajectoryPoint extParam = {.x = (ext_coord_t) point[0], .y = (ext_coord_t) point[1], .z = (ext_coord_t) point[2]};

    return extParam;
}

static core_systemParams_t _convertExtToCore_systemParams(ext_systemParams& ext)
{
    core_systemParams_t coreParam = {
        .timestep_seconds = ext.timestep_seconds,
        .user_fX = ext.user_forces.fX,
        .user_fY = ext.user_forces.fY,
        .user_fZ = ext.user_forces.fZ,
    };

    return coreParam; 
}

static ext_snapshotData _convertCoreToExt_snapshotParams(core_snapshotData_t& core)
{
    ext_snapshotData ext = {};

    ext.time_seconds = core.time_seconds;
    ext.state.x_dot = core.state.x_dot; 
    ext.state.y_dot = core.state.y_dot; 
    ext.state.z_dot = core.state.z_dot; 
    ext.state.x = core.state.x; 
    ext.state.y = core.state.y; 
    ext.state.z = core.state.z; 
    ext.state.roll_dot = core.state.roll_dot; 
    ext.state.pitch_dot = core.state.pitch_dot; 
    ext.state.yaw_dot = core.state.yaw_dot; 
    ext.state.roll = core.state.roll; 
    ext.state.pitch = core.state.pitch; 
    ext.state.yaw = core.state.yaw; 

    ext.err.xErr = core.errors.x;
    ext.err.yErr = core.errors.y;
    ext.err.zErr = core.errors.z;
    ext.err.yawErr = core.errors.yaw;

    return ext;
}

/* ext functions */

bool ext_initRocket_FFLQR01(ext_initRocketParams params)
{

    /* Initialize core, return true if error */

    // Struct conversion
    core_rocketParams_t rPar = _convertExtToCore_rocketParams(params.params, params.actuatorLimits);

    // Core initialization
    bool ret = core_init();
    RETURN_IF_TRUE(ret);

    // Rocket initialization
    ret = core_rocketFfLqr01_init(rPar);
    RETURN_IF_TRUE(ret);

    // Trajectory initialization
    ret = core_trajectoryInit();
    RETURN_IF_TRUE(ret);

    return ret;
}

bool ext_initQuadRotor_FFLQR01(ext_initQuadRotorParams params)
{

    /* Initialize core, return true if error */

    // Struct conversion
    core_quadRotorParams_t rPar = _convertExtToCore_quadRotorParams(params.params, params.actuatorLimits);

    // Core initialization
    bool ret = core_init();
    RETURN_IF_TRUE(ret);

    // QuadRotor initialization
    ret = core_quadRotorFfLqr01_init(rPar);
    RETURN_IF_TRUE(ret);

    // Trajectory initialization
    ret = core_trajectoryInit();
    RETURN_IF_TRUE(ret);

    return ret;
}

bool ext_initQuadRotor_MPC01(ext_initQuadRotorParams params)
{

    /* Initialize core, return true if error */

    // Struct conversion (same physical params as the FF+LQR quadrotor)
    core_quadRotorParams_t rPar = _convertExtToCore_quadRotorParams(params.params, params.actuatorLimits);

    // Core initialization
    bool ret = core_init();
    RETURN_IF_TRUE(ret);

    // QuadRotor MPC initialization
    ret = core_quadRotorMPC01_init(rPar);
    RETURN_IF_TRUE(ret);

    // Trajectory initialization
    ret = core_trajectoryInit();
    RETURN_IF_TRUE(ret);

    return ret;
}

bool ext_trajectory_append_poly4(ext_trajectoryPoly4Params_t params)
{
    bool ret = false;

    core_trajectoryPoly4Params_t core_params = _convertExtToCore_trajectoryPoly4Params(params);

    ret = core_trajectoryAppendPoly4(core_params);
    
    return ret;
}

bool ext_trajectory_append_point(ext_trajectoryPointParams_t params)
{
     bool ret = false;

    core_trajectoryPointParams_t core_params = _convertExtToCore_trajectoryPointParams(params);

    ret = core_trajectoryAppendPoint(core_params);
    
    return ret;
}

bool ext_trajectory_remove_last_item(void)
{
    bool ret = false;

    ret = core_trajectoryRemoveLastItem();

    return ret;
}

ext_trajectoryPoint ext_trajectory_get_point(ext_coord_t t)
{
    Vec3 p = {};
    if(core_getTrajectoryPoint(t, p))
    {
        // Err
        return {};
    }

    return _convertExtToCore_trajectoryPoint(p);
}

bool ext_setSystemParams(ext_systemParams params)
{
    // Struct conversion
    core_systemParams_t corePar = _convertExtToCore_systemParams(params);

    return core_setSystemParams(corePar);
}

ext_snapshotData ext_getSnapshot(void)
{
    core_snapshotData_t corePar = {};
    bool ret = core_getSnapshot(corePar);

    ext_snapshotData extPar = _convertCoreToExt_snapshotParams(corePar);
    extPar.isError = ret;

    return extPar;
}


ext_plantSnapshotData ext_getPlantSnapshot(void)
{
    core_plantSnapshotData_t corePar = {};
    bool ret = core_getPlantSnapshot(corePar);

    ext_plantSnapshotData extPar = {};
    extPar.time_seconds = corePar.time_seconds;
    /* float carries the sequence exactly up to 2^24 samples */
    extPar.sequence = (ext_coord_t)corePar.sequence;

    extPar.state.x_dot = corePar.state.x_dot;
    extPar.state.y_dot = corePar.state.y_dot;
    extPar.state.z_dot = corePar.state.z_dot;
    extPar.state.x = corePar.state.x;
    extPar.state.y = corePar.state.y;
    extPar.state.z = corePar.state.z;
    extPar.state.roll_dot = corePar.state.roll_dot;
    extPar.state.pitch_dot = corePar.state.pitch_dot;
    extPar.state.yaw_dot = corePar.state.yaw_dot;
    extPar.state.roll = corePar.state.roll;
    extPar.state.pitch = corePar.state.pitch;
    extPar.state.yaw = corePar.state.yaw;

    extPar.isAttached = corePar.isAttached;
    extPar.isReadyToStart = corePar.isReadyToStart;
    extPar.isError = ret;

    return extPar;
}

bool ext_run(void)
{
    return core_run();
}

bool ext_stop(void)
{
    return core_stop();
}

bool ext_beginStaging(ext_coord_t safetyAltitude)
{
    return core_beginStaging(safetyAltitude);
}

bool ext_stopStaging(void)
{
    return core_stopStaging();
}

// --------------------------------------------------------------------------- //
// Logger / profiler inspection. Each "get" packs its payload as newline-        //
// separated, tab-delimited text into a fixed char buffer that the JS side       //
// parses. The producers (logging, profiling) never contain '\t'/'\n' in module  //
// or scope names; log messages are developer-controlled and assumed likewise.   //
// --------------------------------------------------------------------------- //

ext_logBatch ext_getLogBatch(void)
{
    ext_logBatch out = {};
    std::size_t off = 0;
    std::size_t count = 0;

    /* stop pulling while a full max-size line may not fit, so snprintf never
       truncates a record mid-way (+32 leaves room for the timestamp field) */
    const std::size_t maxLine = CDS_LOG_NAME_MAX + CDS_LOG_LINE_MAX + 40;
    while (sizeof(out.lines) - off > maxLine)
    {
        cds_log::RecentLinesSink::Line line;
        if (!cds_log::uiSink().Pop(line)) break;
        char ts[28];
        cds_log::formatTimestamp(ts, sizeof(ts), line.timestampNs);
        // timestamp \t LEVEL \t module \t text
        const int w = std::snprintf(out.lines + off, sizeof(out.lines) - off,
                                    "%s\t%s\t%s\t%s\n",
                                    ts, cds_log::levelName(line.level), line.module, line.text);
        if (w > 0) off += static_cast<std::size_t>(w);
        ++count;
    }
    out.lines[off] = '\0';
    out.count = static_cast<ext_coord_t>(count);
    out.dropped = static_cast<ext_coord_t>(cds_log::uiSink().TakeDropped());
    return out;
}

ext_moduleList ext_getLogModules(void)
{
    ext_moduleList out = {};
    std::size_t off = 0;
    std::size_t count = 0;
    const std::size_t n = cds_log::registry().count();
    for (std::size_t i = 0; i < n; ++i)
    {
        const cds_log::moduleId_t id = static_cast<cds_log::moduleId_t>(i);
        const int w = std::snprintf(out.list + off, sizeof(out.list) - off, "%zu\t%s\t%d\t%u\n",
                                    i, cds_log::registry().name(id),
                                    static_cast<int>(cds_log::registry().level(id)),
                                    cds_log::registry().sampleN(id));
        if (w < 0 || static_cast<std::size_t>(w) >= sizeof(out.list) - off) break;
        off += static_cast<std::size_t>(w);
        ++count;
    }
    out.list[off] = '\0';
    out.count = static_cast<ext_coord_t>(count);
    return out;
}

bool ext_setLogLevel(ext_logLevelParams params)
{
    const std::size_t m = static_cast<std::size_t>(params.module);
    const int lvl = static_cast<int>(params.level);
    if (m >= cds_log::registry().count()) return true;
    if (lvl < 0 || lvl > static_cast<int>(cds_log::Level::Off)) return true;
    cds_log::registry().setLevel(static_cast<cds_log::moduleId_t>(m),
                                 static_cast<cds_log::Level>(lvl));
    cds_log::registry().setSample(static_cast<cds_log::moduleId_t>(m),
                                  static_cast<std::uint32_t>(params.sampleN));
    return false;
}

ext_moduleList ext_getProfileModules(void)
{
    ext_moduleList out = {};
    std::size_t off = 0;
    std::size_t count = 0;
    const std::size_t n = cds_profile::registry().moduleCount();
    for (std::size_t i = 0; i < n; ++i)
    {
        const cds_profile::moduleId_t id = static_cast<cds_profile::moduleId_t>(i);
        const int w = std::snprintf(out.list + off, sizeof(out.list) - off, "%zu\t%s\t%d\n",
                                    i, cds_profile::registry().moduleName(id),
                                    cds_profile::registry().moduleEnabled(id) ? 1 : 0);
        if (w < 0 || static_cast<std::size_t>(w) >= sizeof(out.list) - off) break;
        off += static_cast<std::size_t>(w);
        ++count;
    }
    out.list[off] = '\0';
    out.count = static_cast<ext_coord_t>(count);
    return out;
}

bool ext_setProfileEnabled(ext_profileEnableParams params)
{
    const std::size_t m = static_cast<std::size_t>(params.module);
    if (m >= cds_profile::registry().moduleCount()) return true;
    cds_profile::registry().setEnabled(static_cast<cds_profile::moduleId_t>(m), params.enabled);
    return false;
}

ext_profileTable ext_getProfileTable(void)
{
    ext_profileTable out = {};
    std::size_t off = 0;
    std::size_t count = 0;

    cds_profile::Snapshot snap;
    if (cds_profile::registry().snapshot(snap))
    {
        out.table[0] = '\0'; // nothing published yet
        out.count = 0;
        return out;
    }

    cds_profile::Registry& reg = cds_profile::registry();
    for (std::size_t i = 0; i < snap.count; ++i)
    {
        const cds_profile::scopeId_t s = static_cast<cds_profile::scopeId_t>(i);
        const cds_profile::moduleId_t mod = reg.scopeModule(s);
        if (!reg.moduleEnabled(mod)) continue; // only scopes of enabled modules

        const cds_profile::ScopeStats& st = snap.stats[i];
        const bool isVal = reg.scopeIsValue(s);
        const double k = isVal ? 1.0 : 1.0 / 1000.0; // ns -> us for timed scopes
        // module\tscope\tkind\tcount\tmean\tstd\tmin\tmax\tp50\tp95\tp99
        const int w = std::snprintf(out.table + off, sizeof(out.table) - off,
            "%s\t%s\t%s\t%llu\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
            reg.moduleName(mod), reg.scopeName(s), isVal ? "val" : "us",
            static_cast<unsigned long long>(st.count),
            st.mean() * k, st.stddev() * k, st.min * k, st.max * k,
            st.p50 * k, st.p95 * k, st.p99 * k);
        if (w < 0 || static_cast<std::size_t>(w) >= sizeof(out.table) - off) break;
        off += static_cast<std::size_t>(w);
        ++count;
    }
    out.table[off] = '\0';
    out.count = static_cast<ext_coord_t>(count);
    return out;
}

bool ext_resetProfile(void)
{
    cds_profile::registry().resetAll();
    return false;
}

bool ext_setDiagFiles(ext_diagFiles params)
{
    // server-side only: on wasm there is no real filesystem, so these are no-ops
    cds_log::fileSink().setEnabled(params.logFile);
    cds_profile::registry().setRawLogging(params.profileRaw);
    return false;
}

// Snapshot the recorder state (model + plant) into the wire struct. Flags cross
// as ext_coord_t (0.0/1.0) so the char[] modelName can share the struct.
// modelName is a "model + plant" summary of whoever is active.
static ext_recordStatus _recordStatus(void)
{
    ext_recordStatus out = {};
    cds_record::IRecorder* m = cds_record::activeModelRecorder();
    cds_record::IRecorder* p = cds_record::activePlantRecorder();

    out.active      = (m || p) ? 1 : 0;
    out.enabled     = ((m && m->enabled()) || (p && p->enabled())) ? 1 : 0;
    out.droppedRows = static_cast<ext_coord_t>((m ? m->dropped() : 0) +
                                               (p ? p->dropped() : 0));

    // "Model + Plant" summary (either side may be absent), truncated to fit.
    out.modelName[0] = '\0';
    std::snprintf(out.modelName, sizeof(out.modelName), "%s%s%s",
                  m ? m->name() : "",
                  (m && p) ? " + " : "",
                  p ? p->name() : "");
    return out;
}

ext_recordStatus ext_setRecording(ext_recordParams params)
{
    // server-side only (wasm has no filesystem). Toggles both the active model
    // and the active plant recorder; the drain thread rotates each CSV on the
    // transition.
    if (cds_record::IRecorder* m = cds_record::activeModelRecorder()) m->setEnabled(params.enabled);
    if (cds_record::IRecorder* p = cds_record::activePlantRecorder()) p->setEnabled(params.enabled);
    return _recordStatus();
}

ext_recordStatus ext_getRecordStatus(void)
{
    return _recordStatus();
}

// ---- controller parameters (interactive tuning of the active controller) ----

ext_controllerManifest ext_getControllerManifest(void)
{
    ext_controllerManifest out = {};   // empty text if no model / no exposed params
    core_getControllerManifest(out.text, sizeof(out.text));
    return out;
}

bool ext_setControllerParam(ext_controllerParamSet params)
{
    // id crosses the wire as ext_coord_t (no int on the wire); round to the
    // nearest integer manifest id.
    const int id = static_cast<int>(params.id + (params.id >= 0 ? 0.5f : -0.5f));
    return core_setControllerParam(id, params.value);
}
